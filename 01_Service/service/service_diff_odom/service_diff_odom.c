/**
 * @file    service_diff_odom.c
 * @brief   差速底盘里程计服务实现
 * @note    - 位移由左右轮累计tick解算，航向由IMU端口提供
 *          - IMU绝对航向经set_pose偏置对齐到世界坐标
 *          - OSAL状态码在本层统一映射为platform_err_t
 */

#include <math.h>

#include "service_diff_odom.h"

#include "imu_port.h"
#include "motor_port.h"
#include "osal_mutex.h"
#include "osal_task.h"

#define ODOM_YAW_PERIOD_DEG  360.0F  /* 航向角周期，deg */
#define ODOM_YAW_HALF_DEG    180.0F  /* 航向角半周期，deg */
#define ODOM_PI_RAD           3.14159265358979323846F /* 圆周率 */
#define ODOM_RAD_PER_DEG      (ODOM_PI_RAD / 180.0F)  /* deg转rad */
#define ODOM_MS_PER_S         1000.0F                  /* 每秒毫秒数 */
#define ODOM_I32_MAX          2147483647               /* int32上界 */
#define ODOM_I32_MIN          (-ODOM_I32_MAX - 1)      /* int32下界 */
#define ODOM_I32_MAX_F        2147483520.0F /* float安全转换上界 */
#define ODOM_I32_MIN_F        (-2147483648.0F) /* float安全转换下界 */
#define ODOM_U32_MAX          0xFFFFFFFFU /* uint32上界 */

/** 单周期解算结果，由任务传入commit。 */
typedef struct {
    float    move_mm;     /* 底盘中心位移，mm */
    float    dyaw_deg;    /* 顺时针航向增量，deg */
    float    yaw_new_deg; /* 新的归一化航向，deg */
    float    w_deg_s;     /* 顺时针角速度（陀螺仪），deg/s */
    uint32_t elapsed_ms;  /* 距上次有效样本的周期时长，ms */
    uint32_t pose_epoch;  /* 采样对应的位姿校准代次 */
} odom_step_t;

static server_odom_cfg_t       g_odom_cfg;    /* 里程计配置 */
static osal_task_handle_t      g_odom_task;   /* 里程计任务句柄 */
static osal_mutex_handle_t     g_odom_mutex;  /* 里程计状态互斥锁 */
static server_odom_state_t     g_odom_state;  /* 里程计状态快照 */
static float                   g_odom_x_mm;   /* X方向浮点累计位置 */
static float                   g_odom_y_mm;   /* Y方向浮点累计位置 */
static uint32_t                g_pose_epoch;  /* 位姿校准代次 */

/**
 * @brief  将OSAL状态码映射为平台统一状态码
 * @param  st OSAL返回状态
 * @retval 对应的platform_err_t
 */
static platform_err_t odom_map_osal(osal_status_t st)
{
    switch (st) {
        case OSAL_OK:
            return PLATFORM_ERR_OK;
        case OSAL_ERR_PARAM:
            return PLATFORM_ERR_PARAM;
        case OSAL_ERR_TIMEOUT:
            return PLATFORM_ERR_TIMEOUT;
        case OSAL_ERR_NO_MEMORY:
            return PLATFORM_ERR_NO_MEMORY;
        case OSAL_ERR_BUSY:
            return PLATFORM_ERR_BUSY;
        case OSAL_ERR_NOT_SUPPORTED:
            return PLATFORM_ERR_NOT_SUPPORTED;
        default:
            return PLATFORM_ERR_FAIL;
    }
}

static float odom_norm_yaw(float yaw_deg)
{
    float norm_deg; /* 归一化后的航向角 */

    norm_deg = fmodf(yaw_deg + ODOM_YAW_HALF_DEG,
                     ODOM_YAW_PERIOD_DEG);
    if (norm_deg < 0.0F) {
        norm_deg += ODOM_YAW_PERIOD_DEG;
    }

    return norm_deg - ODOM_YAW_HALF_DEG;
}

/**
 * @brief  将有限浮点数四舍五入并饱和为int32
 * @param  value 待转换数值
 * @retval 转换后的int32数值
 */
static int32_t odom_round_i32(float value)
{
    if (!isfinite(value)) {
        return 0;
    }
    if (value > ODOM_I32_MAX_F) {
        return ODOM_I32_MAX;
    }
    if (value <= ODOM_I32_MIN_F) {
        return ODOM_I32_MIN;
    }

    if (value >= 0.0F) {
        return (int32_t)(value + 0.5F);
    }
    return (int32_t)(value - 0.5F);
}

/**
 * @brief  按二进制补码模运算计算累计tick差值
 * @param  now  当前累计tick
 * @param  last 上次累计tick
 * @retval 当前值相对上次值的有符号增量
 */
static int64_t odom_tick_delta(int64_t now, int64_t last)
{
    uint64_t diff;    /* 无符号模减结果 */
    uint64_t u64_max; /* uint64最大值 */
    uint64_t i64_max; /* int64正数最大值 */

    u64_max = ~(uint64_t)0;
    i64_max = u64_max >> 1;
    diff = (uint64_t)now - (uint64_t)last;

    if (diff <= i64_max) {
        return (int64_t)diff;
    }
    return -1 - (int64_t)(u64_max - diff);
}

static void odom_wait_period(osal_tick_type_t *last_wake)
{
    osal_status_t st; /* OSAL定周期延时状态 */

    st = osal_task_wait_until(last_wake, SERVER_ODOM_PERIOD_MS);
    if (st != OSAL_OK) {
        /* 周期机制异常时挂起自身，避免空转抢占 */
        (void)osal_task_suspend(g_odom_task);
        *last_wake = osal_task_get_tick_count();
    }
}

/**
 * @brief  原子发布运行模式并清零瞬时速度
 * @param  mode 待发布的运行模式
 * @retval PLATFORM_ERR_OK或OSAL Mutex错误码
 */
static platform_err_t odom_set_mode(server_odom_mode_t mode)
{
    platform_err_t err; /* OSAL映射后的返回状态 */

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_odom_state.vx_mm_s = 0;
    g_odom_state.vy_mm_s = 0;
    g_odom_state.w_deg_s = 0.0F;
    g_odom_state.mode = mode;

    return odom_map_osal(osal_mutex_give(g_odom_mutex));
}

/**
 * @brief  读取当前位姿校准代次
 * @param  epoch 输出校准代次，不可为NULL
 * @retval PLATFORM_ERR_OK或OSAL Mutex错误码
 */
static platform_err_t odom_get_epoch(uint32_t *epoch)
{
    platform_err_t err; /* OSAL映射后的返回状态 */

    if (epoch == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    *epoch = g_pose_epoch;
    return odom_map_osal(osal_mutex_give(g_odom_mutex));
}

/**
 * @brief  基线重建：锁内校验代次并重算IMU航向偏置
 * @param  att_yaw_deg IMU当前航向，顺时针，deg
 * @param  pose_epoch  采样对应的位姿校准代次
 * @param  yaw_off_deg 输出航向偏置，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_BUSY(代次换代) / OSAL Mutex错误码
 */
static platform_err_t odom_rebase(float att_yaw_deg,
                                  uint32_t pose_epoch,
                                  float *yaw_off_deg)
{
    platform_err_t err; /* OSAL映射后的返回状态 */

    if (yaw_off_deg == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (g_pose_epoch != pose_epoch) {
        err = odom_map_osal(osal_mutex_give(g_odom_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_BUSY;
    }

    /* 偏置使 IMU航向+偏置 == 当前位姿航向 */
    *yaw_off_deg = odom_norm_yaw(g_odom_state.yaw_deg - att_yaw_deg);
    g_odom_state.vx_mm_s = 0;
    g_odom_state.vy_mm_s = 0;
    g_odom_state.w_deg_s = 0.0F;
    g_odom_state.mode = PLATFORM_ODOM_ACTIVE;

    return odom_map_osal(osal_mutex_give(g_odom_mutex));
}

/**
 * @brief  基于锁内最新姿态提交一个里程增量
 * @param  step 单周期解算结果，不可为NULL且时长非0
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_BUSY(代次换代) / PLATFORM_ERR_FAIL /
 *         OSAL Mutex错误码
 */
static platform_err_t odom_commit_step(const odom_step_t *step)
{
    float yaw_mid; /* 本周期中点航向，rad */
    float next_x;  /* 本周期后的X浮点位置 */
    float next_y;  /* 本周期后的Y浮点位置 */
    float scale;   /* 位移到每秒速度的换算系数 */
    float vx_mm_s; /* 本周期底盘前向速度 */
    platform_err_t err; /* OSAL映射后的返回状态 */

    if (step == NULL || step->elapsed_ms == 0U) {
        return PLATFORM_ERR_PARAM;
    }

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (g_pose_epoch != step->pose_epoch) {
        err = odom_map_osal(osal_mutex_give(g_odom_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_BUSY;
    }

    /* 中点航向积分：沿上次航向加半个IMU角增量的方向平移 */
    yaw_mid = (g_odom_state.yaw_deg + step->dyaw_deg * 0.5F) *
              ODOM_RAD_PER_DEG;
    next_x = g_odom_x_mm + step->move_mm * cosf(yaw_mid);
    next_y = g_odom_y_mm + step->move_mm * sinf(yaw_mid);
    scale = ODOM_MS_PER_S / (float)step->elapsed_ms;
    vx_mm_s = step->move_mm * scale;

    if (!isfinite(next_x) || !isfinite(next_y) ||
        !isfinite(step->yaw_new_deg) || !isfinite(vx_mm_s) ||
        !isfinite(step->w_deg_s)) {
        g_odom_state.vx_mm_s = 0;
        g_odom_state.vy_mm_s = 0;
        g_odom_state.w_deg_s = 0.0F;
        g_odom_state.mode = PLATFORM_ODOM_ERROR;
        err = odom_map_osal(osal_mutex_give(g_odom_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_FAIL;
    }

    g_odom_x_mm = next_x;
    g_odom_y_mm = next_y;
    g_odom_state.x_mm = odom_round_i32(next_x);
    g_odom_state.y_mm = odom_round_i32(next_y);
    g_odom_state.vx_mm_s = odom_round_i32(vx_mm_s);
    g_odom_state.vy_mm_s = 0;
    g_odom_state.yaw_deg = step->yaw_new_deg; /* IMU绝对航向 */
    g_odom_state.w_deg_s = step->w_deg_s;     /* 陀螺仪角速度 */
    g_odom_state.mode = PLATFORM_ODOM_ACTIVE;

    return odom_map_osal(osal_mutex_give(g_odom_mutex));
}

/** 将里程计标记为错误；Mutex异常时由任务下一周期重试。 */
static void odom_mark_error(void)
{
    platform_err_t err; /* 状态发布结果 */

    err = odom_set_mode(PLATFORM_ODOM_ERROR);
    if (PLATFORM_IS_ERR(err)) {
        return;
    }
}

/**
 * @brief  周期融合双轮累计tick与IMU航向并更新里程计状态。
 *
 * 每个周期分5个阶段推进，任一阶段失败即等待下一周期重来：
 *   1 计时累加      —— dt累积，跳周期不丢时间
 *   2 采样          —— 记录epoch后读双轮tick与IMU航向
 *   3 基线建立/重建 —— 首次或epoch换代，存差分起点并求航向偏置
 *   4 运动学解算    —— tick增量→中心位移；IMU角差→航向增量
 *   5 提交与推进    —— commit锁内校验epoch，成功才滚动基线
 */
static void server_odom_task(void *argument)
{
    imu_port_att_t att = {0};   /* 当前IMU航向姿态 */
    odom_step_t step = {0};     /* 本周期解算结果 */
    int64_t left_now = 0;       /* 当前左轮累计tick */
    int64_t right_now = 0;      /* 当前右轮累计tick */
    int64_t last_left = 0;      /* 上次有效左轮累计tick */
    int64_t last_right = 0;     /* 上次有效右轮累计tick */
    int64_t delta_left;         /* 本周期左轮tick增量 */
    int64_t delta_right;        /* 本周期右轮tick增量 */
    float left_mm;              /* 本周期左轮位移，mm */
    float right_mm;             /* 本周期右轮位移，mm */
    float yaw_off = 0.0F;       /* IMU航向到位姿航向的偏置 */
    float yaw_new;              /* 本周期偏置后的航向，deg */
    float last_yaw = 0.0F;      /* 上次提交的航向，deg */
    uint32_t elapsed_ms = 0U;   /* 距上次有效样本的周期时长 */
    uint32_t base_epoch = 0U;   /* 当前差分基线的位姿代次 */
    uint32_t start_epoch;       /* 采样开始前的位姿代次 */
    uint8_t have_base = 0U;     /* 差分基线有效标志 */
    osal_tick_type_t last_wake; /* 上次周期唤醒tick */
    platform_err_t err;         /* 平台接口返回状态 */

    (void)argument;
    last_wake = osal_task_get_tick_count();

    while (1) {
        /* ---- 阶段1：计时累加 ----------------------------------------
         * 上次基线有效时，把本周期时长累加进dt；上一周期若因异常/重置跳过
         * 提交，dt会继续累积，保证速度估计的分母是真实间隔而非固定周期。
         * 无基线时dt保持0，由阶段3建立基线时统一清零。
         */
        if (have_base != 0U) {
            if (elapsed_ms <= ODOM_U32_MAX - SERVER_ODOM_PERIOD_MS) {
                elapsed_ms += SERVER_ODOM_PERIOD_MS;
            } else {
                elapsed_ms = ODOM_U32_MAX;
            }
        }

        /* ---- 阶段2：采样 --------------------------------------------
         * 采样前记录位姿代次(epoch)。采样到提交之间若有外部reset
         * 插入，由commit/rebase在锁内比对epoch兜底(BUSY)。
         * 双轮tick与IMU航向紧邻读取，视为同一时刻样本。
         */
        err = odom_get_epoch(&start_epoch);
        if (PLATFORM_IS_ERR(err)) {
            odom_wait_period(&last_wake);
            continue;
        }

        err = motor_port_get_ticks(&left_now, &right_now);
        if (PLATFORM_IS_OK(err)) {
            err = imu_port_get_att(&att);
        }
        if (err == PLATFORM_ERR_NO_RESOURCE) {
            /* IMU数据未就绪不算故障，保持模式等下一周期 */
            odom_wait_period(&last_wake);
            continue;
        }
        if (PLATFORM_IS_ERR(err)) {
            odom_mark_error();
            odom_wait_period(&last_wake);
            continue;
        }

        /* ---- 阶段3：基线建立/重建 -----------------------------------
         * 首次进入或epoch换代（外部reset过）时，本周期不做积分：
         * 存双轮差分起点与IMU航向基线，并在锁内重算航向偏置，
         * 使发布航向从当前位姿航向连续延伸。
         */
        if (have_base == 0U || start_epoch != base_epoch) {
            err = odom_rebase(att.yaw_deg, start_epoch, &yaw_off);
            if (PLATFORM_IS_ERR(err)) {
                odom_wait_period(&last_wake);
                continue;
            }
            last_left = left_now;
            last_right = right_now;
            last_yaw = odom_norm_yaw(att.yaw_deg + yaw_off);
            elapsed_ms = 0U;
            base_epoch = start_epoch;
            have_base = 1U;
            odom_wait_period(&last_wake);
            continue;
        }

        /* ---- 阶段4：运动学解算 --------------------------------------
         * 位移：tick增量 → 单轮位移(mm) → 底盘中心位移取均值。
         * 航向：IMU绝对角加偏置，与上次提交航向做最短角差；
         * 单周期|Δyaw|须小于180°，10ms周期下地面车恒成立。
         * 角速度直接取陀螺仪，不用角差除以dt。
         */
        delta_left = odom_tick_delta(left_now, last_left);
        delta_right = odom_tick_delta(right_now, last_right);
        left_mm = (float)delta_left * (float)g_odom_cfg.left_sign *
                  g_odom_cfg.left_mm_tick;
        right_mm = (float)delta_right * (float)g_odom_cfg.right_sign *
                   g_odom_cfg.right_mm_tick;
        yaw_new = odom_norm_yaw(att.yaw_deg + yaw_off);
        step.move_mm = (left_mm + right_mm) * 0.5F;
        step.dyaw_deg = odom_norm_yaw(yaw_new - last_yaw);
        step.yaw_new_deg = yaw_new;
        step.w_deg_s = att.w_deg_s;
        step.elapsed_ms = elapsed_ms;
        step.pose_epoch = start_epoch;

        if (!isfinite(left_mm) || !isfinite(right_mm) ||
            !isfinite(step.move_mm) || !isfinite(step.dyaw_deg)) {
            odom_mark_error();
            odom_wait_period(&last_wake);
            continue;
        }

        /* ---- 阶段5：提交与推进基线 ----------------------------------
         * 带start_epoch提交，由commit在持锁状态下校验epoch：
         * 返回BUSY表示采样到提交之间发生了reset，本次积分作废
         * 并重建基线；其他错误保留基线，等下一周期重试（dt继续累加）。
         * 提交成功才滚动双轮与航向基线并清零dt。
         */
        err = odom_commit_step(&step);
        if (err == PLATFORM_ERR_BUSY) {
            elapsed_ms = 0U;
            have_base = 0U;
            odom_wait_period(&last_wake);
            continue;
        }
        if (PLATFORM_IS_ERR(err)) {
            odom_wait_period(&last_wake);
            continue;
        }

        last_left = left_now;
        last_right = right_now;
        last_yaw = yaw_new;
        elapsed_ms = 0U;
        odom_wait_period(&last_wake);
    }
}

platform_err_t server_odom_init(const server_odom_cfg_t *cfg)
{
    motor_port_cfg_t port_cfg; /* 电机端口槽位绑定 */
    platform_err_t err;        /* 平台接口返回状态 */

    if (cfg == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (!isfinite(cfg->left_mm_tick) ||
        !isfinite(cfg->right_mm_tick) ||
        cfg->left_mm_tick <= 0.0F ||
        cfg->right_mm_tick <= 0.0F) {
        return PLATFORM_ERR_PARAM;
    }
    if ((cfg->left_sign != -1 && cfg->left_sign != 1) ||
        (cfg->right_sign != -1 && cfg->right_sign != 1)) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_odom_mutex != NULL || g_odom_task != NULL) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 槽位范围与left==right由电机端口负责校验 */
    port_cfg.left_id = cfg->left_id;
    port_cfg.right_id = cfg->right_id;
    err = motor_port_init(&port_cfg);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = imu_port_init();
    if (PLATFORM_IS_ERR(err)) {
        (void)motor_port_deinit();
        return err;
    }

    err = odom_map_osal(osal_mutex_create(&g_odom_mutex));
    if (PLATFORM_IS_ERR(err)) {
        (void)imu_port_deinit();
        (void)motor_port_deinit();
        return err;
    }

    g_odom_cfg = *cfg;
    g_odom_state = (server_odom_state_t){0};
    g_odom_x_mm = 0.0F;
    g_odom_y_mm = 0.0F;
    g_pose_epoch = 0U;

    err = odom_map_osal(osal_task_create("odom_task", server_odom_task,
                                         SERVER_ODOM_STACK_SIZE,
                                         SERVER_ODOM_TASK_PRIO,
                                         &g_odom_task, NULL));

    /* 任务创建失败时逆序回收Mutex与两个端口 */
    if (PLATFORM_IS_ERR(err)) {
        g_odom_cfg = (server_odom_cfg_t){0};
        g_pose_epoch = 0U;
        (void)osal_mutex_delete(g_odom_mutex);
        g_odom_mutex = NULL;
        (void)imu_port_deinit();
        (void)motor_port_deinit();
        return err;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t server_odom_deinit(void)
{
    platform_err_t err;     /* 平台接口返回状态 */
    platform_err_t imu_err; /* IMU端口注销结果 */
    platform_err_t mot_err; /* 电机端口注销结果 */

    if (g_odom_mutex == NULL || g_odom_task == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (osal_task_get_current_handle() == g_odom_task) {
        return PLATFORM_ERR_BUSY;
    }

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    (void)osal_task_delete(g_odom_task); /* 任务态调用不失败 */
    g_odom_task = NULL;
    g_odom_cfg = (server_odom_cfg_t){0};
    g_odom_state = (server_odom_state_t){0};
    g_odom_x_mm = 0.0F;
    g_odom_y_mm = 0.0F;
    g_pose_epoch = 0U;

    err = odom_map_osal(osal_mutex_give(g_odom_mutex));
    (void)osal_mutex_delete(g_odom_mutex);
    g_odom_mutex = NULL;

    /* 任务已删除，端口只剩本线程访问，安全注销 */
    imu_err = imu_port_deinit();
    mot_err = motor_port_deinit();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (PLATFORM_IS_ERR(imu_err)) {
        return imu_err;
    }
    return mot_err;
}

platform_err_t server_odom_set_pose(const server_odom_pose_t *pose)
{
    float yaw_deg;      /* 归一化后的航向角 */
    platform_err_t err; /* 平台接口返回状态 */

    if (pose == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (!isfinite(pose->yaw_deg)) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_odom_task == NULL || g_odom_mutex == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    yaw_deg = odom_norm_yaw(pose->yaw_deg);

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (g_odom_task == NULL) {
        err = odom_map_osal(osal_mutex_give(g_odom_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_odom_state.x_mm = pose->x_mm;
    g_odom_state.y_mm = pose->y_mm;
    g_odom_x_mm = (float)pose->x_mm;
    g_odom_y_mm = (float)pose->y_mm;
    g_odom_state.yaw_deg = yaw_deg;
    g_odom_state.vx_mm_s = 0;
    g_odom_state.vy_mm_s = 0;
    g_odom_state.w_deg_s = 0.0F;
    g_pose_epoch++;

    return odom_map_osal(osal_mutex_give(g_odom_mutex));
}

platform_err_t server_odom_get(server_odom_state_t *state)
{
    server_odom_state_t snapshot; /* 同一解算时刻的状态快照 */
    platform_err_t err;           /* 平台接口返回状态 */

    if (state == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_odom_task == NULL || g_odom_mutex == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = odom_map_osal(osal_mutex_take(g_odom_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (g_odom_task == NULL) {
        err = odom_map_osal(osal_mutex_give(g_odom_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    snapshot = g_odom_state;
    err = odom_map_osal(osal_mutex_give(g_odom_mutex));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    *state = snapshot;
    return PLATFORM_ERR_OK;
}
