/**
 * @file    service_track_ctrl.c
 * @brief   差速底盘循迹控制服务实现
 * @note    - 模式与目标速度作为一次完整命令提交
 *          - 公共速度单位为mm/s，不暴露电机转速或RTOS类型
 *          - IMU绝对航向经set_pose偏置对齐到世界坐标
 *          - OSAL状态码在本层统一映射为platform_err_t
 */

#include <math.h>

#include "service_track_ctrl.h"

#include "imu_ctrl_port.h"
#include "motor_ctrl_port.h"
#include "track_port.h"
#include "osal_mutex.h"
#include "osal_task.h"

#define TRACK_ERR_LPF_ALPHA        0.2f   /* 循迹模块低通滤波系数 */
#define TRACK_ERR_SOS_MM           10.0f  /* 循迹危险阈值 */
#define TRACK_YAW_PERIOD_DEG       360.0f /* 航向角周期，deg */
#define TRACK_YAW_HALF_DEG         180.0f /* 航向角半周期，deg */
#define TRACK_PI_RAD               3.14159265358979323846f /* 圆周率 */
#define TRACK_RAD_PER_DEG          (TRACK_PI_RAD / 180.0f) /* deg转rad */
#define TRACK_K1_ERR               0.2f   /* 回线增益(deg/s)/mm，待整定 */
#define TRACK_K2_YAW               1.8f   /* 锁向增益(deg/s)/deg，待整定 */
#define TRACK_WHEEL_DIST_MM        135.0f /* 左右轮距mm，待实车标定 */
#define TRACK_SENSOR_TO_WHEEL_MM   160.0f /* 前传感器到后轮轴距离 */

#define TRACK_TURN_LINEAR_VEL_COMP 0.0f   /* 原地转弯时的补偿量 */

#define TRACK_TURN_KP              10.0f   /* 转向比例(deg/s)/deg，待整定 */
#define TRACK_TURN_KI              0.0f   /* 转向积分，0即纯PD，待整定 */
#define TRACK_TURN_KD              3.5f   /* 转向微分增益，待整定 */
#define TRACK_TURN_W_MAX_DEG_S     360.0f /* 转向角速度限幅，待整定 */
#define TRACK_TURN_I_MAX_DEG_S     20.0f  /* 转向积分项限幅，待整定 */
#define TRACK_PID_DT_S (SERVER_CTRL_PERIOD_MS / 1000.0f) /* 控制周期，s */

/** 位置式PID参数与运行状态。 */
typedef struct {
    float kp;        /* 比例增益 */
    float ki;        /* 积分增益 */
    float kd;        /* 微分增益 */
    float out_max;   /* 输出限幅，绝对值 */
    float integ_max; /* 积分项限幅，绝对值 */
    float integ;     /* 积分项累加值 */
    float prev_err;  /* 上周期误差 */
} track_pid_t;

static uint8_t g_track_ctrl_inited = 0U; /* 服务初始化标志 */
static uint8_t g_track_state = 0U;       /* 当前循迹控制状态 */
static float   g_target_yaw_deg = 0.0f;  /* 目标航向角，deg */
static float   g_target_v_mm_s = 0.0f;   /* 目标车体速度，mm/s */
static osal_task_handle_t  g_track_task;  /* 控制任务句柄 */
static osal_mutex_handle_t g_track_mutex; /* 命令三元组互斥锁 */

/**
 * @brief  将OSAL状态码映射为平台统一状态码
 * @param  st OSAL返回状态
 * @retval 对应的platform_err_t
 */
static platform_err_t track_map_osal(osal_status_t st)
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

/**
 * @brief  将角度归一化到[-180, 180)
 * @param  yaw_deg 任意角度，deg
 * @retval 归一化后的角度，deg
 */
static float track_norm_yaw(float yaw_deg)
{
    float norm_deg; /* 归一化后的角度 */

    norm_deg = fmodf(yaw_deg + TRACK_YAW_HALF_DEG,
                     TRACK_YAW_PERIOD_DEG);
    if (norm_deg < 0.0f) {
        norm_deg += TRACK_YAW_PERIOD_DEG;
    }

    return norm_deg - TRACK_YAW_HALF_DEG;
}

/**
 * @brief  复位PID积分并对齐微分基准
 * @param  pid PID实例
 * @param  err 当前误差，作为下周期微分基准
 */
static void track_pid_reset(track_pid_t *pid, float err)
{
    pid->integ = 0.0f;
    pid->prev_err = err;
}

/**
 * @brief  位置式PID单步解算
 * @param  pid PID实例，积分与误差状态随调用推进
 * @param  err 当前误差
 * @retval 限幅后的控制输出
 * @note   周期固定为TRACK_PID_DT_S，Ki为0时退化为PD
 */
static float track_pid_calc(track_pid_t *pid, float err)
{
    float out; /* 未限幅的PID输出 */

    /* 积分先累加后钳位，长时间大误差不致饱和 */
    pid->integ += pid->ki * err * TRACK_PID_DT_S;
    if (pid->integ > pid->integ_max) {
        pid->integ = pid->integ_max;
    } else if (pid->integ < -pid->integ_max) {
        pid->integ = -pid->integ_max;
    }

    out = pid->kp * err + pid->integ +
          pid->kd * (err - pid->prev_err) / TRACK_PID_DT_S;
    pid->prev_err = err;

    /* 输出限幅，保护下游差速解算 */
    if (out > pid->out_max) {
        out = pid->out_max;
    } else if (out < -pid->out_max) {
        out = -pid->out_max;
    }

    return out;
}

/**
 * @brief  采样循迹端口并低通滤波横向误差
 * @retval 滤波后的车体横向误差，右正左负，mm
 * @note   端口读取失败时返回0，等效直行不修正
 */
static float track_err_get(void)
{
    static float track_err = 0.0f;   /* 滤波后的横向误差状态 */
    track_port_result_t track_state; /* 循迹端口采样结果 */
    platform_err_t err;              /* 端口读取状态 */

    err = track_port_get(&track_state);
    if (PLATFORM_IS_ERR(err)) {
        /* 端口未初始化或读取失败：放弃修正，直行最可预期 */
        return 0.0f;
    }

    switch (track_state.state) {
        case TRACK_PORT_NO_LINE:
            if (fabsf(track_err) > TRACK_ERR_SOS_MM) {
                /* 大误差丢线视为冲出线外，保持上次修正找回 */
                return track_err;
            }
            /* 小误差丢线视为线间空隙，直行通过 */
            return 0.0f;
        case TRACK_PORT_VALID:
            track_err = TRACK_ERR_LPF_ALPHA * track_state.body_err_mm +
                        (1.0f - TRACK_ERR_LPF_ALPHA) * track_err;
            return track_err;
        case TRACK_PORT_AMBIGUOUS:
        default:
            /* 多线或未知状态：信息不可用，冻结滤波保持上次误差 */
            return track_err;
    }
}

/**
 * @brief  差速解算并下发左右轮目标速度
 * @param  v_mm_s  车体前向速度，mm/s，正值向前
 * @param  w_deg_s 车体角速度，deg/s，顺时针为正
 */
static void track_apply_motion(float v_mm_s, float w_deg_s)
{
    float diff_mm_s; /* 单侧差速分量，mm/s */

    /* 顺时针(w>0)转向时左轮在外圈，左加右减 */
    diff_mm_s = w_deg_s * TRACK_RAD_PER_DEG *
                (TRACK_WHEEL_DIST_MM * 0.5f);
    motor_ctrl_set(v_mm_s + diff_mm_s, v_mm_s - diff_mm_s);
}

/** 维持SERVER_CTRL_PERIOD_MS控制周期；异常时挂起自身。 */
static void track_wait_period(osal_tick_type_t *last_wake)
{
    osal_status_t st; /* OSAL定周期延时状态 */

    st = osal_task_wait_until(last_wake, SERVER_CTRL_PERIOD_MS);
    if (st != OSAL_OK) {
        /* 周期机制异常时挂起自身，避免空转抢占 */
        (void)osal_task_suspend(NULL);
        *last_wake = osal_task_get_tick_count();
    }
}

/**
 * @brief  循迹控制周期任务，按快照命令解算并输出
 * @param  argument 任务参数，未使用
 */
static void server_track_ctrl_task(void *argument)
{
    float track_err_now;        /* 滤波后横向误差，mm */
    float yaw_now_deg;          /* 当前顺时针航向角，deg */
    float yaw_err_deg;          /* 归一化航向误差，deg */
    float target_w;             /* 目标顺时针角速度，deg/s */
    float yaw_tgt_deg;          /* 快照的目标航向，deg */
    float v_tgt_mm_s;           /* 快照的目标速度，mm/s */
    float yaw_tgt_prev;         /* 上周期目标航向，deg */
    uint8_t mode_now;           /* 快照的控制模式 */
    uint8_t mode_prev;          /* 上周期控制模式 */
    track_pid_t turn_pid;       /* TURN航向PID实例 */
    osal_tick_type_t last_wake; /* 上次周期唤醒tick */
    platform_err_t err;         /* 平台接口返回状态 */

    (void)argument;
    mode_prev = (uint8_t)TRACK_CTRL_MODE_STOP;
    yaw_tgt_prev = 0.0f;
    /* TURN转向PID装配：Ki=0纯PD起步，增益待实车整定 */
    turn_pid.kp = TRACK_TURN_KP;
    turn_pid.ki = TRACK_TURN_KI;
    turn_pid.kd = TRACK_TURN_KD;
    turn_pid.out_max = TRACK_TURN_W_MAX_DEG_S;
    turn_pid.integ_max = TRACK_TURN_I_MAX_DEG_S;
    track_pid_reset(&turn_pid, 0.0f);
    last_wake = osal_task_get_tick_count();

    while (1) {
        /* 锁内快照一次完整命令，解算全程使用本地副本 */
        err = track_map_osal(osal_mutex_take(g_track_mutex,
                                             OSAL_MAX_DELAY));
        if (PLATFORM_IS_ERR(err)) {
            track_wait_period(&last_wake);
            continue;
        }
        mode_now = g_track_state;
        yaw_tgt_deg = g_target_yaw_deg;
        v_tgt_mm_s = g_target_v_mm_s;
        err = track_map_osal(osal_mutex_give(g_track_mutex));
        if (PLATFORM_IS_ERR(err)) {
            track_wait_period(&last_wake);
            continue;
        }

        /* 离开STOP先使能电机闭环，失败不推进模式下周期重试 */
        if (mode_now != (uint8_t)TRACK_CTRL_MODE_STOP &&
            mode_prev == (uint8_t)TRACK_CTRL_MODE_STOP) {
            err = motor_ctrl_start();
            if (PLATFORM_IS_ERR(err)) {
                track_wait_period(&last_wake);
                continue;
            }
        }

        track_err_now = track_err_get();
        yaw_now_deg = imu_ctrl_get();

        switch (mode_now) {
            case TRACK_CTRL_MODE_STOP:
                /* 停止模式：失能电机输出，等待下一命令 */
                motor_ctrl_stop();
                break;
            case TRACK_CTRL_MODE_TRACK_DIR:
                /* 已知方向巡线：横向误差回线+目标航向锁向 */
                yaw_err_deg = track_norm_yaw(yaw_tgt_deg -
                                             yaw_now_deg);
                /* err右偏取负逆时针回线，yaw_err>0取正顺时针追向 */
                target_w = TRACK_K2_YAW * yaw_err_deg -
                           TRACK_K1_ERR * (track_err_now + 
                           TRACK_SENSOR_TO_WHEEL_MM * sinf(yaw_err_deg * TRACK_RAD_PER_DEG));
                track_apply_motion(v_tgt_mm_s, target_w);
                break;
            case TRACK_CTRL_MODE_TRACK:
                /* 纯巡线模式：仅PID跟线 */

                break;
            case TRACK_CTRL_MODE_TURN:
                /* 转向模式：航向误差位置式PID解算转向角速度 */
                yaw_err_deg = track_norm_yaw(yaw_tgt_deg -
                                             yaw_now_deg);
                if (mode_prev != (uint8_t)TRACK_CTRL_MODE_TURN ||
                    yaw_tgt_deg != yaw_tgt_prev) {
                    /* 新进模式或目标更新：复位防积分残留与微分踢 */
                    track_pid_reset(&turn_pid, yaw_err_deg);
                }
                target_w = track_pid_calc(&turn_pid, yaw_err_deg);
                track_apply_motion(TRACK_TURN_LINEAR_VEL_COMP,
                                   target_w);
                break;
            default:
                break;
        }

        /* 历史状态延至此更新，供TURN分支识别新命令做复位 */
        mode_prev = mode_now;
        yaw_tgt_prev = yaw_tgt_deg;
        track_wait_period(&last_wake);
    }
}

platform_err_t track_ctrl_init(void)
{
    track_port_result_t probe; /* 探测读取的循迹结果 */
    platform_err_t err;        /* 平台接口返回状态 */

    if (g_track_ctrl_inited != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 探测循迹Port就绪；电机/航向Port次序由装配根保证 */
    err = track_port_get(&probe);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    err = track_map_osal(osal_mutex_create(&g_track_mutex));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_track_state = (uint8_t)TRACK_CTRL_MODE_STOP;
    g_target_yaw_deg = 0.0f;
    g_target_v_mm_s = 0.0f;

    err = track_map_osal(osal_task_create("track_ctrl",
                                          server_track_ctrl_task,
                                          SERVER_CTRL_STACK_SIZE,
                                          SERVER_CTRL_TASK_PRIO,
                                          &g_track_task, NULL));
    if (PLATFORM_IS_ERR(err)) {
        (void)osal_mutex_delete(g_track_mutex);
        g_track_mutex = NULL;
        return err;
    }

    g_track_ctrl_inited = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t track_ctrl_deinit(void)
{
    platform_err_t err; /* 平台接口返回状态 */

    if (g_track_ctrl_inited == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (osal_task_get_current_handle() == g_track_task) {
        return PLATFORM_ERR_BUSY;
    }

    err = track_map_osal(osal_mutex_take(g_track_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    (void)osal_task_delete(g_track_task); /* 任务态调用不失败 */
    g_track_task = NULL;
    g_track_ctrl_inited = 0U;
    motor_ctrl_stop(); /* 服务退场前失能输出，车体滑行 */
    g_track_state = (uint8_t)TRACK_CTRL_MODE_STOP;
    g_target_yaw_deg = 0.0f;
    g_target_v_mm_s = 0.0f;

    err = track_map_osal(osal_mutex_give(g_track_mutex));
    (void)osal_mutex_delete(g_track_mutex);
    g_track_mutex = NULL;

    /* Port生命周期归装配根，本层不做Port反初始化 */
    return err;
}

platform_err_t track_ctrl_set_mode(track_ctrl_mode_t mode,
                                   float target_yaw_deg,
                                   int32_t speed_mm_s)
{
    platform_err_t err; /* 平台接口返回状态 */

    if (mode != TRACK_CTRL_MODE_STOP &&
        mode != TRACK_CTRL_MODE_TRACK_DIR &&
        mode != TRACK_CTRL_MODE_TRACK &&
        mode != TRACK_CTRL_MODE_TURN) {
        return PLATFORM_ERR_PARAM;
    }
    if (mode == TRACK_CTRL_MODE_STOP && speed_mm_s != 0) {
        return PLATFORM_ERR_PARAM;
    }
    if ((mode == TRACK_CTRL_MODE_TRACK_DIR ||
         mode == TRACK_CTRL_MODE_TURN) &&
        !isfinite(target_yaw_deg)) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_track_ctrl_inited == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = track_map_osal(osal_mutex_take(g_track_mutex, OSAL_MAX_DELAY));
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    if (g_track_ctrl_inited == 0U) {
        /* 等锁期间服务被deinit，放弃本次提交 */
        err = track_map_osal(osal_mutex_give(g_track_mutex));
        if (PLATFORM_IS_ERR(err)) {
            return err;
        }
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* 三个字段锁内一次写完，任务按同一拍快照消费 */
    if (mode == TRACK_CTRL_MODE_TRACK_DIR ||
        mode == TRACK_CTRL_MODE_TURN) {
        g_target_yaw_deg = track_norm_yaw(target_yaw_deg);
    } else {
        g_target_yaw_deg = 0.0f;
    }
    g_target_v_mm_s = (float)speed_mm_s;
    g_track_state = (uint8_t)mode;

    return track_map_osal(osal_mutex_give(g_track_mutex));
}
