/**
 * @file    bsp_adapter_port_motor.c
 * @brief   双路电机Adapter：绑定GPIO/PWM/编码器/节拍与Driver
 * @note    任务侧改状态时短暂屏蔽TIM9；状态读取使用序号双查快照。
 */

#include "bsp_adapter_port_motor.h"

#include <stddef.h>

#include "board_motor_config.h"
#include "bsp_motor_driver.h"
#include "bsp_wrapper_motor.h"
#include "encoder_port.h"
#include "io_port.h"
#include "pwm_port.h"
#include "timer_port.h"

typedef struct {
    motor_core_t              core;      /* 电机Driver对象 */
    motor_hw_if_t             hw;        /* 注入Driver的硬件接口 */
    core_pwm_t                pwm_id;    /* 逻辑PWM通道 */
    core_io_id_t              in1_id;    /* 方向IN1 */
    core_io_id_t              in2_id;    /* 方向IN2 */
    core_enc_t                enc_id;    /* 逻辑编码器 */
    motor_dir_t               fwd_dir;   /* 逻辑正向物理方向 */
    int8_t                    enc_sign;  /* 编码器方向修正 */
    uint32_t                  enc_cpr;   /* 输出轴每转计数 */
    float                     filter;    /* 本电机速度滤波系数 */
    motor_pid_cfg_t           pid_cfg;   /* 本电机默认控制参数 */
    volatile uint32_t         fault;     /* MOTOR_FAULT_*锁存 */
    volatile uint32_t         seq;       /* 偶数表示快照稳定 */
    volatile bsp_motor_state_t snap;     /* 控制拍末状态快照 */
} motor_adp_t;

static motor_adp_t s_motor[CFG_MOTOR_COUNT];

/** @brief Driver错误到平台错误的语义映射 */
static platform_err_t motor_map_status(motor_status_t status)
{
    switch (status) {
        case MOTOR_OK:
            return PLATFORM_ERR_OK;
        case MOTOR_ERR_PARAM:
            return PLATFORM_ERR_PARAM;
        case MOTOR_ERR_STATE:
            return PLATFORM_ERR_NOT_INITIALIZED;
        case MOTOR_ERR_HW:
        default:
            return PLATFORM_ERR_FAIL;
    }
}

/** @brief 从Wrapper设备表取Adapter上下文 */
static motor_adp_t *motor_get_ctx(bsp_motor_dev_t *dev)
{
    if ((dev == NULL) || (dev->ctx == NULL)) {
        return NULL;
    }
    return (motor_adp_t *)dev->ctx;
}

/** @brief 安全切换方向脚并写入PWM */
static motor_status_t motor_hw_output(void *ctx,
                                      motor_dir_t dir,
                                      uint16_t pwm)
{
    motor_adp_t *motor = (motor_adp_t *)ctx; /* 目标电机 */
    core_io_level_t in1;                    /* IN1目标电平 */
    core_io_level_t in2;                    /* IN2目标电平 */

    if ((motor == NULL) ||
        ((dir != MOTOR_DIR_CW) && (dir != MOTOR_DIR_CCW))) {
        return MOTOR_ERR_PARAM;
    }
    if (PLATFORM_IS_ERR(core_pwm_set(motor->pwm_id, 0U))) {
        return MOTOR_ERR_HW;
    }

    if (pwm == 0U) {
        in1 = CORE_IO_LOW;
        in2 = CORE_IO_LOW;
    } else if (dir == MOTOR_DIR_CW) {
        in1 = CORE_IO_HIGH;
        in2 = CORE_IO_LOW;
    } else {
        in1 = CORE_IO_LOW;
        in2 = CORE_IO_HIGH;
    }

    if (PLATFORM_IS_ERR(core_io_write(motor->in1_id, in1)) ||
        PLATFORM_IS_ERR(core_io_write(motor->in2_id, in2))) {
        return MOTOR_ERR_HW;
    }
    if ((pwm != 0U) &&
        PLATFORM_IS_ERR(core_pwm_set(motor->pwm_id, pwm))) {
        return MOTOR_ERR_HW;
    }
    return MOTOR_OK;
}

/** @brief 将Driver状态写入任务侧一致快照 */
static void motor_snap_write(motor_adp_t *motor)
{
    motor_core_state_t state; /* Driver当前状态 */
    bsp_motor_run_t run;      /* Wrapper运行状态 */

    if (motor_core_get_state(&motor->core, &state) != MOTOR_OK) {
        return;
    }
    if (motor->fault != MOTOR_FAULT_NONE) {
        run = BSP_MOTOR_ERROR;
    } else if (state.enabled != 0U) {
        run = BSP_MOTOR_ACTIVE;
    } else {
        run = BSP_MOTOR_OFF;
    }

    motor->seq++;
    motor->snap.target_rps = state.target_rps;
    motor->snap.speed_rps  = state.speed_rps;
    motor->snap.raw_rps    = state.raw_rps;
    motor->snap.p_term     = state.p_term;
    motor->snap.i_term     = state.i_term;
    motor->snap.ff_term    = state.ff_term;
    motor->snap.position   = state.position;
    motor->snap.duty       = state.duty;
    motor->snap.fault      = motor->fault;
    motor->snap.run        = run;
    motor->seq++;
}

/** @brief 单电机执行一拍编码器与闭环更新 */
static void motor_tick_update(motor_adp_t *motor)
{
    uint16_t ticks;        /* 当前硬件编码器计数 */
    platform_err_t enc_st; /* 编码器读取结果 */
    motor_status_t drv_st; /* Driver控制结果 */

    if (motor->core.inited == 0U) {
        return;
    }

    enc_st = core_enc_read(motor->enc_id, &ticks);
    if (PLATFORM_IS_ERR(enc_st)) {
        motor->fault |= MOTOR_FAULT_HW;
        (void)motor_core_stop(&motor->core);
        motor_snap_write(motor);
        return;
    }

    drv_st = motor_core_update(&motor->core, ticks);
    if (drv_st != MOTOR_OK) {
        motor->fault |= MOTOR_FAULT_HW;
        (void)motor_core_stop(&motor->core);
    }
    motor_snap_write(motor);
}

/** @brief TIM9控制节拍ISR入口 */
static void motor_tick_isr(void)
{
    uint32_t i; /* 电机槽位游标 */

    for (i = 0U; i < CFG_MOTOR_COUNT; i++) {
        motor_tick_update(&s_motor[i]);
    }
}

/** @brief 装配Driver配置并初始化一个槽位 */
static platform_err_t motor_op_init(bsp_motor_dev_t *dev)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_core_cfg_t cfg;                    /* Driver初始化配置 */
    platform_err_t err;                      /* MCU Port结果 */
    uint16_t pwm_max;                        /* TIM1实际ARR */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    err = core_pwm_get_max(motor->pwm_id, &pwm_max);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    cfg.fwd_dir          = motor->fwd_dir;
    cfg.enc_sign         = motor->enc_sign;
    cfg.pwm_max          = pwm_max;
    cfg.enc_cpr          = motor->enc_cpr;
    cfg.sample_s         = CFG_MOTOR_TICK_S;
    cfg.filter           = motor->filter;
    cfg.pid_cfg          = motor->pid_cfg;

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    motor->fault = MOTOR_FAULT_NONE;
    if (motor_core_init(&motor->core, &motor->hw, &cfg) != MOTOR_OK) {
        (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
        return PLATFORM_ERR_PARAM;
    }
    motor_snap_write(motor);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);

    if (motor_hw_output(motor, motor->fwd_dir, 0U) != MOTOR_OK) {
        return PLATFORM_ERR_FAIL;
    }
    err = core_pwm_start(motor->pwm_id);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = core_enc_start(motor->enc_id);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    return core_timer_start(CFG_MOTOR_TICK_ID);
}

/** @brief 安全停止并释放一个槽位的MCU能力 */
static platform_err_t motor_op_deinit(bsp_motor_dev_t *dev)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    platform_err_t ret = PLATFORM_ERR_OK;    /* 首个错误 */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    if ((motor->core.inited != 0U) &&
        (motor_core_stop(&motor->core) != MOTOR_OK)) {
        ret = PLATFORM_ERR_FAIL;
    }
    motor->core.inited = 0U;
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);

    if (PLATFORM_IS_ERR(core_enc_stop(motor->enc_id)) &&
        PLATFORM_IS_OK(ret)) {
        ret = PLATFORM_ERR_FAIL;
    }
    if (PLATFORM_IS_ERR(core_pwm_stop(motor->pwm_id)) &&
        PLATFORM_IS_OK(ret)) {
        ret = PLATFORM_ERR_FAIL;
    }
    return ret;
}

/** @brief 使能一个槽位闭环运行 */
static platform_err_t motor_op_start(bsp_motor_dev_t *dev)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_status_t status;                   /* Driver结果 */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    motor->fault = MOTOR_FAULT_NONE;
    status = motor_core_start(&motor->core);
    motor_snap_write(motor);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    return motor_map_status(status);
}

/** @brief 停止一个槽位并清零目标 */
static platform_err_t motor_op_stop(bsp_motor_dev_t *dev)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_status_t status;                   /* Driver结果 */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    status = motor_core_stop(&motor->core);
    motor_snap_write(motor);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    return motor_map_status(status);
}

/** @brief 更新一个槽位目标转速 */
static platform_err_t motor_op_set_rps(bsp_motor_dev_t *dev, float rps)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    return motor_map_status(motor_core_set_rps(&motor->core, rps));
}

/** @brief 在线更新一个槽位PID参数 */
static platform_err_t motor_op_set_pid(
    bsp_motor_dev_t *dev,
    const bsp_motor_pid_t *cfg)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_pid_cfg_t pid_cfg;                 /* Driver参数 */
    motor_status_t status;                   /* Driver结果 */

    if ((motor == NULL) || (cfg == NULL)) {
        return PLATFORM_ERR_PARAM;
    }

    pid_cfg.kp      = cfg->kp;
    pid_cfg.ki      = cfg->ki;
    pid_cfg.kd      = cfg->kd;
    pid_cfg.kff     = cfg->kff;
    pid_cfg.ff_bias = cfg->ff_bias;
    pid_cfg.i_limit = cfg->i_limit;
    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    status = motor_core_set_pid(&motor->core, &pid_cfg);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    return motor_map_status(status);
}

/** @brief 读取一个槽位PID参数 */
static platform_err_t motor_op_get_pid(bsp_motor_dev_t *dev,
                                       bsp_motor_pid_t *cfg)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_pid_cfg_t pid_cfg;                 /* Driver参数 */
    motor_status_t status;                   /* Driver结果 */

    if ((motor == NULL) || (cfg == NULL)) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    status = motor_core_get_pid(&motor->core, &pid_cfg);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    if (status != MOTOR_OK) {
        return motor_map_status(status);
    }

    cfg->kp      = pid_cfg.kp;
    cfg->ki      = pid_cfg.ki;
    cfg->kd      = pid_cfg.kd;
    cfg->kff     = pid_cfg.kff;
    cfg->ff_bias = pid_cfg.ff_bias;
    cfg->i_limit = pid_cfg.i_limit;
    return PLATFORM_ERR_OK;
}

/** @brief 在线更新一个槽位速度滤波系数 */
static platform_err_t motor_op_set_filt(bsp_motor_dev_t *dev,
                                        float filter)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_status_t status;                   /* Driver结果 */

    if (motor == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    status = motor_core_set_filt(&motor->core, filter);
    motor_snap_write(motor);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    return motor_map_status(status);
}

/** @brief 读取一个槽位速度滤波系数 */
static platform_err_t motor_op_get_filt(bsp_motor_dev_t *dev,
                                        float *filter)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    motor_status_t status;                   /* Driver结果 */

    if ((motor == NULL) || (filter == NULL)) {
        return PLATFORM_ERR_PARAM;
    }

    (void)core_timer_mask(CFG_MOTOR_TICK_ID);
    status = motor_core_get_filt(&motor->core, filter);
    (void)core_timer_unmask(CFG_MOTOR_TICK_ID);
    return motor_map_status(status);
}

/** @brief 通过序号双查读取一个槽位状态 */
static platform_err_t motor_op_get_state(
    bsp_motor_dev_t *dev,
    bsp_motor_state_t *state)
{
    motor_adp_t *motor = motor_get_ctx(dev); /* 目标上下文 */
    bsp_motor_state_t copy;                  /* 一致本地副本 */
    uint32_t seq_before;                     /* 读取前序号 */
    uint32_t seq_after;                      /* 读取后序号 */

    if ((motor == NULL) || (state == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    if (motor->core.inited == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    do {
        seq_before = motor->seq;
        if ((seq_before & 1U) != 0U) {
            continue;
        }
        copy.target_rps = motor->snap.target_rps;
        copy.speed_rps  = motor->snap.speed_rps;
        copy.raw_rps    = motor->snap.raw_rps;
        copy.p_term     = motor->snap.p_term;
        copy.i_term     = motor->snap.i_term;
        copy.ff_term    = motor->snap.ff_term;
        copy.position   = motor->snap.position;
        copy.duty       = motor->snap.duty;
        copy.fault      = motor->snap.fault;
        copy.run        = motor->snap.run;
        seq_after = motor->seq;
    } while (seq_before != seq_after);

    *state = copy;
    return PLATFORM_ERR_OK;
}

platform_err_t motor_adp_register(void)
{
    bsp_motor_dev_t dev; /* Wrapper设备表 */
    platform_err_t err;  /* 注册结果 */

    s_motor[0].pwm_id   = CFG_MOTOR_A_PWM;
    s_motor[0].in1_id   = CFG_MOTOR_A_IN1;
    s_motor[0].in2_id   = CFG_MOTOR_A_IN2;
    s_motor[0].enc_id   = CFG_MOTOR_A_ENC;
    s_motor[0].enc_cpr  = CFG_MOTOR_A_CPR;
    s_motor[0].filter   = CFG_MOTOR_A_FILTER;
    s_motor[0].pid_cfg.kp = CFG_MOTOR_A_PID_KP;
    s_motor[0].pid_cfg.ki = CFG_MOTOR_A_PID_KI;
    s_motor[0].pid_cfg.kd = CFG_MOTOR_A_PID_KD;
    s_motor[0].pid_cfg.kff = CFG_MOTOR_A_PID_KFF;
    s_motor[0].pid_cfg.ff_bias = CFG_MOTOR_A_PID_FF_BIAS;
    s_motor[0].pid_cfg.i_limit = CFG_MOTOR_A_PID_I_LIMIT;
    s_motor[0].fwd_dir  = (CFG_MOTOR_A_FWD_CW != 0U)
                        ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    s_motor[0].enc_sign = (int8_t)CFG_MOTOR_A_ENC_SIGN;

    s_motor[1].pwm_id   = CFG_MOTOR_B_PWM;
    s_motor[1].in1_id   = CFG_MOTOR_B_IN1;
    s_motor[1].in2_id   = CFG_MOTOR_B_IN2;
    s_motor[1].enc_id   = CFG_MOTOR_B_ENC;
    s_motor[1].enc_cpr  = CFG_MOTOR_B_CPR;
    s_motor[1].filter   = CFG_MOTOR_B_FILTER;
    s_motor[1].pid_cfg.kp = CFG_MOTOR_B_PID_KP;
    s_motor[1].pid_cfg.ki = CFG_MOTOR_B_PID_KI;
    s_motor[1].pid_cfg.kd = CFG_MOTOR_B_PID_KD;
    s_motor[1].pid_cfg.kff = CFG_MOTOR_B_PID_KFF;
    s_motor[1].pid_cfg.ff_bias = CFG_MOTOR_B_PID_FF_BIAS;
    s_motor[1].pid_cfg.i_limit = CFG_MOTOR_B_PID_I_LIMIT;
    s_motor[1].fwd_dir  = (CFG_MOTOR_B_FWD_CW != 0U)
                        ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    s_motor[1].enc_sign = (int8_t)CFG_MOTOR_B_ENC_SIGN;

    s_motor[0].hw.ctx       = &s_motor[0];
    s_motor[0].hw.pf_output = motor_hw_output;
    s_motor[1].hw.ctx       = &s_motor[1];
    s_motor[1].hw.pf_output = motor_hw_output;

    err = core_timer_reg(CFG_MOTOR_TICK_ID, motor_tick_isr);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    dev.pf_init      = motor_op_init;
    dev.pf_deinit    = motor_op_deinit;
    dev.pf_start     = motor_op_start;
    dev.pf_stop      = motor_op_stop;
    dev.pf_set_rps   = motor_op_set_rps;
    dev.pf_set_pid   = motor_op_set_pid;
    dev.pf_get_pid   = motor_op_get_pid;
    dev.pf_set_filt  = motor_op_set_filt;
    dev.pf_get_filt  = motor_op_get_filt;
    dev.pf_get_state = motor_op_get_state;

    dev.ctx = &s_motor[0];
    err = bsp_motor_reg(CFG_MOTOR_A_SLOT, &dev);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    dev.ctx = &s_motor[1];
    return bsp_motor_reg(CFG_MOTOR_B_SLOT, &dev);
}
