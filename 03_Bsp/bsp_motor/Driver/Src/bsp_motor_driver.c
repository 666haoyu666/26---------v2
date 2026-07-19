/**
 * @file    bsp_motor_driver.c
 * @brief   编码器直流电机闭环Driver实现
 * @note    uint16_t回绕差值要求单周期计数变化不超过32767。
 */

#include "bsp_motor_driver.h"

#include <stddef.h>

#define MOTOR_ZERO_RPS (0.0001f) /* 目标停转判定阈值 */

/** @brief 判断浮点数是否为有限值 */
static uint8_t float_valid(float value)
{
    return ((value - value) == 0.0f) ? 1U : 0U;
}

/** @brief 选择带符号输出对应的物理方向 */
static motor_dir_t pick_dir(motor_dir_t fwd_dir, int32_t duty)
{
    if (duty >= 0) {
        return fwd_dir;
    }
    return (fwd_dir == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
}

/** @brief 更新编码器位置与速度 */
static void encoder_update(motor_core_t *motor, uint16_t ticks)
{
    int16_t raw_delta; /* 16位回绕后的原始增量 */
    int32_t delta;     /* 方向修正后的本拍增量 */

    if (motor->first_tick != 0U) {
        motor->last_ticks = ticks;
        motor->first_tick = 0U;
        return;
    }

    raw_delta = (int16_t)(uint16_t)(ticks - motor->last_ticks);
    motor->last_ticks = ticks;
    delta = (int32_t)raw_delta * (int32_t)motor->cfg.enc_sign;
    motor->position += delta;
    motor->raw_rps = (float)delta /
                     ((float)motor->cfg.enc_cpr * motor->cfg.sample_s);

    if (motor->speed_ready == 0U) {
        motor->speed_rps = motor->raw_rps;
        motor->speed_ready = 1U;
    } else {
        motor->speed_rps += motor->cfg.filter *
                            (motor->raw_rps - motor->speed_rps);
    }
}

/** @brief 将带符号控制量应用到硬件 */
static motor_status_t apply_output(motor_core_t *motor, int32_t duty)
{
    motor_dir_t dir; /* 最终物理旋转方向 */
    uint16_t pwm;    /* 最终无符号PWM */

    if (duty > (int32_t)motor->cfg.pwm_max) {
        duty = (int32_t)motor->cfg.pwm_max;
    } else if (duty < -(int32_t)motor->cfg.pwm_max) {
        duty = -(int32_t)motor->cfg.pwm_max;
    }

    dir = pick_dir(motor->cfg.fwd_dir, duty);
    pwm = (duty < 0) ? (uint16_t)(-duty) : (uint16_t)duty;
    if (motor->hw->pf_output(motor->hw->ctx, dir, pwm) != MOTOR_OK) {
        return MOTOR_ERR_HW;
    }

    motor->duty = duty;
    return MOTOR_OK;
}

motor_status_t motor_core_init(motor_core_t *motor,
                               const motor_hw_if_t *hw,
                               const motor_core_cfg_t *cfg)
{
    if ((motor == NULL) || (hw == NULL) || (cfg == NULL) ||
        (hw->pf_output == NULL)) {
        return MOTOR_ERR_PARAM;
    }
    if (((cfg->fwd_dir != MOTOR_DIR_CW) &&
         (cfg->fwd_dir != MOTOR_DIR_CCW)) ||
        ((cfg->enc_sign != 1) && (cfg->enc_sign != -1)) ||
        (cfg->pwm_max == 0U) || (cfg->enc_cpr == 0U) ||
        (float_valid(cfg->sample_s) == 0U) ||
        (float_valid(cfg->filter) == 0U) ||
        (cfg->sample_s <= 0.0f) ||
        (cfg->filter < 0.0f) || (cfg->filter > 1.0f)) {
        return MOTOR_ERR_PARAM;
    }

    motor->hw          = hw;
    motor->cfg         = *cfg;
    motor->target_rps  = 0.0f;
    motor->speed_rps   = 0.0f;
    motor->raw_rps     = 0.0f;
    motor->position    = 0;
    motor->duty        = 0;
    motor->last_ticks  = 0U;
    motor->enabled     = 0U;
    motor->first_tick  = 1U;
    motor->speed_ready = 0U;
    if (motor_pid_init(&motor->pid, &cfg->pid_cfg,
                       cfg->sample_s, (float)cfg->pwm_max) != PID_OK) {
        return MOTOR_ERR_PARAM;
    }

    motor->inited = 1U;
    return MOTOR_OK;
}

motor_status_t motor_core_start(motor_core_t *motor)
{
    motor_status_t err; /* 安全清零输出结果 */

    if (motor == NULL) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }
    if (motor->enabled != 0U) {
        return MOTOR_OK;
    }

    err = apply_output(motor, 0);
    if (err != MOTOR_OK) {
        return err;
    }
    if (motor_pid_reset(&motor->pid) != PID_OK) {
        return MOTOR_ERR_STATE;
    }

    motor->enabled = 1U;
    return MOTOR_OK;
}

motor_status_t motor_core_stop(motor_core_t *motor)
{
    motor_status_t err; /* 输出清零结果 */

    if (motor == NULL) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    err = apply_output(motor, 0);
    motor->enabled    = 0U;
    motor->target_rps = 0.0f;
    (void)motor_pid_reset(&motor->pid);
    return err;
}

motor_status_t motor_core_set_rps(motor_core_t *motor, float rps)
{
    if ((motor == NULL) || (float_valid(rps) == 0U)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    motor->target_rps = rps;
    return MOTOR_OK;
}

motor_status_t motor_core_set_pid(motor_core_t *motor,
                                  const motor_pid_cfg_t *cfg)
{
    if ((motor == NULL) || (cfg == NULL)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }
    return (motor_pid_set(&motor->pid, cfg) == PID_OK)
         ? MOTOR_OK : MOTOR_ERR_PARAM;
}

motor_status_t motor_core_get_pid(const motor_core_t *motor,
                                  motor_pid_cfg_t *cfg)
{
    if ((motor == NULL) || (cfg == NULL)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }
    return (motor_pid_get(&motor->pid, cfg) == PID_OK)
         ? MOTOR_OK : MOTOR_ERR_STATE;
}

motor_status_t motor_core_set_filt(motor_core_t *motor, float filter)
{
    if ((motor == NULL) || (float_valid(filter) == 0U) ||
        (filter < 0.0f) || (filter > 1.0f)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    motor->cfg.filter = filter;
    motor->speed_rps = motor->raw_rps;
    motor->speed_ready = 1U;
    return MOTOR_OK;
}

motor_status_t motor_core_get_filt(const motor_core_t *motor,
                                   float *filter)
{
    if ((motor == NULL) || (filter == NULL)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    *filter = motor->cfg.filter;
    return MOTOR_OK;
}

motor_status_t motor_core_update(motor_core_t *motor, uint16_t ticks)
{
    float pid_out;  /* PID浮点输出 */
    int32_t duty;   /* 四舍五入后的带符号PWM */

    if (motor == NULL) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    encoder_update(motor, ticks);
    if (motor->enabled == 0U) {
        return MOTOR_OK;
    }

    if ((motor->target_rps < MOTOR_ZERO_RPS) &&
        (motor->target_rps > -MOTOR_ZERO_RPS)) {
        (void)motor_pid_reset(&motor->pid);
        return apply_output(motor, 0);
    }
    if (motor_pid_step(&motor->pid, motor->target_rps,
                       motor->speed_rps, &pid_out) != PID_OK) {
        return MOTOR_ERR_STATE;
    }

    duty = (pid_out >= 0.0f) ? (int32_t)(pid_out + 0.5f)
                              : (int32_t)(pid_out - 0.5f);
    return apply_output(motor, duty);
}

motor_status_t motor_core_get_state(const motor_core_t *motor,
                                    motor_core_state_t *state)
{
    if ((motor == NULL) || (state == NULL)) {
        return MOTOR_ERR_PARAM;
    }
    if (motor->inited == 0U) {
        return MOTOR_ERR_STATE;
    }

    state->target_rps = motor->target_rps;
    state->speed_rps  = motor->speed_rps;
    state->raw_rps    = motor->raw_rps;
    state->p_term     = motor->pid.p_term;
    state->i_term     = motor->pid.integral;
    state->ff_term    = motor->pid.ff_term;
    state->position   = motor->position;
    state->duty       = motor->duty;
    state->enabled    = motor->enabled;
    return MOTOR_OK;
}
