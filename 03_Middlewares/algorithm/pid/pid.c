/**
 * @file    pid.c
 * @brief   PID加线性前馈控制器实现
 * @note    系数采用秒制；在线更新参数时清除积分与微分历史。
 */

#include "pid.h"

#include <stddef.h>

/** @brief 判断浮点数是否为有限值 */
static uint8_t float_valid(float value)
{
    return ((value - value) == 0.0f) ? 1U : 0U;
}

/** @brief 校验控制参数范围 */
static uint8_t pid_cfg_valid(const motor_pid_cfg_t *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }
    if ((float_valid(cfg->kp) == 0U) ||
        (float_valid(cfg->ki) == 0U) ||
        (float_valid(cfg->kd) == 0U) ||
        (float_valid(cfg->kff) == 0U) ||
        (float_valid(cfg->ff_bias) == 0U) ||
        (float_valid(cfg->i_limit) == 0U)) {
        return 0U;
    }
    if ((cfg->kp < 0.0f) || (cfg->ki < 0.0f) ||
        (cfg->kd < 0.0f) || (cfg->kff < 0.0f) ||
        (cfg->ff_bias < 0.0f) || (cfg->i_limit < 0.0f)) {
        return 0U;
    }
    return 1U;
}

pid_status_t motor_pid_init(motor_pid_t *pid,
                            const motor_pid_cfg_t *cfg,
                            float sample_s,
                            float out_limit)
{
    if ((pid == NULL) || (pid_cfg_valid(cfg) == 0U) ||
        (float_valid(sample_s) == 0U) ||
        (float_valid(out_limit) == 0U) ||
        (sample_s <= 0.0f) || (out_limit <= 0.0f)) {
        return PID_ERR_PARAM;
    }

    pid->cfg       = *cfg;
    pid->sample_s  = sample_s;
    pid->out_limit = out_limit;
    pid->integral  = 0.0f;
    pid->p_term    = 0.0f;
    pid->ff_term   = 0.0f;
    pid->last_err  = 0.0f;
    pid->output    = 0.0f;
    pid->ready     = 0U;
    pid->inited    = 1U;
    return PID_OK;
}

pid_status_t motor_pid_set(motor_pid_t *pid,
                           const motor_pid_cfg_t *cfg)
{
    if ((pid == NULL) || (pid_cfg_valid(cfg) == 0U)) {
        return PID_ERR_PARAM;
    }
    if (pid->inited == 0U) {
        return PID_ERR_STATE;
    }

    pid->cfg = *cfg;
    return motor_pid_reset(pid);
}

pid_status_t motor_pid_get(const motor_pid_t *pid,
                           motor_pid_cfg_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL)) {
        return PID_ERR_PARAM;
    }
    if (pid->inited == 0U) {
        return PID_ERR_STATE;
    }

    *cfg = pid->cfg;
    return PID_OK;
}

pid_status_t motor_pid_reset(motor_pid_t *pid)
{
    if (pid == NULL) {
        return PID_ERR_PARAM;
    }
    if (pid->inited == 0U) {
        return PID_ERR_STATE;
    }

    pid->integral = 0.0f;
    pid->p_term   = 0.0f;
    pid->ff_term  = 0.0f;
    pid->last_err = 0.0f;
    pid->output   = 0.0f;
    pid->ready    = 0U;
    return PID_OK;
}

pid_status_t motor_pid_step(motor_pid_t *pid,
                            float target,
                            float feedback,
                            float *output)
{
    float error;      /* 本拍速度误差 */
    float derivative; /* 误差微分 */
    float feed_fwd;   /* 带方向前馈 */
    float result;     /* 限幅前总输出 */

    if ((pid == NULL) || (output == NULL) ||
        (float_valid(target) == 0U) ||
        (float_valid(feedback) == 0U)) {
        return PID_ERR_PARAM;
    }
    if (pid->inited == 0U) {
        return PID_ERR_STATE;
    }

    error = target - feedback;
    pid->p_term = pid->cfg.kp * error;
    pid->integral += pid->cfg.ki * error * pid->sample_s;
    if (pid->integral > pid->cfg.i_limit) {
        pid->integral = pid->cfg.i_limit;
    } else if (pid->integral < -pid->cfg.i_limit) {
        pid->integral = -pid->cfg.i_limit;
    }

    derivative = 0.0f;
    if (pid->ready != 0U) {
        derivative = (error - pid->last_err) / pid->sample_s;
    }
    feed_fwd = 0.0f;
    if (target > 0.0f) {
        feed_fwd = pid->cfg.kff * target + pid->cfg.ff_bias;
    } else if (target < 0.0f) {
        feed_fwd = pid->cfg.kff * target - pid->cfg.ff_bias;
    }
    pid->ff_term = feed_fwd;

    result = pid->ff_term + pid->p_term + pid->integral +
             pid->cfg.kd * derivative;
    if (result > pid->out_limit) {
        result = pid->out_limit;
    } else if (result < -pid->out_limit) {
        result = -pid->out_limit;
    }

    pid->last_err = error;
    pid->output   = result;
    pid->ready    = 1U;
    *output       = result;
    return PID_OK;
}
