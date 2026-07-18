/**
 * @file    pid.c
 * @brief   PID控制器
 * @author  bojing
 */

#include "pid.h"

/**
 * @brief 实例化PID控制器对象
 * 
 * @param pid PID控制器对象
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 输出上限
 * @param output_min 输出下限
 * 
 * @return PID_OK
 * @return PID_ERROR
 * @return PID_ERRORPARAMETER
 */
pid_status_t motor_pid_inst(
    motor_pid_t *const pid,
    float kp,
    float ki,
    float kd,
    float output_max,
    float output_min,
    float integral_limit)    
{
    pid_status_t ret = PID_OK;

    if(pid == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("PID_ERRORPARAMETER\r\n");
#endif // DEBUG         
        return PID_ERRORPARAMETER;
    }

    if(kp < 0.0f || ki < 0.0f || kd < 0.0f || integral_limit < 0.0f)
    {
#ifdef DEBUG
        DEBUG_OUT("PID_ERRORPARAMETER\r\n");    
#endif // DEBUG
        return PID_ERRORPARAMETER;
    }

    if(output_max <= output_min)
    {
#ifdef DEBUG
        DEBUG_OUT("PID_ERRORPARAMETER\r\n");
#endif // DEBUG
        return PID_ERRORPARAMETER;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_limit = integral_limit;

    pid->target_speed = 0.0f;
    pid->feedback_speed = 0.0f;

    pid->dead_band = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->is_inited = 1;

    return ret;
}

/**
 * @brief PID计算
 * 
 * @param pid PID控制器对象
 * @param target_speed 目标速度
 * @param feedback_speed 反馈速度
 * 
 * @return float PID输出值
 */    
float motor_pid_calculate(
    motor_pid_t *const pid,
    float target_speed,
    float feedback_speed)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if (pid->is_inited == 0U)
    {
        return 0.0f;
    }

    /* 保存目标速度和反馈速度 */
    pid->target_speed = target_speed;
    pid->feedback_speed = feedback_speed;

    /* 1. 计算当前误差 */
    pid->error =
        pid->target_speed - pid->feedback_speed;

    /* 2. 死区处理 */
    if ((pid->error < pid->dead_band) &&
        (pid->error > -pid->dead_band))
    {
        pid->error = 0.0f;
    }

    /* 3. 误差积分 */
    pid->integral += (pid->ki)* (pid->error);

    /* 4. 积分限幅，防止积分饱和 */
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    /*
     * 5. 位置式PID计算
     *
     * output =
     *     Kr × 目标速度
     *   + Kp × 当前误差
     *   + Ki × 误差累计
     *   + Kd × 误差变化量
     */
    if(pid->target_speed>0){
        pid->output =
            (600.0f * pid->target_speed + 300.0f)
            + pid->kp * pid->error
            + pid->integral
            + pid->kd * (pid->error - pid->last_error);
    }
    else if(pid->target_speed<0){
        pid->output =
            (677.23f * target_speed - 314.03f)
            + pid->kp * pid->error
            + pid->integral
            + pid->kd * (pid->error - pid->last_error);
    }

    /* 6. 输出限幅 */
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }

    /* 7. 保存当前误差，下一次作为上次误差 */
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 清零PID运行状态
 *
 * @param pid PID控制器对象
 *
 * @retval PID_OK
 * @retval PID_ERRORPARAMETER
 */
pid_status_t motor_pid_reset(
    motor_pid_t *const pid)
{
    pid_status_t ret = PID_OK;

    if (pid == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("PID_ERRORPARAMETER\r\n");
#endif
        return PID_ERRORPARAMETER;
    }

    if (pid->is_inited == 0U)
    {
#ifdef DEBUG
        DEBUG_OUT("PID_ERROR\r\n");
#endif
        return PID_ERROR;
    }

    pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->target_speed = 0.0f;
    pid->feedback_speed = 0.0f;

    return ret;
}
