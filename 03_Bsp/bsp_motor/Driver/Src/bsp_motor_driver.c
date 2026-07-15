/**
 * @file    bsp_motor_driver.c
 * @brief   编码器电机对象实现
 * @author  bojing
 */

#include "bsp_motor_driver.h"

static motor_dir_t motor_pick_dir(motor_dir_t base_dir, uint8_t negative)
{
    if (negative == 0U)
    {
        /* 正输出方向(默认方向) */
        return base_dir;
    }

    /* 负输出方向(默认方向反向) */
    return (base_dir == MOTOR_DIR_CW)
         ? MOTOR_DIR_CCW
         : MOTOR_DIR_CW;
}

/**
 * @brief  实例化单电机对象
 *
 * @param  motor         电机对象指针
 * @param  pf_set_enable 底层电机使能接口
 * @param  pf_set_output 底层PWM输出接口
 * @param  dir           电机默认旋转方向
 * @param  pwm_max       最大PWM值
 * @retval MOTOR_OK             实例化成功
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_inst(
    motor_driver_t *const motor,    
    bsp_motor_set_enable_t pf_set_enable,
    bsp_motor_set_output_t pf_set_output,
    motor_dir_t dir,
    uint16_t pwm_max)
{
    motor_status_t ret = MOTOR_OK;

    if (motor == NULL || pf_set_enable == NULL || pf_set_output == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");
#endif // DEBUG        
        return MOTOR_ERRORPARAMETER;
    }

    if (dir != MOTOR_DIR_CW && dir != MOTOR_DIR_CCW)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");
#endif // DEBUG
        return MOTOR_ERRORPARAMETER;
    }

    motor->is_enabled = 0;

    motor->output = 0;
    motor->dir = dir;
    motor->pwm_max= pwm_max;

    motor->pf_set_enable = pf_set_enable;
    motor->pf_set_output = pf_set_output;

    motor->is_inited = 1;

    return ret;
}

/**
 * @brief  使能单电机对象
 * @param  motor 电机对象
 * @param  enable 1：使能，0：失能
 * @retval MOTOR_OK / MOTOR_ERROR / MOTOR_ERRORPARAMETER
 */
motor_status_t motor_driver_enable(
    motor_driver_t *const motor, 
    uint8_t enable)
{
    motor_status_t ret = MOTOR_OK;

    if (motor == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");
#endif // DEBUG
        return MOTOR_ERRORPARAMETER;
    }

    if (!motor->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    if (enable != 0U)
    {
        /* 每次重新启动时清除旧积分 */
        (void)motor_pid_reset(&motor->pid);
    }    

    ret=motor->pf_set_enable(enable);
    if(ret != MOTOR_OK)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return ret;
    }

    motor->is_enabled = enable;    

    return ret;
}


/**
 * @brief  设置电机输出方向和PWM
 *
 * @param  motor         电机对象指针
 * @param  output        0>正转，<0反转
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_set_output(
    motor_driver_t *const motor,
    int16_t output)
{
    motor_status_t ret = MOTOR_OK;

    motor_dir_t actual_dir;    /* 实际输出方向 */
    uint16_t pwm;              /* 实际PWM输出值 */
    int32_t limited_output;    /* 实际output输出 */ 
    if (motor == NULL )
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");
#endif // DEBUG
        return MOTOR_ERRORPARAMETER;
    }

    if (!motor->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    if (!motor->is_enabled)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    limited_output=(int32_t)output;

    /* 输出限幅 */
    if (limited_output > (int32_t)motor->pwm_max)
    {
        limited_output = (int32_t)motor->pwm_max;
    }
    else if (limited_output < -(int32_t)motor->pwm_max)
    {
        limited_output = -(int32_t)motor->pwm_max;
    }

    actual_dir = motor_pick_dir(
        motor->dir, 
        (limited_output < 0) ? 1U : 0U);

    if (limited_output<0)
    {
        pwm=(uint16_t)(-limited_output);
    }
    else{
        pwm=(uint16_t)limited_output;
    }
        

    ret=motor->pf_set_output(actual_dir, pwm);
    if(ret != MOTOR_OK)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return ret;
    }

    motor->output = (int16_t)limited_output;

    return ret;
}


/**
 * @brief 设置电机目标速度
 *
 * @param motor             电机对象
 * @param target_speed_rps  目标输出轴速度，单位：转/秒
 *
 * target_speed_rps > 0：逻辑正向
 * target_speed_rps < 0：逻辑反向
 * target_speed_rps = 0：目标停止
 */
motor_status_t motor_driver_set_target_speed(
    motor_driver_t *const motor,
    float target_speed_rps)
{
    motor_status_t ret = MOTOR_OK;

    if (motor == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");  
#endif // DEBUG
        return MOTOR_ERRORPARAMETER;
    }

    if (!motor->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    /*
     * 以下情况复位PID（启用时需先读取旧目标old_target）：
     * 1. 新目标为停止；
     * 2. 正转切换为反转；
     * 3. 反转切换为正转。
     */
//    if (((target_speed_rps > -0.001f) &&
//         (target_speed_rps < 0.001f)) ||

//        ((old_target > 0.001f) &&
//         (target_speed_rps < -0.001f)) ||

//        ((old_target < -0.001f) &&
//         (target_speed_rps > 0.001f)))
//    {
//        (void)motor_pid_reset(&motor->pid);
//    }

    motor->target_speed_rps = target_speed_rps;

    return ret;
}

/**
 * @brief  周期更新编码器速度和位置
 * 
 * @param  motor 电机对象指针
 * 
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_encoder_update(
    motor_driver_t *const motor)
{
    motor_status_t ret = MOTOR_OK;
	
    if (motor == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");  
#endif // DEBUG        
        return MOTOR_ERRORPARAMETER;
    }

    if (!motor->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG        
        return MOTOR_ERROR;
    }

    if (!motor->is_enabled)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG        
        return MOTOR_ERROR;
    }

    encoder_status_t rets =encoder_update(&motor->encoder);
    if(rets != ENCODER_OK)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    return ret;
}

/**
 * @brief  PID计算，更新电机输出PWM
 * 
 * @param  motor 电机对象指针
 * 
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */   
motor_status_t motor_driver_pid_update(
    motor_driver_t *const motor)
{
    float pid_output;
    int16_t motor_output;

    const float target_zero_epsilon = 0.001f;
    const float reverse_speed_threshold = 0.10f;

    if (motor == NULL)
    {
        return MOTOR_ERRORPARAMETER;
    }

    if ((motor->is_inited == 0U) ||
        (motor->is_enabled == 0U))
    {
        return MOTOR_ERROR;
    }

    /*
     * 目标速度为0：
     * 停止电机并清除PID状态。
     */
    if ((motor->target_speed_rps < target_zero_epsilon) &&
        (motor->target_speed_rps > -target_zero_epsilon))
    {
        (void)motor_pid_reset(&motor->pid);

        return motor_driver_set_output(
            motor,
            0);
    }

    /*
     * 安全换向：
     *
     * 目标要求反转，但电机仍明显正转；
     * 或目标要求正转，但电机仍明显反转。
     *
     * 此时先断开PWM，让电机减速到接近0。
     */
    if (((motor->target_speed_rps < 0.0f) &&
         (motor->encoder.speed_rps >
          reverse_speed_threshold)) ||

        ((motor->target_speed_rps > 0.0f) &&
         (motor->encoder.speed_rps <
          -reverse_speed_threshold)))
    {
        (void)motor_pid_reset(&motor->pid);

        return motor_driver_set_output(
            motor,
            0);
    }

    /* 计算PID */
    pid_output = motor_pid_calculate(
        &motor->pid,
        motor->target_speed_rps,
        motor->encoder.speed_rps);

    /*
     * 目标正转时，只允许正输出；
     * 目标反转时，只允许负输出。
     */
//    if ((motor->target_speed_rps > 0.0f) &&
//        (pid_output < 0.0f))
//    {
//        pid_output = 0.0f;
//    }
//    else if ((motor->target_speed_rps < 0.0f) &&
//             (pid_output > 0.0f))
//    {
//        pid_output = 0.0f;
//    }

    /* PWM限幅 */
    if (pid_output > (float)motor->pwm_max)
    {
        pid_output = (float)motor->pwm_max;
    }
    else if (pid_output <
             -(float)motor->pwm_max)
    {
        pid_output =
            -(float)motor->pwm_max;
    }

    motor_output = (int16_t)pid_output;

    return motor_driver_set_output(
        motor,
        motor_output);
}

/**
 * @brief  设置电机停止
 * @param  motor 电机对象
 * @retval MOTOR_OK / MOTOR_ERROR / MOTOR_ERRORPARAMETER
 */
motor_status_t motor_driver_stop(
    motor_driver_t *motor)
{
    motor_status_t ret = MOTOR_OK;

    if (motor == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERRORPARAMETER\r\n");
#endif // DEBUG
        return MOTOR_ERRORPARAMETER;
    }

    if (!motor->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }

    if (!motor->is_enabled)
    {
#ifdef DEBUG
        DEBUG_OUT("MOTOR_ERROR\r\n");
#endif // DEBUG
        return MOTOR_ERROR;
    }    

    /* 先将PWM输出设置为0 */
    ret = motor->pf_set_output(motor->dir, 0U);
    if (ret != MOTOR_OK)
    {
        return ret;
    }

    /* 再关闭驱动 */
    ret = motor->pf_set_enable(0U);
    if (ret != MOTOR_OK)
    {
        return ret;
    }

    motor->is_enabled = 0;
    motor->output = 0;

    motor->target_speed_rps = 0.0f;

    /* 清除位置式PID的误差、积分和输出 */
    (void)motor_pid_reset(&motor->pid);

    return ret;
}
