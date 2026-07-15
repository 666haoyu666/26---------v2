/**
 * @file    pwm_port.h
 * @brief   MCU逻辑PWM统一Port接口
 * @note    - 逻辑通道到物理定时器/通道的映射只存在于同层pwm_port.c
 *          - set_duty仅写比较寄存器，线程态与ISR均可调用，不阻塞
 *          - 定时器基参数(频率/ARR)由MX_TIMx_Init配置，本层不改
 */

#ifndef PWM_PORT_H
#define PWM_PORT_H

#include "platform_error.h"
#include "platform_type.h"

/** 逻辑PWM通道，值=pwm_port.c映射表下标。 */
typedef enum {
    EN_CORE_PWM_MOTOR_A = 0, /* A电机PWM：TIM1_CH2(PA9) */
    EN_CORE_PWM_MOTOR_B,     /* B电机PWM：TIM1_CH1(PA8) */
    EN_CORE_PWM_NUM          /* 通道总数 */
} en_core_pwm_t;

/**
 * @brief  启动一路PWM输出，占空比保持当前比较值
 * @param  id 逻辑PWM通道
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 * @note   建议先core_pwm_set_duty(id,0)再启动，保证上电静止
 */
platform_err_t core_pwm_start(en_core_pwm_t id);

/**
 * @brief  停止一路PWM输出，引脚回到非有效电平
 * @param  id 逻辑PWM通道
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 */
platform_err_t core_pwm_stop(en_core_pwm_t id);

/**
 * @brief  设置一路PWM占空比原始值
 * @param  id   逻辑PWM通道
 * @param  duty 比较值，0到core_pwm_get_max()，越界报PARAM
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 * @note   ISR上下文可调用
 */
platform_err_t core_pwm_set_duty(en_core_pwm_t id, uint16_t duty);

/**
 * @brief  读取一路PWM占空比满幅值(=定时器ARR)
 * @param  id  逻辑PWM通道
 * @param  max 输出满幅值，不可为NULL；失败时不修改
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 */
platform_err_t core_pwm_get_max(en_core_pwm_t id, uint16_t *max);

#endif /* PWM_PORT_H */
