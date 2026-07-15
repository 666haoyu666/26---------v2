/**
 * @file    pwm_port.c
 * @brief   MCU逻辑PWM Port实现（STM32F411 HAL）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - TIM1由MX_TIM1_Init配置：ARR=10000，约5kHz
 *          - start仅首次调用HAL（重复start幂等）；stop为软停：
 *            占空比清0输出恒低电平，通道保持运行。因两电机共用
 *            TIM1，HAL_TIM_PWM_Stop会无条件关断MOE殃及另一通道，
 *            故本层不做硬停
 *          - set_duty只写CCR寄存器，可在ISR中调用
 */

#include "pwm_port.h"

#include "platform_def.h"
#include "tim.h"

/** 逻辑PWM通道到物理定时器/通道的映射项。 */
typedef struct {
    TIM_HandleTypeDef *htim;    /* 物理定时器句柄 */
    uint32_t           channel; /* HAL通道标识 */
} pwm_map_t;

/** 逻辑PWM通道表，ID=下标。 */
static const pwm_map_t s_pwm_map[EN_CORE_PWM_NUM] = {
    { &htim1, TIM_CHANNEL_2 }, /* EN_CORE_PWM_MOTOR_A：PA9 */
    { &htim1, TIM_CHANNEL_1 }, /* EN_CORE_PWM_MOTOR_B：PA8 */
};

/** 1=该通道已启动；HAL通道重复Start会报错，以此保证幂等。 */
static uint8_t s_pwm_started[EN_CORE_PWM_NUM];

platform_err_t core_pwm_start(en_core_pwm_t id)
{
    if (id >= EN_CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_pwm_started[id] != 0U) {
        return PLATFORM_ERR_OK; /* 已启动，幂等 */
    }
    if (HAL_TIM_PWM_Start(s_pwm_map[id].htim,
                          s_pwm_map[id].channel) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_pwm_started[id] = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_stop(en_core_pwm_t id)
{
    if (id >= EN_CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    /* 软停：占空比清0输出恒低，通道保持运行不动MOE */
    __HAL_TIM_SET_COMPARE(s_pwm_map[id].htim,
                          s_pwm_map[id].channel, 0U);
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_set_duty(en_core_pwm_t id, uint16_t duty)
{
    if (id >= EN_CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if ((uint32_t)duty >
        __HAL_TIM_GET_AUTORELOAD(s_pwm_map[id].htim)) {
        return PLATFORM_ERR_PARAM;
    }

    __HAL_TIM_SET_COMPARE(s_pwm_map[id].htim,
                          s_pwm_map[id].channel, (uint32_t)duty);
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_get_max(en_core_pwm_t id, uint16_t *max)
{
    if ((id >= EN_CORE_PWM_NUM) || (max == NULL)) {
        return PLATFORM_ERR_PARAM;
    }

    *max = (uint16_t)__HAL_TIM_GET_AUTORELOAD(s_pwm_map[id].htim);
    return PLATFORM_ERR_OK;
}
