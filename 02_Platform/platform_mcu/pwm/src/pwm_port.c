/**
 * @file    pwm_port.c
 * @brief   TIM1双路电机PWM逻辑映射
 * @note    停止采用CCR清零，避免关闭共享TIM1的MOE影响另一通道。
 */

#include "pwm_port.h"

#include <stddef.h>

#include "tim.h"

typedef struct {
    TIM_HandleTypeDef  *htim;   /* 物理定时器 */
    uint32_t           channel; /* HAL通道 */
} pwm_map_t;

static const pwm_map_t s_pwm_map[CORE_PWM_NUM] = {
    { &htim1, TIM_CHANNEL_2 },
    { &htim1, TIM_CHANNEL_1 }
};

static uint8_t s_pwm_started[CORE_PWM_NUM];

platform_err_t core_pwm_start(core_pwm_t id)
{
    if (id >= CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_pwm_started[id] != 0U) {
        return PLATFORM_ERR_OK;
    }
    if (HAL_TIM_PWM_Start(s_pwm_map[id].htim,
                          s_pwm_map[id].channel) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_pwm_started[id] = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_stop(core_pwm_t id)
{
    if (id >= CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    __HAL_TIM_SET_COMPARE(s_pwm_map[id].htim,
                          s_pwm_map[id].channel, 0U);
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_set(core_pwm_t id, uint16_t duty)
{
    if (id >= CORE_PWM_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if ((uint32_t)duty >
        __HAL_TIM_GET_AUTORELOAD(s_pwm_map[id].htim)) {
        return PLATFORM_ERR_PARAM;
    }

    __HAL_TIM_SET_COMPARE(s_pwm_map[id].htim,
                          s_pwm_map[id].channel, duty);
    return PLATFORM_ERR_OK;
}

platform_err_t core_pwm_get_max(core_pwm_t id, uint16_t *max_duty)
{
    if ((id >= CORE_PWM_NUM) || (max_duty == NULL)) {
        return PLATFORM_ERR_PARAM;
    }

    *max_duty = (uint16_t)
                __HAL_TIM_GET_AUTORELOAD(s_pwm_map[id].htim);
    return PLATFORM_ERR_OK;
}
