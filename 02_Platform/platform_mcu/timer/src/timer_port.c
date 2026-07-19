/**
 * @file    timer_port.c
 * @brief   TIM9控制节拍映射与HAL周期回调分发
 * @note    本文件唯一实现HAL_TIM_PeriodElapsedCallback并兼顾TIM11时基。
 */

#include "timer_port.h"

#include <stddef.h>
#include <stdint.h>

#include "tim.h"

typedef struct {
    TIM_HandleTypeDef  *htim; /* 物理定时器 */
    IRQn_Type          irqn;  /* 共享NVIC中断线 */
} timer_map_t;

static const timer_map_t s_timer_map[CORE_TIMER_NUM] = {
    { &htim9, TIM1_BRK_TIM9_IRQn }
};

static core_timer_cb_t s_timer_cb[CORE_TIMER_NUM];
static uint8_t s_timer_started[CORE_TIMER_NUM];

platform_err_t core_timer_reg(core_timer_t id, core_timer_cb_t cb)
{
    if ((id >= CORE_TIMER_NUM) || (cb == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_timer_cb[id] != NULL) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_timer_cb[id] = cb;
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_start(core_timer_t id)
{
    if (id >= CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_timer_started[id] != 0U) {
        return PLATFORM_ERR_OK;
    }
    if (HAL_TIM_Base_Start_IT(s_timer_map[id].htim) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_timer_started[id] = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_stop(core_timer_t id)
{
    if (id >= CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_timer_started[id] == 0U) {
        return PLATFORM_ERR_OK;
    }
    if (HAL_TIM_Base_Stop_IT(s_timer_map[id].htim) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_timer_started[id] = 0U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_mask(core_timer_t id)
{
    if (id >= CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    HAL_NVIC_DisableIRQ(s_timer_map[id].irqn);
    __DSB();
    __ISB();
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_unmask(core_timer_t id)
{
    if (id >= CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    HAL_NVIC_EnableIRQ(s_timer_map[id].irqn);
    return PLATFORM_ERR_OK;
}

/**
 * @brief  HAL定时器周期回调统一分发
 * @param  htim 触发回调的定时器
 * @note   ISR上下文，仅执行已注册短回调。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t i; /* 逻辑定时器游标 */

    if (htim->Instance == TIM11) {
        HAL_IncTick();
        return;
    }

    for (i = 0U; i < (uint32_t)CORE_TIMER_NUM; i++) {
        if (htim == s_timer_map[i].htim) {
            if (s_timer_cb[i] != NULL) {
                s_timer_cb[i]();
            }
            return;
        }
    }
}
