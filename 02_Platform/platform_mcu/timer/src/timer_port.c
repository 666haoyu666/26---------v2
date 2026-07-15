/**
 * @file    timer_port.c
 * @brief   MCU逻辑周期定时器Port实现（STM32F411 HAL）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - TIM9基参数(10ms)由MX_TIM9_Init配置，本层不重复初始化
 *          - mask/unmask走NVIC：TIM1_BRK与TIM9共线，屏蔽期间
 *            TIM1刹车中断同样被延迟（本工程未用TIM1刹车）
 */

#include "timer_port.h"

#include "platform_def.h"
#include "tim.h"

/** 逻辑定时器到物理TIM句柄/中断线的映射项。 */
typedef struct {
    TIM_HandleTypeDef *htim; /* 物理定时器句柄 */
    IRQn_Type          irqn; /* 所在NVIC中断线 */
} timer_map_t;

/** 逻辑定时器表，ID=下标。 */
static const timer_map_t s_timer_map[EN_CORE_TIMER_NUM] = {
    { &htim9, TIM1_BRK_TIM9_IRQn }, /* EN_CORE_TIMER_CTRL_10MS */
};

/** 各逻辑定时器注册的周期回调（Adapter ISR入口）。 */
static core_timer_cb_t s_cb[EN_CORE_TIMER_NUM];

platform_err_t core_timer_reg_isr(en_core_timer_t id,
                                  core_timer_cb_t cb)
{
    if ((id >= EN_CORE_TIMER_NUM) || (cb == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    /* 拒绝重复注册，防止运行期暗中换表 */
    if (s_cb[id] != NULL) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_cb[id] = cb;
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_start(en_core_timer_t id)
{
    HAL_StatusTypeDef hal; /* HAL调用结果 */

    if (id >= EN_CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    hal = HAL_TIM_Base_Start_IT(s_timer_map[id].htim);
    /* 已在运行返回BUSY，视为成功保证幂等 */
    if ((hal != HAL_OK) && (hal != HAL_BUSY)) {
        return PLATFORM_ERR_FAIL;
    }
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_stop(en_core_timer_t id)
{
    if (id >= EN_CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (HAL_TIM_Base_Stop_IT(s_timer_map[id].htim) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_isr_mask(en_core_timer_t id)
{
    if (id >= EN_CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    HAL_NVIC_DisableIRQ(s_timer_map[id].irqn);
    /* 屏障确保屏蔽生效且无在途ISR后才返回 */
    __DSB();
    __ISB();
    return PLATFORM_ERR_OK;
}

platform_err_t core_timer_isr_unmask(en_core_timer_t id)
{
    if (id >= EN_CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    HAL_NVIC_EnableIRQ(s_timer_map[id].irqn);
    return PLATFORM_ERR_OK;
}

void core_timer_isr_entry(en_core_timer_t id)
{
    if (id >= EN_CORE_TIMER_NUM) {
        return;
    }
    if (s_cb[id] != NULL) {
        s_cb[id](); /* ISR内不处理回调返回值 */
    }
}
