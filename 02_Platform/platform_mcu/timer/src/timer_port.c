/**
 * @file    timer_port.c
 * @brief   MCU逻辑周期定时器Port实现（STM32F411 HAL）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - TIM9基参数(10ms)由MX_TIM9_Init配置，本层不重复初始化
 *          - HAL_TIM_PeriodElapsedCallback弱符号唯一实现在本层：
 *            TIM11喂HAL时基（原CubeMX main.c逻辑，已删除main.c副本，
 *            CubeMX重新生成后需再删），其余实例按映射表分发
 *          - 本仓库HAL的Base_Start_IT在状态非READY时返回HAL_ERROR
 *            且运行期状态恒为BUSY，重复启动必报错，故以s_started
 *            标志保证幂等（两电机共用一个节拍即两次start）
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

/** 1=该定时器已启动；保证重复start幂等，不依赖HAL状态机。 */
static uint8_t s_started[EN_CORE_TIMER_NUM];

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
    if (id >= EN_CORE_TIMER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_started[id] != 0U) {
        return PLATFORM_ERR_OK; /* 已启动，幂等 */
    }
    if (HAL_TIM_Base_Start_IT(s_timer_map[id].htim) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_started[id] = 1U;
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

    s_started[id] = 0U;
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

/**
 * @brief  TIM周期到点回调（HAL弱符号唯一实现）
 * @param  htim 触发的TIM句柄
 * @note   ISR上下文：TIM11喂HAL时基，其余按映射表分发到
 *         各能力Adapter注册的节拍回调；仅路由，不阻塞
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t i; /* 映射表游标 */

    /* HAL时基：TIM11每1ms递增uwTick（原CubeMX main.c逻辑） */
    if (htim->Instance == TIM11) {
        HAL_IncTick();
        return;
    }

    for (i = 0U; i < (uint32_t)EN_CORE_TIMER_NUM; i++) {
        if (htim == s_timer_map[i].htim) {
            if (s_cb[i] != NULL) {
                s_cb[i](); /* ISR内不处理回调返回值 */
            }
            return;
        }
    }
}
