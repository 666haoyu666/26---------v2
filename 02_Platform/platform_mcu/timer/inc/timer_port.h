/**
 * @file    timer_port.h
 * @brief   MCU逻辑周期定时器统一Port接口（控制节拍）
 * @note    - 逻辑定时器到物理TIM的映射只存在于同层timer_port.c
 *          - 周期回调在定时器更新中断内直跑，必须短小且不阻塞
 *          - HAL_TIM_PeriodElapsedCallback由CubeMX main.c持有，
 *            main.c的USER CODE内按实例路由到core_timer_isr_entry
 */

#ifndef TIMER_PORT_H
#define TIMER_PORT_H

#include "platform_error.h"
#include "platform_type.h"

/** 逻辑周期定时器实例，值=timer_port.c映射表下标。 */
typedef enum {
    EN_CORE_TIMER_CTRL_10MS = 0, /* 10ms控制节拍：TIM9 */
    EN_CORE_TIMER_NUM            /* 实例总数 */
} en_core_timer_t;

/** 周期到点回调，ISR上下文。 */
typedef void (*core_timer_cb_t)(void);

/**
 * @brief  注册逻辑定时器的周期回调（Adapter ISR入口）
 * @param  id 逻辑定时器实例
 * @param  cb 周期回调，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT(重复注册)
 * @note   先注册再core_timer_start
 */
platform_err_t core_timer_reg_isr(en_core_timer_t id,
                                  core_timer_cb_t cb);

/**
 * @brief  启动逻辑定时器周期中断
 * @param  id 逻辑定时器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 * @note   已在运行时视为成功，幂等
 */
platform_err_t core_timer_start(en_core_timer_t id);

/**
 * @brief  停止逻辑定时器周期中断
 * @param  id 逻辑定时器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 */
platform_err_t core_timer_stop(en_core_timer_t id);

/**
 * @brief  短暂屏蔽逻辑定时器中断（NVIC级，含同步屏障）
 * @param  id 逻辑定时器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 * @note   线程态与节拍ISR互斥用；屏蔽期间到点请求保持挂起，
 *         解除后补发不丢拍；不可嵌套，须与unmask成对且极短
 */
platform_err_t core_timer_isr_mask(en_core_timer_t id);

/**
 * @brief  解除逻辑定时器中断屏蔽
 * @param  id 逻辑定时器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 */
platform_err_t core_timer_isr_unmask(en_core_timer_t id);

/**
 * @brief  周期到点分发入口，仅由main.c的HAL回调路由调用
 * @param  id 触发的逻辑定时器实例
 * @note   ISR上下文
 */
void core_timer_isr_entry(en_core_timer_t id);

#endif /* TIMER_PORT_H */
