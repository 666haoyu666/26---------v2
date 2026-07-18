/**
 * @file    encoder_port.h
 * @brief   MCU编码器统一Port接口（A相上升沿1倍频解码）
 * @note    - 逻辑编码器到物理引脚/EXTI的映射只存在于同层encoder_port.c
 *          - A相上升沿EXTI采样B相电平定方向：每个上升沿经注册回调
 *            上报step=+1/-1，ISR上下文；高CPR电机（万级计数/转）
 *            用1倍频把中断量压到4倍频的四分之一
 *          - EXTI的NVIC优先级由本层设置，必须与控制节拍定时器相同：
 *            两条ISR互不抢占，计数累加与读清零才无竞争
 */

#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include "platform_error.h"
#include "platform_type.h"

/** 逻辑编码器实例，值=encoder_port.c映射表下标。 */
typedef enum {
    EN_CORE_ENCODER_1 = 0, /* 编码器1：A=PB7,B=PB6 */
    EN_CORE_ENCODER_2,     /* 编码器2：A=PB5,B=PB4 */
    EN_CORE_ENCODER_NUM    /* 实例总数 */
} en_core_encoder_t;

/** 计数步进回调：每个A相上升沿一次，ISR上下文，step=+1/-1。 */
typedef void (*core_encoder_step_cb_t)(int8_t step);

/**
 * @brief  注册逻辑编码器的步进回调（Adapter ISR入口）
 * @param  id 逻辑编码器实例
 * @param  cb 步进回调，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT(重复注册)
 * @note   先注册再core_encoder_start
 */
platform_err_t core_encoder_reg_isr(en_core_encoder_t id,
                                    core_encoder_step_cb_t cb);

/**
 * @brief  启动一路编码器：清除挂起边沿并使能EXTI中断
 * @param  id 逻辑编码器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 */
platform_err_t core_encoder_start(en_core_encoder_t id);

/**
 * @brief  停止一路编码器步进上报
 * @param  id 逻辑编码器实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 * @note   EXTI中断线与其他编码器共享，保持使能，仅软停上报
 */
platform_err_t core_encoder_stop(en_core_encoder_t id);

#endif /* ENCODER_PORT_H */
