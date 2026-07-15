/**
 * @file    bsp_adapter_port_motor.h
 * @brief   电机能力Adapter注册入口，具体器件与板级绑定唯一落点
 * @note    - 由app_periph_init()调用：先register挂ISR入口与设备表，
 *            再经Wrapper群体drv_adapter_motor_init()初始化硬件
 *          - 公共头不暴露器件、HAL或RTOS类型
 */

#ifndef BSP_ADAPTER_PORT_MOTOR_H
#define BSP_ADAPTER_PORT_MOTOR_H

#include "bsp_wrapper_motor.h"

/**
 * @brief  装配A/B两轮电机设备表并注册到Wrapper槽位
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_STATE(重复注册) / MOTOR_DRV_ERR_FAIL
 * @note   仅注册节拍/编码器ISR入口与设备表，不触碰电机硬件；
 *         硬件初始化由drv_adapter_motor_init()逐槽位执行，
 *         初始化完成后电机处OFF态，start后才出力
 */
motor_drv_status_t drv_adapter_motor_register(void);

#endif /* BSP_ADAPTER_PORT_MOTOR_H */
