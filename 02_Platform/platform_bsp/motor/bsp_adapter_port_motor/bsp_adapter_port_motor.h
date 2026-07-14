/**
 * @file    bsp_adapter_port_motor.h
 * @brief   电机能力Adapter注册入口，具体器件与板级绑定唯一落点
 * @note    - 由app_periph_init()在内核启动前调用，禁止阻塞
 *          - 公共头不暴露器件、HAL或RTOS类型
 */

#ifndef BSP_ADAPTER_PORT_MOTOR_H
#define BSP_ADAPTER_PORT_MOTOR_H

#include "bsp_wrapper_motor.h"

/**
 * @brief  装配电机设备表并注册到Wrapper槽位
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_NOTREG / MOTOR_DRV_ERR_FAIL
 * @note   bsp_motor的driver/handler落地前无可绑定器件，
 *         恒返回MOTOR_DRV_ERR_NOTREG明示能力缺席
 */
motor_drv_status_t drv_adapter_motor_register(void);

#endif /* BSP_ADAPTER_PORT_MOTOR_H */
