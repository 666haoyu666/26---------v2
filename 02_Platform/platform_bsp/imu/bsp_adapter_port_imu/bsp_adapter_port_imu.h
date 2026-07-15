/**
 * @file    bsp_adapter_port_imu.h
 * @brief   IMU Adapter静态装配与注册入口
 * @note    - 具体器件(维特WT系列)、串口绑定与缓冲策略仅在实现中出现
 *          - reg由组合根在Wrapper首次调用前调用，不创建OS资源
 *          - ISR入口为实现内静态函数，reg时注册进usart Port分发表
 */

#ifndef BSP_ADP_IMU_H
#define BSP_ADP_IMU_H

#include "bsp_wrapper_imu.h"

/**
 * @brief  装配维特WT系列IMU Driver并注册到IMU Wrapper
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT
 * @note   同时注册ISR回调到usart Port；DMA接收由bsp_imu_init()启动
 */
platform_err_t bsp_imu_adp_reg(void);

#endif /* BSP_ADP_IMU_H */
