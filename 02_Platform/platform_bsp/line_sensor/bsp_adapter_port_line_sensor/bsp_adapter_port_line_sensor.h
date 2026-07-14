/**
 * @file    bsp_adapter_port_line_sensor.h
 * @brief   循迹传感器Adapter静态装配与注册入口
 * @note    - 具体Driver、板级几何/极性和MCU输入组仅在Adapter实现中绑定
 *          - 由组合根在Wrapper首次调用前注册，不创建OS资源
 *          - 当前能力采用线程态轮询，无Handler和ISR桥接入口
 */

#ifndef BSP_ADP_LSENSOR_H
#define BSP_ADP_LSENSOR_H

#include "bsp_wrapper_line_sensor.h"

/**
 * @brief  装配八路灰度Driver并注册到循迹传感器Wrapper
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT
 * @note   本函数只做静态装配；硬件能力由bsp_lsensor_init()启动
 */
platform_err_t bsp_lsensor_adp_reg(void);

#endif /* BSP_ADP_LSENSOR_H */
