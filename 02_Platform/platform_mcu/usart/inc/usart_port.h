/**
 * @file    usart_port.h
 * @brief   MCU逻辑串口统一Port接口（接收会话）
 * @note    - 逻辑口到物理USART/DMA的映射只存在于同层usart_port.c
 *          - rx_start为空闲分段DMA接收：每收完一段(空闲/缓冲满)
 *            由本层HAL回调按注册表分发到各能力Adapter的ISR入口
 *          - start/stop线程态与ISR均可调用，内部无阻塞等待
 */

#ifndef USART_PORT_H
#define USART_PORT_H

#include "platform_error.h"

/** 逻辑串口实例，值=usart_port.c映射表下标。 */
typedef enum {
    EN_CORE_USART_IMU = 0, /* IMU数据口 */
    EN_CORE_USART_NUM      /* 实例总数 */
} en_core_usart_t;

/** 接收事件回调：一段接收完成(空闲/缓冲满)，ISR上下文。 */
typedef platform_err_t (*core_usart_rx_cb_t)(uint16_t size);

/** 接收错误回调：负责重启接收恢复，ISR上下文。 */
typedef platform_err_t (*core_usart_err_cb_t)(void);

/**
 * @brief  注册逻辑串口的接收事件/错误回调（Adapter ISR入口）
 * @param  id     逻辑串口实例
 * @param  rx_cb  接收事件回调，不可为NULL
 * @param  err_cb 错误回调，可为NULL(该口不做错误恢复)
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT(重复注册)
 * @note   仅限内核启动前单线程调用；先注册再rx_start
 */
platform_err_t core_usart_reg_isr(en_core_usart_t id,
                                  core_usart_rx_cb_t rx_cb,
                                  core_usart_err_cb_t err_cb);

/**
 * @brief  在给定缓冲上启动一段空闲分段DMA接收
 * @param  id  逻辑串口实例
 * @param  buf 接收缓冲，会话期间必须常驻有效
 * @param  len 缓冲字节数，>0
 * @retval PLATFORM_ERR_OK/PARAM/NOT_SUPPORTED(未配RX DMA)/
 *         BUSY/FAIL
 */
platform_err_t core_usart_rx_start(en_core_usart_t id,
                                   uint8_t *buf, uint16_t len);

/**
 * @brief  终止当前接收会话
 * @param  id 逻辑串口实例
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 * @note   含短暂DMA停止等待，仅初始化/错误恢复路径使用
 */
platform_err_t core_usart_rx_stop(en_core_usart_t id);

#endif /* USART_PORT_H */
