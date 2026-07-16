/**
 * @file    myprintf.h
 * @brief   提供调用方缓冲区格式化和USART6 DMA发送接口
 * @note    - 线程态格式化后复制到静态队列，调用方缓冲区可立即复用
 *          - DMA完成与错误入口由HAL回调驱动，禁止主动从ISR格式化
 */

#ifndef MYPRINTF_H
#define MYPRINTF_H

#include <stddef.h>
#include <stdint.h>

#include "platform_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  格式化文本并排入USART6 DMA发送队列
 * @param  buf      调用方提供的字符缓冲区
 * @param  buf_size 缓冲区总容量，包含字符串结尾字符
 * @param  fmt      printf风格格式字符串
 * @retval PLATFORM_ERR_OK / PARAM / NO_MEMORY / BUSY / FAIL
 * @note   OK表示消息已入队；队列满返回BUSY且累计丢帧计数
 * @note   缓冲区不足时不发送截断内容；禁止在ISR中调用
 */
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...);

/**
 * @brief  读取DMA队列累计丢帧数
 * @retval 启动失败、发送错误及队列满导致的累计丢帧数
 */
uint32_t myprintf_get_drop(void);

/**
 * @brief  USART6发送错误回调入口
 * @note   仅由HAL_UART_ErrorCallback在ISR上下文调用
 */
void myprintf_dma_err_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* MYPRINTF_H */
