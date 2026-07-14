/**
 * @file    myprintf.h
 * @brief   提供调用方缓冲区格式化和调试串口发送接口
 * @note    当前实现使用USART6阻塞发送，禁止在ISR中调用
 */

#ifndef MYPRINTF_H
#define MYPRINTF_H

#include <stddef.h>

#include "platform_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  格式化文本并通过USART6阻塞发送
 * @param  buf      调用方提供的字符缓冲区
 * @param  buf_size 缓冲区总容量，包含字符串结尾字符
 * @param  fmt      printf风格格式字符串
 * @retval PLATFORM_ERR_OK / PARAM / NO_MEMORY / BUSY / TIMEOUT / FAIL
 * @note   缓冲区不足时不发送截断内容；禁止在ISR中调用
 * @note   发送超时为1000ms；当前不提供多任务串口仲裁
 */
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...);

#ifdef __cplusplus
}
#endif

#endif /* MYPRINTF_H */
