/**
 * @file    myprintf.c
 * @brief   格式化文本并通过USART6阻塞发送
 * @note    直接依赖Cube USART/HAL是临时技术债，发送超时为1秒
 */

#include "myprintf.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "usart.h"

#define MYPRINTF_TX_MS    1000U /* USART6阻塞发送超时，单位ms */

/**
 * @brief  将HAL串口状态映射到项目公共错误码
 * @param  hal_err HAL返回状态
 * @retval 对应platform_err_t，未知状态归入FAIL
 */
static platform_err_t map_hal_err(HAL_StatusTypeDef hal_err)
{
    switch (hal_err) {
        case HAL_OK:
            return PLATFORM_ERR_OK;

        case HAL_BUSY:
            return PLATFORM_ERR_BUSY;

        case HAL_TIMEOUT:
            return PLATFORM_ERR_TIMEOUT;

        case HAL_ERROR:
        default:
            return PLATFORM_ERR_FAIL;
    }
}

/**
 * @brief  格式化文本并通过USART6阻塞发送
 * @param  buf      调用方提供的字符缓冲区
 * @param  buf_size 缓冲区总容量，包含字符串结尾字符
 * @param  fmt      printf风格格式字符串
 * @retval PLATFORM_ERR_OK / PARAM / NO_MEMORY / BUSY / TIMEOUT / FAIL
 */
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...)
{
    va_list args;              /* 可变参数游标 */
    int fmt_len;               /* 不含结尾字符的格式化长度 */
    HAL_StatusTypeDef hal_err; /* USART6发送结果 */

    if (buf == NULL || buf_size == 0U || fmt == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    va_start(args, fmt);
    fmt_len = vsnprintf(buf, buf_size, fmt, args);
    va_end(args);

    if (fmt_len < 0) {
        buf[0] = '\0';
        return PLATFORM_ERR_FAIL;
    }

    /* 截断文本可能掩盖关键信息，因此缓冲区不足时不发送。 */
    if ((size_t)fmt_len >= buf_size) {
        return PLATFORM_ERR_NO_MEMORY;
    }

    if ((size_t)fmt_len > (size_t)UINT16_MAX) {
        return PLATFORM_ERR_NO_RESOURCE;
    }

    if (fmt_len == 0) {
        return PLATFORM_ERR_OK;
    }

    hal_err = HAL_UART_Transmit(&huart6,
                                (uint8_t *)buf,
                                (uint16_t)fmt_len,
                                MYPRINTF_TX_MS);
    return map_hal_err(hal_err);
}
