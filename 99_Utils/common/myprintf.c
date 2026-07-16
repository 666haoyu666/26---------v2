/**
 * @file    myprintf.c
 * @brief   格式化文本并通过静态队列异步发送到USART6
 * @note    - 调用方数据先复制入队，DMA不持有调用方缓冲区
 *          - 直接依赖Cube USART/HAL是临时技术债
 */

#include "myprintf.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#define MYPRINTF_MSG_MAX  256U /* 单条DMA消息最大字节数 */
#define MYPRINTF_QUEUE_NUM 8U  /* DMA发送队列固定槽位数 */

/** 单条DMA消息，发送期间所在槽位保持只读。 */
typedef struct {
    uint16_t len;                    /* 本条有效字节数 */
    uint8_t  data[MYPRINTF_MSG_MAX]; /* DMA持有的数据副本 */
} myprintf_msg_t;

static myprintf_msg_t s_tx_queue[MYPRINTF_QUEUE_NUM]; /* 静态发送队列 */
static volatile uint8_t s_tx_head;  /* 下一入队槽位 */
static volatile uint8_t s_tx_tail;  /* 当前发送/待发送槽位 */
static volatile uint8_t s_tx_count; /* 队列有效消息数 */
static volatile uint8_t s_tx_busy;  /* 1=USART6 DMA正在发送 */
static volatile uint32_t s_tx_drop; /* 累计丢帧数 */

static uint32_t irq_lock(void);
static void irq_unlock(uint32_t primask);
static HAL_StatusTypeDef dma_start_locked(void);
static void queue_pop_locked(void);

/** @brief 保存中断状态并进入短临界区。 */
static uint32_t irq_lock(void)
{
    uint32_t primask; /* 进入临界区前的PRIMASK */

    primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/** @brief 恢复进入临界区前的中断状态。 */
static void irq_unlock(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

/** @brief 移除当前队首消息，调用方保证队列非空。 */
static void queue_pop_locked(void)
{
    s_tx_tail++;
    if (s_tx_tail >= MYPRINTF_QUEUE_NUM) {
        s_tx_tail = 0U;
    }
    s_tx_count--;
}

/**
 * @brief  临界区内启动队首DMA
 * @retval HAL_OK / HAL_BUSY / HAL_ERROR
 */
static HAL_StatusTypeDef dma_start_locked(void)
{
    HAL_StatusTypeDef hal_err; /* DMA启动结果 */

    if (s_tx_busy != 0U || s_tx_count == 0U) {
        return HAL_OK;
    }

    hal_err = HAL_UART_Transmit_DMA(&huart6,
                                    s_tx_queue[s_tx_tail].data,
                                    s_tx_queue[s_tx_tail].len);
    if (hal_err == HAL_OK) {
        s_tx_busy = 1U;
    }
    return hal_err;
}

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
 * @brief  格式化文本并排入USART6 DMA发送队列
 * @param  buf      调用方提供的字符缓冲区
 * @param  buf_size 缓冲区总容量，包含字符串结尾字符
 * @param  fmt      printf风格格式字符串
 * @retval PLATFORM_ERR_OK / PARAM / NO_MEMORY / BUSY / FAIL
 */
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...)
{
    va_list args;              /* 可变参数游标 */
    int fmt_len;               /* 不含结尾字符的格式化长度 */
    HAL_StatusTypeDef hal_err; /* DMA启动结果 */
    myprintf_msg_t *msg;       /* 本次占用的队列槽位 */
    uint32_t primask;          /* 入队临界区中断状态 */

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

    if ((size_t)fmt_len > MYPRINTF_MSG_MAX) {
        return PLATFORM_ERR_NO_MEMORY;
    }

    if (fmt_len == 0) {
        return PLATFORM_ERR_OK;
    }

    primask = irq_lock();
    if (s_tx_count >= MYPRINTF_QUEUE_NUM) {
        s_tx_drop++;
        irq_unlock(primask);
        return PLATFORM_ERR_BUSY;
    }

    msg = &s_tx_queue[s_tx_head];
    memcpy(msg->data, buf, (size_t)fmt_len);
    msg->len = (uint16_t)fmt_len;
    s_tx_head++;
    if (s_tx_head >= MYPRINTF_QUEUE_NUM) {
        s_tx_head = 0U;
    }
    s_tx_count++;

    hal_err = dma_start_locked();
    if (hal_err != HAL_OK && hal_err != HAL_BUSY) {
        /* DMA未接管队首数据，丢弃该帧避免队列永久堵塞。 */
        queue_pop_locked();
        s_tx_drop++;
    }
    irq_unlock(primask);

    /* HAL_BUSY表示消息已安全入队，完成回调或下一次调用会重试。 */
    if (hal_err == HAL_BUSY) {
        return PLATFORM_ERR_OK;
    }
    return map_hal_err(hal_err);
}

uint32_t myprintf_get_drop(void)
{
    uint32_t primask; /* 读取计数时的中断状态 */
    uint32_t drop;    /* 丢帧计数快照 */

    primask = irq_lock();
    drop = s_tx_drop;
    irq_unlock(primask);
    return drop;
}

/**
 * @brief  USART6 DMA发送完成回调
 * @param  huart HAL回传的串口句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef hal_err; /* 下一帧DMA启动结果 */
    uint32_t primask;          /* 队列推进临界区中断状态 */

    if (huart != &huart6) {
        return;
    }

    primask = irq_lock();
    if (s_tx_busy != 0U && s_tx_count != 0U) {
        queue_pop_locked();
    }
    s_tx_busy = 0U;

    hal_err = dma_start_locked();
    if (hal_err != HAL_OK && hal_err != HAL_BUSY &&
        s_tx_count != 0U) {
        queue_pop_locked();
        s_tx_drop++;
    }
    irq_unlock(primask);
}

void myprintf_dma_err_isr(void)
{
    HAL_StatusTypeDef hal_err; /* 下一帧DMA启动结果 */
    uint32_t primask;          /* 队列恢复临界区中断状态 */

    primask = irq_lock();
    if (s_tx_busy != 0U && s_tx_count != 0U) {
        /* 当前帧可能只发送了一部分，整帧丢弃并由序号检测缺口。 */
        queue_pop_locked();
        s_tx_drop++;
    }
    s_tx_busy = 0U;

    hal_err = dma_start_locked();
    if (hal_err != HAL_OK && hal_err != HAL_BUSY &&
        s_tx_count != 0U) {
        queue_pop_locked();
        s_tx_drop++;
    }
    irq_unlock(primask);
}
