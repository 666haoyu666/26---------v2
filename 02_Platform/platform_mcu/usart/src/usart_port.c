/**
 * @file    usart_port.c
 * @brief   MCU逻辑串口Port实现（STM32F411 HAL，空闲分段DMA接收）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - 串口参数由MX_USARTx_UART_Init配置，本层不重复初始化
 *          - 启动后关DMA半传事件，只保留空闲/整包完成上报
 */

#include "usart_port.h"

#include "platform_def.h"
#include "usart.h"


#include "gpio.h"

//#define UART_DEBUG 1

#ifdef UART_DEBUG
#include "myprintf.h"
char buf[64];
#endif

/** 逻辑串口到HAL句柄的映射表，ID=下标。 */
static UART_HandleTypeDef *const s_usart_map[EN_CORE_USART_NUM] = {
    &huart1, /* EN_CORE_USART_IMU：USART1(PB3=RX) */
};

/** 各逻辑口注册的接收事件回调（Adapter ISR入口）。 */
static core_usart_rx_cb_t s_rx_cb[EN_CORE_USART_NUM];

/** 各逻辑口注册的错误恢复回调，NULL=该口不恢复。 */
static core_usart_err_cb_t s_err_cb[EN_CORE_USART_NUM];

platform_err_t core_usart_rx_start(en_core_usart_t id,
                                   uint8_t *buf, uint16_t len)
{
    UART_HandleTypeDef *huart; /* 目标物理串口 */
    HAL_StatusTypeDef   hal;   /* HAL调用结果 */

    if ((id >= EN_CORE_USART_NUM) || (buf == NULL) || (len == 0U)) {
#if UART_DEBUG
        myprintf(buf, 64, "core_usart_rx_start: ERR_PARAM\n");
#endif
        return PLATFORM_ERR_PARAM;
    }
    huart = s_usart_map[id];
    if (huart->hdmarx == NULL) {
#if UART_DEBUG
        myprintf(buf, 64, "core_usart_rx_start: huart=%p has no RX DMA\n", huart);
#endif
        return PLATFORM_ERR_NOT_SUPPORTED; /* 未配置RX DMA */
    }

    hal = HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, len);
    if (hal == HAL_BUSY) {
#if UART_DEBUG
        myprintf(buf, 64, "core_usart_rx_start: huart=%p ERR_BUSY\n", huart);
#endif
        return PLATFORM_ERR_BUSY;
    }
    if (hal != HAL_OK) {
#if UART_DEBUG
        myprintf(buf, 64, "core_usart_rx_start: huart=%p ERR_FAIL\n", huart);
#endif
        return PLATFORM_ERR_FAIL;
    }
    /* 关半传事件：每段只在空闲/整包完成时上报一次 */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    return PLATFORM_ERR_OK;
}

platform_err_t core_usart_rx_stop(en_core_usart_t id)
{
    if (id >= EN_CORE_USART_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (HAL_UART_AbortReceive(s_usart_map[id]) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }
    return PLATFORM_ERR_OK;
}

platform_err_t core_usart_reg_isr(en_core_usart_t id,
                                  core_usart_rx_cb_t rx_cb,
                                  core_usart_err_cb_t err_cb)
{
    if ((id >= EN_CORE_USART_NUM) || (rx_cb == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    /* 拒绝重复注册，防止运行期暗中换表 */
    if (s_rx_cb[id] != NULL) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_rx_cb[id]  = rx_cb;
    s_err_cb[id] = err_cb;
    return PLATFORM_ERR_OK;
}

/**
 * @brief  接收事件分发：调对应口注册的Adapter回调
 * @param  id   触发的逻辑串口
 * @param  size 本段接收字节数
 */
static void core_usart_rx_event(en_core_usart_t id, uint16_t size)
{
    if (s_rx_cb[id] != NULL) {
        (void)s_rx_cb[id](size); /* ISR内不处理回调返回值 */
    }
}

/** @brief 错误分发：调对应口注册的错误恢复回调 */
static void core_usart_rx_error(en_core_usart_t id)
{
    if (s_err_cb[id] != NULL) {
        (void)s_err_cb[id]();
    }
}

/**
 * @brief  UART 空闲 / 接收事件回调（HAL 弱符号唯一实现）
 * @param  huart 触发的 UART 句柄
 * @param  size  本次收到的字节数
 * @note   ISR 上下文：仅路由，不阻塞、不打高频日志
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_12 ,GPIO_PIN_SET);
    uint32_t i; /* 映射表游标 */

    for (i = 0U; i < (uint32_t)EN_CORE_USART_NUM; i++) {
        if (huart == s_usart_map[i]) {
            core_usart_rx_event((en_core_usart_t)i, size);
						HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_12 ,GPIO_PIN_RESET);
            return;
        }
    }
#if UART_DEBUG
    myprintf(buf, 64, "HAL_UARTEx_RxEventCallback: unknown huart=%p, size=%u\n", huart, size);
#endif
		
}

/**
 * @brief  UART 错误回调（HAL 弱符号唯一实现）
 * @param  huart 出错的 UART 句柄
 * @note   路由到注册的错误恢复回调，重启对应串口接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t i; /* 映射表游标 */

    for (i = 0U; i < (uint32_t)EN_CORE_USART_NUM; i++) {
        if (huart == s_usart_map[i]) {
            core_usart_rx_error((en_core_usart_t)i);
            return;
        }
    }
#if UART_DEBUG
    myprintf(buf, 64, "HAL_UART_ErrorCallback: unknown huart=%p\n", huart);
#endif
}
