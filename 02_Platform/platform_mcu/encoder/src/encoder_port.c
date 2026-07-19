/**
 * @file    encoder_port.c
 * @brief   双路硬件编码器映射（A=TIM4，B=TIM3）
 * @note    定时器保持16位自由回绕，差值与方向修正在Driver完成。
 */

#include "encoder_port.h"

#include <stddef.h>

#include "tim.h"

static TIM_HandleTypeDef *const s_enc_map[CORE_ENC_NUM] = {
    &htim4,
    &htim3
};

static uint8_t s_enc_started[CORE_ENC_NUM];

platform_err_t core_enc_start(core_enc_t id)
{
    if (id >= CORE_ENC_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_enc_started[id] != 0U) {
        return PLATFORM_ERR_OK;
    }

    __HAL_TIM_SET_COUNTER(s_enc_map[id], 0U);
    if (HAL_TIM_Encoder_Start(s_enc_map[id],
                              TIM_CHANNEL_ALL) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_enc_started[id] = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_enc_stop(core_enc_t id)
{
    if (id >= CORE_ENC_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_enc_started[id] == 0U) {
        return PLATFORM_ERR_OK;
    }
    if (HAL_TIM_Encoder_Stop(s_enc_map[id],
                             TIM_CHANNEL_ALL) != HAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    s_enc_started[id] = 0U;
    return PLATFORM_ERR_OK;
}

platform_err_t core_enc_read(core_enc_t id, uint16_t *ticks)
{
    if ((id >= CORE_ENC_NUM) || (ticks == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_enc_started[id] == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *ticks = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_map[id]);
    return PLATFORM_ERR_OK;
}
