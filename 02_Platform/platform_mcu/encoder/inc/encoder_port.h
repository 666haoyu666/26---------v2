/**
 * @file    encoder_port.h
 * @brief   TIM3/TIM4硬件编码器逻辑接口
 */

#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include <stdint.h>

#include "platform_error.h"

typedef enum {
    CORE_ENC_MOTOR_A = 0,
    CORE_ENC_MOTOR_B,
    CORE_ENC_NUM
} core_enc_t;

platform_err_t core_enc_start(core_enc_t id);
platform_err_t core_enc_stop(core_enc_t id);
platform_err_t core_enc_read(core_enc_t id, uint16_t *ticks);

#endif /* ENCODER_PORT_H */
