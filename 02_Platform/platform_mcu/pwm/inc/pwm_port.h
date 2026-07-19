/**
 * @file    pwm_port.h
 * @brief   MCU逻辑PWM接口
 */

#ifndef PWM_PORT_H
#define PWM_PORT_H

#include <stdint.h>

#include "platform_error.h"

typedef enum {
    CORE_PWM_MOTOR_A = 0,
    CORE_PWM_MOTOR_B,
    CORE_PWM_NUM
} core_pwm_t;

platform_err_t core_pwm_start(core_pwm_t id);
platform_err_t core_pwm_stop(core_pwm_t id);
platform_err_t core_pwm_set(core_pwm_t id, uint16_t duty);
platform_err_t core_pwm_get_max(core_pwm_t id, uint16_t *max_duty);

#endif /* PWM_PORT_H */
