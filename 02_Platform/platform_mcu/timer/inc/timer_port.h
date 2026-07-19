/**
 * @file    timer_port.h
 * @brief   TIM9周期控制节拍逻辑接口
 */

#ifndef TIMER_PORT_H
#define TIMER_PORT_H

#include "platform_error.h"

typedef enum {
    CORE_TIMER_CTRL_10MS = 0,
    CORE_TIMER_NUM
} core_timer_t;

typedef void (*core_timer_cb_t)(void);

platform_err_t core_timer_reg(core_timer_t id, core_timer_cb_t cb);
platform_err_t core_timer_start(core_timer_t id);
platform_err_t core_timer_stop(core_timer_t id);
platform_err_t core_timer_mask(core_timer_t id);
platform_err_t core_timer_unmask(core_timer_t id);

#endif /* TIMER_PORT_H */
