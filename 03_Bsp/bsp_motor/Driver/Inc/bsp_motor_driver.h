/**
 * @file    bsp_motor_driver.h
 * @brief   编码器直流电机闭环Driver
 * @note    不依赖HAL/RTOS；硬件输出和编码器计数由Adapter注入。
 */

#ifndef BSP_MOTOR_DRIVER_H
#define BSP_MOTOR_DRIVER_H

#include <stdint.h>

#include "pid.h"

typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_PARAM,
    MOTOR_ERR_STATE,
    MOTOR_ERR_HW
} motor_status_t;

typedef enum {
    MOTOR_DIR_CW = 0,
    MOTOR_DIR_CCW
} motor_dir_t;

typedef motor_status_t (*motor_output_t)(void *ctx,
                                         motor_dir_t dir,
                                         uint16_t pwm);

typedef struct {
    void            *ctx;       /* Adapter硬件上下文 */
    motor_output_t  pf_output;  /* 方向与PWM输出接口 */
} motor_hw_if_t;

typedef struct {
    motor_dir_t      fwd_dir;    /* 逻辑正向对应物理方向 */
    int8_t           enc_sign;   /* 编码器方向修正，+1/-1 */
    uint16_t         pwm_max;    /* PWM满幅 */
    uint32_t         enc_cpr;    /* 输出轴每转计数 */
    float            sample_s;   /* 控制周期，s */
    float            filter;     /* 速度滤波系数，0到1 */
    motor_pid_cfg_t  pid_cfg;    /* 默认闭环参数 */
} motor_core_cfg_t;

typedef struct {
    float    target_rps; /* 目标输出轴转速 */
    float    speed_rps;  /* 滤波后输出轴转速 */
    float    raw_rps;    /* 本拍原始转速 */
    float    p_term;     /* 当前比例输出项 */
    float    i_term;     /* 当前积分输出项 */
    float    ff_term;    /* 当前前馈输出项 */
    int64_t  position;   /* 累计编码器计数 */
    int32_t  duty;       /* 已应用带符号PWM */
    uint8_t  enabled;    /* 闭环使能标记 */
} motor_core_state_t;

typedef struct {
    const motor_hw_if_t  *hw;         /* 注入硬件接口 */
    motor_core_cfg_t     cfg;         /* 固定配置 */
    motor_pid_t          pid;         /* PID控制器 */
    float                target_rps;  /* 当前目标转速 */
    float                speed_rps;   /* 滤波速度 */
    float                raw_rps;     /* 原始速度 */
    int64_t              position;    /* 累计位置计数 */
    int32_t              duty;        /* 已应用带符号PWM */
    uint16_t             last_ticks;  /* 上拍定时器计数 */
    uint8_t              inited;      /* 初始化标记 */
    uint8_t              enabled;     /* 闭环使能标记 */
    uint8_t              first_tick;  /* 首拍基准标记 */
    uint8_t              speed_ready; /* 滤波基准标记 */
} motor_core_t;

motor_status_t motor_core_init(motor_core_t *motor,
                               const motor_hw_if_t *hw,
                               const motor_core_cfg_t *cfg);
motor_status_t motor_core_start(motor_core_t *motor);
motor_status_t motor_core_stop(motor_core_t *motor);
motor_status_t motor_core_set_rps(motor_core_t *motor, float rps);
motor_status_t motor_core_set_pid(motor_core_t *motor,
                                  const motor_pid_cfg_t *cfg);
motor_status_t motor_core_get_pid(const motor_core_t *motor,
                                  motor_pid_cfg_t *cfg);
motor_status_t motor_core_set_filt(motor_core_t *motor, float filter);
motor_status_t motor_core_get_filt(const motor_core_t *motor,
                                   float *filter);
motor_status_t motor_core_update(motor_core_t *motor, uint16_t ticks);
motor_status_t motor_core_get_state(const motor_core_t *motor,
                                    motor_core_state_t *state);

#endif /* BSP_MOTOR_DRIVER_H */
