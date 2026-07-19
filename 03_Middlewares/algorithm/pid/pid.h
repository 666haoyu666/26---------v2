/**
 * @file    pid.h
 * @brief   可在线更新参数的PID加线性前馈控制器
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>

typedef enum {
    PID_OK = 0,
    PID_ERR_PARAM,
    PID_ERR_STATE
} pid_status_t;

typedef struct {
    float  kp;       /* 比例增益 */
    float  ki;       /* 积分增益，1/s */
    float  kd;       /* 微分增益，s */
    float  kff;      /* 速度前馈斜率 */
    float  ff_bias;  /* 非零目标的静摩擦补偿 */
    float  i_limit;  /* 积分项绝对值上限 */
} motor_pid_cfg_t;

typedef struct {
    motor_pid_cfg_t  cfg;       /* 当前可调参数 */
    float            sample_s;  /* 固定采样周期，s */
    float            out_limit; /* 输出绝对值上限 */
    float            integral;  /* 积分项 */
    float            p_term;    /* 当前比例输出项 */
    float            ff_term;   /* 当前前馈输出项 */
    float            last_err;  /* 上拍误差 */
    float            output;    /* 当前输出 */
    uint8_t          inited;    /* 初始化标记 */
    uint8_t          ready;     /* 微分基准有效标记 */
} motor_pid_t;

pid_status_t motor_pid_init(motor_pid_t *pid,
                            const motor_pid_cfg_t *cfg,
                            float sample_s,
                            float out_limit);
pid_status_t motor_pid_set(motor_pid_t *pid,
                           const motor_pid_cfg_t *cfg);
pid_status_t motor_pid_get(const motor_pid_t *pid,
                           motor_pid_cfg_t *cfg);
pid_status_t motor_pid_reset(motor_pid_t *pid);
pid_status_t motor_pid_step(motor_pid_t *pid,
                            float target,
                            float feedback,
                            float *output);

#endif /* MOTOR_PID_H */
