/**
 * @file    bsp_wrapper_motor.h
 * @brief   器件无关的双路电机能力注册与安全转发接口
 * @note    注册仅在组合根执行；运行期设备表只读。
 */

#ifndef BSP_WRAPPER_MOTOR_H
#define BSP_WRAPPER_MOTOR_H

#include <stdint.h>

#include "platform_error.h"

#define MOTOR_SLOT_COUNT (2U)      /* 稳定物理槽位数量 */
#define MOTOR_FAULT_NONE (0U)      /* 无故障 */
#define MOTOR_FAULT_HW   (1U << 0) /* 控制硬件链路故障 */

typedef enum {
    BSP_MOTOR_OFF = 0,
    BSP_MOTOR_ACTIVE,
    BSP_MOTOR_ERROR
} bsp_motor_run_t;

typedef struct {
    float  kp;       /* 比例增益 */
    float  ki;       /* 积分增益，1/s */
    float  kd;       /* 微分增益，s */
    float  kff;      /* 速度前馈斜率 */
    float  ff_bias;  /* 静摩擦补偿 */
    float  i_limit;  /* 积分项绝对值上限 */
} bsp_motor_pid_t;

typedef struct {
    float            target_rps; /* 当前目标转速 */
    float            speed_rps;  /* 滤波后转速 */
    float            raw_rps;    /* 原始转速 */
    float            p_term;     /* 当前比例输出项 */
    float            i_term;     /* 当前积分输出项 */
    float            ff_term;    /* 当前前馈输出项 */
    int64_t          position;   /* 累计编码器计数 */
    int32_t          duty;       /* 已应用带符号PWM */
    uint32_t         fault;      /* MOTOR_FAULT_*位图 */
    bsp_motor_run_t  run;        /* 当前运行状态 */
} bsp_motor_state_t;

typedef struct bsp_motor_dev bsp_motor_dev_t;

struct bsp_motor_dev {
    void            *ctx; /* Adapter私有上下文 */
    platform_err_t  (*pf_init)(bsp_motor_dev_t *dev);
    platform_err_t  (*pf_deinit)(bsp_motor_dev_t *dev);
    platform_err_t  (*pf_start)(bsp_motor_dev_t *dev);
    platform_err_t  (*pf_stop)(bsp_motor_dev_t *dev);
    platform_err_t  (*pf_set_rps)(bsp_motor_dev_t *dev, float rps);
    platform_err_t  (*pf_set_pid)(bsp_motor_dev_t *dev,
                                  const bsp_motor_pid_t *cfg);
    platform_err_t  (*pf_get_pid)(bsp_motor_dev_t *dev,
                                  bsp_motor_pid_t *cfg);
    platform_err_t  (*pf_set_filt)(bsp_motor_dev_t *dev, float filter);
    platform_err_t  (*pf_get_filt)(bsp_motor_dev_t *dev, float *filter);
    platform_err_t  (*pf_get_state)(bsp_motor_dev_t *dev,
                                    bsp_motor_state_t *state);
};

platform_err_t bsp_motor_reg(uint32_t id,
                             const bsp_motor_dev_t *dev);
platform_err_t bsp_motor_init_all(void);
platform_err_t bsp_motor_deinit_all(void);
platform_err_t bsp_motor_start(uint32_t id);
platform_err_t bsp_motor_stop(uint32_t id);
platform_err_t bsp_motor_set_rps(uint32_t id, float rps);
platform_err_t bsp_motor_set_pid(uint32_t id,
                                 const bsp_motor_pid_t *cfg);
platform_err_t bsp_motor_get_pid(uint32_t id, bsp_motor_pid_t *cfg);
platform_err_t bsp_motor_set_filt(uint32_t id, float filter);
platform_err_t bsp_motor_get_filt(uint32_t id, float *filter);
platform_err_t bsp_motor_get_state(uint32_t id,
                                   bsp_motor_state_t *state);

#endif /* BSP_WRAPPER_MOTOR_H */
