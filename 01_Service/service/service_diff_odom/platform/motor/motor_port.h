/**
 * @file    motor_port.h
 * @brief   里程计电机端口：左右轮累计tick只读访问
 * @note    - 只绑定槽位并转发读取，设备生命周期归组合根
 *          - 供odom装配线程与解算任务单线程使用
 */

#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H

#include "platform_error.h"

/** 左右轮到物理电机槽位的绑定配置。 */
typedef struct {
    uint32_t left_id;   /* 左轮物理电机槽位 */
    uint32_t right_id;  /* 右轮物理电机槽位 */
} motor_port_cfg_t;

/**
 * @brief  绑定左右轮槽位并探测底层电机可用性
 * @param  cfg 绑定配置，不可为NULL；两槽位须有效且不同
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT / 底层电机错误
 */
platform_err_t motor_port_init(const motor_port_cfg_t *cfg);

/**
 * @brief  解除左右轮槽位绑定
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED
 */
platform_err_t motor_port_deinit(void);

/**
 * @brief  读取左右轮原始累计tick
 * @param  left_tick  输出左轮累计tick，不可为NULL
 * @param  right_tick 输出右轮累计tick，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层电机错误
 * @note   失败时不修改输出
 */
platform_err_t motor_port_get_ticks(int64_t *left_tick,
                                    int64_t *right_tick);

#endif /* MOTOR_PORT_H */
