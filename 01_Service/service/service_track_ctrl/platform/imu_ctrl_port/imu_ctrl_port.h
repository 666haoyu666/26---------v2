/**
 * @file    imu_ctrl_port.h
 * @brief   循迹控制航向状态端口契约
 * @note    - 通过里程计一致快照获取航向，不直接读IMU
 *          - 航向角与角速度以顺时针为正
 *          - 仅支持线程态调用，不支持ISR调用
 */

#ifndef IMU_CTRL_PORT_H
#define IMU_CTRL_PORT_H

#include "platform_error.h"

/** 循迹控制所需的航向状态。 */
typedef struct {
    int32_t x_mm;       /* 世界坐标X位置，mm */
    int32_t y_mm;       /* 世界坐标Y位置，mm */
    float  yaw_deg;  /* 顺时针航向角，范围[-180, 180) */
    float  w_deg_s;  /* 顺时针航向角速度，单位度/秒 */
} imu_ctrl_data_t;

/**
 * @brief  绑定里程计航向能力并探测可用性
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_ALREADY_INIT /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_FAIL
 * @note   不初始化或接管里程计服务生命周期
 */
platform_err_t imu_ctrl_init(void);

/**
 * @brief  解除里程计航向能力绑定
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED
 * @note   不反初始化里程计服务
 */
platform_err_t imu_ctrl_deinit(void);

/**
 * @brief  读取里程计同一解算拍的位置与航向快照
 * @param  data 输出快照，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   读取失败时仍返回最近一次有效快照，便于控制冻结
 */
platform_err_t imu_ctrl_get_data(imu_ctrl_data_t *data);

/**
 * @brief  读取里程计对齐后的顺时针航向角
 * @retval 航向角，单位deg，范围[-180, 180)
 * @note   读取失败时冻结返回最近一次有效值，初值0
 */
float imu_ctrl_get(void);

#endif /* IMU_CTRL_PORT_H */
