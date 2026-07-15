/**
 * @file    imu_port.h
 * @brief   里程计IMU端口：航向角与航向角速度（顺时针为正）
 * @note    - 器件为右手系（逆时针为正），经IMU_PORT_YAW_SIGN
 *            统一为顺时针为正
 *          - 读路径仅odom解算任务串行调用（Wrapper单读者约定）
 */

#ifndef IMU_PORT_H
#define IMU_PORT_H

#include "platform_error.h"

/* 航向符号：-1.0f=器件逆时针正转顺时针正；装反改+1.0f */
#ifndef IMU_PORT_YAW_SIGN
#define IMU_PORT_YAW_SIGN  (-1.0F)
#endif

/** 单拍航向姿态，符号约定与odom一致（顺时针为正）。 */
typedef struct {
    float yaw_deg;  /* 顺时针航向角，[-180,180) */
    float w_deg_s;  /* 顺时针航向角速度（陀螺仪Z轴） */
} imu_port_att_t;

/**
 * @brief  绑定IMU并探测底层可用性
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_ALREADY_INIT /
 *         底层IMU错误
 * @note   暂无有效数据(NO_RESOURCE)视为可用
 */
platform_err_t imu_port_init(void);

/**
 * @brief  解除IMU绑定
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED
 */
platform_err_t imu_port_deinit(void);

/**
 * @brief  读取最近一拍航向角与航向角速度
 * @param  att 输出姿态，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED /
 *         PLATFORM_ERR_NO_RESOURCE(暂无有效数据) / 底层IMU错误
 * @note   失败时不修改输出
 */
platform_err_t imu_port_get_att(imu_port_att_t *att);

#endif /* IMU_PORT_H */
