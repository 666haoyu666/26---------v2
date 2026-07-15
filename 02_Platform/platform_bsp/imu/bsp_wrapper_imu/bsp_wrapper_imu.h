/**
 * @file    bsp_wrapper_imu.h
 * @brief   IMU稳定能力、注册与安全转发契约
 * @note    - 单IMU实例、姿态+角速度能力，器件差异由Adapter隔离
 *          - 错误码统一使用platform_err_t，由Adapter完成Driver映射
 *          - 注册仅限内核启动前；运行期能力表只读
 *          - read在线程态由单一任务串行调用；ISR路径走Adapter入口
 */

#ifndef BSP_WRAPPER_IMU_H
#define BSP_WRAPPER_IMU_H

#include "platform_error.h"

/** 单拍姿态与角速度结果，右手系。 */
typedef struct {
    float roll_deg;   /* 横滚角X，单位 度 */
    float pitch_deg;  /* 俯仰角Y，单位 度 */
    float yaw_deg;    /* 偏航角Z，单位 度 */
    float gyro_x_dps; /* 角速度Wx，单位 度/秒 */
    float gyro_y_dps; /* 角速度Wy，单位 度/秒 */
    float gyro_z_dps; /* 角速度Wz，单位 度/秒 */
} bsp_imu_data_t;

/** Adapter装配的IMU能力表，所有操作回调均必填。 */
typedef struct {
    platform_err_t (*pf_init)(void);
    platform_err_t (*pf_deinit)(void);
    platform_err_t (*pf_read)(bsp_imu_data_t *data);
} bsp_imu_dev_t;

/**
 * @brief  注册IMU能力表，供Adapter在组合根初始化阶段调用
 * @param  dev 能力表；Wrapper整体拷贝持有
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT
 * @note   重复注册返回ALREADY_INIT；仅限内核启动前单线程调用
 */
platform_err_t bsp_imu_reg(const bsp_imu_dev_t *dev);

/**
 * @brief  初始化已注册的IMU并启动数据接收
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         PLATFORM_ERR_ALREADY_INIT / Adapter回调错误
 * @note   重复初始化返回ALREADY_INIT
 */
platform_err_t bsp_imu_init(void);

/**
 * @brief  停止IMU数据接收，保留能力表以支持再次初始化
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         Adapter回调错误
 */
platform_err_t bsp_imu_deinit(void);

/**
 * @brief  读取最近一拍姿态与角速度
 * @param  data 输出结果，不可为NULL；失败时保持原值
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED /
 *         PLATFORM_ERR_NO_RESOURCE(暂无有效数据)
 */
platform_err_t bsp_imu_read(bsp_imu_data_t *data);

#endif /* BSP_WRAPPER_IMU_H */
