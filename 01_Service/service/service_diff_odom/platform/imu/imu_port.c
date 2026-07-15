/**
 * @file    imu_port.c
 * @brief   里程计IMU端口：转发IMU Wrapper并统一航向符号
 * @note    仅odom装配线程与解算任务使用，不做并发保护
 */

#include <math.h>

#include "imu_port.h"

#include "bsp_wrapper_imu.h"
#include "platform_def.h"

#define IMU_YAW_PERIOD_DEG 360.0F /* 航向角周期，deg */
#define IMU_YAW_HALF_DEG   180.0F /* 航向角半周期，deg */

static uint8_t g_imu_bound; /* 1=IMU绑定有效 */

/**
 * @brief  将航向角归一化到[-180,180)
 * @param  yaw_deg 任意航向角，deg
 * @retval 归一化后的航向角
 */
static float imu_norm_yaw(float yaw_deg)
{
    float norm_deg; /* 归一化后的航向角 */

    norm_deg = fmodf(yaw_deg + IMU_YAW_HALF_DEG, IMU_YAW_PERIOD_DEG);
    if (norm_deg < 0.0F) {
        norm_deg += IMU_YAW_PERIOD_DEG;
    }

    return norm_deg - IMU_YAW_HALF_DEG;
}

platform_err_t imu_port_init(void)
{
    bsp_imu_data_t probe; /* 探测读取的IMU数据 */
    platform_err_t err;   /* 底层读取结果 */

    if (g_imu_bound != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 探测底层是否就绪；刚上电暂无数据不算故障 */
    err = bsp_imu_read(&probe);
    if (PLATFORM_IS_ERR(err) && err != PLATFORM_ERR_NO_RESOURCE) {
        return err;
    }

    g_imu_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t imu_port_deinit(void)
{
    if (g_imu_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_imu_bound = 0U;
    return PLATFORM_ERR_OK;
}

platform_err_t imu_port_get_att(imu_port_att_t *att)
{
    bsp_imu_data_t data; /* 底层单拍姿态与角速度 */
    platform_err_t err;  /* 底层读取结果 */

    if (att == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_imu_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = bsp_imu_read(&data);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    /* 器件右手系→odom顺时针为正，同一符号作用于角与角速度 */
    att->yaw_deg = imu_norm_yaw(IMU_PORT_YAW_SIGN * data.yaw_deg);
    att->w_deg_s = IMU_PORT_YAW_SIGN * data.gyro_z_dps;
    return PLATFORM_ERR_OK;
}
