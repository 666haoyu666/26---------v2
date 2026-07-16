/**
 * @file    imu_ctrl_port.c
 * @brief   循迹控制航向端口：转发里程计世界坐标航向
 * @note    - 航向取自odom一致快照，不直接读IMU（单读者约定）
 *          - 仅循迹装配线程与控制任务使用，不做并发保护
 */

#include "imu_ctrl_port.h"

#include <stddef.h>

#include "service_diff_odom.h"

static uint8_t g_imu_ctrl_bound; /* 1=航向能力绑定有效 */
static imu_ctrl_data_t g_last_data; /* 最近一次有效里程计快照 */

platform_err_t imu_ctrl_init(void)
{
    server_odom_state_t probe; /* 探测读取的里程计快照 */
    platform_err_t err;        /* 底层读取结果 */

    if (g_imu_ctrl_bound != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 探测里程计服务是否就绪，未初始化在装配期即暴露 */
    err = server_odom_get(&probe);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_last_data.x_mm = probe.x_mm;
    g_last_data.y_mm = probe.y_mm;
    g_last_data.yaw_deg = probe.yaw_deg;
    g_last_data.w_deg_s = probe.w_deg_s;
    g_imu_ctrl_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t imu_ctrl_deinit(void)
{
    if (g_imu_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_imu_ctrl_bound = 0U;
    g_last_data = (imu_ctrl_data_t){0};
    return PLATFORM_ERR_OK;
}

platform_err_t imu_ctrl_get_data(imu_ctrl_data_t *data)
{
    server_odom_state_t state; /* 里程计一致状态快照 */
    platform_err_t err;        /* 底层读取结果 */

    if (data == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    /* 所有失败路径均向控制器提供最近一次有效快照。 */
    *data = g_last_data;
    if (g_imu_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = server_odom_get(&state);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_last_data.x_mm = state.x_mm;
    g_last_data.y_mm = state.y_mm;
    g_last_data.yaw_deg = state.yaw_deg;
    g_last_data.w_deg_s = state.w_deg_s;
    *data = g_last_data;
    return PLATFORM_ERR_OK;
}

float imu_ctrl_get(void)
{
    imu_ctrl_data_t data; /* 最近有效的航向快照 */

    (void)imu_ctrl_get_data(&data);
    return data.yaw_deg;
}
