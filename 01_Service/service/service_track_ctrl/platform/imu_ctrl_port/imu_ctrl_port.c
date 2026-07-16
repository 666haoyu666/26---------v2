/**
 * @file    imu_ctrl_port.c
 * @brief   循迹控制航向端口：转发里程计世界坐标航向
 * @note    - 航向取自odom一致快照，不直接读IMU（单读者约定）
 *          - 仅循迹装配线程与控制任务使用，不做并发保护
 */

#include "imu_ctrl_port.h"

#include "service_diff_odom.h"

static uint8_t g_imu_ctrl_bound; /* 1=航向能力绑定有效 */
static float   g_last_yaw_deg;   /* 最近一次有效航向，deg */

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

    g_last_yaw_deg = probe.yaw_deg;
    g_imu_ctrl_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t imu_ctrl_deinit(void)
{
    if (g_imu_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_imu_ctrl_bound = 0U;
    g_last_yaw_deg = 0.0f;
    return PLATFORM_ERR_OK;
}

float imu_ctrl_get(void)
{
    server_odom_state_t state; /* 里程计一致状态快照 */
    platform_err_t err;        /* 底层读取结果 */

    if (g_imu_ctrl_bound == 0U) {
        return g_last_yaw_deg;
    }

    err = server_odom_get(&state);
    if (PLATFORM_IS_ERR(err)) {
        /* 读取失败冻结航向，避免向0跳变引发猛打角 */
        return g_last_yaw_deg;
    }

    g_last_yaw_deg = state.yaw_deg;
    return g_last_yaw_deg;
}
