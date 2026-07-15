/**
 * @file    motor_port.c
 * @brief   里程计电机端口：绑定电机Wrapper槽位并转发tick读取
 * @note    仅odom装配线程与解算任务使用，不做并发保护
 */

#include "motor_port.h"

#include "bsp_wrapper_motor.h"
#include "platform_def.h"

static motor_port_cfg_t g_port_cfg;   /* 左右轮槽位绑定 */
static uint8_t          g_port_bound; /* 1=槽位绑定有效 */

platform_err_t motor_port_init(const motor_port_cfg_t *cfg)
{
    motor_drv_state_t state; /* 探测读取的电机状态 */
    platform_err_t err;      /* 底层读取结果 */

    if (cfg == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (cfg->left_id >= MOTOR_DRV_MAX_NUM ||
        cfg->right_id >= MOTOR_DRV_MAX_NUM ||
        cfg->left_id == cfg->right_id) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_port_bound != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 探测两槽位，未注册/未初始化在装配期即暴露 */
    err = drv_adapter_motor_get_state(cfg->left_id, &state);
    if (PLATFORM_IS_OK(err)) {
        err = drv_adapter_motor_get_state(cfg->right_id, &state);
    }
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_port_cfg = *cfg;
    g_port_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t motor_port_deinit(void)
{
    if (g_port_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_port_bound = 0U;
    g_port_cfg = (motor_port_cfg_t){0};
    return PLATFORM_ERR_OK;
}

platform_err_t motor_port_get_ticks(int64_t *left_tick,
                                    int64_t *right_tick)
{
    motor_drv_state_t left_st;  /* 左轮同拍状态快照 */
    motor_drv_state_t right_st; /* 右轮同拍状态快照 */
    platform_err_t err;         /* 底层读取结果 */

    if (left_tick == NULL || right_tick == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_port_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = drv_adapter_motor_get_state(g_port_cfg.left_id, &left_st);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = drv_adapter_motor_get_state(g_port_cfg.right_id, &right_st);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    *left_tick = left_st.position;
    *right_tick = right_st.position;
    return PLATFORM_ERR_OK;
}
