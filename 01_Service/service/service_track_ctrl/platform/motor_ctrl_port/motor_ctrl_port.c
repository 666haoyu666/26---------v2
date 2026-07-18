/**
 * @file    motor_ctrl_port.c
 * @brief   循迹控制电机端口：车轮mm/s换算电机RPS并转发
 * @note    - 旋向与硬件绑定由Adapter隔离，正RPS即正向前进
 *          - 仅循迹装配线程与控制任务使用，不做并发保护
 */

#include <math.h>

#include "motor_ctrl_port.h"

#include "bsp_wrapper_motor.h"
#include "platform_def.h"

#define MOTOR_CTRL_WHEEL_NUM (2U) /* 差速底盘车轮数 */

static motor_ctrl_cfg_t g_ctrl_cfg;   /* 左右轮槽位与换算配置 */
static uint8_t          g_ctrl_bound; /* 1=槽位绑定有效 */

platform_err_t motor_ctrl_init(const motor_ctrl_cfg_t *cfg)
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
    if (!isfinite(cfg->left_mm_rev) || cfg->left_mm_rev <= 0.0f ||
        !isfinite(cfg->right_mm_rev) || cfg->right_mm_rev <= 0.0f) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_ctrl_bound != 0U) {
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

    g_ctrl_cfg = *cfg;
    g_ctrl_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t motor_ctrl_deinit(void)
{
    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_ctrl_bound = 0U;
    g_ctrl_cfg = (motor_ctrl_cfg_t){0};
    return PLATFORM_ERR_OK;
}

platform_err_t motor_ctrl_start(void)
{
    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* Wrapper无单槽使能，作用于全部已注册电机 */
    return drv_adapter_motor_start();
}

void motor_ctrl_stop(void)
{
    if (g_ctrl_bound == 0U) {
        return;
    }

    /* 尽力失能全部已注册电机，失败由wrapper故障位锁存 */
    (void)drv_adapter_motor_stop();
}

void motor_ctrl_set(float left_mm_s, float right_mm_s)
{
    motor_drv_target_t targets[MOTOR_CTRL_WHEEL_NUM]; /* 目标表 */

    if (g_ctrl_bound == 0U) {
        return;
    }
    if (!isfinite(left_mm_s) || !isfinite(right_mm_s)) {
        return;
    }

    /* mm/s除以每转行程得输出轴RPS，两轮经校验后依次应用 */
    targets[0].id = g_ctrl_cfg.left_id;
    targets[0].rps = left_mm_s / g_ctrl_cfg.left_mm_rev;
    targets[1].id = g_ctrl_cfg.right_id;
    targets[1].rps = (right_mm_s / g_ctrl_cfg.right_mm_rev)*1.0f;
    (void)drv_adapter_motor_set_targets(targets, MOTOR_CTRL_WHEEL_NUM);
}

platform_err_t motor_ctrl_get(motor_ctrl_state_t *state)
{
    motor_drv_state_t left;     /* 左轮Wrapper状态 */
    motor_drv_state_t right;    /* 右轮Wrapper状态 */
    motor_ctrl_state_t snapshot; /* 单次完整输出快照 */
    platform_err_t err;         /* Wrapper读取结果 */

    if (state == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = drv_adapter_motor_get_state(g_ctrl_cfg.left_id, &left);
    if (PLATFORM_IS_OK(err)) {
        err = drv_adapter_motor_get_state(g_ctrl_cfg.right_id, &right);
    }
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    snapshot.left_tgt_mm_s = left.target_rps * g_ctrl_cfg.left_mm_rev;
    snapshot.right_tgt_mm_s = right.target_rps * g_ctrl_cfg.right_mm_rev;
    snapshot.left_act_mm_s = left.rps * g_ctrl_cfg.left_mm_rev;
    snapshot.right_act_mm_s = right.rps * g_ctrl_cfg.right_mm_rev;
    snapshot.left_fault = left.fault_flags;
    snapshot.right_fault = right.fault_flags;
    snapshot.left_run = (uint32_t)left.run_state;
    snapshot.right_run = (uint32_t)right.run_state;
    *state = snapshot;
    return PLATFORM_ERR_OK;
}
