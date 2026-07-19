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

static motor_ctrl_cfg_t g_ctrl_cfg;   /* 左右轮槽位与换算配置 */
static uint8_t          g_ctrl_bound; /* 1=槽位绑定有效 */

platform_err_t motor_ctrl_init(const motor_ctrl_cfg_t *cfg)
{
    bsp_motor_state_t state; /* 探测读取的电机状态 */
    platform_err_t err;      /* 底层读取结果 */

    if (cfg == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (cfg->left_id >= MOTOR_SLOT_COUNT ||
        cfg->right_id >= MOTOR_SLOT_COUNT ||
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
    err = bsp_motor_get_state(cfg->left_id, &state);
    if (PLATFORM_IS_OK(err)) {
        err = bsp_motor_get_state(cfg->right_id, &state);
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
    platform_err_t err;       /* 左轮使能结果 */
    platform_err_t right_err; /* 右轮使能结果 */

    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* 两轮独立使能，右轮不因左轮失败而跳过，报首个错误 */
    err = bsp_motor_start(g_ctrl_cfg.left_id);
    right_err = bsp_motor_start(g_ctrl_cfg.right_id);
    return PLATFORM_IS_ERR(err) ? err : right_err;
}

platform_err_t motor_left_start(void)
{
    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return bsp_motor_start(g_ctrl_cfg.left_id);
}

void motor_ctrl_stop(void)
{
    if (g_ctrl_bound == 0U) {
        return;
    }

    /* 尽力失能两轮，失败由Adapter故障位锁存 */
    (void)bsp_motor_stop(g_ctrl_cfg.left_id);
    (void)bsp_motor_stop(g_ctrl_cfg.right_id);
}

void motor_ctrl_set(float left_mm_s, float right_mm_s)
{
    if (g_ctrl_bound == 0U) {
        return;
    }
    if (!isfinite(left_mm_s) || !isfinite(right_mm_s)) {
        return;
    }

    /* mm/s除以每转行程得输出轴RPS，两轮依次应用不回滚 */
    (void)bsp_motor_set_rps(g_ctrl_cfg.left_id,
                            left_mm_s / g_ctrl_cfg.left_mm_rev);
    (void)bsp_motor_set_rps(g_ctrl_cfg.right_id,
                            right_mm_s / g_ctrl_cfg.right_mm_rev);
}

platform_err_t motor_left_set(float left_mm_s)
{
    float rps; /* 左轮输出轴目标转速 */

    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (!isfinite(left_mm_s)) {
        return PLATFORM_ERR_PARAM;
    }

    /* mm/s除以左轮每转行程得输出轴RPS，仅下发左轮槽位。 */
    rps = left_mm_s / g_ctrl_cfg.left_mm_rev;
    return bsp_motor_set_rps(g_ctrl_cfg.left_id, rps);
}

platform_err_t motor_ctrl_get(motor_ctrl_state_t *state)
{
    bsp_motor_state_t left;      /* 左轮Wrapper状态 */
    bsp_motor_state_t right;     /* 右轮Wrapper状态 */
    motor_ctrl_state_t snapshot; /* 单次完整输出快照 */
    platform_err_t err;          /* Wrapper读取结果 */

    if (state == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_ctrl_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = bsp_motor_get_state(g_ctrl_cfg.left_id, &left);
    if (PLATFORM_IS_OK(err)) {
        err = bsp_motor_get_state(g_ctrl_cfg.right_id, &right);
    }
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    snapshot.left_tgt_mm_s = left.target_rps * g_ctrl_cfg.left_mm_rev;
    snapshot.right_tgt_mm_s = right.target_rps * g_ctrl_cfg.right_mm_rev;
    snapshot.left_act_mm_s = left.speed_rps * g_ctrl_cfg.left_mm_rev;
    snapshot.right_act_mm_s = right.speed_rps * g_ctrl_cfg.right_mm_rev;
    snapshot.left_duty = left.duty;
    snapshot.right_duty = right.duty;
    snapshot.left_fault = left.fault;
    snapshot.right_fault = right.fault;
    snapshot.left_run = (uint32_t)left.run;
    snapshot.right_run = (uint32_t)right.run;
    *state = snapshot;
    return PLATFORM_ERR_OK;
}
