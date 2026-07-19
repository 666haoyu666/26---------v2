/**
 * @file    bsp_wrapper_motor.c
 * @brief   电机Wrapper注册表与器件无关转发实现
 */

#include "bsp_wrapper_motor.h"

#include <stddef.h>

typedef enum {
    MOTOR_GROUP_INIT = 0,
    MOTOR_GROUP_DEINIT
} motor_group_op_t;

static bsp_motor_dev_t s_motor_dev[MOTOR_SLOT_COUNT];
static uint8_t s_motor_reg[MOTOR_SLOT_COUNT];

/** @brief 获取一个已注册设备 */
static bsp_motor_dev_t *motor_slot_get(uint32_t id)
{
    if ((id >= MOTOR_SLOT_COUNT) || (s_motor_reg[id] == 0U)) {
        return NULL;
    }
    return &s_motor_dev[id];
}

/** @brief 对全部已注册槽位执行初始化类操作 */
static platform_err_t motor_group_apply(motor_group_op_t op)
{
    platform_err_t first_err = PLATFORM_ERR_OK; /* 首个错误 */
    uint32_t done = 0U;                         /* 已执行数量 */
    uint32_t i;                                 /* 槽位游标 */

    for (i = 0U; i < MOTOR_SLOT_COUNT; i++) {
        bsp_motor_dev_t *dev = motor_slot_get(i); /* 当前设备 */
        platform_err_t err;                       /* 当前结果 */

        if (dev == NULL) {
            continue;
        }
        err = (op == MOTOR_GROUP_INIT) ? dev->pf_init(dev)
                                        : dev->pf_deinit(dev);
        done++;
        if (PLATFORM_IS_ERR(err) && PLATFORM_IS_OK(first_err)) {
            first_err = err;
        }
    }

    return (done == 0U) ? PLATFORM_ERR_NOT_INITIALIZED : first_err;
}

platform_err_t bsp_motor_reg(uint32_t id,
                             const bsp_motor_dev_t *dev)
{
    if ((id >= MOTOR_SLOT_COUNT) || (dev == NULL) ||
        (dev->ctx == NULL) || (dev->pf_init == NULL) ||
        (dev->pf_deinit == NULL) || (dev->pf_start == NULL) ||
        (dev->pf_stop == NULL) || (dev->pf_set_rps == NULL) ||
        (dev->pf_set_pid == NULL) || (dev->pf_get_pid == NULL) ||
        (dev->pf_set_filt == NULL) || (dev->pf_get_filt == NULL) ||
        (dev->pf_get_state == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_motor_reg[id] != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_motor_dev[id] = *dev;
    s_motor_reg[id] = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t bsp_motor_init_all(void)
{
    return motor_group_apply(MOTOR_GROUP_INIT);
}

platform_err_t bsp_motor_deinit_all(void)
{
    return motor_group_apply(MOTOR_GROUP_DEINIT);
}

platform_err_t bsp_motor_start(uint32_t id)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if (id >= MOTOR_SLOT_COUNT) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_start(dev);
}

platform_err_t bsp_motor_stop(uint32_t id)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if (id >= MOTOR_SLOT_COUNT) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_stop(dev);
}

platform_err_t bsp_motor_set_rps(uint32_t id, float rps)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || ((rps - rps) != 0.0f)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_set_rps(dev, rps);
}

platform_err_t bsp_motor_set_pid(uint32_t id,
                                 const bsp_motor_pid_t *cfg)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || (cfg == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_set_pid(dev, cfg);
}

platform_err_t bsp_motor_get_pid(uint32_t id, bsp_motor_pid_t *cfg)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || (cfg == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_get_pid(dev, cfg);
}

platform_err_t bsp_motor_set_filt(uint32_t id, float filter)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || ((filter - filter) != 0.0f) ||
        (filter < 0.0f) || (filter > 1.0f)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_set_filt(dev, filter);
}

platform_err_t bsp_motor_get_filt(uint32_t id, float *filter)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || (filter == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_get_filt(dev, filter);
}

platform_err_t bsp_motor_get_state(uint32_t id,
                                   bsp_motor_state_t *state)
{
    bsp_motor_dev_t *dev; /* 目标设备 */

    if ((id >= MOTOR_SLOT_COUNT) || (state == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = motor_slot_get(id);
    return (dev == NULL) ? PLATFORM_ERR_NOT_INITIALIZED
                         : dev->pf_get_state(dev, state);
}
