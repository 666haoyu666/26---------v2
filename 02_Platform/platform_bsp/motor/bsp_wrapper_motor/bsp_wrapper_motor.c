/**
 * @file    bsp_wrapper_motor.c
 * @brief   电机Wrapper实现：注册表持有、参数校验与器件无关转发
 * @note    - 注册表仅在组合根初始化阶段写入，运行期只读故不加锁
 *          - 不含任何器件、HAL、RTOS知识，绑定全部在Adapter完成
 *          - 错误码统一platform_err_t：未注册报NOT_INITIALIZED，
 *            重复注册报ALREADY_INIT，参数非法报PARAM
 */

#include "bsp_wrapper_motor.h"

#include <stddef.h>

/** 已注册设备表存储，索引即稳定物理槽位。 */
static motor_drv_t s_motor_dev[MOTOR_DRV_MAX_NUM];

/** 槽位注册位图，位i=1表示槽位i已注册。 */
static uint8_t s_motor_reg_map = 0U;

/** 群体操作选择子，仅本文件使用。 */
typedef enum {
    MOTOR_OP_INIT = 0, /* pf_init */
    MOTOR_OP_DEINIT,   /* pf_deinit */
    MOTOR_OP_START,    /* pf_start */
    MOTOR_OP_STOP      /* pf_stop */
} motor_op_t;

static motor_drv_t *slot_get(uint32_t id);
static platform_err_t group_apply(motor_op_t op);

/**
 * @brief  取指定槽位的已注册设备表
 * @param  id 稳定物理槽位
 * @retval 设备表指针，越界或未注册返回NULL
 */
static motor_drv_t *slot_get(uint32_t id)
{
    if ((id >= MOTOR_DRV_MAX_NUM) ||
        ((s_motor_reg_map & (uint8_t)(1U << id)) == 0U)) {
        return NULL;
    }
    return &s_motor_dev[id];
}

/**
 * @brief  对全部已注册槽位执行同签名op
 * @param  op 群体操作选择子
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层首个错误
 * @note   单槽失败不中断遍历，保证其余电机仍被处理
 */
static platform_err_t group_apply(motor_op_t op)
{
    platform_err_t first_err = PLATFORM_ERR_OK; /* 首个底层错误 */
    uint32_t       done_cnt  = 0U;              /* 实际执行槽位数 */
    uint32_t       i;                           /* 槽位游标 */

    for (i = 0U; i < MOTOR_DRV_MAX_NUM; i++) {
        motor_drv_t   *dev = slot_get(i); /* 当前槽位设备表 */
        platform_err_t ret;               /* 单槽位执行结果 */

        if (dev == NULL) {
            continue;
        }
        switch (op) {
        case MOTOR_OP_INIT:
            ret = dev->pf_init(dev);
            break;
        case MOTOR_OP_DEINIT:
            ret = dev->pf_deinit(dev);
            break;
        case MOTOR_OP_START:
            ret = dev->pf_start(dev);
            break;
        case MOTOR_OP_STOP:
        default:
            ret = dev->pf_stop(dev);
            break;
        }
        done_cnt++;
        if (PLATFORM_IS_ERR(ret) && PLATFORM_IS_OK(first_err)) {
            first_err = ret;
        }
    }

    if (done_cnt == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    return first_err;
}

platform_err_t drv_adapter_motor_reg(uint32_t index,
                                     const motor_drv_t *dev)
{
    if ((index >= MOTOR_DRV_MAX_NUM) || (dev == NULL) ||
        (dev->pf_init == NULL) || (dev->pf_deinit == NULL) ||
        (dev->pf_start == NULL) || (dev->pf_stop == NULL) ||
        (dev->pf_set_rps == NULL) || (dev->pf_get_state == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    /* 拒绝重复注册，防止运行期暗中换表 */
    if ((s_motor_reg_map & (uint8_t)(1U << index)) != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_motor_dev[index]     = *dev;
    s_motor_dev[index].idx = index; /* 槽位以注册索引为准 */
    s_motor_reg_map |= (uint8_t)(1U << index);
    return PLATFORM_ERR_OK;
}

platform_err_t drv_adapter_motor_init(void)
{
    return group_apply(MOTOR_OP_INIT);
}

platform_err_t drv_adapter_motor_deinit(void)
{
    return group_apply(MOTOR_OP_DEINIT);
}

platform_err_t drv_adapter_motor_start(void)
{
    return group_apply(MOTOR_OP_START);
}

platform_err_t bsp_motor_start(uint32_t id)
{
    motor_drv_t *dev; /* 目标槽位设备表 */

    if (id >= MOTOR_DRV_MAX_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    dev = slot_get(id);
    if (dev == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    return dev->pf_start(dev);
}

platform_err_t drv_adapter_motor_stop(void)
{
    return group_apply(MOTOR_OP_STOP);
}

platform_err_t drv_adapter_motor_set_rps(uint32_t id, float rps)
{
    motor_drv_t *dev; /* 目标槽位设备表 */

    /* 有限值自减恒为0；NaN与±inf自减均为NaN，一并拒绝 */
    if ((id >= MOTOR_DRV_MAX_NUM) || ((rps - rps) != 0.0f)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = slot_get(id);
    if (dev == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    return dev->pf_set_rps(dev, rps);
}

platform_err_t drv_adapter_motor_set_targets(
    const motor_drv_target_t *targets,
    uint32_t count)
{
    uint32_t i; /* 条目游标 */

    if ((targets == NULL) || (count == 0U) ||
        (count > MOTOR_DRV_MAX_NUM)) {
        return PLATFORM_ERR_PARAM;
    }

    /* 伪原子：先整体校验，任一非法则不产生任何副作用 */
    for (i = 0U; i < count; i++) {
        if ((targets[i].id >= MOTOR_DRV_MAX_NUM) ||
            ((targets[i].rps - targets[i].rps) != 0.0f)) {
            return PLATFORM_ERR_PARAM;
        }
        if (slot_get(targets[i].id) == NULL) {
            return PLATFORM_ERR_NOT_INITIALIZED;
        }
    }

    for (i = 0U; i < count; i++) {
        motor_drv_t   *dev = slot_get(targets[i].id); /* 已校验 */
        platform_err_t ret = dev->pf_set_rps(dev, targets[i].rps);

        /* 中途失败即返回；已应用目标不回退，故称伪原子 */
        if (PLATFORM_IS_ERR(ret)) {
            return ret;
        }
    }
    return PLATFORM_ERR_OK;
}

platform_err_t drv_adapter_motor_get_state(uint32_t id,
                                           motor_drv_state_t *state)
{
    motor_drv_t *dev; /* 目标槽位设备表 */

    if ((id >= MOTOR_DRV_MAX_NUM) || (state == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    dev = slot_get(id);
    if (dev == NULL) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    return dev->pf_get_state(dev, state);
}
