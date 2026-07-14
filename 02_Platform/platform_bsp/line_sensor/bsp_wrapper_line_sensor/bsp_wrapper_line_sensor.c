/**
 * @file    bsp_wrapper_line_sensor.c
 * @brief   循迹传感器Wrapper：能力表持有、生命周期门禁与安全转发
 * @note    - 不含器件、HAL、RTOS知识，全部具体绑定在Adapter完成
 *          - 契约限定注册在内核启动前、调用在单一任务，故不加锁
 */

#include "bsp_wrapper_line_sensor.h"

#include "platform_def.h"

/** Adapter注册的能力表副本。 */
static bsp_lsensor_dev_t s_dev;

/** 1=能力表已注册。 */
static uint8_t s_registered = 0U;

/** 1=已成功初始化，可读取。 */
static uint8_t s_active = 0U;

platform_err_t bsp_lsensor_reg(const bsp_lsensor_dev_t *dev)
{
    if ((dev == NULL) || (dev->pf_init == NULL) ||
        (dev->pf_deinit == NULL) || (dev->pf_read == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    /* 拒绝重复注册，防止运行期暗中换表 */
    if (s_registered != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_dev        = *dev;
    s_registered = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t bsp_lsensor_init(void)
{
    platform_err_t err; /* Adapter回调结果 */

    if (s_registered == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (s_active != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_dev.pf_init();
    if (PLATFORM_IS_ERR(err)) {
        return err; /* 失败不置位，允许上层重试 */
    }
    s_active = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t bsp_lsensor_deinit(void)
{
    platform_err_t err; /* Adapter回调结果 */

    if ((s_registered == 0U) || (s_active == 0U)) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_dev.pf_deinit();
    if (PLATFORM_IS_ERR(err)) {
        return err; /* 失败保持ACTIVE，硬件状态未知时不装作已关闭 */
    }
    s_active = 0U; /* 能力表保留，支持再次init */
    return PLATFORM_ERR_OK;
}

platform_err_t bsp_lsensor_read(bsp_lsensor_result_t *result)
{
    if (result == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (s_active == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* 失败时不修改result由Adapter回调契约保证 */
    return s_dev.pf_read(result);
}
