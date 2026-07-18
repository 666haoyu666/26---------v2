/**
 * @file    track_port.c
 * @brief   循迹端口：转发循迹Wrapper并转换车体误差语义
 * @note    - 底层偏差为线相对车体，本层取反为车体相对线
 *          - 仅循迹装配线程与控制任务使用，不做并发保护
 */

#include "track_port.h"

#include "bsp_wrapper_line_sensor.h"
#include "platform_def.h"

static uint8_t g_track_bound; /* 1=循迹采集能力绑定有效 */

platform_err_t track_port_init(void)
{
    bsp_lsensor_result_t probe; /* 探测读取的识别结果 */
    platform_err_t err;         /* 底层读取结果 */

    if (g_track_bound != 0U) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* 探测底层是否就绪，未注册/未初始化在装配期即暴露 */
    err = bsp_lsensor_read(&probe);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    g_track_bound = 1U;
    return PLATFORM_ERR_OK;
}

platform_err_t track_port_deinit(void)
{
    if (g_track_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    g_track_bound = 0U;
    return PLATFORM_ERR_OK;
}

platform_err_t track_port_get(track_port_result_t *result)
{
    bsp_lsensor_result_t raw; /* 底层单拍识别结果 */
    platform_err_t err;       /* 底层读取结果 */

    if (result == NULL) {
        return PLATFORM_ERR_PARAM;
    }
    if (g_track_bound == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = bsp_lsensor_read(&raw);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    switch (raw.track) {
        case BSP_LSENSOR_NO_LINE:
            result->body_err_mm = 0.0f;
            result->state = TRACK_PORT_NO_LINE;
            return PLATFORM_ERR_OK;
        case BSP_LSENSOR_TRACKING:
            result->body_err_mm = -raw.offset_mm;
            result->state = TRACK_PORT_VALID;
            return PLATFORM_ERR_OK;
        case BSP_LSENSOR_AMBIGUOUS:
            result->body_err_mm = 0.0f;
            result->state = TRACK_PORT_AMBIGUOUS;
            return PLATFORM_ERR_OK;
        default:
            /* 底层给出契约外状态，按故障处理且不修改输出 */
            return PLATFORM_ERR_FAIL;
    }
}
