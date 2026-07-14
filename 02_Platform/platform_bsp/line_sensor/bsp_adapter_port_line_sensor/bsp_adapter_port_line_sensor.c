/**
 * @file    bsp_adapter_port_line_sensor.c
 * @brief   循迹传感器Adapter：绑定八路灰度Driver、板级映射与MCU输入组
 * @note    - 唯一知道器件型号、几何/极性参数和输入组接线的位置
 *          - 状态码与识别枚举的Driver->platform映射全部在本层完成
 *          - 读取失败时不修改输出，满足Wrapper契约
 */

#include "bsp_adapter_port_line_sensor.h"

#include "board_line_sensor_config.h"
#include "bsp_lsensor_driver.h"
#include "io_port.h"
#include "platform_def.h"

#define LSENSOR_RAW_MASK     (0x7FU) /* 八路原始电平有效位 */

static lsensor_drv_status_t lsensor_get_map(uint8_t *raw_map);
static platform_err_t lsensor_map_err(lsensor_drv_status_t status);
static platform_err_t lsensor_adp_init(void);
static platform_err_t lsensor_adp_deinit(void);
static platform_err_t lsensor_adp_read(bsp_lsensor_result_t *result);

/**
 * @brief  将MCU输入组读取适配为driver原始位图回调
 * @param  raw_map 八路原始电平位图输出
 * @retval LSENSOR_DRV_OK / LSENSOR_DRV_ERR_PARAM / LSENSOR_DRV_ERR_IO
 */
static lsensor_drv_status_t lsensor_get_map(uint8_t *raw_map)
{
    platform_err_t err;    /* MCU输入组读取结果 */
    uint32_t       bitmap; /* MCU层返回的组电平位图 */

    if (raw_map == NULL) {
        return LSENSOR_DRV_ERR_PARAM;
    }

    err = core_iogroup_read(BOARD_LSENSOR_IOGROUP, &bitmap);
    if (PLATFORM_IS_ERR(err) ||
        ((bitmap & ~LSENSOR_RAW_MASK) != 0U)) {
        return LSENSOR_DRV_ERR_IO;
    }

    *raw_map = (uint8_t)bitmap;
    return LSENSOR_DRV_OK;
}

/**
 * @brief  将driver状态码映射为平台错误码
 * @param  status driver调用结果
 * @retval 对应platform_err_t，未知状态归入FAIL
 */
static platform_err_t lsensor_map_err(lsensor_drv_status_t status)
{
    switch (status) {
        case LSENSOR_DRV_OK:
            return PLATFORM_ERR_OK;

        case LSENSOR_DRV_ERR_PARAM:
            return PLATFORM_ERR_PARAM;

        case LSENSOR_DRV_ERR_STATE:
            return PLATFORM_ERR_NOT_INITIALIZED;

        case LSENSOR_DRV_ERR_IO:
        default:
            return PLATFORM_ERR_FAIL;
    }
}

/**
 * @brief  依据板级参数组装配置并初始化driver
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 */
static platform_err_t lsensor_adp_init(void)
{
    lsensor_drv_cfg_t cfg; /* 本次driver装配配置 */
    uint8_t           i;   /* 传感器槽位游标 */

    cfg.positions_mm = BOARD_LSENSOR_POSITIONS_MM;
    cfg.enabled_mask = (uint8_t)BOARD_LSENSOR_ENABLED_MASK;
    cfg.active_low   = (uint8_t)BOARD_LSENSOR_ACTIVE_LOW;
    cfg.get_map      = lsensor_get_map;

    return lsensor_map_err(lsensor_drv_init(&cfg));
}

/**
 * @brief  注销driver
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_FAIL
 */
static platform_err_t lsensor_adp_deinit(void)
{
    return lsensor_map_err(lsensor_drv_deinit());
}

/**
 * @brief  读取一拍识别结果并转换为platform域类型
 * @param  result 输出结果，失败时保持原值
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_FAIL
 */
static platform_err_t lsensor_adp_read(bsp_lsensor_result_t *result)
{
    lsensor_drv_result_t drv_res; /* driver单拍识别结果 */
    lsensor_drv_status_t status;  /* driver调用结果 */
    bsp_lsensor_track_t  track;   /* 映射后的识别状态 */

    if (result == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    status = lsensor_drv_read(&drv_res);
    if (status != LSENSOR_DRV_OK) {
        return lsensor_map_err(status); /* 不修改result */
    }

    switch (drv_res.track) {
        case LSENSOR_DRV_NO_LINE:
            track = BSP_LSENSOR_NO_LINE;
            break;

        case LSENSOR_DRV_TRACKING:
            track = BSP_LSENSOR_TRACKING;
            break;

        case LSENSOR_DRV_AMBIGUOUS:
            track = BSP_LSENSOR_AMBIGUOUS;
            break;

        default:
            return PLATFORM_ERR_FAIL; /* 未知枚举不透传 */
    }

    result->offset_mm = drv_res.offset_mm;
    result->track     = track;
    return PLATFORM_ERR_OK;
}

platform_err_t bsp_lsensor_adp_reg(void)
{
    bsp_lsensor_dev_t dev; /* 待注册的能力表 */

    dev.pf_init   = lsensor_adp_init;
    dev.pf_deinit = lsensor_adp_deinit;
    dev.pf_read   = lsensor_adp_read;

    return bsp_lsensor_reg(&dev);
}
