/**
 * @file    bsp_wrapper_line_sensor.h
 * @brief   七路循迹传感器稳定能力、注册与安全转发契约
 * @note    - 单传感器组、七个稳定物理槽位，器件差异由Adapter隔离
 *          - 错误码统一使用platform_err_t，由Adapter完成Driver映射
 *          - 注册仅限内核启动前；运行期能力表只读
 *          - 接口仅在线程态由单一采集任务串行调用，不支持ISR调用
 */

#ifndef BSP_WRAPPER_LINE_SENSOR_H
#define BSP_WRAPPER_LINE_SENSOR_H

#include "platform_error.h"

#define BSP_LSENSOR_SLOT_MAX (7U) /* 稳定物理槽位容量 */

/** 单拍循迹线识别状态。 */
typedef enum {
    BSP_LSENSOR_NO_LINE   = 0,         /* 无槽位命中 */
    BSP_LSENSOR_TRACKING  = 1,         /* 单一连续簇，偏差有效 */
    BSP_LSENSOR_AMBIGUOUS = 2,         /* 多簇或全部启用槽位命中 */
    BSP_LSENSOR_TRACK_RSVD = 0x7FFFFFFF /* 保留，固定枚举宽度 */
} bsp_lsensor_track_t;

/** 单拍传感器结果。 */
typedef struct {
    float                offset_mm; /* 线相对车体偏差，右正左负 */
    bsp_lsensor_track_t  track;     /* 识别状态 */
} bsp_lsensor_result_t;

/** Adapter装配的单传感器组能力表，所有操作回调均必填。 */
typedef struct {
    platform_err_t  (*pf_init)(void);
    platform_err_t  (*pf_deinit)(void);
    platform_err_t  (*pf_read)(bsp_lsensor_result_t *result);
} bsp_lsensor_dev_t;

/**
 * @brief  注册传感器能力表，供Adapter在组合根初始化阶段调用
 * @param  dev 能力表；Wrapper整体拷贝持有
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT
 * @note   重复注册返回ALREADY_INIT；仅限内核启动前单线程调用
 */
platform_err_t bsp_lsensor_reg(const bsp_lsensor_dev_t *dev);

/**
 * @brief  初始化已注册的传感器组
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         PLATFORM_ERR_ALREADY_INIT / Adapter回调错误
 * @note   重复初始化返回ALREADY_INIT
 */
platform_err_t bsp_lsensor_init(void);

/**
 * @brief  反初始化传感器组，保留已注册能力表以支持再次初始化
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         Adapter回调错误
 */
platform_err_t bsp_lsensor_deinit(void);

/**
 * @brief  读取并识别一拍七路循迹传感器
 * @param  result 输出结果，不可为NULL；失败时保持原值
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / Adapter回调错误
 */
platform_err_t bsp_lsensor_read(bsp_lsensor_result_t *result);

#endif /* BSP_WRAPPER_LINE_SENSOR_H */
