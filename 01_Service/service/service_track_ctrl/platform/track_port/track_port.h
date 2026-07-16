/**
 * @file    track_port.h
 * @brief   循迹模块车体横向误差端口契约
 * @note    - 误差表示车体相对循迹线中心的偏移
 *          - 右偏为正，左偏为负，单位为mm
 *          - 仅支持线程态调用，不支持ISR调用
 */

#ifndef TRACK_PORT_H
#define TRACK_PORT_H

#include "platform_error.h"

/** 循迹识别状态。 */
typedef enum {
    TRACK_PORT_NO_LINE = 0,          /* 未检测到循迹线 */
    TRACK_PORT_VALID,                /* 单一连续轨迹，误差有效 */
    TRACK_PORT_AMBIGUOUS,            /* 多轨迹或全部通道命中 */
    TRACK_PORT_STATE_RSVD = 0x7FFFFFFF /* 保留，固定枚举宽度 */
} track_port_state_t;

/** 循迹模块向上提供的车体误差结果。 */
typedef struct {
    float               body_err_mm; /* 车体横向误差，右正左负 */
    track_port_state_t  state;       /* 循迹识别状态 */
} track_port_result_t;

/**
 * @brief  初始化循迹误差采集能力
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_ALREADY_INIT /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   由共享Port生命周期所有者在消费者启动前调用
 */
platform_err_t track_port_init(void);

/**
 * @brief  反初始化循迹误差采集能力
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   仅在所有消费者已停止后调用
 */
platform_err_t track_port_deinit(void);

/**
 * @brief  读取最近一拍车体横向误差
 * @param  result 输出结果，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   仅VALID状态下body_err_mm有效；失败时不修改输出
 */
platform_err_t track_port_get(track_port_result_t *result);

#endif /* TRACK_PORT_H */
