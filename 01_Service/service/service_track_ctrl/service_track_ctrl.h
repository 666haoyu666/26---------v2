/**
 * @file    service_track_ctrl.h
 * @brief   差速底盘循迹控制服务公共接口
 * @note    - 模式与目标速度作为一次完整命令提交
 *          - 公共速度单位为mm/s，不暴露电机转速或RTOS类型
 *          - 轮径等机械参数由底盘装配层管理
 */

#ifndef SERVICE_TRACK_CTRL_H
#define SERVICE_TRACK_CTRL_H

#include "platform_error.h"

#define SERVER_CTRL_STACK_SIZE  2048U  /* 运动控制任务栈大小，字节 */
#define SERVER_CTRL_TASK_PRIO   24U    /* 运动控制任务优先级 */
#define SERVER_CTRL_PERIOD_MS   10U    /* 运动控制周期，ms */

/** 循迹控制模式。 */
typedef enum {
    TRACK_CTRL_MODE_STOP = 0,  /* 停止，speed_mm_s无效 */
    TRACK_CTRL_MODE_TRACK_DIR, /* 按已知循线方向循迹 */
    TRACK_CTRL_MODE_TRACK,     /* 纯循迹，由算法自行判断方向 */
    TRACK_CTRL_MODE_TURN,      /* 转弯控制 */
    TRACK_CTRL_MODE_RSVD = 0x7FFFFFFF /* 保留，固定枚举宽度 */
} track_ctrl_mode_t;

/**
 * @brief  初始化循迹控制状态并创建控制任务
 * @retval PLATFORM_ERR_OK /
 *         PLATFORM_ERR_ALREADY_INIT / PLATFORM_ERR_NO_MEMORY /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_FAIL
 * @note   调用前要求共享Port与里程计服务已就绪
 */
platform_err_t track_ctrl_init(void);

/**
 * @brief  停止控制任务并清除循迹控制状态
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         PLATFORM_ERR_BUSY / PLATFORM_ERR_TIMEOUT / PLATFORM_ERR_FAIL
 * @note   调用前须停止其他循迹控制API调用
 */
platform_err_t track_ctrl_deinit(void);

/**
 * @brief  切换控制模式并设置该模式的目标速度
 * @param  mode       目标模式，须为TRACK_CTRL_MODE_*有效值，
 *                    STOP模式下 speed_mm_s无效
 * @param  target_yaw_deg 转向模式下的目标航向角，单位度；
 *                        仅在TURN和TRACK_DIR模式下有效，范围[-180, 180)；
 * @param  speed_mm_s 车体目标速度，单位mm/s；正向前进，负向后退
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_BUSY /
 *         PLATFORM_ERR_FAIL
 * @note   STOP模式仅接受0；航向输入会归一化到[-180, 180)；
 *         成功后模式、航向与速度作为一条命令同拍生效
 */
platform_err_t track_ctrl_set_mode(track_ctrl_mode_t mode,
                                   float target_yaw_deg,
                                   int32_t speed_mm_s);

#endif /* SERVICE_TRACK_CTRL_H */
