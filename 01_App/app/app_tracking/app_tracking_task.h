/**
 * @file    app_tracking_task.h
 * @brief   循迹小车调试与运行场景入口
 */

#ifndef APP_TRACKING_TASK_H
#define APP_TRACKING_TASK_H

#include "platform_error.h"

/**
 * @brief  执行A左轮单阶跃并在停车后导出完整10ms快照
 * @retval PLATFORM_ERR_OK / 控制服务或OSAL错误
 * @note   默认0.5s基线、1200mm/s保持2.5s、停车1s
 */
platform_err_t app_motor_tune_run(void);

#endif /* APP_TRACKING_TASK_H */
