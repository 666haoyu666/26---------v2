/**
 * @file    board_line_sensor_config.h
 * @brief   灰度传感器组板级映射
 * @note    - 定义必须严格来源于00_Docs/01_资源分配表
 *          - IOGROUP必须与io_port.c输入组表下标对齐
 *          - 间距与有效电平实测后回填并同步资源分配表
 */


#include "platform_def.h"

#ifndef BOARD_LINE_SENSOR_CONFIG_H
#define BOARD_LINE_SENSOR_CONFIG_H

#define BOARD_LSENSOR_IOGROUP      (0U)   /* 逻辑输入组ID，组表下标 */
#define BOARD_LSENSOR_ENABLED_MASK (0x7FU) /* 7路装配槽位位图 */
#define BOARD_LSENSOR_ACTIVE_LOW   (0U)   /* 1=压线为低电平，待实测 */
#define BOARD_LSENSOR_PITCH_MM     (10.0f) /* 探头中心距mm，待实测 */
#define BOARD_LSENSOR_POSITIONS_MM \
    { -39.7f, -25.2f, -10.7f, 0.0f, 10.7f, 25.2f, 39.7f } /* 槽位坐标mm */


#endif /* BOARD_LINE_SENSOR_CONFIG_H */
