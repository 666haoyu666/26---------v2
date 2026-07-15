/**
 * @file    bsp_motor_driver.h
 * @brief   编码器电机对象实现
 * @author  bojing
 */

#ifndef BSP_MOTOR_DRIVER_H
#define BSP_MOTOR_DRIVER_H

#include <stdio.h>
#include <stdint.h>

#include "encoder_motor.h"
#include "pid.h"

//#define DEBUG
#define DEBUG_OUT(X) printf(X)

/* 电机操作状态 */
typedef enum
{
    MOTOR_OK             = 0,      /* 操作成功 */
    MOTOR_ERROR          = 1,      /* 通用运行错误 */
    MOTOR_ERRORTIMEOUT   = 2,      /* 操作超时 */
    MOTOR_ERRORRESOURCE  = 3,      /* 所需资源不可用 */
    MOTOR_ERRORPARAMETER = 4,      /* 参数错误 */
    MOTOR_ERRORNOMEMORY  = 5,      /* 内存分配失败 */
    MOTOR_ERRORISR       = 6,      /* 不允许在中断服务函数中执行 */
    MOTOR_RESERVED       = 0xFF,   /* 保留状态 */
} motor_status_t;

/* 电机默认旋转方向 */
typedef enum
{
    MOTOR_DIR_CW  = 0,       /* 顺时针旋转 */
    MOTOR_DIR_CCW = 1,       /* 逆时针旋转 */
} motor_dir_t;

/* 电机使能接口 */
typedef motor_status_t (*bsp_motor_set_enable_t)(uint8_t enable);

/* 电机PWM设置接口 */
typedef motor_status_t (*bsp_motor_set_output_t)(
    motor_dir_t dir,
    uint16_t pwm);



/* 电机驱动对象 */
typedef struct motor_driver
{
    /* 对象状态 */
    uint8_t is_inited;             /* 是否已实例化 */
    uint8_t is_enabled;            /* 是否已使能 */

    /* 电机输出配置 */
    uint16_t pwm_max;              /* 最大PWM输出值 */
    motor_dir_t dir;               /* 正输出对应的默认方向 */

    /*电机运行状态  */
    int16_t output;                 /* 当前输出值，正数表示正转，负数表示反转 */
    float target_speed_rps;         /* 目标转速，单位：转/秒 */

    /* 底层操作接口 */
    bsp_motor_set_enable_t pf_set_enable; /* 电机使能接口 */
    bsp_motor_set_output_t pf_set_output; /* 电机PWM设置接口 */

    /* 电机控制对象 */
    encoder_t encoder;             /* 编码器对象 */
    motor_pid_t pid;               /* PID控制器对象 */

} motor_driver_t;

/**
 * @brief  实例化单电机对象
 *
 * @param  motor         电机对象指针
 * @param  pf_set_enable 底层电机使能接口
 * @param  pf_set_output 底层PWM输出接口
 * @param  dir           电机默认旋转方向
 * @param  pwm_max       最大PWM值
 * @retval MOTOR_OK             实例化成功
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_inst(
    motor_driver_t *const motor,
    bsp_motor_set_enable_t pf_set_enable,
    bsp_motor_set_output_t pf_set_output,
    motor_dir_t dir,
    uint16_t pwm_max);

/**
 * @brief  使能或失能单电机对象
 *
 * @param  motor         电机对象指针
 * @param  enable        1：使能，0：失能
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_enable(
    motor_driver_t *const motor,
    uint8_t enable);

/**
 * @brief  设置电机输出方向和PWM
 *
 * @param  motor         电机对象指针
 * @param  output        0>正转，<0反转
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_set_output(
    motor_driver_t *const motor,
    int16_t output);


/**
 * @brief 设置目标速度
 *
 * @param motor             电机对象
 * @param target_speed_rps  目标输出轴速度，单位：转/秒
 *
 * target_speed_rps > 0：逻辑正向
 * target_speed_rps < 0：逻辑反向
 * target_speed_rps = 0：目标停止
 */
motor_status_t motor_driver_set_target_speed(
    motor_driver_t *const motor,
    float target_speed_rps);

/**
 * @brief  周期更新编码器速度和位置
 *
 * @param  motor 电机对象指针
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_encoder_update(
    motor_driver_t *const motor);


/**
 * @brief  PID计算，更新电机输出PWM
 *
 * @param  motor 电机对象指针
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_pid_update(
    motor_driver_t *const motor);


/**
 * @brief  停止电机
 *
 * @param  motor      电机对象指针
 *
 * @retval MOTOR_OK             操作成功
 * @retval MOTOR_ERROR          底层接口执行失败
 * @retval MOTOR_ERRORPARAMETER 参数错误
 */
motor_status_t motor_driver_stop(
    motor_driver_t *const motor);

#endif /* BSP_MOTOR_DRIVER_H */
