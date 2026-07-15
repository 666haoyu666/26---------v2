/**
 * @file    pid.h
 * @brief   PID控制器
 * @author  bojing
 */

#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <stdint.h>

//#define DEBUG
#define DEBUG_OUT(X) printf(X)

/*PID操作状态*/
typedef enum
{
    PID_OK               = 0,    /* 操作成功 */
    PID_ERROR            = 1,    /* 操作失败 */
    PID_ERRORPARAMETER   = 2,    /* 参数错误 */
    PID_RESERVED         = 0xFF, /* 保留状态 */
} pid_status_t;


/* 电机PID控制器 */
typedef struct
{
    /* 对象状态 */
    uint8_t is_inited;          /* 是否已实例化 */

    /* PID 参数 */
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */

    /* 控制量 */
    float target_speed;         /* 目标速度 */
    float feedback_speed;       /* 反馈速度 */

    /* 误差状态 */
    float error;                /* 当前误差 e(k) */
    float last_error;           /* 上一次误差 e(k-1) */


    float integral;             /* 误差积分 */
    float integral_limit;       /* 积分限幅 */

    /* 误差死区 */
    float dead_band;

    /* 输出状态 */
    float delta_output;         /* 本次 PID 输出增量 */
    float output;               /* 累加后的 PID 输出 */

    /* 输出限制 */
    float output_max;           /* PID 输出上限 */
    float output_min;           /* PID 输出下限 */

} motor_pid_t;

/**
 * @brief 实例化PID控制器对象
 *
 * @param pid PID控制器对象
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 输出上限
 * @param output_min 输出下限
 * @param integral_limit 积分限幅
 *
 * @return PID_OK
 * @return PID_ERROR
 * @return PID_ERRORPARAMETER
 */
pid_status_t motor_pid_inst(
    motor_pid_t *const pid,
    float kp,
    float ki,
    float kd,
    float output_max,
    float output_min,
    float integral_limit);


/**
 * @brief PID计算
 *
 * @param pid PID控制器对象
 * @param target_speed 目标速度
 * @param feedback_speed 反馈速度
 *
 * @return float PID输出值
 */
float motor_pid_calculate(
    motor_pid_t *const pid,
    float target_speed,
    float feedback_speed);

/**
 * @brief 清零PID运行状态
 *
 * @param pid PID控制器对象
 *
 * @retval PID_OK
 * @retval PID_ERRORPARAMETER
 */
pid_status_t motor_pid_reset(
    motor_pid_t *const pid);

#endif // PID_H
