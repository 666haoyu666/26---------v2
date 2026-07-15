/**
 * @file    encoder_motor.h
 * @brief   编码器
 * @author  bojing
 * @note    本层只负责编码器的计数处理，
 *          GPIO 中断负责采集编码器读数
 */

#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <stdio.h>
#include <stdint.h>

//#define DEBUG
#define DEBUG_OUT(X) printf(X)


/* 编码器操作状态 */
typedef enum
{
    ENCODER_OK             = 0,      /* 操作成功 */
    ENCODER_ERROR          = 1,      /* 通用运行错误 */
    ENCODER_ERRORTIMEOUT   = 2,      /* 操作超时 */
    ENCODER_ERRORRESOURCE  = 3,      /* 所需资源不可用 */
    ENCODER_ERRORPARAMETER = 4,      /* 参数错误 */
    ENCODER_ERRORNOMEMORY  = 5,      /* 内存分配失败 */
    ENCODER_ERRORISR       = 6,      /* 不允许在中断服务函数中执行 */
    ENCODER_RESERVED       = 0xFF,   /* 保留状态 */
} encoder_status_t;


/* 编码器方向修正 */
typedef enum
{
    ENCODER_SIGN_NORMAL   =  1,
    ENCODER_SIGN_REVERSED = -1,
} encoder_sign_t;


/* 编码器对象 */
typedef struct
{
    /* 对象状态 */
    uint8_t is_inited;              /* 是否已实例化 */

    /* 固定参数 */
    encoder_sign_t direction_sign;  /* 原始计数方向修正 */
    uint32_t counts_per_rev;        /* 输出轴旋转一圈的GPIO有效计数 */
    float sample_period_s;          /* 固定更新周期与定时器一致，单位：秒 */

    /* 中断计数 */
    volatile int32_t pending_count; /* GPIO 中断中累加的原始计数 */

    /* 编码器反馈 */
    int32_t delta_count;            /* 本周期经过方向修正后的计数增量 */
    int64_t position_ticks;         /* 经过方向修正后的累计位置计数 */
	
	
    float raw_speed_rps;     				/* 原始滤波 */
    float speed_rps;          			/* 滤波速度 */
		float filter_alpha;       			/* 一阶低通滤波系数 */
    uint8_t filter_is_ready;  			/* 首次滤波标志 */

} encoder_t;


/**
 * @brief 实例化编码器对象
 *
 * @param encoder          编码器对象
 * @param direction_sign   原始计数方向修正
 * @param counts_per_rev   输出轴旋转一圈的有效计数
 * @param sample_period_s  固定更新周期，单位：秒
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_inst(
    encoder_t *const encoder,
    encoder_sign_t direction_sign,
    uint32_t counts_per_rev,
    float sample_period_s);


/**
 * @brief 在 GPIO 中断中增加一次原始计数
 *
 * @param encoder 编码器对象
 * @param step    本次计数变化，只允许为 +1 或 -1
 *
 * @note
 * 本函数由 GPIO 中断服务函数调用。
 * step 由硬件中断层根据 A/B 相电平判断。
 */
void encoder_count_isr(
    encoder_t *const encoder,
    int8_t step);


/**
 * @brief 周期更新编码器速度和位置
 *
 * @param encoder 编码器对象
 *
 * @note
 * 调用本函数时，需要保证 pending_count 的读取和清零
 * 不会被 GPIO 中断打断。
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_update(
    encoder_t *const encoder);


/**
 * @brief 清零编码器运行状态
 *
 * @param encoder 编码器对象
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_reset(
    encoder_t *const encoder);


#endif /* ENCODER_MOTOR_H */

