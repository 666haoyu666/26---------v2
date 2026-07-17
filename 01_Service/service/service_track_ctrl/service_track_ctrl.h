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

/* 1=编译同拍控制插桩，0=移除快照与打印代码。 */
#ifndef TRACK_TRACE_EN
#define TRACK_TRACE_EN 1U
#endif

/** 循迹控制模式。 */
typedef enum {
    TRACK_CTRL_MODE_STOP = 0,  /* 停止，speed_mm_s无效 */
    TRACK_CTRL_MODE_TRACK_DIR, /* 按已知循线方向循迹 */
    TRACK_CTRL_MODE_TRACK,     /* 纯循迹，由算法自行判断方向 */
    TRACK_CTRL_MODE_TURN,      /* 转弯控制 */
    TRACK_CTRL_MODE_MOTOR_A,   /* A左轮单轮速度内环标定 */
    TRACK_CTRL_MODE_RSVD = 0x7FFFFFFF /* 保留，固定枚举宽度 */
} track_ctrl_mode_t;

#if TRACK_TRACE_EN
/** 单个10ms控制周期内采集的完整插桩快照。 */
typedef struct {
    uint32_t tick_ms;         /* HAL uwTick采样时间，ms */
    uint32_t cycle;           /* 控制周期递增序号 */
    uint32_t mode;            /* track_ctrl_mode_t */
    uint32_t line_state;      /* track_port_state_t */
    int32_t  port_st;         /* 循迹Port读取状态 */
    int32_t  imu_st;          /* 里程计快照读取状态 */
    int32_t  motor_st;        /* 电机反馈读取状态 */
    int32_t  x_mm;            /* 同拍里程计X位置 */
    int32_t  y_mm;            /* 同拍里程计Y位置 */
    float    yaw_deg;         /* 同拍航向角 */
    float    yaw_rate_deg_s;  /* 同拍航向角速度 */
    float    raw_err_mm;      /* 本拍原始二值质心误差 */
    float    filt_err_mm;     /* 控制实际使用的滤波误差 */
    float    yaw_tgt_deg;     /* 本拍目标航向 */
    float    yaw_err_deg;     /* 归一化航向误差 */
    float    w_cmd_deg_s;     /* 目标车体角速度 */
    float    v_cmd_mm_s;      /* 目标车体前向速度 */
    float    left_cmd_mm_s;   /* 本拍左轮下发目标 */
    float    right_cmd_mm_s;  /* 本拍右轮下发目标 */
    float    left_act_mm_s;   /* 左轮最近反馈速度 */
    float    right_act_mm_s;  /* 右轮最近反馈速度 */
    int32_t  left_duty;       /* 左轮当前PWM原始输出 */
    int32_t  right_duty;      /* 右轮当前PWM原始输出 */
    uint32_t left_fault;      /* 左轮故障位 */
    uint32_t right_fault;     /* 右轮故障位 */
} track_trace_t;
#endif

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
 * @param  speed_mm_s 车体目标速度，单位mm/s；正向前进，负向后退；
 *                    MOTOR_A模式下表示A左轮单轮目标速度
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_BUSY /
 *         PLATFORM_ERR_FAIL
 * @note   STOP模式仅接受0；航向输入会归一化到[-180, 180)；
 *         成功后模式、航向与速度作为一条命令同拍生效
 */
platform_err_t track_ctrl_set_mode(track_ctrl_mode_t mode,
                                   float target_yaw_deg,
                                   int32_t speed_mm_s);

#if TRACK_TRACE_EN
/**
 * @brief  读取最近一个完整10ms控制周期的插桩快照
 * @param  trace 输出快照，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_NO_RESOURCE
 */
platform_err_t track_ctrl_trace_get(track_trace_t *trace);
#endif

#endif /* SERVICE_TRACK_CTRL_H */
