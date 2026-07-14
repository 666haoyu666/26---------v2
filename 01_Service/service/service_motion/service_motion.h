/**
 * @file    service_motion.h
 * @brief   差速小车运动领域服务公共接口
 * @note    同步单消费者；轮速单位mm/s，正值前进，负值后退
 */

#ifndef SERVICE_MOTION_H
#define SERVICE_MOTION_H

/** 运动服务调用状态。 */
typedef enum {
    MOTION_OK           = 0,    /* 操作成功 */
    MOTION_ERR_PARAM    = 1,    /* 输入参数非法或目标超限 */
    MOTION_ERR_STATE    = 2,    /* 服务生命周期状态错误 */
    MOTION_ERR_ACTUATOR = 3,    /* 电机执行或反馈链路不可用 */
    MOTION_ERR_RESERVED = 0xFF  /* 保留值 */
} motion_status_t;

/** 差速底盘当前运行状态。 */
typedef enum {
    MOTION_RUN_STOPPED = 0, /* 驱动已停止 */
    MOTION_RUN_ACTIVE  = 1, /* 驱动已使能 */
    MOTION_RUN_FAULT   = 2  /* 执行或反馈链路故障 */
} motion_run_t;

/** 同一控制时刻的左右轮线速度目标。 */
typedef struct {
    float  left_mm_s;    /* 左轮线速度，正值表示小车前进 */
    float  right_mm_s;   /* 右轮线速度，正值表示小车前进 */
} motion_cmd_t;

/** 左右轮目标、反馈与运行状态的一致快照。 */
typedef struct {
    motion_cmd_t  target;      /* 当前已接受的轮速目标 */
    motion_cmd_t  feedback;    /* 当前测得的实际轮速 */
    motion_run_t  run_state;   /* 当前驱动运行状态 */
} motion_state_t;

/**
 * @brief  初始化运动服务及其私有电机端口
 * @retval MOTION_OK / MOTION_ERR_STATE / MOTION_ERR_ACTUATOR
 * @note   初始化成功后处于STOPPED；重复初始化返回MOTION_ERR_STATE
 */
motion_status_t motion_init(void);

/**
 * @brief  停止驱动并注销运动服务
 * @retval MOTION_OK / MOTION_ERR_STATE / MOTION_ERR_ACTUATOR
 * @note   注销失败时保持已初始化，调用方可再次停止或注销
 */
motion_status_t motion_deinit(void);

/**
 * @brief  同次提交左右轮线速度目标并按需使能驱动
 * @param  cmd 目标指针，不可为NULL；两项必须为有限且未超限的值
 * @retval MOTION_OK / MOTION_ERR_PARAM / MOTION_ERR_STATE /
 *         MOTION_ERR_ACTUATOR
 * @note   任一目标非法时整组拒绝；两项均为0.0f时等价于motion_stop
 */
motion_status_t motion_set_speed(const motion_cmd_t *cmd);

/**
 * @brief  撤销左右轮驱动并进入STOPPED状态
 * @retval MOTION_OK / MOTION_ERR_STATE / MOTION_ERR_ACTUATOR
 * @note   保留服务资源和轮速反馈；不承诺机械立即静止
 */
motion_status_t motion_stop(void);

/**
 * @brief  读取左右轮目标、反馈与运行状态的一致快照
 * @param  state 输出状态，不可为NULL；调用失败时保持原值
 * @retval MOTION_OK / MOTION_ERR_PARAM / MOTION_ERR_STATE /
 *         MOTION_ERR_ACTUATOR
 */
motion_status_t motion_get_state(motion_state_t *state);

#endif /* SERVICE_MOTION_H */
