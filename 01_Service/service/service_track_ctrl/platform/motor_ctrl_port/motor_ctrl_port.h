/**
 * @file    motor_ctrl_port.h
 * @brief   循迹控制左右轮速度输出端口契约
 * @note    - 左右轮速度均使用mm/s，正值表示向前
 *          - 轮子速度到电机RPS的转换仅在Port实现中完成
 *          - 电机旋向、MCU实例与硬件绑定由Adapter隔离
 */

#ifndef MOTOR_CTRL_PORT_H
#define MOTOR_CTRL_PORT_H

#include "platform_error.h"

/** 左右轮槽位与速度单位转换配置。 */
typedef struct {
    uint32_t  left_id;       /* 左轮物理电机槽位 */
    uint32_t  right_id;      /* 右轮物理电机槽位 */
    float     left_mm_rev;   /* 左轮每转行程，mm/rev */
    float     right_mm_rev;  /* 右轮每转行程，mm/rev */
} motor_ctrl_cfg_t;

/** 一个控制周期的左右轮目标速度。 */
typedef struct {
    float  left_mm_s;   /* 左轮目标速度，正值向前 */
    float  right_mm_s;  /* 右轮目标速度，正值向前 */
} motor_ctrl_target_t;

/**
 * @brief  绑定左右轮槽位并探测电机能力
 * @param  cfg 绑定配置，不可为NULL；槽位须有效且不同，
 *             每转行程须为有限正数
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT / 底层电机错误
 * @note   电机注册和初始化须由组合根先行完成
 */
platform_err_t motor_ctrl_init(const motor_ctrl_cfg_t *cfg);

/**
 * @brief  解除左右轮槽位绑定
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED
 * @note   不反初始化底层电机设备
 */
platform_err_t motor_ctrl_deinit(void);

/**
 * @brief  使能左右轮速度闭环输出
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   Wrapper无单槽使能，作用于全部已注册电机槽位
 */
platform_err_t motor_ctrl_start(void);

/**
 * @brief  失能电机输出并进入滑行状态，尽力而为
 * @note   作用于全部已注册电机槽位；未初始化时静默返回
 */
void motor_ctrl_stop(void);

/**
 * @brief  设置左右轮目标速度并换算为电机转速下发
 * @param  left_mm_s  左轮目标速度，mm/s，须为有限值
 * @param  right_mm_s 右轮目标速度，mm/s，须为有限值
 * @note   未初始化或含非法值时整体忽略本次命令；
 *         两轮经校验后依次应用，中途失败不回滚已应用目标
 */
void motor_ctrl_set(float left_mm_s, float right_mm_s);

#endif /* MOTOR_CTRL_PORT_H */
