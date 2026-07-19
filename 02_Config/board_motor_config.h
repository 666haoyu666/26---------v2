/**
 * @file    board_motor_config.h
 * @brief   双路编码器电机板级映射与闭环默认参数
 * @note    CPR按TIM3/TIM4实际计数填写，不等同于编码器铭牌脉冲数。
 */

#ifndef BOARD_MOTOR_CONFIG_H
#define BOARD_MOTOR_CONFIG_H

#define CFG_MOTOR_COUNT       (2U)     /* 电机数量 */
#define CFG_MOTOR_TICK_S      (0.010f) /* TIM9控制周期，s */
#define CFG_MOTOR_CPR         (20407U) /* 每输出轴转TIM计数，需实测 */
#define CFG_MOTOR_MAX_RPS     (50.0f)  /* 串口目标转速限幅 */

#define CFG_MOTOR_TICK_ID     (CORE_TIMER_CTRL_10MS) /* TIM9 */

#define CFG_MOTOR_A_SLOT      (0U)               /* A左轮槽位 */
#define CFG_MOTOR_A_PWM       (CORE_PWM_MOTOR_A) /* TIM1_CH2 */
#define CFG_MOTOR_A_IN1       (0U) /* io_port下标: PB12 AIN1 */
#define CFG_MOTOR_A_IN2       (1U) /* io_port下标: PB13 AIN2 */
#define CFG_MOTOR_A_ENC       (CORE_ENC_MOTOR_A) /* TIM4 */
#define CFG_MOTOR_A_CPR       (CFG_MOTOR_CPR)    /* A轮每转计数 */
#define CFG_MOTOR_A_DIA_MM    (65.0f)             /* A轮直径mm */
#define CFG_MOTOR_A_MM_REV    \
    (CFG_MOTOR_A_DIA_MM * 3.1415927f)             /* A轮每转行程 */
#define CFG_MOTOR_A_FILTER    (0.30f)              /* A轮滤波系数 */
#define CFG_MOTOR_A_PID_KP    (199.8f)               /* A轮比例增益 */
#define CFG_MOTOR_A_PID_KI    (50.0f)               /* A轮积分增益 */
#define CFG_MOTOR_A_PID_KD    (0.0f)               /* A轮微分增益 */
#define CFG_MOTOR_A_PID_KFF   (750.0f)               /* A轮前馈斜率 */
#define CFG_MOTOR_A_PID_FF_BIAS (265.0f)             /* A轮静摩擦补偿 */
#define CFG_MOTOR_A_PID_I_LIMIT (100.0f)          /* A轮积分限幅 */
#define CFG_MOTOR_A_FWD_CW    (1U)               /* 正转为顺时针 */
#define CFG_MOTOR_A_ENC_SIGN  (1)                /* 编码器方向修正 */

#define CFG_MOTOR_B_SLOT      (1U)               /* B右轮槽位 */
#define CFG_MOTOR_B_PWM       (CORE_PWM_MOTOR_B) /* TIM1_CH1 */
#define CFG_MOTOR_B_IN1       (2U) /* io_port下标: PB14 BIN1 */
#define CFG_MOTOR_B_IN2       (3U) /* io_port下标: PB15 BIN2 */
#define CFG_MOTOR_B_ENC       (CORE_ENC_MOTOR_B) /* TIM3 */
#define CFG_MOTOR_B_CPR       (CFG_MOTOR_CPR)    /* B轮每转计数 */
#define CFG_MOTOR_B_DIA_MM    (65.0f)             /* B轮直径mm */
#define CFG_MOTOR_B_MM_REV    \
    (CFG_MOTOR_B_DIA_MM * 3.1415927f)             /* B轮每转行程 */
#define CFG_MOTOR_B_FILTER    (0.30f)              /* B轮滤波系数 */
#define CFG_MOTOR_B_PID_KP    (200.0f)               /* B轮比例增益 */
#define CFG_MOTOR_B_PID_KI    (50.0f)               /* B轮积分增益 */
#define CFG_MOTOR_B_PID_KD    (0.0f)               /* B轮微分增益 */
#define CFG_MOTOR_B_PID_KFF   (720.0f)               /* B轮前馈斜率 */
#define CFG_MOTOR_B_PID_FF_BIAS (370.0f)             /* B轮静摩擦补偿 */
#define CFG_MOTOR_B_PID_I_LIMIT (100.0f)          /* B轮积分限幅 */
#define CFG_MOTOR_B_FWD_CW    (1U)               /* 整车前进为顺时针 */
#define CFG_MOTOR_B_ENC_SIGN  (-1)                /* 换向后反馈保持为正 */

#endif /* BOARD_MOTOR_CONFIG_H */
