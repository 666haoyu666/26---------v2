/**
 * @file    board_motor_config.h
 * @brief   电机（A左/B右两轮）板级映射与默认参数
 * @note    - 定义必须严格来源于00_Docs/01_资源分配表
 *          - IO下标与io_port.c单路表对齐，PWM/编码器/定时器
 *            实例与各core port映射表对齐
 *          - 方向约定：IN1高+IN2低=顺时针；A顺时针=前进(左轮)，
 *            B逆时针=前进(右轮)，相对车头方向
 *          - 编码器A/B相极性与PID参数为测试工程沿用值，实测后回填
 */

#ifndef BOARD_MOTOR_CONFIG_H
#define BOARD_MOTOR_CONFIG_H

/* ---------------- 公共参数 ---------------- */

#define BOARD_MOTOR_TICK_TIMER (EN_CORE_TIMER_CTRL_10MS) /* 控制节拍 */
#define BOARD_MOTOR_TICK_S     (0.010f)  /* 节拍周期s，与TIM9一致 */
#define BOARD_MOTOR_CPR        (1061U)   /* 输出轴每圈计数(4倍频) */

/* PWM满幅不在此配置：Adapter初始化时经core_pwm_get_max() */
/* 运行期读取定时器ARR，跟随CubeMX配置自动适应 */

#define BOARD_MOTOR_PID_KP     (300.0f)  /* 速度环比例，待实测回填 */
#define BOARD_MOTOR_PID_KI     (50.0f)   /* 速度环积分，待实测回填 */
#define BOARD_MOTOR_PID_KD     (0.0f)    /* 速度环微分，待实测回填 */
#define BOARD_MOTOR_PID_ILIMIT (7500.0f) /* 积分限幅，待实测回填 */

/* ---------------- A电机（左轮） ---------------- */

#define BOARD_MOTOR_A_SLOT         (0U) /* Wrapper物理槽位 */
#define BOARD_MOTOR_A_PWM          (EN_CORE_PWM_MOTOR_A) /* PA9 */
#define BOARD_MOTOR_A_IO_IN1       (0U) /* io_port下标: PB12 AIN1 */
#define BOARD_MOTOR_A_IO_IN2       (1U) /* io_port下标: PB13 AIN2 */
#define BOARD_MOTOR_A_ENCODER      (EN_CORE_ENCODER_1) /* PB7/PB6 */
#define BOARD_MOTOR_A_FWD_CW       (0U) /* 1=正输出为顺时针(前进) */
#define BOARD_MOTOR_A_ENC_REVERSED (1U) /* 1=计数反相，待实测回填 */

/* ---------------- B电机（右轮） ---------------- */

#define BOARD_MOTOR_B_SLOT         (1U) /* Wrapper物理槽位 */
#define BOARD_MOTOR_B_PWM          (EN_CORE_PWM_MOTOR_B) /* PA8 */
#define BOARD_MOTOR_B_IO_IN1       (2U) /* io_port下标: PB14 BIN1 */
#define BOARD_MOTOR_B_IO_IN2       (3U) /* io_port下标: PB15 BIN2 */
#define BOARD_MOTOR_B_ENCODER      (EN_CORE_ENCODER_2) /* PB5/PB4 */
#define BOARD_MOTOR_B_FWD_CW       (1U) /* 1=正输出为顺时针(前进) */
#define BOARD_MOTOR_B_ENC_REVERSED (0U) /* 1=计数反相，待实测回填 */

#endif /* BOARD_MOTOR_CONFIG_H */
