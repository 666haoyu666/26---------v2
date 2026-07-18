# 01_资源分配表

记录引脚、定时器、DMA、IRQ、总线和逻辑实例分配。板级 Config 与 MCU Port 映射只能来源于这里。

## 循迹灰度传感器

| 逻辑槽位 | MCU 引脚 | 模式 | 位图位 |
|---:|---|---|---:|
| 0 | PA0 | GPIO 输入 | bit0 |
| 1 | PA1 | GPIO 输入 | bit1 |
| 2 | PA2 | GPIO 输入 | bit2 |
| 3 | PA3 | GPIO 输入 | bit3 |
| 4 | PA4 | GPIO 输入 | bit4 |
| 5 | PA5 | GPIO 输入 | bit5 |
| 6 | PA6 | GPIO 输入 | bit6 |

当前为 7 路连续输入组，PA7 不属于循迹传感器资源。有效电平与槽位
几何坐标由 `02_Config/board_line_sensor_config.h` 配置。

## 电机（A 左轮 / B 右轮）

| 资源 | MCU 引脚/外设 | 逻辑实例 | 说明 |
|---|---|---|---|
| A 方向 IN1 / IN2 | PB12 / PB13 | `core_io` 0 / 1 | IN1 高 + IN2 低 = 顺时针 |
| B 方向 IN1 / IN2 | PB14 / PB15 | `core_io` 2 / 3 | 同上 |
| A PWM | TIM1_CH2 (PA9) | `EN_CORE_PWM_MOTOR_A` | ARR=9999，约 10kHz |
| B PWM | TIM1_CH1 (PA8) | `EN_CORE_PWM_MOTOR_B` | 同上 |
| A 编码器 | A=PB7(EXTI 上升沿), B=PB6(输入) | `EN_CORE_ENCODER_1` | A 相上升沿 1 倍频采 B 相 |
| B 编码器 | A=PB5(EXTI 上升沿), B=PB4(输入) | `EN_CORE_ENCODER_2` | 同上 |
| 控制节拍 | TIM9 更新中断 | `EN_CORE_TIMER_CTRL_10MS` | 10ms，NVIC 优先级 5 |
| 编码器中断 | EXTI9_5（仅 PB7/PB5） | —（encoder_port 持有） | NVIC 优先级 5 |

约束与约定：

- **EXTI 与 TIM9 必须同 NVIC 优先级(5)**：编码器计数 ISR 与 10ms
  控制 ISR 互不抢占，计数累加与读清零才无竞争。改动任一优先级
  必须同步评审 `encoder_port.c` 与 `bsp_adapter_port_motor.c`。
- `EXTI9_5_IRQHandler` 由 `encoder_port.c` 持有（.ioc 未勾选 EXTI
  NVIC），ISR 内直读直清 EXTI->PR 且只动 PB7/PB5 两条线，不经 HAL
  回调链。若在 CubeMX 勾选 NVIC 重新生成，会产生重复定义，必须
  删除其中一份；后续新增线 5..9 的 EXTI 需求要在此扩展路由。
- 编码器解码为 A 相上升沿 1 倍频（高 CPR 电机 40904 脉冲/转@4x，
  1x 后 10226 计数/转，中断量为 4x 的四分之一）。B 相在 A 上升沿
  时采样：B 低=+1、B 高=-1，极性由 `BOARD_MOTOR_x_ENC_REVERSED` 修正。
- 标签债务：main.h 中 PB6 宏名为 `PB6_E2B_Pin`，物理上是编码器 1
  （左轮 A 电机）的 B 相，.ioc 标签待纠正为 E1B。
- `HAL_TIM_PeriodElapsedCallback` 由 `timer_port.c` 唯一持有并统一
  分发（含 TIM11 HAL 时基喂 tick）。**CubeMX 重新生成 main.c 会再
  生成同名函数导致重复定义链接错误，必须删除 main.c 那份。**
- 本仓库 HAL 的 `HAL_TIM_Base_Start_IT` 在状态非 READY 时返回
  HAL_ERROR（运行期状态恒为 BUSY），重复启动必报错；MCU Port 层
  以自有 started 标志保证 start 幂等，禁止依赖 HAL 状态机容错。
- PWM 满幅由 Adapter 初始化时经 `core_pwm_get_max()` 运行期读取
  （=定时器 ARR），CubeMX 改 PWM 分辨率无需改配置。
- 全部 BSP Wrapper 对上错误码统一 `platform_err_t`：槽位/能力未
  注册报 NOT_INITIALIZED，重复注册报 ALREADY_INIT。
- 正方向约定（相对车头）：A 顺时针 = 前进，B 逆时针 = 前进。
- 编码器 A/B 相极性、CPR 与 PID 参数由
  `02_Config/board_motor_config.h` 配置，实测后回填。
- Wrapper 槽位：0 = A（左），1 = B（右）；左右轮底盘语义由
  Service 层维护，槽位本身不承载。
