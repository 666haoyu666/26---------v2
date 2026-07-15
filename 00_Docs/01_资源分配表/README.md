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
| A PWM | TIM1_CH2 (PA9) | `EN_CORE_PWM_MOTOR_A` | ARR=10000，约 5kHz |
| B PWM | TIM1_CH1 (PA8) | `EN_CORE_PWM_MOTOR_B` | 同上 |
| A 编码器 | E1A=PB7, E1B=PB6 | `EN_CORE_ENCODER_1` | EXTI 双边沿 4 倍频 |
| B 编码器 | E2A=PB5, E2B=PB4 | `EN_CORE_ENCODER_2` | 同上 |
| 控制节拍 | TIM9 更新中断 | `EN_CORE_TIMER_CTRL_10MS` | 10ms，NVIC 优先级 5 |
| 编码器中断 | EXTI9_5 / EXTI4 | —（encoder_port 持有） | NVIC 优先级 5 |

约束与约定：

- **EXTI 与 TIM9 必须同 NVIC 优先级(5)**：编码器计数 ISR 与 10ms
  控制 ISR 互不抢占，计数累加与读清零才无竞争。改动任一优先级
  必须同步评审 `encoder_port.c` 与 `bsp_adapter_port_motor.c`。
- EXTI4/EXTI9_5 的 IRQHandler 与 `HAL_GPIO_EXTI_Callback` 由
  `encoder_port.c` 持有（.ioc 未勾选 EXTI NVIC）。若在 CubeMX 勾选
  NVIC 重新生成，会产生重复定义，必须删除其中一份。
- 正方向约定（相对车头）：A 顺时针 = 前进，B 逆时针 = 前进。
- 编码器 A/B 相极性、CPR 与 PID 参数由
  `02_Config/board_motor_config.h` 配置，实测后回填。
- Wrapper 槽位：0 = A（左），1 = B（右）；左右轮底盘语义由
  Service 层维护，槽位本身不承载。
