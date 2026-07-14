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
