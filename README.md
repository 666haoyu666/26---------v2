# 26 电赛·平台化循迹小车 V2

本仓库当前是一个空的 V2.5 平台化骨架：只有目录、占位文件和架构约束，尚未加入任何业务源码、CubeMX 工程、RTOS 内核或 Keil 工程。

## 目标调用链

```text
APP
  -> SERVICE
  -> Service/APP Platform Port
  -> BSP Wrapper
  -> Adapter 回调
  -> BSP Driver/Handler
  -> MCU Port
  -> Core/HAL
```

并行的操作系统链：

```text
APP/SERVICE/Middleware/Adapter
  -> OSAL Wrapper
  -> OS Implementation
  -> 03_Os/FreeRTOS
```

## 顶层目录

| 目录 | 职责 |
|---|---|
| `00_Docs` | 资源分配、架构、接口协议和模块设计 |
| `01_App` | 启动编排、组合根、用户流程和 ISR 桥接 |
| `01_Service` | 传感、循迹、运动等领域服务及其私有 Port |
| `02_Config` | 板级映射、功能裁剪、版本和 RTOS 参数 |
| `02_Platform/platform_bsp` | 器件无关 Wrapper 与具体绑定 Adapter |
| `02_Platform/platform_mcu` | GPIO/PWM/ENC/TIM/USART 等 MCU 抽象 |
| `02_Platform/platform_os` | 公共 OSAL、Wrapper 和 RTOS Implementation |
| `03_Bsp` | 具体器件 Driver 与可选 Handler |
| `03_Middlewares` | 产品管理的通用算法、通信与文件系统中间件 |
| `03_Os` | RTOS 内核与移植源码 |
| `04_Toolchain` | 链接脚本和工具链配置 |
| `99_Utils` | 与硬件、OS 和业务无关的纯工具 |
| `Core/Drivers/Middlewares` | CubeMX、HAL、CMSIS 和厂家生成/第三方底座 |
| `MDK-ARM` | Keil 工程与构建配置 |

## 第一条建议闭环

先建立“感知、决策、执行”三段式闭环：

```text
app_tracking_task
  -> service_sensor
       -> line_sensor_port
            -> bsp_wrapper_line_sensor
            -> bsp_adapter_port_line_sensor
            -> bsp_line_sensor_driver
            -> gpio_port
       -> imu_port
            -> 独立 IMU 能力链（器件与总线确定后补齐）
  -> service_tracking
  -> service_motion
```

`service_sensor` 只负责采集、校准、时间戳和有效性；`service_tracking` 负责循迹状态机与外环控制；`service_motion` 负责底盘运动与电机目标。契约先从上向下设计，代码再从 MCU Port 向上实现。每次只完成一条可编译、可测试、可真机验证的纵向能力链。

## 基本规则

- APP 优先只调用 SERVICE 公共 API。
- SERVICE 业务只通过自己的 Port 使用设备能力。
- Wrapper 不得包含 Adapter、Driver、HAL、RTOS 或业务头。
- Adapter 是器件、板卡、MCU、OSAL 和 Config 的唯一具体绑定点。
- Driver/Handler 只依赖注入接口，不直接依赖 HAL、FreeRTOS、APP 或 SERVICE。
- `Core/Src/main.c` 只负责 Cube 初始化、组合根入口和 OS 启动。
- 新增源码后必须同步 Keil Group、FilePath、IncludePath 与功能宏。
