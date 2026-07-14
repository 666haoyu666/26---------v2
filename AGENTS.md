# AGENTS.md

## 项目状态

本工程是 26 电赛循迹小车的 V2.5 平台化空骨架。空目录与占位文件是预期状态；没有明确需求时不要提前生成大批接口或实现。

## 架构基线

唯一主业务链：

```text
01_App
  -> 01_Service
  -> Service/APP Port
  -> 02_Platform/platform_bsp/.../bsp_wrapper_*
  -> 已注册的 Adapter 回调
  -> 03_Bsp Driver/Handler
  -> 02_Platform/platform_mcu
  -> Core/HAL
```

禁止创建旧版 `05_Server`、`02_BSP_Platform`、`02_MCU_Platform`、`02_OS_Platform`、`03_Config` 或 `04_Common_Utils`。

## 边界规则

- APP/SERVICE 不包含 `main.h`、Cube 外设头、STM32 HAL 或 FreeRTOS 原生头。
- SERVICE 业务只包含自己的公共头和私有 Port 头；Wrapper 只允许出现在 Port 的 `.c` 中。
- Wrapper 只定义稳定能力、注册/选择和安全转发，不知道具体 Driver 或 Adapter。
- Adapter 负责依赖注入、实例装配、板级绑定、单位和状态码转换。
- Driver/Handler 不动态分配内存，不包含 HAL、FreeRTOS、APP、SERVICE 或 Wrapper。
- MCU 公共头不得泄漏 HAL 句柄、寄存器或 RTOS 实现类型。
- 只有 OS Implementation 可以直接包含和调用 FreeRTOS 原生接口。
- `02_Config` 不保存运行期状态或创建资源；`99_Utils` 只放纯工具。

## 实施顺序

1. 先冻结业务语义、Port 与 Wrapper 契约。
2. 从 MCU Port、Driver、Adapter、Wrapper、Port、SERVICE 到 APP 自底向上实现。
3. Adapter 在 Wrapper 第一次调用前由组合根集中注册。
4. 需要 OS 对象的模块拆分为静态 `register/mount` 与资源就绪后的 `start`。
5. 每个能力同时闭环源码、注册、资源、ISR、Keil 构建、替身测试和真机验证。

修改前先读根 `README.md` 与 `00_Docs` 中相关文档。

