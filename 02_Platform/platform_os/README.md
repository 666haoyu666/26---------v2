# platform_os

本目录提供项目唯一的操作系统适配层。产品已经固定使用 FreeRTOS，因此不保留
RTOS 选择宏、后端注册表或 Zephyr 占位实现；稳定边界仍按以下调用链分层：

```text
APP / SERVICE / Middleware / Adapter
  -> inc/osal_*.h
  -> os_wrapper/src/osal_*.c
  -> os_wrapper/inc/os_internal.h
  -> os_implementation/freertos/src/os_freertos_*.c
  -> 03_Os/FreeRTOS
```

## 目录职责

- `inc`：公开 OSAL 状态、句柄和 task/mutex/queue/semaphore/timer/heap API。
- `os_wrapper`：参数校验、失败输出初始化和稳定语义，不包含 FreeRTOS 头。
- `os_implementation/freertos`：唯一实现，负责原生句柄转换、毫秒/Tick
  换算、线程/ISR 分支和 `portYIELD_FROM_ISR`。
- `03_Os`：由 CubeMX 或上游导入的 FreeRTOS 内核与移植源码，不属于 OSAL。

## 固定契约

- `osal_task_create()` 的 `stack_size` 单位为字节。
- `osal_task_delay()` 使用原生 Tick；其余 delay/period/timeout 使用毫秒。
- `OSAL_MAX_DELAY` 表示永久等待，FreeRTOS Implementation 将其映射为
  `portMAX_DELAY`。
- 队列、信号量、软件定时器的可用操作会自动选择线程态或 ISR 安全接口；
  ISR 分支忽略超时并在需要时触发 yield。
- mutex、heap、资源创建/删除、普通临界区和调度控制只允许在线程态调用。
- `osal_task_start()` 只供 `Core/main` 启动编排使用，业务模块不得启动调度器。
- 句柄是公开的不透明类型，上层不得解引用或转换为 FreeRTOS 原生句柄。
- Driver/Wrapper 禁止使用动态内存；`osal_heap_*` 仅供明确允许分配的上层模块。

## 构建要求

构建目标应加入全部 `os_wrapper/src/osal_*.c` 和
`os_implementation/freertos/src/os_freertos_*.c`，并暴露以下头文件路径：

```text
02_Platform/platform_os/inc
02_Platform/platform_os/os_wrapper/inc
02_Platform/platform_os/os_implementation/freertos/inc
```

只有最后一个私有路径需要访问 FreeRTOS、CMSIS 和 `FreeRTOSConfig.h`。
不得同时编译旧版 `02_OS_Platform` 或其他 `osal_*.c`，避免重复符号。
