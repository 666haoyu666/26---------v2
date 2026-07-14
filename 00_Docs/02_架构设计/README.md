# 02_架构设计

记录分层、依赖方向、初始化顺序、资源所有权、生命周期和组合根设计。

## OSAL 架构

本产品固定使用 FreeRTOS，不设置可选 RTOS 后端。操作系统依赖仍必须收敛在：

```text
APP/SERVICE/Middleware/Adapter
  -> OSAL 公共接口
  -> OSAL Wrapper
  -> FreeRTOS Implementation
  -> 03_Os/FreeRTOS
```

Wrapper 负责参数和稳定语义；只有 FreeRTOS Implementation 可以包含原生
FreeRTOS/CMSIS 头、转换原生句柄、判断 ISR 上下文和调用 `xTask*`、
`xQueue*`、`xSemaphore*`、`xTimer*`。`03_Os` 只保存内核与移植源码。
