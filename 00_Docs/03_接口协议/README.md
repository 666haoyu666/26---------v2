# 03_接口协议

记录跨模块数据模型、单位、状态码、超时、线程/ISR 语义和失败后的输出约定。

## OSAL 公共语义

- OSAL API 返回 `osal_status_t`；参数、超时、内存不足、忙、上下文不支持和
  状态错误必须区分，调用方不得忽略可能失败的返回值。
- 任务创建的栈大小单位为字节；`osal_task_delay()` 单位为 Tick，其余超时、
  周期和延时单位为毫秒；`OSAL_MAX_DELAY` 表示永久等待。
- 队列、信号量和软件定时器的 ISR 安全操作由 Implementation 自动分流；
  ISR 中超时参数不生效。mutex、heap、创建/删除和调度控制禁止在 ISR 调用。
- 创建函数失败时输出句柄保持 `NULL`；查询函数失败时数值输出保持 `0`。
- 软件定时器回调运行在 FreeRTOS timer service task，禁止阻塞等待自身命令。
