# myprintf 模块设计

## 接口

```c
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...);
```

- 调用方持有并提供格式化缓冲区；格式化后复制到模块静态队列，调用方可
  立即复用原缓冲区。
- 使用 `vsnprintf()` 格式化；空指针、零容量返回 `PLATFORM_ERR_PARAM`。
- 格式化失败返回 `PLATFORM_ERR_FAIL`；缓冲区不足时返回
  `PLATFORM_ERR_NO_MEMORY`，且不发送截断内容。
- 空字符串视为成功，不发起串口传输。
- 成功格式化后排入 8 槽静态队列，每槽最多 256 字节，通过 USART6 TX DMA
  异步发送；`PLATFORM_ERR_OK` 表示已入队。
- DMA完成回调推进队列；队列满返回 `PLATFORM_ERR_BUSY`，并累计丢帧数。

## 上下文与技术债

- 仅在线程态调用 `myprintf()`；DMA完成/错误入口在ISR内只推进静态队列。
- 入队和回调使用短关中断临界区，多任务调用不会交叉覆盖消息。
- USART6 当前也用于蓝牙串口，引入蓝牙协议后必须统一所有权和仲裁。
- 按临时需求，`99_Utils/common/myprintf.c` 直接依赖 Cube `usart.h` 和 HAL；
  后续应把发送能力迁移到 MCU USART Port，并向纯格式化工具注入发送接口。

## 验证

- Keil 构建验证头文件路径、C99 可变参数、`vsnprintf()` 和 HAL 链接。
- 真机检查正常格式、零长度、缓冲区不足、队列满、连续DMA发送和错误恢复。
- USART6 TX DMA的Normal模式完成链还需要使能USART6全局中断，不能只使能
  DMA2 Stream6中断。
