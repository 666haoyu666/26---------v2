# myprintf 模块设计

## 接口

```c
platform_err_t myprintf(char *buf,
                        size_t buf_size,
                        const char *fmt,
                        ...);
```

- 调用方持有并提供格式化缓冲区，模块不分配动态内存，也不保存缓冲区。
- 使用 `vsnprintf()` 格式化；空指针、零容量返回 `PLATFORM_ERR_PARAM`。
- 格式化失败返回 `PLATFORM_ERR_FAIL`；缓冲区不足时返回
  `PLATFORM_ERR_NO_MEMORY`，且不发送截断内容。
- 空字符串视为成功，不发起串口传输。
- 成功格式化后通过 USART6 阻塞发送，发送超时为 1000 ms；HAL 状态按语义
  映射到公共错误码。

## 上下文与技术债

- 仅在线程态使用，禁止从 ISR 调用阻塞发送。
- 当前没有串口互斥或消息队列，多任务并发调用可能返回忙或产生竞争。
- USART6 当前也用于蓝牙串口，引入蓝牙协议后必须统一所有权和仲裁。
- 按临时需求，`99_Utils/common/myprintf.c` 直接依赖 Cube `usart.h` 和 HAL；
  后续应把发送能力迁移到 MCU USART Port，并向纯格式化工具注入发送接口。

## 验证

- Keil 构建验证头文件路径、C99 可变参数、`vsnprintf()` 和 HAL 链接。
- 真机检查正常格式、零长度、缓冲区不足以及 USART6 忙/超时返回值。
