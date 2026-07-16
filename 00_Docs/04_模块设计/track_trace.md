# 循迹控制同拍插桩

## 开关与采样

- `TRACK_TRACE_EN=1`：控制任务每10 ms发布一份完整快照，默认任务每100 ms
  读取最近快照并通过USART6 DMA发送。
- `TRACK_TRACE_EN=0`：快照与CSV输出不编译；DMA版 `myprintf` 保留可供其他
  调试日志使用。
- 快照采用双缓冲发布，CSV中的全部控制量来自同一个10 ms控制周期。
- 第一列时间戳在控制任务内直接读取HAL `uwTick`，单位为ms。

## CSV列

| 列 | 字段 | 含义 |
|---:|---|---|
| 1 | `tick_ms` | 同拍 `uwTick` 时间戳 |
| 2 | `cycle` | 10 ms控制周期递增序号 |
| 3 | `mode` | `track_ctrl_mode_t` |
| 4 | `line_state` | 0无线路，1有效，2歧义 |
| 5 | `port_st` | 循迹Port读取状态码 |
| 6 | `imu_st` | 里程计快照读取状态码 |
| 7 | `motor_st` | 电机反馈读取状态码 |
| 8-9 | `x_mm,y_mm` | 同拍里程计位置 |
| 10-11 | `yaw,yaw_rate` | 航向角与角速度 |
| 12 | `raw_err` | 七路二值探头原始质心误差 |
| 13 | `filt_err` | 控制器实际使用的低通误差 |
| 14-15 | `yaw_tgt,yaw_err` | 目标航向与归一化航向误差 |
| 16-17 | `w_cmd,v_cmd` | 车体角速度与前向速度命令 |
| 18-19 | `left_cmd,right_cmd` | 本拍左右轮目标速度 |
| 20-21 | `left_act,right_act` | 左右轮最近反馈速度 |
| 22-23 | `left_fault,right_fault` | 左右轮故障位 |
| 24 | `tx_drop` | DMA队列累计丢帧数 |
| 25 | `log_fail` | CSV入队调用累计失败数 |

正常情况下相邻CSV行的 `cycle` 约增加10；若增加量明显更大，同时
`tx_drop`或`log_fail`增长，说明蓝牙链路吞吐不足或DMA队列发生丢帧。

## DMA完成链

USART6 TX DMA使用Normal模式，完成路径为：

```text
DMA2_Stream6_IRQHandler
  -> HAL_DMA_IRQHandler
  -> HAL内部开启USART TC中断
  -> USART6_IRQHandler
  -> HAL_UART_IRQHandler
  -> HAL_UART_TxCpltCallback
  -> myprintf发送下一条队列消息
```

因此必须同时开启DMA2 Stream6和USART6全局中断。
