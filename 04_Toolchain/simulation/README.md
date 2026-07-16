# 直线循迹仿真

`straight_line_sim.py` 从当前固件源码读取控制周期、速度、轮距、探头到
后轴距离、七路探头坐标和外环增益，然后仿真以下闭环：

```text
七路二值探头 + IMU yaw
  -> 固件同款误差低通与后轴换算
  -> 目标角速度
  -> 左右轮目标速度
  -> 可配置电机延迟/惯性/左右增益/限速
  -> 差速车运动学
```

## 运行

```powershell
python 04_Toolchain/simulation/straight_line_sim.py
```

输出到 `04_Toolchain/simulation/output`：

- `straight_line_report.json`：参数快照、假设和各场景控制指标；
- `straight_line_timeseries.csv`：每个控制周期的状态和控制量；
- `straight_line_response.svg`：轨迹、后轴误差、yaw 和角速度对比图。

直接双击 `straight_line_simulator.html` 可打开交互网页。网页以俯视动画展示
黑线、车体、左右驱动轮、前置七路探头、吃线状态和历史轨迹，并提供播放、
暂停、重置、时间轴和倍速控制。初始状态、误差方向、电机惯性、纯延迟、
左轮残余增益、K1 与 K2 均可调节。

## 默认场景

- 后轴横向偏移 30 mm，理想/标称/劣化电机三组对照；
- 前探头居中但车身斜 10°；
- Port 契约符号与当前 `track_port.c` 原样透传符号对照。
- 保持 K2=1.8，对 K1=0.25/0.30/0.35/0.40 做候选整定对照。

黑线有效宽度暂按 20 mm。标称电机使用 60 ms 一阶惯性和 10 ms 延迟；
劣化电机使用 180 ms 惯性、40 ms 延迟、左轮 8% 增益损失和 350 mm/s
限速。这些是诊断用假设，不代表实车测量结果。要判断真实电机，应把轮速阶跃
日志拟合得到的时间常数、纯延迟、左右增益和最大轮速回填到脚本场景中。

`port_sign=-1` 表示 Wrapper 的“线相对探头”转换成 Port 的“车体相对线”；
`port_sign=+1` 表示当前 `track_port.c` 将 `raw.offset_mm` 原样上送。该对照用于
检查实车接线方向与源码注释是否一致。
