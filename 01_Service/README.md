# 01_Service

负责传感、循迹、运动等领域能力。`service_diff_odom` 融合双轮编码器位移与 IMU 航向解算平面位姿（其私有 `platform/motor`、`platform/imu` 端口只读绑定 BSP Wrapper；odom 任务是全车唯一 IMU 读者，其他服务经 `server_odom_get` 取航向）。`service_tracking` 只负责循迹策略与闭环控制，`service_motion` 负责底盘运动。业务文件只依赖公共 API、私有 Port、OSAL、Config、Middleware 与 Utils。
