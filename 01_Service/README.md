# 01_Service

负责传感、循迹、运动等领域能力。`service_sensor` 统一组织循迹传感器和 IMU 的采集 Port，`service_tracking` 只负责循迹策略与闭环控制，`service_motion` 负责底盘运动。业务文件只依赖公共 API、私有 Port、OSAL、Config、Middleware 与 Utils。
