/**
 * @file    wt_imu.h
 * @brief   维特 WT 系列 IMU 协议解析（纯函数，平台无关）
 * @note    - 零 HAL / 零状态：只解析调用方给的字节，可在 PC 单测
 *          - DMA 双缓冲与最新值缓存归适配层，本层不持状态
 *          - 懒扫描方案：解析在任务上下文调用，中断不解析
 */

#ifndef WT_IMU_H
#define WT_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform_type.h"

/* WT 系列每个输出周期按配置连发若干子包(各 11 字节)：
 *   0x55 0x52 …  角速度(Wx/Wy/Wz)
 *   0x55 0x53 …  角度(Roll/Pitch/Yaw)
 * 字节 8..9 为电压/版本号，不解析；其余 ID(如 0x51 加速度)
 * 校验通过则整帧跳过。parse 全缓冲扫描，一趟把各子包解完。 */
#define WT_IMU_FRAME_LEN    11U     /* 单个子包(帧)字节数，
                                    * 非一次传输长度 */
#define WT_IMU_FRAME_HEAD   0x55U   /* 子包帧头 */
#define WT_IMU_ID_GYRO      0x52U   /* 角速度子包 ID */
#define WT_IMU_ID_ANGLE     0x53U   /* 角度子包 ID */

/* 状态码：OK==0，适配层据此逐层映射 */
typedef enum {
    WT_IMU_OK        = 0,    /* 成功 */
    WT_IMU_ERR       = 1,    /* 通用错误 / 本段无有效帧 */
    WT_IMU_ERR_PARAM = 2,    /* 参数非法 */
    WT_IMU_ERR_INIT  = 3,    /* 未初始化 */
    WT_IMU_ERR_RES   = 4,    /* 资源不可用 / 暂无有效数据 */
    WT_IMU_ERR_TMO   = 5,    /* 操作超时（适配层映射 HAL_TIMEOUT） */
} wt_imu_status_t;

/* 一次解析的输出：本段里解出的最新角度 / 角速度 */
typedef struct {
    float   roll_deg;     /* 横滚角 X，单位 度（has_angle 为真时有效） */
    float   pitch_deg;    /* 俯仰角 Y，单位 度（has_angle 为真时有效） */
    float   yaw_deg;      /* 偏航角 Z，单位 度（has_angle 为真时有效） */

    float   gyro_x_dps;   /* 角速度 Wx，单位 度/秒（has_gyro 为真时有效） */
    float   gyro_y_dps;   /* 角速度 Wy，单位 度/秒（has_gyro 为真时有效） */
    float   gyro_z_dps;   /* 角速度 Wz，单位 度/秒（has_gyro 为真时有效） */

    uint8_t has_angle;    /* 本段是否解出角度帧 */
    uint8_t has_gyro;     /* 本段是否解出角速度帧 */
} wt_imu_sample_t;

/**
 * @brief  扫描一段原始字节，解出最新角度 / 角速度
 * @param  data 原始字节缓冲（DMA 收到的内容）
 * @param  len  有效字节数
 * @param  out  解析结果；同段多帧各量取最后一帧，未解出则对应 has_*=0
 * @note   找包头 0x55 → 校验 → 解码；纯函数、零平台依赖
 * @retval WT_IMU_OK / WT_IMU_ERR(无有效帧) / WT_IMU_ERR_PARAM
 */
wt_imu_status_t wt_imu_parse(const uint8_t *data, uint16_t len,
                             wt_imu_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WT_IMU_H */
