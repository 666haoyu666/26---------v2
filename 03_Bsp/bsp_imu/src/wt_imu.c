/**
 * @file    wt_imu.c
 * @brief   维特 WT 系列 IMU 协议解析（纯函数，平台无关）
 * @note    - 全缓冲找包头 0x55，逐帧校验解码，兼容前导乱码
 *          - 同段多帧时各量取最后一帧（最新）；零状态、可单测
 *          - 角速度满量程按 WT9011G4K 取 4000，见 WT_IMU_GYRO_FS
 */

#include "wt_imu.h"

#include <stddef.h>     /* NULL */

/* WT9011G4K 角速度系数为 4000°/s；普通 WT 系列(2000°/s)改这里 */
#define WT_IMU_GYRO_FS   4000.0f   /* 角速度满量程，单位 度/秒 */
#define WT_IMU_ANGLE_FS   180.0f   /* 角度满量程，单位 度 */
#define WT_IMU_RAW_FS   32768.0f   /* int16 满量程 */

/* 帧内数据区偏移(低字节在前)；f[8..9] 电压/版本号不解析 */
#define OFS_X   2U      /* WxL / RollL */
#define OFS_Y   4U      /* WyL / PitchL */
#define OFS_Z   6U      /* WzL / YawL */

/**
 * @brief 校验一帧 11 字节子帧的校验和
 *        （前 10 字节累加 == 第 11 字节）
 */
static uint8_t csum_ok(const uint8_t *f)
{
    uint8_t s = 0U;     /* 累加和 */
    uint8_t i;          /* 字节下标 */

    for (i = 0U; i < (WT_IMU_FRAME_LEN - 1U); i++) {
        s = (uint8_t)(s + f[i]);
    }
    return (s == f[WT_IMU_FRAME_LEN - 1U]) ? 1U : 0U;
}

/** @brief 小端拼 int16：p[0] 低字节，p[1] 高字节 */
static int16_t le16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8U) | (uint16_t)p[0]);
}

/** @brief 原始 int16 按满量程 fs 换算为物理量 */
static float raw2phys(int16_t raw, float fs)
{
    return ((float)raw * fs) / WT_IMU_RAW_FS;
}

/** @brief 解码 0x52 角速度帧：Wx/Wy/Wz，单位 度/秒 */
static void decode_gyro(const uint8_t *f, wt_imu_sample_t *out)
{
    out->gyro_x_dps = raw2phys(le16(&f[OFS_X]), WT_IMU_GYRO_FS);
    out->gyro_y_dps = raw2phys(le16(&f[OFS_Y]), WT_IMU_GYRO_FS);
    out->gyro_z_dps = raw2phys(le16(&f[OFS_Z]), WT_IMU_GYRO_FS);
    out->has_gyro = 1U;
}

/** @brief 解码 0x53 角度帧：Roll/Pitch/Yaw，单位 度 */
static void decode_angle(const uint8_t *f, wt_imu_sample_t *out)
{
    out->roll_deg = raw2phys(le16(&f[OFS_X]), WT_IMU_ANGLE_FS);
    out->pitch_deg = raw2phys(le16(&f[OFS_Y]), WT_IMU_ANGLE_FS);
    out->yaw_deg = raw2phys(le16(&f[OFS_Z]), WT_IMU_ANGLE_FS);
    out->has_angle = 1U;
}

wt_imu_status_t wt_imu_parse(const uint8_t *data, uint16_t len,
                             wt_imu_sample_t *out)
{
    uint16_t i = 0U;            /* 扫描下标 */
    const uint8_t *f = NULL;    /* 当前候选帧首地址 */

    if ((data == NULL) || (out == NULL)) {
        return WT_IMU_ERR_PARAM;
    }
    *out = (wt_imu_sample_t){ 0 };      /* 各量与 has_* 全部清零 */

    while (i < len) {
        /* 找包头，乱码逐字节跳 */
        if (data[i] != WT_IMU_FRAME_HEAD) {
            i++;
            continue;
        }
        /* 尾部不足一帧，收尾 */
        if ((uint16_t)(len - i) < WT_IMU_FRAME_LEN) {
            break;
        }
        f = &data[i];
        /* 校验失败只前进 1 字节 */
        if (csum_ok(f) == 0U) {
            i++;
            continue;
        }
        /* 命中已知 ID 则解码；其余合法子包(如 0x51 加速度)跳过 */
        if (f[1] == WT_IMU_ID_GYRO) {
            decode_gyro(f, out);
        } else if (f[1] == WT_IMU_ID_ANGLE) {
            decode_angle(f, out);
        }
        i = (uint16_t)(i + WT_IMU_FRAME_LEN);   /* 整帧消费 */
    }

    return ((out->has_angle != 0U) || (out->has_gyro != 0U)) ?
           WT_IMU_OK : WT_IMU_ERR;
}
