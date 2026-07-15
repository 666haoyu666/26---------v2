/**
 * @file    bsp_adapter_port_imu.c
 * @brief   IMU Adapter：绑定WT驱动、逻辑串口与DMA乒乓缓冲(懒扫描)
 * @note    - 唯一知道器件型号、串口绑定和缓冲策略的位置
 *          - ISR入口经core_usart_reg_isr注册，由usart Port分发
 *          - rx_isr只记长度+切缓冲+重启接收，解析在read(任务态)
 *          - read要求同段解出角度+角速度两种帧，否则视为暂无数据
 *          - 无OS依赖：ISR与read经缓冲下标复核检测覆盖，无锁
 *          - 调试开启时ISR只记计数，日志统一在任务态输出
 */

#include "bsp_adapter_port_imu.h"

#include "board_imu_config.h"
#include "platform_def.h"
#include "usart_port.h"
#include "wt_imu.h"

#ifndef IMU_DEBUG
#define IMU_DEBUG 0U /* 1=开启USART6插桩日志，0=完全关闭 */
#endif

#if IMU_DEBUG
#include "myprintf.h"
#define IMU_LOG_LEN 192U /* 插桩日志格式化缓冲字节数 */
static char s_log_buf[IMU_LOG_LEN];
#define IMU_LOG(...)                                                   \
    do {                                                               \
        (void)myprintf(s_log_buf, sizeof(s_log_buf),                    \
                       "[imu_adp] " __VA_ARGS__);                       \
    } while (0)
#else
#define IMU_LOG(...) do { } while (0)
#endif

#define IMU_RX_LEN 64U /* 单块DMA接收缓冲字节数 */

/** DMA乒乓双缓冲，ISR切换、read懒解析。 */
static uint8_t s_buf[2][IMU_RX_LEN];

/** 每块最近收完的有效长度（ISR写/任务读）。 */
static volatile uint16_t s_len[2] = { 0U, 0U };

/** DMA正写入的缓冲下标（ISR写/任务读）。 */
static volatile uint8_t s_active = 0U;

/** 1=接收已启动，read/isr可工作。 */
static volatile uint8_t s_started = 0U;

#if IMU_DEBUG
/** Adapter接收事件入口累计次数，仅用于任务态诊断。 */
static volatile uint32_t s_rx_cnt = 0U;

/** Adapter错误入口累计次数，仅用于任务态诊断。 */
static volatile uint32_t s_err_cnt = 0U;

/** DMA启动/重启失败累计次数。 */
static volatile uint32_t s_arm_fail = 0U;

/** 最近一次接收事件上报的字节数。 */
static volatile uint16_t s_last_size = 0U;

/** 最近一次DMA启动/重启结果。 */
static volatile platform_err_t s_last_arm = PLATFORM_ERR_OK;
#endif

static platform_err_t arm_rx(uint8_t idx);
static platform_err_t imu_adp_init(void);
static platform_err_t imu_adp_deinit(void);
static platform_err_t imu_adp_read(bsp_imu_data_t *data);
static platform_err_t imu_rx_isr(uint16_t size);
static platform_err_t imu_err_isr(void);
#if IMU_DEBUG
static void imu_dbg_log(uint8_t finished, uint16_t len,
                        wt_imu_status_t parse_st,
                        const wt_imu_sample_t *smp,
                        platform_err_t result);
#endif

#if IMU_DEBUG
/**
 * @brief  在线程态输出一次接收链路快照
 * @param  finished 最近完成缓冲下标
 * @param  len      最近完成缓冲有效长度
 * @param  parse_st WT协议解析结果
 * @param  smp      WT协议解析输出
 * @param  result   本次Adapter读取结果
 */
static void imu_dbg_log(uint8_t finished, uint16_t len,
                        wt_imu_status_t parse_st,
                        const wt_imu_sample_t *smp,
                        platform_err_t result)
{
    uint32_t rx_cnt;   /* 接收事件计数快照 */
    uint32_t err_cnt;  /* UART错误计数快照 */
    uint32_t arm_fail; /* DMA重启失败计数快照 */
    uint16_t last_size; /* 最近接收长度快照 */

    rx_cnt = s_rx_cnt;
    err_cnt = s_err_cnt;
    arm_fail = s_arm_fail;
    last_size = s_last_size;

    IMU_LOG("read=%d rx=%lu err=%lu arm_fail=%lu size=%u "
            "act=%u len=%u/%u arm=%d parse=%d ag=%u/%u "
            "buf%u[%u]=%02X %02X %02X %02X\n",
            (int)result, (unsigned long)rx_cnt,
            (unsigned long)err_cnt, (unsigned long)arm_fail,
            (unsigned int)last_size, (unsigned int)s_active,
            (unsigned int)s_len[0], (unsigned int)s_len[1],
            (int)s_last_arm, (int)parse_st,
            (unsigned int)smp->has_angle,
            (unsigned int)smp->has_gyro,
            (unsigned int)finished,
            (unsigned int)len,
            (unsigned int)s_buf[finished][0],
            (unsigned int)s_buf[finished][1],
            (unsigned int)s_buf[finished][2],
            (unsigned int)s_buf[finished][3]);
}
#endif

/**
 * @brief  在指定缓冲上重启空闲分段DMA接收
 * @param  idx 目标缓冲下标，0/1
 * @retval PLATFORM_ERR_OK / 底层错误码
 */
static platform_err_t arm_rx(uint8_t idx)
{
    platform_err_t err; /* 底层启动结果 */

    err = core_usart_rx_start(BOARD_IMU_USART, s_buf[idx],
                              IMU_RX_LEN);
#if IMU_DEBUG
    s_last_arm = err;
    if (PLATFORM_IS_ERR(err)) {
        s_arm_fail++;
    }
#endif
    if (PLATFORM_IS_OK(err)) {
        s_active = idx; /* 标记DMA当前写入块 */
    }
    return err;
}

/**
 * @brief  从干净状态启动DMA接收（Wrapper pf_init）
 * @retval PLATFORM_ERR_OK / 底层错误码
 */
static platform_err_t imu_adp_init(void)
{
    platform_err_t err; /* 启动结果 */

    s_len[0] = 0U;
    s_len[1] = 0U;
#if IMU_DEBUG
    s_rx_cnt = 0U;
    s_err_cnt = 0U;
    s_arm_fail = 0U;
    s_last_size = 0U;
    s_last_arm = PLATFORM_ERR_OK;
#endif
    err = arm_rx(0U);
    if (PLATFORM_IS_ERR(err)) {
        IMU_LOG("init arm fail=%d\n", (int)err);
        return err;
    }
    s_started = 1U;
    IMU_LOG("init ok, rx_len=%u\n", (unsigned int)IMU_RX_LEN);
    return PLATFORM_ERR_OK;
}

/**
 * @brief  停止DMA接收（Wrapper pf_deinit）
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_FAIL
 */
static platform_err_t imu_adp_deinit(void)
{
    platform_err_t err; /* 停止结果 */

    err = core_usart_rx_stop(BOARD_IMU_USART);
    if (PLATFORM_IS_ERR(err)) {
        IMU_LOG("deinit fail=%d\n", (int)err);
        return err;
    }
    s_started = 0U;
    IMU_LOG("deinit ok\n");
    return PLATFORM_ERR_OK;
}

/**
 * @brief  懒解析最近收完的缓冲并输出姿态/角速度
 * @param  data 输出结果，失败时保持原值
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / PLATFORM_ERR_NO_RESOURCE
 */
static platform_err_t imu_adp_read(bsp_imu_data_t *data)
{
    wt_imu_sample_t smp;      /* 本次解析结果 */
    wt_imu_status_t parse_st; /* WT协议解析结果 */
    platform_err_t  result;   /* 本次Adapter读取结果 */
    uint8_t          finished; /* 最近收完的缓冲下标 */
    uint16_t         len;     /* 该缓冲有效长度 */

    if (data == NULL) {
        IMU_LOG("read param null\n");
        return PLATFORM_ERR_PARAM;
    }
    if (s_started == 0U) {
        IMU_LOG("read before init\n");
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* DMA正写s_active，最近收完的是另一块 */
    finished = (uint8_t)(s_active ^ 1U);
    len = s_len[finished];
    if (len > IMU_RX_LEN) {
        len = IMU_RX_LEN;
    }
    parse_st = wt_imu_parse(s_buf[finished], len, &smp);
    result = PLATFORM_ERR_NO_RESOURCE;

    if (parse_st == WT_IMU_OK) {
        /* 解析期间若该块被DMA重新占用，则丢弃本次结果。 */
        if ((uint8_t)(s_active ^ 1U) != finished) {
            result = PLATFORM_ERR_NO_RESOURCE;
        } else if ((smp.has_angle == 0U) ||
                   (smp.has_gyro == 0U)) {
            result = PLATFORM_ERR_NO_RESOURCE;
        } else {
            data->roll_deg   = smp.roll_deg;
            data->pitch_deg  = smp.pitch_deg;
            data->yaw_deg    = smp.yaw_deg;
            data->gyro_x_dps = smp.gyro_x_dps;
            data->gyro_y_dps = smp.gyro_y_dps;
            data->gyro_z_dps = smp.gyro_z_dps;
            result = PLATFORM_ERR_OK;
        }
    }

#if IMU_DEBUG
    imu_dbg_log(finished, len, parse_st, &smp, result);
#endif
    return result;
}

/**
 * @brief  UART接收事件ISR入口（空闲/整包完成）
 * @param  size 本段接收字节数
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层重启失败错误码
 * @note   仅记长度+切缓冲+重启接收，不解析（最短中断）
 */
static platform_err_t imu_rx_isr(uint16_t size)
{
    uint8_t finished; /* 刚收完的缓冲下标 */

#if IMU_DEBUG
    s_rx_cnt++;
    s_last_size = size;
#endif
    if (s_started == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    /* 当前DMA写的就是刚收完的块，记录其有效长度 */
    finished = s_active;
    s_len[finished] = (size <= IMU_RX_LEN) ? size : IMU_RX_LEN;
    /* 切另一块继续收，形成乒乓（arm_rx内更新s_active） */
    return arm_rx((uint8_t)(finished ^ 1U));
}

/**
 * @brief  UART错误ISR入口：重启接收恢复
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层重启失败错误码
 */
static platform_err_t imu_err_isr(void)
{
    platform_err_t err; /* 重启结果 */

#if IMU_DEBUG
    s_err_cnt++;
#endif
    if (s_started == 0U) {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    /* 先原地重启；异常状态下先终止会话再重启一次 */
    err = arm_rx(s_active);
    if (PLATFORM_IS_ERR(err)) {
        (void)core_usart_rx_stop(BOARD_IMU_USART);
        err = arm_rx(s_active);
    }
    return err;
}

platform_err_t bsp_imu_adp_reg(void)
{
    bsp_imu_dev_t  dev; /* 待注册的能力表 */
    platform_err_t err; /* ISR注册结果 */

    /* 先挂ISR分发（此时未启动接收，回调不会触发） */
    err = core_usart_reg_isr(BOARD_IMU_USART,
                             imu_rx_isr, imu_err_isr);
    if (PLATFORM_IS_ERR(err)) {
        IMU_LOG("isr reg fail=%d\n", (int)err);
        return err;
    }
    IMU_LOG("isr reg ok\n");

    dev.pf_init   = imu_adp_init;
    dev.pf_deinit = imu_adp_deinit;
    dev.pf_read   = imu_adp_read;

    err = bsp_imu_reg(&dev);
    if (PLATFORM_IS_ERR(err)) {
        IMU_LOG("wrapper reg fail=%d\n", (int)err);
        return err;
    }
    IMU_LOG("wrapper reg ok\n");
    return PLATFORM_ERR_OK;
}
