/**
 * @file    app_tracking_task.c
 * @brief   A左轮速度内环阶跃调参场景
 * @note    APP只提交mm/s业务命令；RPS换算与槽位转发由Port完成
 */

#include "app_tracking_task.h"

#include "myprintf.h"
#include "osal_task.h"
#include "service_track_ctrl.h"

#define MOTOR_TUNE_TARGET_MM_S (400) /* 单阶跃目标速度，mm/s */
#define MOTOR_TUNE_SAMPLE_MS   (10U)   /* 控制快照采样周期，ms */
#define MOTOR_TUNE_EXPORT_MS   (20U)   /* 停车后CSV发送间隔，ms */
#define MOTOR_TUNE_SAMPLE_MAX  (400U)  /* 静态快照容量 */
#define MOTOR_TUNE_STEP_NUM    (3U)    /* 基线、阶跃、停车 */
#define MOTOR_TUNE_RETRY_MAX   (3U)    /* 单行发送最大尝试次数 */
#define MOTOR_TUNE_BUF_SIZE    (128U)  /* 单条CSV静态缓冲长度 */

/** A左轮单阶跃目标，最后一档通过STOP模式关闭输出。 */
static const int32_t s_tune_speed[MOTOR_TUNE_STEP_NUM] = {
    0, MOTOR_TUNE_TARGET_MM_S, 0
};

/** 各阶段10ms样本数：0.5s基线、2.5s响应、1s停车。 */
static const uint16_t s_hold_cnt[MOTOR_TUNE_STEP_NUM] = {
    50U, 250U, 100U
};

#if TRACK_TRACE_EN
static char     s_log_buf[MOTOR_TUNE_BUF_SIZE]; /* CSV发送缓冲 */
static uint32_t s_cap_tick[MOTOR_TUNE_SAMPLE_MAX]; /* 原始时间戳 */
static float    s_cap_cmd[MOTOR_TUNE_SAMPLE_MAX];  /* 左轮目标 */
static float    s_cap_act[MOTOR_TUNE_SAMPLE_MAX];  /* 左轮反馈 */
static int32_t  s_cap_duty[MOTOR_TUNE_SAMPLE_MAX]; /* 左轮PWM */
static uint32_t s_cap_flt[MOTOR_TUNE_SAMPLE_MAX];  /* 左轮故障 */
static int32_t  s_cap_st[MOTOR_TUNE_SAMPLE_MAX];   /* 快照读取状态 */
static uint32_t s_cap_cnt;                         /* 已采样数量 */

/**
 * @brief 将最近10ms控制快照保存到静态数组
 * @note  采样期间不发送串口，避免DMA队列影响控制响应
 */
static void tune_capture(void)
{
    track_trace_t trace = {0}; /* 最近完整控制快照 */
    platform_err_t err;        /* 快照读取结果 */

    if (s_cap_cnt >= MOTOR_TUNE_SAMPLE_MAX) {
        return;
    }
    err = track_ctrl_trace_get(&trace);

    s_cap_tick[s_cap_cnt] = trace.tick_ms;
    s_cap_cmd[s_cap_cnt] = trace.left_cmd_mm_s;
    s_cap_act[s_cap_cnt] = trace.left_act_mm_s;
    s_cap_duty[s_cap_cnt] = trace.left_duty;
    s_cap_flt[s_cap_cnt] = trace.left_fault;
    s_cap_st[s_cap_cnt] = (int32_t)err;
    s_cap_cnt++;
}

/**
 * @brief 停车后慢速导出完整快照
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_FAIL
 * @note  列为index,tick,cmd,act,duty,fault,trace_st,drop,fail
 */
static platform_err_t tune_export(void)
{
    platform_err_t err; /* 当前行发送结果 */
    uint32_t drop;      /* DMA队列累计丢帧数 */
    uint32_t fail = 0U; /* 本次导出累计重试数 */
    uint32_t idx;       /* 快照游标 */
    uint32_t retry;     /* 当前行发送尝试次数 */

    /* index=-1为导出开始，tick字段携带有效样本总数。 */
    drop = myprintf_get_drop();
    err = myprintf(s_log_buf, sizeof(s_log_buf),
                   "-1,%lu,0.00,0.00,0,0,0,%lu,0\r\n",
                   (unsigned long)s_cap_cnt,
                   (unsigned long)drop);
    if (PLATFORM_IS_ERR(err)) {
        fail++;
    }
    if (osal_task_delay_ms(MOTOR_TUNE_EXPORT_MS) != OSAL_OK) {
        return PLATFORM_ERR_FAIL;
    }

    for (idx = 0U; idx < s_cap_cnt; idx++) {
        err = PLATFORM_ERR_FAIL;
        for (retry = 0U; retry < MOTOR_TUNE_RETRY_MAX; retry++) {
            drop = myprintf_get_drop();
            err = myprintf(
                s_log_buf, sizeof(s_log_buf),
                "%ld,%lu,%.2f,%.2f,%ld,%lu,%ld,%lu,%lu\r\n",
                (long)idx,
                (unsigned long)s_cap_tick[idx],
                s_cap_cmd[idx],
                s_cap_act[idx],
                (long)s_cap_duty[idx],
                (unsigned long)s_cap_flt[idx],
                (long)s_cap_st[idx],
                (unsigned long)drop,
                (unsigned long)fail);
            if (PLATFORM_IS_OK(err)) {
                break;
            }
            fail++;
            if (osal_task_delay_ms(MOTOR_TUNE_EXPORT_MS) != OSAL_OK) {
                return PLATFORM_ERR_FAIL;
            }
        }
        if (PLATFORM_IS_ERR(err)) {
            /* 调试日志失败不影响已完成的安全停车与后续样本。 */
            continue;
        }
        if (osal_task_delay_ms(MOTOR_TUNE_EXPORT_MS) != OSAL_OK) {
            return PLATFORM_ERR_FAIL;
        }
    }

    /* index=-2为导出结束，tick字段再次携带样本总数。 */
    drop = myprintf_get_drop();
    err = myprintf(s_log_buf, sizeof(s_log_buf),
                   "-2,%lu,0.00,0.00,0,0,0,%lu,%lu\r\n",
                   (unsigned long)s_cap_cnt,
                   (unsigned long)drop,
                   (unsigned long)fail);
    if (PLATFORM_IS_ERR(err)) {
        /* 结束标记失败不应把调试场景升级为系统故障。 */
        return PLATFORM_ERR_OK;
    }

    return PLATFORM_ERR_OK;
}
#endif

platform_err_t app_motor_tune_run(void)
{
    platform_err_t err; /* 控制服务调用结果 */
    uint32_t step_idx;  /* 阶跃目标游标 */
    uint32_t sample_idx; /* 当前阶段样本游标 */

#if TRACK_TRACE_EN
    s_cap_cnt = 0U;
#endif

    for (step_idx = 0U; step_idx < MOTOR_TUNE_STEP_NUM; step_idx++) {
        if (step_idx == (MOTOR_TUNE_STEP_NUM - 1U)) {
            err = track_ctrl_set_mode(TRACK_CTRL_MODE_STOP, 0.0f, 0);
        } else {
            err = track_ctrl_set_mode(TRACK_CTRL_MODE_MOTOR_A, 0.0f,
                                      s_tune_speed[step_idx]);
        }
        if (PLATFORM_IS_ERR(err)) {
            (void)track_ctrl_set_mode(TRACK_CTRL_MODE_STOP, 0.0f, 0);
            return err;
        }

        for (sample_idx = 0U; sample_idx < s_hold_cnt[step_idx];
             sample_idx++) {
            if (osal_task_delay_ms(MOTOR_TUNE_SAMPLE_MS) != OSAL_OK) {
                (void)track_ctrl_set_mode(TRACK_CTRL_MODE_STOP, 0.0f, 0);
                return PLATFORM_ERR_FAIL;
            }
#if TRACK_TRACE_EN
            tune_capture();
#endif
        }
    }

    err = track_ctrl_set_mode(TRACK_CTRL_MODE_STOP, 0.0f, 0);
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

#if TRACK_TRACE_EN
    if (osal_task_delay_ms(200U) != OSAL_OK) {
        return PLATFORM_ERR_FAIL;
    }
    return tune_export();
#else
    return PLATFORM_ERR_OK;
#endif
}
