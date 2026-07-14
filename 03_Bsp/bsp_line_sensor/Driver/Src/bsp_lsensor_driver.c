/**
 * @file    bsp_lsensor_driver.c
 * @brief   八路二值灰度driver核心：注入取数、归一与簇分析
 * @note    - 单实例持只读cfg副本，无跨拍数据状态
 *          - 取数只走init注入的get_map，不触碰任何平台接口
 *          - 判定规则与几何规格见《灰度传感器模块设计》4.3
 */

#include "bsp_lsensor_driver.h"

/** 单实例驱动状态。 */
static struct {
    lsensor_drv_cfg_t cfg;   /* init拷贝的只读配置副本 */
    uint8_t           ready; /* 1=已完成init */
} s_drv;

static void calc_result(uint8_t line_map, lsensor_drv_result_t *res);

/**
 * @brief  簇分析纯计算内核
 * @param  line_map 已归一的压线位图，禁用槽位逐位过滤
 * @param  res      输出结果，调用方保证非NULL
 */
static void calc_result(uint8_t line_map, lsensor_drv_result_t *res)
{
    float   sum_mm  = 0.0f; /* 命中槽位坐标累加 */
    uint8_t hit_cnt = 0U;   /* 命中槽位数 */
    uint8_t run_cnt = 0U;   /* 连续命中段数 */
    uint8_t en_cnt  = 0U;   /* 启用槽位数 */
    uint8_t in_run  = 0U;   /* 1=正处于命中段内 */
    uint8_t i;              /* 槽位游标 */

    for (i = 0U; i < LSENSOR_DRV_SLOT_MAX; i++) {
        uint8_t bit = (uint8_t)(1U << i); /* 当前槽位掩码 */

        if ((s_drv.cfg.enabled_mask & bit) == 0U) {
            in_run = 0U; /* 禁用槽位视作连续性断点 */
            continue;
        }
        en_cnt++;
        if ((line_map & bit) != 0U) {
            if (in_run == 0U) {
                run_cnt++;
            }
            in_run = 1U;
            hit_cnt++;
            sum_mm += s_drv.cfg.positions_mm[i];
        } else {
            in_run = 0U;
        }
    }

    if (hit_cnt == 0U) {
        res->track     = LSENSOR_DRV_NO_LINE;
        res->offset_mm = 0.0f;
    } else if ((run_cnt >= 2U) || (hit_cnt == en_cnt)) {
        res->track     = LSENSOR_DRV_AMBIGUOUS; /* 多簇或全路触发 */
        res->offset_mm = 0.0f;
    } else {
        res->track     = LSENSOR_DRV_TRACKING;
        res->offset_mm = sum_mm / (float)hit_cnt;
    }
}

lsensor_drv_status_t lsensor_drv_init(const lsensor_drv_cfg_t *cfg)
{
    float   prev_mm  = 0.0f; /* 上一启用槽位坐标 */
    uint8_t has_prev = 0U;   /* 1=已出现启用槽位 */
    uint8_t i;               /* 槽位游标 */

    if ((cfg == NULL) || (cfg->get_map == NULL) ||
        (cfg->enabled_mask == 0U)) {
        return LSENSOR_DRV_ERR_PARAM;
    }

    for (i = 0U; i < LSENSOR_DRV_SLOT_MAX; i++) {
        float pos = cfg->positions_mm[i]; /* 当前槽位坐标 */

        if ((cfg->enabled_mask & (uint8_t)(1U << i)) == 0U) {
            continue; /* 禁用槽位坐标不参与校验 */
        }
        /* 拒绝NaN（自比较为假即NaN），启用槽位须严格递增 */
        if ((pos != pos) ||
            ((has_prev != 0U) && (pos <= prev_mm))) {
            return LSENSOR_DRV_ERR_PARAM;
        }
        prev_mm  = pos;
        has_prev = 1U;
    }

    /* 重复init直接覆盖，ALREADY语义由上层契约负责 */
    s_drv.cfg   = *cfg;
    s_drv.ready = 1U;
    return LSENSOR_DRV_OK;
}

lsensor_drv_status_t lsensor_drv_deinit(void)
{
    /* 最小实现：清就绪标志与注入指针，几何残值无害 */
    s_drv.ready       = 0U;
    s_drv.cfg.get_map = NULL;
    return LSENSOR_DRV_OK;
}

lsensor_drv_status_t lsensor_drv_read(lsensor_drv_result_t *out)
{
    lsensor_drv_result_t res;      /* 成功前的本地结果 */
    uint8_t              raw = 0U; /* 注入源原始电平位图 */
    uint8_t              map;      /* 归一后压线位图 */

    if (out == NULL) {
        return LSENSOR_DRV_ERR_PARAM;
    }
    if (s_drv.ready == 0U) {
        return LSENSOR_DRV_ERR_STATE;
    }
    if (s_drv.cfg.get_map(&raw) != LSENSOR_DRV_OK) {
        return LSENSOR_DRV_ERR_IO; /* 失败路径不修改out */
    }

    /* 归一：低有效按位取反成"1=压线"，再裁剪到启用槽位 */
    map = (s_drv.cfg.active_low != 0U) ? (uint8_t)(~raw) : raw;
    map = (uint8_t)(map & s_drv.cfg.enabled_mask);

    calc_result(map, &res);
    *out = res;
    return LSENSOR_DRV_OK;
}
