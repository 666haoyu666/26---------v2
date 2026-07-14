/**
 * @file    bsp_lsensor_driver.h
 * @brief   八路二值灰度传感器组driver合同（单实例）
 * @note    - 单实例模块，无handler；由line_sensor的Adapter装配注入
 *          - 硬件访问经init注入的get_map获取，本头文件不见平台接口
 *          - 簇分析为纯计算内核（输入=归一后压线位图），PC可单测
 *          - 判定规则与几何规格见《灰度传感器模块设计》4.3,
 *          - 注意连续性的断点，这里采用的处理是：禁用槽位视作连续性断点
 */

#ifndef BSP_LSENSOR_DRIVER_H
#define BSP_LSENSOR_DRIVER_H

#include "platform_def.h"

#define LSENSOR_DRV_SLOT_MAX (8U) /* 稳定物理槽位容量，与平台契约一致 */

/** driver状态码，仅表达本次调用结果。 */
typedef enum {
    LSENSOR_DRV_OK        = 0,    /* 操作成功 */
    LSENSOR_DRV_ERR_PARAM = 1,    /* 参数非法 */
    LSENSOR_DRV_ERR_STATE = 2,    /* 未初始化 */
    LSENSOR_DRV_ERR_IO    = 3,    /* 注入数据源读取失败 */
    LSENSOR_DRV_RESERVED  = 0xFF  /* 保留 */
} lsensor_drv_status_t;

/** 本拍线位置识别结果，platform侧枚举映射在契约实现区完成。 */
typedef enum {
    LSENSOR_DRV_NO_LINE = 0, /* 无槽位命中 */
    LSENSOR_DRV_TRACKING,    /* 唯一连续命中簇，偏差有效 */
    LSENSOR_DRV_AMBIGUOUS    /* 多簇不连续或全路命中 */
} lsensor_drv_track_t;

/** 注入的数据源：读一拍8路原始电平位图，位i=槽位i，1=高电平。 */
typedef lsensor_drv_status_t (*pf_lsensor_get_t)(uint8_t *raw_map);

/** 驱动配置，init时整体拷贝持有；几何+极性+数据源。 */
typedef struct {
    float   positions_mm[LSENSOR_DRV_SLOT_MAX]; /* 槽位i坐标，右正左负 */
    uint8_t enabled_mask; /* 位i=1已装配；置0槽位视作连续性断点 */
    uint8_t active_low;   /* 1=压线为低电平，读取后按位取反归一 */
    /* 外部依赖注入 */
    pf_lsensor_get_t get_map; /* 读一拍原始位图，不可为NULL */
} lsensor_drv_cfg_t;

/** 单拍处理结果。 */
typedef struct {
    float               offset_mm; /* 线偏差mm，仅TRACKING有效，否则0 */
    lsensor_drv_track_t track;     /* 本拍识别结果 */
} lsensor_drv_result_t;

/**
 * @brief  初始化传感器驱动，校验并持有配置副本
 * @param  cfg 驱动配置，cfg与get_map均不可为NULL
 * @retval LSENSOR_DRV_OK / LSENSOR_DRV_ERR_PARAM
 * @note   要求enabled_mask非0，启用槽位positions_mm严格递增；
 *         重复init直接覆盖，ALREADY语义由上层契约负责
 */
lsensor_drv_status_t lsensor_drv_init(const lsensor_drv_cfg_t *cfg);

/**
 * @brief  注销驱动并清除持有的配置
 * @retval LSENSOR_DRV_OK
 */
lsensor_drv_status_t lsensor_drv_deinit(void);

/**
 * @brief  经注入数据源取一拍原始位图，归一后完成识别
 * @param  out 输出结果，不可为NULL；失败时不修改
 * @retval LSENSOR_DRV_OK / LSENSOR_DRV_ERR_PARAM /
 *         LSENSOR_DRV_ERR_STATE / LSENSOR_DRV_ERR_IO
 * @note   归一：active_low为1时按位取反，再与enabled_mask求交
 */
lsensor_drv_status_t lsensor_drv_read(lsensor_drv_result_t *out);

#endif /* BSP_LSENSOR_DRIVER_H */
