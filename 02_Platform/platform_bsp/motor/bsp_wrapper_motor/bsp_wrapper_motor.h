/**
 * @file    bsp_wrapper_motor.h
 * @brief   电机能力Wrapper：稳定物理槽位注册表与器件无关转发API
 * @note    - 错误码统一使用platform_err_t，由Adapter完成Driver映射：
 *            槽位未注册报NOT_INITIALIZED，重复注册报ALREADY_INIT
 *          - 槽位id不承载左右轮等底盘语义，底盘映射由Service负责
 *          - 注册仅限组合根初始化阶段单线程完成，运行期注册表只读
 *          - 群体API作用于全部已注册槽位，无任何注册时报NOT_INITIALIZED
 *          - 快照一致性由设备op实现负责（控制节拍在Adapter侧）
 */

#ifndef BSP_WRAPPER_MOTOR_H
#define BSP_WRAPPER_MOTOR_H

#include "platform_error.h"
#include "platform_type.h"

#define MOTOR_DRV_MAX_NUM  (4U)      /* 稳定物理电机槽位容量 */
#define MOTOR_DRV_FLT_NONE (0U)      /* fault_flags无故障基值 */
#define MOTOR_DRV_FLT_HW   (1U << 0) /* 输出/控制硬件链路失败锁存 */

/** 单电机模块运行状态。 */
typedef enum {
    MOTOR_DRV_RUN_OFF = 0, /* 失能滑行 */
    MOTOR_DRV_RUN_ACTIVE,  /* 闭环运行 */
    MOTOR_DRV_RUN_ERROR    /* 故障锁存 */
} motor_drv_run_t;

/** 单电机一致状态快照。 */
typedef struct {
    float           target_rps;   /* 当前目标转速 */
    float           rps;          /* 滤波后输出轴转速 */
    int64_t         position;     /* 原始累计tick */
    motor_drv_run_t run_state;    /* 模块运行状态 */
    uint32_t        fault_flags;  /* 锁存故障位，MOTOR_DRV_FLT_* */
    int32_t         applied_duty; /* 已应用占空比 */
} motor_drv_state_t;

/** 多电机伪原子设定的单条目标。 */
typedef struct {
    uint32_t id;  /* 稳定物理槽位 */
    float    rps; /* 输出轴目标转速 */
} motor_drv_target_t;

/** 单电机设备能力表，由Adapter装配后注册；op全部必填。 */
typedef struct motor_drv {
    uint32_t idx;       /* 注册槽位，reg成功后由wrapper回填 */
    uint32_t dev_id;    /* 器件标识，Adapter自定义 */
    void    *user_data; /* Adapter私有上下文 */
    platform_err_t (*pf_init)(struct motor_drv *dev);
    platform_err_t (*pf_deinit)(struct motor_drv *dev);
    platform_err_t (*pf_start)(struct motor_drv *dev);
    platform_err_t (*pf_stop)(struct motor_drv *dev);
    platform_err_t (*pf_set_rps)(struct motor_drv *dev, float rps);
    platform_err_t (*pf_get_state)(struct motor_drv *dev,
                                   motor_drv_state_t *state);
} motor_drv_t;

/**
 * @brief  注册一个电机设备表到指定槽位，Adapter专用
 * @param  index 稳定物理槽位，0到MOTOR_DRV_MAX_NUM-1
 * @param  dev   设备表，op全部必填；wrapper整体拷贝持有
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_ALREADY_INIT(重复注册)
 * @note   仅限组合根初始化阶段调用
 */
platform_err_t drv_adapter_motor_reg(uint32_t index,
                                     const motor_drv_t *dev);

/**
 * @brief  初始化全部已注册电机，成功后仍处OFF态
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层首个错误
 */
platform_err_t drv_adapter_motor_init(void);

/**
 * @brief  反初始化全部已注册电机并释放器件资源
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层首个错误
 */
platform_err_t drv_adapter_motor_deinit(void);

/**
 * @brief  使能全部已注册电机，保留累计位置，进入ACTIVE态
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层首个错误
 */
platform_err_t drv_adapter_motor_start(void);

/**
 * @brief  使能指定物理槽位的单个电机
 * @param  id 稳定物理槽位
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 * @note   用于单电机标定；不改变其他槽位的运行状态
 */
platform_err_t bsp_motor_start(uint32_t id);

/**
 * @brief  失能全部已注册电机并滑行，保留累计位置，进入OFF态
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_NOT_INITIALIZED /
 *         底层首个错误
 */
platform_err_t drv_adapter_motor_stop(void);

/**
 * @brief  设置一个电机的目标转速
 * @param  id  稳定物理槽位
 * @param  rps 目标转速，须为有限值；限幅策略由底层配置决定
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 */
platform_err_t drv_adapter_motor_set_rps(uint32_t id, float rps);

/**
 * @brief  伪原子设置多电机目标：全部校验通过才开始应用
 * @param  targets 目标数组，不可为NULL
 * @param  count   条目数，1到MOTOR_DRV_MAX_NUM
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层首个错误
 * @note   校验后逐个应用，中途失败即返回，已应用目标不回退
 */
platform_err_t drv_adapter_motor_set_targets(
    const motor_drv_target_t *targets,
    uint32_t count);

/**
 * @brief  读取一个电机的同拍状态快照
 * @param  id    稳定物理槽位
 * @param  state 输出状态，不可为NULL；失败时不修改
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM /
 *         PLATFORM_ERR_NOT_INITIALIZED / 底层错误
 */
platform_err_t drv_adapter_motor_get_state(uint32_t id,
                                           motor_drv_state_t *state);

#endif /* BSP_WRAPPER_MOTOR_H */
