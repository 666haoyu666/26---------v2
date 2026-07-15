/**
 * @file    bsp_wrapper_motor.h
 * @brief   电机能力Wrapper：稳定物理槽位注册表与器件无关转发API
 * @note    - 槽位id不承载左右轮等底盘语义，底盘映射由Service负责
 *          - 注册仅限内核启动前单线程完成，运行期注册表只读
 *          - 群体API作用于全部已注册槽位，无任何注册时报NOTREG
 *          - 快照一致性由设备op实现负责（控制节拍在Handler侧）
 */

#ifndef BSP_WRAPPER_MOTOR_H
#define BSP_WRAPPER_MOTOR_H

#include <stdint.h>

#define MOTOR_DRV_MAX_NUM  (4U) /* 稳定物理电机槽位容量 */
#define MOTOR_DRV_FLT_NONE (0U) /* fault_flags无故障基值 */
#define MOTOR_DRV_FLT_HW   (1U << 0) /* 输出/控制硬件链路失败锁存 */

/** wrapper状态码，仅表达本次调用结果。 */
typedef enum {
    MOTOR_DRV_OK         = 0,         /* 操作成功 */
    MOTOR_DRV_ERR_PARAM  = 1,         /* 参数非法 */
    MOTOR_DRV_ERR_NOTREG = 2,         /* 槽位未注册 */
    MOTOR_DRV_ERR_STATE  = 3,         /* 状态不允许，如重复注册 */
    MOTOR_DRV_ERR_FAIL   = 4,         /* 底层操作失败 */
    MOTOR_DRV_RESERVED   = 0x7FFFFFFF /* 保留，固定枚举宽度 */
} motor_drv_status_t;

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
    motor_drv_status_t (*pf_init)(struct motor_drv *dev);
    motor_drv_status_t (*pf_deinit)(struct motor_drv *dev);
    motor_drv_status_t (*pf_start)(struct motor_drv *dev);
    motor_drv_status_t (*pf_stop)(struct motor_drv *dev);
    motor_drv_status_t (*pf_set_rps)(struct motor_drv *dev, float rps);
    motor_drv_status_t (*pf_get_state)(struct motor_drv *dev,
                                       motor_drv_state_t *state);
} motor_drv_t;

/**
 * @brief  注册一个电机设备表到指定槽位，Adapter专用
 * @param  index 稳定物理槽位，0到MOTOR_DRV_MAX_NUM-1
 * @param  dev   设备表，op全部必填；wrapper整体拷贝持有
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM / MOTOR_DRV_ERR_STATE
 * @note   重复注册同一槽位返回ERR_STATE；仅限内核启动前调用
 */
motor_drv_status_t drv_adapter_motor_reg(uint32_t index,
                                         const motor_drv_t *dev);

/**
 * @brief  初始化全部已注册电机，成功后仍处OFF态
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_NOTREG / 底层首个错误
 */
motor_drv_status_t drv_adapter_motor_init(void);

/**
 * @brief  反初始化全部已注册电机并释放器件资源
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_NOTREG / 底层首个错误
 */
motor_drv_status_t drv_adapter_motor_deinit(void);

/**
 * @brief  使能全部已注册电机，保留累计位置，进入ACTIVE态
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_NOTREG / 底层首个错误
 */
motor_drv_status_t drv_adapter_motor_start(void);

/**
 * @brief  失能全部已注册电机并滑行，保留累计位置，进入OFF态
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_NOTREG / 底层首个错误
 */
motor_drv_status_t drv_adapter_motor_stop(void);

/**
 * @brief  设置一个电机的目标转速
 * @param  id  稳定物理槽位
 * @param  rps 目标转速，须为有限值；限幅策略由底层配置决定
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_NOTREG / 底层错误
 */
motor_drv_status_t drv_adapter_motor_set_rps(uint32_t id, float rps);

/**
 * @brief  伪原子设置多电机目标：全部校验通过才开始应用
 * @param  targets 目标数组，不可为NULL
 * @param  count   条目数，1到MOTOR_DRV_MAX_NUM
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_NOTREG / 底层首个错误
 * @note   校验后逐个应用，中途失败即返回，已应用目标不回退
 */
motor_drv_status_t drv_adapter_motor_set_targets(
    const motor_drv_target_t *targets,
    uint32_t count);

/**
 * @brief  读取一个电机的同拍状态快照
 * @param  id    稳定物理槽位
 * @param  state 输出状态，不可为NULL；失败时不修改
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_NOTREG / 底层错误
 */
motor_drv_status_t drv_adapter_motor_get_state(uint32_t id,
                                               motor_drv_state_t *state);

#endif /* BSP_WRAPPER_MOTOR_H */
