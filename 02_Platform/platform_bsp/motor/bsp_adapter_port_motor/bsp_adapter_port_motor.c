/**
 * @file    bsp_adapter_port_motor.c
 * @brief   电机Adapter：绑定bsp_motor驱动、板级映射与PWM/IO/编码器/节拍
 * @note    - 唯一知道电机接线、编码器映射、CPR与PID参数的位置
 *          - 10ms控制节拍经core_timer在TIM9中断内直跑：
 *            编码器速度更新 -> PID -> 写PWM，OFF态只维持里程
 *          - 编码器步进经core_encoder在EXTI中断内注入驱动计数；
 *            EXTI与TIM9同NVIC优先级，两条ISR互不抢占故计数无竞争
 *          - 任务侧与节拍ISR的并发约定：
 *            读状态走序号双查快照(seq偶数稳定，写方为ISR天然原子)；
 *            写目标转速为对齐单字存储，天然原子；
 *            init/start/stop/deinit改写驱动对象，以core_timer_isr_mask
 *            短暂屏蔽节拍防止半旧对象被控制环使用，不可嵌套
 *          - 故障策略：节拍内任一底层失败即锁存FLT_HW并断输出滑行，
 *            进入ERROR态；stop->start清除锁存后重新闭环
 */

#include "bsp_adapter_port_motor.h"

#include "board_motor_config.h"
#include "bsp_motor_driver.h"
#include "encoder_port.h"
#include "io_port.h"
#include "platform_def.h"
#include "pwm_port.h"
#include "timer_port.h"

/** 单电机板级绑定与运行上下文。 */
typedef struct {
    motor_driver_t         drv;      /* 驱动对象(含编码器/PID) */
    en_core_pwm_t          pwm_id;   /* 逻辑PWM通道 */
    core_io_id_t           in1_io;   /* IN1逻辑IO */
    core_io_id_t           in2_io;   /* IN2逻辑IO */
    en_core_encoder_t      enc_id;   /* 逻辑编码器实例 */
    motor_dir_t            fwd_dir;  /* 正输出对应旋转方向 */
    encoder_sign_t         enc_sign; /* 编码器计数极性 */
    bsp_motor_set_enable_t pf_en;    /* 本电机使能跳板 */
    bsp_motor_set_output_t pf_out;   /* 本电机输出跳板 */
    volatile uint8_t       active;   /* 1=闭环运行(节拍驱动) */
    volatile uint32_t      fault;    /* 锁存故障位MOTOR_DRV_FLT_* */
    volatile uint32_t      seq;      /* 快照序号，偶数=稳定 */
    /* volatile强制任务侧seq双查循环每轮重读，防编译器缓存旧副本 */
    volatile motor_drv_state_t snap; /* 节拍末写入的状态快照 */
} motor_adp_t;

/** 两轮上下文：[0]=A左轮，[1]=B右轮。 */
static motor_adp_t s_motor[2];

static motor_drv_status_t map_motor_st(motor_status_t st);
static motor_drv_status_t map_platform_err(platform_err_t err);
static motor_adp_t *motor_ctx(motor_drv_t *dev);
static motor_status_t motor_hw_out(motor_adp_t *m,
                                   motor_dir_t dir, uint16_t pwm);
static motor_status_t motor_hw_en(motor_adp_t *m, uint8_t enable);
static motor_status_t motor_a_hw_set_enable(uint8_t enable);
static motor_status_t motor_a_hw_set_output(motor_dir_t dir,
                                            uint16_t pwm);
static motor_status_t motor_b_hw_set_enable(uint8_t enable);
static motor_status_t motor_b_hw_set_output(motor_dir_t dir,
                                            uint16_t pwm);
static void motor_a_enc_step_isr(int8_t step);
static void motor_b_enc_step_isr(int8_t step);
static void motor_snap_write(motor_adp_t *m);
static void motor_tick_update(motor_adp_t *m);
static void motor_tick_isr(void);
static motor_drv_status_t motor_op_init(motor_drv_t *dev);
static motor_drv_status_t motor_op_deinit(motor_drv_t *dev);
static motor_drv_status_t motor_op_start(motor_drv_t *dev);
static motor_drv_status_t motor_op_stop(motor_drv_t *dev);
static motor_drv_status_t motor_op_set_rps(motor_drv_t *dev,
                                           float rps);
static motor_drv_status_t motor_op_get_state(motor_drv_t *dev,
                                             motor_drv_state_t *state);

/**
 * @brief  将驱动状态码映射为wrapper状态码
 * @param  st 驱动调用结果
 * @retval 对应motor_drv_status_t，未知状态归入FAIL
 */
static motor_drv_status_t map_motor_st(motor_status_t st)
{
    switch (st) {
        case MOTOR_OK:
            return MOTOR_DRV_OK;

        case MOTOR_ERRORPARAMETER:
            return MOTOR_DRV_ERR_PARAM;

        default:
            return MOTOR_DRV_ERR_FAIL;
    }
}

/**
 * @brief  将平台错误码映射为wrapper状态码
 * @param  err MCU Port调用结果
 * @retval 对应motor_drv_status_t，未知错误归入FAIL
 */
static motor_drv_status_t map_platform_err(platform_err_t err)
{
    switch (err) {
        case PLATFORM_ERR_OK:
            return MOTOR_DRV_OK;

        case PLATFORM_ERR_PARAM:
            return MOTOR_DRV_ERR_PARAM;

        case PLATFORM_ERR_ALREADY_INIT:
            return MOTOR_DRV_ERR_STATE;

        default:
            return MOTOR_DRV_ERR_FAIL;
    }
}

/**
 * @brief  从设备表取回本电机上下文
 * @param  dev wrapper回传的设备表
 * @retval 上下文指针，非法时返回NULL
 */
static motor_adp_t *motor_ctx(motor_drv_t *dev)
{
    if ((dev == NULL) || (dev->user_data == NULL)) {
        return NULL;
    }
    return (motor_adp_t *)dev->user_data;
}

/**
 * @brief  单电机硬件输出：方向脚+PWM占空比
 * @param  m   电机上下文
 * @param  dir 目标旋转方向，pwm=0时忽略
 * @param  pwm PWM原始值，0=双低滑行
 * @retval MOTOR_OK / MOTOR_ERROR / MOTOR_ERRORPARAMETER
 * @note   先断PWM再切方向脚，避免换向瞬间冲击
 */
static motor_status_t motor_hw_out(motor_adp_t *m,
                                   motor_dir_t dir, uint16_t pwm)
{
    core_io_level_t in1; /* IN1目标电平 */
    core_io_level_t in2; /* IN2目标电平 */

    if (PLATFORM_IS_ERR(core_pwm_set_duty(m->pwm_id, 0U))) {
        return MOTOR_ERROR;
    }

    if (pwm == 0U) {
        in1 = CORE_IO_LOW; /* 双低滑行 */
        in2 = CORE_IO_LOW;
    } else if (dir == MOTOR_DIR_CW) {
        in1 = CORE_IO_HIGH; /* IN1高+IN2低=顺时针 */
        in2 = CORE_IO_LOW;
    } else if (dir == MOTOR_DIR_CCW) {
        in1 = CORE_IO_LOW;
        in2 = CORE_IO_HIGH;
    } else {
        return MOTOR_ERRORPARAMETER;
    }

    if (PLATFORM_IS_ERR(core_io_write(m->in1_io, in1)) ||
        PLATFORM_IS_ERR(core_io_write(m->in2_io, in2))) {
        return MOTOR_ERROR;
    }

    if (pwm == 0U) {
        return MOTOR_OK;
    }
    if (PLATFORM_IS_ERR(core_pwm_set_duty(m->pwm_id, pwm))) {
        return MOTOR_ERROR;
    }
    return MOTOR_OK;
}

/**
 * @brief  单电机硬件使能
 * @param  m      电机上下文
 * @param  enable 1=使能，0=断输出滑行
 * @retval MOTOR_OK / MOTOR_ERROR / MOTOR_ERRORPARAMETER
 * @note   使能本身无硬件动作，方向与占空比随首次输出建立
 */
static motor_status_t motor_hw_en(motor_adp_t *m, uint8_t enable)
{
    if ((enable != 0U) && (enable != 1U)) {
        return MOTOR_ERRORPARAMETER;
    }
    if (enable == 0U) {
        return motor_hw_out(m, m->fwd_dir, 0U);
    }
    return MOTOR_OK;
}

/* 驱动底层接口无上下文参数，为两电机提供显式跳板 */

/** @brief A电机使能跳板（驱动pf_set_enable） */
static motor_status_t motor_a_hw_set_enable(uint8_t enable)
{
    return motor_hw_en(&s_motor[0], enable);
}

/** @brief A电机输出跳板（驱动pf_set_output） */
static motor_status_t motor_a_hw_set_output(motor_dir_t dir,
                                            uint16_t pwm)
{
    return motor_hw_out(&s_motor[0], dir, pwm);
}

/** @brief B电机使能跳板（驱动pf_set_enable） */
static motor_status_t motor_b_hw_set_enable(uint8_t enable)
{
    return motor_hw_en(&s_motor[1], enable);
}

/** @brief B电机输出跳板（驱动pf_set_output） */
static motor_status_t motor_b_hw_set_output(motor_dir_t dir,
                                            uint16_t pwm)
{
    return motor_hw_out(&s_motor[1], dir, pwm);
}

/** @brief A电机编码器步进ISR入口（EXTI上下文） */
static void motor_a_enc_step_isr(int8_t step)
{
    encoder_count_isr(&s_motor[0].drv.encoder, step);
}

/** @brief B电机编码器步进ISR入口（EXTI上下文） */
static void motor_b_enc_step_isr(int8_t step)
{
    encoder_count_isr(&s_motor[1].drv.encoder, step);
}

/**
 * @brief  写一拍状态快照（仅节拍ISR或屏蔽节拍后调用）
 * @param  m 电机上下文
 * @note   seq奇数=写窗口；写方相对任务侧天然原子，
 *         任务侧以seq双查检测覆盖
 */
static void motor_snap_write(motor_adp_t *m)
{
    motor_drv_run_t run; /* 本拍运行状态 */

    if (m->fault != MOTOR_DRV_FLT_NONE) {
        run = MOTOR_DRV_RUN_ERROR;
    } else if (m->active != 0U) {
        run = MOTOR_DRV_RUN_ACTIVE;
    } else {
        run = MOTOR_DRV_RUN_OFF;
    }

    m->seq++; /* 进入写窗口 */
    m->snap.target_rps   = m->drv.target_speed_rps;
    m->snap.rps          = m->drv.encoder.speed_rps;
    m->snap.position     = m->drv.encoder.position_ticks;
    m->snap.run_state    = run;
    m->snap.fault_flags  = m->fault;
    m->snap.applied_duty = (int32_t)m->drv.output;
    m->seq++; /* 退出写窗口 */
}

/**
 * @brief  单电机一拍控制更新（TIM9 ISR上下文）
 * @param  m 电机上下文
 * @note   OFF态仍更新编码器，滑行/被推动时里程与速度保持有效；
 *         底层失败即锁存故障、断输出并退出闭环
 */
static void motor_tick_update(motor_adp_t *m)
{
    if (m->drv.is_inited == 0U) {
        return;
    }

    (void)encoder_update(&m->drv.encoder);

    if (m->active != 0U) {
        if (motor_driver_pid_update(&m->drv) != MOTOR_OK) {
            m->fault |= MOTOR_DRV_FLT_HW;
            (void)motor_driver_stop(&m->drv);
            m->active = 0U;
        }
    }

    motor_snap_write(m);
}

/** @brief 10ms控制节拍入口（TIM9 ISR上下文） */
static void motor_tick_isr(void)
{
    motor_tick_update(&s_motor[0]);
    motor_tick_update(&s_motor[1]);
}

/**
 * @brief  初始化单电机（Wrapper pf_init）
 * @param  dev 设备表
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM / MOTOR_DRV_ERR_FAIL
 * @note   完成后硬件静止、OFF态；重复init视为重新装配
 */
static motor_drv_status_t motor_op_init(motor_drv_t *dev)
{
    motor_adp_t       *m = motor_ctx(dev); /* 本电机上下文 */
    motor_drv_status_t ret;                /* 装配结果 */
    platform_err_t     err;                /* Port调用结果 */

    if (m == NULL) {
        return MOTOR_DRV_ERR_PARAM;
    }

    /* 屏蔽节拍，防止重初始化期间控制环使用半旧对象 */
    (void)core_timer_isr_mask(BOARD_MOTOR_TICK_TIMER);

    m->active = 0U;
    m->fault  = MOTOR_DRV_FLT_NONE;

    ret = map_motor_st(motor_driver_inst(&m->drv, m->pf_en,
                                         m->pf_out, m->fwd_dir,
                                         (uint16_t)BOARD_MOTOR_PWM_MAX));
    if (ret == MOTOR_DRV_OK) {
        if (encoder_inst(&m->drv.encoder, m->enc_sign,
                         BOARD_MOTOR_CPR,
                         BOARD_MOTOR_TICK_S) != ENCODER_OK) {
            ret = MOTOR_DRV_ERR_FAIL;
        }
    }
    if (ret == MOTOR_DRV_OK) {
        if (motor_pid_inst(&m->drv.pid, BOARD_MOTOR_PID_KP,
                           BOARD_MOTOR_PID_KI, BOARD_MOTOR_PID_KD,
                           (float)BOARD_MOTOR_PWM_MAX,
                           -(float)BOARD_MOTOR_PWM_MAX,
                           BOARD_MOTOR_PID_ILIMIT) != PID_OK) {
            ret = MOTOR_DRV_ERR_FAIL;
        }
    }
    if (ret == MOTOR_DRV_OK) {
        motor_snap_write(m); /* 初始OFF快照 */
    }

    (void)core_timer_isr_unmask(BOARD_MOTOR_TICK_TIMER);
    if (ret != MOTOR_DRV_OK) {
        return ret;
    }

    /* 硬件置于安全静止：断PWM+方向双低，再放行外设 */
    if (motor_hw_out(m, m->fwd_dir, 0U) != MOTOR_OK) {
        return MOTOR_DRV_ERR_FAIL;
    }
    err = core_pwm_start(m->pwm_id);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }
    err = core_encoder_start(m->enc_id);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }

    /* 节拍为两电机共享，重复start幂等 */
    err = core_timer_start(BOARD_MOTOR_TICK_TIMER);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }
    return MOTOR_DRV_OK;
}

/**
 * @brief  反初始化单电机（Wrapper pf_deinit）
 * @param  dev 设备表
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM / MOTOR_DRV_ERR_FAIL
 * @note   停输出、停编码器上报、停PWM通道；
 *         节拍定时器为共享资源保持运行，控制环按is_inited跳过
 */
static motor_drv_status_t motor_op_deinit(motor_drv_t *dev)
{
    motor_adp_t       *m   = motor_ctx(dev);  /* 本电机上下文 */
    motor_drv_status_t ret = MOTOR_DRV_OK;    /* 首个失败结果 */

    if (m == NULL) {
        return MOTOR_DRV_ERR_PARAM;
    }

    (void)core_timer_isr_mask(BOARD_MOTOR_TICK_TIMER);
    m->active = 0U;
    if (m->drv.is_enabled != 0U) {
        if (motor_driver_stop(&m->drv) != MOTOR_OK) {
            ret = MOTOR_DRV_ERR_FAIL;
        }
    }
    m->drv.is_inited = 0U; /* 驱动无deinit接口，显式退出装配态 */
    (void)core_timer_isr_unmask(BOARD_MOTOR_TICK_TIMER);

    if (PLATFORM_IS_ERR(core_encoder_stop(m->enc_id))) {
        ret = (ret == MOTOR_DRV_OK) ? MOTOR_DRV_ERR_FAIL : ret;
    }
    (void)core_pwm_set_duty(m->pwm_id, 0U);
    if (PLATFORM_IS_ERR(core_pwm_stop(m->pwm_id))) {
        ret = (ret == MOTOR_DRV_OK) ? MOTOR_DRV_ERR_FAIL : ret;
    }
    return ret;
}

/**
 * @brief  使能单电机进入闭环（Wrapper pf_start）
 * @param  dev 设备表
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_STATE(未init) / MOTOR_DRV_ERR_FAIL
 * @note   清除故障锁存并复位PID；目标转速保持上次stop清零后的值
 */
static motor_drv_status_t motor_op_start(motor_drv_t *dev)
{
    motor_adp_t       *m   = motor_ctx(dev); /* 本电机上下文 */
    motor_drv_status_t ret = MOTOR_DRV_OK;   /* 使能结果 */

    if (m == NULL) {
        return MOTOR_DRV_ERR_PARAM;
    }
    if (m->drv.is_inited == 0U) {
        return MOTOR_DRV_ERR_STATE;
    }

    (void)core_timer_isr_mask(BOARD_MOTOR_TICK_TIMER);
    m->fault = MOTOR_DRV_FLT_NONE;
    if (m->active == 0U) {
        ret = map_motor_st(motor_driver_enable(&m->drv, 1U));
        if (ret == MOTOR_DRV_OK) {
            m->active = 1U;
        }
    }
    (void)core_timer_isr_unmask(BOARD_MOTOR_TICK_TIMER);
    return ret;
}

/**
 * @brief  失能单电机滑行（Wrapper pf_stop）
 * @param  dev 设备表
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_STATE(未init) / MOTOR_DRV_ERR_FAIL
 * @note   保留累计位置；目标转速被清零，重启后需重新设置
 */
static motor_drv_status_t motor_op_stop(motor_drv_t *dev)
{
    motor_adp_t       *m   = motor_ctx(dev); /* 本电机上下文 */
    motor_drv_status_t ret = MOTOR_DRV_OK;   /* 停止结果 */

    if (m == NULL) {
        return MOTOR_DRV_ERR_PARAM;
    }
    if (m->drv.is_inited == 0U) {
        return MOTOR_DRV_ERR_STATE;
    }

    (void)core_timer_isr_mask(BOARD_MOTOR_TICK_TIMER);
    if (m->active != 0U) {
        m->active = 0U;
        ret = map_motor_st(motor_driver_stop(&m->drv));
    }
    (void)core_timer_isr_unmask(BOARD_MOTOR_TICK_TIMER);
    return ret;
}

/**
 * @brief  设置单电机目标转速（Wrapper pf_set_rps）
 * @param  dev 设备表
 * @param  rps 输出轴目标转速，正=前进方向
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_STATE(未init)
 * @note   对齐单字浮点存储，相对节拍ISR天然原子，无需屏蔽
 */
static motor_drv_status_t motor_op_set_rps(motor_drv_t *dev,
                                           float rps)
{
    motor_adp_t *m = motor_ctx(dev); /* 本电机上下文 */

    if (m == NULL) {
        return MOTOR_DRV_ERR_PARAM;
    }
    if (m->drv.is_inited == 0U) {
        return MOTOR_DRV_ERR_STATE;
    }

    return map_motor_st(
        motor_driver_set_target_speed(&m->drv, rps));
}

/**
 * @brief  读取单电机同拍状态快照（Wrapper pf_get_state）
 * @param  dev   设备表
 * @param  state 输出快照，失败时不修改
 * @retval MOTOR_DRV_OK / MOTOR_DRV_ERR_PARAM /
 *         MOTOR_DRV_ERR_STATE(未init)
 * @note   seq双查：写方为ISR整体先行，读中被打断则seq变化重读
 */
static motor_drv_status_t motor_op_get_state(motor_drv_t *dev,
                                             motor_drv_state_t *state)
{
    motor_adp_t      *m = motor_ctx(dev); /* 本电机上下文 */
    motor_drv_state_t copy;               /* 本地一致副本 */
    uint32_t          seq_before;         /* 读前序号 */
    uint32_t          seq_after;          /* 读后序号 */

    if ((m == NULL) || (state == NULL)) {
        return MOTOR_DRV_ERR_PARAM;
    }
    if (m->drv.is_inited == 0U) {
        return MOTOR_DRV_ERR_STATE;
    }

    do {
        seq_before = m->seq;
        if ((seq_before & 1U) != 0U) {
            continue; /* 写窗口内，重读 */
        }
        copy.target_rps   = m->snap.target_rps;
        copy.rps          = m->snap.rps;
        copy.position     = m->snap.position;
        copy.run_state    = m->snap.run_state;
        copy.fault_flags  = m->snap.fault_flags;
        copy.applied_duty = m->snap.applied_duty;
        seq_after = m->seq;
    } while (seq_before != seq_after);

    *state = copy;
    return MOTOR_DRV_OK;
}

motor_drv_status_t drv_adapter_motor_register(void)
{
    motor_drv_t        dev; /* 待注册的设备表 */
    platform_err_t     err; /* ISR注册结果 */
    motor_drv_status_t ret; /* 槽位注册结果 */

    /* A电机（左轮）板级绑定 */
    s_motor[0].pwm_id   = BOARD_MOTOR_A_PWM;
    s_motor[0].in1_io   = (core_io_id_t)BOARD_MOTOR_A_IO_IN1;
    s_motor[0].in2_io   = (core_io_id_t)BOARD_MOTOR_A_IO_IN2;
    s_motor[0].enc_id   = BOARD_MOTOR_A_ENCODER;
    s_motor[0].fwd_dir  = (BOARD_MOTOR_A_FWD_CW != 0U)
                        ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    s_motor[0].enc_sign = (BOARD_MOTOR_A_ENC_REVERSED != 0U)
                        ? ENCODER_SIGN_REVERSED
                        : ENCODER_SIGN_NORMAL;
    s_motor[0].pf_en    = motor_a_hw_set_enable;
    s_motor[0].pf_out   = motor_a_hw_set_output;

    /* B电机（右轮）板级绑定 */
    s_motor[1].pwm_id   = BOARD_MOTOR_B_PWM;
    s_motor[1].in1_io   = (core_io_id_t)BOARD_MOTOR_B_IO_IN1;
    s_motor[1].in2_io   = (core_io_id_t)BOARD_MOTOR_B_IO_IN2;
    s_motor[1].enc_id   = BOARD_MOTOR_B_ENCODER;
    s_motor[1].fwd_dir  = (BOARD_MOTOR_B_FWD_CW != 0U)
                        ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    s_motor[1].enc_sign = (BOARD_MOTOR_B_ENC_REVERSED != 0U)
                        ? ENCODER_SIGN_REVERSED
                        : ENCODER_SIGN_NORMAL;
    s_motor[1].pf_en    = motor_b_hw_set_enable;
    s_motor[1].pf_out   = motor_b_hw_set_output;

    /* 挂ISR分发（此时节拍未启动、EXTI未放行，回调不会触发） */
    err = core_timer_reg_isr(BOARD_MOTOR_TICK_TIMER, motor_tick_isr);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }
    err = core_encoder_reg_isr(BOARD_MOTOR_A_ENCODER,
                               motor_a_enc_step_isr);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }
    err = core_encoder_reg_isr(BOARD_MOTOR_B_ENCODER,
                               motor_b_enc_step_isr);
    if (PLATFORM_IS_ERR(err)) {
        return map_platform_err(err);
    }

    /* 装配设备表并逐槽位注册，op全部必填 */
    dev.idx          = 0U; /* 注册成功后由wrapper回填 */
    dev.pf_init      = motor_op_init;
    dev.pf_deinit    = motor_op_deinit;
    dev.pf_start     = motor_op_start;
    dev.pf_stop      = motor_op_stop;
    dev.pf_set_rps   = motor_op_set_rps;
    dev.pf_get_state = motor_op_get_state;

    dev.dev_id    = 0U; /* A电机 */
    dev.user_data = &s_motor[0];
    ret = drv_adapter_motor_reg(BOARD_MOTOR_A_SLOT, &dev);
    if (ret != MOTOR_DRV_OK) {
        return ret;
    }

    dev.dev_id    = 1U; /* B电机 */
    dev.user_data = &s_motor[1];
    return drv_adapter_motor_reg(BOARD_MOTOR_B_SLOT, &dev);
}
