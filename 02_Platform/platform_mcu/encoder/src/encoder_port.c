/**
 * @file    encoder_port.c
 * @brief   MCU正交编码器Port实现（STM32F411 HAL，EXTI双边沿4倍频）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - 引脚模式(双边沿EXTI)由MX_GPIO_Init配置，本层不重复初始化
 *          - .ioc未勾选EXTI的NVIC，故EXTI4/EXTI9_5的IRQHandler与
 *            HAL_GPIO_EXTI_Callback由本层唯一持有；若日后在CubeMX
 *            勾选NVIC生成同名函数，必须删除其中一份
 *          - NVIC优先级固定为ENC_EXTI_NVIC_PRIO，必须与TIM9控制节拍
 *            相同：EXTI计数ISR与10ms控制ISR互不抢占，编码器计数的
 *            累加(读改写)与读清零才无竞争
 */

#include "encoder_port.h"

#include "main.h"
#include "platform_def.h"

/** EXTI的NVIC抢占优先级，必须与TIM9(TIM1_BRK_TIM9_IRQn)一致。 */
#define ENC_EXTI_NVIC_PRIO (5U)

/** 单编码器最多占用的EXTI中断线数。 */
#define ENC_IRQN_MAX (2U)

/** 逻辑编码器到物理引脚/EXTI中断线的映射项。 */
typedef struct {
    GPIO_TypeDef *port_a;              /* A相GPIO端口 */
    uint16_t      pin_a;               /* A相HAL引脚掩码 */
    GPIO_TypeDef *port_b;              /* B相GPIO端口 */
    uint16_t      pin_b;               /* B相HAL引脚掩码 */
    IRQn_Type     irqn[ENC_IRQN_MAX];  /* 占用的EXTI中断线 */
    uint8_t       irqn_cnt;            /* 有效中断线数 */
} encoder_map_t;

/** 逻辑编码器表，ID=下标。 */
static const encoder_map_t s_enc_map[EN_CORE_ENCODER_NUM] = {
    /* EN_CORE_ENCODER_1：E1A=PB7,E1B=PB6，同在EXTI9_5 */
    { PB7_E1A_GPIO_Port, PB7_E1A_Pin,
      PB6_E1B_GPIO_Port, PB6_E1B_Pin,
      { EXTI9_5_IRQn, EXTI9_5_IRQn }, 1U },
    /* EN_CORE_ENCODER_2：E2A=PB5(EXTI9_5),E2B=PB4(EXTI4) */
    { PB5_E2A_GPIO_Port, PB5_E2A_Pin,
      PB4_E2B_GPIO_Port, PB4_E2B_Pin,
      { EXTI9_5_IRQn, EXTI4_IRQn }, 2U },
};

/**
 * 正交解码状态转移表（与已验证的电机测试工程一致）。
 * 状态=(A<<1)|B，下标=(上次状态<<2)|本次状态；
 * +1/-1为有效跳变，0为无变化或非法跳变(丢边沿)。
 */
static const int8_t s_step_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/** 各编码器上次相位状态，ISR内更新。 */
static uint8_t s_last_state[EN_CORE_ENCODER_NUM];

/** 各编码器注册的步进回调（Adapter ISR入口）。 */
static core_encoder_step_cb_t s_cb[EN_CORE_ENCODER_NUM];

/** 1=该编码器已启动上报（ISR读/线程写）。 */
static volatile uint8_t s_started[EN_CORE_ENCODER_NUM];

/**
 * @brief  读取一路编码器当前相位状态
 * @param  map 编码器映射项
 * @retval 相位状态，(A<<1)|B
 */
static uint8_t encoder_read_state(const encoder_map_t *map)
{
    uint8_t phase_a; /* A相电平 */
    uint8_t phase_b; /* B相电平 */

    phase_a = (HAL_GPIO_ReadPin(map->port_a, map->pin_a)
               == GPIO_PIN_SET) ? 1U : 0U;
    phase_b = (HAL_GPIO_ReadPin(map->port_b, map->pin_b)
               == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((phase_a << 1U) | phase_b);
}

/**
 * @brief  单编码器一次边沿事件的正交解码与上报
 * @param  id 逻辑编码器实例
 * @note   ISR上下文；仅解码+回调，最短中断
 */
static void encoder_decode_isr(en_core_encoder_t id)
{
    uint8_t now_state; /* 本次相位状态 */
    uint8_t table_idx; /* 转移表下标 */
    int8_t  step;      /* 本次计数步进 */

    if (s_started[id] == 0U) {
        return;
    }

    now_state = encoder_read_state(&s_enc_map[id]);
    table_idx = (uint8_t)((s_last_state[id] << 2U) | now_state);
    step      = s_step_table[table_idx];

    s_last_state[id] = now_state;

    if ((step != 0) && (s_cb[id] != NULL)) {
        s_cb[id](step);
    }
}

platform_err_t core_encoder_reg_isr(en_core_encoder_t id,
                                    core_encoder_step_cb_t cb)
{
    if ((id >= EN_CORE_ENCODER_NUM) || (cb == NULL)) {
        return PLATFORM_ERR_PARAM;
    }
    /* 拒绝重复注册，防止运行期暗中换表 */
    if (s_cb[id] != NULL) {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_cb[id] = cb;
    return PLATFORM_ERR_OK;
}

platform_err_t core_encoder_start(en_core_encoder_t id)
{
    const encoder_map_t *map; /* 目标编码器映射 */
    uint8_t              i;   /* 中断线游标 */

    if (id >= EN_CORE_ENCODER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    map = &s_enc_map[id];

    /* 先锁存当前相位再放行上报，避免首个跳变被误判方向 */
    s_last_state[id] = encoder_read_state(map);
    __HAL_GPIO_EXTI_CLEAR_IT(map->pin_a);
    __HAL_GPIO_EXTI_CLEAR_IT(map->pin_b);
    s_started[id] = 1U;

    for (i = 0U; i < map->irqn_cnt; i++) {
        HAL_NVIC_SetPriority(map->irqn[i], ENC_EXTI_NVIC_PRIO, 0U);
        HAL_NVIC_EnableIRQ(map->irqn[i]);
    }
    return PLATFORM_ERR_OK;
}

platform_err_t core_encoder_stop(en_core_encoder_t id)
{
    if (id >= EN_CORE_ENCODER_NUM) {
        return PLATFORM_ERR_PARAM;
    }

    /* EXTI中断线与其他编码器共享，仅软停本路上报 */
    s_started[id] = 0U;
    return PLATFORM_ERR_OK;
}

/**
 * @brief  GPIO外部中断回调（HAL弱符号唯一实现）
 * @param  GPIO_Pin 触发的引脚掩码
 * @note   ISR上下文：按引脚路由到所属编码器解码；
 *         非编码器EXTI需求出现时在此扩展路由
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t i; /* 映射表游标 */

    for (i = 0U; i < (uint32_t)EN_CORE_ENCODER_NUM; i++) {
        if ((GPIO_Pin == s_enc_map[i].pin_a) ||
            (GPIO_Pin == s_enc_map[i].pin_b)) {
            encoder_decode_isr((en_core_encoder_t)i);
            return;
        }
    }
}

/**
 * @brief  EXTI4中断服务函数（E2B=PB4）
 * @note   .ioc未勾选EXTI NVIC，本层持有强符号
 */
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PB4_E2B_Pin);
}

/**
 * @brief  EXTI9_5中断服务函数（E2A=PB5,E1B=PB6,E1A=PB7）
 * @note   .ioc未勾选EXTI NVIC，本层持有强符号
 */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PB5_E2A_Pin);
    HAL_GPIO_EXTI_IRQHandler(PB6_E1B_Pin);
    HAL_GPIO_EXTI_IRQHandler(PB7_E1A_Pin);
}
