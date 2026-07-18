/**
 * @file    encoder_port.c
 * @brief   MCU编码器Port实现（STM32F411，A相上升沿1倍频解码）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - 引脚模式由MX_GPIO_Init配置：A相(PB7/PB5)上升沿EXTI，
 *            B相(PB6/PB4)普通输入，本层不重复初始化
 *          - 高CPR电机（约10226计数/转）中断量大，ISR走EXTI->PR与
 *            IDR直读，不经HAL_GPIO_EXTI_IRQHandler回调链，最短中断
 *          - .ioc未勾选EXTI的NVIC，EXTI9_5_IRQHandler由本层唯一
 *            持有；若日后CubeMX勾选NVIC生成同名函数，必须删除一份；
 *            后续非编码器EXTI(线5..9)需求也在此扩展路由
 *          - NVIC优先级固定为ENC_EXTI_NVIC_PRIO，必须与TIM9控制节拍
 *            相同：EXTI计数ISR与10ms控制ISR互不抢占，编码器计数的
 *            累加(读改写)与读清零才无竞争
 *          - 板级标签注意：main.h中PB6宏名为PB6_E2B_Pin，物理上是
 *            编码器1(左轮A电机)的B相，.ioc标签待纠正
 */

#include "encoder_port.h"

#include "main.h"
#include "platform_def.h"

/** EXTI的NVIC抢占优先级，必须与TIM9(TIM1_BRK_TIM9_IRQn)一致。 */
#define ENC_EXTI_NVIC_PRIO (4U)

/** 本层持有的EXTI9_5线掩码：E1A=PB7与E2A=PB5。 */
#define ENC_EXTI9_5_MASK ((uint32_t)(PB7_E1A_Pin | PB5_E2A_Pin))

/** 逻辑编码器到物理引脚的映射项；A相=EXTI上升沿，B相=普通输入。 */
typedef struct {
    GPIO_TypeDef *port_a; /* A相GPIO端口 */
    uint16_t      pin_a;  /* A相HAL引脚掩码，=EXTI线位 */
    GPIO_TypeDef *port_b; /* B相GPIO端口 */
    uint16_t      pin_b;  /* B相HAL引脚掩码 */
    IRQn_Type     irqn;   /* A相所在EXTI中断线 */
} encoder_map_t;

/** 逻辑编码器表，ID=下标。 */
static const encoder_map_t s_enc_map[EN_CORE_ENCODER_NUM] = {
    /* EN_CORE_ENCODER_1：A=PB7，B=PB6(宏名E2B，物理E1的B相) */
    { PB7_E1A_GPIO_Port, PB7_E1A_Pin,
      PB6_E2B_GPIO_Port, PB6_E2B_Pin, EXTI9_5_IRQn },
    /* EN_CORE_ENCODER_2：A=PB5，B=PB4 */
    { PB5_E2A_GPIO_Port, PB5_E2A_Pin,
      PB4_E2B_GPIO_Port, PB4_E2B_Pin, EXTI9_5_IRQn },
};

/** 各编码器注册的步进回调（Adapter ISR入口）。 */
static core_encoder_step_cb_t s_cb[EN_CORE_ENCODER_NUM];

/** 1=该编码器已启动上报（ISR读/线程写）。 */
static volatile uint8_t s_started[EN_CORE_ENCODER_NUM];

/**
 * @brief  单编码器A相上升沿事件：采样B相定方向并上报
 * @param  id 逻辑编码器实例
 * @note   ISR上下文；正交90度相位下A上升沿时B相电平即方向：
 *         B低=+1，B高=-1（极性由板级配置在Adapter侧修正）
 */
static void encoder_step_isr(en_core_encoder_t id)
{
    const encoder_map_t *map = &s_enc_map[id]; /* 本路映射 */
    int8_t               step;                 /* 本次计数步进 */

    if ((s_started[id] == 0U) || (s_cb[id] == NULL)) {
        return;
    }

    /* 一条IDR读指令取B相电平，避免HAL读引脚开销 */
    step = ((map->port_b->IDR & (uint32_t)map->pin_b) == 0U)
         ? (int8_t)1 : (int8_t)-1;
    s_cb[id](step);
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

    if (id >= EN_CORE_ENCODER_NUM) {
        return PLATFORM_ERR_PARAM;
    }
    map = &s_enc_map[id];

    /* 丢弃启动前挂起的边沿再放行上报 */
    __HAL_GPIO_EXTI_CLEAR_IT(map->pin_a);
    s_started[id] = 1U;

    HAL_NVIC_SetPriority(map->irqn, ENC_EXTI_NVIC_PRIO, 0U);
    HAL_NVIC_EnableIRQ(map->irqn);
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
 * @brief  EXTI9_5中断服务函数（E1A=PB7，E2A=PB5）
 * @note   .ioc未勾选EXTI NVIC，本层持有强符号；
 *         直读直清PR且只动编码器线位，最短中断
 */
void EXTI9_5_IRQHandler(void)
{
    uint32_t pending; /* 本层持有线的挂起位图 */

    pending  = EXTI->PR & ENC_EXTI9_5_MASK;
    EXTI->PR = pending; /* 写1清除，不影响其他线 */

    if ((pending & (uint32_t)PB7_E1A_Pin) != 0U) {
        encoder_step_isr(EN_CORE_ENCODER_1);
    }
    if ((pending & (uint32_t)PB5_E2A_Pin) != 0U) {
        encoder_step_isr(EN_CORE_ENCODER_2);
    }
}
