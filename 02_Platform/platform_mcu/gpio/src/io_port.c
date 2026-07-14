/**
 * @file    io_port.c
 * @brief   MCU逻辑IO统一Port实现（STM32F411 HAL）
 * @note    - 逻辑ID=映射表下标，映射唯一依据《核心引脚分配表》
 *          - 引脚模式由MX_GPIO_Init配置，本层不重复初始化
 *          - 输入组一次IDR读取加位序打包，保证组内同拍
 */

#include "io_port.h"

#include "main.h"
#include "platform_def.h"

/* 灰度输入组IDR掩码：PA0..PA7连续8路，bit i=PAi=槽位i */
#define IOGRP_LSENSOR_MASK                              \
    ((uint32_t)(PA0_LSENSOR_0_Pin | PA1_LSENSOR_1_Pin | \
                PA2_LSENSOR_2_Pin | PA3_LSENSOR_3_Pin | \
                PA4_LSENSOR_4_Pin | PA5_LSENSOR_5_Pin | \
                PA6_LSENSOR_6_Pin | PA7_LSENSOR_7_Pin))

#define IOGRP_LSENSOR_SHIFT (0U) /* 灰度组bit0在IDR中的位号 */

/** 单路逻辑IO到物理引脚的映射项。 */
typedef struct {
    GPIO_TypeDef *port;   /* 物理GPIO端口 */
    uint16_t      pin;    /* HAL引脚掩码 */
    uint8_t       is_out; /* 1=输出，0=输入 */
} io_map_t;

/** 逻辑输入组映射项；组内引脚必须同端口且位序连续。 */
typedef struct {
    GPIO_TypeDef *port;  /* 物理GPIO端口 */
    uint32_t      mask;  /* 组内引脚IDR掩码 */
    uint8_t       shift; /* 组内bit0对应的IDR位号 */
} iogroup_map_t;

/** 单路逻辑IO表，ID=下标：0..3为电机方向脚。 */
static const io_map_t s_io_map[] = {
    { PB12_AIN1_GPIO_Port, PB12_AIN1_Pin, 1U }, /* 0: AIN1 */
    { PB13_AIN2_GPIO_Port, PB13_AIN2_Pin, 1U }, /* 1: AIN2 */
    { PB14_BIN1_GPIO_Port, PB14_BIN1_Pin, 1U }, /* 2: BIN1 */
    { PB15_BIN2_GPIO_Port, PB15_BIN2_Pin, 1U }, /* 3: BIN2 */
};

/** 输入组表，ID=下标：0对齐板级BOARD_LSENSOR_IOGROUP。 */
static const iogroup_map_t s_iogroup_map[] = {
    /* 0: 灰度组PA0..PA7 */
    { PA0_LSENSOR_0_GPIO_Port, IOGRP_LSENSOR_MASK,
      IOGRP_LSENSOR_SHIFT },
};

/**
 * @brief  写单路逻辑IO电平
 * @param  id    单路逻辑IO标识，必须为输出IO
 * @param  level 目标逻辑电平
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 */
platform_err_t core_io_write(
    core_io_id_t id,
    core_io_level_t level)
{
    GPIO_PinState pin_state; /* 写入HAL的引脚电平 */

    if (id >= ARRAY_SIZE(s_io_map) || s_io_map[id].is_out == 0U) {
        return PLATFORM_ERR_PARAM;
    }
    if (level != CORE_IO_LOW && level != CORE_IO_HIGH) {
        return PLATFORM_ERR_PARAM;
    }

    pin_state = (level == CORE_IO_HIGH) ? GPIO_PIN_SET
                                        : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(s_io_map[id].port, s_io_map[id].pin,
                      pin_state);
    return PLATFORM_ERR_OK;
}

/**
 * @brief  读取单路逻辑IO当前电平
 * @param  id    单路逻辑IO标识
 * @param  level 输出逻辑电平，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 * @note   读取失败时不修改输出内容
 */
platform_err_t core_io_read(
    core_io_id_t id,
    core_io_level_t *level)
{
    GPIO_PinState pin_state; /* HAL返回的引脚电平 */

    if (id >= ARRAY_SIZE(s_io_map) || level == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    pin_state = HAL_GPIO_ReadPin(s_io_map[id].port,
                                 s_io_map[id].pin);
    *level = (pin_state == GPIO_PIN_SET) ? CORE_IO_HIGH
                                         : CORE_IO_LOW;
    return PLATFORM_ERR_OK;
}

/**
 * @brief  翻转单路逻辑输出电平
 * @param  id 单路逻辑IO标识，必须为输出IO
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 */
platform_err_t core_io_toggle(core_io_id_t id)
{
    if (id >= ARRAY_SIZE(s_io_map) || s_io_map[id].is_out == 0U) {
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_TogglePin(s_io_map[id].port, s_io_map[id].pin);
    return PLATFORM_ERR_OK;
}

/**
 * @brief  读取逻辑输入组当前一拍原始电平位图
 * @param  id     逻辑输入组标识
 * @param  bitmap 位i对应组内第i路，1表示高电平，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM
 * @note   高于组宽的位补0；读取失败时不修改输出内容
 */
platform_err_t core_iogroup_read(
    core_iogroup_id_t id,
    uint32_t *bitmap)
{
    uint32_t idr_snap; /* 本拍IDR快照 */

    if (id >= ARRAY_SIZE(s_iogroup_map) || bitmap == NULL) {
        return PLATFORM_ERR_PARAM;
    }

    /* 一条IDR读指令保证组内各路同拍 */
    idr_snap = s_iogroup_map[id].port->IDR;
    *bitmap  = (idr_snap & s_iogroup_map[id].mask) >>
               s_iogroup_map[id].shift;
    return PLATFORM_ERR_OK;
}
