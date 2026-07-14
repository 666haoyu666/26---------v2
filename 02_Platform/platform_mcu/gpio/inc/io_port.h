/**
 * @file    io_port.h
 * @brief   MCU逻辑IO统一Port接口
 * @note    逻辑IO及输入组到物理GPIO的映射只存在于同层io_port.c
 */

#ifndef IO_PORT_H
#define IO_PORT_H

#include "platform_error.h"
#include "platform_type.h"

typedef uint32_t core_io_id_t;
typedef uint32_t core_iogroup_id_t;

/** 逻辑IO电平。 */
typedef enum {
    CORE_IO_LOW = 0,
    CORE_IO_HIGH
} core_io_level_t;

/**
 * @brief  写单路逻辑IO电平
 * @param  id    单路逻辑IO标识
 * @param  level 目标逻辑电平
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 */
platform_err_t core_io_write(
    core_io_id_t id,
    core_io_level_t level);

/**
 * @brief  读取单路逻辑IO电平
 * @param  id    单路逻辑IO标识
 * @param  level 输出逻辑电平，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 * @note    读取失败时不修改输出内容
 */
platform_err_t core_io_read(
    core_io_id_t id,
    core_io_level_t *level);

/**
 * @brief  翻转单路逻辑输出电平
 * @param  id 单路逻辑IO标识
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 */
platform_err_t core_io_toggle(core_io_id_t id);

/**
 * @brief  读取逻辑输入组当前一拍原始电平位图
 * @param  id     逻辑输入组标识
 * @param  bitmap 位i对应组内第i路，1表示高电平，不可为NULL
 * @retval PLATFORM_ERR_OK / PLATFORM_ERR_PARAM / PLATFORM_ERR_FAIL
 * @note    高于组宽的位补0；读取失败时不修改输出内容
 */
platform_err_t core_iogroup_read(
    core_iogroup_id_t id,
    uint32_t *bitmap);

#endif /* IO_PORT_H */
