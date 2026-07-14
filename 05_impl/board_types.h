/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file board_types.h
 *
 * @par dependencies
 * - None
 *
 * @author Jack | R&D Dept. | EternalChip
 *
 * @brief Provide the board base types for STM32F411 platform.
 *
 * This file is the board/compiler type source. It does not include stdint.h
 * or other standard library headers, so platform_common can control the public
 * type vocabulary used by upper layers.
 *
 * @version V1.0 2026-05-10
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BOARD_TYPES_H__
#define __BOARD_TYPES_H__

//******************************** Defines **********************************//

typedef signed char         int8;
typedef unsigned char       uint8;
typedef signed short        int16;
typedef unsigned short      uint16;
typedef signed int          int32;
typedef unsigned int        uint32;
typedef signed long long    int64;
typedef unsigned long long  uint64;
typedef float               float32;
typedef double              float64;

typedef uint8               bool;

#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif

#endif /* __BOARD_TYPES_H__ */
