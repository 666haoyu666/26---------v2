/**
 * @file  os_freertos_internal.h
 * @brief FreeRTOS-only native includes, checks and time conversion helpers.
 */

#ifndef OS_FREERTOS_INTERNAL_H
#define OS_FREERTOS_INTERNAL_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "cmsis_compiler.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "os_internal.h"

#if (configSUPPORT_DYNAMIC_ALLOCATION != 1)
#error "OSAL requires FreeRTOS dynamic allocation"
#endif

#if (configUSE_MUTEXES != 1)
#error "OSAL mutex support requires configUSE_MUTEXES"
#endif

#if (configUSE_COUNTING_SEMAPHORES != 1)
#error "OSAL counting semaphores require configUSE_COUNTING_SEMAPHORES"
#endif

#if (configUSE_TIMERS != 1) || (INCLUDE_xTimerPendFunctionCall != 1)
#error "OSAL timers require timers and xTimerPendFunctionCall"
#endif

#if (INCLUDE_vTaskDelete != 1) || (INCLUDE_vTaskSuspend != 1) || \
    (INCLUDE_vTaskDelay != 1)
#error "OSAL tasks require delete, suspend and delay support"
#endif

#if (INCLUDE_xTaskResumeFromISR != 1) || \
    (INCLUDE_vTaskDelayUntil != 1)
#error "OSAL tasks require resume-from-ISR and delay-until support"
#endif

#if (configUSE_16_BIT_TICKS != 0)
#error "OSAL public ticks require 32-bit FreeRTOS TickType_t"
#endif

static inline uint8_t os_freertos_in_isr(void)
{
    return (__get_IPSR() != 0U) ? 1U : 0U;
}

static inline TickType_t os_freertos_ms_ticks(osal_tick_type_t time_ms)
{
    uint64_t ticks; /* Rounded-up native tick count. */

    if (time_ms == OSAL_MAX_DELAY) {
        return portMAX_DELAY;
    }

    ticks = ((uint64_t)time_ms * (uint64_t)configTICK_RATE_HZ + 999U) /
            1000U;
    if (ticks >= (uint64_t)portMAX_DELAY) {
        ticks = (uint64_t)portMAX_DELAY - 1U;
    }

    return (TickType_t)ticks;
}

static inline osal_tick_type_t os_freertos_ticks_ms(TickType_t ticks)
{
    uint64_t time_ms; /* Milliseconds after truncating fractional ticks. */

    time_ms = ((uint64_t)ticks * 1000U) /
              (uint64_t)configTICK_RATE_HZ;
    if (time_ms > (uint64_t)OSAL_MAX_DELAY) {
        return OSAL_MAX_DELAY;
    }

    return (osal_tick_type_t)time_ms;
}

#endif /* OS_FREERTOS_INTERNAL_H */
