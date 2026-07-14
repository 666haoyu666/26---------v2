/**
 * @file  os_freertos_timer.c
 * @brief Map private OS timer operations and callbacks to FreeRTOS.
 */

#include "os_freertos_internal.h"

#include <string.h>

typedef struct {
    char                      timer_name[OSAL_NAME_MAX_LEN];
    osal_timer_cb_function_t  func;
    void                      *arg;
    osal_timer_handle_t       timer_handle;
} os_timer_record_t;

static void os_timer_callback(TimerHandle_t native_timer)
{
    os_timer_record_t *record; /* OSAL callback metadata. */

    record = (os_timer_record_t *)pvTimerGetTimerID(native_timer);
    if (record == NULL || record->func == NULL) {
        return;
    }

    record->func(record->timer_handle, record->arg);
}

static void os_timer_free_cb(void *record, uint32_t unused)
{
    (void)unused;
    vPortFree(record);
}

osal_status_t os_timer_create(osal_timer_handle_t *timer_handle,
                              const char *timer_name,
                              osal_tick_type_t timer_period_ms,
                              uint8_t auto_reload,
                              osal_timer_cb_function_t timer_cb,
                              void *argument)
{
    os_timer_record_t *record; /* Persistent callback metadata. */
    TimerHandle_t native_timer; /* Newly created native timer. */
    size_t name_len;             /* Validated timer name length. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    record = pvPortMalloc(sizeof(*record));
    if (record == NULL) {
        return OSAL_ERR_NO_MEMORY;
    }

    name_len = strlen(timer_name);
    memset(record, 0, sizeof(*record));
    memcpy(record->timer_name, timer_name, name_len + 1U);
    record->func = timer_cb;
    record->arg = argument;

    native_timer = xTimerCreate(record->timer_name,
                                os_freertos_ms_ticks(timer_period_ms),
                                (UBaseType_t)auto_reload,
                                record,
                                os_timer_callback);
    if (native_timer == NULL) {
        vPortFree(record);
        return OSAL_ERR_NO_MEMORY;
    }

    *timer_handle = (osal_timer_handle_t)native_timer;
    record->timer_handle = *timer_handle;
    return OSAL_OK;
}

osal_status_t os_timer_start(osal_timer_handle_t timer_handle,
                             osal_tick_type_t timeout_ms)
{
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native timer command result. */

    if (os_freertos_in_isr() != 0U) {
        status = xTimerStartFromISR((TimerHandle_t)timer_handle,
                                    &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xTimerStart((TimerHandle_t)timer_handle,
                             os_freertos_ms_ticks(timeout_ms));
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_timer_stop(osal_timer_handle_t timer_handle,
                            osal_tick_type_t timeout_ms)
{
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native timer command result. */

    if (os_freertos_in_isr() != 0U) {
        status = xTimerStopFromISR((TimerHandle_t)timer_handle,
                                   &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xTimerStop((TimerHandle_t)timer_handle,
                            os_freertos_ms_ticks(timeout_ms));
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_timer_change(osal_timer_handle_t timer_handle,
                              osal_tick_type_t new_period_ms,
                              osal_tick_type_t timeout_ms)
{
    TickType_t new_period;           /* Converted native timer period. */
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native timer command result. */

    new_period = os_freertos_ms_ticks(new_period_ms);
    if (os_freertos_in_isr() != 0U) {
        status = xTimerChangePeriodFromISR((TimerHandle_t)timer_handle,
                                           new_period,
                                           &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xTimerChangePeriod((TimerHandle_t)timer_handle,
                                    new_period,
                                    os_freertos_ms_ticks(timeout_ms));
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_timer_delete(osal_timer_handle_t timer_handle,
                              osal_tick_type_t timeout_ms)
{
    TimerHandle_t native_timer; /* Native timer being deleted. */
    void *record;               /* Callback metadata freed in daemon order. */
    TickType_t wait_ticks;      /* Timer command queue wait duration. */
    BaseType_t status;          /* Native timer command result. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    native_timer = (TimerHandle_t)timer_handle;
    wait_ticks = os_freertos_ms_ticks(timeout_ms);
    record = pvTimerGetTimerID(native_timer);
    status = xTimerDelete(native_timer, wait_ticks);
    if (status != pdPASS) {
        return OSAL_ERR_BUSY;
    }

    status = xTimerPendFunctionCall(os_timer_free_cb,
                                    record,
                                    0U,
                                    wait_ticks);
    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_timer_reset(osal_timer_handle_t timer_handle,
                             osal_tick_type_t timeout_ms)
{
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native timer command result. */

    if (os_freertos_in_isr() != 0U) {
        status = xTimerResetFromISR((TimerHandle_t)timer_handle,
                                    &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xTimerReset((TimerHandle_t)timer_handle,
                             os_freertos_ms_ticks(timeout_ms));
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_timer_period(osal_timer_handle_t timer_handle,
                              osal_tick_type_t *period_ms)
{
    TickType_t period_ticks; /* Current native timer period. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    period_ticks = xTimerGetPeriod((TimerHandle_t)timer_handle);
    *period_ms = os_freertos_ticks_ms(period_ticks);
    return OSAL_OK;
}
