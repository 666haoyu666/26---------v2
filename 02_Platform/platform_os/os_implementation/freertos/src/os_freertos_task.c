/**
 * @file  os_freertos_task.c
 * @brief Map private OS task and scheduler operations to FreeRTOS.
 */

#include "os_freertos_internal.h"

#include <string.h>

osal_status_t os_task_create(const char *task_name,
                             osal_task_entry entry,
                             size_t stack_size,
                             osal_priority_t priority,
                             osal_task_handle_t *task_handle,
                             void *argument)
{
    TaskHandle_t handle = NULL; /* Newly created native task. */
    size_t stack_depth;         /* Stack depth in native words. */
    BaseType_t status;          /* Native task creation result. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }
    if (memchr(task_name, '\0', configMAX_TASK_NAME_LEN) == NULL ||
        priority >= (osal_priority_t)configMAX_PRIORITIES) {
        return OSAL_ERR_PARAM;
    }

    stack_depth = stack_size / sizeof(StackType_t);
    if ((stack_size % sizeof(StackType_t)) != 0U) {
        stack_depth++;
    }
    if (stack_depth > (size_t)(configSTACK_DEPTH_TYPE)-1) {
        return OSAL_ERR_PARAM;
    }

    status = xTaskCreate(entry,
                         task_name,
                         (configSTACK_DEPTH_TYPE)stack_depth,
                         argument,
                         (UBaseType_t)priority,
                         (task_handle != NULL) ? &handle : NULL);
    if (status != pdPASS) {
        return OSAL_ERR_NO_MEMORY;
    }

    if (task_handle != NULL) {
        *task_handle = (osal_task_handle_t)handle;
    }
    return OSAL_OK;
}

osal_status_t os_task_delete(osal_task_handle_t task_handle)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskDelete((TaskHandle_t)task_handle);
    return OSAL_OK;
}

osal_status_t os_task_start(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskStartScheduler();
    return OSAL_ERR_NO_MEMORY;
}

osal_status_t os_task_suspend(osal_task_handle_t task_handle)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskSuspend((TaskHandle_t)task_handle);
    return OSAL_OK;
}

osal_status_t os_task_resume(osal_task_handle_t task_handle)
{
    BaseType_t task_woken; /* Whether the resumed task should preempt. */

    if (os_freertos_in_isr() != 0U) {
        task_woken = xTaskResumeFromISR((TaskHandle_t)task_handle);
        portYIELD_FROM_ISR(task_woken);
    } else {
        vTaskResume((TaskHandle_t)task_handle);
    }

    return OSAL_OK;
}

osal_status_t os_task_suspend_all(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskSuspendAll();
    return OSAL_OK;
}

osal_status_t os_task_delay(uint32_t ticks)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskDelay((TickType_t)ticks);
    return OSAL_OK;
}

osal_status_t os_task_delay_ms(uint32_t delay_ms)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vTaskDelay(os_freertos_ms_ticks((osal_tick_type_t)delay_ms));
    return OSAL_OK;
}

osal_status_t os_task_wait_until(osal_tick_type_t *last_wake,
                                 uint32_t period_ms)
{
    TickType_t wake_tick;    /* Native previous wake tick. */
    TickType_t period_ticks; /* Native period tick count. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    wake_tick = (TickType_t)*last_wake;
    period_ticks = os_freertos_ms_ticks((osal_tick_type_t)period_ms);
    vTaskDelayUntil(&wake_tick, period_ticks);
    *last_wake = (osal_tick_type_t)wake_tick;
    return OSAL_OK;
}

osal_status_t os_critical_enter(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    taskENTER_CRITICAL();
    return OSAL_OK;
}

osal_status_t os_critical_exit(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    taskEXIT_CRITICAL();
    return OSAL_OK;
}

osal_status_t os_task_yield(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    portYIELD();
    return OSAL_OK;
}

osal_status_t os_irq_enable(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    taskENABLE_INTERRUPTS();
    return OSAL_OK;
}

osal_status_t os_irq_disable(void)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    taskDISABLE_INTERRUPTS();
    return OSAL_OK;
}

osal_tick_type_t os_task_get_tick(void)
{
    TickType_t ticks; /* Current native tick count. */

    if (os_freertos_in_isr() != 0U) {
        ticks = xTaskGetTickCountFromISR();
    } else {
        ticks = xTaskGetTickCount();
    }

    return (osal_tick_type_t)ticks;
}

osal_task_handle_t os_task_get_current(void)
{
    return (osal_task_handle_t)xTaskGetCurrentTaskHandle();
}
