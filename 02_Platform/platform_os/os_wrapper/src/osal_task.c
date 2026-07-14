/**
 * @file  osal_task.c
 * @brief Validate stable task semantics before entering the OS backend.
 */

#include "osal_task.h"

#include <string.h>

#include "os_internal.h"

osal_status_t osal_task_create(const char *task_name,
                               osal_task_entry entry,
                               size_t stack_size,
                               osal_priority_t priority,
                               osal_task_handle_t *task_handle,
                               void *argument)
{
    if (task_handle != NULL) {
        *task_handle = NULL;
    }
    if (task_name == NULL || entry == NULL || stack_size == 0U) {
        return OSAL_ERR_PARAM;
    }
    if (memchr(task_name, '\0', OSAL_NAME_MAX_LEN) == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_task_create(task_name,
                          entry,
                          stack_size,
                          priority,
                          task_handle,
                          argument);
}

osal_status_t osal_task_delete(osal_task_handle_t task_handle)
{
    return os_task_delete(task_handle);
}

osal_status_t osal_task_start(void)
{
    return os_task_start();
}

osal_status_t osal_task_suspend(osal_task_handle_t task_handle)
{
    return os_task_suspend(task_handle);
}

osal_status_t osal_task_resume(osal_task_handle_t task_handle)
{
    if (task_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_task_resume(task_handle);
}

osal_status_t osal_task_suspend_all(void)
{
    return os_task_suspend_all();
}

osal_status_t osal_task_delay(uint32_t ticks)
{
    return os_task_delay(ticks);
}

osal_status_t osal_task_delay_ms(uint32_t delay_ms)
{
    return os_task_delay_ms(delay_ms);
}

osal_status_t osal_task_wait_until(osal_tick_type_t *last_wake,
                                   uint32_t period_ms)
{
    if (last_wake == NULL || period_ms == 0U ||
        period_ms == OSAL_MAX_DELAY) {
        return OSAL_ERR_PARAM;
    }

    return os_task_wait_until(last_wake, period_ms);
}

osal_status_t osal_enter_critical(void)
{
    return os_critical_enter();
}

osal_status_t osal_exit_critical(void)
{
    return os_critical_exit();
}

osal_status_t osal_port_yield(void)
{
    return os_task_yield();
}

osal_status_t osal_task_enable_interrupts(void)
{
    return os_irq_enable();
}

osal_status_t osal_task_disable_interrupts(void)
{
    return os_irq_disable();
}

osal_tick_type_t osal_task_get_tick_count(void)
{
    return os_task_get_tick();
}

osal_task_handle_t osal_task_get_current_handle(void)
{
    return os_task_get_current();
}
