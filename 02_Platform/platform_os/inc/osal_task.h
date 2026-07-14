/**
 * @file  osal_task.h
 * @brief OSAL task, scheduler, tick and critical-section API.
 */

#ifndef OSAL_TASK_H
#define OSAL_TASK_H

#include "osal_status.h"
#include "osal_types.h"

/**
 * @brief  Create a task; stack_size is expressed in bytes.
 * @param  task_name   Task name shorter than OSAL_NAME_MAX_LEN.
 * @param  entry       Task entry function.
 * @param  stack_size  Requested stack size in bytes.
 * @param  priority    Native-independent numeric priority.
 * @param  task_handle Optional output handle; may be NULL.
 * @param  argument    Task entry argument.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_task_create(const char *task_name,
                               osal_task_entry entry,
                               size_t stack_size,
                               osal_priority_t priority,
                               osal_task_handle_t *task_handle,
                               void *argument);

/** Delete a task; NULL selects the current task. */
osal_status_t osal_task_delete(osal_task_handle_t task_handle);

/** Start the scheduler; success does not return. */
osal_status_t osal_task_start(void);

/** Suspend a task; NULL selects the current task. */
osal_status_t osal_task_suspend(osal_task_handle_t task_handle);

/** Resume a task, automatically using the ISR-safe path when required. */
osal_status_t osal_task_resume(osal_task_handle_t task_handle);

/** Suspend scheduler dispatch without disabling interrupts. */
osal_status_t osal_task_suspend_all(void);

/** Delay the current task by a raw OS tick count. */
osal_status_t osal_task_delay(uint32_t ticks);

/** Delay the current task by a millisecond duration. */
osal_status_t osal_task_delay_ms(uint32_t delay_ms);

/**
 * @brief  Maintain a fixed task period from the previous wake tick.
 * @param  last_wake Last wake tick, updated after a successful wait.
 * @param  period_ms Period in ms; must be nonzero and finite.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_task_wait_until(osal_tick_type_t *last_wake,
                                   uint32_t period_ms);

/** Enter a task-context critical section. */
osal_status_t osal_enter_critical(void);

/** Exit a task-context critical section. */
osal_status_t osal_exit_critical(void);

/** Yield the processor from task context. */
osal_status_t osal_port_yield(void);

/** Enable interrupts through the RTOS port. */
osal_status_t osal_task_enable_interrupts(void);

/** Disable interrupts through the RTOS port. */
osal_status_t osal_task_disable_interrupts(void);

/** Get the current native tick value, including from ISR context. */
osal_tick_type_t osal_task_get_tick_count(void);

/** Get the currently running task handle. */
osal_task_handle_t osal_task_get_current_handle(void);

#endif /* OSAL_TASK_H */
