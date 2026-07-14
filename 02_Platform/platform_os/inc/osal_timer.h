/**
 * @file  osal_timer.h
 * @brief OSAL software timer API with callback argument adaptation.
 */

#ifndef OSAL_TIMER_H
#define OSAL_TIMER_H

#include "osal_status.h"
#include "osal_types.h"

/**
 * @brief  Create a software timer; period and later timeouts use ms.
 * @param  timer_handle    Receives the opaque timer handle.
 * @param  timer_name      Name copied by OSAL, including terminator.
 * @param  timer_period_ms Initial nonzero period in ms.
 * @param  auto_reload     0 for one-shot, 1 for automatic reload.
 * @param  timer_cb        Callback executed by the timer service task.
 * @param  argument        User callback argument.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_timer_create(osal_timer_handle_t *timer_handle,
                                const char *timer_name,
                                osal_tick_type_t timer_period_ms,
                                uint8_t auto_reload,
                                osal_timer_cb_function_t timer_cb,
                                void *argument);

/** Start a timer; automatically uses the ISR-safe command when required. */
osal_status_t osal_timer_start(osal_timer_handle_t timer_handle,
                               osal_tick_type_t timeout_ms);

/** Stop a timer; automatically uses the ISR-safe command when required. */
osal_status_t osal_timer_stop(osal_timer_handle_t timer_handle,
                              osal_tick_type_t timeout_ms);

/** Change a timer period in ms. */
osal_status_t osal_timer_period_change(osal_timer_handle_t timer_handle,
                                       osal_tick_type_t new_period_ms,
                                       osal_tick_type_t timeout_ms);

/** Delete a timer from task context. */
osal_status_t osal_timer_delete(osal_timer_handle_t timer_handle,
                                osal_tick_type_t timeout_ms);

/** Reset a timer; automatically uses the ISR-safe command when required. */
osal_status_t osal_timer_reset(osal_timer_handle_t timer_handle,
                               osal_tick_type_t timeout_ms);

/** Read the timer period in ms; output is zeroed on failure. */
osal_status_t osal_timer_period_get(osal_timer_handle_t timer_handle,
                                    osal_tick_type_t *period_ms);

#endif /* OSAL_TIMER_H */
