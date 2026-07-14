/**
 * @file  osal_timer.c
 * @brief Validate software timer semantics before entering the OS backend.
 */

#include "osal_timer.h"

#include <string.h>

#include "os_internal.h"

osal_status_t osal_timer_create(osal_timer_handle_t *timer_handle,
                                const char *timer_name,
                                osal_tick_type_t timer_period_ms,
                                uint8_t auto_reload,
                                osal_timer_cb_function_t timer_cb,
                                void *argument)
{
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    *timer_handle = NULL;
    if (timer_name == NULL || timer_cb == NULL || timer_period_ms == 0U) {
        return OSAL_ERR_PARAM;
    }
    if (auto_reload > 1U ||
        memchr(timer_name, '\0', OSAL_NAME_MAX_LEN) == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_create(timer_handle,
                           timer_name,
                           timer_period_ms,
                           auto_reload,
                           timer_cb,
                           argument);
}

osal_status_t osal_timer_start(osal_timer_handle_t timer_handle,
                               osal_tick_type_t timeout_ms)
{
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_start(timer_handle, timeout_ms);
}

osal_status_t osal_timer_stop(osal_timer_handle_t timer_handle,
                              osal_tick_type_t timeout_ms)
{
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_stop(timer_handle, timeout_ms);
}

osal_status_t osal_timer_period_change(osal_timer_handle_t timer_handle,
                                       osal_tick_type_t new_period_ms,
                                       osal_tick_type_t timeout_ms)
{
    if (timer_handle == NULL || new_period_ms == 0U) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_change(timer_handle, new_period_ms, timeout_ms);
}

osal_status_t osal_timer_delete(osal_timer_handle_t timer_handle,
                                osal_tick_type_t timeout_ms)
{
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_delete(timer_handle, timeout_ms);
}

osal_status_t osal_timer_reset(osal_timer_handle_t timer_handle,
                               osal_tick_type_t timeout_ms)
{
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_reset(timer_handle, timeout_ms);
}

osal_status_t osal_timer_period_get(osal_timer_handle_t timer_handle,
                                    osal_tick_type_t *period_ms)
{
    if (period_ms == NULL) {
        return OSAL_ERR_PARAM;
    }

    *period_ms = 0U;
    if (timer_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_timer_period(timer_handle, period_ms);
}
