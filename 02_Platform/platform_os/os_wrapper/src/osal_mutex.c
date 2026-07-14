/**
 * @file  osal_mutex.c
 * @brief Validate stable mutex semantics before entering the OS backend.
 */

#include "osal_mutex.h"

#include "os_internal.h"

osal_status_t osal_mutex_create(osal_mutex_handle_t *mutex_handle)
{
    if (mutex_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    *mutex_handle = NULL;
    return os_mutex_create(mutex_handle);
}

osal_status_t osal_mutex_delete(osal_mutex_handle_t mutex_handle)
{
    if (mutex_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_mutex_delete(mutex_handle);
}

osal_status_t osal_mutex_give(osal_mutex_handle_t mutex_handle)
{
    if (mutex_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_mutex_give(mutex_handle);
}

osal_status_t osal_mutex_take(osal_mutex_handle_t mutex_handle,
                              osal_tick_type_t timeout_ms)
{
    if (mutex_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_mutex_take(mutex_handle, timeout_ms);
}
