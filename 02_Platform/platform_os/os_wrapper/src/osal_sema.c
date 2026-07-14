/**
 * @file  osal_sema.c
 * @brief Validate semaphore semantics before entering the OS backend.
 */

#include "osal_sema.h"

#include "os_internal.h"

osal_status_t osal_sema_counting_create(osal_sema_handle_t *sema_handle,
                                        uint32_t max_count,
                                        uint32_t init_count)
{
    if (sema_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    *sema_handle = NULL;
    if (max_count == 0U || init_count > max_count) {
        return OSAL_ERR_PARAM;
    }

    return os_sema_create_count(sema_handle, max_count, init_count);
}

osal_status_t osal_sema_binary_create(osal_sema_handle_t *sema_handle)
{
    if (sema_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    *sema_handle = NULL;
    return os_sema_create_bin(sema_handle);
}

osal_status_t osal_sema_delete(osal_sema_handle_t sema_handle)
{
    if (sema_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_sema_delete(sema_handle);
}

osal_status_t osal_sema_give(osal_sema_handle_t sema_handle)
{
    if (sema_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_sema_give(sema_handle);
}

osal_status_t osal_sema_take(osal_sema_handle_t sema_handle,
                             osal_tick_type_t timeout_ms)
{
    if (sema_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_sema_take(sema_handle, timeout_ms);
}
