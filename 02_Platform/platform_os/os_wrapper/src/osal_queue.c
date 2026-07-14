/**
 * @file  osal_queue.c
 * @brief Validate stable queue semantics before entering the OS backend.
 */

#include "osal_queue.h"

#include "os_internal.h"

osal_status_t osal_queue_create(size_t queue_depth,
                                size_t data_size,
                                osal_queue_handle_t *queue_handle)
{
    if (queue_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    *queue_handle = NULL;
    if (queue_depth == 0U || data_size == 0U) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_create(queue_depth, data_size, queue_handle);
}

osal_status_t osal_queue_delete(osal_queue_handle_t queue_handle)
{
    if (queue_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_delete(queue_handle);
}

osal_status_t osal_queue_send(osal_queue_handle_t queue_handle,
                              const void *data,
                              osal_tick_type_t timeout_ms)
{
    if (queue_handle == NULL || data == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_send(queue_handle, data, timeout_ms);
}

osal_status_t osal_queue_receive(osal_queue_handle_t queue_handle,
                                 void *data,
                                 osal_tick_type_t timeout_ms)
{
    if (queue_handle == NULL || data == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_receive(queue_handle, data, timeout_ms);
}

osal_status_t osal_queue_peek(osal_queue_handle_t queue_handle,
                              void *data,
                              osal_tick_type_t timeout_ms)
{
    if (queue_handle == NULL || data == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_peek(queue_handle, data, timeout_ms);
}

osal_status_t osal_queue_count(osal_queue_handle_t queue_handle,
                               uint32_t *count)
{
    if (count == NULL) {
        return OSAL_ERR_PARAM;
    }

    *count = 0U;
    if (queue_handle == NULL) {
        return OSAL_ERR_PARAM;
    }

    return os_queue_count(queue_handle, count);
}
