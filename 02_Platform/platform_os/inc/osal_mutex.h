/**
 * @file  osal_mutex.h
 * @brief OSAL mutex API; every operation is task-context only.
 */

#ifndef OSAL_MUTEX_H
#define OSAL_MUTEX_H

#include "osal_status.h"
#include "osal_types.h"

/**
 * @brief  Create a mutex.
 * @param  mutex_handle Receives the opaque mutex handle.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_mutex_create(osal_mutex_handle_t *mutex_handle);

/**
 * @brief  Delete a mutex.
 * @param  mutex_handle Mutex handle to delete.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_mutex_delete(osal_mutex_handle_t mutex_handle);

/**
 * @brief  Release a mutex from task context.
 * @param  mutex_handle Mutex handle to release.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_mutex_give(osal_mutex_handle_t mutex_handle);

/**
 * @brief  Acquire a mutex with a millisecond timeout.
 * @param  mutex_handle Mutex handle to acquire.
 * @param  timeout_ms   Timeout in ms, or OSAL_MAX_DELAY.
 * @retval OSAL_OK or an OSAL error status.
 */
osal_status_t osal_mutex_take(osal_mutex_handle_t mutex_handle,
                              osal_tick_type_t timeout_ms);

#endif /* OSAL_MUTEX_H */
