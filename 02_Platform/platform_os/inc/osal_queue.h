/**
 * @file  osal_queue.h
 * @brief OSAL queue API with automatic task/ISR send and receive paths.
 */

#ifndef OSAL_QUEUE_H
#define OSAL_QUEUE_H

#include "osal_status.h"
#include "osal_types.h"

/** Create a queue with fixed depth and item size. */
osal_status_t osal_queue_create(size_t queue_depth,
                                size_t data_size,
                                osal_queue_handle_t *queue_handle);

/** Delete a queue from task context. */
osal_status_t osal_queue_delete(osal_queue_handle_t queue_handle);

/** Send one item; timeout_ms is ignored in ISR context. */
osal_status_t osal_queue_send(osal_queue_handle_t queue_handle,
                              const void *data,
                              osal_tick_type_t timeout_ms);

/** Receive one item; timeout_ms is ignored in ISR context. */
osal_status_t osal_queue_receive(osal_queue_handle_t queue_handle,
                                 void *data,
                                 osal_tick_type_t timeout_ms);

/** Peek one item without removal; timeout_ms is ignored in ISR context. */
osal_status_t osal_queue_peek(osal_queue_handle_t queue_handle,
                              void *data,
                              osal_tick_type_t timeout_ms);

/** Read the current queued item count; output is zeroed on failure. */
osal_status_t osal_queue_count(osal_queue_handle_t queue_handle,
                               uint32_t *count);

#endif /* OSAL_QUEUE_H */
