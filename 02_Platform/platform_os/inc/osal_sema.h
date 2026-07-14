/**
 * @file  osal_sema.h
 * @brief OSAL binary and counting semaphore API with ISR adaptation.
 */

#ifndef OSAL_SEMA_H
#define OSAL_SEMA_H

#include "osal_status.h"
#include "osal_types.h"

/** Create a counting semaphore with explicit maximum and initial counts. */
osal_status_t osal_sema_counting_create(osal_sema_handle_t *sema_handle,
                                        uint32_t max_count,
                                        uint32_t init_count);

/** Create an initially empty binary semaphore. */
osal_status_t osal_sema_binary_create(osal_sema_handle_t *sema_handle);

/** Delete a semaphore from task context. */
osal_status_t osal_sema_delete(osal_sema_handle_t sema_handle);

/** Give a semaphore, automatically choosing task or ISR semantics. */
osal_status_t osal_sema_give(osal_sema_handle_t sema_handle);

/** Take a semaphore; timeout_ms is ignored in ISR context. */
osal_status_t osal_sema_take(osal_sema_handle_t sema_handle,
                             osal_tick_type_t timeout_ms);

#endif /* OSAL_SEMA_H */
