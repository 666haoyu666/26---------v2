/**
 * @file  osal_heap.h
 * @brief OSAL heap access for modules explicitly allowed to allocate memory.
 */

#ifndef OSAL_HEAP_H
#define OSAL_HEAP_H

#include "osal_types.h"

/** Allocate bytes from the RTOS heap; task context only. */
void *osal_heap_malloc(size_t wanted_size);

/** Release an RTOS heap block from task context; NULL is ignored. */
void osal_heap_free(void *ptr);

#endif /* OSAL_HEAP_H */
