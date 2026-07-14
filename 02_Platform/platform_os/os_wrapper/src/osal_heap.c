/**
 * @file  osal_heap.c
 * @brief Validate OSAL heap requests before entering the OS backend.
 */

#include "osal_heap.h"

#include "os_internal.h"

void *osal_heap_malloc(size_t wanted_size)
{
    if (wanted_size == 0U) {
        return NULL;
    }

    return os_heap_alloc(wanted_size);
}

void osal_heap_free(void *ptr)
{
    if (ptr == NULL) {
        return;
    }

    os_heap_release(ptr);
}
