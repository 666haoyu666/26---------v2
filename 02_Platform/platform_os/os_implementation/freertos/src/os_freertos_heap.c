/**
 * @file  os_freertos_heap.c
 * @brief Map private OS heap operations to the FreeRTOS heap.
 */

#include "os_freertos_internal.h"

void *os_heap_alloc(size_t wanted_size)
{
    if (os_freertos_in_isr() != 0U) {
        return NULL;
    }

    return pvPortMalloc(wanted_size);
}

void os_heap_release(void *ptr)
{
    if (os_freertos_in_isr() != 0U) {
        return;
    }

    vPortFree(ptr);
}
