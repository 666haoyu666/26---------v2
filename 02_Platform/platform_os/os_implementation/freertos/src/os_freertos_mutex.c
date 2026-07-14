/**
 * @file  os_freertos_mutex.c
 * @brief Map private OS mutex operations to FreeRTOS semaphores.
 */

#include "os_freertos_internal.h"

osal_status_t os_mutex_create(osal_mutex_handle_t *mutex_handle)
{
    SemaphoreHandle_t handle; /* Newly created native mutex. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    handle = xSemaphoreCreateMutex();
    if (handle == NULL) {
        return OSAL_ERR_NO_MEMORY;
    }

    *mutex_handle = (osal_mutex_handle_t)handle;
    return OSAL_OK;
}

osal_status_t os_mutex_delete(osal_mutex_handle_t mutex_handle)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vSemaphoreDelete((SemaphoreHandle_t)mutex_handle);
    return OSAL_OK;
}

osal_status_t os_mutex_give(osal_mutex_handle_t mutex_handle)
{
    BaseType_t status; /* Native release result. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    status = xSemaphoreGive((SemaphoreHandle_t)mutex_handle);
    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_STATE;
}

osal_status_t os_mutex_take(osal_mutex_handle_t mutex_handle,
                            osal_tick_type_t timeout_ms)
{
    BaseType_t status; /* Native acquisition result. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    status = xSemaphoreTake((SemaphoreHandle_t)mutex_handle,
                            os_freertos_ms_ticks(timeout_ms));
    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_TIMEOUT;
}
