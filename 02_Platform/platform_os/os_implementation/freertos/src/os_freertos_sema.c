/**
 * @file  os_freertos_sema.c
 * @brief Map private OS semaphore operations to task/ISR FreeRTOS APIs.
 */

#include "os_freertos_internal.h"

osal_status_t os_sema_create_count(osal_sema_handle_t *sema_handle,
                                   uint32_t max_count,
                                   uint32_t init_count)
{
    SemaphoreHandle_t handle; /* Newly created native semaphore. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }
    if (max_count > (uint32_t)(UBaseType_t)-1) {
        return OSAL_ERR_PARAM;
    }

    handle = xSemaphoreCreateCounting((UBaseType_t)max_count,
                                      (UBaseType_t)init_count);
    if (handle == NULL) {
        return OSAL_ERR_NO_MEMORY;
    }

    *sema_handle = (osal_sema_handle_t)handle;
    return OSAL_OK;
}

osal_status_t os_sema_create_bin(osal_sema_handle_t *sema_handle)
{
    SemaphoreHandle_t handle; /* Newly created native semaphore. */

    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    handle = xSemaphoreCreateBinary();
    if (handle == NULL) {
        return OSAL_ERR_NO_MEMORY;
    }

    *sema_handle = (osal_sema_handle_t)handle;
    return OSAL_OK;
}

osal_status_t os_sema_delete(osal_sema_handle_t sema_handle)
{
    if (os_freertos_in_isr() != 0U) {
        return OSAL_ERR_NOT_SUPPORTED;
    }

    vSemaphoreDelete((SemaphoreHandle_t)sema_handle);
    return OSAL_OK;
}

osal_status_t os_sema_give(osal_sema_handle_t sema_handle)
{
    SemaphoreHandle_t handle; /* Native semaphore. */
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native semaphore result. */

    handle = (SemaphoreHandle_t)sema_handle;
    if (os_freertos_in_isr() != 0U) {
        status = xSemaphoreGiveFromISR(handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xSemaphoreGive(handle);
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_BUSY;
}

osal_status_t os_sema_take(osal_sema_handle_t sema_handle,
                           osal_tick_type_t timeout_ms)
{
    SemaphoreHandle_t handle; /* Native semaphore. */
    BaseType_t task_woken = pdFALSE; /* ISR preemption request. */
    BaseType_t status;               /* Native semaphore result. */

    handle = (SemaphoreHandle_t)sema_handle;
    if (os_freertos_in_isr() != 0U) {
        status = xSemaphoreTakeFromISR(handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        status = xSemaphoreTake(handle,
                                os_freertos_ms_ticks(timeout_ms));
    }

    return (status == pdPASS) ? OSAL_OK : OSAL_ERR_TIMEOUT;
}
