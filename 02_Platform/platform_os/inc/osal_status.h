/**
 * @file  osal_status.h
 * @brief OSAL public status codes independent of the RTOS implementation.
 */

#ifndef OSAL_STATUS_H
#define OSAL_STATUS_H

typedef enum {
    OSAL_OK = 0,
    OSAL_ERR_PARAM,
    OSAL_ERR_TIMEOUT,
    OSAL_ERR_NO_MEMORY,
    OSAL_ERR_BUSY,
    OSAL_ERR_NOT_SUPPORTED,
    OSAL_ERR_STATE,
    OSAL_ERR_FAIL
} osal_status_t;

#endif /* OSAL_STATUS_H */
