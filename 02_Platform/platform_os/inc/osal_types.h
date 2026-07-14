/**
 * @file  osal_types.h
 * @brief OSAL public scalar types and opaque resource handles.
 */

#ifndef OSAL_TYPES_H
#define OSAL_TYPES_H

#include <stddef.h>
#include <stdint.h>

#define OSAL_MAX_DELAY     (UINT32_MAX) /* 永久等待 */
#define OSAL_NAME_MAX_LEN  (16U)        /* 名称含结尾空字符的最大长度 */

typedef uint32_t osal_tick_type_t;
typedef uint32_t osal_priority_t;

typedef struct osal_task_obj   *osal_task_handle_t;
typedef struct osal_sema_obj   *osal_sema_handle_t;
typedef struct osal_mutex_obj  *osal_mutex_handle_t;
typedef struct osal_queue_obj  *osal_queue_handle_t;
typedef struct osal_timer_obj  *osal_timer_handle_t;

typedef void (*osal_task_entry)(void *argument);
typedef void (*osal_timer_cb_function_t)(osal_timer_handle_t timer_handle,
                                         void *argument);

#endif /* OSAL_TYPES_H */
