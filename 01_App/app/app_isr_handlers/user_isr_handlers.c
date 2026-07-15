/**
 * @file    user_isr_handlers.c
 * @brief   HAL回调App侧落点（当前无App级路由需求）
 * @note    - UART回调由platform_mcu/usart实现并按注册表分发
 *          - 后续EXTI/TIM等回调需要业务路由时在此强符号化
 */

#include "user_isr_handlers.h"
