/**
 * @file    user_periph_setup.h
 * @brief   组合根外设注册入口
 * @note    由main()在内核启动前调用一次，注册全部BSP Adapter
 */

#ifndef USER_PERIPH_SETUP_H
#define USER_PERIPH_SETUP_H

#include "platform_error.h"

/**
 * @brief  注册全部已就绪的BSP能力Adapter
 * @retval PLATFORM_ERR_OK / 首个失败注册的错误码
 * @note   内核启动前单线程调用；任一注册失败立即返回，
 *         后续注册不再执行
 */
platform_err_t app_periph_init(void);

#endif /* USER_PERIPH_SETUP_H */
