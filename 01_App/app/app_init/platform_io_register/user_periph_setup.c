/**
 * @file    user_periph_setup.c
 * @brief   组合根：集中注册各BSP能力Adapter
 * @note    - 每完成一个BSP能力即在此挂接其注册入口
 *          - 内核启动前执行，注册函数不得阻塞等待OS资源  
 *          - 这里简单将初始化放在这里了，后续可考虑在OS任务中初始化
 */

#include "user_periph_setup.h"

#include "bsp_adapter_port_line_sensor.h"

platform_err_t app_periph_init(void)
{
    platform_err_t err; /* 当前注册结果 */

    //灰度传感器
    err = bsp_lsensor_adp_reg();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = bsp_lsensor_init();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    /* motor能力：bsp_motor的driver落地后在此挂接其注册入口 */
    return PLATFORM_ERR_OK;
}
