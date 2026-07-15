/**
 * @file    user_periph_setup.c
 * @brief   组合根：集中注册各BSP能力Adapter
 * @note    - 每完成一个BSP能力即在此挂接其注册入口
 *          - 内核启动后执行 
 */

#include "user_periph_setup.h"

#include "bsp_adapter_port_imu.h"
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

    /* IMU（维特WT系列）：注册并启动USART1 DMA空闲接收 */
    err = bsp_imu_adp_reg();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = bsp_imu_init();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    /* motor能力：bsp_motor的driver落地后在此挂接其注册入口 */
    return PLATFORM_ERR_OK;
}
