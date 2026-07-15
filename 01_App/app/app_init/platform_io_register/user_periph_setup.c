/**
 * @file    user_periph_setup.c
 * @brief   组合根：集中注册各BSP能力Adapter
 * @note    - 每完成一个BSP能力即在此挂接其注册入口
 *          - 内核启动后执行 
 */

#include "user_periph_setup.h"

#include "bsp_adapter_port_imu.h"
#include "bsp_adapter_port_line_sensor.h"
#include "bsp_adapter_port_motor.h"

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

    /* 电机（A左/B右两轮）：注册设备表并初始化， */
    /* 初始化后处OFF态不出力，起转由上层start触发 */
    err = drv_adapter_motor_register();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }
    err = drv_adapter_motor_init();
    if (PLATFORM_IS_ERR(err)) {
        return err;
    }

    return PLATFORM_ERR_OK;
}
