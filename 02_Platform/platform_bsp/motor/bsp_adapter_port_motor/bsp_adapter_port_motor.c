/**
 * @file    bsp_adapter_port_motor.c
 * @brief   电机Adapter：器件实例化、接口注入与Wrapper注册
 * @note    - 本层未来持有driver实例、板级映射和显式签名适配函数，
 *            并以drv_adapter_motor_reg()逐槽位注册设备表
 *          - 控制节拍、编码器与PWM绑定均不上浮到Wrapper或Service
 */

#include "bsp_adapter_port_motor.h"

motor_drv_status_t drv_adapter_motor_register(void)
{
    /* bsp_motor的driver/handler尚未迁移：不注册占位设备表， */
    /* 显式报告无器件，上层据此裁剪电机能力 */
    return MOTOR_DRV_ERR_NOTREG;
}
