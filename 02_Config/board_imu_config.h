/**
 * @file    board_imu_config.h
 * @brief   IMU板级映射
 * @note    - 维特WT系列挂USART1：PB3=RX,PA15=TX,115200,RX走DMA
 *          - 逻辑串口必须与usart_port.c映射表对齐
 */

#ifndef BOARD_IMU_CONFIG_H
#define BOARD_IMU_CONFIG_H

#define BOARD_IMU_USART (EN_CORE_USART_IMU) /* IMU逻辑串口 */

#endif /* BOARD_IMU_CONFIG_H */
