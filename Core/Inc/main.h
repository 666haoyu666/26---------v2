/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PA0_LSENSOR_0_Pin GPIO_PIN_0
#define PA0_LSENSOR_0_GPIO_Port GPIOA
#define PA1_LSENSOR_1_Pin GPIO_PIN_1
#define PA1_LSENSOR_1_GPIO_Port GPIOA
#define PA2_LSENSOR_2_Pin GPIO_PIN_2
#define PA2_LSENSOR_2_GPIO_Port GPIOA
#define PA3_LSENSOR_3_Pin GPIO_PIN_3
#define PA3_LSENSOR_3_GPIO_Port GPIOA
#define PA4_LSENSOR_4_Pin GPIO_PIN_4
#define PA4_LSENSOR_4_GPIO_Port GPIOA
#define PA5_LSENSOR_5_Pin GPIO_PIN_5
#define PA5_LSENSOR_5_GPIO_Port GPIOA
#define PA6_LSENSOR_6_Pin GPIO_PIN_6
#define PA6_LSENSOR_6_GPIO_Port GPIOA
#define PA7_LSENSOR_7_Pin GPIO_PIN_7
#define PA7_LSENSOR_7_GPIO_Port GPIOA
#define PB0_ADC1_IN8_Pin GPIO_PIN_0
#define PB0_ADC1_IN8_GPIO_Port GPIOB
#define PB12_AIN1_Pin GPIO_PIN_12
#define PB12_AIN1_GPIO_Port GPIOB
#define PB13_AIN2_Pin GPIO_PIN_13
#define PB13_AIN2_GPIO_Port GPIOB
#define PB14_BIN1_Pin GPIO_PIN_14
#define PB14_BIN1_GPIO_Port GPIOB
#define PB15_BIN2_Pin GPIO_PIN_15
#define PB15_BIN2_GPIO_Port GPIOB
#define PA8_PWMB_TIM1_Pin GPIO_PIN_8
#define PA8_PWMB_TIM1_GPIO_Port GPIOA
#define PA9_PWMA_TIM1_Pin GPIO_PIN_9
#define PA9_PWMA_TIM1_GPIO_Port GPIOA
#define PA11_BT_UART_TX_Pin GPIO_PIN_11
#define PA11_BT_UART_TX_GPIO_Port GPIOA
#define PA12_BT_UART_RX_Pin GPIO_PIN_12
#define PA12_BT_UART_RX_GPIO_Port GPIOA
#define PA15_IMU_UART_TX_Pin GPIO_PIN_15
#define PA15_IMU_UART_TX_GPIO_Port GPIOA
#define PB3_IMU_UART_RX_Pin GPIO_PIN_3
#define PB3_IMU_UART_RX_GPIO_Port GPIOB
#define PB4_E2B_Pin GPIO_PIN_4
#define PB4_E2B_GPIO_Port GPIOB
#define PB5_E2A_Pin GPIO_PIN_5
#define PB5_E2A_GPIO_Port GPIOB
#define PB6_E2B_Pin GPIO_PIN_6
#define PB6_E2B_GPIO_Port GPIOB
#define PB7_E1A_Pin GPIO_PIN_7
#define PB7_E1A_GPIO_Port GPIOB
#define PB8_I2C1_SCL_Pin GPIO_PIN_8
#define PB8_I2C1_SCL_GPIO_Port GPIOB
#define PB9_I2C1_SDA_Pin GPIO_PIN_9
#define PB9_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
