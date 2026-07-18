/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB12_AIN1_Pin|PB13_AIN2_Pin|PB14_BIN1_Pin|PB15_BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0_LSENSOR_0_Pin PA1_LSENSOR_1_Pin PA2_LSENSOR_2_Pin PA3_LSENSOR_3_Pin
                           PA4_LSENSOR_4_Pin PA5_LSENSOR_5_Pin PA6_LSENSOR_6_Pin PA7_LSENSOR_7_Pin */
  GPIO_InitStruct.Pin = PA0_LSENSOR_0_Pin|PA1_LSENSOR_1_Pin|PA2_LSENSOR_2_Pin|PA3_LSENSOR_3_Pin
                          |PA4_LSENSOR_4_Pin|PA5_LSENSOR_5_Pin|PA6_LSENSOR_6_Pin|PA7_LSENSOR_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12_AIN1_Pin PB13_AIN2_Pin PB14_BIN1_Pin PB15_BIN2_Pin */
  GPIO_InitStruct.Pin = PB12_AIN1_Pin|PB13_AIN2_Pin|PB14_BIN1_Pin|PB15_BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4_E2B_Pin PB6_E2B_Pin */
  GPIO_InitStruct.Pin = PB4_E2B_Pin|PB6_E2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5_E2A_Pin PB7_E1A_Pin */
  GPIO_InitStruct.Pin = PB5_E2A_Pin|PB7_E1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
