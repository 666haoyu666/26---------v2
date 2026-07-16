/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_periph_setup.h"

#include "board_motor_config.h"
#include "bsp_wrapper_motor.h"
#include "myprintf.h"
#include "service_diff_odom.h"
#include "bsp_wrapper_imu.h"
#include "service_track_ctrl.h"
#include "motor_ctrl_port.h"
#include "imu_ctrl_port.h"
#include "track_port.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Odom演示标定：轮径为占位值，实测后回填 */
#define ODOM_DEMO_WHEEL_DIA_MM (65.0f)
#define ODOM_DEMO_MM_TICK      (ODOM_DEMO_WHEEL_DIA_MM * 3.1415927f / \
                                (float)BOARD_MOTOR_CPR)
/* 循迹控制装配：每转行程随轮径实测同步更新 */
#define CTRL_DEMO_MM_REV       (ODOM_DEMO_WHEEL_DIA_MM * 3.1415927f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  /* 组合根：内核启动前注册全部BSP Adapter */
  if (PLATFORM_IS_ERR(app_periph_init()))
  {
    Error_Handler();
  }

  /* 里程计装配：槽位对齐board_motor_config，标定值待实测回填 */
  server_odom_cfg_t odom_cfg;
  odom_cfg.left_id = BOARD_MOTOR_A_SLOT;
  odom_cfg.right_id = BOARD_MOTOR_B_SLOT;
  odom_cfg.left_mm_tick = ODOM_DEMO_MM_TICK;
  odom_cfg.right_mm_tick = ODOM_DEMO_MM_TICK;
  odom_cfg.left_sign = 1;
  odom_cfg.right_sign = 1;
  if (PLATFORM_IS_ERR(server_odom_init(&odom_cfg)))
  {
    Error_Handler();
  }

  /* 循迹控制装配：Port先行、服务最后；左右槽位与odom一致 */
  motor_ctrl_cfg_t ctrl_cfg;
  ctrl_cfg.left_id = BOARD_MOTOR_A_SLOT;
  ctrl_cfg.right_id = BOARD_MOTOR_B_SLOT;
  ctrl_cfg.left_mm_rev = CTRL_DEMO_MM_REV;
  ctrl_cfg.right_mm_rev = CTRL_DEMO_MM_REV;
  if (PLATFORM_IS_ERR(motor_ctrl_init(&ctrl_cfg)))
  {
    Error_Handler();
  }
  if (PLATFORM_IS_ERR(imu_ctrl_init()))
  {
    Error_Handler();
  }
  if (PLATFORM_IS_ERR(track_port_init()))
  {
    Error_Handler();
  }
  if (PLATFORM_IS_ERR(track_ctrl_init()))
  {
    Error_Handler();
  }
	osDelay(2000);

  char buf[128];
  track_ctrl_set_mode(TRACK_CTRL_MODE_TRACK_DIR,
                      0,200);
  server_odom_state_t odom_state,last_odom_state;
  track_port_result_t track_state;
  float target_yaw = 0;
  last_odom_state.x_mm = 0;
  last_odom_state.y_mm = 0;
  uint8_t uart_hz;
  for(;;)
  {
    
    
    server_odom_get(&odom_state);
    track_port_get(&track_state);
    if(TRACK_PORT_NO_LINE   == track_state.state ||
       TRACK_PORT_AMBIGUOUS == track_state.state ){
      uint32_t distance;
      distance =  (odom_state.x_mm - last_odom_state.x_mm) *
                  (odom_state.x_mm - last_odom_state.x_mm) +
                  (odom_state.y_mm - last_odom_state.y_mm) *
                  (odom_state.y_mm - last_odom_state.y_mm);
      if (360000 > distance)
      {
        
        osDelay(100);
        continue;
      }
			
      last_odom_state = odom_state;
      target_yaw+=90;
      track_ctrl_set_mode(TRACK_CTRL_MODE_TRACK_DIR,
                      target_yaw,200);
    }
    
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

