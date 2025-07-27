/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "motor_task.h"
#include "chassis.h"
#include "robot_cmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for _ChassisTask */
osThreadId_t _ChassisTaskHandle;
const osThreadAttr_t _ChassisTask_attributes = {
  .name = "_ChassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _MotorTask */
osThreadId_t _MotorTaskHandle;
const osThreadAttr_t _MotorTask_attributes = {
  .name = "_MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _RobotCmdTask */
osThreadId_t _RobotCmdTaskHandle;
const osThreadAttr_t _RobotCmdTask_attributes = {
  .name = "_RobotCmdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void _chassisTask(void *argument);
void _motorTask(void *argument);
void _robotCmdTask(void *argument);

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
  /* creation of _ChassisTask */
  _ChassisTaskHandle = osThreadNew(_chassisTask, NULL, &_ChassisTask_attributes);

  /* creation of _MotorTask */
  _MotorTaskHandle = osThreadNew(_motorTask, NULL, &_MotorTask_attributes);

  /* creation of _RobotCmdTask */
  _RobotCmdTaskHandle = osThreadNew(_robotCmdTask, NULL, &_RobotCmdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header__chassisTask */
/**
  * @brief  Function implementing the _ChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header__chassisTask */
void _chassisTask(void *argument)
{
  /* USER CODE BEGIN _chassisTask */
  /* Infinite loop */
  for(;;)
  {
    ChassisTask();
    osDelay(1);
  }
  /* USER CODE END _chassisTask */
}

/* USER CODE BEGIN Header__motorTask */
/**
* @brief Function implementing the _MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__motorTask */
void _motorTask(void *argument)
{
  /* USER CODE BEGIN _motorTask */
  /* Infinite loop */
  for(;;)
  {
    MotorControlTask();
    osDelay(1);
  }
  /* USER CODE END _motorTask */
}

/* USER CODE BEGIN Header__robotCmdTask */
/**
* @brief Function implementing the _RobotCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__robotCmdTask */
void _robotCmdTask(void *argument)
{
  /* USER CODE BEGIN _robotCmdTask */
  /* Infinite loop */
  for(;;)
  {
    RobotCmdTask();
    osDelay(1);
  }
  /* USER CODE END _robotCmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

