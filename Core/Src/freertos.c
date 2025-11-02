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
#include "daemon.h"
#include "Gimbal.h"
#include "ins_task.h"
#include "shoot.h"
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
UBaseType_t GimbalLevel;
UBaseType_t MotorLevel;
UBaseType_t ChassisLevel;
UBaseType_t InsLevel;
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
/* Definitions for _DaemonTask */
osThreadId_t _DaemonTaskHandle;
const osThreadAttr_t _DaemonTask_attributes = {
  .name = "_DaemonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _GimbalTask */
osThreadId_t _GimbalTaskHandle;
const osThreadAttr_t _GimbalTask_attributes = {
  .name = "_GimbalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _INSTask */
osThreadId_t _INSTaskHandle;
const osThreadAttr_t _INSTask_attributes = {
  .name = "_INSTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _ShootTask */
osThreadId_t _ShootTaskHandle;
const osThreadAttr_t _ShootTask_attributes = {
  .name = "_ShootTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void _chassisTask(void *argument);
void _motorTask(void *argument);
void _robotCmdTask(void *argument);
void _daemonTask(void *argument);
void _gimbalTask(void *argument);
void INSTask(void *argument);
void _shootTask(void *argument);

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

  /* creation of _DaemonTask */
  _DaemonTaskHandle = osThreadNew(_daemonTask, NULL, &_DaemonTask_attributes);

  /* creation of _GimbalTask */
  _GimbalTaskHandle = osThreadNew(_gimbalTask, NULL, &_GimbalTask_attributes);

  /* creation of _INSTask */
  _INSTaskHandle = osThreadNew(INSTask, NULL, &_INSTask_attributes);

  /* creation of _ShootTask */
  _ShootTaskHandle = osThreadNew(_shootTask, NULL, &_ShootTask_attributes);

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
    osDelay(2);
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
    MotorLevel = uxTaskGetStackHighWaterMark(NULL);
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
    osDelay(2);
  }
  /* USER CODE END _robotCmdTask */
}

/* USER CODE BEGIN Header__daemonTask */
/**
* @brief Function implementing the _DaemonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__daemonTask */
void _daemonTask(void *argument)
{
  /* USER CODE BEGIN _daemonTask */
  /* Infinite loop */
  for(;;)
  {
    DaemonTask();
    osDelay(10);
  }
  /* USER CODE END _daemonTask */
}

/* USER CODE BEGIN Header__gimbalTask */
/**
* @brief Function implementing the _GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__gimbalTask */
void _gimbalTask(void *argument)
{
  /* USER CODE BEGIN _gimbalTask */
  /* Infinite loop */
  for(;;)
  {
    GimbalTask();
    GimbalLevel = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
  /* USER CODE END _gimbalTask */
}

/* USER CODE BEGIN Header_INSTask */
/**
* @brief Function implementing the _INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INSTask */
void INSTask(void *argument)
{
  /* USER CODE BEGIN INSTask */
  /* Infinite loop */
  for(;;)
  {
    INS_Task();
    InsLevel = uxTaskGetStackHighWaterMark(NULL);
    osDelay(1);
  }
  /* USER CODE END INSTask */
}

/* USER CODE BEGIN Header__shootTask */
/**
* @brief Function implementing the _ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__shootTask */
void _shootTask(void *argument)
{
  /* USER CODE BEGIN _shootTask */
  /* Infinite loop */
  for(;;)
  {
    ShootTask();
    osDelay(1);
  }
  /* USER CODE END _shootTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

