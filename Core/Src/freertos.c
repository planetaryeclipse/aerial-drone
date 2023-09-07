/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "comms_types.h"
#include "sens_types.h"
#include "printf/printf.h"
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
/* Definitions for blink */
osThreadId_t blinkHandle;
const osThreadAttr_t blink_attributes = {
  .name = "blink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for readPressrSens */
osThreadId_t readPressrSensHandle;
const osThreadAttr_t readPressrSens_attributes = {
  .name = "readPressrSens",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for writeLogger */
osThreadId_t writeLoggerHandle;
const osThreadAttr_t writeLogger_attributes = {
  .name = "writeLogger",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radioManager */
osThreadId_t radioManagerHandle;
const osThreadAttr_t radioManager_attributes = {
  .name = "radioManager",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for readMag */
osThreadId_t readMagHandle;
const osThreadAttr_t readMag_attributes = {
  .name = "readMag",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readInertial */
osThreadId_t readInertialHandle;
const osThreadAttr_t readInertial_attributes = {
  .name = "readInertial",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readRngs */
osThreadId_t readRngsHandle;
const osThreadAttr_t readRngs_attributes = {
  .name = "readRngs",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cntrlActuators */
osThreadId_t cntrlActuatorsHandle;
const osThreadAttr_t cntrlActuators_attributes = {
  .name = "cntrlActuators",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for actCmds */
osMessageQueueId_t actCmdsHandle;
const osMessageQueueAttr_t actCmds_attributes = {
  .name = "actCmds"
};
/* Definitions for pressrToLog */
osMessageQueueId_t pressrToLogHandle;
const osMessageQueueAttr_t pressrToLog_attributes = {
  .name = "pressrToLog"
};
/* Definitions for pressrToSend */
osMessageQueueId_t pressrToSendHandle;
const osMessageQueueAttr_t pressrToSend_attributes = {
  .name = "pressrToSend"
};
/* Definitions for rng1ToLog */
osMessageQueueId_t rng1ToLogHandle;
const osMessageQueueAttr_t rng1ToLog_attributes = {
  .name = "rng1ToLog"
};
/* Definitions for rng1ToSend */
osMessageQueueId_t rng1ToSendHandle;
const osMessageQueueAttr_t rng1ToSend_attributes = {
  .name = "rng1ToSend"
};
/* Definitions for rng2ToLog */
osMessageQueueId_t rng2ToLogHandle;
const osMessageQueueAttr_t rng2ToLog_attributes = {
  .name = "rng2ToLog"
};
/* Definitions for rng2ToSend */
osMessageQueueId_t rng2ToSendHandle;
const osMessageQueueAttr_t rng2ToSend_attributes = {
  .name = "rng2ToSend"
};
/* Definitions for actCmdsToLog */
osMessageQueueId_t actCmdsToLogHandle;
const osMessageQueueAttr_t actCmdsToLog_attributes = {
  .name = "actCmdsToLog"
};
/* Definitions for imuToLog */
osMessageQueueId_t imuToLogHandle;
const osMessageQueueAttr_t imuToLog_attributes = {
  .name = "imuToLog"
};
/* Definitions for imuToSend */
osMessageQueueId_t imuToSendHandle;
const osMessageQueueAttr_t imuToSend_attributes = {
  .name = "imuToSend"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBlink(void *argument);
void ReadPressureSensor(void *argument);
void WriteLogger(void *argument);
void RadioManager(void *argument);
void ReadMag(void *argument);
void ReadInertial(void *argument);
void ReadRngs(void *argument);
void ControlActuators(void *argument);

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

  /* Create the queue(s) */
  /* creation of actCmds */
  actCmdsHandle = osMessageQueueNew (16, sizeof(actuator_t), &actCmds_attributes);

  /* creation of pressrToLog */
  pressrToLogHandle = osMessageQueueNew (16, sizeof(sens_pressr_t), &pressrToLog_attributes);

  /* creation of pressrToSend */
  pressrToSendHandle = osMessageQueueNew (16, sizeof(sens_pressr_t), &pressrToSend_attributes);

  /* creation of rng1ToLog */
  rng1ToLogHandle = osMessageQueueNew (16, sizeof(sens_rng_t), &rng1ToLog_attributes);

  /* creation of rng1ToSend */
  rng1ToSendHandle = osMessageQueueNew (16, sizeof(sens_rng_t), &rng1ToSend_attributes);

  /* creation of rng2ToLog */
  rng2ToLogHandle = osMessageQueueNew (16, sizeof(sens_rng_t), &rng2ToLog_attributes);

  /* creation of rng2ToSend */
  rng2ToSendHandle = osMessageQueueNew (16, sizeof(sens_rng_t), &rng2ToSend_attributes);

  /* creation of actCmdsToLog */
  actCmdsToLogHandle = osMessageQueueNew (16, sizeof(actuator_t), &actCmdsToLog_attributes);

  /* creation of imuToLog */
  imuToLogHandle = osMessageQueueNew (16, sizeof(sens_imu_t), &imuToLog_attributes);

  /* creation of imuToSend */
  imuToSendHandle = osMessageQueueNew (16, sizeof(sens_imu_t), &imuToSend_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink */
  blinkHandle = osThreadNew(StartBlink, NULL, &blink_attributes);

  /* creation of readPressrSens */
  readPressrSensHandle = osThreadNew(ReadPressureSensor, NULL, &readPressrSens_attributes);

  /* creation of writeLogger */
  writeLoggerHandle = osThreadNew(WriteLogger, NULL, &writeLogger_attributes);

  /* creation of radioManager */
  radioManagerHandle = osThreadNew(RadioManager, NULL, &radioManager_attributes);

  /* creation of readMag */
  readMagHandle = osThreadNew(ReadMag, NULL, &readMag_attributes);

  /* creation of readInertial */
  readInertialHandle = osThreadNew(ReadInertial, NULL, &readInertial_attributes);

  /* creation of readRngs */
  readRngsHandle = osThreadNew(ReadRngs, NULL, &readRngs_attributes);

  /* creation of cntrlActuators */
  cntrlActuatorsHandle = osThreadNew(ControlActuators, NULL, &cntrlActuators_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBlink */
/**
  * @brief  Function implementing the blink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink */
void StartBlink(void *argument)
{
  /* USER CODE BEGIN StartBlink */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      osDelay(250);
  }
  /* USER CODE END StartBlink */
}

/* USER CODE BEGIN Header_ReadPressureSensor */
/**
* @brief Function implementing the readPressrSens thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadPressureSensor */
__weak void ReadPressureSensor(void *argument)
{
  /* USER CODE BEGIN ReadPressureSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReadPressureSensor */
}

/* USER CODE BEGIN Header_WriteLogger */
/**
* @brief Function implementing the writeLogger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WriteLogger */
__weak void WriteLogger(void *argument)
{
  /* USER CODE BEGIN WriteLogger */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WriteLogger */
}

/* USER CODE BEGIN Header_RadioManager */
/**
* @brief Function implementing the radioManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RadioManager */
__weak void RadioManager(void *argument)
{
  /* USER CODE BEGIN RadioManager */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RadioManager */
}

/* USER CODE BEGIN Header_ReadMag */
/**
* @brief Function implementing the readMag thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadMag */
__weak void ReadMag(void *argument)
{
  /* USER CODE BEGIN ReadMag */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReadMag */
}

/* USER CODE BEGIN Header_ReadInertial */
/**
* @brief Function implementing the readInertial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadInertial */
__weak void ReadInertial(void *argument)
{
  /* USER CODE BEGIN ReadInertial */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReadInertial */
}

/* USER CODE BEGIN Header_ReadRngs */
/**
* @brief Function implementing the readRngs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadRngs */
__weak void ReadRngs(void *argument)
{
  /* USER CODE BEGIN ReadRngs */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReadRngs */
}

/* USER CODE BEGIN Header_ControlActuators */
/**
* @brief Function implementing the cntrlActuators thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlActuators */
__weak void ControlActuators(void *argument)
{
  /* USER CODE BEGIN ControlActuators */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ControlActuators */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

