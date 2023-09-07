/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define R_ELEV_PWM_Pin GPIO_PIN_1
#define R_ELEV_PWM_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define L_FLAP_PWM_Pin GPIO_PIN_6
#define L_FLAP_PWM_GPIO_Port GPIOA
#define PRESSR_OUT_Pin GPIO_PIN_1
#define PRESSR_OUT_GPIO_Port GPIOB
#define PRESSR_SCK_Pin GPIO_PIN_2
#define PRESSR_SCK_GPIO_Port GPIOB
#define RUDDER_PWM_Pin GPIO_PIN_10
#define RUDDER_PWM_GPIO_Port GPIOB
#define OPENLOGGER_DTR_Pin GPIO_PIN_12
#define OPENLOGGER_DTR_GPIO_Port GPIOB
#define IMU_PWR_Pin GPIO_PIN_6
#define IMU_PWR_GPIO_Port GPIOC
#define L_AILERON_PWM_Pin GPIO_PIN_7
#define L_AILERON_PWM_GPIO_Port GPIOC
#define R_FLAP_PWM_Pin GPIO_PIN_8
#define R_FLAP_PWM_GPIO_Port GPIOC
#define R_AILERON_PWM_Pin GPIO_PIN_9
#define R_AILERON_PWM_GPIO_Port GPIOC
#define RADIO_TX_Pin GPIO_PIN_9
#define RADIO_TX_GPIO_Port GPIOA
#define RADIO_RX_Pin GPIO_PIN_10
#define RADIO_RX_GPIO_Port GPIOA
#define OPENLOGGER_TX_Pin GPIO_PIN_11
#define OPENLOGGER_TX_GPIO_Port GPIOA
#define OPENLOGGER_RX_Pin GPIO_PIN_12
#define OPENLOGGER_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define L_ELEV_PWM_Pin GPIO_PIN_15
#define L_ELEV_PWM_GPIO_Port GPIOA
#define RNG2_TRIG_Pin GPIO_PIN_10
#define RNG2_TRIG_GPIO_Port GPIOC
#define RNG1_TRIG_Pin GPIO_PIN_11
#define RNG1_TRIG_GPIO_Port GPIOC
#define RNG2_ECHO_Pin GPIO_PIN_12
#define RNG2_ECHO_GPIO_Port GPIOC
#define RNG2_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define RNG1_ECHO_Pin GPIO_PIN_2
#define RNG1_ECHO_GPIO_Port GPIOD
#define RNG1_ECHO_EXTI_IRQn EXTI2_IRQn
#define L_SPEED_PWM_Pin GPIO_PIN_6
#define L_SPEED_PWM_GPIO_Port GPIOB
#define R_SPEED_PWM_Pin GPIO_PIN_7
#define R_SPEED_PWM_GPIO_Port GPIOB
#define IMU_I2C_SCL_Pin GPIO_PIN_8
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin GPIO_PIN_9
#define IMU_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
