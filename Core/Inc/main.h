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
#include "stm32l5xx_hal.h"

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
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define BMX_Data_Pin GPIO_PIN_0
#define BMX_Data_GPIO_Port GPIOF
#define BMX_Clock_Pin GPIO_PIN_1
#define BMX_Clock_GPIO_Port GPIOF
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define YAW_PWM_Pin GPIO_PIN_9
#define YAW_PWM_GPIO_Port GPIOE
#define PITCH_PWM_Pin GPIO_PIN_11
#define PITCH_PWM_GPIO_Port GPIOE
#define Mavlink_TX_Pin GPIO_PIN_10
#define Mavlink_TX_GPIO_Port GPIOB
#define MAVLINK_RX_Pin GPIO_PIN_11
#define MAVLINK_RX_GPIO_Port GPIOB
#define ST_LINK_VCP_TX_Pin GPIO_PIN_7
#define ST_LINK_VCP_TX_GPIO_Port GPIOG
#define ST_LINK_VCP_RX_Pin GPIO_PIN_8
#define ST_LINK_VCP_RX_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOA
#define NEOM8_TX_Pin GPIO_PIN_12
#define NEOM8_TX_GPIO_Port GPIOC
#define NEOM8_RX_Pin GPIO_PIN_2
#define NEOM8_RX_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
