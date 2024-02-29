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
#include "stm32f1xx_hal.h"

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
#define RELAY_TRIGGER_AIRPOS_Pin GPIO_PIN_13
#define RELAY_TRIGGER_AIRPOS_GPIO_Port GPIOC
#define RELAY_TRIGGER_AIRNEG_Pin GPIO_PIN_14
#define RELAY_TRIGGER_AIRNEG_GPIO_Port GPIOC
#define RELAY_TRIGGER_PC_Pin GPIO_PIN_15
#define RELAY_TRIGGER_PC_GPIO_Port GPIOC
#define IND_PC_Pin GPIO_PIN_0
#define IND_PC_GPIO_Port GPIOA
#define IND_PC_EXTI_IRQn EXTI0_IRQn
#define LED_FAULT_Pin GPIO_PIN_3
#define LED_FAULT_GPIO_Port GPIOA
#define LED_INDICATOR_Pin GPIO_PIN_4
#define LED_INDICATOR_GPIO_Port GPIOA
#define LED_OKAY_Pin GPIO_PIN_5
#define LED_OKAY_GPIO_Port GPIOA
#define IND_PWR_FAN1_Pin GPIO_PIN_10
#define IND_PWR_FAN1_GPIO_Port GPIOB
#define IND_PWR_FAN1_EXTI_IRQn EXTI15_10_IRQn
#define IND_PWR_FAN2_Pin GPIO_PIN_11
#define IND_PWR_FAN2_GPIO_Port GPIOB
#define IND_PWR_FAN2_EXTI_IRQn EXTI15_10_IRQn
#define IND_PWR_IMD_Pin GPIO_PIN_12
#define IND_PWR_IMD_GPIO_Port GPIOB
#define IND_PWR_IMD_EXTI_IRQn EXTI15_10_IRQn
#define REQUEST_TS_Pin GPIO_PIN_3
#define REQUEST_TS_GPIO_Port GPIOB
#define REQUEST_TS_EXTI_IRQn EXTI3_IRQn
#define IND_60V_Pin GPIO_PIN_4
#define IND_60V_GPIO_Port GPIOB
#define IND_60V_EXTI_IRQn EXTI4_IRQn
#define RELAY_AUX_AIRPOS_Pin GPIO_PIN_5
#define RELAY_AUX_AIRPOS_GPIO_Port GPIOB
#define RELAY_AUX_AIRPOS_EXTI_IRQn EXTI9_5_IRQn
#define RELAY_AUX_AIRNEG_Pin GPIO_PIN_6
#define RELAY_AUX_AIRNEG_GPIO_Port GPIOB
#define RELAY_AUX_AIRNEG_EXTI_IRQn EXTI9_5_IRQn
#define RELAY_AUX_PC_Pin GPIO_PIN_7
#define RELAY_AUX_PC_GPIO_Port GPIOB
#define RELAY_AUX_PC_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
