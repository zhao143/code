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
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define DS18B20_Pin GPIO_PIN_4
#define DS18B20_GPIO_Port GPIOA
#define TB6612_STBY_Pin GPIO_PIN_6
#define TB6612_STBY_GPIO_Port GPIOA
#define Beep_Pin GPIO_PIN_0
#define Beep_GPIO_Port GPIOB
#define Relay_Pin GPIO_PIN_1
#define Relay_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_12
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_8
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_11
#define PWMA_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
