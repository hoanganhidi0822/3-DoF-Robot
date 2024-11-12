/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LimL1_Pin GPIO_PIN_4
#define LimL1_GPIO_Port GPIOA
#define LimL2_Pin GPIO_PIN_5
#define LimL2_GPIO_Port GPIOA
#define LimL3_Pin GPIO_PIN_6
#define LimL3_GPIO_Port GPIOA
#define Mag_Pin GPIO_PIN_7
#define Mag_GPIO_Port GPIOA
#define motorL1_Pin GPIO_PIN_0
#define motorL1_GPIO_Port GPIOB
#define motorL2_Pin GPIO_PIN_1
#define motorL2_GPIO_Port GPIOB
#define motorL3_Pin GPIO_PIN_2
#define motorL3_GPIO_Port GPIOB
#define M1dir_Pin GPIO_PIN_13
#define M1dir_GPIO_Port GPIOF
#define M2dir_Pin GPIO_PIN_14
#define M2dir_GPIO_Port GPIOF
#define M3dir_Pin GPIO_PIN_15
#define M3dir_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
