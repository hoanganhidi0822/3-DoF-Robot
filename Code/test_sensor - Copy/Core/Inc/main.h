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
#define PUL3_Pin GPIO_PIN_1
#define PUL3_GPIO_Port GPIOA
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOA
#define PUL2_Pin GPIO_PIN_3
#define PUL2_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOA
#define PUL1_Pin GPIO_PIN_5
#define PUL1_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_6
#define DIR1_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_7
#define EN_GPIO_Port GPIOA
#define LIMIT_LINK3_Pin GPIO_PIN_8
#define LIMIT_LINK3_GPIO_Port GPIOA
#define LIMIT_LINK1_Pin GPIO_PIN_9
#define LIMIT_LINK1_GPIO_Port GPIOA
#define LIMIT_LINK2_Pin GPIO_PIN_10
#define LIMIT_LINK2_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_11
#define RELAY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
