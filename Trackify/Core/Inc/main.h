/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
#define BAROMETER_INT_Pin GPIO_PIN_13
#define BAROMETER_INT_GPIO_Port GPIOC
#define BAT_EN_Pin GPIO_PIN_2
#define BAT_EN_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_12
#define WP_GPIO_Port GPIOB
#define SIMCON_RST_Pin GPIO_PIN_15
#define SIMCON_RST_GPIO_Port GPIOB
#define PWRKEY_Pin GPIO_PIN_8
#define PWRKEY_GPIO_Port GPIOA
#define RING_Pin GPIO_PIN_11
#define RING_GPIO_Port GPIOA
#define DTR_Pin GPIO_PIN_12
#define DTR_GPIO_Port GPIOA
#define PIEZO_Pin GPIO_PIN_5
#define PIEZO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
