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
#include "stm32g4xx_hal.h"

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
#define ADC1_IN15_Batt_Pin GPIO_PIN_0
#define ADC1_IN15_Batt_GPIO_Port GPIOB
#define Direction_L_A_Pin GPIO_PIN_1
#define Direction_L_A_GPIO_Port GPIOB
#define Direction_L_B_Pin GPIO_PIN_2
#define Direction_L_B_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_11
#define SPI2_CS2_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Direction_R_B_Pin GPIO_PIN_6
#define Direction_R_B_GPIO_Port GPIOC
#define Direction_R_A_Pin GPIO_PIN_7
#define Direction_R_A_GPIO_Port GPIOC
#define STBY_Pin GPIO_PIN_8
#define STBY_GPIO_Port GPIOC
#define PWM_R_TIM3_CH4_Pin GPIO_PIN_9
#define PWM_R_TIM3_CH4_GPIO_Port GPIOC
#define PWM_L_TIM2_CH4_Pin GPIO_PIN_10
#define PWM_L_TIM2_CH4_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_INT_Pin GPIO_PIN_2
#define SPI3_INT_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
