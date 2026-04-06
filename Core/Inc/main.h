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
typedef enum
{
  APP_ERROR_NONE = 0,
  APP_ERROR_CLOCK_CONFIG = 1,
  APP_ERROR_FDCAN1_INIT = 2,
  APP_ERROR_FDCAN1_CLOCK_CONFIG = 3,
  APP_ERROR_SPI1_INIT = 4,
  APP_ERROR_SPI2_INIT = 5,
  APP_ERROR_USART2_INIT = 6,
  APP_ERROR_USART2_TXFIFO = 7,
  APP_ERROR_USART2_RXFIFO = 8,
  APP_ERROR_USART2_FIFO_DISABLE = 9,
  APP_ERROR_USART2_CLOCK_CONFIG = 10,
  APP_ERROR_CAN_START = 11,
  APP_ERROR_BMI330_INIT = 12,
  APP_ERROR_BMP384_INIT = 13,
  APP_ERROR_NEO6M_INIT = 14,
  APP_ERROR_ASSERT = 15
} app_error_code_t;

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
void Error_HandlerEx(app_error_code_t code);
void Error_Handler_SetCode(app_error_code_t code);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMI330_CS_Pin GPIO_PIN_0
#define BMI330_CS_GPIO_Port GPIOB
#define BMI330_INT1_Pin GPIO_PIN_1
#define BMI330_INT1_GPIO_Port GPIOB
#define BMI330_INT2_Pin GPIO_PIN_2
#define BMI330_INT2_GPIO_Port GPIOB
#define BMP384_CS_Pin GPIO_PIN_10
#define BMP384_CS_GPIO_Port GPIOB
#define BMP384_INT_Pin GPIO_PIN_11
#define BMP384_INT_GPIO_Port GPIOB
#define STATUS_LED_2_Pin GPIO_PIN_6
#define STATUS_LED_2_GPIO_Port GPIOB
#define CAN_STB_Pin GPIO_PIN_7
#define CAN_STB_GPIO_Port GPIOB
#define STATUS_LED_1_Pin GPIO_PIN_9
#define STATUS_LED_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
