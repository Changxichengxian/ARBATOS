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
#define IST8310_INT_Pin GPIO_PIN_3
#define IST8310_INT_GPIO_Port GPIOE
#define IST8310_INT_EXTI_IRQn EXTI3_IRQn
#define IST8310_RST_Pin GPIO_PIN_2
#define IST8310_RST_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define Heat_PWM_Pin GPIO_PIN_5
#define Heat_PWM_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOG
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOG
#define MPU6500_CS_Pin GPIO_PIN_6
#define MPU6500_CS_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOG
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOF
#define SD_EXTI_Pin GPIO_PIN_15
#define SD_EXTI_GPIO_Port GPIOE
#define SD_EXTI_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

#define BOARD_SD_ENABLE 0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
