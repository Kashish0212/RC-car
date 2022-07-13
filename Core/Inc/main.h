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
#define fwd_btn_Pin GPIO_PIN_13
#define fwd_btn_GPIO_Port GPIOC
#define rvs_btn_Pin GPIO_PIN_14
#define rvs_btn_GPIO_Port GPIOC
#define left_btn_Pin GPIO_PIN_15
#define left_btn_GPIO_Port GPIOC
#define Encoder_mode_TIM5_CH1_Pin GPIO_PIN_0
#define Encoder_mode_TIM5_CH1_GPIO_Port GPIOA
#define Encoder_mode_TIM5_CH2_Pin GPIO_PIN_1
#define Encoder_mode_TIM5_CH2_GPIO_Port GPIOA
#define motor_left_axle_Pin GPIO_PIN_5
#define motor_left_axle_GPIO_Port GPIOA
#define motor_right_axle_Pin GPIO_PIN_6
#define motor_right_axle_GPIO_Port GPIOA
#define stop_pin_Pin GPIO_PIN_8
#define stop_pin_GPIO_Port GPIOB
#define right_btn_Pin GPIO_PIN_9
#define right_btn_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
