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
void USART1_CommandIRQ(void);
void Protocol_ReceiveByte(uint8_t byte);
void Stepper_TimerElapsed(TIM_TypeDef *timer_instance);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define YL_STEP_Pin GPIO_PIN_0
#define YL_STEP_GPIO_Port GPIOA
#define YL_DIR_Pin GPIO_PIN_1
#define YL_DIR_GPIO_Port GPIOA
#define YR_STEP_Pin GPIO_PIN_2
#define YR_STEP_GPIO_Port GPIOA
#define YR_DIR_Pin GPIO_PIN_3
#define YR_DIR_GPIO_Port GPIOA
/* Y axis limit sensors.  Both modules are active-low. */
#define Y_LIMIT_HOME_Pin GPIO_PIN_4
#define Y_LIMIT_HOME_GPIO_Port GPIOA
#define Y_LIMIT_END_Pin GPIO_PIN_5
#define Y_LIMIT_END_GPIO_Port GPIOA

typedef struct
{
  volatile uint32_t target_steps;
  volatile uint32_t step_count;
  volatile uint32_t target_pps;
  volatile uint32_t current_pps;
  volatile uint32_t acceleration_pps2;
  volatile uint16_t request_id;
  volatile uint8_t pulse_is_high;
  volatile uint8_t running;
  volatile uint8_t done_pending;
} StepperAxis;

extern volatile StepperAxis stepper_axes[];

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
