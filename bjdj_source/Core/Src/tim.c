/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM2/TIM3 time bases for the two STEP/DIR axes.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "tim.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

static void MX_TIM_CommonInit(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
  TIM_ClockConfigTypeDef clock_source = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = 71;  /* 72 MHz / 72 = 1 MHz timer tick. */
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 999;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim) != HAL_OK) Error_Handler();

  clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim, &clock_source) != HAL_OK) Error_Handler();
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK) Error_Handler();
}

void MX_TIM2_Init(void)
{
  MX_TIM_CommonInit(&htim2, TIM2);
}

void MX_TIM3_Init(void)
{
  MX_TIM_CommonInit(&htim3, TIM3);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_base_handle)
{
  if (tim_base_handle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  else if (tim_base_handle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_base_handle)
{
  if (tim_base_handle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
  else if (tim_base_handle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Stepper_TimerElapsed(htim->Instance);
}
