/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

TIM_HandleTypeDef htim1;

void TimStart(struct timeout *timeout, uint32_t timeout_ms)
{
	if (timeout == NULL) {
		Error_Handler();
	}

	timeout->timestamp_ms = HAL_GetTick();
	timeout->timeout_ms = timeout_ms;
	timeout->start_flag = true;
}

void TimStop(struct timeout *timeout)
{
	if (timeout == NULL) {
		Error_Handler();
	}

	timeout->start_flag = false;
}

bool TimStarted(struct timeout *timeout)
{
	if (timeout == NULL) {
		Error_Handler();
	}

	return timeout->start_flag;
}

bool TimElapsed(struct timeout *timeout)
{
	if (timeout == NULL) {
		Error_Handler();
	}

	if (!timeout->start_flag) {
		return false;
	}

	if ((HAL_GetTick() - timeout->timestamp_ms) > timeout->timeout_ms) {
		timeout->start_flag = false;
		return true;
	}

	return false;
}

bool TimCheck(struct timeout *timeout, uint32_t msTime)
{
	if ((HAL_GetTick() - timeout->timestamp_ms) > msTime) {
		return true;
	}

	return false;
}

void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 32000;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 50;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)

    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  }
}
