/**
 ******************************************************************************
 * @file           : tim.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for timer functions
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "indication.h"
#include "led.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
struct timeout {
	volatile bool start_flag;
	volatile uint32_t timeout_ms;
	volatile uint32_t timestamp_ms;
};

extern TIM_HandleTypeDef htim1;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
extern void MX_TIM1_Init(void);
extern void TimStart(struct timeout *timeout, uint32_t timeout_ms);
extern bool TimStarted(struct timeout *timeout);
extern void TimStop(struct timeout *timeout);
extern bool TimElapsed(struct timeout *timeout);
extern bool TimCheck(struct timeout *timeout, uint32_t msTime);


/******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif /* __TIM_H__ */
