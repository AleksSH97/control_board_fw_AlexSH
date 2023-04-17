/**
 ******************************************************************************
 * @file           : button.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for button
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUTTON_H_
#define BUTTON_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "log.h"
#include "indication.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define BUTTON_DEBOUNCE_TIME_MS    (50u)

#define BUTTON_Pin          GPIO_PIN_0
#define BUTTON_GPIO_Port    GPIOA


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
enum button_mode {
	BUTTON_ONE_CLICK = 0,
	BUTTON_DOUBLE_CLICK,
	BUTTON_HELD_PRESSED
};


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void ButtonInit(void);
bool ButtonIsPushed(void);
void ButtonCheckMode(void);
void ButtonTask(void *argumet);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* BUTTON_H_ */
