/**
 ******************************************************************************
 * @file           : rtc_i2c.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file of RTC I2C
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

#ifndef LOWLEVEL_I2C_RTC_I2C_H_
#define LOWLEVEL_I2C_RTC_I2C_H_

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"

#include "log.h"
#include "rtc.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define ADDR_WORD          (true)
#define ADDR_BYTE          (false)
#define PROJ_UNUSED(x)                ((void)(x))


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/



/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t RtcI2cInit(void);
uint8_t RtcI2cReadByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes);
uint8_t RtcI2cWriteByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes);


/******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* LOWLEVEL_I2C_RTC_I2C_H_ */
