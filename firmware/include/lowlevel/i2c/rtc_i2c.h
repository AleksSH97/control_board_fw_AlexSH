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

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define ADDR_WORD          (true)
#define ADDR_BYTE          (false)
#define PROJ_UNUSED(x)     ((void)(x))


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t RtcI2cInit(void);
uint8_t RtcI2cReadByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes);
uint8_t RtcI2cWriteByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes);
uint8_t RtcI2cReadByteInterrupt(uint8_t device, uint8_t address, void *buffer, uint16_t length);
uint8_t RtcI2cReadBufferInterrupt(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t length);
uint8_t RtcI2cWriteByteInterrupt(uint8_t device, uint8_t address, uint8_t b);
uint8_t RtcI2cWriteBufferInterrupt(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t length);


uint8_t RtcI2cGetDate(RTC_DATE_t *date);
uint8_t RtcI2cGetTime(RTC_TIME_t *time);
uint8_t RtcI2cSetDate(RTC_DATE_t *date);
uint8_t RtcI2cSetTime(RTC_TIME_t *time);


/******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* LOWLEVEL_I2C_RTC_I2C_H_ */
