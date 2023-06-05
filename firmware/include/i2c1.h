/**
 ******************************************************************************
 * @file           : i2c1.h
 * @author         : Konstantin Soloviev
 * @author         : Dmitry Karasev        <karasev@voltsbattery.com>
 * @brief          : Header for I2C1 peripheral driver module. It contains
 *                   global variables, functions and defines.
 ******************************************************************************
 * ----------------- Copyright (c) 2020 VOLTS Battery LLC ------------------- *
 ******************************************************************************
 * This module is a confidential and proprietary property of VOLTS Battery
 * and possession or use of this module requires written permission
 * of VOLTS Battery.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_I2C1_H_
#define _DRV_I2C1_H_


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


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
extern volatile bool i2c1_mode_write;
extern volatile bool i2c1_address_sended;
extern volatile bool i2c1_repeated_start;
extern volatile uint8_t *i2c1_buffer;
extern volatile uint16_t i2c1_length;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void I2C1_Init(void);
bool I2C1_WriteByte(uint8_t device, bool addr_word, uint16_t address, uint8_t b);
bool I2C1_WriteBuffer(uint8_t device, bool addr_word, uint16_t address, void *buffer, uint16_t length);
bool I2C1_ReadByte(uint8_t device, bool addr_word, uint16_t address, uint8_t *b);
bool I2C1_ReadBuffer(uint8_t device, bool addr_word, uint16_t address, void *buffer, uint16_t length);


/******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif  /* _DRV_I2C1_H_ */
