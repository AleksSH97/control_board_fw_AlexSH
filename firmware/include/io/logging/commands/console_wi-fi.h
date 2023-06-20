/**
 ******************************************************************************
 * @file           : console_wi-fi.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for console wi-fi commands
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IO_LOGGING_COMMANDS_CONSOLE_WI_FI_H_
#define IO_LOGGING_COMMANDS_CONSOLE_WI_FI_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#include "microrl.h"
#include "microrl_config.h"
#include "wi-fi.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
int ConsoleWiFi(microrl_t *microrl_ptr, int argc, const char * const *argv);
void Console_WIFiPrintMenu(void);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* IO_LOGGING_COMMANDS_CONSOLE_WI_FI_H_ */
