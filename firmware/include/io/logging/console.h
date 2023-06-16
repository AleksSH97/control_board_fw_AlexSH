/**
 ******************************************************************************
 * @file           : console.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for console
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONSOLE_H_
#define CONSOLE_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#include "log.h"
#include "rtc.h"
#include "microrl.h"
#include "microrl_config.h"
#include "wi-fi.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
/**
 * \brief           Skip login + passw for debug
 */
#ifndef CONSOLE_NO_PASSW
#define CONSOLE_NO_PASSW                1
#endif


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
extern bool esp8266_update;

/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void ConsoleInit(void);
void ConsoleStart(void);
char ConsoleGetChar(void);
void ConsoleClearScreenSetup(void);

int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleExecuteMain(microrl_t* microrl_ptr, int argc, const char* const *argv);
int ConsoleBuff(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleCalendar(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleAudio(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleAccelerometer(microrl_t *microrl_ptr, int argc, const char * const *argv);
char **ConsoleComplete(int argc, const char * const *argv);

void ConsoleGetVersion(char* ver_str);
void ConsoleInsertChar(char ch);

void ConsolePrintHelp(void);
void ConsolePrintBuff(microrl_t *microrl_ptr);
void ConsolePrintWelcome(microrl_t *microrl_ptr);
void ConsolePrintVisualizer(microrl_t *microrl_ptr);
void ConsoleSigint(microrl_t *microrl_ptr);

/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* CONSOLE_H_ */
