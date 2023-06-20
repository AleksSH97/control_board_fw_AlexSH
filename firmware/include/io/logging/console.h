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

typedef void (*help_command_fn)(void);

typedef enum
{
  CONSOLE_OK = 0x00,
  CONSOLE_NO_CMD = 0x01,

  CONSOLE_ERROR = 0x10,
  CONSOLE_ERROR_UNKNOWN_COMMAND,
  CONSOLE_ERROR_MAX_ARGS
} console_error_t;

typedef struct
{
  help_command_fn    help_command;
} console_ctrl_t;

typedef struct
{
  console_error_t error;
  console_ctrl_t fns;
} console_t;

extern console_t console;

/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void ConsoleInit(void);
void ConsoleStart(void);
char ConsoleGetChar(void);
void ConsoleClearScreenSetup(void);
void ConsoleClearScreen(void);
void ConsoleBack(void);
void ConsoleError(void);

int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleExecuteMain(microrl_t* microrl_ptr, int argc, const char* const *argv);
int ConsoleBuff(microrl_t *microrl_ptr, int argc, const char * const *argv);
int ConsoleCalendar(microrl_t *microrl_ptr, int argc, const char * const *argv);

char **ConsoleComplete(int argc, const char * const *argv);

void ConsoleGetVersion(char* ver_str);
void ConsoleInsertChar(char ch);

void ConsolePrintHelp(void);
void ConsolePrintBuff(microrl_t *microrl_ptr);
void ConsolePrintWelcome(microrl_t *microrl_ptr);
void ConsoleSigint(microrl_t *microrl_ptr);

void ConsoleSetHelp(void (*fn)(void));

/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* CONSOLE_H_ */
