/**
 ******************************************************************************
 * @file           : io_system.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for input/output system
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IO_LOGGING_IO_SYSTEM_H_
#define IO_LOGGING_IO_SYSTEM_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lwprintf/lwprintf.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
typedef enum {
    IO_IDLE = 0x00,
    IO_LOGS = 0x01,
    IO_CONSOLE = 0x02,
} IOSYS_MODE;

typedef void (*rx_handler_fn)(char rx);

typedef struct {
    rx_handler_fn rx_handler;
    IOSYS_MODE mode;
    osTimerId_t  soft_timeout;

    bool flag;
} IO_SYSTEM;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void IoSystemSetMode(IOSYS_MODE mode);
void IoSystemInit(void);
IOSYS_MODE IoSystemGetMode(void);
void IoSystemClearRxQueue(void);
bool IoSystemGetByte(uint8_t *data, uint32_t timeout_ms);
bool IoSystemIsTxBufferFull(void);
bool IoSystemPutDataToRxBuffer(const void* data, size_t len);
bool IoSystemPutDataToTxBuffer(const void* data, size_t len);
void IoSystemTxTask(void *argument);
void IoSystemRxTask(void *argument);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* IO_LOGGING_IO_SYSTEM_H_ */
