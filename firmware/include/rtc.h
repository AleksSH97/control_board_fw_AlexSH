/**
 ******************************************************************************
 * @file           : rtc.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for RTC
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RTC_H_
#define RTC_H_

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
  RTC_OK = 0x00,
  RTC_INIT_ERROR,
  RTC_TRANSMIT_ERROR,
  RTC_RECEIVE_ERROR,
  RTC_FLAG_ERROR,
} RtcError_t;



/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTC_H_ */
