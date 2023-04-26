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

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "rtc_i2c.h"
#include "log.h"

#include "lwprintf/lwprintf.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define RTC_REQUEST_WRITE          (0x00)
#define RTC_REQUEST_READ           (0x01)

#define RTC_HW_ADDRESS             (0xD0)


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
typedef enum {
  RTC_OK = 0x00,
  RTC_INIT_ERROR,
  RTC_TRANSMIT_ERROR,
  RTC_RECEIVE_ERROR,
  RTC_FLAG_ERROR,
} RTC_STATUS_t;

typedef struct {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint16_t ms;
} RTC_TIME_t;

typedef struct {
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint8_t year;
} RTC_DATE_t;

typedef struct {
  RTC_TIME_t time;
  RTC_DATE_t date;
} RTC_INFO_t;

extern RTC_INFO_t rtc_info;
extern RTC_STATUS_t rtc_status;

/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t RtcInit(void);
void RtcErrorHandler(RTC_STATUS_t error);
void RtcInitTask(void);

/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTC_H_ */
