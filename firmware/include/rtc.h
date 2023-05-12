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

#define RTC_DATE_BUF_SIZE          (11u)
#define RTC_TIME_BUF_SIZE          (8u)


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
typedef enum {
  RTC_OK = 0x00,
  RTC_INIT_ERROR,
  RTC_TRANSMIT_ERROR,
  RTC_RECEIVE_ERROR,
  RTC_FLAG_ERROR,
  RTC_ZERO_POINTER_ERROR,
  RTC_SET_MODE_ERROR,
  RTC_SET_DATE_BUFFER_ERROR,
  RTC_SET_TIME_BUFFER_ERROR,
  RTC_CHECK_DATE_ERROR,
  RTC_CHECK_TIME_ERROR,
  RTC_UNDEFINED_ERROR = 0xFF
} RTC_ERROR_t;

typedef enum {
  RTC_GET_DATE = 0x00,
  RTC_GET_TIME,
  RTC_IDLE = 0xFF
} RTC_MODE_t;

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
  uint16_t year;
} RTC_DATE_t;

typedef struct {
  RTC_TIME_t time;
  RTC_DATE_t date;
  RTC_ERROR_t error;
  RTC_MODE_t mode;

  char date_buf[RTC_DATE_BUF_SIZE];
  char time_buf[RTC_TIME_BUF_SIZE];
} RTC_INFO_t;

extern RTC_INFO_t rtc_info;

/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t RtcInit(void);
void RtcErrorHandler(RTC_ERROR_t error);
void RtcInitTask(void);
uint8_t RtcGetDate(void);
uint8_t RtcSetDate(char *buf);
uint8_t RtcSetTime(char *buf);
uint8_t RtcGetTime(void);
uint8_t RtcSetMode(RTC_MODE_t mode);
RTC_MODE_t RtcGetMode(void);
void RtcSetError(RTC_ERROR_t error);
RTC_ERROR_t RtcGetError(void);

/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTC_H_ */
