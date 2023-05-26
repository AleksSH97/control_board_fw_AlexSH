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

#include "log.h"

#include "lwprintf/lwprintf.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define RTCRST_Pin                 LL_GPIO_PIN_0
#define RTCRST_GPIO_Port           GPIOC
#define RTCINT_Pin                 LL_GPIO_PIN_1
#define RTCINT_GPIO_Port           GPIOC

#define RTC_HW_ADDRESS             (0xD0)

#define RTC_REG_SECONDS            (0x00)
#define RTC_REG_MINUTES            (0x01)
#define RTC_REG_HOURS              (0x02)
#define RTC_REG_DAY                (0x03)
#define RTC_REG_DATE               (0x04)
#define RTC_REG_MONTH_CENTURY      (0x05)
#define RTC_REG_YEAR               (0x06)
#define RTC_REG_ALARM_1_SECONDS    (0x07)
#define RTC_REG_ALARM_1_MINUTES    (0x08)
#define RTC_REG_ALARM_1_HOURS      (0x09)
#define RTC_REG_ALARM_1_DAY_DATE   (0x0A)
#define RTC_REG_ALARM_2_MINUTES    (0x0B)
#define RTC_REG_ALARM_2_HOURS      (0x0C)
#define RTC_REG_ALARM_2_DAY_DATE   (0x0D)
#define RTC_REG_CONTROL            (0x0E)
#define RTC_REG_STATUS             (0x0F)
#define RTC_REG_AGING_OFFSET       (0x10)
#define RTC_REG_TEMP_MSB           (0x11)
#define RTC_REG_TEMP_LSB           (0x12)
#define RTC_REG_TEST               (0x13)
#define RTC_REGS_SRAM_OFFSET       (0x14)
#define RTC_REGS_SRAM_LENGTH       (0xFF - 0x14)
#define RTC_REGS_SRAM_END          (0xFF)

#define ADDR_WORD          (true)
#define ADDR_BYTE          (false)

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

typedef enum
{
  RTC_GET_DATE = 0x00,
  RTC_GET_TIME,
  RTC_IDLE = 0xFF
} RTC_MODE_t;

typedef struct
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint16_t ms;
} RTC_TIME_t;

typedef struct
{
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint16_t year;
} RTC_DATE_t;

typedef struct
{
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
