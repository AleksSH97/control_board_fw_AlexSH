/**
 ******************************************************************************
 * @file           : wi-fi.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for WI-FI
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WI_FI_H_
#define WI_FI_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "esp/esp.h"
#include "esp/esp_private.h"
#include "esp/esp_parser.h"

#include "log.h"
#include "io_system.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define WIFI_NUM_OF_ERRORS           (255u)

#define WIFI_MAX_SCAN_ERRORS         (4u)
#define WIFI_MAX_JOIN_ERRORS         (2u)
#define WIFI_MAX_NET_CHECK_ERRORS    (10u)

#define WIFI_MODE_AP                 (true)
#define WIFI_MODE_ST                 (false)


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
typedef enum
{
  WIFI_OK = 0x00,
  WIFI_INIT_ERROR,
  WIFI_START_ERROR,

  WIFI_SET_ERROR,

  WIFI_SCAN_AP_ERROR,
  WIFI_JOIN_ST_ERROR,
  WIFI_NET_CHECK_ERROR,

  WIFI_UNKNOWN = 0xFF,
} WIFI_ERROR_t;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t WiFiSetError(WIFI_ERROR_t error);
uint8_t WiFiGetError(void);

void WiFiErrorHandler(WIFI_ERROR_t error);

uint8_t WiFiStart(bool mode_ap);
void WiFiApTask(void *argument);
void WiFiStTask(void *argument);


/******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif /* WI_FI_H_ */
