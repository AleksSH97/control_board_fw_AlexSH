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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define WIFI_MODE_AP                 (true)
#define WIFI_MODE_ST                 (false)

#ifndef WIFI_CMSIS_OS2_ENA
#define WIFI_CMSIS_OS2_ENA           1
#endif

#ifndef WIFI_USE_LWESP
#define WIFI_USE_LWESP               0
#endif

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

  WIFI_CLOSE_CONNECTION_ERROR,

  WIFI_UNKNOWN = 0xFF,
} WIFI_ERROR_t;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
uint8_t WiFiSetError(WIFI_ERROR_t error);
uint8_t WiFiGetError(void);

void WiFiErrorHandler(WIFI_ERROR_t error);
void WiFiStop(void);
void WiFiGetMac(void);
uint8_t WiFiGetInfoAp(void);

uint8_t WiFiStart(bool mode_ap);
void WiFiInit(void);
void WiFiApTask(void *argument);
void WiFiStTask(void *argument);

void TaskWiFiST(void const *argument);
void TaskWiFiAP(void const *argument);


/******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif /* WI_FI_H_ */
