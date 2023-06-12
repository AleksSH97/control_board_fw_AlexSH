/**
 ******************************************************************************
 * @file           : config.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file of config
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/


typedef enum
{
  CONFIG_INIT_OK = 0,
  CONFIG_INIT_RESTORE_OK,
  CONFIG_INIT_INIT_OK,
  CONFIG_INIT_UPDATE_OK,
  CONFIG_INIT_ERROR_READ,
  CONFIG_INIT_ERROR_INIT,
  CONFIG_INIT_ERROR_UPDATE
} CONFIG_INIT_RESULT;

typedef struct __attribute__((__packed__))
{
  uint16_t size;
  uint16_t dev_type;
  uint16_t id;
} CONFIG_INFO;

typedef struct __attribute__((__packed__))
{
  bool enabled;
  char apn[32];
  char login[16];
  char password[16];
} CONFIG_GPRS;

typedef struct __attribute__((__packed__))
{
  bool enabled;
  char ssid[32];
  char passw[32];
} CONFIG_WIFI;

typedef struct __attribute__((__packed__))
{
  u32 pin;
  char local[32];
  char host[32];
  char login[16];
  char passw[16];
  u16 port;
  u8 data_publish_timeout_s;
} CONFIG_MQTT;

typedef struct __attribute__((__packed__))
{
  uint16_t crc16;
  CONFIG_INFO info;

  bool beep_enabled;

  CONFIG_GPRS gprs;
  CONFIG_WIFI wifi;
  CONFIG_MQTT mqtt;
} ESS_CONFIG;

extern ESS_CONFIG config;
extern ESS_CONFIG init_config;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* APP_CONFIG_H_ */
