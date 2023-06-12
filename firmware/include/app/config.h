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
// INIT CONFIG_ID
#define CONFIG_ID    (1u)
#define CONFIG_DEVICE_ESS_CONTROl_BOARD    (0x05u)

#define CONFIG_DEVICE_TYPE     (CONFIG_DEVICE_ESS_CONTROl_BOARD)


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

typedef enum
{
  CONFIG_RECORD_TYPE_BOOL = 0,
  CONFIG_RECORD_TYPE_S8,
  CONFIG_RECORD_TYPE_U8,
  CONFIG_RECORD_TYPE_S16,
  CONFIG_RECORD_TYPE_U16,
  CONFIG_RECORD_TYPE_S32,
  CONFIG_RECORD_TYPE_U32,
  CONFIG_RECORD_TYPE_U32_HEX,
  CONFIG_RECORD_TYPE_FLOAT,
  CONFIG_RECORD_TYPE_STRING
} CONFIG_RECORDS_TYPES;

typedef enum
{
  CONFIG_RECORD_GROUP_NONE = 0,
  CONFIG_RECORD_GROUP_GPRS,
  CONFIG_RECORD_GROUP_WIFI,
  CONFIG_RECORD_GROUP_MQTT,
  CONFIG_RECORD_GROUP_WIREN,
  CONFIG_RECORD_GROUP_GRID_CHECK,
  CONFIG_RECORD_GROUP_PWR_LOAD,
  CONFIG_RECORD_GROUP_PV_LOAD,
  CONFIG_RECORD_GROUP_INVPAROP,
  CONFIG_RECORD_GROUP_FAN_CTRL
} CONFIG_RECORDS_GROUPS;

typedef struct
{

} CONFIG_RECORD_BOOL;

typedef struct
{
  int8_t min;
  int8_t max;
} CONFIG_RECORD_S8;

typedef struct
{
  uint8_t min;
  uint8_t max;
} CONFIG_RECORD_U8;

typedef struct
{
  int16_t min;
  int16_t max;
} CONFIG_RECORD_S16;

typedef struct
{
  uint16_t min;
  uint16_t max;
} CONFIG_RECORD_U16;

typedef struct
{
  int32_t min;
  int32_t max;
} CONFIG_RECORD_S32;

typedef struct
{
  uint32_t min;
  uint32_t max;
} CONFIG_RECORD_U32;

typedef struct
{
  float min;
  float max;
} CONFIG_RECORD_FLOAT;

typedef struct
{
  uint8_t max;
} CONFIG_RECORD_STRING;

typedef struct
{
  void *config;
  uint8_t type;
  uint8_t group;
  uint8_t perm;
  char *name;
  union
  {
    CONFIG_RECORD_BOOL   bool_t;
    CONFIG_RECORD_S8     int8_t;
    CONFIG_RECORD_U8     uint8_t;
    CONFIG_RECORD_S16    int16_t;
    CONFIG_RECORD_U16    uint16_t;
    CONFIG_RECORD_S32    int32_t;
    CONFIG_RECORD_U32    uint32_t;
    CONFIG_RECORD_FLOAT  float_t;
    CONFIG_RECORD_STRING string_t;
  };
} CONFIG_DESCRIPTIONS;


/******************************************************************************/
/******************************************************************************/
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
  uint32_t pin;
  char local[32];
  char host[32];
  char login[16];
  char passw[16];
  uint16_t port;
  uint8_t data_publish_timeout_s;
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
