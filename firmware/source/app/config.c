/**
 ******************************************************************************
 * @file           : config.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Config of board
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "config.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
ESS_CONFIG config;

ESS_CONFIG init_config = {
    .crc16 = 0xFFFF,
    .info = {
      .size = sizeof(ESS_CONFIG),
      .dev_type = CONFIG_DEVICE_TYPE,
      .id = CONFIG_ID
    },
    .beep_enabled = false,
    .gprs = {
        .enabled = false,
        .apn = {"internet"},
        .login = {"gdata"},
        .password = {"gdata"}
    },
    .wifi = {
        .enabled = true,
        .ssid = {"ssid"},
        .passw = {"passw"}
    },
    .mqtt = {
      .pin   = 10001,
      .local = {"192.168.9.1"},
      .host  = {"broker.***.com"},
      .login = {""},
      .passw = {""},
      .port  = 61000,
      .data_publish_timeout_s = 5
    }
};

CONFIG_DESCRIPTIONS config_description[] =
{
    {&config.beep_enabled,                    CONFIG_RECORD_TYPE_BOOL,     CONFIG_RECORD_GROUP_NONE,         0x00, "beep.enabled"       ,  {.bool_t      = {}}},
    {&config.gprs.enabled,                    CONFIG_RECORD_TYPE_BOOL,     CONFIG_RECORD_GROUP_GPRS,         0x00, "gprs.enabled"       ,  {.bool_t      = {}}},
    {&config.gprs.apn,                        CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_GPRS,         0x00, "gprs.apn"           ,  {.string_t    = {31}}},
    {&config.gprs.login,                      CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_GPRS,         0x00, "gprs.login"         ,  {.string_t    = {15}}},
    {&config.gprs.password,                   CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_GPRS,         0x00, "gprs.password"      ,  {.string_t    = {15}}},
    {&config.wifi.enabled,                    CONFIG_RECORD_TYPE_BOOL,     CONFIG_RECORD_GROUP_WIFI,         0x00, "wifi.enabled"       ,  {.bool_t      = {}}},
    {&config.wifi.ssid,                       CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_WIFI,         0x00, "wifi.ssid"          ,  {.string_t    = {31}}},
    {&config.wifi.passw,                      CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_WIFI,         0x00, "wifi.passw"         ,  {.string_t    = {31}}},
    {&config.mqtt.pin,                        CONFIG_RECORD_TYPE_U32,      CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.pin"           ,  {.uint32_t    = {10001, 99999}}},
    {&config.mqtt.local,                      CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.local"         ,  {.string_t    = {31}}},
    {&config.mqtt.host,                       CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.host"          ,  {.string_t    = {31}}},
    {&config.mqtt.port,                       CONFIG_RECORD_TYPE_U16,      CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.port"          ,  {.uint16_t    = {1, 65535}}},
    {&config.mqtt.login,                      CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.login"         ,  {.string_t    = {15}}},
    {&config.mqtt.passw,                      CONFIG_RECORD_TYPE_STRING,   CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.passw"         ,  {.string_t    = {15}}},
    {&config.mqtt.data_publish_timeout_s,     CONFIG_RECORD_TYPE_U8,       CONFIG_RECORD_GROUP_MQTT,         0x00, "mqtt.publish_s"     ,  {.uint8_t     = {5, 250}}},
};

CONFIG_INIT_RESULT init_result;

/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/


/******************************************************************************/

























