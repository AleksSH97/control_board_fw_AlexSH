/**
 * \file            esp_config.h
 * \brief           User-defined configs file
 */

/*
 * Copyright (c) 2018 Tilen Majerle
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ESP-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#ifndef ESP_HDR_CONFIG_H
#define ESP_HDR_CONFIG_H

/*
 * Open "include/esp/esp_config_default.h" and
 * copy & replace here settings you want to change values
 */
#define ESP_CFG_OS                          1
#define ESP_CFG_SYS_PORT                    ESP_SYS_PORT_CMSIS_OS2

#define ESP_CFG_CONN_MAX_DATA_LEN           2048
#define ESP_CFG_CONN_MAX_RECV_BUFF_SIZE     1460

#define ESP_CFG_AT_PORT_BAUDRATE            115200

#define ESP_CFG_MODE_STATION                1
#define ESP_CFG_MODE_ACCESS_POINT           1

#define ESP_CFG_DBG                         ESP_DBG_OFF

#define ESP_CFG_INPUT_USE_PROCESS           0

#define ESP_CFG_MAX_SSID_LENGTH             32
#define ESP_CFG_MAX_PWD_LENGTH              32

#define ESP_USE_TX_RX_INTERRUPT             1

#define ESP_CFG_NETCONN                     1
#define ESP_CFG_PING                        1

/* After user configuration, call default config to merge config together */
#include "esp/esp_config_default.h"

#endif /* ESP_HDR_CONFIG_H */
