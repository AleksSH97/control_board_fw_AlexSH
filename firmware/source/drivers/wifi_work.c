/**
 ******************************************************************************
 * @file           : wifi.c
 * @author         : Konstantin Soloviev
 * @author         : Dmitry Karasev        <karasev@voltsbattery.com>
 * @brief          : WiFi network manager for ESP8266 module
 ******************************************************************************
 * ----------------- Copyright (c) 2020 VOLTS Battery LLC ------------------- *
 ******************************************************************************
 * This module is a confidential and proprietary property of VOLTS Battery
 * and possession or use of this module requires written permission
 * of VOLTS Battery.
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <string.h>

#include "app/typedefs.h"
#include "cmsis_os.h"

#include "drivers/leds.h"
#include "drivers/apkey.h"
#include "drivers/system.h"

#include "esp/esp.h"
#include "esp/esp_private.h"
#include "esp/esp_parser.h"

#include "app/io/printf.h"
#include "app/io/sprintf.h"
#include "app/io/io_system.h"
#include "app/config.h"
#include "app/shell/shell.h"
#include "app/network/mqtt.h"
#include "app/network/broker.h"

#include "devices/gsm.h"

#include "devices/wifi.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define WIFI_STACK_SIZE              (512)

#define WIFI_RD_BUFF_SIZE            (1024)

#define WIFI_MAX_SCAN_ERRORS         (4)
#define WIFI_MAX_JOIN_ERRORS         (2)
#define WIFI_MAX_NET_CHECK_ERRORS    (10)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
typedef struct
{
  esp_conn_p conn;
  esp_netconn_p netconn_server;
  esp_netconn_p netconn_client;
  esp_pbuf_p pbuf;
  bool restart;
  bool ap_ready;
  bool esp_ready;
  bool sta_ready;
  bool host_connected;
  bool rd_ok;
  u16  readed;
  u8   *rd_buff_p;
  u8   rd_buff[WIFI_RD_BUFF_SIZE];
} WIFI_DATA;

static WIFI_DATA wifi;

static osThreadId wifiApTaskHandle;
static osThreadId wifiStTaskHandle;

volatile bool wifi_ap_mode;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static void TaskWiFiST(void const *argument);
static void TaskWiFiAP(void const *argument);

static char *prvESPErrorHandler(espr_t err);

static espr_t esp_callback_func(esp_evt_t* evt);
static espr_t conn_evt(esp_evt_t* evt);


/******************************************************************************/



/**
 * @brief  ESP8266 wifi module initialization.
 * @retval none
 */
void WIFI_Init(void)
{
  /* Fill by zero the instance of WIFI_DATA struct */
  memset(&wifi, 0, sizeof(wifi));

  /* Init esp */
  espr_t res = esp_init(esp_callback_func, 0);
  if (res != espOK)
    Printf_LogCRLF(CLR_RD"ESP init FAIL! (%s)"CLR_DEF, prvESPErrorHandler(res));

  /* Clear all rtos task handlers */
  wifiStTaskHandle = NULL;
  wifiApTaskHandle = NULL;

  /* By default, the wifi module works in ST mode  */
  WIFI_Start(MODE_WIFI_ST);
}
/******************************************************************************/



/**
 * @brief  This function used for debug MemUsed logs.
 * @retval u32: wifiApTask or wifiStTask total stack size
 */
u32 WIFI_GetWiFiTaskStackSize(void)
{
  return (WIFI_STACK_SIZE);
}
/******************************************************************************/



/**
 * @brief  This function used for debug MemUsed logs.
 * @retval u32: wifiApTask or wifiStTask free stack size
 */
u32 WIFI_GetWiFiTaskStackSizeFree(void)
{
  if (wifi_ap_mode)
    return (wifiApTaskHandle ? uxTaskGetStackHighWaterMark(wifiApTaskHandle) : 0UL);
  else
    return (wifiStTaskHandle ? uxTaskGetStackHighWaterMark(wifiStTaskHandle) : 0UL);
}
/******************************************************************************/




bool WIFI_GetRSSI(s16 *rssi)
{
  esp_sta_info_ap_t ap_info = {0};
  if (esp_sta_get_ap_info(&ap_info, NULL, NULL, 1) != espOK)
    return false;

  *rssi = ap_info.rssi;

  return true;
}
/******************************************************************************/




bool WIFI_IsOk(void)
{
  return wifi.esp_ready;
}
/******************************************************************************/



/**
 * @brief  Function implementing the wifiStTask thread.
 *         This task drives the esp module in STA mode.
 * @param  argument: Not used
 * @retval None
 */
static void TaskWiFiST(void const *argument)
{
  u8 errors_scan_ap   = 0;
  u8 errors_join_st   = 0;
  u8 errors_net_check = 0;

  /* Cyclic check that wifi module is ready */
  while (!wifi.esp_ready)
    osDelay(100);

  for (;;)
  {
    wifi.conn = NULL;

    esp_reset_with_delay(ESP_CFG_RESET_DELAY_DEFAULT, NULL, NULL, 1);

    esp_ap_t aps[10];
    size_t apf;

    espr_t res = esp_set_wifi_mode(ESP_MODE_STA, 0, NULL, NULL, 1);
    if (res == espOK)
    {
      Printf_LogCRLF(CLR_GR"WiFi mode is now "CLR_YL"ST"CLR_DEF);

      bool config_ap_found = false;
      while (!config_ap_found)
      {
        Printf_LogCRLF("WiFi Access points scanning ...");
        LEDs_Yellow(LED_CTRL_BLINK, 33, 330, 0);

        res = esp_sta_list_ap(NULL, aps, ESP_ARRAYSIZE(aps), &apf, NULL, NULL, 1);
        if (res == espOK)
        {
          Printf_LogCRLF(CLR_GR"WiFi Access point scan OK"CLR_DEF);

          for (u8 i = 0; i < apf; i++)
          {
            Printf_LogCRLF(CLR_GR"Wifi AP found: \"%s\", RSSI: %i dBm"CLR_DEF, aps[i].ssid, aps[i].rssi);

            if (strcmp(config.wifi.ssid, aps[i].ssid) == 0)
              config_ap_found = true;
          }

          if (config_ap_found)
          {
            errors_scan_ap = 0;
            LEDs_Yellow(LED_CTRL_BLINK, 33, 33, 0);
            Printf_LogCRLF("WiFi connecting to \"%s\" network ...", config.wifi.ssid);

            res = esp_sta_join(config.wifi.ssid, config.wifi.passw, NULL, 0, NULL, NULL, 1);
            if (res == espOK)
            {
              esp_ip_t ip;
              esp_sta_copy_ip(&ip, NULL, NULL);
              Printf_LogCRLF(CLR_GR"WiFi connected to \"%s\" access point OK"CLR_DEF, config.wifi.ssid);
              Printf_LogCRLF(CLR_GR"WiFi station IP address: %u.%u.%u.%u"CLR_DEF, (int) ip.ip[0],
                                                                                      (int) ip.ip[1],
                                                                                      (int) ip.ip[2],
                                                                                      (int) ip.ip[3]);

              errors_join_st = 0;
            }
            else
            {
              config_ap_found = false;
              Printf_LogCRLF(CLR_RD"ERROR: WiFi connection to \"%s\" network fault! (%s)"CLR_DEF, config.wifi.ssid, prvESPErrorHandler(res));
              osDelay(1000);
              if (++errors_join_st > WIFI_MAX_JOIN_ERRORS)
              {
                osDelay(30000);
                errors_join_st = 0;
                continue;
              }
              else
                continue;
            }
          }
          else
          {
            Printf_LogCRLF(CLR_RD"ERROR: WiFi Access point \"%s\" is not found or has a weak signal!"CLR_DEF, config.wifi.ssid);
            osDelay(5000);
            if (++errors_scan_ap > WIFI_MAX_SCAN_ERRORS)
            {
              osDelay(60000);
              errors_scan_ap = 0;
              continue;
            }
            else
              continue;
          }
        }
        else
          Printf_LogCRLF(CLR_RD"ERROR: WiFi Access point scan failed (%s)"CLR_DEF, prvESPErrorHandler(res));
      }
    }
    else
      Printf_LogCRLF(CLR_RD"ERROR: WiFi set mode ST failed (%s)"CLR_DEF, prvESPErrorHandler(res));


    Printf_LogCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);


    for (;;)
    {
      if (!esp_sta_is_joined())
      {
        osDelay(1000);
        break;
      }


      if (wifi.restart)
      {
        wifi.restart = false;
        break;
      }


      if (!wifi.sta_ready)
      {
        res = esp_ping("8.8.8.8", NULL, NULL, NULL, 1);
        if (res == espOK)
        {
          errors_net_check = 0;
          LEDs_Yellow(LED_CTRL_OFF, 0, 0, 0);
          wifi.sta_ready = true;
          Printf_LogCRLF(CLR_GR"Internet connection \"%s\" OK"CLR_DEF, config.wifi.ssid);

          if (!mqtt_wifi_transport && !esp8266_onair)
          {
            Printf_LogCRLF(CLR_GR"Switching MQTT transport to WiFi"CLR_DEF);
            MQTTClient_Stop();
          }
        }
        else
        {
          if (++errors_net_check > WIFI_MAX_NET_CHECK_ERRORS)
          {
            errors_net_check = 0;
            Printf_LogCRLF(CLR_RD"ERROR: \"%s\" access point doesn't have internet connection!"CLR_DEF, config.wifi.ssid);
            Printf_LogCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);
            continue;
          }
          else
          {
            osDelay(1000);
            continue;
          }
        }
      }


      if (esp8266_onair)
      {
        wifi.sta_ready = false;
        esp8266_logs = true;
        esp_update_sw(NULL, NULL, 1);
        esp8266_logs = false;
        esp8266_onair = false;
        wifi.restart = true;
        GSM_Start();
      }

      osDelay(100);
    }
  }

  osThreadTerminate(NULL);
}
/******************************************************************************/



/**
 * @brief  Function implementing the wifiApTask thread.
 *         This task drives the esp module in AP mode.
 * @param  argument: Not used
 * @retval None
 */
static void TaskWiFiAP(void const *argument)
{
  /* Cyclic check that wifi module is ready */
  while (!wifi.esp_ready)
    osDelay(100);

  for (;;)
  {
    wifi.netconn_server = NULL;
    wifi.netconn_client = NULL;

    esp_reset_with_delay(ESP_CFG_RESET_DELAY_DEFAULT, NULL, NULL, 1);

    wifi.ap_ready = false;
    esp_sta_t stas[1];
    size_t staf;

    espr_t res = esp_set_wifi_mode(ESP_MODE_AP, 0, NULL, NULL, 1);
    if (res == espOK)
    {
      esp_ip_t ip, gw, nm;
      const char* str = NULL;

      str = config.mqtt.local;
      espi_parse_ip(&str, &ip);
      str = config.mqtt.local;
      espi_parse_ip(&str, &gw);
      str = "255.255.255.0";
      espi_parse_ip(&str, &nm);
      Printf_LogCRLF(CLR_GR"WiFi mode is now "CLR_YL"AP"CLR_DEF);

      res = esp_ap_setip(&ip, &gw, &nm, 0, NULL, NULL, 1);
      if (res == espOK)
      {
        res = esp_ap_configure("VOLTS_NET", "volts_local", 9, ESP_ECN_WPA2_PSK, ESP_ARRAYSIZE(stas), 0, 1, NULL, NULL, 1);
        if (res == espOK)
        {
          res = esp_ap_list_sta(stas, ESP_ARRAYSIZE(stas), &staf, NULL, NULL, 1);
          if (res == espOK)
          {
            Printf_LogCRLF(CLR_GR"WiFi Stations scan OK"CLR_DEF);

            for (u8 i = 0; i < staf; i++)
              Printf_LogCRLF(CLR_GR"Wifi Station found: %u.%u.%u.%u"CLR_DEF, stas[i].ip.ip[0], stas[i].ip.ip[1], stas[i].ip.ip[2], stas[i].ip.ip[3]);

            wifi.ap_ready = true;

            wifi.netconn_server = esp_netconn_new(ESP_NETCONN_TYPE_TCP);
            if (wifi.netconn_server != NULL)
            {
              res = esp_netconn_bind(wifi.netconn_server, config.mqtt.port);
              if (res == espOK)
              {
                Printf_LogCRLF(CLR_GR"Server netconn listens on port %u"CLR_DEF, config.mqtt.port);

                res = esp_netconn_listen(wifi.netconn_server);

                for (;;)
                {
                  res = esp_netconn_accept(wifi.netconn_server, &wifi.netconn_client);
                  if (res == espOK)
                  {
                    wifi.host_connected = true;
                    esp_pbuf_p pbuf = NULL;
                    Printf_LogCRLF(CLR_GR"NETCONN new client connected"CLR_DEF);

                    esp_netconn_set_receive_timeout(wifi.netconn_client, 1000);
                    for (;;)
                    {
                      res = esp_netconn_receive(wifi.netconn_client, &pbuf);
                      if (res == espOK)
                      {
                        Printf_LogCRLF(CLR_GR"NETCONN data received, %u/%u bytes"CLR_DEF, (int) esp_pbuf_length(pbuf, 1), (int) esp_pbuf_length(pbuf, 0));

                        if (wifi.pbuf == NULL)
                          wifi.pbuf = pbuf;
                        else
                          esp_pbuf_cat(wifi.pbuf, pbuf);

                        Broker_Parsing(REMOTE_CONNECT, (char *) esp_pbuf_data(pbuf), esp_pbuf_length(pbuf, 0));

                        esp_pbuf_free(wifi.pbuf);
                        wifi.pbuf = NULL;
                      }
                      else if (res == espTIMEOUT)
                      {
                        if (!wifi.host_connected)
                          break;
                        if (wifi.restart)
                          break;
                      }
                      else
                      {
                        Printf_LogCRLF(CLR_RD"NETCONN receiving error (%s)"CLR_DEF, prvESPErrorHandler(res));
                        break;
                      }
                    }
                    if (wifi.netconn_client)
                    {
                      esp_netconn_close(wifi.netconn_client);
                      esp_netconn_delete(wifi.netconn_client);
                      wifi.netconn_client = NULL;
                    }
                    if (wifi.pbuf != NULL)
                    {
                      esp_pbuf_free(wifi.pbuf);
                      wifi.pbuf = NULL;
                    }
                  }
                  else
                  {
                    Printf_LogCRLF(CLR_RD"NETCONN connection accept error (%s)"CLR_DEF, prvESPErrorHandler(res));
                    break;
                  }
                  if (wifi.restart)
                  {
                    wifi.restart = false;
                    break;
                  }
                }
              }
              else
                Printf_LogCRLF(CLR_RD"NETCONN netconn_server cannot bind to port (%s)"CLR_DEF, prvESPErrorHandler(res));
            }
            else
              Printf_LogCRLF(CLR_RD"Cannot create netconn_server NETCONN"CLR_DEF);
            if (wifi.netconn_server)
            {
              esp_netconn_close(wifi.netconn_server);
              esp_netconn_delete(wifi.netconn_server);
              wifi.netconn_server = NULL;
            }
          }
          else
            Printf_LogCRLF(CLR_RD"WiFi Stations scan failed (%s)"CLR_DEF, prvESPErrorHandler(res));
        }
        else
          Printf_LogCRLF(CLR_RD"WiFi configure AP failed (%s)"CLR_DEF, prvESPErrorHandler(res));
      }
      else
        Printf_LogCRLF(CLR_RD"WiFi set IP AP failed (%s)"CLR_DEF, prvESPErrorHandler(res));
    }
    else
      Printf_LogCRLF(CLR_RD"WiFi set mode AP failed (%s)"CLR_DEF, prvESPErrorHandler(res));
  }

  osThreadTerminate(NULL);
}
/******************************************************************************/




bool WIFI_IsReady(void)
{
  if (wifi_ap_mode)
    return (wifi.host_connected);
  else
    return (wifi.sta_ready);
}
/******************************************************************************/



/**
 * @brief  ESP8266 wifi errors.
 * @param  err: Result enumeration used across application functions
 * @retval char: String that describes error code passed to the function
 */
static char *prvESPErrorHandler(espr_t err)
{
  switch (err)
  {
    case espOK:                   return ("OK");                                                        break;
    case espOKIGNOREMORE:         return ("Ignore sending more data");                                  break;
    case espERR:                  return ("AT error");                                                  break;
    case espPARERR:               return ("Wrong parameters");                                          break;
    /* Reboot board if memory leak detected */
    case espERRMEM:               NVIC_SystemReset(); return ("Memory error");                          break;
    case espTIMEOUT:              return ("Timeout");                                                   break;
    case espCONT:                 return ("Still some command to be processed in current command");     break;
    case espCLOSED:               return ("Connection just closed");                                    break;
    case espINPROG:               return ("Operation is in progress");                                  break;
    case espERRNOIP:              return ("Station does not have IP address");                          break;
    /* This is impossible state, when the device is connected to MQTT broker and start the second connection */
    case espERRNOFREECONN:        NVIC_SystemReset(); return ("There is no free connection available to start");
    case espERRCONNTIMEOUT:       return ("Timeout received when connection to access point");          break;
    case espERRPASS:              return ("Invalid password for access point");                         break;
    case espERRNOAP:              return ("No access point found with specific SSID and MAC address");  break;
    case espERRCONNFAIL:          return ("Connection failed to access point");                         break;
    case espERRWIFINOTCONNECTED:  return ("Wifi not connected to access point");                        break;
    case espERRNODEVICE:          return ("Device is not present");                                     break;
    case espERRBLOCKING:          return ("Blocking mode command is not allowed");                      break;
    default:                      return ("???");
  };
}
/******************************************************************************/



/**
 * @brief  This function stops corresponding task depending on
 *         the wifi module run mode.
 * @retval None
 */
void WIFI_Stop(void)
{
  osThreadId st_task = wifiStTaskHandle;
  osThreadId ap_task = wifiApTaskHandle;

  if (st_task || ap_task)
  {
    /* The 'osThreadSuspendAll()/osThreadResumeAll()' way to branch
       the terminate of 2 threads is recommended by freertos.org */
    osThreadSuspendAll();

    /* Kill corresponding task depending on the WiFi mode*/
    if (wifiStTaskHandle != NULL)
    {
      wifiStTaskHandle = NULL;
      osThreadTerminate(st_task);
    }
    if (wifiApTaskHandle != NULL)
    {
      wifiApTaskHandle = NULL;
      osThreadTerminate(ap_task);
    }

    osThreadResumeAll();
  }

  if (wifi.pbuf)
  {
    esp_pbuf_free(wifi.pbuf);
    wifi.pbuf = NULL;
  }

  if (wifi.netconn_client)
  {
    esp_netconn_close(wifi.netconn_client);
    esp_netconn_delete(wifi.netconn_client);
    wifi.netconn_client = NULL;
  }

  if (wifi.netconn_server)
  {
    esp_netconn_close(wifi.netconn_server);
    esp_netconn_delete(wifi.netconn_server);
    wifi.netconn_server = NULL;
  }

  if (wifi.conn)
  {
    esp_conn_close(wifi.conn, 1);
    wifi.conn = NULL;
    esp_sta_quit(NULL, NULL, 1);
  }

  wifi.restart = false;
  wifi.ap_ready = false;
  wifi.sta_ready = false;
  wifi.host_connected = false;

  LEDs_Red(LED_CTRL_ON, 0, 0, 0);
}
/******************************************************************************/



/**
 * @brief  This function starts ESP8266 wifi module in the specified mode.
 * @param  mode_ap: 'true' value starts wifi module in AP mode
 * @retval None
 */
void WIFI_Start(bool mode_ap)
{
  wifi_ap_mode = mode_ap;

  if (mode_ap == MODE_WIFI_AP)
  {
    osThreadDef(wifiApTask, TaskWiFiAP, osPriorityNormal, 0, WIFI_STACK_SIZE);
    wifiApTaskHandle = osThreadCreate(osThread(wifiApTask), NULL);
  }
  else if (mode_ap == MODE_WIFI_ST)
  {
    osThreadDef(wifiStTask, TaskWiFiST, osPriorityNormal, 0, WIFI_STACK_SIZE);
    wifiStTaskHandle = osThreadCreate(osThread(wifiStTask), NULL);
  }

  Printf_LogCRLF("Switch WiFi to %s mode ..."CLR_DEF, wifi_ap_mode ? CLR_YL"AP"CLR_GR : CLR_YL"ST"CLR_GR);

  if (wifi_ap_mode)
    LEDs_Red(LED_CTRL_BLINK, 100, 900, 0);
  else
    LEDs_Red(LED_CTRL_OFF, 0, 0, 0);
}
/******************************************************************************/



/**
 * @brief  This function set restart flag to restart wifi module.
 * @retval None
 */
void WIFI_Restart(void)
{
  wifi.restart = true;
}
/******************************************************************************/



/**
 * @brief  This callback function performs activities depending
 *         on the passed event.
 * @retval espr_t: Result enumeration used across application functions
 */
static espr_t esp_callback_func(esp_evt_t* evt)
{
  switch (esp_evt_get_type(evt))
  {
    case ESP_EVT_AT_VERSION_NOT_SUPPORTED:
    {
      Printf_LogCRLF(CLR_RD"This version API ESP8266 is not supported!"CLR_DEF);
      break;
    }
    case ESP_EVT_INIT_FINISH:
    {
      wifi.esp_ready = true;
      Printf_LogCRLF(CLR_GR"WiFi initialized OK"CLR_DEF);
      break;
    }
    case ESP_EVT_RESET_DETECTED:
    {
      wifi.restart = false;
      wifi.ap_ready = false;
      wifi.sta_ready = false;
      wifi.host_connected = false;
      Printf_LogCRLF("WiFi to reset ...");
      break;
    }
    case ESP_EVT_RESET:
    {
      wifi.restart = false;
      wifi.ap_ready = false;
      wifi.sta_ready = false;
      wifi.host_connected = false;
      Printf_LogCRLF(CLR_GR"WiFi reset OK"CLR_DEF);
      break;
    }
    case ESP_EVT_RESTORE:
    {
      wifi.restart = false;
      wifi.ap_ready = false;
      wifi.sta_ready = false;
      wifi.host_connected = false;
      Printf_LogCRLF(CLR_GR"WiFi restore OK"CLR_DEF);
      break;
    }
    case ESP_EVT_CMD_TIMEOUT:
    {
      Printf_LogCRLF(CLR_RD"WiFi command timeout"CLR_DEF);
      break;
    }
    case ESP_EVT_WIFI_CONNECTED:
    {
      Printf_LogCRLF(CLR_GR"WiFi AP connected OK"CLR_DEF);
      //wifi.sta_ready = true;
      break;
    }
    case ESP_EVT_WIFI_GOT_IP:
    {
      Printf_LogCRLF(CLR_GR"WiFi AP got IP"CLR_DEF);
      break;
    }
    case ESP_EVT_WIFI_DISCONNECTED:
    {
      Printf_LogCRLF(CLR_RD"WiFi AP disconnected!"CLR_DEF);
      wifi.host_connected = false;
      if (mqtt_wifi_transport)
        MQTTClient_Stop();
      break;
    }
    case ESP_EVT_WIFI_IP_ACQUIRED:
    {
      Printf_LogCRLF(CLR_GR"WiFi AP IP acquired"CLR_DEF);
      break;
    }
    case ESP_EVT_STA_LIST_AP:
    {
      Printf_LogCRLF(CLR_GR"WiFi APs listed"CLR_DEF);
      break;
    }
    case ESP_EVT_STA_JOIN_AP:
    {
      espr_t status = esp_evt_sta_join_ap_get_result(evt);
      if (status == espOK)
      {
        esp_ip_t ip;
        esp_sta_copy_ip(&ip, NULL, NULL);
        Printf_LogCRLF(CLR_GR"WiFi join to AP (%u.%u.%u.%u)"CLR_DEF, ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
      }
      else
      {
        wifi.host_connected = false;
        Printf_LogCRLF(CLR_RD"WiFi AP join ERROR! (%u)"CLR_DEF, status);
      }
      break;
    }
    case ESP_EVT_PING:
    {
      break;
    }
    case ESP_EVT_STA_INFO_AP:
    {
      break;
    }
    case ESP_EVT_AP_CONNECTED_STA:
    {
      esp_mac_t *mac;
      mac = esp_evt_ap_connected_sta_get_mac(evt);
      Printf_LogCRLF(CLR_GR"WiFi station connected MAC %X:%X:%X:%X:%X:%X"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
      break;
    }
    case ESP_EVT_AP_DISCONNECTED_STA:
    {
      esp_mac_t *mac;
      mac = esp_evt_ap_disconnected_sta_get_mac(evt);
      Printf_LogCRLF(CLR_RD"WiFi station disconnected! (MAC %X:%X:%X:%X:%X:%X)"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
      wifi.host_connected = false;
      wifi.restart = true;
      break;
    }
    case ESP_EVT_AP_IP_STA:
    {
      esp_ip_t *ip;
      ip = esp_evt_ap_ip_sta_get_ip(evt);
      memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
      Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
      Printf_LogCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
      break;
    }
    case ESP_EVT_SERVER:
    {
      espr_t res = esp_evt_server_get_result(evt);
      esp_port_t port = esp_evt_server_get_port(evt);
      u8 ena = esp_evt_server_is_enable(evt);
      Printf_LogCRLF(CLR_GR"NETCONN server: res=%u, port=%u, ena=%u"CLR_DEF, res, port, ena);
//      esp_ip_t *ip;
//      ip = esp_evt_ap_ip_sta_get_ip(evt);
//      memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
//      Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
//      Printf_LogCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
//      wifi.sta_ready = true;
      break;
    }
    default:
    {
      Printf_LogCRLF("WiFi ESP callback.%u? ", esp_evt_get_type(evt));
      break;
    }
  }
  return espOK;
}
/******************************************************************************/




static espr_t conn_evt(esp_evt_t* evt)
{
  esp_conn_p conn = esp_conn_get_from_evt(evt);

  if (conn == NULL)
    return espERR;

  switch (esp_evt_get_type(evt))
  {
    case ESP_EVT_CONN_ACTIVE:
    {
      Printf_LogCRLF(CLR_GR"WiFi connection is active"CLR_DEF);
      break;
    }
    case ESP_EVT_CONN_ERROR:
    {
      Printf_LogCRLF(CLR_RD"WiFi connection ERROR"CLR_DEF);
      break;
    }
    case ESP_EVT_CONN_CLOSE:
    {
      espr_t status = esp_evt_conn_close_get_result(evt);
      if (status == espOK)
        Printf_LogCRLF(CLR_RD"WiFi connection closed by %s"CLR_DEF, evt->evt.conn_active_close.forced ? "user" : "remote host");
      else
        Printf_LogCRLF(CLR_RD"WiFi connection close ERROR! (%u)"CLR_DEF, status);

      wifi.sta_ready = false;
      if (mqtt_wifi_transport)
        MQTTClient_Stop();
      break;
    }
    case ESP_EVT_CONN_RECV:
    {
      esp_pbuf_p pbuf = evt->evt.conn_data_recv.buff;
//      Printf_LogCRLF(CLR_YL"WiFi connection data received"CLR_DEF);
      if (pbuf != NULL)
      {
        size_t len;
        len = esp_pbuf_length(pbuf, 1);
//        Printf_LogCRLF(CLR_YL"WiFi length of data: %u bytes"CLR_DEF, (int) len);
        if (len > WIFI_RD_BUFF_SIZE)
          len = WIFI_RD_BUFF_SIZE;
        memcpy(wifi.rd_buff, esp_pbuf_data(pbuf), len);
        wifi.readed = len;
        wifi.rd_buff_p = wifi.rd_buff;
        wifi.rd_ok = true;
      }
      esp_conn_recved(conn, pbuf);
      break;
    }
    case ESP_EVT_CONN_SEND:
    {
      break;
    }
    case ESP_EVT_CONN_POLL:
    {
      break;
    }
    default:
    {
      Printf_LogCRLF("WiFi conn callback.%u? ", esp_evt_get_type(evt));
      break;
    }
  }
  return espOK;
}
/******************************************************************************/




u16 WIFI_WiFiClientIsDataAvailable(void)
{
  return (wifi.rd_ok ? wifi.readed : 0);
}
/******************************************************************************/




bool WIFI_WiFiClientConnect(char *host, u16 port)
{
  if (wifi_ap_mode)
  {
    return (WIFI_WiFiClientIsConnected());
  }
  else
  {
    LEDs_Yellow(LED_CTRL_BLINK, 200, 200, 0);

    if (WIFI_WiFiClientIsConnected())
      return (true);

    Printf_LogCRLF("WiFi connecting to %s:%u ...", host, port);
    espr_t res = esp_conn_start(&wifi.conn, ESP_CONN_TYPE_TCP, host, port, NULL, conn_evt, 1);
    if (res == espOK)
    {
      wifi.host_connected = true;
      Printf_LogCRLF(CLR_GR"WiFi connecting to host OK"CLR_DEF);
      return (true);
    }
    Printf_LogCRLF(CLR_RD"WiFi connection to host failed! (%s)"CLR_DEF, prvESPErrorHandler(res));
    wifi.sta_ready = false;

    return (false);
  }
}
/******************************************************************************/




bool WIFI_WiFiClientIsConnected(void)
{
  return (wifi.host_connected);
}
/******************************************************************************/




void WIFI_WiFiClientStop(void)
{
  if (wifi_ap_mode)
  {
    if (wifi.netconn_client)
      esp_netconn_close(wifi.netconn_client);
  }
  else
  {
    if (wifi.conn)
    {
      esp_conn_close(wifi.conn, 1);
      esp_sta_quit(NULL, NULL, 1);
    }
  }
//  wifi.restart = true;
  wifi.ap_ready = false;
  wifi.sta_ready = false;
  wifi.host_connected = false;
}
/******************************************************************************/




void WIFI_WiFiClientFlush(void)
{
  // flush rx buffer
}
/******************************************************************************/




void WIFI_WiFiClientPeek(void)
{
  // not implemented
}
/******************************************************************************/




u16 WIFI_WiFiClientWrite(u8 *buff, u16 len)
{
  if (wifi_ap_mode)
  {
    Broker_Parsing(LOCAL_CONNECT, (char *) buff, len);
    return (len);
  }
  else
  {
    if (!WIFI_WiFiClientIsConnected())
      return (0);

    size_t writed = 0;
    esp_conn_send(wifi.conn, buff, len, &writed, 1);
    return (writed);
  }
//for (u16 l = 0; l < len; l++)
//PutLogString(CONT, "%c", (buff[l] >= '0') ? buff[l] : '_');
//Printf_LogCRLF("");
//for (u16 l = 0; l < len; l++)
//PutLogString(CONT, "%X ", buff[l]);
//Printf_LogCRLF("");
}
/******************************************************************************/




u16 WIFI_WiFiNetconnWrite(u8 *buff, u16 len)
{
  if (wifi_ap_mode)
  {
//    if (!WIFI_WiFiClientIsConnected())
//      return (0);

    espr_t res = esp_netconn_write(wifi.netconn_client, buff, len);
    if (res == espOK)
    {
      res = esp_netconn_flush(wifi.netconn_client);
      if (res == espOK)
      {
        return (len);
      }
      else
        Printf_LogCRLF(CLR_RD"WiFi flush netconn failed! (%s)"CLR_DEF, prvESPErrorHandler(res));
    }
    else
      Printf_LogCRLF(CLR_RD"WiFi write netconn failed! (%s)"CLR_DEF, prvESPErrorHandler(res));
//Printf_LogCRLF("res=%u", res);
//for (u16 l = 0; l < len; l++)
//PutLogString(CONT, "%c", (buff[l] >= '0') ? buff[l] : '_');
//Printf_LogCRLF("");
//for (u16 l = 0; l < len; l++)
//PutLogString(CONT, "%X ", buff[l]);
//Printf_LogCRLF("");
  }
  return (0);
}
/******************************************************************************/




u16 WIFI_WiFiClientRead(u8 *buff, u16 len)
{
  if (!WIFI_WiFiClientIsConnected())
    return (0);

  if (wifi.rd_ok)
  {
    if (len > wifi.readed)
      len = wifi.readed;
    memcpy(buff, wifi.rd_buff_p, len);
    wifi.rd_buff_p += len;
    wifi.readed -= len;
    if (!wifi.readed)
      wifi.rd_ok = false;

    return (len);
  }
  wifi.rd_ok = false;

  return (0);
}
/******************************************************************************/




u16 WIFI_WiFiLocalWrite(u8 *buff, u16 len)
{
  memcpy(wifi.rd_buff, buff, len);
  wifi.readed = len;
  wifi.rd_buff_p = wifi.rd_buff;
  wifi.rd_ok = true;
  return (len);
}
/******************************************************************************/
