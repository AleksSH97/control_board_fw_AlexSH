/**
 ******************************************************************************
 * @file           : wi-fi.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Wi-Fi driver
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdlib.h>

#include "wi-fi.h"

#include "lwesp/lwesp.h"
#include "lwesp/lwesp_netconn.h"
#include "lwesp/lwesp_ping.h"
#include "lwesp/lwesp_private.h"
#include "lwesp/lwesp_parser.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define WIFI_STACK_SIZE              (512u)

#define WIFI_RD_BUFF_SIZE            (1024u)

#define WIFI_MAX_SCAN_ERRORS         (4u)
#define WIFI_MAX_JOIN_ERRORS         (2u)
#define WIFI_MAX_NET_CHECK_ERRORS    (10u)

#define WIFI_NUM_OF_ERRORS           (255u)

#define WIFI_MAX_SCAN_ERRORS         (4u)
#define WIFI_MAX_JOIN_ERRORS         (2u)
#define WIFI_MAX_NET_CHECK_ERRORS    (10u)

#define WIFI_RF_CHANNEL              (9u)

#define WIFI_NOT_HIDE                (0u)
#define WIFI_HIDE                    (1u)

#define WIFI_NOT_DEFAULT             (0u)
#define WIFI_DEFAULT                 (1u)

#define WIFI_BLOCKING                (1u)

#define WIFI_RECEIVE_TIMEOUT         (1000u)

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
osThreadId_t WiFiApTaskHandle;
osThreadId_t WiFiStTaskHandle;

const osThreadAttr_t WifiApTask_attributes = {
      .name = "WifiApTask",
      .stack_size = 128 * 4,
      .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t WifiStTask_attributes = {
      .name = "WifiStTask",
      .stack_size = 128 * 4,
      .priority = (osPriority_t) osPriorityNormal,
};

typedef struct
{
  lwesp_conn_p connection;
  lwesp_netconn_p netconnection_server;
  lwesp_netconn_p netconnection_client;
  lwesp_pbuf_p    packet_buffer;

  WIFI_ERROR_t error;

  bool ap_mode;
  bool restart;
  bool ap_ready;
  bool esp_ready;
  bool sta_ready;
  bool host_connected;
  bool rd_ok;
} WIFI_DATA_t;

static WIFI_DATA_t wifi;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
lwespr_t esp_callback_function(lwesp_evt_t* event);
char *prvlwespErrorHandler(lwespr_t error);

uint8_t prvWiFiResetWithDelay(void);
uint8_t prvWiFiSetMode(uint8_t mode);
uint8_t prvWiFiListAp(lwesp_ap_t *access_point, size_t *access_point_find);
uint8_t prvWiFiAccessPointsFound(size_t access_point_find, lwesp_ap_t *access_point, bool *config_ap_found);
uint8_t prvWiFiStaJoin(void);
uint8_t prvWiFiCopyIp(lwesp_ip_t *ip);
uint8_t prvWiFiStaIsJoined(void);
uint8_t prvWiFiPing(void);
uint8_t prvWiFiParseIp(const char **str, lwesp_ip_t *ip);
uint8_t prvWiFiSetIp(lwesp_ip_t *ip, lwesp_ip_t *gw, lwesp_ip_t *nm);
uint8_t prvWiFiApConfigure(const char *ssid, const char *password, uint8_t channel, lwesp_ecn_t encryption, uint8_t max_stations, uint8_t hide, uint8_t def, const lwesp_api_cmd_evt_fn evt_fn, void *const evt_argument, const uint32_t blocking);
uint8_t prvWiFiApListSta(lwesp_sta_t *stations, size_t *stations_quantity, const uint32_t blocking);
uint8_t prvWiFiConnectionNew(WIFI_DATA_t *wifi);
uint8_t prvWiFiBindConnection(lwesp_netconn_p netconnection_server, uint16_t port);
uint8_t prvWiFiListenConnection(lwesp_netconn_p netconnection_server);
uint8_t prvWiFiAcceptConnection(lwesp_netconn_p netconnection_server, lwesp_netconn_p *netconnection_client);
uint8_t prvWiFiReceiveConnection(lwesp_netconn_p netconnection_client, lwesp_pbuf_p* pbuf);
uint8_t prvWiFiCloseConnection(lwesp_conn_p connection, const uint32_t blocking);

void prvWiFiStationList(lwesp_sta_t *stations, size_t stations_quantity);
void prvWiFiSetReceiveTimeout(lwesp_netconn_p netconnection_client, uint32_t timeout);
void prvWiFiConcatenatePacketBuffers(lwesp_pbuf_p head, const lwesp_pbuf_p tail);
void prvWiFiFreePacketBuffer(lwesp_pbuf_p packet_buffer);
void prvWiFiNetConnectionClose(lwesp_netconn_p netconnection_client);
void prvWiFiNetConnectionDelete(lwesp_netconn_p netconnection_client);


/******************************************************************************/


/**
 * @brief  Wi-Fi module initialization.
 * @retval Current error instance
 */
void WiFiInit(void)
{
  PrintfLogsCRLF(CLR_RD"WI-FI INIT"CLR_DEF);

  uint8_t res = WIFI_OK;

  memset(&wifi, 0, sizeof(wifi));

  lwespr_t output = lwesp_init(esp_callback_function, 0);
  if (output != lwespOK)
    PrintfLogsCRLF(CLR_RD"ESP init FAIL! (%s)"CLR_DEF, prvlwespErrorHandler(output));

  WiFiStTaskHandle = NULL;
  WiFiApTaskHandle = NULL;

  res = WiFiStart(WIFI_MODE_ST);

  if (res != WIFI_OK)
    WiFiErrorHandler(res);
}
/******************************************************************************/




/**
 * @brief  Start WiFi module in the specified mode.
 * @param  mode_ap: 'true' value starts wifi module in AP mode
 * @retval uint8_t: Current error instance
 */
uint8_t WiFiStart(bool mode_ap)
{
  PrintfLogsCRLF(CLR_DEF"WI-FI START"CLR_DEF);

  wifi.ap_mode = mode_ap;

  if (mode_ap == WIFI_MODE_AP)
  {
    WiFiApTaskHandle = osThreadNew(WiFiApTask, NULL, &WifiApTask_attributes);
    if (WiFiApTaskHandle == NULL)
      return WIFI_START_ERROR;
  }
  else if (mode_ap == WIFI_MODE_ST)
  {
    PrintfLogsCRLF(CLR_DEF"WI-FI MODE ST"CLR_DEF);
    WiFiStTaskHandle = osThreadNew(WiFiStTask, NULL, &WifiStTask_attributes);
    if (WiFiStTaskHandle == NULL)
      return WIFI_START_ERROR;
  }

  PrintfLogsCRLF("Switch WiFi to %s mode ..."CLR_DEF,
      wifi.ap_mode ? CLR_YL"AP"CLR_GR : CLR_YL"ST"CLR_GR);

  if (wifi.ap_mode)
    IndicationLedRed();
  else
    IndicationLedYellow();

  return WIFI_OK;
}
/******************************************************************************/




/**
 * @brief  Function implementing the wifiApTask thread.
 *         This task drives the esp module in AP mode.
 * @param  argument: Not used
 * @retval None
 */
void WiFiApTask(void *argument)
{
  while (!wifi.esp_ready)
    osDelay(100);

  uint8_t res = lwespOK;

  for (;;)
  {
    if (WiFiGetError() != WIFI_OK)
    {
      WiFiErrorHandler(WiFiGetError());
      continue;
    }

    wifi.netconnection_server = NULL;
    wifi.netconnection_client = NULL;

    res = prvWiFiResetWithDelay();

    if (res != lwespOK)
      continue;

    wifi.ap_ready = false;
    lwesp_sta_t stations[1];
    size_t stations_quantity;

    res = prvWiFiSetMode(LWESP_MODE_AP);

    if (res != lwespOK)
      continue;

    lwesp_ip_t ip, gw, nm;
    const char *str = NULL;

    str = config.mqtt.local;
    res = prvWiFiParseIp(&str, &ip);

    if (res != lwespOK)
      continue;

    str = config.mqtt.local;
    res = prvWiFiParseIp(&str, &gw);

    if (res != lwespOK)
      continue;

    str = "255.255.255.0";
    res = prvWiFiParseIp(&str, &nm);

    if (res != lwespOK)
      continue;

    PrintfLogsCRLF(CLR_GR"WiFi mode is now "CLR_YL"AP"CLR_DEF);

    res = prvWiFiSetIp(&ip, &gw, &nm);

    if (res != lwespOK)
      continue;

    res = prvWiFiApConfigure("ESS_BOARD", "ess_local", WIFI_RF_CHANNEL, LWESP_ECN_WPA2_PSK,
        LWESP_ARRAYSIZE(stations), WIFI_NOT_HIDE, WIFI_DEFAULT, NULL, NULL, WIFI_BLOCKING);

    if (res != lwespOK)
      continue;

    res = prvWiFiApListSta(stations, &stations_quantity, WIFI_BLOCKING);

    if (res != lwespOK)
      continue;

    prvWiFiStationList(stations, stations_quantity);
    res = prvWiFiConnectionNew(&wifi);

    if (res != lwespOK)
      continue;

    res = prvWiFiBindConnection(wifi.netconnection_server, config.mqtt.port);

    if (res != lwespOK)
      continue;

    res = prvWiFiListenConnection(wifi.netconnection_server);

    if (res != lwespOK)
      continue;

    for (;;)
    {
      res = prvWiFiAcceptConnection(wifi.netconnection_server, &wifi.netconnection_client);

      if (res != lwespOK)
      {
        if (wifi.restart)
        {
          wifi.restart = false;
          break;
        }
        continue;
      }

      wifi.host_connected = true;
      lwesp_pbuf_p packet_buffer = NULL;

      prvWiFiSetReceiveTimeout(wifi.netconnection_client, WIFI_RECEIVE_TIMEOUT);

      for (;;)
      {
        res = prvWiFiReceiveConnection(wifi.netconnection_client, &packet_buffer);

        if (res == lwespTIMEOUT)
        {
          if (!wifi.host_connected)
            break;
          if (wifi.restart)
            break;
        }
        else if (res != lwespOK)
          break;

        PrintfLogsCRLF(CLR_GR"NETCONN data received, %u/%u bytes"CLR_DEF, (int) lwesp_pbuf_length(packet_buffer, 1), (int) lwesp_pbuf_length(packet_buffer, 0));

        if (wifi.packet_buffer == NULL)
          wifi.packet_buffer = packet_buffer;
        else
          prvWiFiConcatenatePacketBuffers(wifi.packet_buffer, packet_buffer);

        //TODO WRITE BROKET PARSER

        prvWiFiFreePacketBuffer(wifi.packet_buffer);
        wifi.packet_buffer = NULL;
      }

      if (wifi.netconnection_client)
      {
        prvWiFiNetConnectionClose(wifi.netconnection_client);
        prvWiFiNetConnectionDelete(wifi.netconnection_client);
        wifi.netconnection_client = NULL;
      }
      if (wifi.packet_buffer != NULL)
      {
        prvWiFiFreePacketBuffer(wifi.packet_buffer);
        wifi.packet_buffer = NULL;
      }
    }
  }
}
/******************************************************************************/




/**
 * @brief  Function implementing the wifiStTask thread.
 *         This task drives the esp module in STA mode.
 * @param  argument: Not used
 * @return None
 */
void WiFiStTask(void *argument)
{
  while (!wifi.esp_ready)
    osDelay(100);

  uint8_t errors_scan_ap   = 0;
  uint8_t errors_join_st   = 0;
  uint8_t errors_net_check = 0;
  uint8_t res = lwespOK;

  for (;;)
  {
    if (WiFiGetError() != WIFI_OK)
    {
      WiFiErrorHandler(WiFiGetError());
      continue;
    }

    wifi.connection = NULL;

    //WiFi reset with delay
    res = prvWiFiResetWithDelay();

    if (res != lwespOK)
      continue;

    lwesp_ap_t access_point[10];
    size_t access_point_find;

    //WiFi set mode ST
    res = prvWiFiSetMode(LWESP_MODE_STA);

    if (res != lwespOK)
      continue;

    //WiFi start searching for access point
    bool config_ap_found = false;

    while (!config_ap_found)
    {
      PrintfLogsCRLF("WiFi Access points scanning ...");
      IndicationLedYellowBlink(5);

      res = prvWiFiListAp(access_point, &access_point_find);

      if (res != lwespOK)
        continue;

      //WiFi check founded access points
      res = prvWiFiAccessPointsFound(access_point_find, access_point, &config_ap_found);

      if (!config_ap_found)
      {
        osDelay(5000);
        if (++errors_scan_ap > WIFI_MAX_SCAN_ERRORS)
        {
          osDelay(60000);
          errors_scan_ap = 0;
          continue;
        }
      }

      errors_scan_ap = 0;
      IndicationLedYellowBlink(2);
      PrintfLogsCRLF("WiFi connecting to \"%s\" network ...", config.wifi.ssid);

      //WiFi join as station to access point
      res = prvWiFiStaJoin();

      if (res != lwespOK)
      {
        config_ap_found = false;
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

      //WiFi copy IP
      lwesp_ip_t ip;
      res = prvWiFiCopyIp(&ip);

      errors_join_st = 0;

      if (res != lwespOK)
        continue;
    }

    PrintfLogsCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);

    for (;;)
    {
      if (!prvWiFiStaIsJoined())
        break;

      if (wifi.restart)
      {
        wifi.restart = false;
        break;
      }

      if (!wifi.sta_ready)
      {
        res = prvWiFiPing();

        if (res != lwespOK)
        {
          if (++errors_net_check > WIFI_MAX_NET_CHECK_ERRORS)
          {
            errors_net_check = 0;
            continue;
          }
          else
          {
            osDelay(1000);
            continue;
          }
        }

        errors_net_check = 0;
        IndicationLedYellowBlink(3);
        wifi.sta_ready = true;

        PrintfLogsCRLF(CLR_GR"Internet connection \"%s\" OK"CLR_DEF, config.wifi.ssid);

//        if (!mqtt_wifi_transport && !esp8266_onair)
//        {
//          PrintfLogsCRLF(CLR_GR"Switching MQTT transport to WiFi"CLR_DEF);
//          MQTTClient_Stop();
//        }
      }

      osDelay(100);
    }
  }

  osThreadTerminate(NULL);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set current error
 * @param[in]      error: error which need to be set
 */
uint8_t WiFiSetError(WIFI_ERROR_t error)
{
  if (error > WIFI_NUM_OF_ERRORS)
    return WIFI_SET_ERROR;

  wifi.error = error;

  return WIFI_OK;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi get current error
 * @param[in]      error: error which need to be set
 */
uint8_t WiFiGetError(void)
{
  return wifi.error;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi get current error
 * @param[in]      error: error which need to be set
 */
void WiFiErrorHandler(WIFI_ERROR_t error)
{
  switch (error)
  {
    case WIFI_OK:
      break;
    case WIFI_INIT_ERROR:
    {
      PrintfLogsCRLF("\t"CLR_DEF"ERROR WIFI: "CLR_RD"INIT"CLR_DEF);
      break;
    }
    default:
    {
      PrintfLogsCRLF("\t"CLR_DEF"ERROR RTC: "CLR_RD"UNDEFINED"CLR_DEF);
      break;
    }
  }
}
/******************************************************************************/



/**
 * @brief          Wi-Fi stops task
 * @retval         NONE
 */
void WiFiStop(void)
{
  osThreadId_t st_task = WiFiStTaskHandle;
  osThreadId_t ap_task = WiFiApTaskHandle;

  if (st_task || ap_task)
  {
    osKernelLock();

    if (WiFiStTaskHandle != NULL)
    {
      WiFiStTaskHandle = NULL;
      osThreadTerminate(st_task);
    }
    if (WiFiApTaskHandle != NULL)
    {
      WiFiApTaskHandle = NULL;
      osThreadTerminate(ap_task);
    }

    osKernelUnlock();
  }

  if (wifi.packet_buffer)
  {
    prvWiFiFreePacketBuffer(wifi.packet_buffer);
    wifi.packet_buffer = NULL;
  }
  if (wifi.netconnection_client)
  {
    prvWiFiNetConnectionClose(wifi.netconnection_client);
    prvWiFiNetConnectionDelete(wifi.netconnection_client);
    wifi.netconnection_client = NULL;
  }
  if (wifi.netconnection_server)
  {
    prvWiFiNetConnectionClose(wifi.netconnection_server);
    prvWiFiNetConnectionDelete(wifi.netconnection_server);
    wifi.netconnection_server = NULL;
  }
  if (wifi.connection)
  {
    uint8_t res = WIFI_OK;
    res = prvWiFiCloseConnection(wifi.connection, WIFI_BLOCKING);

    if (res != lwespOK)
      WiFiSetError(WIFI_CLOSE_CONNECTION_ERROR);
  }

  wifi.restart = false;
  wifi.ap_ready = false;
  wifi.sta_ready = false;
  wifi.host_connected = false;

  IndicationLedGreenBlink(5);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi get MAC address
 * @return         NONE
 */
void WiFiGetMac(void)
{
  lwesp_mac_t mac_addr = {0};
  lwesp_sta_getmac(&mac_addr, NULL, NULL, WIFI_BLOCKING);
  PrintfLogsCRLF("\t"CLR_YL"ESP8266 MAC %02X:%02X:%02X:%02X:%02X:%02X"CLR_DEF, mac_addr.mac[0], mac_addr.mac[1],
                                                                                 mac_addr.mac[2], mac_addr.mac[3],
                                                                                 mac_addr.mac[4], mac_addr.mac[5]);
}
/******************************************************************************/




/**
 * @brief          Get current access point information
 * @return         Current lwespr_t struct state
 */
uint8_t WiFiGetInfoAp(void)
{
  uint8_t res = lwespOK;

  lwesp_sta_info_ap_t ap_info = {0};
  res = lwesp_sta_get_ap_info(&ap_info, NULL, NULL, WIFI_BLOCKING);

  if (res != lwespOK)
  {
    PrintfLogsCRLF("\t"CLR_YL"ESP8266 is not connected to AP"CLR_DEF);
    return res;
  }

  PrintfLogsCRLF("\t"CLR_YL"ESP8266 AP \"%s\" RSSI %d dB"CLR_DEF, ap_info.ssid, ap_info.rssi);

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ST_mode reset ESP8266 with delay
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiResetWithDelay(void)
{
  uint8_t res = lwespOK;
  res = lwesp_reset_with_delay(LWESP_CFG_RESET_DELAY_DEFAULT, NULL, NULL, WIFI_BLOCKING);
  PrintfLogsCRLF(CLR_DEF"WiFi Reset: (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set mode
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiSetMode(uint8_t mode)
{
  uint8_t res = lwespOK;
  res = lwesp_set_wifi_mode(mode, NULL, NULL, WIFI_BLOCKING);

  if (mode == LWESP_MODE_STA)
    PrintfLogsCRLF(CLR_DEF"WiFi set mode ST (%s)"CLR_DEF, prvlwespErrorHandler(res));
  else if (mode == LWESP_MODE_AP)
    PrintfLogsCRLF(CLR_DEF"WiFi set mode AP (%s)"CLR_DEF, prvlwespErrorHandler(res));
  else
    PrintfLogsCRLF(CLR_DEF"WiFi set mode ST and AP (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ST_mode list of access points
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiListAp(lwesp_ap_t *access_point, size_t *access_point_find)
{
  uint8_t res = lwespOK;
  res = lwesp_sta_list_ap(NULL, access_point, LWESP_ARRAYSIZE(access_point),
      access_point_find, NULL, NULL, 1);

  PrintfLogsCRLF(CLR_DEF"WiFi Access point scan: (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi access poits check
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiAccessPointsFound(size_t access_point_find, lwesp_ap_t *access_point, bool *config_ap_found)
{
  uint8_t res = lwespOK;

  for (uint8_t i = 0; i < access_point_find; i++)
  {
    PrintfLogsCRLF(CLR_GR"Wifi AP found: \"%s\", RSSI: %i dBm"CLR_DEF, access_point[i].ssid, access_point[i].rssi);

    if (strcmp(config.wifi.ssid, access_point[i].ssid) == 0)
      *config_ap_found = true;
  }

  PrintfLogsCRLF("WiFi Access point \"%s\" is (%s)"CLR_DEF, config.wifi.ssid, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi join as station to access point
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiStaJoin(void)
{
  uint8_t res = lwespOK;
  res = lwesp_sta_join(config.wifi.ssid, config.wifi.passw, NULL, NULL, NULL, WIFI_BLOCKING);
  osDelay(1000);

  PrintfLogsCRLF(CLR_DEF"WiFi connection to \"%s\" network (%s)"CLR_DEF, config.wifi.ssid, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi have joined sta
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiStaIsJoined(void)
{
  uint8_t res = lwespOK;
  res = lwesp_sta_is_joined();

  if (!res)
    osDelay(1000);

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi copy IP
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiCopyIp(lwesp_ip_t *ip)
{
  uint8_t res = lwespOK;
  res = lwesp_sta_copy_ip(ip, NULL, NULL, NULL);

  if (res != lwespOK)
  {
    PrintfLogsCRLF(CLR_DEF"Copy IP fault! (%s)"CLR_DEF, prvlwespErrorHandler(res));
    return res;
  }
  else
  {
    PrintfLogsCRLF(CLR_GR"WiFi connected to \"%s\" access point OK"CLR_DEF, config.wifi.ssid);
    PrintfLogsCRLF(CLR_GR"WiFi station IP address: %u.%u.%u.%u"CLR_DEF, (int) ip->addr.ip4.addr[0],
    (int) ip->addr.ip4.addr[1], (int) ip->addr.ip4.addr[2], (int) ip->addr.ip4.addr[3]);
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ping
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiPing(void)
{
  uint8_t res = lwespOK;
  res = lwesp_ping("8.8.8.8", NULL, NULL, NULL, 1);

  if (res != lwespOK)
  {
    PrintfLogsCRLF(CLR_RD"ERROR: \"%s\" access point doesn't have internet connection!"CLR_DEF, config.wifi.ssid);
    PrintfLogsCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi parse IP
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiParseIp(const char **str, lwesp_ip_t *ip)
{
  uint8_t res = lwespOK;
  uint8_t parse_result = lwespi_parse_ip(str, ip);

  if (parse_result != 1)
  {
    //res = lwespERRPARSEIP;
    PrintfLogsCRLF(CLR_DEF"Parse IP (%s)"CLR_DEF, prvlwespErrorHandler(res));
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set IP
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiSetIp(lwesp_ip_t *ip, lwesp_ip_t *gw, lwesp_ip_t *nm)
{
  uint8_t res = lwespOK;

  res = lwesp_ap_setip(ip, gw, nm, NULL, NULL, WIFI_BLOCKING);
  PrintfLogsCRLF(CLR_DEF"WiFi set IP AP (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi configure AP
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiApConfigure(const char *ssid, const char *password, uint8_t channel, lwesp_ecn_t encryption, uint8_t max_stations,
    uint8_t hide, uint8_t def, const lwesp_api_cmd_evt_fn evt_fn, void *const evt_argument, const uint32_t blocking)
{
  uint8_t res = lwespOK;

  res = lwesp_ap_set_config(ssid, password, channel, encryption, max_stations, hide, evt_fn, evt_argument, blocking);
  PrintfLogsCRLF(CLR_DEF"WiFi configure AP (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi list of stations connected to access point
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiApListSta(lwesp_sta_t *stations, size_t *stations_quantity, const uint32_t blocking)
{
  uint8_t res = lwespOK;

  res = lwesp_ap_list_sta(stations, LWESP_ARRAYSIZE(stations), stations_quantity, NULL, NULL, blocking);
  PrintfLogsCRLF(CLR_DEF"WiFi station scan (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi list of found stations
 * @return         None
 */
void prvWiFiStationList(lwesp_sta_t *stations, size_t stations_quantity)
{
  for (uint8_t i = 0; i < stations_quantity; i++)
    PrintfLogsCRLF(CLR_GR"Wifi Station found: %u.%u.%u.%u"CLR_DEF, stations[i].ip.addr.ip4.addr[0],
        stations[i].ip.addr.ip4.addr[1], stations[i].ip.addr.ip4.addr[2], stations[i].ip.addr.ip4.addr[3]);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi create new netconn connection
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiConnectionNew(WIFI_DATA_t *wifi)
{
  wifi->ap_ready = true;
  wifi->netconnection_server = lwesp_netconn_new(LWESP_NETCONN_TYPE_TCP);

  if (wifi->netconnection_server == NULL)
  {
    PrintfLogsCRLF(CLR_RD"Cannot create netconn_server NETCONN"CLR_DEF);
    if (wifi->netconnection_server)
    {
      prvWiFiNetConnectionClose(wifi->netconnection_server);
      prvWiFiNetConnectionDelete(wifi->netconnection_server);
    }
    return lwespERRCONNFAIL;
  }

  return lwespOK;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi bind a connection to a specific port
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiBindConnection(lwesp_netconn_p netconnection_server, uint16_t port)
{
  uint8_t res = lwespOK;

  res = lwesp_netconn_bind(netconnection_server, port);

  PrintfLogsCRLF(CLR_DEF"Netconn on port %u (%s)"CLR_DEF, config.mqtt.port, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi listen on previously binded connection
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiListenConnection(lwesp_netconn_p netconnection_server)
{
  uint8_t res = lwespOK;

  res = lwesp_netconn_listen(netconnection_server);

  PrintfLogsCRLF(CLR_DEF"Listening to net connection (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi accept a new connection
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiAcceptConnection(lwesp_netconn_p netconnection_server, lwesp_netconn_p *netconnection_client)
{
  uint8_t res = lwespOK;

  res = lwesp_netconn_accept(netconnection_server, netconnection_client);

  PrintfLogsCRLF(CLR_DEF"Accept to new connection (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set timeout for receiving data
 * @return         NONE
 */
void prvWiFiSetReceiveTimeout(lwesp_netconn_p netconnection_client, uint32_t timeout)
{
  lwesp_netconn_set_receive_timeout(netconnection_client, timeout);
  PrintfLogsCRLF(CLR_DEF"Receive timeout is set to"CLR_GR "(%u)" "seconds"CLR_DEF, timeout);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi free packet buffer
 * @return         NONE
 */
void prvWiFiFreePacketBuffer(lwesp_pbuf_p packet_buffer)
{
  lwesp_pbuf_free(packet_buffer);
  PrintfLogsCRLF(CLR_DEF"Free packet buffer");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi receive data from connection
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiReceiveConnection(lwesp_netconn_p netconnection_client, lwesp_pbuf_p* pbuf)
{
  uint8_t res = lwespOK;

  res = lwesp_netconn_receive(netconnection_client, pbuf);

  PrintfLogsCRLF(CLR_DEF"NETCONN data receiving (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi connection close
 * @return         Current lwespr_t struct state
 */
uint8_t prvWiFiCloseConnection(lwesp_conn_p connection, const uint32_t blocking)
{
  uint8_t res = lwespOK;

  res = lwesp_conn_close(connection, blocking);

  PrintfLogsCRLF(CLR_DEF"Connection close (%s)"CLR_DEF, prvlwespErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi concatenate 2 packet buffers together to one big packet
 * @return         NONE
 */
void prvWiFiConcatenatePacketBuffers(lwesp_pbuf_p head, const lwesp_pbuf_p tail)
{
  lwesp_pbuf_cat(head, tail);
  PrintfLogsCRLF(CLR_DEF"Concatenated 2 buffers into one");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi close netconn connection
 * @return         NONE
 */
void prvWiFiNetConnectionClose(lwesp_netconn_p netconnection_client)
{
  lwesp_netconn_close(netconnection_client);
  PrintfLogsCRLF(CLR_DEF"Closed netconnection");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi delete netconn connection
 * @return         NONE
 */
void prvWiFiNetConnectionDelete(lwesp_netconn_p netconnection_client)
{
  lwesp_netconn_delete(netconnection_client);
  PrintfLogsCRLF(CLR_DEF"Deleted netconnection");
}
/******************************************************************************/




lwespr_t esp_callback_function(lwesp_evt_t* event)
{
  switch (lwesp_evt_get_type(event))
    {
      case LWESP_EVT_AT_VERSION_NOT_SUPPORTED:
      {
        PrintfLogsCRLF(CLR_RD"This version API ESP8266 is not supported!"CLR_DEF);
        break;
      }
      case LWESP_EVT_INIT_FINISH:
      {
        wifi.esp_ready = true;
        PrintfLogsCRLF(CLR_GR"WiFi initialized OK"CLR_DEF);
        break;
      }
      case LWESP_EVT_RESET_DETECTED:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF("WiFi to reset ...");
        break;
      }
      case LWESP_EVT_RESET:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF(CLR_GR"WiFi reset OK"CLR_DEF);
        break;
      }
      case LWESP_EVT_RESTORE:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF(CLR_GR"WiFi restore OK"CLR_DEF);
        break;
      }
      case LWESP_EVT_CMD_TIMEOUT:
      {
        PrintfLogsCRLF(CLR_RD"WiFi command timeout"CLR_DEF);
        break;
      }
      case LWESP_EVT_WIFI_CONNECTED:
      {
        PrintfLogsCRLF(CLR_GR"WiFi AP connected OK"CLR_DEF);
        //wifi.sta_ready = true;
        break;
      }
      case LWESP_EVT_WIFI_GOT_IP:
      {
        PrintfLogsCRLF(CLR_GR"WiFi AP got IP"CLR_DEF);
        break;
      }
//      case ESP_EVT_WIFI_DISCONNECTED:
//      {
//        PrintfLogsCRLF(CLR_RD"WiFi AP disconnected!"CLR_DEF);
//        wifi.host_connected = false;
//        if (mqtt_wifi_transport)
//          MQTTClient_Stop();
//        break;
//      }
      case LWESP_EVT_WIFI_IP_ACQUIRED:
      {
        PrintfLogsCRLF(CLR_GR"WiFi AP IP acquired"CLR_DEF);
        break;
      }
      case LWESP_EVT_STA_LIST_AP:
      {
        PrintfLogsCRLF(CLR_GR"WiFi APs listed"CLR_DEF);
        break;
      }
      case LWESP_EVT_STA_JOIN_AP:
      {
        lwespr_t status = lwesp_evt_sta_join_ap_get_result(event);
        if (status == lwespOK)
        {
          lwesp_ip_t ip;
          lwesp_sta_copy_ip(&ip, NULL, NULL, NULL);
          PrintfLogsCRLF(CLR_GR"WiFi join to AP (%u.%u.%u.%u)"CLR_DEF, ip.addr.ip4.addr[0], ip.addr.ip4.addr[1], ip.addr.ip4.addr[2], ip.addr.ip4.addr[3]);
        }
        else
        {
          wifi.host_connected = false;
          PrintfLogsCRLF(CLR_RD"WiFi AP join ERROR! (%u)"CLR_DEF, status);
        }
        break;
      }
//      case LWESP_EVT_PING:
//      {
//        break;
//      }
      case LWESP_EVT_STA_INFO_AP:
      {
        break;
      }
      case LWESP_EVT_AP_CONNECTED_STA:
      {
        lwesp_mac_t *mac;
        mac = lwesp_evt_ap_connected_sta_get_mac(event);
        PrintfLogsCRLF(CLR_GR"WiFi station connected MAC %X:%X:%X:%X:%X:%X"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
        break;
      }
      case LWESP_EVT_AP_DISCONNECTED_STA:
      {
        lwesp_mac_t *mac;
        mac = lwesp_evt_ap_disconnected_sta_get_mac(event);
        PrintfLogsCRLF(CLR_RD"WiFi station disconnected! (MAC %X:%X:%X:%X:%X:%X)"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
        wifi.host_connected = false;
        wifi.restart = true;
        break;
      }
//      case ESP_EVT_AP_IP_STA:
//      {
//        lwesp_ip_t *ip;
//        ip = esp_evt_ap_ip_sta_get_ip(event);
//        memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
//        Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
//        PrintfLogsCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
//        break;
//      }
      case LWESP_EVT_SERVER:
      {
        lwespr_t res = lwesp_evt_server_get_result(event);
        lwesp_port_t port = lwesp_evt_server_get_port(event);
        uint8_t ena = lwesp_evt_server_is_enable(event);
        PrintfLogsCRLF(CLR_GR"NETCONN server: res=%u, port=%u, ena=%u"CLR_DEF, res, port, ena);
  //      lwesp_ip_t *ip;
  //      ip = esp_evt_ap_ip_sta_get_ip(evt);
  //      memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
  //      Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
  //      PrintfLogsCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
  //      wifi.sta_ready = true;
        break;
      }
      default:
      {
        PrintfLogsCRLF("WiFi ESP callback.%u? ", lwesp_evt_get_type(event));
        break;
      }
    }
    return lwespOK;
}
/******************************************************************************/




/**
 * @brief  ESP8266 wifi errors.
 * @param  err: Result enumeration used across application functions
 * @retval char: String that describes error code passed to the function
 */
char *prvlwespErrorHandler(lwespr_t error)
{
  switch (error)
  {
    case lwespOK:                   return (CLR_GR"OK");                                                        break;
    case lwespOKIGNOREMORE:         return (CLR_RD"Ignore sending more data");                                  break;
    case lwespERR:                  return (CLR_RD"AT error");                                                  break;
//    case lwespPARERR:               return (CLR_RD"Wrong parameters");                                          break;
    /* Reboot board if memory leak detected */
    case lwespERRMEM:               NVIC_SystemReset(); return ("Memory error");                          break;
    case lwespTIMEOUT:              return (CLR_RD"Timeout");                                                   break;
    case lwespCONT:                 return (CLR_RD"Still some command to be processed in current command");     break;
    case lwespCLOSED:               return (CLR_RD"Connection just closed");                                    break;
    case lwespINPROG:               return (CLR_RD"Operation is in progress");                                  break;
    case lwespERRNOIP:              return (CLR_RD"Station does not have IP address");                          break;
    /* This is impossible state, when the device is connected to MQTT broker and start the second connection */
    case lwespERRNOFREECONN:        NVIC_SystemReset(); return (CLR_RD"There is no free connection available to start");
    case lwespERRCONNTIMEOUT:       return (CLR_RD"Timeout received when connection to access point");          break;
    case lwespERRPASS:              return (CLR_RD"Invalid password for access point");                         break;
    case lwespERRNOAP:              return (CLR_RD"No access point found with specific SSID and MAC address");  break;
    case lwespERRCONNFAIL:          return (CLR_RD"Connection failed to access point");                         break;
    case lwespERRWIFINOTCONNECTED:  return (CLR_RD"Wifi not connected to access point");                        break;
    case lwespERRNODEVICE:          return (CLR_RD"Device is not present");                                     break;
    case lwespERRBLOCKING:          return (CLR_RD"Blocking mode command is not allowed");                      break;
    //case lwespERRPARSEIP:           return (CLR_RD"Parse IP error");                                            break;
    default:                      return (CLR_RD"???");
  };
}
/******************************************************************************/











