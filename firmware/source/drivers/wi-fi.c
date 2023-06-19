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
  esp_conn_p connection;
  esp_netconn_p netconnection_server;
  esp_netconn_p netconnection_client;
  esp_pbuf_p    packet_buffer;

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
espr_t esp_callback_function(esp_evt_t* event);
char *prvESPErrorHandler(espr_t error);

uint8_t prvWiFiResetWithDelay(void);
uint8_t prvWiFiSetMode(uint8_t mode);
uint8_t prvWiFiListAp(esp_ap_t *access_point, size_t *access_point_find);
uint8_t prvWiFiAccessPointsFound(size_t access_point_find, esp_ap_t *access_point, bool *config_ap_found);
uint8_t prvWiFiStaJoin(void);
uint8_t prvWiFiCopyIp(esp_ip_t *ip);
uint8_t prvWiFiStaIsJoined(void);
uint8_t prvWiFiPing(void);
uint8_t prvWiFiParseIp(const char **str, esp_ip_t *ip);
uint8_t prvWiFiSetIp(esp_ip_t *ip, esp_ip_t *gw, esp_ip_t *nm);
uint8_t prvWiFiApConfigure(const char *ssid, const char *password, uint8_t channel, esp_ecn_t encryption, uint8_t max_stations, uint8_t hide, uint8_t def, const esp_api_cmd_evt_fn evt_fn, void *const evt_argument, const uint32_t blocking);
uint8_t prvWiFiApListSta(esp_sta_t *stations, size_t *stations_quantity, const uint32_t blocking);
uint8_t prvWiFiConnectionNew(WIFI_DATA_t *wifi);
uint8_t prvWiFiBindConnection(esp_netconn_p netconnection_server, uint16_t port);
uint8_t prvWiFiListenConnection(esp_netconn_p netconnection_server);
uint8_t prvWiFiAcceptConnection(esp_netconn_p netconnection_server, esp_netconn_p *netconnection_client);
uint8_t prvWiFiReceiveConnection(esp_netconn_p netconnection_client, esp_pbuf_p* pbuf);
uint8_t prvWiFiCloseConnection(esp_conn_p connection, const uint32_t blocking);

void prvWiFiStationList(esp_sta_t *stations, size_t stations_quantity);
void prvWiFiSetReceiveTimeout(esp_netconn_p netconnection_client, uint32_t timeout);
void prvWiFiConcatenatePacketBuffers(esp_pbuf_p head, const esp_pbuf_p tail);
void prvWiFiFreePacketBuffer(esp_pbuf_p packet_buffer);
void prvWiFiNetConnectionClose(esp_netconn_p netconnection_client);
void prvWiFiNetConnectionDelete(esp_netconn_p netconnection_client);


/******************************************************************************/


/**
 * @brief  Wi-Fi module initialization.
 * @retval Current error instance
 */
void WiFiInit(void)
{
  PrintfConsoleCRLF(CLR_RD"WI-FI INIT"CLR_DEF);

  uint8_t res = WIFI_OK;

  memset(&wifi, 0, sizeof(wifi));

  espr_t output = esp_init(esp_callback_function, 0);
  if (output != espOK)
    PrintfConsoleCRLF(CLR_RD"ESP init FAIL! (%s)"CLR_DEF, prvESPErrorHandler(output));

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
  PrintfConsoleCRLF(CLR_DEF"WI-FI START"CLR_DEF);

  wifi.ap_mode = mode_ap;

  if (mode_ap == WIFI_MODE_AP)
  {
    WiFiApTaskHandle = osThreadNew(WiFiApTask, NULL, &WifiApTask_attributes);
    if (WiFiApTaskHandle == NULL)
      return WIFI_START_ERROR;
  }
  else if (mode_ap == WIFI_MODE_ST)
  {
    PrintfConsoleCRLF(CLR_DEF"WI-FI MODE ST"CLR_DEF);
    WiFiStTaskHandle = osThreadNew(WiFiStTask, NULL, &WifiStTask_attributes);
    if (WiFiStTaskHandle == NULL)
      return WIFI_START_ERROR;
  }

  PrintfConsoleCRLF("Switch WiFi to %s mode ..."CLR_DEF,
      wifi.ap_mode ? CLR_YL"AP"CLR_GR : CLR_YL"ST"CLR_GR);

  if (wifi.ap_mode)
    IndicationLedError();
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

  uint8_t res = espOK;

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

    if (res != espOK)
      continue;

    wifi.ap_ready = false;
    esp_sta_t stations[1];
    size_t stations_quantity;

    res = prvWiFiSetMode(ESP_MODE_AP);

    if (res != espOK)
      continue;

    esp_ip_t ip, gw, nm;
    const char *str = NULL;

    str = config.mqtt.local;
    res = prvWiFiParseIp(&str, &ip);

    if (res != espOK)
      continue;

    str = config.mqtt.local;
    res = prvWiFiParseIp(&str, &gw);

    if (res != espOK)
      continue;

    str = "255.255.255.0";
    res = prvWiFiParseIp(&str, &nm);

    if (res != espOK)
      continue;

    PrintfConsoleCRLF(CLR_GR"WiFi mode is now "CLR_YL"AP"CLR_DEF);

    res = prvWiFiSetIp(&ip, &gw, &nm);

    if (res != espOK)
      continue;

    res = prvWiFiApConfigure("ESS_BOARD", "ess_local", WIFI_RF_CHANNEL, ESP_ECN_WPA2_PSK,
        ESP_ARRAYSIZE(stations), WIFI_NOT_HIDE, WIFI_DEFAULT, NULL, NULL, WIFI_BLOCKING);

    if (res != espOK)
      continue;

    res = prvWiFiApListSta(stations, &stations_quantity, WIFI_BLOCKING);

    if (res != espOK)
      continue;

    prvWiFiStationList(stations, stations_quantity);
    res = prvWiFiConnectionNew(&wifi);

    if (res != espOK)
      continue;

    res = prvWiFiBindConnection(wifi.netconnection_server, config.mqtt.port);

    if (res != espOK)
      continue;

    res = prvWiFiListenConnection(wifi.netconnection_server);

    if (res != espOK)
      continue;

    for (;;)
    {
      res = prvWiFiAcceptConnection(wifi.netconnection_server, &wifi.netconnection_client);

      if (res != espOK)
      {
        if (wifi.restart)
        {
          wifi.restart = false;
          break;
        }
        continue;
      }

      wifi.host_connected = true;
      esp_pbuf_p packet_buffer = NULL;

      prvWiFiSetReceiveTimeout(wifi.netconnection_client, WIFI_RECEIVE_TIMEOUT);

      for (;;)
      {
        res = prvWiFiReceiveConnection(wifi.netconnection_client, &packet_buffer);

        if (res == espTIMEOUT)
        {
          if (!wifi.host_connected)
            break;
          if (wifi.restart)
            break;
        }
        else if (res != espOK)
          break;

        PrintfConsoleCRLF(CLR_GR"NETCONN data received, %u/%u bytes"CLR_DEF, (int) esp_pbuf_length(packet_buffer, 1), (int) esp_pbuf_length(packet_buffer, 0));

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
  uint8_t res = espOK;

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

    if (res != espOK)
      continue;

    esp_ap_t access_point[10];
    size_t access_point_find;

    //WiFi set mode ST
    res = prvWiFiSetMode(ESP_MODE_STA);

    if (res != espOK)
      continue;

    //WiFi start searching for access point
    bool config_ap_found = false;

    while (!config_ap_found)
    {
      PrintfConsoleCRLF("WiFi Access points scanning ...");
      IndicationLedYellowBlink(5);

      res = prvWiFiListAp(access_point, &access_point_find);

      if (res != espOK)
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
      PrintfConsoleCRLF("WiFi connecting to \"%s\" network ...", config.wifi.ssid);

      //WiFi join as station to access point
      res = prvWiFiStaJoin();

      if (res != espOK)
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
      esp_ip_t ip;
      res = prvWiFiCopyIp(&ip);

      errors_join_st = 0;

      if (res != espOK)
        continue;
    }

    PrintfConsoleCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);

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

        if (res != espOK)
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

        PrintfConsoleCRLF(CLR_GR"Internet connection \"%s\" OK"CLR_DEF, config.wifi.ssid);

//        if (!mqtt_wifi_transport && !esp8266_onair)
//        {
//          PrintfConsoleCRLF(CLR_GR"Switching MQTT transport to WiFi"CLR_DEF);
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
      PrintfConsoleCRLF("\t"CLR_DEF"ERROR WIFI: "CLR_RD"INIT"CLR_DEF);
      break;
    }
    default:
    {
      PrintfConsoleCRLF("\t"CLR_DEF"ERROR RTC: "CLR_RD"UNDEFINED"CLR_DEF);
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

    if (res != espOK)
      WiFiSetError(WIFI_CLOSE_CONNECTION_ERROR);
  }

  wifi.restart = false;
  wifi.ap_ready = false;
  wifi.sta_ready = false;
  wifi.host_connected = false;

  IndicationLedError();
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ST_mode reset ESP8266 with delay
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiResetWithDelay(void)
{
  uint8_t res = espOK;
  res = esp_reset_with_delay(ESP_CFG_RESET_DELAY_DEFAULT, NULL, NULL, 1);
  PrintfConsoleCRLF(CLR_DEF"WiFi Reset: (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set mode
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiSetMode(uint8_t mode)
{
  uint8_t res = espOK;
  res = esp_set_wifi_mode(mode, 0, NULL, NULL, 1);

  if (mode == ESP_MODE_STA)
    PrintfConsoleCRLF(CLR_DEF"WiFi set mode ST (%s)"CLR_DEF, prvESPErrorHandler(res));
  else if (mode == ESP_MODE_AP)
    PrintfConsoleCRLF(CLR_DEF"WiFi set mode AP (%s)"CLR_DEF, prvESPErrorHandler(res));
  else
    PrintfConsoleCRLF(CLR_DEF"WiFi set mode ST and AP (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ST_mode list of access points
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiListAp(esp_ap_t *access_point, size_t *access_point_find)
{
  uint8_t res = espOK;
  res = esp_sta_list_ap(NULL, access_point, ESP_ARRAYSIZE(access_point),
      access_point_find, NULL, NULL, 1);

  PrintfConsoleCRLF(CLR_DEF"WiFi Access point scan: (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi access poits check
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiAccessPointsFound(size_t access_point_find, esp_ap_t *access_point, bool *config_ap_found)
{
  uint8_t res = espOK;

  for (uint8_t i = 0; i < access_point_find; i++)
  {
    PrintfConsoleCRLF(CLR_GR"Wifi AP found: \"%s\", RSSI: %i dBm"CLR_DEF, access_point[i].ssid, access_point[i].rssi);

    if (strcmp(config.wifi.ssid, access_point[i].ssid) == 0)
      *config_ap_found = true;
  }

  PrintfConsoleCRLF("WiFi Access point \"%s\" is (%s)"CLR_DEF, config.wifi.ssid, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi join as station to access point
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiStaJoin(void)
{
  uint8_t res = espOK;
  res = esp_sta_join(config.wifi.ssid, config.wifi.passw, NULL, 0, NULL, NULL, 1);
  osDelay(1000);

  PrintfConsoleCRLF(CLR_DEF"WiFi connection to \"%s\" network (%s)"CLR_DEF, config.wifi.ssid, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi have joined sta
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiStaIsJoined(void)
{
  uint8_t res = espOK;
  res = esp_sta_is_joined();

  if (!res)
    osDelay(1000);

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi copy IP
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiCopyIp(esp_ip_t *ip)
{
  uint8_t res = espOK;
  res = esp_sta_copy_ip(ip, NULL, NULL);

  if (res != espOK)
  {
    PrintfConsoleCRLF(CLR_DEF"Copy IP fault! (%s)"CLR_DEF, prvESPErrorHandler(res));
    return res;
  }
  else
  {
    PrintfConsoleCRLF(CLR_GR"WiFi connected to \"%s\" access point OK"CLR_DEF, config.wifi.ssid);
    PrintfConsoleCRLF(CLR_GR"WiFi station IP address: %u.%u.%u.%u"CLR_DEF, (int) ip->ip[0],
    (int) ip->ip[1], (int) ip->ip[2], (int) ip->ip[3]);
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi ping
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiPing(void)
{
  uint8_t res = espOK;
  res = esp_ping("8.8.8.8", NULL, NULL, NULL, 1);

  if (res != espOK)
  {
    PrintfConsoleCRLF(CLR_RD"ERROR: \"%s\" access point doesn't have internet connection!"CLR_DEF, config.wifi.ssid);
    PrintfConsoleCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi parse IP
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiParseIp(const char **str, esp_ip_t *ip)
{
  uint8_t res = espOK;
  uint8_t parse_result = espi_parse_ip(str, ip);

  if (parse_result != 1)
  {
    res = espERRPARSEIP;
    PrintfConsoleCRLF(CLR_DEF"Parse IP (%s)"CLR_DEF, prvESPErrorHandler(res));
  }

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set IP
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiSetIp(esp_ip_t *ip, esp_ip_t *gw, esp_ip_t *nm)
{
  uint8_t res = espOK;

  res = esp_ap_setip(ip, gw, nm, 0, NULL, NULL, 1);
  PrintfConsoleCRLF(CLR_DEF"WiFi set IP AP (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi configure AP
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiApConfigure(const char *ssid, const char *password, uint8_t channel, esp_ecn_t encryption, uint8_t max_stations,
    uint8_t hide, uint8_t def, const esp_api_cmd_evt_fn evt_fn, void *const evt_argument, const uint32_t blocking)
{
  uint8_t res = espOK;

  res = esp_ap_configure(ssid, password, channel, encryption, max_stations, hide, def, evt_fn, evt_argument, blocking);
  PrintfConsoleCRLF(CLR_DEF"WiFi configure AP (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi list of stations connected to access point
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiApListSta(esp_sta_t *stations, size_t *stations_quantity, const uint32_t blocking)
{
  uint8_t res = espOK;

  res = esp_ap_list_sta(stations, ESP_ARRAYSIZE(stations), stations_quantity, NULL, NULL, blocking);
  PrintfConsoleCRLF(CLR_DEF"WiFi station scan (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi list of found stations
 * @return         None
 */
void prvWiFiStationList(esp_sta_t *stations, size_t stations_quantity)
{
  for (uint8_t i = 0; i < stations_quantity; i++)
    PrintfConsoleCRLF(CLR_GR"Wifi Station found: %u.%u.%u.%u"CLR_DEF, stations[i].ip.ip[0], stations[i].ip.ip[1], stations[i].ip.ip[2], stations[i].ip.ip[3]);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi create new netconn connection
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiConnectionNew(WIFI_DATA_t *wifi)
{
  wifi->ap_ready = true;
  wifi->netconnection_server = esp_netconn_new(ESP_NETCONN_TYPE_TCP);

  if (wifi->netconnection_server == NULL)
  {
    PrintfConsoleCRLF(CLR_RD"Cannot create netconn_server NETCONN"CLR_DEF);
    if (wifi->netconnection_server)
    {
      prvWiFiNetConnectionClose(wifi->netconnection_server);
      prvWiFiNetConnectionDelete(wifi->netconnection_server);
    }
    return espERRCONNFAIL;
  }

  return espOK;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi bind a connection to a specific port
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiBindConnection(esp_netconn_p netconnection_server, uint16_t port)
{
  uint8_t res = espOK;

  res = esp_netconn_bind(netconnection_server, port);

  PrintfConsoleCRLF(CLR_DEF"Netconn on port %u (%s)"CLR_DEF, config.mqtt.port, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi listen on previously binded connection
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiListenConnection(esp_netconn_p netconnection_server)
{
  uint8_t res = espOK;

  res = esp_netconn_listen(netconnection_server);

  PrintfConsoleCRLF(CLR_DEF"Listening to net connection (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi accept a new connection
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiAcceptConnection(esp_netconn_p netconnection_server, esp_netconn_p *netconnection_client)
{
  uint8_t res = espOK;

  res = esp_netconn_accept(netconnection_server, netconnection_client);

  PrintfConsoleCRLF(CLR_DEF"Accept to new connection (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi set timeout for receiving data
 * @return         NONE
 */
void prvWiFiSetReceiveTimeout(esp_netconn_p netconnection_client, uint32_t timeout)
{

  esp_netconn_set_receive_timeout(netconnection_client, timeout);
  PrintfConsoleCRLF(CLR_DEF"Receive timeout is set to"CLR_GR "(%u)" "seconds"CLR_DEF, timeout);
}
/******************************************************************************/




/**
 * @brief          Wi-Fi free packet buffer
 * @return         NONE
 */
void prvWiFiFreePacketBuffer(esp_pbuf_p packet_buffer)
{
  esp_pbuf_free(packet_buffer);
  PrintfConsoleCRLF(CLR_DEF"Free packet buffer");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi receive data from connection
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiReceiveConnection(esp_netconn_p netconnection_client, esp_pbuf_p* pbuf)
{
  uint8_t res = espOK;

  res = esp_netconn_receive(netconnection_client, pbuf);

  PrintfConsoleCRLF(CLR_DEF"NETCONN data receiving (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi connection close
 * @return         Current espr_t struct state
 */
uint8_t prvWiFiCloseConnection(esp_conn_p connection, const uint32_t blocking)
{
  uint8_t res = espOK;

  res = esp_conn_close(connection, blocking);

  PrintfConsoleCRLF(CLR_DEF"Connection close (%s)"CLR_DEF, prvESPErrorHandler(res));

  return res;
}
/******************************************************************************/




/**
 * @brief          Wi-Fi concatenate 2 packet buffers together to one big packet
 * @return         NONE
 */
void prvWiFiConcatenatePacketBuffers(esp_pbuf_p head, const esp_pbuf_p tail)
{
  esp_pbuf_cat(head, tail);
  PrintfConsoleCRLF(CLR_DEF"Concatenated 2 buffers into one");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi close netconn connection
 * @return         NONE
 */
void prvWiFiNetConnectionClose(esp_netconn_p netconnection_client)
{
  esp_netconn_close(netconnection_client);
  PrintfConsoleCRLF(CLR_DEF"Closed netconnection");
}
/******************************************************************************/




/**
 * @brief          Wi-Fi delete netconn connection
 * @return         NONE
 */
void prvWiFiNetConnectionDelete(esp_netconn_p netconnection_client)
{
  esp_netconn_delete(netconnection_client);
  PrintfConsoleCRLF(CLR_DEF"Deleted netconnection");
}
/******************************************************************************/




espr_t esp_callback_function(esp_evt_t* event)
{
  switch (esp_evt_get_type(event))
    {
      case ESP_EVT_AT_VERSION_NOT_SUPPORTED:
      {
        PrintfConsoleCRLF(CLR_RD"This version API ESP8266 is not supported!"CLR_DEF);
        break;
      }
      case ESP_EVT_INIT_FINISH:
      {
        wifi.esp_ready = true;
        PrintfConsoleCRLF(CLR_GR"WiFi initialized OK"CLR_DEF);
        break;
      }
      case ESP_EVT_RESET_DETECTED:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfConsoleCRLF("WiFi to reset ...");
        break;
      }
      case ESP_EVT_RESET:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfConsoleCRLF(CLR_GR"WiFi reset OK"CLR_DEF);
        break;
      }
      case ESP_EVT_RESTORE:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfConsoleCRLF(CLR_GR"WiFi restore OK"CLR_DEF);
        break;
      }
      case ESP_EVT_CMD_TIMEOUT:
      {
        PrintfConsoleCRLF(CLR_RD"WiFi command timeout"CLR_DEF);
        break;
      }
      case ESP_EVT_WIFI_CONNECTED:
      {
        PrintfConsoleCRLF(CLR_GR"WiFi AP connected OK"CLR_DEF);
        //wifi.sta_ready = true;
        break;
      }
      case ESP_EVT_WIFI_GOT_IP:
      {
        PrintfConsoleCRLF(CLR_GR"WiFi AP got IP"CLR_DEF);
        break;
      }
//      case ESP_EVT_WIFI_DISCONNECTED:
//      {
//        PrintfConsoleCRLF(CLR_RD"WiFi AP disconnected!"CLR_DEF);
//        wifi.host_connected = false;
//        if (mqtt_wifi_transport)
//          MQTTClient_Stop();
//        break;
//      }
      case ESP_EVT_WIFI_IP_ACQUIRED:
      {
        PrintfConsoleCRLF(CLR_GR"WiFi AP IP acquired"CLR_DEF);
        break;
      }
      case ESP_EVT_STA_LIST_AP:
      {
        PrintfConsoleCRLF(CLR_GR"WiFi APs listed"CLR_DEF);
        break;
      }
      case ESP_EVT_STA_JOIN_AP:
      {
        espr_t status = esp_evt_sta_join_ap_get_result(event);
        if (status == espOK)
        {
          esp_ip_t ip;
          esp_sta_copy_ip(&ip, NULL, NULL);
          PrintfConsoleCRLF(CLR_GR"WiFi join to AP (%u.%u.%u.%u)"CLR_DEF, ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
        }
        else
        {
          wifi.host_connected = false;
          PrintfConsoleCRLF(CLR_RD"WiFi AP join ERROR! (%u)"CLR_DEF, status);
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
        mac = esp_evt_ap_connected_sta_get_mac(event);
        PrintfConsoleCRLF(CLR_GR"WiFi station connected MAC %X:%X:%X:%X:%X:%X"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
        break;
      }
      case ESP_EVT_AP_DISCONNECTED_STA:
      {
        esp_mac_t *mac;
        mac = esp_evt_ap_disconnected_sta_get_mac(event);
        PrintfConsoleCRLF(CLR_RD"WiFi station disconnected! (MAC %X:%X:%X:%X:%X:%X)"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
        wifi.host_connected = false;
        wifi.restart = true;
        break;
      }
//      case ESP_EVT_AP_IP_STA:
//      {
//        esp_ip_t *ip;
//        ip = esp_evt_ap_ip_sta_get_ip(event);
//        memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
//        Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
//        PrintfConsoleCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
//        break;
//      }
      case ESP_EVT_SERVER:
      {
        espr_t res = esp_evt_server_get_result(event);
        esp_port_t port = esp_evt_server_get_port(event);
        uint8_t ena = esp_evt_server_is_enable(event);
        PrintfConsoleCRLF(CLR_GR"NETCONN server: res=%u, port=%u, ena=%u"CLR_DEF, res, port, ena);
  //      esp_ip_t *ip;
  //      ip = esp_evt_ap_ip_sta_get_ip(evt);
  //      memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
  //      Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
  //      PrintfConsoleCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
  //      wifi.sta_ready = true;
        break;
      }
      default:
      {
        PrintfConsoleCRLF("WiFi ESP callback.%u? ", esp_evt_get_type(event));
        break;
      }
    }
    return espOK;
}
/******************************************************************************/




/**
 * @brief  ESP8266 wifi errors.
 * @param  err: Result enumeration used across application functions
 * @retval char: String that describes error code passed to the function
 */
char *prvESPErrorHandler(espr_t error)
{
  switch (error)
  {
    case espOK:                   return (CLR_GR"OK");                                                        break;
    case espOKIGNOREMORE:         return (CLR_RD"Ignore sending more data");                                  break;
    case espERR:                  return (CLR_RD"AT error");                                                  break;
    case espPARERR:               return (CLR_RD"Wrong parameters");                                          break;
    /* Reboot board if memory leak detected */
    case espERRMEM:               NVIC_SystemReset(); return ("Memory error");                          break;
    case espTIMEOUT:              return (CLR_RD"Timeout");                                                   break;
    case espCONT:                 return (CLR_RD"Still some command to be processed in current command");     break;
    case espCLOSED:               return (CLR_RD"Connection just closed");                                    break;
    case espINPROG:               return (CLR_RD"Operation is in progress");                                  break;
    case espERRNOIP:              return (CLR_RD"Station does not have IP address");                          break;
    /* This is impossible state, when the device is connected to MQTT broker and start the second connection */
    case espERRNOFREECONN:        NVIC_SystemReset(); return (CLR_RD"There is no free connection available to start");
    case espERRCONNTIMEOUT:       return (CLR_RD"Timeout received when connection to access point");          break;
    case espERRPASS:              return (CLR_RD"Invalid password for access point");                         break;
    case espERRNOAP:              return (CLR_RD"No access point found with specific SSID and MAC address");  break;
    case espERRCONNFAIL:          return (CLR_RD"Connection failed to access point");                         break;
    case espERRWIFINOTCONNECTED:  return (CLR_RD"Wifi not connected to access point");                        break;
    case espERRNODEVICE:          return (CLR_RD"Device is not present");                                     break;
    case espERRBLOCKING:          return (CLR_RD"Blocking mode command is not allowed");                      break;
    case espERRPARSEIP:           return (CLR_RD"Parse IP error");                                            break;
    default:                      return (CLR_RD"???");
  };
}
/******************************************************************************/











