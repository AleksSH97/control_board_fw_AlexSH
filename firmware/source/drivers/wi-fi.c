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

#define WIFI_MODE_AP                 (true)
#define WIFI_MODE_ST                 (false)

#define WIFI_RF_CHANNEL              (9u)

#define WIFI_NOT_HIDE                (0u)
#define WIFI_HIDE                    (1u)

#define WIFI_NOT_DEFAULT             (0u)
#define WIFI_DEFAULT                 (1u)

#define WIFI_BLOCKING                (1u)

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
  esp_pbuf_p    pbuf;

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


/******************************************************************************/


/**
 * @brief  Wi-Fi module initialization.
 * @retval Current error instance
 */
uint8_t WiFiInit(void)
{
  uint8_t res = WIFI_OK;

  memset(&wifi, 0, sizeof(wifi));

  espr_t output = esp_init(esp_callback_function, 0);
  if (output != espOK)
  {
    PrintfLogsCRLF(CLR_RD"ESP init FAIL! (%s)"CLR_DEF, prvESPErrorHandler(output));
    return WIFI_INIT_ERROR;
  }

  WiFiStTaskHandle = NULL;
  WiFiApTaskHandle = NULL;

  res = WiFiStart(WIFI_MODE_ST);

  return res;
}
/******************************************************************************/




/**
 * @brief  Start WiFi module in the specified mode.
 * @param  mode_ap: 'true' value starts wifi module in AP mode
 * @retval uint8_t: Current error instance
 */
WIFI_ERROR_t WiFiStart(bool mode_ap)
{
  wifi.ap_mode = mode_ap;

  if (mode_ap == WIFI_MODE_AP)
  {
    WiFiApTaskHandle = osThreadNew(WiFiApTask, NULL, &WifiApTask_attributes);
    if (WiFiApTaskHandle == NULL)
      return WIFI_START_ERROR;
  }
  else if (mode_ap == WIFI_MODE_ST)
  {
    WiFiStTaskHandle = osThreadNew(WiFiStTask, NULL, &WifiStTask_attributes);
    if (WiFiStTaskHandle == NULL)
      return WIFI_START_ERROR;
  }

  PrintfLogsCRLF("Switch WiFi to %s mode ..."CLR_DEF,
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

    PrintfLogsCRLF(CLR_GR"WiFi mode is now "CLR_YL"AP"CLR_DEF);

    res = prvWiFiSetIp(&ip, &gw, &nm);

    if (res != espOK)
      continue;
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
      PrintfLogsCRLF("WiFi Access points scanning ...");
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
      PrintfLogsCRLF("WiFi connecting to \"%s\" network ...", config.wifi.ssid);

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
      PrintfLogsCRLF("\t"CLR_DEF"ERROR WIFI: "CLR_RD"INIT"CLR_DEF);
    default:
      PrintfLogsCRLF("\t"CLR_DEF"ERROR RTC: "CLR_RD"UNDEFINED"CLR_DEF);
  }
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
  PrintfLogsCRLF(CLR_DEF"WiFi Reset: (%s)"CLR_DEF, prvESPErrorHandler(res));

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
    PrintfLogsCRLF(CLR_DEF"WiFi set mode ST (%s)"CLR_DEF, prvESPErrorHandler(res));
  else if (mode == ESP_MODE_AP)
    PrintfLogsCRLF(CLR_DEF"WiFi set mode AP (%s)"CLR_DEF, prvESPErrorHandler(res));
  else
    PrintfLogsCRLF(CLR_DEF"WiFi set mode ST and AP (%s)"CLR_DEF, prvESPErrorHandler(res));

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

  PrintfLogsCRLF(CLR_DEF"WiFi Access point scan: (%s)"CLR_DEF, prvESPErrorHandler(res));

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
    PrintfLogsCRLF(CLR_GR"Wifi AP found: \"%s\", RSSI: %i dBm"CLR_DEF, access_point[i].ssid, access_point[i].rssi);

    if (strcmp(config.wifi.ssid, access_point[i].ssid) == 0)
      *config_ap_found = true;
  }

  PrintfLogsCRLF("WiFi Access point \"%s\" is (%s)"CLR_DEF, config.wifi.ssid, prvESPErrorHandler(res));

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

  PrintfLogsCRLF(CLR_DEF"WiFi connection to \"%s\" network (%s)"CLR_DEF, config.wifi.ssid, prvESPErrorHandler(res));

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
    PrintfLogsCRLF(CLR_DEF"Copy IP fault! (%s)"CLR_DEF, prvESPErrorHandler(res));
    return res;
  }
  else
  {
    PrintfLogsCRLF(CLR_GR"WiFi connected to \"%s\" access point OK"CLR_DEF, config.wifi.ssid);
    PrintfLogsCRLF(CLR_GR"WiFi station IP address: %u.%u.%u.%u"CLR_DEF, (int) ip->ip[0],
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
    PrintfLogsCRLF(CLR_RD"ERROR: \"%s\" access point doesn't have internet connection!"CLR_DEF, config.wifi.ssid);
    PrintfLogsCRLF("Checking \"%s\" for internet connection ...", config.wifi.ssid);
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
    PrintfLogsCRLF(CLR_DEF"Parse IP (%s)"CLR_DEF, prvESPErrorHandler(res));
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

  return res;
}
/******************************************************************************/




espr_t esp_callback_function(esp_evt_t* event)
{
  switch (esp_evt_get_type(event))
    {
      case ESP_EVT_AT_VERSION_NOT_SUPPORTED:
      {
        PrintfLogsCRLF(CLR_RD"This version API ESP8266 is not supported!"CLR_DEF);
        break;
      }
      case ESP_EVT_INIT_FINISH:
      {
        wifi.esp_ready = true;
        PrintfLogsCRLF(CLR_GR"WiFi initialized OK"CLR_DEF);
        break;
      }
      case ESP_EVT_RESET_DETECTED:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF("WiFi to reset ...");
        break;
      }
      case ESP_EVT_RESET:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF(CLR_GR"WiFi reset OK"CLR_DEF);
        break;
      }
      case ESP_EVT_RESTORE:
      {
        wifi.restart = false;
        wifi.ap_ready = false;
        wifi.sta_ready = false;
        wifi.host_connected = false;
        PrintfLogsCRLF(CLR_GR"WiFi restore OK"CLR_DEF);
        break;
      }
      case ESP_EVT_CMD_TIMEOUT:
      {
        PrintfLogsCRLF(CLR_RD"WiFi command timeout"CLR_DEF);
        break;
      }
      case ESP_EVT_WIFI_CONNECTED:
      {
        PrintfLogsCRLF(CLR_GR"WiFi AP connected OK"CLR_DEF);
        //wifi.sta_ready = true;
        break;
      }
      case ESP_EVT_WIFI_GOT_IP:
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
      case ESP_EVT_WIFI_IP_ACQUIRED:
      {
        PrintfLogsCRLF(CLR_GR"WiFi AP IP acquired"CLR_DEF);
        break;
      }
      case ESP_EVT_STA_LIST_AP:
      {
        PrintfLogsCRLF(CLR_GR"WiFi APs listed"CLR_DEF);
        break;
      }
      case ESP_EVT_STA_JOIN_AP:
      {
        espr_t status = esp_evt_sta_join_ap_get_result(event);
        if (status == espOK)
        {
          esp_ip_t ip;
          esp_sta_copy_ip(&ip, NULL, NULL);
          PrintfLogsCRLF(CLR_GR"WiFi join to AP (%u.%u.%u.%u)"CLR_DEF, ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
        }
        else
        {
          wifi.host_connected = false;
          PrintfLogsCRLF(CLR_RD"WiFi AP join ERROR! (%u)"CLR_DEF, status);
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
        PrintfLogsCRLF(CLR_GR"WiFi station connected MAC %X:%X:%X:%X:%X:%X"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
        break;
      }
      case ESP_EVT_AP_DISCONNECTED_STA:
      {
        esp_mac_t *mac;
        mac = esp_evt_ap_disconnected_sta_get_mac(event);
        PrintfLogsCRLF(CLR_RD"WiFi station disconnected! (MAC %X:%X:%X:%X:%X:%X)"CLR_DEF, mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
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
//        PrintfLogsCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
//        break;
//      }
      case ESP_EVT_SERVER:
      {
        espr_t res = esp_evt_server_get_result(event);
        esp_port_t port = esp_evt_server_get_port(event);
        uint8_t ena = esp_evt_server_is_enable(event);
        PrintfLogsCRLF(CLR_GR"NETCONN server: res=%u, port=%u, ena=%u"CLR_DEF, res, port, ena);
  //      esp_ip_t *ip;
  //      ip = esp_evt_ap_ip_sta_get_ip(evt);
  //      memset(mqtt_local_ip, 0, sizeof(mqtt_local_ip));
  //      Sprintf(mqtt_local_ip, "%d.%d.%d.%d", ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3]);
  //      PrintfLogsCRLF(CLR_GR"WiFi station got IP %s"CLR_DEF, mqtt_local_ip);
  //      wifi.sta_ready = true;
        break;
      }
      default:
      {
        PrintfLogsCRLF("WiFi ESP callback.%u? ", esp_evt_get_type(event));
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











