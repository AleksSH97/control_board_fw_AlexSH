/**
 ******************************************************************************
 * @file           : wi-fi.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : WI-FI driver
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


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
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
    Printf_LogsCRLF(CLR_RD"ESP init FAIL! (%s)"CLR_DEF, prvESPErrorHandler(output));

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











