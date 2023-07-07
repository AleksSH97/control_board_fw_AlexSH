/**
 ******************************************************************************
 * @file           : console_wi-fi.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Console system for wi-fi commands
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "console_wi-fi.h"

#include "console.h"
#include "config.h"
#if    !WIFI_USE_LWESP
#include "esp/system/esp_ll.h"
#include "esp/esp_sta.h"
#include "esp/esp_private.h"
#endif

#if    WIFI_USE_LWESP
#include "lwesp/lwesp.h"
#include "system/lwesp_ll.h"
#include "lwesp/lwesp_private.h"
#include "lwesp/lwesp_mem.h"
#include "lwesp/lwesp_input.h"
#endif

/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define _CMD_HELP                   "help"
#define _CMD_BACK                   "back"

#define CONSOLE_MATCH               0u


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/


/******************************************************************************/




/**
 * @brief           WIFI menu for console
 * @param[in]       @ref microrl_ptr pointer
 * @param[in]       argc: shows how many arguments were entered on the command line
 * @param[in]       argv: vector of string, it's elements are symbols
 * @return          return 0 if everything is ok
 */
int ConsoleWiFi(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
  int i = 0;
  uint8_t res = 0x00;
  ConsoleSetHelp(Console_WIFiPrintMenu);

  while (i < argc)
  {
    if (strcmp(argv[i], "update") == CONSOLE_MATCH)
    {
        ConsoleClearScreen();
        PrintfConsoleCRLF("\tUPDATING WIFI");

        WiFiStop();
        res = WiFiStart(WIFI_MODE_ST);

        if (res != espOK)
          PrintfConsoleCRLF("ERROR: START WI-FI");

#if    WIFI_USE_LWESP
        lwesp_ll_deinit(NULL);
#endif

#if   !WIFI_USE_LWESP
        esp_ll_deinit(NULL);
#endif
        configure_uart(esp.ll.uart.baudrate);
        //TODO PREPARE FOR ESP UPDATE IO UART
        esp8266_update = true;

    }
    else if (strcmp(argv[i], "init") == CONSOLE_MATCH)
    {
      ConsoleClearScreen();
      PrintfConsoleCRLF("\tINIT WIFI");
      IoSystemClearRxQueue();
      LogClearQueues();
      IoSystemSetMode(IO_LOGS);
      WiFiInit();
    }
    else if (strcmp(argv[i], "ssid") == CONSOLE_MATCH)
    {
      if (++i < argc)
      {
        if (strlen(argv[i]) > 20)
        {
          PrintfConsoleCRLF(CLR_RD"\tToo big ssid name!"CLR_DEF);
          ConsoleError();
        }
        else
        {
          char buf[32];
          memcpy(buf, argv[i], 32);
          for (uint8_t i = 0; i < 32; i++)
          {
            config.wifi.ssid[i] = buf[i];
          }
          PrintfConsoleCRLF(CLR_DEF"\tNew ssid:" CLR_GR"%s"CLR_DEF, config.wifi.ssid);
        }
      }
    }
    else if (strcmp(argv[i], "password") == CONSOLE_MATCH)
    {
      if (++i < argc)
      {
        if (strlen(argv[i]) > 20)
        {
          PrintfConsoleCRLF(CLR_RD"\tToo big password!"CLR_DEF);
          ConsoleError();
        }
        else
        {
          char buf[32];
          memcpy(buf, argv[i], 32);
          for (uint8_t i = 0; i < 32; i++)
          {
            config.wifi.passw[i] = buf[i];
          }
          PrintfConsoleCRLF(CLR_DEF"\tNew password:" CLR_GR"%s"CLR_DEF, config.wifi.passw);
        }
      }
    }
    else if (strcmp(argv[i], "configs") == CONSOLE_MATCH)
    {
      PrintfConsoleCRLF(CLR_DEF"\tSSID:" CLR_GR"%s"CLR_DEF, config.wifi.ssid);
      PrintfConsoleCRLF(CLR_DEF"\tPASSWORD" CLR_GR"%s"CLR_DEF, config.wifi.passw);
    }
    else if (strcmp(argv[i], "version") == CONSOLE_MATCH)
    {
      PrintfConsoleCRLF("\t"CLR_YL"ESP8266 AT  v%u.%u.%u"CLR_DEF, esp.m.version_at.major, esp.m.version_at.minor, esp.m.version_at.patch);
      PrintfConsoleCRLF("\t"CLR_YL"ESP8266 SDK v%u.%u.%u"CLR_DEF, esp.m.version_sdk.major, esp.m.version_sdk.minor, esp.m.version_sdk.patch);
      WiFiGetMac();
      res = WiFiGetInfoAp();

      if (res != espOK)
        return CONSOLE_ERROR;
    }
    else if (strcmp(argv[i], "ap") == CONSOLE_MATCH)
    {
      ConsoleClearScreen();
      PrintfConsoleCRLF("\t"CLR_GR"OK, switching ESP8266 mode to "CLR_YL"AP"CLR_DEF);
      IoSystemClearRxQueue();
      LogClearQueues();
      IoSystemSetMode(IO_LOGS);

      WiFiStop();
      WiFiStart(WIFI_MODE_AP);
      return CONSOLE_OK;
    }
    else if (strcmp(argv[i], _CMD_BACK) == CONSOLE_MATCH)
    {
      ConsoleBack();
    }
    else
    {
      ConsoleError();
    }
    i++;
  }

  return CONSOLE_OK;
}
/******************************************************************************/




/**
 * @brief          Print WI-FI menu
 */
void Console_WIFiPrintMenu(void)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("List of wi-fi commands:");
  PrintfConsoleCRLF("\tupdate             -  update ESP");
  PrintfConsoleCRLF("\tinit               -  init ESP");
  PrintfConsoleCRLF("\tversion            -  check current version");
  PrintfConsoleCRLF("\tssid               -  change wifi ssid");
  PrintfConsoleCRLF("\tpassword           -  change wifi password");
  PrintfConsoleCRLF("\tconfigs            -  check ssid and passw");
  PrintfConsoleCRLF("\tback               -  back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/

