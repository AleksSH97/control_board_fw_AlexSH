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
#include "esp/system/esp_ll.h"
#include "esp/esp_sta.h"
#include "esp/esp_private.h"


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

        esp_ll_deinit(NULL);
        configure_uart(esp.ll.uart.baudrate);
        //TODO PREPARE FOR ESP UPDATE IO UART
        esp8266_update = true;

    }
    else if (strcmp(argv[i], "init") == CONSOLE_MATCH)
    {
      ConsoleClearScreen();
      PrintfConsoleCRLF("\tINIT WIFI");
      WiFiInit();
    }
    else if (strcmp(argv[i], "version") == CONSOLE_MATCH)
    {
      PrintfConsoleCRLF("\t"CLR_YL"ESP8266 AT  v%u.%u.%u"CLR_DEF, esp.m.version_at.major, esp.m.version_at.minor, esp.m.version_at.patch);
      PrintfConsoleCRLF("\t"CLR_YL"ESP8266 SDK v%u.%u.%u"CLR_DEF, esp.m.version_sdk.major, esp.m.version_sdk.minor, esp.m.version_sdk.patch);
      WiFiGetMac();
      res = WiFiGetInfoAp();

      if (res != espOK)
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
  PrintfConsoleCRLF("\tback               -  back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/

