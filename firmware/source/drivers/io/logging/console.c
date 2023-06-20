/**
 ******************************************************************************
 * @file           : console.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Console system
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "console.h"

#include "esp/system/esp_ll.h"
#include "esp/esp_sta.h"
#include "esp/esp_private.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define _ENDLINE_SEQ                "\r\n"

/* Definition commands word */
#define _CMD_HELP                   "help"
#define _CMD_CLEAR                  "clear"
#define _CMD_LOGIN                  "login"
#define _CMD_LOGOUT                 "logout"

#define _CMD_CALENDAR               "calendar"
#define _CMD_DATE                   "date"
#define _CMD_BACK                   "back"
#define _CMD_TIME                   "time"

#define _CMD_WIFI                   "wifi"

/* Arguments for set/clear */
#define _SCMD_RD                    "?"
#define _SCMD_SAVE                  "save"

#define _NUM_OF_CMD                 9
#define _NUM_OF_SETCLEAR_SCMD       2

#if MICRORL_CFG_USE_ECHO_OFF
#define SESSION_ADMIN_LOGIN        "admin"
#define SESSION_ADMIN_PASSW        "17391739"
#endif /* MICRORL_CFG_USE_ECHO_OFF */

#define CONSOLE_MATCH               0u

#if MICRORL_CFG_USE_ECHO_OFF
/* Session status flags */
uint8_t  logged_in = 0;
uint8_t  passw_in = 0;
#endif /* MICRORL_CFG_USE_ECHO_OFF */


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
microrl_t microrl;
microrl_t *microrl_ptr = &microrl;

char *keyword[] = {_CMD_HELP, _CMD_CLEAR, _CMD_LOGIN, _CMD_LOGOUT
        , _CMD_CALENDAR, _CMD_DATE, _CMD_BACK, _CMD_TIME};    //available  commands

char *read_save_key[] = {_SCMD_RD, _SCMD_SAVE};            // 'read/save' command arguments
char *compl_word [_NUM_OF_CMD + 1];                        // array for completion

typedef void (*help_command_fn)(void);

typedef enum
{
  CONSOLE_OK = 0x00,
  CONSOLE_NO_CMD = 0x01,

  CONSOLE_ERROR = 0x10,
  CONSOLE_ERROR_UNKNOWN_COMMAND,
  CONSOLE_ERROR_MAX_ARGS
} console_error_t;

typedef struct
{
  help_command_fn    help_command;
} console_ctrl_t;

typedef struct
{
  console_error_t error;
  console_ctrl_t fns;
} console_t;

console_error_t console_error;
console_t console;

bool esp8266_update;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
void prvConsoleClearScreen(void);
void prvConsoleBack(void);
void prvConsoleClearScreenSimple(microrl_t *microrl_ptr);
static void prvConsolePrint(microrl_t *microrl_ptr, const char *str);
void prvConsolePrintCalendar(void);
void prvConsoleSetHelp(void (*fn)(void));
void prvConsoleError(void);
void prvConsolePrintWiFi(void);


/******************************************************************************/


/**
 * @brief          Console (MicroRL init)
 */
void ConsoleInit(void)
{
  microrl_init(microrl_ptr, prvConsolePrint, ConsoleExecute);

  prvConsoleSetHelp(ConsolePrintHelp);

#if MICRORL_CFG_USE_COMPLETE
  /* Set callback for auto-completion */
  microrl_set_complete_callback(microrl_ptr, ConsoleComplete);
#endif /* MICRORL_CFG_USE_COMPLETE */

#if CONSOLE_NO_PASSW
  logged_in = 1;
  PrintfConsoleCont(CLR_CLR);
  console.fns.help_command();
  microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
#endif /* CONSOLE_NO_PASSW */

#if MICRORL_CFG_USE_CTRL_C
  /* Set callback for Ctrl+C handling */
  microrl_set_sigint_callback(microrl_ptr, ConsoleSigint);
#endif /* MICRORL_CFG_USE_CTRL_C */
}
/******************************************************************************/




/**
 * @brief          Console start
 */
void ConsoleStart(void)
{
  IoSystemClearRxQueue();
  LogClearQueues();
  ConsoleInit();
}
/******************************************************************************/




/**
 * @brief          Console printf
 */
void prvConsolePrint(microrl_t *microrl_ptr, const char *str)
{
  UNUSED(microrl_ptr);
  PrintfConsoleCont("%s", str);
}
/******************************************************************************/




/**
 * @brief          Console insert char
 */
void ConsoleInsertChar(char ch)
{
  microrl_processing_input(microrl_ptr, &ch, 1);
}
/******************************************************************************/




/**
 * @brief          Get char from keyboard
 * @return         io_uart.transmit: current symbol to transmit
 */
char ConsoleGetChar(void)
{
  return (char)io_uart.transmit;
}
/******************************************************************************/




/**
 * @brief           Basic menu in console
 * @param[in]       @ref microrl_ptr pointer
 * @param[in]       argc: shows how many arguments were entered on the command line
 * @param[in]       argv: vector of string, it's elements are symbols
 * @return          return 0 if everything is ok
 */
#if MICRORL_CFG_USE_ECHO_OFF
int ConsoleExecuteMain(microrl_t* microrl_ptr, int argc, const char* const *argv) {
#else
int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv) {
#endif /* MICRORL_CFG_USE_ECHO_OFF || __DOXYGEN__ */
  int i = 0;
  prvConsoleSetHelp(ConsolePrintHelp);

  while (i < argc)
  {
    if (strcmp(argv[i], _CMD_HELP) == CONSOLE_MATCH)
    {
      console.fns.help_command();
    }
    else if (strcmp(argv[i], _CMD_CLEAR) == CONSOLE_MATCH)
    {
      prvConsoleClearScreen();
    }
    else if (strcmp(argv[i], _CMD_LOGOUT) == CONSOLE_MATCH)
    {
      prvConsoleClearScreen();
      IoSystemClearRxQueue();
      LogClearQueues();
      microrl_set_execute_callback(microrl_ptr, ConsoleExecute);
      IoSystemSetMode(IO_LOGS);
    }
    else if (strcmp(argv[i], _CMD_CALENDAR) == CONSOLE_MATCH)
    {
      PrintfConsoleCRLF("\tChoose your action with calendar: ");
      prvConsolePrintCalendar();
      microrl_set_execute_callback(microrl_ptr, ConsoleCalendar);
    }
    else if (strcmp(argv[i], _CMD_WIFI) == CONSOLE_MATCH)
    {
      PrintfConsoleCRLF("\tChoose your action with wi-fi: ");
      prvConsolePrintWiFi();
      microrl_set_execute_callback(microrl_ptr, ConsoleWiFi);
    }
    else
    {
      prvConsoleError();
    }
    i++;
  }

  return CONSOLE_OK;
}
/******************************************************************************/




/**
 * @brief           Login menu in console
 * @param[in]       @ref microrl_ptr pointer
 * @param[in]       argc: shows how many arguments were entered on the command line
 * @param[in]       argv: vector of string, it's elements are symbols
 * @return          return 0 if everything is ok
 */
int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
  int i = 0;

  while (i < argc)
  {
    if (strcmp(argv[i], _CMD_LOGIN) == CONSOLE_MATCH)
    {
      if (++i < argc)
      {
        if (strcmp(argv[i], SESSION_ADMIN_LOGIN) == CONSOLE_MATCH)
        {
          PrintfConsoleCRLF("\tEnter password:");
          microrl_set_echo(microrl_ptr, MICRORL_ECHO_ONCE);
          passw_in = 1;
          return CONSOLE_OK;
        }
        else
        {
          PrintfConsoleCRLF("\tLogin name doesn't registered. Try again.");
          IndicationLedRed();
          return CONSOLE_ERROR;
        }
      }
      else
      {
        PrintfConsoleCRLF("\tEnter your login after 'login' command.");
        return CONSOLE_OK;
      }
    }
    else if (passw_in == 1)
    {
      if (strcmp(argv[i], SESSION_ADMIN_PASSW) == CONSOLE_MATCH)
      {
        PrintfConsoleCRLF("\tLogged in successfully.");
        passw_in = 0;
        logged_in = 1;
        ConsolePrintHelp();
        microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
        return CONSOLE_OK;
      }
      else
      {
        PrintfConsoleCRLF("\tWrong password. Try again.");
        IndicationLedRed();
        passw_in = 0;
        return CONSOLE_ERROR;
      }
    }
    else
    {
        PrintfConsoleCRLF("\tYou need to login first!");
        PrintfConsoleCRLF("\tlogin YOUR_LOGIN");
        IndicationLedRed();
        return CONSOLE_ERROR;
    }
    i++;
  }

  return CONSOLE_OK;
}
/******************************************************************************/




/**
 * @brief           Calendar menu in console
 * @param[in]       @ref microrl_ptr pointer
 * @param[in]       argc: shows how many arguments were entered on the command line
 * @param[in]       argv: vector of string, it's elements are symbols
 * @return          return 0 if everything is ok
 */
int ConsoleCalendar(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
  int i = 0;
  uint8_t res = 0x00;
  prvConsoleSetHelp(prvConsolePrintCalendar);

  while (i < argc)
  {
    if (strcmp(argv[i], _CMD_DATE) == CONSOLE_MATCH)
    {
      if (++i < argc)
      {
        if (strcmp(argv[i], "?") == CONSOLE_MATCH)
        {
          res = RtcSetStatus(RTC_GET_DATE);
          if (res != RTC_OK)
          {
            RtcSetError(res);
            return CONSOLE_OK;
          }
        }
        else if ((strlen(argv[i]) != 10) || (argv[i][2] != '.') || (argv[i][5] != '.'))
        {
          PrintfConsoleCRLF("\t"CLR_RD"\tERROR: date format is not DD:MM:YYYY"CLR_DEF);
          return CONSOLE_OK;
        }
        else
        {
          char buf[11];
          memcpy(buf, argv[i], 10);
          res = RtcSetDate(buf);
          if (res != RTC_OK)
          {
            RtcSetError(res);
            return CONSOLE_OK;
          }
        }
      }
    }
    else if (strcmp(argv[i], _CMD_TIME) == CONSOLE_MATCH)
    {
      if (++i < argc)
      {
        if (strcmp(argv[i], "?") == CONSOLE_MATCH)
        {
          res = RtcSetStatus(RTC_GET_TIME);
          if (res != RTC_OK)
          {
            RtcSetError(res);
            return CONSOLE_OK;
          }
        }
        else if ((strlen(argv[i]) != 8) || (argv[i][2] != ':') || (argv[i][5] != ':'))
        {
          PrintfConsoleCRLF("\t"CLR_RD"\tERROR: time format is not HH:MM:SS"CLR_DEF);
          return CONSOLE_OK;
        }
        else
        {
          char buf[9];
          memcpy(buf, argv[i], 8);
          res = RtcSetTime(buf);
          if (res != RTC_OK)
          {
            RtcSetError(res);
            return CONSOLE_OK;
          }
        }
      }
    }
    else if (strcmp(argv[i], _CMD_BACK) == CONSOLE_MATCH)
    {
      prvConsoleBack();
    }
    else
    {
      prvConsoleError();
    }
    i++;
  }

  return CONSOLE_OK;
}
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
  prvConsoleSetHelp(prvConsolePrintWiFi);

  while (i < argc)
  {
    if (strcmp(argv[i], "update") == CONSOLE_MATCH)
    {
        prvConsoleClearScreen();
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
      prvConsoleClearScreen();
      PrintfConsoleCRLF("\tINIT WIFI");
      WiFiInit();
    }
    else if (strcmp(argv[i], _CMD_BACK) == CONSOLE_MATCH)
    {
      prvConsoleBack();
    }
    else
    {
      prvConsoleError();
    }
    i++;
  }

  return CONSOLE_OK;
}
/******************************************************************************/




/**
 * @brief           Clear console screen fn extern
 */
void ConsoleClearScreenSetup(void)
{
  prvConsoleClearScreenSimple(microrl_ptr);
}
/******************************************************************************/




/**
 * @brief           Clear console screen fn
 */
void prvConsoleClearScreen(void)
{
  PrintfConsoleCRLF("\033[2J");
  PrintfConsoleCRLF("\033[H");
}
/******************************************************************************/




/**
 * @brief           Clear screen simple
 */
void prvConsoleClearScreenSimple(microrl_t *microrl_ptr)
{
  PrintfConsoleCRLF("\033[2J");
}
/******************************************************************************/




/**
 * @brief           Back to main menu command
 */
void prvConsoleBack(void)
{
  PrintfConsoleCRLF("\tBACK TO MAIN MENU"CLR_DEF);
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("");
  prvConsoleSetHelp(ConsolePrintHelp);
  console.fns.help_command();
  microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
}
/******************************************************************************/




/**
 * @brief           Console error message
 */
void prvConsoleError(void)
{
  PrintfConsoleCRLF("\tUndefined command");
  IndicationLedRed();
  console.fns.help_command();
}
/******************************************************************************/




#if MICRORL_CFG_USE_CTRL_C || __DOXYGEN__
/**
 * @brief           Ctrl+C terminal signal function
 * @param[in]       mrl: \ref microrl_t working instance
 */
void ConsoleSigint(microrl_t *microrl_ptr)
{
  prvConsoleClearScreen();
}
/******************************************************************************/




/**
 * @brief           Completion callback for microrl library
 * @param[in]       argc: shows how many arguments were entered on the command line
 * @param[in]       argv: vector of string, it's elements are symbols
 * @return          return complete word (string)
 */
char **ConsoleComplete(int argc, const char * const *argv)
{
  int j = 0;

  compl_word[0] = NULL;

  if (argc == 1)
  {
    char *bit = (char *)argv[argc - 1];

    for (int i = 0; i < _NUM_OF_CMD; i++)
    {
      if (strstr(keyword[i], bit) == keyword[i])
      {
        compl_word[j++] = keyword[i];
      }
    }
  }
  else if ((argc > 1) && ((strcmp (argv[0], _CMD_LOGIN) == CONSOLE_MATCH)))
  {  /*If command needs subcommands */
    /* Iterate through subcommand */
    for (int i = 0; i <  _NUM_OF_SETCLEAR_SCMD; i++)
    {
      if (strstr (read_save_key[i], argv[argc-1]) == read_save_key [i])
      {
        compl_word[j++] = read_save_key[i];
      }
    }
  }
  else
  { /* If there is no token in cmdline, just print all available token */
    for (; j < _NUM_OF_CMD; j++)
    {
      compl_word[j] = keyword[j];
    }
  }
  compl_word[j] = NULL; /* Last ptr in array always must be NULL */

  return compl_word;
}
#endif /* MICRORL_CFG_USE_COMPLETE || __DOXYGEN__ */




/**
 * @brief          Print "help"
 */
void ConsolePrintHelp(void)
{
  char ver_str[6] = {0};
  ConsoleGetVersion(ver_str);
  PrintfConsoleCont("");
  PrintfConsoleCont("Console "CLR_GR "v ");
  PrintfConsoleCont(ver_str);
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF(CLR_YL"(modified by AlexSH)"CLR_DEF);
  PrintfConsoleCRLF(CLR_YL"https://github.com/alexVOLTS"CLR_DEF);
  PrintfConsoleCRLF("");

#if MICRORL_CFG_USE_ECHO_OFF
  if (!logged_in)
    PrintfConsoleCRLF("\tlogin YOUR_LOGIN      - 'admin' in this example");
#endif /* MICRORL_CFG_USE_ECHO_OFF */

  PrintfConsoleCRLF("List of commands:");
  PrintfConsoleCRLF("\tclear               - clear screen");
  PrintfConsoleCRLF("\tlogout              - end session");
  PrintfConsoleCRLF("\tcalendar            - calendar config menu");
  PrintfConsoleCRLF("\twifi                - start wifi");

#if MICRORL_CFG_USE_COMPLETE
  PrintfConsoleCRLF("Use TAB key for completion");
#endif /* MICRORL_CFG_USE_COMPLETE */
}
/******************************************************************************/




/**
 * @brief          Print buff menu
 */
void ConsolePrintBuff(microrl_t *microrl_ptr)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("List of buff commands:");
  PrintfConsoleCRLF("\treset               - reset buffer");
  PrintfConsoleCRLF("\tfree                - free buff memory");
  PrintfConsoleCRLF("\tcheck               - check your buff");
  PrintfConsoleCRLF("\tback                - back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/




/**
 * @brief          Print visualizer menu
 */
void ConsolePrintVisualizer(microrl_t *microrl_ptr)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("List of audio visualizer commands:");
  PrintfConsoleCRLF("\tenable             -  enable visualization");
  PrintfConsoleCRLF("\tback               -  back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/




/**
 * @brief          Print welcome message after swapping from logs to console
 */
void ConsolePrintWelcome(microrl_t *microrl_ptr)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("\tWelcome to console!");
  PrintfConsoleCRLF("\tEnter your login, than your password please");
}
/******************************************************************************/




/**
 * @brief          Print visualizer menu
 */
void prvConsolePrintCalendar(void)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("List of calendar commands:");
  PrintfConsoleCRLF("\tdate               -  set date");
  PrintfConsoleCRLF("\ttime               -  set date");
  PrintfConsoleCRLF("\tback               -  back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/




/**
 * @brief          Print WI-FI menu
 */
void prvConsolePrintWiFi(void)
{
  PrintfConsoleCRLF("");
  PrintfConsoleCRLF("List of wi-fi commands:");
  PrintfConsoleCRLF("\tupdate             -  update ESP");
  PrintfConsoleCRLF("\tinit               -  init ESP");
  PrintfConsoleCRLF("\tback               -  back to main menu");
  PrintfConsoleCRLF("");
}
/******************************************************************************/




/**
 * @brief          Set help print function
 */
void prvConsoleSetHelp(void (*fn)(void))
{
  console.fns.help_command = fn;
}
/******************************************************************************/




/**
 * @brief          Get console version
 */
void ConsoleGetVersion(char* ver_str)
{
  uint32_t ver = microrl_get_version();

  ver_str[0] = (char)((ver >> 16) & 0x000000FF) + '0';
  ver_str[1] = '.';
  ver_str[2] = (char)((ver >> 8) & 0x000000FF) + '0';
  ver_str[3] = '.';
  ver_str[4] = (char)(ver & 0x000000FF) + '0';
}
/******************************************************************************/
