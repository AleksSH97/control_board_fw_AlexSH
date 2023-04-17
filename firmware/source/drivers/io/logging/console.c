/**
 ******************************************************************************
 * @file           : console.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Console system (MicroRL)
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "console.h"

/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define _STM32_DEMO_VER             "1.1"

#define _ENDLINE_SEQ                "\r\n"

/* Definition commands word */
#define _CMD_HELP                   "help"
#define _CMD_CLEAR                  "clear"
#define _CMD_LOGIN                  "login"
#define _CMD_LOGOUT                 "logout"

#define _CMD_AUDIO                  "audio"
#define _CMD_ACCELERO               "accelero"
#define _CMD_ENABLE                 "enable"

#define _CMD_BUFF                   "buff"
#define _CMD_RESET                  "reset"
#define _CMD_FREE                   "free"
#define _CMD_CHECK                  "check"
#define _CMD_BACK                   "back"
/* Arguments for set/clear */
#define _SCMD_RD                    "?"
#define _SCMD_SAVE                  "save"

#define _NUM_OF_CMD                 12
#define _NUM_OF_SETCLEAR_SCMD       2

#if MICRORL_CFG_USE_ECHO_OFF
#define SESSION_ADMIN_LOGIN        "admin"
#define SESSION_ADMIN_PASSW        "17391739"
#endif /* MICRORL_CFG_USE_ECHO_OFF */

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

char *keyword[] = {_CMD_HELP, _CMD_CLEAR, _CMD_LOGIN, _CMD_LOGOUT,
        _CMD_BUFF, _CMD_AUDIO, _CMD_RESET, _CMD_FREE,
        _CMD_ACCELERO, _CMD_ENABLE, _CMD_BACK};    //available  commands

char *read_save_key[] = {_SCMD_RD, _SCMD_SAVE};            // 'read/save' command arguments
char *compl_word [_NUM_OF_CMD + 1];                        // array for completion

/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static void prvConsoleClearScreen(microrl_t *microrl_ptr);
void prvConsoleClearScreenSimple(microrl_t *microrl_ptr);
static void prvConsolePrint(microrl_t *microrl_ptr, const char *str);



/******************************************************************************/


/**
 * @brief          Console (MicroRL init)
 */
void ConsoleInit(void)
{
    microrl_init(microrl_ptr, prvConsolePrint, ConsoleExecute);

#if MICRORL_CFG_USE_COMPLETE
    /* Set callback for auto-completion */
    microrl_set_complete_callback(microrl_ptr, ConsoleComplete);
#endif /* MICRORL_CFG_USE_COMPLETE */

#if CONSOLE_NO_PASSW
    logged_in = 1;
    PrintfConsoleCont(CLR_CLR);
    ConsolePrintHelp(microrl_ptr);
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
    if (MicrophoneGetActivate()) {
        MicrophoneSetActivate(MICROPHONE_OFF);
    }

    AccelerometerSetStatus(ACCELERO_IDLE);
    IoSystemClearRxQueue();
    LogClearQueues();
    MicrophoneVisualizationClearQueue();
    ConsoleInit();
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
 * @brief          Console printf
 */
void prvConsolePrint(microrl_t *microrl_ptr, const char *str)
{
    UNUSED(microrl_ptr);
    PrintfConsoleCont("%s", str);
}
/******************************************************************************/



/**
 * @brief          Get char from keyboard
 */
char ConsoleGetChar(void)
{
   return (char)data_uart.console_input;
}
/******************************************************************************/




/**
 * \brief           Basic menu in console
 * \param[in]
 */
#if MICRORL_CFG_USE_ECHO_OFF
int ConsoleExecuteMain(microrl_t* microrl_ptr, int argc, const char* const *argv) {
#else
int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv) {
#endif /* MICRORL_CFG_USE_ECHO_OFF || __DOXYGEN__ */
    int i = 0;

    while (i < argc) {
        if (strcmp(argv[i], _CMD_HELP) == 0) {
            ConsolePrintHelp(microrl_ptr);        // print help
        }
        else if (strcmp(argv[i], _CMD_CLEAR) == 0) {
            prvConsoleClearScreen(microrl_ptr);
        }
        else if (strcmp(argv[i], _CMD_LOGOUT) == 0) {
            prvConsoleClearScreen(microrl_ptr);
            IoSystemClearRxQueue();
            LogClearQueues();
            microrl_set_execute_callback(microrl_ptr, ConsoleExecute);
            IoSystemSetMode(IO_LOGS);
        }
        else if (strcmp(argv[i], _CMD_BUFF) == 0) {
            prvConsolePrint(microrl_ptr, "\tChoose your action with buffer: " _ENDLINE_SEQ);
            ConsolePrintBuff(microrl_ptr);
            microrl_set_execute_callback(microrl_ptr, ConsoleBuff);
        }
        else if (strcmp(argv[i], _CMD_AUDIO) == 0) {
            prvConsolePrint(microrl_ptr, "\tChoose your action with audio: " _ENDLINE_SEQ);
            ConsolePrintVisualizer(microrl_ptr);
            microrl_set_execute_callback(microrl_ptr, ConsoleAudio);
        }
        else if (strcmp(argv[i], _CMD_ACCELERO) == 0) {
            prvConsolePrint(microrl_ptr, "\tChoose your action with accelerometer: " _ENDLINE_SEQ);
            ConsolePrintAccelerometer(microrl_ptr);
            microrl_set_execute_callback(microrl_ptr, ConsoleAccelerometer);
        }
        else {
            IndicationLedError();
            prvConsolePrint(microrl_ptr, "command: '");
            prvConsolePrint(microrl_ptr, (char*)argv[i]);
            prvConsolePrint(microrl_ptr, "' Not found." _ENDLINE_SEQ);
        }
        i++;
    }

    return 0;
}
/******************************************************************************/




/**
 * \brief           Login menu in console
 * \param[in]
 */
int ConsoleExecute(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
    int i = 0;

    while (i < argc) {
        if (strcmp(argv[i], _CMD_LOGIN) == 0) {
            if (++i < argc) {
                if (strcmp(argv[i], SESSION_ADMIN_LOGIN) == 0) {
                    prvConsolePrint(microrl_ptr, "\tEnter password:" _ENDLINE_SEQ);
                    microrl_set_echo(microrl_ptr, MICRORL_ECHO_ONCE);
                    passw_in = 1;
                    return 0;
                }
                else {
                    prvConsolePrint(microrl_ptr, "\tLogin name doesn't registered. Try again." _ENDLINE_SEQ);
                    IndicationLedError();
                    return 1;
                }
            }
            else {
                prvConsolePrint(microrl_ptr, "\tEnter your login after 'login' command." _ENDLINE_SEQ);
                return 0;
            }
        }
        else if (passw_in == 1) {
            if (strcmp(argv[i], SESSION_ADMIN_PASSW) == 0) {
                prvConsolePrint(microrl_ptr, "\tLogged in successfully." _ENDLINE_SEQ);
                passw_in = 0;
                logged_in = 1;
                ConsolePrintHelp(microrl_ptr);
                microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
                return 0;
            }
            else {
                prvConsolePrint(microrl_ptr, "\tWrong password. Try again." _ENDLINE_SEQ);
                IndicationLedError();
                passw_in = 0;
                return 1;
            }
        }
        else {
            prvConsolePrint(microrl_ptr, "\tYou need to login first!" _ENDLINE_SEQ);
            prvConsolePrint(microrl_ptr, "\tlogin YOUR_LOGIN"_ENDLINE_SEQ);
            IndicationLedError();
            return 1;
        }
        i++;
    }

    return 0;
}
/******************************************************************************/




/**
 * \brief           Buff menu in console
 * \param[in]
 */
int ConsoleBuff(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
    int i = 0;

    while (i < argc) {
        if (strcmp(argv[i], _CMD_RESET) == 0) {
            lwrb_reset(&debug_uart.lwrb_rx);
            prvConsolePrint(microrl_ptr, "\tBuffer successfully reseted" _ENDLINE_SEQ);
        }
        else if (strcmp(argv[i], _CMD_FREE) == 0) {
            lwrb_free(&debug_uart.lwrb_rx);
            prvConsolePrint(microrl_ptr, "\tBuffer successfully free" _ENDLINE_SEQ);
        }
        else if (strcmp(argv[i], _CMD_CHECK) == 0) {
            prvConsolePrint(microrl_ptr, "\tHere's your buffer:  ");
            // TODO insert buffer to show
            prvConsolePrint(microrl_ptr, "\t " _ENDLINE_SEQ);
        }
        else if (strcmp(argv[i], _CMD_BACK) == 0) {
            prvConsolePrint(microrl_ptr, "\tBack to main menu" _ENDLINE_SEQ _ENDLINE_SEQ _ENDLINE_SEQ);
            ConsolePrintHelp(microrl_ptr);
            microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
        }
        else {
            prvConsolePrint(microrl_ptr, "\tUndefined command" _ENDLINE_SEQ);
            IndicationLedError();
            ConsolePrintBuff(microrl_ptr);
        }
        i++;
    }

    return 0;
}
/******************************************************************************/




/**
 * \brief           Visualizer menu in console
 * \param[in]
 */
int ConsoleAudio(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
    int i = 0;

    while (i < argc) {
        if (strcmp(argv[i], _CMD_ENABLE) == 0) {

            if (!MicrophoneGetActivate()) {
                MicrophoneSetActivate(MICROPHONE_ON);
            }

            AudioSetStatus(AUDIO_INIT);
            IoSystemSetMode(IO_LOGS);
        }
        else if (strcmp(argv[i], _CMD_BACK) == 0) {
            prvConsolePrint(microrl_ptr, "\tBack to main menu" _ENDLINE_SEQ _ENDLINE_SEQ _ENDLINE_SEQ);
            ConsolePrintHelp(microrl_ptr);
            microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
        }
        else {
            prvConsolePrint(microrl_ptr, "\tUndefined command" _ENDLINE_SEQ);
            IndicationLedError();
            ConsolePrintVisualizer(microrl_ptr);
        }
        i++;
    }

        return 0;
}
/******************************************************************************/




/**
 * \brief           Accelerometer menu in console
 * \param[in]
 */
int ConsoleAccelerometer(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
    int i = 0;

    while (i < argc) {
        if (strcmp(argv[i], _CMD_ENABLE) == 0) {
            AccelerometerSetStatus(ACCELERO_XYZ);
            IoSystemSetMode(IO_LOGS);
        }
        else if (strcmp(argv[i], _CMD_BACK) == 0) {
            prvConsolePrint(microrl_ptr, "\tBack to main menu" _ENDLINE_SEQ _ENDLINE_SEQ _ENDLINE_SEQ);
            ConsolePrintHelp(microrl_ptr);
            AccelerometerSetStatus(ACCELERO_IDLE);
            microrl_set_execute_callback(microrl_ptr, ConsoleExecuteMain);
        }
        else {
            prvConsolePrint(microrl_ptr, "\tUndefined command" _ENDLINE_SEQ);
            IndicationLedError();
            ConsolePrintAccelerometer(microrl_ptr);
        }
        i++;
    }

        return 0;
}
/******************************************************************************/




/**
 * \brief           Clear console screen fn extern
 */
void ConsoleClearScreenSetup(void)
{
    prvConsoleClearScreenSimple(microrl_ptr);
}
/******************************************************************************/




/**
 * \brief           Clear console screen fn
 */
void prvConsoleClearScreen(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, "\033[2J" _ENDLINE_SEQ);    // ESC seq for clear entire screen
    prvConsolePrint(microrl_ptr, "\033[H" _ENDLINE_SEQ);     // ESC seq for move cursor at left-top corner
}
/******************************************************************************/




/**
 * \brief           Clear screen simple
 */
void prvConsoleClearScreenSimple(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, "\033[2J" _ENDLINE_SEQ);    // ESC seq for clear entire screen
}
/******************************************************************************/




#if MICRORL_CFG_USE_CTRL_C || __DOXYGEN__
/**
 * \brief           Ctrl+C terminal signal function
 * \param[in]       mrl: \ref microrl_t working instance
 */
void ConsoleSigint(microrl_t *microrl_ptr)
{
    MicrophoneSetActivate(MICROPHONE_OFF);
    AccelerometerSetStatus(ACCELERO_IDLE);
    microrl_set_execute_callback(microrl_ptr, ConsoleAudio);
    prvConsoleClearScreen(microrl_ptr);
    ConsolePrintVisualizer(microrl_ptr);
}




//*****************************************************************************
// completion callback for microrl library
char **ConsoleComplete(microrl_t *microrl_ptr, int argc, const char * const *argv)
{
    UNUSED(microrl_ptr);
    int j = 0;

    compl_word[0] = NULL;

    if (argc == 1) {
        char *bit = (char *)argv[argc - 1];

        for (int i = 0; i < _NUM_OF_CMD; i++) {
            if (strstr(keyword[i], bit) == keyword[i]) {
                compl_word[j++] = keyword[i];
            }
        }
    }
    else if ((argc > 1) && ((strcmp (argv[0], _CMD_LOGIN) == 0))) {  /*If command needs subcommands */
        /* Iterate through subcommand */
        for (int i = 0; i <  _NUM_OF_SETCLEAR_SCMD; i++) {
            if (strstr (read_save_key[i], argv[argc-1]) == read_save_key [i]) {
                compl_word[j++] = read_save_key[i];
            }
        }
    }
    else { /* If there is no token in cmdline, just print all available token */
        for (; j < _NUM_OF_CMD; j++) {
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
void ConsolePrintHelp(microrl_t *microrl_ptr)
{
    char ver_str[6] = {0};
    ConsoleGetVersion(ver_str);
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "MicroRL v");
    prvConsolePrint(microrl_ptr, ver_str);
    prvConsolePrint(microrl_ptr, " library DEMO v");
    prvConsolePrint(microrl_ptr, _STM32_DEMO_VER);
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);

#if MICRORL_CFG_USE_ECHO_OFF
    if (!logged_in) {
        prvConsolePrint(microrl_ptr, "\tlogin YOUR_LOGIN      - 'admin' in this example"_ENDLINE_SEQ);
    }
#endif /* MICRORL_CFG_USE_ECHO_OFF */

    prvConsolePrint(microrl_ptr, "List of commands:"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tclear               - clear screen"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tlogout              - end session"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\taudio               - audio configuration menu"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tbuff                - buff configuration menu"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\taccelero            - accelerometer configuration menu"_ENDLINE_SEQ);

#if MICRORL_CFG_USE_COMPLETE
    prvConsolePrint(microrl_ptr, "Use TAB key for completion"_ENDLINE_SEQ _ENDLINE_SEQ);
#endif /* MICRORL_CFG_USE_COMPLETE */
}
/******************************************************************************/




/**
 * @brief          Print buff menu
 */
void ConsolePrintBuff(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "List of buff commands:"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\treset               - reset buffer"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tfree                - free buff memory"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tcheck               - check your buff"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tback                - back to main menu"_ENDLINE_SEQ _ENDLINE_SEQ);
}
/******************************************************************************/




/**
 * @brief          Print visualizer menu
 */
void ConsolePrintVisualizer(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "List of audio visualizer commands:"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tenable             -  enable visualization"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tback               -  back to main menu"_ENDLINE_SEQ _ENDLINE_SEQ);
}
/******************************************************************************/




/**
 * @brief          Print visualizer menu
 */
void ConsolePrintAccelerometer(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "List of aceelerometer commands:"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tenable             -  show visualization"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tback               -  back to main menu"_ENDLINE_SEQ _ENDLINE_SEQ);
}
/******************************************************************************/




/**
 * @brief          Print welcome message after swapping from logs to console
 */
void ConsolePrintWelcome(microrl_t *microrl_ptr)
{
    prvConsolePrint(microrl_ptr, _ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tWelcome to console!"_ENDLINE_SEQ);
    prvConsolePrint(microrl_ptr, "\tEnter your login, than your password please"_ENDLINE_SEQ);
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
