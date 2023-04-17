/**
 ******************************************************************************
 * @file           : log.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : logging system
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "log.h"
#include "uart.h"

#include "lwprintf/lwprintf.h"




/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define LOGS_QUEUE_SIZE            (512U)
#define CONSOLE_QUEUE_SIZE         (512U)




/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
osMessageQueueId_t consoleQueueHandle;
osMessageQueueId_t logsQueueHandle;

static lwprintf_t console;
static lwprintf_t logs;

const osMessageQueueAttr_t consoleQueueAttributes = {
        .name = "consoleQueue",
};

const osMessageQueueAttr_t logsQueueAttributes = {
        .name = "logsQueue",
};




/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static int prvLwprintfLogsOut(int ch, lwprintf_t* p);
static int prvLwprintfConsoleOut(int ch, lwprintf_t* p);

/******************************************************************************/




/**
 * @brief          Init logging system
 */
void LogInit(void)
{
    consoleQueueHandle = osMessageQueueNew(512, sizeof(uint8_t), &consoleQueueAttributes);
    logsQueueHandle = osMessageQueueNew(512, sizeof(uint8_t), &logsQueueAttributes);

    lwprintf_init_ex(&console, prvLwprintfConsoleOut);
    lwprintf_init_ex(&logs, prvLwprintfLogsOut);
}
/******************************************************************************/




/**
 * @brief          Clear Queues of logs and console
 */
void LogClearQueues(void)
{
    osMessageQueueReset(logsQueueHandle);
    osMessageQueueReset(consoleQueueHandle);
}
/******************************************************************************/




/**
 * @brief          Printf of logs
 */
int PrintfLogs(const char *fmt, ...)
{
    if (IoSystemGetMode() != IO_LOGS) {
        return 0;
    }

    va_list args;
    int len;

    va_start(args, fmt);
    len = lwprintf_vprintf_ex(&logs, fmt, args);
    va_end(args);

    return (len);
}
/******************************************************************************/




/**
 * @brief          Printf of console
 */
int PrintfConsole(const char *fmt, ...)
{
    if (IoSystemGetMode() != IO_CONSOLE) {
        return 0;
    }

    va_list args;
    int len;

    va_start(args, fmt);
    len = lwprintf_vprintf_ex(&console, fmt, args);
    va_end(args);

    return (len);
}
/******************************************************************************/




/**
 * @brief          Printf logs out (add to logs queue)
 */
static int prvLwprintfLogsOut(int ch, lwprintf_t* p)
{
    uint8_t c = (uint8_t)ch;

    if (c == '\0') {
        return ch;           //to prevent printing '0' in the end of any (char*)
    }

    osMessageQueuePut(logsQueueHandle, &ch, 0, 200);

    return (ch);
}
/******************************************************************************/




/**
 * @brief          Printf console out (add to console queue)
 */
static int prvLwprintfConsoleOut(int ch, lwprintf_t* p)
{
    uint8_t c = (uint8_t)ch;

    if (c == '\0') {
        return ch;           //to prevent printing '0' in the end of any (char*)
    }

    osMessageQueuePut(consoleQueueHandle, &ch, 0, 200);

    return (ch);
}
/******************************************************************************/




/**
 * @brief          Print welcome message at the start
 */
void LogPrintWelcomeMsg(void)
{
    PrintfLogsCRLF(CLR_DEF"");
    PrintfLogsCRLF("");
    PrintfLogsCRLF("\tWelcome to STM32F407 Discovery firmware with" CLR_RD " FreeRTOS");
    PrintfLogsCRLF(CLR_DEF"");
    PrintfLogsCRLF("");
}
/******************************************************************************/
