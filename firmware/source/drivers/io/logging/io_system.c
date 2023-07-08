/**
 ******************************************************************************
 * @file           : io_system.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : i/o system
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "io_system.h"

#include "stm32f4xx_ll_dma.h"

#if    !WIFI_USE_LWESP
#include "esp/system/esp_ll.h"
#include "esp/esp_sta.h"
#include "esp/esp_private.h"
#endif


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define SOFT_TIMEOUT_MS             (1000U)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static IO_SYSTEM io_system;
osThreadId_t RxTaskHandle;
osThreadId_t TxTaskHandle;

osMessageQueueId_t uartRxQueueHandle;

const osThreadAttr_t RxTask_attributes = {
      .name = "RxTask",
      .stack_size = 256 * 4,
      .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t TxTask_attributes = {
      .name = "TxTask",
      .stack_size = 256 * 4,
      .priority = (osPriority_t) osPriorityNormal,
};

const osMessageQueueAttr_t uartRxQueueAttributes = {
      .name = "uartRxQueue",
};

volatile uint8_t esp8266_logs;

typedef struct
{
  uint8_t data[32];
  uint8_t id;
} IO_SYS_MSG;

IO_SYS_MSG msg;
/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static void prvIoSystemSetRxHandler(char rx);
static void prvIoLogsRxHandler(char rx);
static void prvIoConsoleRxHandler(char rx);

/******************************************************************************/




/**
 * @brief          IO init
 */
void IoSystemInit(void)
{
  IoSystemSetMode(IO_LOGS);

  esp8266_update = false;

  uint8_t init = 0x00;
  uart_ctrl_t fns = {0};

  fns.init = IoUartInit;
  fns.receive_byte = UARTReceiveByte;
  fns.send_byte = UARTSendByte;

  if (!UARTInit(&io_uart, &fns))
      init = 1;

  LogInit();
  ConsoleInit();

  if (init)
    LogPrintErrorMsg();

  RxTaskHandle = osThreadNew(IoSystemRxTask, NULL, &RxTask_attributes);
  TxTaskHandle = osThreadNew(IoSystemTxTask, NULL, &TxTask_attributes);
  uartRxQueueHandle = osMessageQueueNew(IOUART_RX_QUEUE_SIZE, sizeof(uint8_t), &uartRxQueueAttributes);
}
/******************************************************************************/




/**
 * @brief          Set IO mode of operation
 */
void IoSystemSetMode(IOSYS_MODE mode)
{
  io_system.mode = mode;
}
/******************************************************************************/




/**
 * @brief          Get IO mode of operation
 */
IOSYS_MODE IoSystemGetMode(void)
{
  return io_system.mode;
}
/******************************************************************************/




/**
 * @brief          Receive task
 */
void IoSystemRxTask(void *argument)
{
  uint8_t rx = 0x00;
  msg.id = 0U;

  LogPrintWelcomeMsg();

  for(;;)
  {
    //msg.data[0] = 0x00;

    if (!(IoSystemReadDataFromRxBuffer(msg.data)))
      continue;

    osMessageQueuePut(uartRxQueueHandle, msg.data, 0, 100);

    if (!(IoSystemGetByte(&rx, 100)))
      continue;

    IoSystemClearRxQueue();

    prvIoSystemSetRxHandler(rx);

    if (io_system.rx_handler != NULL)
      io_system.rx_handler(rx);
  }

  osThreadTerminate(NULL);
}
/******************************************************************************/




/**
 * @brief          Transmit task
 */
void IoSystemTxTask(void *argument)
{
  for(;;)
  {
    if (IoSystemGetMode() == IO_CONSOLE)
    {
      uint8_t msg = 0x00;

      if (!(IoSystemIsTxBufferFull()))
      {
        osStatus_t event = osMessageQueueGet(consoleQueueHandle, &msg, NULL, 200);

        if (event != osOK)
          continue;

        IoSystemPutDataToTxBuffer(&msg, sizeof(uint8_t));
      }

      io_uart.fns.send_byte(IOUART_Periph, &io_uart.lwrb_tx);
    }
    else if (IoSystemGetMode() == IO_LOGS)
    {
      if (esp8266_update)
        continue;

      uint8_t msg = 0x00;

      if (!(IoSystemIsTxBufferFull()))
      {
        osStatus_t event = osMessageQueueGet(logsQueueHandle, &msg, NULL, 200);

        if (event != osOK)
          continue;

        IoSystemPutDataToTxBuffer(&msg, sizeof(uint8_t));
      }

      io_uart.fns.send_byte(IOUART_Periph, &io_uart.lwrb_tx);
    }
  }

  osThreadTerminate(NULL);
}
/******************************************************************************/




/**
 * @brief          IO get byte
 */
bool IoSystemGetByte(uint8_t *data, uint32_t timeout_ms)
{
  *data = 0x00;

  osStatus_t event = osMessageQueueGet(uartRxQueueHandle, data, NULL, timeout_ms);

  if (event == osOK)
    return true;
  else
    return false;
}
/******************************************************************************/




/**
 * @brief          Set RX handler function
 */
void prvIoSystemSetRxHandler(char rx)
{
  if (IoSystemGetMode() == IO_CONSOLE)
  {
    io_system.rx_handler = prvIoConsoleRxHandler;
    return;
  }

//  if (IoSystemGetMode() == IO_LOGS)
//      return;

  IoSystemSetMode(IO_LOGS);
  io_system.rx_handler = prvIoLogsRxHandler;
}
/******************************************************************************/




/**
 * @brief          IO console RX handler
 */
void prvIoConsoleRxHandler(char rx)
{
  ConsoleInsertChar(rx);
}
/******************************************************************************/




/**
 * @brief          IO logs RX handler
 */
void prvIoLogsRxHandler(char rx)
{
  if ((rx == 'T') || (rx == 't'))
  {
    IoSystemSetMode(IO_CONSOLE);
    ConsoleStart();
    IndicationLedGreen();
    return;
  }

  if ((rx == 'x') || (rx == 'X'))
  {
    WiFiInit();
  }

  if ((rx == 'u') || (rx == 'U'))
  {
    uint8_t res = 0x00;

    WiFiStop();
    res = WiFiStart(WIFI_MODE_ST);

    if (res != espOK)
      PrintfLogsCRLF("ERROR: START WI-FI");

    esp_ll_deinit(NULL);
    configure_uart(esp.ll.uart.baudrate);

    esp8266_update = true;
  }

  if ((rx == 'v') || (rx == 'V'))
  {
    PrintfLogsCRLF("\t"CLR_YL"ESP8266 AT  v%u.%u.%u"CLR_DEF, esp.m.version_at.major, esp.m.version_at.minor, esp.m.version_at.patch);
    PrintfLogsCRLF("\t"CLR_YL"ESP8266 SDK v%u.%u.%u"CLR_DEF, esp.m.version_sdk.major, esp.m.version_sdk.minor, esp.m.version_sdk.patch);
    WiFiGetMac();
    WiFiGetInfoAp();
  }

  if ((rx == 'L') || (rx == 'l'))
    IoSystemSetMode(IO_LOGS);
}
/******************************************************************************/




/**
 * @brief          Clear RX queue
 */
void IoSystemClearRxQueue(void)
{
  osMessageQueueReset(uartRxQueueHandle);
}
/******************************************************************************/




/**
 * @brief          Check TX buffer is full
 */
bool IoSystemIsTxBufferFull(void)
{
  return (lwrb_get_free(&io_uart.lwrb_tx) == 0 ? true : false);
}
/******************************************************************************/




/**
 * @brief         Put data to TX ring buffer
 */
bool IoSystemPutDataToTxBuffer(const void* data, size_t len)
{
  if (lwrb_get_free(&io_uart.lwrb_tx) == 0)
    return false;

  return (lwrb_write(&io_uart.lwrb_tx, data, len) > 0 ? true : false);
}
/******************************************************************************/




/**
 * @brief          Read data from RX ring buffer
 */
bool IoSystemReadDataFromRxBuffer(void* data)
{
  if (lwrb_get_free(&io_uart.lwrb_rx) == 0)
    return false;

  return ((lwrb_read(&io_uart.lwrb_rx, data, sizeof(uint8_t))) > 0 ? true : false);
}
/******************************************************************************/



/**
 * @brief          Put data to RX ring buffer
 */
bool IoSystemPutDataToRxBuffer(const void* data, size_t len)
{
  if (lwrb_get_free(&io_uart.lwrb_rx) == 0)
    return false;

  return (lwrb_write(&io_uart.lwrb_rx, data, len) > 0 ? true : false);
}
/******************************************************************************/
