/**
 * \file            esp_ll.c
 * \brief           Low-level communication with ESP device for STM32F405RG using DMA
 */

/*
 * Copyright (c) 2018 Tilen Majerle
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ESP-AT.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Modified by:     Aleksandr Shabalin
 */


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "esp/esp.h"
#include "esp/esp_mem.h"
#include "esp/esp_input.h"
#include "system/esp_ll.h"
#include "cmsis_os.h"

#if !__DOXYGEN__
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_pwr.h"

#include "log.h"
#include "console.h"
#include "dma.h"
#include "indication.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define ESP_TX_Pin LL_GPIO_PIN_12
#define ESP_TX_GPIO_Port GPIOC
#define ESP_RX_Pin LL_GPIO_PIN_2
#define ESP_RX_GPIO_Port GPIOD
#define ESP_RST_Pin LL_GPIO_PIN_10
#define ESP_RST_GPIO_Port GPIOA
#define ESP_CTRL_Pin LL_GPIO_PIN_15
#define ESP_CTRL_GPIO_Port GPIOA

#define ESP_USART_DMA_RX_BUFF_SIZE 0x1000
#define ESP_MEM_SIZE 0x1000

#if !defined(ESP_USART_RDR_NAME)
#define ESP_USART_RDR_NAME RDR
#endif /* !defined(LWESP_USART_RDR_NAME) */


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static osThreadId_t usart_ll_thread_id;
static osMessageQueueId_t usart_ll_mbox_id;

const osSemaphoreAttr_t esptxSemaphore_attr =
{
    .name = "esptxSemaphore",
    .attr_bits = 0U,
    .cb_mem = NULL,
    .cb_size = 0U
};

const osMutexAttr_t esptxMutex_attr =
{
  .name = "esptxMutex",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0U
};


static uint8_t initialized, is_running;
static uint8_t usart_mem[ESP_USART_DMA_RX_BUFF_SIZE];
static size_t old_pos;

/******************************************************************************/


/**
 * \brief           USART data processing
 */
static void usart_ll_thread(void* arg)
{
  size_t pos;

  ESP_UNUSED(arg);

  for (;;)
  {
    if (esp8266_update)
      continue;

    void* d;

    /* Wait for the event message from DMA or USART */
    osMessageQueueGet(usart_ll_mbox_id, &d, NULL, osWaitForever);

    /* Read data */
    pos = sizeof(usart_mem) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0);

    if (pos != old_pos && is_running) {
      if (pos > old_pos)
      {
        esp_input_process(&usart_mem[old_pos], pos - old_pos);
      }
      else
      {
        esp_input_process(&usart_mem[old_pos], sizeof(usart_mem) - old_pos);
        if (pos > 0)
        {
          esp_input_process(&usart_mem[0], pos);
        }
      }
      old_pos = pos;
    }
  }
}


/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
void configure_uart(uint32_t baudrate)
{
  static LL_USART_InitTypeDef USART_InitStruct;
  static LL_DMA_InitTypeDef DMA_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  if (!initialized)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
    __DSB();

    GPIO_InitStruct.Pin = ESP_RST_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ESP_RST_GPIO_Port, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(ESP_RST_GPIO_Port, ESP_RST_Pin);

    GPIO_InitStruct.Pin = ESP_CTRL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ESP_CTRL_GPIO_Port, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(ESP_CTRL_GPIO_Port, ESP_CTRL_Pin);

    if (esp8266_update)
      LL_GPIO_ResetOutputPin(ESP_CTRL_GPIO_Port, ESP_CTRL_Pin);

    GPIO_InitStruct.Pin = ESP_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(ESP_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ESP_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(ESP_RX_GPIO_Port, &GPIO_InitStruct);

    LL_USART_DeInit(UART5);
    LL_USART_StructInit(&USART_InitStruct);
    USART_InitStruct.BaudRate = baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART5, &USART_InitStruct);
    LL_USART_DisableIT_CTS(UART5);
    LL_USART_ConfigAsyncMode(UART5);

    NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x05, 0x00));
    NVIC_EnableIRQ(UART5_IRQn);

    LL_USART_EnableIT_IDLE(UART5);
    LL_USART_EnableIT_PE(UART5);
    LL_USART_EnableIT_ERROR(UART5);
    LL_USART_EnableDMAReq_RX(UART5);

    is_running = 0;

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    __DSB();

    LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);
    DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&UART5->DR;
    DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)usart_mem;
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_InitStruct.NbData = sizeof(usart_mem);
    DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;

    LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &DMA_InitStruct);

    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_FE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_0);

    NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x05, 0x00));
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    old_pos = 0;
    is_running = 1;

    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    LL_USART_Enable(UART5);
  }
  else
  {
    osDelay(10);
    LL_USART_Disable(UART5);
    USART_InitStruct.BaudRate = baudrate;
    LL_USART_Init(UART5, &USART_InitStruct);
    LL_USART_Enable(UART5);
  }

  if (usart_ll_mbox_id == NULL)
  {
    usart_ll_mbox_id = osMessageQueueNew(10, sizeof(void*), NULL);
  }
  if (usart_ll_thread_id == NULL)
  {
    const osThreadAttr_t attr = {.stack_size = 1024};
    usart_ll_thread_id = osThreadNew(usart_ll_thread, usart_ll_mbox_id, &attr);
  }

/*
 * Force ESP hardware reset
 * after initialization to make sure device is ready and
 * not in undefined state from previous AT usage
 */
  if (!initialized)
  {
//KT2(ON);
    LL_GPIO_ResetOutputPin(ESP_RST_GPIO_Port, ESP_RST_Pin);
    osDelay(100);
    LL_GPIO_SetOutputPin(ESP_RST_GPIO_Port, ESP_RST_Pin);
    osDelay(200);
//KT2(OFF);
  }
}

/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t send_data(const void* data, size_t len)
{
  if (esp8266_update)
    return (0);

  const uint8_t* d = data;
  for (size_t i = 0; i < len; ++i, ++d)
  {
    LL_USART_TransmitData8(UART5, *d);
    while (!LL_USART_IsActiveFlag_TXE(UART5)) {}
  }
  return len;
}

/**
 * \brief           Callback function called from initialization process
 * \note            This function may be called multiple times if AT baudrate is changed from application
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \param[in]       baudrate: Baudrate to use on AT port
 * \return          Member of \ref espr_t enumeration
 */
espr_t
esp_ll_init(esp_ll_t* ll)
{
    static uint8_t memory[0x6000];
    esp_mem_region_t mem_regions[] = {
        {memory, sizeof(memory)}
    };

    if (!initialized) {
        esp_mem_assignmemory(mem_regions, ESP_ARRAYSIZE(mem_regions)); /* Assign memory for allocations */
    }

    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
    }

    configure_uart(ll->uart.baudrate);          /* Initialize UART for communication */
    initialized = 1;
    return espOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \return          \ref espOK on success, member of \ref espr_t enumeration otherwise
 */
espr_t
esp_ll_deinit(esp_ll_t* ll)
{
  if (usart_ll_mbox_id != NULL) {
      osMessageQueueId_t tmp = usart_ll_mbox_id;
      usart_ll_mbox_id = NULL;
      osMessageQueueDelete(tmp);
  }
  if (usart_ll_thread_id != NULL) {
      osThreadId_t tmp = usart_ll_thread_id;
      usart_ll_thread_id = NULL;
      osThreadTerminate(tmp);
  }
  initialized = 0;                    /* Clear initialized flag */
  PROJ_UNUSED(ll);
  return espOK;
}

/**
 * \brief           UART5 IRQ handler
 */
void UART5_IRQHandler(void)
{
  LL_USART_ClearFlag_IDLE(UART5);
  LL_USART_ClearFlag_PE(UART5);
  LL_USART_ClearFlag_FE(UART5);
  LL_USART_ClearFlag_ORE(UART5);
  LL_USART_ClearFlag_NE(UART5);
  IndicationLedYellowBlink(3);

  if (esp8266_update)
  {
    LL_USART_TransmitData8(UART4, LL_USART_ReceiveData8(UART5));
  }
  else
  {
    if (usart_ll_mbox_id != NULL)
    {
      void* d = (void*)1;
      osMessageQueuePut(usart_ll_mbox_id, &d, 0, 0);
    }
  }
}

/**
 * \brief           UART DMA stream/channel handler
 */
void DMA1_Stream0_IRQHandler(void)
{
  LL_DMA_ClearFlag_TC7(DMA1);
  LL_DMA_ClearFlag_HT7(DMA1);
  IndicationLedGreenBlink(3);

  if (esp8266_update)
  {
    LL_USART_TransmitData8(UART4, LL_USART_ReceiveData8(UART5));
  }
  else
  {
    if (usart_ll_mbox_id != NULL)
    {
      void* d = (void*)1;
      osMessageQueuePut(usart_ll_mbox_id, &d, 0, 0);
    }
  }
}

#endif /* !DOXYGEN */
