/**
 * \file            esp_ll.c
 * \brief           Low-level communication with ESP device for STM32F405RG-VOLTS using DMA
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


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static osSemaphoreId_t esptxSemaphoreHandle;
static osMutexId_t esptxMutexHandle;

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


static uint8_t initialized;


/******************************************************************************/


/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
void
configure_uart(uint32_t baudrate) {
    if (!initialized)
    {
      LL_USART_InitTypeDef USART_InitStruct;

      LL_GPIO_InitTypeDef GPIO_InitStruct;

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

      NVIC_SetPriority(UART5_IRQn, 5);
      NVIC_EnableIRQ(UART5_IRQn);

      LL_USART_EnableIT_RXNE(UART5);
      LL_USART_EnableIT_ERROR(UART5);

      // Other stuff

      // OLD
//      osSemaphoreDef(esptxSemaphore);
//      esptxSemaphoreHandle = osSemaphoreCreate(osSemaphore(esptxSemaphore), 1);
//      osSemaphoreWait(esptxSemaphoreHandle, 0);
//      osMutexDef(esptxMutex);
//      esptxMutexHandle = osMutexCreate(osMutex(esptxMutex));

      esptxSemaphoreHandle = osSemaphoreNew(1, 1, &esptxSemaphore_attr);
      esptxMutexHandle = osMutexNew(&esptxMutex_attr);

      LL_USART_Enable(UART5);
//
//#if defined(OS_DEBUG)
//      vQueueAddToRegistry(esptxSemaphoreHandle, "esptxSemaphore");
//      vQueueAddToRegistry(esptxMutexHandle, "esptxMutex");
//#endif /* defined(OS_DEBUG) */

    }
    else
    {
        osDelay(10);
        LL_RCC_ClocksTypeDef rcc_clocks;
        LL_RCC_GetSystemClocksFreq(&rcc_clocks);
        LL_USART_Disable(UART5);
        LL_USART_SetBaudRate(UART5, rcc_clocks.PCLK1_Frequency, LL_USART_OVERSAMPLING_16, baudrate);
        LL_USART_Enable(UART5);
    }

    /*
     * Force ESP hardware reset
     * after initialization to make sure device is ready and
     * not in undefined state from previous AT usage
     */
    if (!initialized) {
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
static size_t
send_data(const void* data, size_t len) {
//    if (!esp8266_update)
//    {
//      size_t i;
//      const uint8_t* d = data;
//
//      while (!LL_USART_IsActiveFlag_TXE(UART5));
//      for (i = 0; i < len; i++, d++) {
//          LL_USART_TransmitData8(UART5, *d);
//          if (esp8266_debug)
//            LL_USART_TransmitData8(UART4, *d);
//          while (!LL_USART_IsActiveFlag_TXE(UART5));
//      }
//    }
    if (esp8266_update) return (0);
    volatile uint8_t *buf;
    buf = (volatile uint8_t *) data;
//    if (esp8266_debug)
//    {
//      Printf_LogCONT(CLR_MG"");
//      for (uint8_t i = 0; i < len; i++)
//        Printf_LogCONT("%c", buf[i]);
//      Printf_LogCRLF(""CLR_DEF);
//    }
    osMutexAcquire(esptxMutexHandle, osWaitForever);
    osMutexAcquire(dma174_MutexHandle, osWaitForever);
    DMA_ConfigTxUART5(buf, len);
    LL_USART_EnableDMAReq_TX(UART5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
    osSemaphoreAcquire(esptxSemaphoreHandle, osWaitForever);
    osMutexRelease(dma174_MutexHandle);
    osMutexRelease(esptxMutexHandle);
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
esp_ll_init(esp_ll_t* ll) {
    static uint8_t memory[0x6000];
    esp_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };

    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */

        esp_mem_assignmemory(mem_regions, ESP_ARRAYSIZE(mem_regions));  /* Assign memory for allocations */
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
esp_ll_deinit(esp_ll_t* ll) {
    initialized = 0;                            /* Clear initialized flag */
    return espOK;
}

void UART5_IRQHandler(void)
{
  uint8_t errors = 0;

  if (LL_USART_IsActiveFlag_TC(UART5))
  {
    LL_USART_ClearFlag_TC(UART5);
    LL_USART_DisableIT_TC(UART5);
    osSemaphoreRelease(esptxSemaphoreHandle);
  }

  if (LL_USART_IsActiveFlag_FE(UART5))
  {
    LL_USART_ClearFlag_FE(UART5);
    errors++;
  }

  if (LL_USART_IsActiveFlag_NE(UART5))
  {
    LL_USART_ClearFlag_NE(UART5);
    errors++;
  }

  if (LL_USART_IsActiveFlag_ORE(UART5))
  {
    LL_USART_ClearFlag_ORE(UART5);
    errors++;
  }

  if (LL_USART_IsActiveFlag_RXNE(UART5))
  {
    if (esp8266_update)
      LL_USART_TransmitData8(UART4, LL_USART_ReceiveData8(UART5));
    else
    {
      uint8_t rx = LL_USART_ReceiveData8(UART5);

#if !ESP_CFG_INPUT_USE_PROCESS
      esp_input(&rx, 1);
#endif

#if ESP_CFG_INPUT_USE_PROCESS
      esp_input_process(&rx, 1);
#endif
    }
  }
  else
  {
    if (errors)
    {
      uint8_t rx = LL_USART_ReceiveData8(UART5);
      PROJ_UNUSED(rx);
    }
  }
}

#endif /* !DOXYGEN */

