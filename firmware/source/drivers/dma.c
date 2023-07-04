/**
 ******************************************************************************
 * @file           : dma.c
 * @author         : Konstantin Soloviev
 * @author         : Dmitry Karasev        <karasev@voltsbattery.com>
 * @brief          : This is the DMA part of the I2C1 and USART5 drivers.
 *                   I2C1 is used for FRAM. USART5 is used by the
 *                   ESP8266 module.
 ******************************************************************************
 * ----------------- Copyright (c) 2020 VOLTS Battery LLC ------------------- *
 ******************************************************************************
 * This module is a confidential and proprietary property of VOLTS Battery
 * and possession or use of this module requires written permission
 * of VOLTS Battery.
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
//#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_usart.h"

#include "lib/esp/system/esp_ll.h"

#include "i2c1.h"

#include "dma.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/



/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
volatile bool i2c1_dma_tx;

osSemaphoreId_t dma174_TxSemaphoreHandle;
osMutexId_t dma174_MutexHandle;

const osSemaphoreAttr_t dma174Semaphore_attr =
{
    .name = "dma174Semaphore",
    .attr_bits = 0U,
    .cb_mem = NULL,
    .cb_size = 0U
};

const osMutexAttr_t dma174Mutex_attr =
{
  .name = "dma174Mutex",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0U
};


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/



/******************************************************************************/




void DMA_Init174(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  __DSB();

  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_7, LL_DMA_CHANNEL_1);
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_STREAM_7,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);

  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  i2c1_dma_tx = false;

  dma174_MutexHandle = osMutexNew(&dma174Mutex_attr);
  dma174_TxSemaphoreHandle = osSemaphoreNew(1, 1, &dma174Semaphore_attr);

//  osSemaphoreWait(dma174_TxSemaphoreHandle, 0);

#if defined(OS_DEBUG)
  vQueueAddToRegistry(dma174_MutexHandle, "dma174Mutex");
  vQueueAddToRegistry(dma174_TxSemaphoreHandle, "dma174TxSemaphore");
#endif /* defined(OS_DEBUG) */
}
/******************************************************************************/




void DMA_ConfigTxI2C1(volatile void *buf, uint16_t len)
{
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_7, LL_DMA_CHANNEL_1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_7,
                         (uint32_t) buf,
                         LL_I2C_DMA_GetRegAddr(I2C1),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, len);
}
/******************************************************************************/




void DMA_ConfigTxUART5(volatile void *buf, uint16_t len)
{
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_7,
                         (uint32_t) buf,
                         LL_USART_DMA_GetRegAddr(UART5),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, len);
//  LL_USART_EnableDMAReq_TX(UART5);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
//  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
}
/******************************************************************************/




//void DMA1_Stream7_IRQHandler(void)
//{
//  LL_DMA_ClearFlag_TC7(DMA1);
//  if (i2c1_dma_tx)
//  {
//    if (i2c1_address_sended)
//      LL_I2C_DisableDMAReq_TX(I2C1);
//    else
//    {
//      if (i2c1_mode_write)
//      {
//        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
//        DMA_ConfigTxI2C1((void *) i2c1_buffer, i2c1_length);
//      }
//      else
//      {
//        i2c1_repeated_start = true;
//        LL_I2C_DisableDMAReq_TX(I2C1);
//        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
//        LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_7);
//        LL_I2C_GenerateStartCondition(I2C1);
//      }
//    }
//  }
//  else
//  {
//    LL_USART_DisableDMAReq_TX(UART5);
//    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
//    LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_7);
//    LL_USART_EnableIT_TC(UART5);
//  }
//}
/******************************************************************************/
