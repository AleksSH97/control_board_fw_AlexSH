/**
 ******************************************************************************
 * @file           : i2c1.c
 * @author         : Konstantin Soloviev
 * @author         : Dmitry Karasev        <karasev@voltsbattery.com>
 * @brief          : I2C1 peripheral driver. I2C1 is used for FRAM
 *                   and external RTC.
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
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"

#include "dma.h"
#include "i2c1.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define I2C_SCL_Pin                   LL_GPIO_PIN_6
#define I2C_SCL_GPIO_Port             GPIOB
#define I2C_SDA_Pin                   LL_GPIO_PIN_7
#define I2C_SDA_GPIO_Port             GPIOB

#define I2C_REQUEST_WRITE             (0x0000)
#define I2C_REQUEST_READ              (0x0001)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static volatile uint8_t i2c1_device;
static volatile bool i2c1_addr_word;
static volatile uint8_t i2c1_address[2];
volatile bool i2c1_mode_write;
volatile bool i2c1_address_sended;
volatile bool i2c1_repeated_start;
static volatile uint8_t i2c1_byte;
volatile uint8_t *i2c1_buffer;
volatile uint16_t i2c1_length;
static volatile bool i2c1_result;

static osMutexId i2c1MutexHandle;
static osSemaphoreId i2c1SemaphoreHandle;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static void prvStartTransaction(uint8_t device, bool addr_word, uint16_t address);
static bool prvWait(uint8_t ms);
static void prvConfigDmaRx(volatile void *buf, uint16_t len);
static void prvStopTx(void);
static void prvStopRx(void);


/******************************************************************************/




void I2C1_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = I2C_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
  LL_GPIO_SetOutputPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  __DSB();

  i2c1_device = 0;
  i2c1_addr_word = false;
  i2c1_address[0] = 0;
  i2c1_address[1] = 0;
  i2c1_mode_write = true;
  i2c1_address_sended = false;
  i2c1_repeated_start = false;
  i2c1_byte = 0;
  i2c1_buffer = NULL;
  i2c1_length = 0;
  i2c1_result = false;

  LL_I2C_InitTypeDef I2C_InitStruct;

  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  LL_I2C_SetOwnAddress2(I2C1, 0);
  LL_I2C_DisableOwnAddress2(I2C1);

  LL_I2C_DisableGeneralCall(I2C1);

  LL_I2C_EnableClockStretching(I2C1);

  LL_I2C_Enable(I2C1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  __DSB();

  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_1);
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_STREAM_5,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);

  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  osMutexDef(i2c1Mutex);
  i2c1MutexHandle = osMutexCreate(osMutex(i2c1Mutex));

  osSemaphoreDef(i2c1Semaphore);
  i2c1SemaphoreHandle = osSemaphoreCreate(osSemaphore(i2c1Semaphore), 1);
  osSemaphoreWait(i2c1SemaphoreHandle, 0);

#if defined(OS_DEBUG)
  vQueueAddToRegistry(i2c1MutexHandle, "i2c1Mutex");
  vQueueAddToRegistry(i2c1SemaphoreHandle, "i2c1Semaphore");
#endif /* defined(OS_DEBUG) */
}
/******************************************************************************/




bool I2C1_WriteByte(uint8_t device, bool addr_word, uint16_t address, uint8_t b)
{
  osMutexWait(i2c1MutexHandle, osWaitForever);
  osMutexWait(dma174_MutexHandle, osWaitForever);
  i2c1_dma_tx = true;
  i2c1_mode_write = true;
  i2c1_byte = b;
  i2c1_buffer = &i2c1_byte;
  i2c1_length = 1;

  prvStartTransaction(device, addr_word, address);
  return (prvWait(2));
}
/******************************************************************************/




bool I2C1_WriteBuffer(uint8_t device, bool addr_word, uint16_t address, void *buffer, uint16_t length)
{
  osMutexWait(i2c1MutexHandle, osWaitForever);
  osMutexWait(dma174_MutexHandle, osWaitForever);
  i2c1_dma_tx = true;
  i2c1_mode_write = true;
  i2c1_buffer = buffer;
  i2c1_length = length;

  prvStartTransaction(device, addr_word, address);
  return (prvWait(5 + (3 * (length / 100))));
}
/******************************************************************************/




bool I2C1_ReadByte(uint8_t device, bool addr_word, uint16_t address, uint8_t *b)
{
  osMutexWait(i2c1MutexHandle, osWaitForever);
  osMutexWait(dma174_MutexHandle, osWaitForever);
  i2c1_dma_tx = true;
  i2c1_mode_write = false;
  i2c1_buffer = b;
  i2c1_length = 1;

  prvStartTransaction(device, addr_word, address);
  return (prvWait(5));
}
/******************************************************************************/




bool I2C1_ReadBuffer(uint8_t device, bool addr_word, uint16_t address, void *buffer, uint16_t length)
{
  osMutexWait(i2c1MutexHandle, osWaitForever);
  osMutexWait(dma174_MutexHandle, osWaitForever);
  i2c1_dma_tx = true;
  i2c1_mode_write = false;
  i2c1_buffer = buffer;
  i2c1_length = length;

  prvStartTransaction(device, addr_word, address);
  return (prvWait(5 + (3 * (length / 100))));
}
/******************************************************************************/




static void prvStartTransaction(uint8_t device, bool addr_word, uint16_t address)
{
  i2c1_device = device;
  i2c1_address[0] = address >> 8;
  i2c1_address[1] = address;
  i2c1_repeated_start = false;
  i2c1_address_sended = false;

  if (i2c1_mode_write)
  {
    if (i2c1_length)
    {
      DMA_ConfigTxI2C1((void *) &i2c1_address[addr_word ? 0 : 1], addr_word ? 2 : 1);
      LL_I2C_EnableDMAReq_TX(I2C1);
      LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
    }
  }
  else
  {
    if (i2c1_length)
    {
      DMA_ConfigTxI2C1((void *) &i2c1_address[addr_word ? 0 : 1], addr_word ? 3 : 1);  // address length +1, if > 1
      LL_I2C_EnableDMAReq_TX(I2C1);
      LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
    }
  }

  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);

  LL_I2C_GenerateStartCondition(I2C1);
}
/******************************************************************************/




static bool prvWait(uint8_t ms)
{
  i2c1_result = false;
  int32_t ok = osSemaphoreWait(i2c1SemaphoreHandle, ms);
  bool ret = (ok == osOK) ? i2c1_result : false;
  i2c1_dma_tx = false;
  osMutexRelease(dma174_MutexHandle);
  osMutexRelease(i2c1MutexHandle);
  return (ret);
}
/******************************************************************************/




static void prvConfigDmaRx(volatile void *buf, uint16_t len)
{
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_5,
                         LL_I2C_DMA_GetRegAddr(I2C1),
                         (uint32_t) buf,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, len);
}
/******************************************************************************/




static void prvStopTx(void)
{
  LL_I2C_DisableDMAReq_TX(I2C1);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_7);
  LL_I2C_DisableIT_EVT(I2C1);
  LL_I2C_DisableIT_ERR(I2C1);
  LL_I2C_GenerateStopCondition(I2C1);
}
/******************************************************************************/




static void prvStopRx(void)
{
  LL_I2C_DisableDMAReq_RX(I2C1);
  LL_I2C_DisableLastDMA(I2C1);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_5);
  LL_I2C_DisableIT_EVT(I2C1);
  LL_I2C_DisableIT_ERR(I2C1);
  LL_I2C_GenerateStopCondition(I2C1);
}
/******************************************************************************/




//void I2C1_EV_IRQHandler(void)
//{
//  if (i2c1_mode_write)
//  {
//    if (LL_I2C_IsActiveFlag_SB(I2C1))
//    {
//      LL_I2C_TransmitData8(I2C1, i2c1_device | I2C_REQUEST_WRITE);
//      return;
//    }
//    if (LL_I2C_IsActiveFlag_ADDR(I2C1))
//    {
//      LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
//      LL_I2C_ClearFlag_ADDR(I2C1);
//      return;
//    }
//    if (LL_I2C_IsActiveFlag_TXE(I2C1))
//    {
//      if (!i2c1_length)
//      {
//        prvStopTx();
//        i2c1_result = true;
//        osSemaphoreRelease(i2c1SemaphoreHandle);
//        return;
//      }
//    }
//    if (LL_I2C_IsActiveFlag_BTF(I2C1))
//    {
//      if (i2c1_address_sended)
//      {
//        prvStopTx();
//        i2c1_result = true;
//        osSemaphoreRelease(i2c1SemaphoreHandle);
//      }
//      else
//      {
//        i2c1_address_sended = true;
//        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
//      }
//      return;
//    }
//  }
//  else
//  {
//    if (LL_I2C_IsActiveFlag_SB(I2C1))
//    {
//      if (i2c1_repeated_start)
//        LL_I2C_TransmitData8(I2C1, i2c1_device | I2C_REQUEST_READ);
//      else
//        LL_I2C_TransmitData8(I2C1, i2c1_device | I2C_REQUEST_WRITE);
//      return;
//    }
//    if (LL_I2C_IsActiveFlag_ADDR(I2C1))
//    {
//      if (i2c1_repeated_start)
//      {
//        prvConfigDmaRx((volatile void *) i2c1_buffer, i2c1_length);
//        if (i2c1_length >= 2)
//        {
//          LL_I2C_EnableLastDMA(I2C1);
//          LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
//          LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
//          LL_I2C_EnableDMAReq_RX(I2C1);
//          LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
//          LL_I2C_ClearFlag_ADDR(I2C1);
//          return;
//        }
//        else
//        {
//          LL_I2C_EnableLastDMA(I2C1);
//          LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
//          LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
//          LL_I2C_EnableDMAReq_RX(I2C1);
//          LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
//          LL_I2C_ClearFlag_ADDR(I2C1);
//          return;
//        }
//      }
//      else
//      {
//        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
//        LL_I2C_ClearFlag_ADDR(I2C1);
//        return;
//      }
//    }
//  }
//}
///******************************************************************************/
//
//
//
//
//void I2C1_ER_IRQHandler(void)
//{
//  if (LL_I2C_IsActiveFlag_AF(I2C1))
//  {
//    LL_I2C_ClearFlag_AF(I2C1);
//    prvStopTx();
//    i2c1_result = false;
//    osSemaphoreRelease(i2c1SemaphoreHandle);
//  }
//  else
//  {
//    prvStopTx();
//    prvStopRx();
//    i2c1_result = false;
//    osSemaphoreRelease(i2c1SemaphoreHandle);
//  }
//}
///******************************************************************************/
//
//
//
//
//void DMA1_Stream5_IRQHandler(void)
//{
//  LL_DMA_ClearFlag_TC5(DMA1);
//  prvStopRx();
//  i2c1_result = true;
//  osSemaphoreRelease(i2c1SemaphoreHandle);
//}
///******************************************************************************/
