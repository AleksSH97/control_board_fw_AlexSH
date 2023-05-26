/**
 ******************************************************************************
 * @file           : rtc_i2c.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : RTC I2C usage file
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"

#include "rtc.h"
#include "log.h"
#include "rtc_i2c.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define I2C_SCL_Pin                   LL_GPIO_PIN_6
#define I2C_SCL_GPIO_Port             GPIOB
#define I2C_SDA_Pin                   LL_GPIO_PIN_7
#define I2C_SDA_GPIO_Port             GPIOB

#define I2C_REQUEST_WRITE             (0x00)
#define I2C_REQUEST_READ              (0x01)

#define I2C_BUFFER_SIZE               (8u)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static osMutexId_t RtcI2cMutexHandle;
static osSemaphoreId_t RtcI2cSemphoreHandle;

const osMutexAttr_t RtcI2cMutex_attr = {
  .name = "RtcI2cMutex",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0U
};

const osSemaphoreAttr_t RtcI2cSemaphore_attr = {
    .name = "RtcI2cSemaphore",
    .attr_bits = 0U,
    .cb_mem = NULL,
    .cb_size = 0U
};

typedef struct
{
  uint8_t     device;
  uint8_t     address;
  uint8_t     byte;
  uint8_t     *buffer;
  uint8_t    length;
  uint8_t    num_bytes;

  volatile bool        addr_word;
  volatile bool        mode_write;
  volatile bool        address_sended;
  volatile bool        repeated_start;
  volatile bool        result;
  volatile bool        is_busy;
} RTC_I2C_t;

RTC_I2C_t rtc_i2c;
uint32_t systicks;
RTC_DATE_t rtc_date;
RTC_TIME_t rtc_time;
uint8_t i2c_buffer[I2C_BUFFER_SIZE];


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
uint8_t prvRtcGPIOInit(void);
uint32_t prvGetTicks(void);
uint8_t prvStartTransaction(uint8_t device, uint8_t address);
void prvResetSysTicks(void);
void prvStopTx(void);
void prvStopRx(void);


/******************************************************************************/


/**
 * @brief          RTC I2C init
 */
uint8_t RtcI2cInit(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = I2C_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
  LL_GPIO_SetOutputPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  __DSB();

  //To prevent SB flag
  //(void) I2C1->SR2;

  rtc_i2c.device = 0;
  rtc_i2c.addr_word = false;
  rtc_i2c.address = 0;
  rtc_i2c.mode_write = true;
  rtc_i2c.address_sended = false;
  rtc_i2c.repeated_start = false;
  rtc_i2c.num_bytes = 0;
  rtc_i2c.byte = 0;
  rtc_i2c.buffer = NULL;
  rtc_i2c.length = 0;
  rtc_i2c.result = false;

  for (uint16_t i = 0; i < I2C_BUFFER_SIZE; i++)
  {
    i2c_buffer[i] = 0;
  }

  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x05, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x05, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

  if (LL_I2C_Init(I2C1, &I2C_InitStruct) != SUCCESS)
    return RTC_INIT_ERROR;

  LL_I2C_SetOwnAddress2(I2C1, 0);
  LL_I2C_Enable(I2C1);

  RtcI2cMutexHandle = osMutexNew(&RtcI2cMutex_attr);
  RtcI2cSemphoreHandle = osSemaphoreNew(1, 1, &RtcI2cSemaphore_attr);

  if (RtcI2cMutexHandle == NULL)
    return RTC_INIT_ERROR;

  if (RtcI2cSemphoreHandle == NULL)
    return RTC_INIT_ERROR;

  if (prvRtcGPIOInit() != RTC_OK)
    return RTC_INIT_ERROR;

  systicks = 0UL;

  return RTC_OK;
}
/******************************************************************************/




uint8_t RtcI2cGetDate(RTC_DATE_t *date)
{
  uint8_t rtc_date_buffer[4];
  uint8_t rc;

  //rc = osSemaphoreAcquire(RtcI2cSemphoreHandle, osWaitForever);
//  if (rc != osOK)
//    return RTC_RECEIVE_ERROR;

  rc = RtcI2cReadBufferInterrupt(RTC_HW_ADDRESS, RTC_REG_DAY, rtc_date_buffer, 4);

  //rc = RtcI2cReadByte(RTC_HW_ADDRESS, RTC_REG_DAY, rtc_date_buffer, 4);

  if (rc != RTC_OK)
  {
    //osSemaphoreRelease(RtcI2cSemphoreHandle);
    return rc;
  }

  date->day   = rtc_date_buffer[0];
  date->date  = ((rtc_date_buffer[1] >> 4) * 10) + (rtc_date_buffer[1] & 0x0F);
  date->month = ((rtc_date_buffer[2] & 0x10) ? 10 : 0) + (rtc_date_buffer[2] & 0x0F);
  date->year  = 2000 + ((rtc_date_buffer[2] & 0x80) ? 100 : 0) + ((rtc_date_buffer[3] >> 4) * 10) + (rtc_date_buffer[3] & 0x0F);

//  osDelay(100);

  //osSemaphoreRelease(RtcI2cSemphoreHandle);

  return RTC_OK;
}
/******************************************************************************/




uint8_t RtcI2cGetTime(RTC_TIME_t *time)
{
  uint8_t rtc_time_buffer[3];
  uint8_t rc;

  rc = osSemaphoreAcquire(RtcI2cSemphoreHandle, osWaitForever);
  if (rc != osOK)
    return RTC_RECEIVE_ERROR;

  rc = RtcI2cReadBufferInterrupt(RTC_HW_ADDRESS, RTC_REG_DAY, rtc_time_buffer, 3);

  if (rc != RTC_OK)
  {
    osSemaphoreRelease(RtcI2cSemphoreHandle);
    return rc;
  }

  // Convert the time from binary to human-readable format.
  time->seconds = ((rtc_time_buffer[0] >> 4) * 10) + (rtc_time_buffer[0] & 0x0F);
  time->minutes = ((rtc_time_buffer[1] >> 4) * 10) + (rtc_time_buffer[1] & 0x0F);

  // Check if the 24-hour mode is enabled.
  if (rtc_time_buffer[2] & 0x40)
    time->hours = ((rtc_time_buffer[2] & 0x10) ? 10 : 0) + (rtc_time_buffer[2] & 0x0F);
  else
    time->hours = ((rtc_time_buffer[2] & 0x20) ? 20 : 0) + ((rtc_time_buffer[2] & 0x10) ? 10 : 0) + (rtc_time_buffer[2] & 0x0F);

  // Get the milliseconds.
  time->ms = prvGetTicks();

  // Cap the milliseconds at 999.
  if (time->ms > 999)
    time->ms = 999;

  osSemaphoreRelease(RtcI2cSemphoreHandle);

  return RTC_OK;
}
/******************************************************************************/




uint8_t RtcI2cSetDate(RTC_DATE_t *date)
{
  uint8_t write_buffer[3];

  osSemaphoreAcquire(RtcI2cSemphoreHandle, osWaitForever);

  write_buffer[0] = ((date->date  / 10) << 4) | ((date->date  % 10) & 0x0F);
  write_buffer[1] = ((date->month / 10) << 4) | ((date->month % 10) & 0x0F);
  write_buffer[2] = ((date->year  / 10) << 4) | ((date->year  % 10) & 0x0F);

  if ((RtcI2cWriteBufferInterrupt(RTC_HW_ADDRESS, RTC_REG_DATE, write_buffer, 3)) != RTC_OK)
    return RTC_TRANSMIT_ERROR;

  osSemaphoreRelease(RtcI2cSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




uint8_t RtcI2cSetTime(RTC_TIME_t *time)
{
  uint8_t write_buffer[3];

  osSemaphoreAcquire(RtcI2cSemphoreHandle, osWaitForever);

  write_buffer[0] = ((time->seconds / 10) << 4) | ((time->seconds % 10) & 0x0F);
  write_buffer[1] = ((time->minutes / 10) << 4) | ((time->minutes % 10) & 0x0F);
  write_buffer[2] = ((time->hours   / 10) << 4) | ((time->hours   % 10) & 0x0F);

  if ((RtcI2cWriteBufferInterrupt(RTC_HW_ADDRESS, RTC_REG_SECONDS, write_buffer, 3)) != RTC_OK)
    return RTC_TRANSMIT_ERROR;

  osSemaphoreRelease(RtcI2cSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC I2C read byte with interrupt
 */
uint8_t RtcI2cReadByteInterrupt(uint8_t device, uint8_t address, void *buffer, uint8_t length)
{
  uint8_t res = RTC_OK;

  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  rtc_i2c.mode_write = false;

//  for (uint8_t i = 0; i < length; i++)
//  {
//    rtc_i2c.buffer[i] = buffer[i];
//  }

  rtc_i2c.length = length;

  res = prvStartTransaction(device, address);

  if (res != RTC_OK)
    return RTC_RECEIVE_ERROR;

  osMutexRelease(RtcI2cMutexHandle);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC I2C read buffer with interrupt
 */
uint8_t RtcI2cReadBufferInterrupt(uint8_t device, uint8_t address, uint8_t *buffer, uint8_t length)
{
  uint8_t res = RTC_OK;

  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  rtc_i2c.mode_write = false;
  rtc_i2c.is_busy = false;
  rtc_i2c.length = length;
  rtc_i2c.buffer = buffer;

  res = prvStartTransaction(device, address);

  if (res != RTC_OK)
    return RTC_RECEIVE_ERROR;

//  for (uint8_t i = 0; i < length; i++)
//  {
//    buffer[i] = i2c_buffer[i];
//  }

  osMutexRelease(RtcI2cMutexHandle);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC I2C write byte with interrupt
 */
uint8_t RtcI2cWriteByteInterrupt(uint8_t device, uint8_t address, uint8_t b)
{
  uint8_t res = RTC_OK;

  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  rtc_i2c.mode_write = true;
  rtc_i2c.byte = b;
  rtc_i2c.buffer[0] = rtc_i2c.byte;
  rtc_i2c.length = 1;

  res = prvStartTransaction(device, address);

  if (res != RTC_OK)
    return RTC_RECEIVE_ERROR;

  osMutexRelease(RtcI2cMutexHandle);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC I2C write byte with interrupt
 */
uint8_t RtcI2cWriteBufferInterrupt(uint8_t device, uint8_t address, uint8_t *buffer, uint8_t length)
{
  uint8_t res = RTC_OK;

  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  rtc_i2c.mode_write = true;
  rtc_i2c.is_busy = false;
  rtc_i2c.length = length;
  rtc_i2c.buffer = buffer;

//  for (uint8_t i = 0; i < length; i++)
//  {
//    i2c_buffer[i] = buffer[i];
//  }

  res = prvStartTransaction(device, address);

  if (res != RTC_OK)
    return RTC_RECEIVE_ERROR;

  osMutexRelease(RtcI2cMutexHandle);

  return res;
}
/******************************************************************************/



/**
 * @brief          RTC I2C read byte
 */
uint8_t RtcI2cReadByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes)
{
  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_SB(I2C1))
  {
    IndicationLedYellow();
  }

  (void) I2C1->SR1;

  LL_I2C_TransmitData8(I2C1, device | I2C_REQUEST_WRITE);
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1)){};
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, address);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)){};

  LL_I2C_GenerateStartCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_SB(I2C1))
  {
    IndicationLedYellow();
  }

  (void) I2C1->SR1;

  LL_I2C_TransmitData8(I2C1, device | I2C_REQUEST_READ);

  while (!LL_I2C_IsActiveFlag_ADDR(I2C1)){};
  LL_I2C_ClearFlag_ADDR(I2C1);

  for(uint16_t i = 0; i < num_bytes; i++)
  {
    if(i < (num_bytes - 1))
    {
      while(!LL_I2C_IsActiveFlag_RXNE(I2C1)){};
      buffer[i] = LL_I2C_ReceiveData8(I2C1);
    }
    else
    {
      LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
      LL_I2C_GenerateStopCondition(I2C1);
      while(!LL_I2C_IsActiveFlag_RXNE(I2C1)){};
      buffer[i] = LL_I2C_ReceiveData8(I2C1);
    }
  }

  IndicationLedGreen();

  osMutexRelease(RtcI2cMutexHandle);

  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC I2C write byte
 */
uint8_t RtcI2cWriteByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes)
{
  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_SB(I2C1)){};

  (void) I2C1->SR1;

  LL_I2C_TransmitData8(I2C1, device | I2C_REQUEST_WRITE);

  while(!LL_I2C_IsActiveFlag_ADDR(I2C1)){};
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, (uint8_t) address);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)){};

  for (uint16_t i = 0; i < num_bytes; i++)
  {
    LL_I2C_TransmitData8(I2C1, buffer[i]);
    while(!LL_I2C_IsActiveFlag_TXE(I2C1)){};
  }
  LL_I2C_GenerateStopCondition(I2C1);

  IndicationLedGreen();

  osMutexRelease(RtcI2cMutexHandle);

  return RTC_OK;
}
/******************************************************************************/




uint8_t prvStartTransaction(uint8_t device, uint8_t address)
{
  rtc_i2c.device = device;
  rtc_i2c.address = address;
  rtc_i2c.repeated_start = false;
  rtc_i2c.address_sended = false;
  rtc_i2c.num_bytes = 0;

  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);

  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C1);

  return RTC_OK;
}
/******************************************************************************/




uint32_t prvGetTicks(void)
{
  return (xTaskGetTickCount() - systicks);
}
/******************************************************************************/




void prvResetSysTicks(void)
{
  systicks = xTaskGetTickCount();
}
/******************************************************************************/




void prvStopTx(void)
{
  LL_I2C_DisableIT_EVT(I2C1);
  LL_I2C_DisableIT_ERR(I2C1);
  LL_I2C_GenerateStopCondition(I2C1);

  rtc_i2c.mode_write = false;
  rtc_i2c.is_busy = false;
}
/******************************************************************************/




void prvStopRx(void)
{
  LL_I2C_DisableIT_EVT(I2C1);
  LL_I2C_DisableIT_ERR(I2C1);
  LL_I2C_GenerateStopCondition(I2C1);

  rtc_i2c.mode_write = false;
  rtc_i2c.is_busy = false;
}
/******************************************************************************/



uint8_t prvRtcGPIOInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  LL_GPIO_ResetOutputPin(GPIOC, RTCRST_Pin);

  GPIO_InitStruct.Pin = RTCRST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  if(LL_GPIO_Init(RTCRST_GPIO_Port, &GPIO_InitStruct) != SUCCESS)
    return RTC_INIT_ERROR;

  GPIO_InitStruct.Pin = RTCINT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  if(LL_GPIO_Init(RTCINT_GPIO_Port, &GPIO_InitStruct) != SUCCESS)
    return RTC_INIT_ERROR;

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE1);

  EXTI_InitStruct.Line_0_31   = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode        = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger     = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  return RTC_OK;
}
/******************************************************************************/




///**
// * @brief          RTC I2C event IRQ
// */
//void I2C1_EV_IRQHandler(void)
//{
//  if (rtc_i2c.mode_write)
//  {
//    //Start bit generated
//    if (LL_I2C_IsActiveFlag_SB(I2C1))
//    {
//      //Sending device I2C ID and requesting WRITING
//      LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_WRITE);
//    }
//    if (LL_I2C_IsActiveFlag_ADDR(I2C1))
//    {
//      LL_I2C_ClearFlag_ADDR(I2C1);
//      LL_I2C_TransmitData8(I2C1, rtc_i2c.address);
//    }
//    if (LL_I2C_IsActiveFlag_TXE(I2C1))
//    {
//      if (rtc_i2c.num_bytes != rtc_i2c.length)
//      {
//        LL_I2C_TransmitData8(I2C1, i2c_buffer[rtc_i2c.num_bytes]);
//        rtc_i2c.num_bytes++;
//      }
//    }
//    if (LL_I2C_IsActiveFlag_BTF(I2C1))
//    {
//      if (rtc_i2c.num_bytes == rtc_i2c.length)
//      {
//        prvStopTx();
//      }
//    }
//  }
//  else
//  {
//    if (LL_I2C_IsActiveFlag_SB(I2C1))
//    {
//      if (rtc_i2c.repeated_start)
//        LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_READ);
//      else
//      {
//        LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_WRITE);
//      }
//    }
//    if (LL_I2C_IsActiveFlag_ADDR(I2C1))
//    {
//      if (rtc_i2c.repeated_start)
//      {
//         LL_I2C_ClearFlag_ADDR(I2C1);
//         LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
//      }
//      else
//      {
//        LL_I2C_ClearFlag_ADDR(I2C1);
//        LL_I2C_TransmitData8(I2C1, rtc_i2c.address);
//        LL_I2C_GenerateStartCondition(I2C1);
//        rtc_i2c.repeated_start = true;
//      }
//    }
//    if (LL_I2C_IsActiveFlag_RXNE(I2C1))
//    {
//      if (!rtc_i2c.mode_write)
//      {
//        if (rtc_i2c.num_bytes < (rtc_i2c.length - 1))
//        {
//          i2c_buffer[rtc_i2c.num_bytes] = LL_I2C_ReceiveData8(I2C1);
//          ++rtc_i2c.num_bytes;
//        }
//        else
//        {
//          LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
//          i2c_buffer[(rtc_i2c.num_bytes)] = LL_I2C_ReceiveData8(I2C1);
//          prvStopRx();
//        }
//      }
//    }
//  }
//}
///******************************************************************************/



/**
 * @brief          RTC I2C event IRQ
 */
void I2C1_EV_IRQHandler(void)
{
  if (LL_I2C_IsActiveFlag_SB(I2C1) && LL_I2C_IsEnabledIT_EVT(I2C1))
  {
    if (rtc_i2c.mode_write)
    {
      LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_WRITE);
    }
    else
    {
      if (rtc_i2c.repeated_start)
      {
        LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_READ);
      }
      else
      {
        LL_I2C_TransmitData8(I2C1, rtc_i2c.device | I2C_REQUEST_WRITE);
      }
    }
  }
  if (LL_I2C_IsActiveFlag_ADDR(I2C1) && LL_I2C_IsEnabledIT_EVT(I2C1))
  {
    if (rtc_i2c.mode_write)
    {
      LL_I2C_ClearFlag_ADDR(I2C1);
      LL_I2C_TransmitData8(I2C1, rtc_i2c.address);
    }
    else
    {
      if (rtc_i2c.length == 1)
      {
        LL_I2C_ClearFlag_ADDR(I2C1);
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
      }
      else
      {
        LL_I2C_ClearFlag_ADDR(I2C1);
        LL_I2C_TransmitData8(I2C1, rtc_i2c.address);
        LL_I2C_GenerateStartCondition(I2C1);
        rtc_i2c.repeated_start = true;
      }
    }
  }
  if (LL_I2C_IsActiveFlag_BTF(I2C1) && LL_I2C_IsEnabledIT_EVT(I2C1))
  {
    if (rtc_i2c.mode_write)
    {
      if (READ_BIT(I2C1->SR1, I2C_SR1_TXE))
      {
        if ((rtc_i2c.length - rtc_i2c.num_bytes) == 0)
        {
          if (!rtc_i2c.repeated_start)
            prvStopTx();
        }
      }
    }
    else
    {
      ;
    }
  }
  if (LL_I2C_IsActiveFlag_TXE(I2C1) && LL_I2C_IsEnabledIT_EVT(I2C1))
  {
    if (rtc_i2c.mode_write)
    {
      if ((rtc_i2c.length - rtc_i2c.num_bytes) > 0)
      {
        LL_I2C_TransmitData8(I2C1, rtc_i2c.buffer[rtc_i2c.num_bytes]);
        rtc_i2c.num_bytes++;
      }
    }
  }
  if (LL_I2C_IsActiveFlag_RXNE(I2C1) && LL_I2C_IsEnabledIT_EVT(I2C1))
  {
    if (!rtc_i2c.mode_write)
    {
      if (rtc_i2c.length == 1)
      {
        *rtc_i2c.buffer = LL_I2C_ReceiveData8(I2C1);
        rtc_i2c.length--;
        rtc_i2c.buffer++;
      }
      if (rtc_i2c.length > 1)
      {
        if (rtc_i2c.length == 2)
        {
          LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
        }
        *rtc_i2c.buffer = LL_I2C_ReceiveData8(I2C1);
        rtc_i2c.length--;
        rtc_i2c.buffer++;
      }
      if (rtc_i2c.length == 0)
      {
        prvStopRx();
      }
//      if ((rtc_i2c.length - rtc_i2c.num_bytes) == 1)
//      {
//        i2c_buffer[(rtc_i2c.num_bytes)] = LL_I2C_ReceiveData8(I2C1);
//        rtc_i2c.num_bytes++;
//      }
//      if ((rtc_i2c.length - rtc_i2c.num_bytes) > 1)
//      {
//        if ((rtc_i2c.length - rtc_i2c.num_bytes) == 2)
//        {
//          LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
//        }
//        i2c_buffer[(rtc_i2c.num_bytes)] = LL_I2C_ReceiveData8(I2C1);
//        rtc_i2c.num_bytes++;
//      }
//      if ((rtc_i2c.length - rtc_i2c.num_bytes) == 0)
//      {
//        prvStopRx();
//      }
    }
  }
}
/******************************************************************************/




/**
 * @brief          RTC I2C error IRQ
 */
void I2C1_ER_IRQHandler(void)
{
  if (LL_I2C_IsActiveFlag_AF(I2C1))
  {
    IndicationLedError();
    LL_I2C_ClearFlag_AF(I2C1);
    osSemaphoreRelease(RtcI2cSemphoreHandle);
  }
  if (LL_I2C_IsActiveFlag_BERR(I2C1))
  {
    IndicationLedError();
    LL_I2C_ClearFlag_BERR(I2C1);
    osSemaphoreRelease(RtcI2cSemphoreHandle);
  }
  if (LL_I2C_IsActiveFlag_ARLO(I2C1))
  {
    IndicationLedError();
    LL_I2C_ClearFlag_ARLO(I2C1);
    osSemaphoreRelease(RtcI2cSemphoreHandle);
  }
  if (LL_I2C_IsActiveFlag_OVR(I2C1))
  {
    IndicationLedError();
    LL_I2C_ClearFlag_OVR(I2C1);
    osSemaphoreRelease(RtcI2cSemphoreHandle);
  }
}
/******************************************************************************/




void EXTI1_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    prvResetSysTicks();
  }
}
/******************************************************************************/











