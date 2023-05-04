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
#include "rtc_i2c.h"


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
typedef struct {
  volatile uint8_t device;
  volatile bool addr_word;
  volatile uint8_t address[2];
  volatile bool mode_write;
  volatile bool address_sended;
  volatile bool repeated_start;
  volatile uint8_t byte;
  volatile uint8_t *buffer;
  volatile uint16_t length;
  volatile bool result;
} RtcI2c_t;

static RtcI2c_t rtc_i2c;

static osMutexId_t RtcI2cMutexHandle;

const osMutexAttr_t RtcI2cMutex_attr = {
  .name = "RtcI2cMutex",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0U
};



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
  (void) I2C1->SR2;

  rtc_i2c.device = 0;
  rtc_i2c.addr_word = false;
  rtc_i2c.address[0] = 0;
  rtc_i2c.address[1] = 0;
  rtc_i2c.mode_write = true;
  rtc_i2c.address_sended = false;
  rtc_i2c.repeated_start = false;
  rtc_i2c.byte = 0;
  rtc_i2c.buffer = NULL;
  rtc_i2c.length = 0;
  rtc_i2c.result = false;

  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

  if (LL_I2C_Init(I2C1, &I2C_InitStruct) != 0)
    return RTC_INIT_ERROR;

  LL_I2C_SetOwnAddress2(I2C1, 0);
  LL_I2C_Enable(I2C1);

  RtcI2cMutexHandle = osMutexNew(&RtcI2cMutex_attr);

  return RTC_OK;
}
/******************************************************************************/



/**
 * @brief          RTC I2C read byte
 */
uint8_t RtcI2cReadByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes)
{
//  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_SB(I2C1))
  {
    IndicationLedError();
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
    IndicationLedReady();
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

  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC I2C write byte
 */
uint8_t RtcI2cWriteByte(uint8_t device, uint8_t address, uint8_t *buffer, uint16_t num_bytes)
{
  uint16_t i;

//  osMutexAcquire(RtcI2cMutexHandle, osWaitForever);

  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_SB(I2C1)){};

  (void) I2C1->SR1;

  LL_I2C_TransmitData8(I2C1, device | RTC_REQUEST_WRITE);

  while(!LL_I2C_IsActiveFlag_ADDR(I2C1)){};

  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, (uint8_t) address);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)){};

  for (i = 0; i < num_bytes; i++)
  {
    LL_I2C_TransmitData8(I2C1, buffer[i]);
    while(!LL_I2C_IsActiveFlag_TXE(I2C1)){};
  }
  LL_I2C_GenerateStopCondition(I2C1);

//  osMutexRelease(RtcI2cMutexHandle);

  return RTC_OK;
}
/******************************************************************************/




void prvStartTransaction(uint8_t device, bool addr_word, uint16_t address)
{

}








