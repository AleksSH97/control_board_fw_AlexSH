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
static osSemaphoreId_t RtcI2cSemphoreHandle;

const osMutexAttr_t RtcI2cMutex_attr = {
  .name = "RtcI2cMutex",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0U
};

const osSemaphoreAttr_t RtcI2cSemaphore_attr = {
    .name = "RtcI2cSemaphore",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0U
};

/******************************************************************************/


/**
 * @brief          RTC I2C init
 */
uint8_t RtcI2cInit(void)
{
  uint8_t res = 0x00;

  LL_I2C_InitTypeDef I2C_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

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
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);

  RtcI2cMutexHandle = osMutexNew(&RtcI2cMutex_attr);
  RtcI2cSemphoreHandle = osSemaphoreNew(1, 1, &RtcI2cSemaphore_attr);

  osSemaphoreAcquire(RtcI2cSemphoreHandle, 0);

  return res;
}
/******************************************************************************/

















