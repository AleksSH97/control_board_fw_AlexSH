/**
 ******************************************************************************
 * @file           : rtc.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : RTC driver
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"

#include <stdlib.h>

#include "rtc.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define RTCRST_Pin                 LL_GPIO_PIN_0
#define RTCRST_GPIO_Port           GPIOC
#define RTCINT_Pin                 LL_GPIO_PIN_1
#define RTCINT_GPIO_Port           GPIOC

#define RTC_REG_SECONDS            (0x00)
#define RTC_REG_MINUTES            (0x01)
#define RTC_REG_HOURS              (0x02)
#define RTC_REG_DAY                (0x03)
#define RTC_REG_MONTH_CENTURY      (0x05)
#define RTC_REG_YEAR               (0x06)
#define RTC_REG_ALARM_1_SECONDS    (0x07)
#define RTC_REG_ALARM_1_MINUTES    (0x08)
#define RTC_REG_ALARM_1_HOURS      (0x09)
#define RTC_REG_ALARM_1_DAY_DATE   (0x0A)
#define RTC_REG_ALARM_2_MINUTES    (0x0B)
#define RTC_REG_ALARM_2_HOURS      (0x0C)
#define RTC_REG_ALARM_2_DAY_DATE   (0x0D)
#define RTC_REG_CONTROL            (0x0E)
#define RTC_REG_STATUS             (0x0F)
#define RTC_REG_AGING_OFFSET       (0x10)
#define RTC_REG_TEMP_MSB           (0x11)
#define RTC_REG_TEMP_LSB           (0x12)
#define RTC_REG_TEST               (0x13)
#define RTC_REGS_SRAM_OFFSET       (0x14)
#define RTC_REGS_SRAM_LENGTH       (0xFF - 0x14)
#define RTC_REGS_SRAM_END          (0xFF)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
osThreadId_t RtcTaskHandle;
static osSemaphoreId_t RtcSemphoreHandle;

const osThreadAttr_t RtcTask_attributes = {
    .name = "RtcTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osSemaphoreAttr_t RtcSemaphore_attr = {
    .name = "RtcSemaphore",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0U
};

RTC_INFO_t rtc_info;
bool rtc_ok;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
uint8_t prvRtcGPIOInit(void);


/******************************************************************************/


/**
 * @brief          RTC task
 */
void RtcTask(void *argument)
{
  rtc_info.status = RtcInit();
  bool read = true;
  bool write = true;

  for(;;)
  {
    if (rtc_info.status != RTC_OK)
      RtcErrorHandler(rtc_info.status);

    if (write)
    {
      RTC_DATE_t date;
      char buf[11];
      buf[0] = 1;
      buf[1] = 1;
      buf[3] = 2;
      buf[4] = 2;
      buf[6] = 1;
      buf[7] = 9;
      buf[8] = 6;
      buf[9] = 3;

      buf[2] = 0;
      buf[5] = 0;
      buf[10] = 0;

      date.day = atoi(&buf[0]);
      date.month = atoi(&buf[3]);
      date.year = atoi(&buf[6]) % 100;

      rtc_info.status = RtcSetDate(&date);
      write = false;
      osDelay(5000);
    }

    if (read)
    {
      rtc_info.status = RtcGetDate(&rtc_info.date);
      read = false;
      osDelay(1000);
    }

    if (!read)
      osDelay(500);
      PrintfConsoleCRLF(CLR_DEF"Date: %02u.%02u.%04u", rtc_info.date.date, rtc_info.date.month, rtc_info.date.year);
  }
}
/******************************************************************************/




void RtcInitTask(void)
{
  RtcTaskHandle = osThreadNew(RtcTask, NULL, &RtcTask_attributes);
}




/**
 * @brief          RTC init
 */
uint8_t RtcInit(void)
{
  uint8_t res = RTC_OK;

  if (RtcI2cInit() != RTC_OK) {
    return RTC_INIT_ERROR;
  }

  if (prvRtcGPIOInit() != RTC_OK) {
    return RTC_INIT_ERROR;
  }

  memset(&rtc_info, 0x00, sizeof(rtc_info));
  rtc_ok = false;

  RtcSemphoreHandle = osSemaphoreNew(1, 1, &RtcSemaphore_attr);

  if (osSemaphoreAcquire(RtcSemphoreHandle, 0) != osOK)
    return RTC_INIT_ERROR;

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC get current date
 */
uint8_t RtcGetDate(RTC_DATE_t *date)
{
  uint8_t read_buffer[4];

//  osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);

  if (RtcI2cReadByte(RTC_REG_DATE, read_buffer, 4) == RTC_OK)
  {
    date->day = read_buffer[0];
    date->date = ((read_buffer[1] >> 4) * 10) + (read_buffer[1] & 0x0F);
    date->month = ((read_buffer[2] & 0x10) ? 10 : 0) + (read_buffer[2] & 0x0F);
    date->year  = 2000 + ((read_buffer[2] & 0x80) ? 100 : 0) + ((read_buffer[3] >> 4) * 10) + (read_buffer[3] & 0x0F);
  }
  else
    return RTC_RECEIVE_ERROR;

//  osSemaphoreRelease(RtcSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC set current date
 */
uint8_t RtcSetDate(RTC_DATE_t *date)
{
  uint8_t write_buffer[3];

//  osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);

  write_buffer[0] = ((date->date  / 10) << 4) | ((date->date  % 10) & 0x0F);
  write_buffer[1] = ((date->month / 10) << 4) | ((date->month % 10) & 0x0F);
  write_buffer[2] = ((date->year  / 10) << 4) | ((date->year  % 10) & 0x0F);

  if ((RtcI2cWriteByte(RTC_REG_DATE, write_buffer, 3)) != 0)
    return RTC_TRANSMIT_ERROR;

//  osSemaphoreRelease(RtcSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC gpio init
 */
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




/**
 * @brief          RTC gpio init
 */
void RtcErrorHandler(RTC_STATUS_t error)
{
  if (error != 0)
    PrintfLogsCRLF(CLR_RD "ERROR I2C");
}











