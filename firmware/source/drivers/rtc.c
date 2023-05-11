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

#define RTC_NUM_OF_ERRORS          (255u)
#define RTC_DATE_BUF_SIZE          (11u)
#define RTC_TIME_BUF_SIZE          (8u)

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

typedef struct {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint16_t ms;
} RTC_TIME_t;

typedef struct {
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint16_t year;
} RTC_DATE_t;

typedef struct {
  RTC_TIME_t time;
  RTC_DATE_t date;
  RTC_ERROR_t error;
  RTC_MODE_t mode;

  char date_buf[RTC_DATE_BUF_SIZE];
  char time_buf[RTC_TIME_BUF_SIZE];
} RTC_INFO_t;

RTC_INFO_t rtc_info;
bool rtc_ok;
uint32_t systicks;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
uint8_t prvRtcGPIOInit(void);
uint8_t prvGetDate(RTC_DATE_t *date);
uint8_t prvSetDate(RTC_DATE_t *date);
uint8_t prvCheckDate(RTC_DATE_t *date);
uint8_t prvCheckTime(RTC_TIME_t *time);
uint8_t prvSetTime(RTC_TIME_t *time);
uint8_t prvGetTime(RTC_TIME_t *time);
uint32_t prvGetTicks(void);
void prvResetSysTicks(void);


/******************************************************************************/


/**
 * @brief          RTC task
 */
void RtcTask(void *argument)
{
  uint8_t error = 0x00;

  error = RtcInit();

  if (error != RTC_OK)
    RtcSetError(error);

  for(;;)
  {
    if (error != RTC_OK)
      RtcSetError(error);

    if (RtcGetError() != RTC_OK)
    {
      RtcErrorHandler(rtc_info.error);
      RtcSetError(RTC_OK);
    }

    switch (RtcGetMode())
    {
      case RTC_GET_DATE:
        error = RtcGetDate();
      case RTC_GET_TIME:
        error = RtcGetTime();
      case RTC_IDLE:
        __NOP();
        break;
    }
  }
}
/******************************************************************************/




/**
 * @brief          RTC init task
 */
void RtcInitTask(void)
{
  RtcTaskHandle = osThreadNew(RtcTask, NULL, &RtcTask_attributes);
}
/******************************************************************************/




/**
 * @brief          RTC init
 */
uint8_t RtcInit(void)
{
  uint8_t dummy;
  systicks = 0UL;

  if (RtcI2cInit() != RTC_OK)
    return RTC_INIT_ERROR;

  if (prvRtcGPIOInit() != RTC_OK) {
    return RTC_INIT_ERROR;
  }

  memset(&rtc_info, 0x00, sizeof(rtc_info));
  rtc_ok = false;

  if (RtcI2cReadByte(RTC_HW_ADDRESS, 0, &dummy, 1) == RTC_OK)
    rtc_ok = true;

  PROJ_UNUSED(dummy);

  osDelay(100);

  if (rtc_ok)
    RtcI2cWriteByte(RTC_HW_ADDRESS, RTC_REG_CONTROL, 0, 1);
  else
    return RTC_INIT_ERROR;

  RtcSemphoreHandle = osSemaphoreNew(1, 1, &RtcSemaphore_attr);

  if (RtcSemphoreHandle == NULL)
    return RTC_INIT_ERROR;

  if (RtcSetMode(RTC_IDLE) != RTC_OK)
    return RTC_INIT_ERROR;

  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC set current date
 * param[in]       buf: Pointer to buffer with input date
 * @return         Current error instance
 */
uint8_t RtcSetDate(char *buf)
{
  uint8_t res = 0x00;
  RTC_DATE_t date;

  memcpy(rtc_info.date_buf, buf, 10);

  rtc_info.date_buf[2] = 0;
  rtc_info.date_buf[5] = 0;
  rtc_info.date_buf[10] = 0;

  if (rtc_info.date_buf == NULL)
    return RTC_SET_DATE_BUFFER_ERROR;

  date.date = atoi(&buf[0]);
  date.month = atoi(&buf[3]);
  date.year = atoi(&buf[6]) % 100;

  res = prvCheckDate(&date);

  if (res != RTC_OK)
    return RTC_CHECK_DATE_ERROR;

  res = prvSetDate(&date);

  if (res == RTC_OK)
    PrintfConsoleCRLF(CLR_DEF"Date set "CLR_GR"successful"CLR_DEF);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC get current date
 */
uint8_t RtcGetDate(void)
{
  uint8_t res = 0x00;
  RTC_DATE_t date;

  res = prvGetDate(&date);

  if(res == RTC_OK)
    PrintfConsoleCRLF(CLR_DEF"Date: %02u.%02u.%04u", date.date, date.month, date.year);

  RtcSetMode(RTC_IDLE);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC set current time
 * param[in]       buf: Pointer to buffer with input time
 * @return         Current error instance
 */
uint8_t RtcSetTime(char *buf)
{
  uint8_t res = 0x00;
  RTC_TIME_t time;

  memcpy(rtc_info.time_buf, buf, 8);

  rtc_info.time_buf[2] = 0;
  rtc_info.time_buf[5] = 0;
  rtc_info.time_buf[8] = 0;

  if (rtc_info.time_buf == NULL)
    return RTC_SET_DATE_BUFFER_ERROR;

  time.hours   = atoi(&buf[0]);
  time.minutes = atoi(&buf[3]);
  time.seconds = atoi(&buf[6]);

  res = prvCheckTime(&time);

  if (res != RTC_OK)
    return RTC_CHECK_TIME_ERROR;

  res = prvSetTime(&time);

  if (res == RTC_OK)
    PrintfConsoleCRLF(CLR_DEF"Time set "CLR_GR"successful"CLR_DEF);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC get current time
 */
uint8_t RtcGetTime(void)
{
  uint8_t res = 0x00;
  RTC_TIME_t time;

  res = prvGetTime(&time);

  if (res == RTC_OK)
    PrintfConsoleCRLF("\t"CLR_GR"RTC time %02u:%02u:%02u.%03u"CLR_DEF, time.hours, time.minutes,
        time.seconds, time.ms);

  RtcSetMode(RTC_IDLE);

  return res;
}
/******************************************************************************/




/**
 * @brief          RTC set current mode
 * param[in]       mode: mode which going to be set
 */
uint8_t RtcSetMode(RTC_MODE_t mode)
{
  if (mode > RTC_NUM_OF_ERRORS)
    return RTC_SET_MODE_ERROR;

  rtc_info.mode = mode;

  return RTC_OK;
}
/******************************************************************************/




/**
 * @brief          RTC get current mode
 * @return         rtc_info.mode: current instance
 */
RTC_MODE_t RtcGetMode(void)
{
  return rtc_info.mode;
}
/******************************************************************************/




/**
 * @brief          RTC set current error
 * param[in]       error: current error
 */
void RtcSetError(RTC_ERROR_t error)
{
  rtc_info.error = error;
}
/******************************************************************************/




/**
 * @brief          RTC get current error
 * @return         rtc_info.error current instance
 */
RTC_ERROR_t RtcGetError(void)
{
  return rtc_info.error;
}
/******************************************************************************/




uint8_t prvGetDate(RTC_DATE_t *date)
{
  uint8_t rtc_date_buffer[4];
  int rc;

  rc = osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);
  if (rc != osOK)
    return RTC_RECEIVE_ERROR;

  rc = RtcI2cReadByte(RTC_HW_ADDRESS, RTC_REG_DAY, rtc_date_buffer, 4);

  if (rc != RTC_OK) {
    osSemaphoreRelease(RtcSemphoreHandle);
    return rc;
  }

  date->day   = rtc_date_buffer[0];
  date->date  = ((rtc_date_buffer[1] >> 4) * 10) + (rtc_date_buffer[1] & 0x0F);
  date->month = ((rtc_date_buffer[2] & 0x10) ? 10 : 0) + (rtc_date_buffer[2] & 0x0F);
  date->year  = 2000 + ((rtc_date_buffer[2] & 0x80) ? 100 : 0) + ((rtc_date_buffer[3] >> 4) * 10) + (rtc_date_buffer[3] & 0x0F);

  osSemaphoreRelease(RtcSemphoreHandle);

  return RTC_OK;
}
/******************************************************************************/




uint8_t prvGetTime(RTC_TIME_t *time)
{
  uint8_t rtc_time_buffer[3];
  int rc;

  rc = osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);
  if (rc != osOK)
    return RTC_RECEIVE_ERROR;

  rc = RtcI2cReadByte(RTC_HW_ADDRESS, RTC_REG_SECONDS, rtc_time_buffer, 3);
  if (rc != RTC_OK) {
    osSemaphoreRelease(RtcSemphoreHandle);
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

  osSemaphoreRelease(RtcSemphoreHandle);

  return RTC_OK;
}
/******************************************************************************/



uint8_t prvSetDate(RTC_DATE_t *date)
{
  uint8_t write_buffer[3];

  osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);

  write_buffer[0] = ((date->date  / 10) << 4) | ((date->date  % 10) & 0x0F);
  write_buffer[1] = ((date->month / 10) << 4) | ((date->month % 10) & 0x0F);
  write_buffer[2] = ((date->year  / 10) << 4) | ((date->year  % 10) & 0x0F);

  if ((RtcI2cWriteByte(RTC_HW_ADDRESS, RTC_REG_DATE, write_buffer, 3)) != 0)
    return RTC_TRANSMIT_ERROR;

  osSemaphoreRelease(RtcSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




uint8_t prvSetTime(RTC_TIME_t *time)
{
  uint8_t write_buffer[3];

  osSemaphoreAcquire(RtcSemphoreHandle, osWaitForever);

  write_buffer[0] = ((time->seconds / 10) << 4) | ((time->seconds % 10) & 0x0F);
  write_buffer[1] = ((time->minutes / 10) << 4) | ((time->minutes % 10) & 0x0F);
  write_buffer[2] = ((time->hours   / 10) << 4) | ((time->hours   % 10) & 0x0F);

  if ((RtcI2cWriteByte(RTC_HW_ADDRESS, RTC_REG_SECONDS, write_buffer, 3)) != RTC_OK)
    return RTC_TRANSMIT_ERROR;

  osSemaphoreRelease(RtcSemphoreHandle);
  return RTC_OK;
}
/******************************************************************************/




uint8_t prvCheckDate(RTC_DATE_t *date)
{
  uint8_t status = RTC_OK;

  if ((date->date < 1) || (date->date > 31))
    status = RTC_CHECK_DATE_ERROR;
  else if ((date->month < 1) || (date->month > 12))
    status = RTC_CHECK_DATE_ERROR;
  else if ((date->year < 0) || (date->year > 99))
    status = RTC_CHECK_DATE_ERROR;

  return status;
}
/******************************************************************************/




uint8_t prvCheckTime(RTC_TIME_t *time)
{
  uint8_t status = RTC_OK;

  if ((time->hours < 0) || (time->hours > 23))
    status = RTC_CHECK_TIME_ERROR;
  if ((time->minutes < 0) || time->minutes > 59)
    status = RTC_CHECK_TIME_ERROR;
  if ((time->seconds < 0) || time->seconds > 59)
    status = RTC_CHECK_TIME_ERROR;

  return status;
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
void RtcErrorHandler(RTC_ERROR_t error)
{
  switch (error)
  {
    case RTC_OK:
      break;
    case RTC_INIT_ERROR:
      PrintfConsoleCRLF(CLR_DEF"ERROR RTC: "CLR_RD"INIT"CLR_DEF);
      break;
    case RTC_TRANSMIT_ERROR:
      PrintfConsoleCRLF(CLR_DEF"ERROR RTC: "CLR_RD"TRANSMIT"CLR_DEF);
      break;
    case RTC_RECEIVE_ERROR:
      PrintfConsoleCRLF(CLR_DEF"ERROR RTC: "CLR_RD"RECEIVE"CLR_DEF);
      break;
    case RTC_FLAG_ERROR:
      PrintfConsoleCRLF(CLR_DEF"ERROR RTC: "CLR_RD"FLAG"CLR_DEF);
      break;
    case RTC_CHECK_DATE_ERROR:
      PrintfConsoleCRLF("\t"CLR_RD"ERROR: date format is not correct"CLR_DEF);
      break;
    case RTC_CHECK_TIME_ERROR:
      PrintfConsoleCRLF("\t"CLR_RD"ERROR: time format is not correct"CLR_DEF);
      break;
    default:
      PrintfConsoleCRLF(CLR_DEF"ERROR RTC: "CLR_RD"UNDEFINED"CLR_DEF);
      break;
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






