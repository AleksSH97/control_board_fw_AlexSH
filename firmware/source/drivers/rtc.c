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
#define RTC_REG_DATE               (0x04)
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

const osThreadAttr_t RtcTask_attributes = {
    .name = "RtcTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

RTC_STATUS_t rtc_status;
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
  rtc_status = RtcInit();

  for (;;)
  {
    if (rtc_status != 0)
      RtcErrorHandler(rtc_status);


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

  memset(&rtc_status, 0x00, sizeof(rtc_status));
  rtc_ok = false;

  return res;
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
    PrintfLogsCRLF(CLR_RD "ERROR INIT I2C");
}











