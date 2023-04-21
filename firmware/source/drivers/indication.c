/**
 ******************************************************************************
 * @file           : indication.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Indication file
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "indication.h"

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
mculed_t mculed[N_LED];

/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
static void prvIndicationLedLoadingSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedButtonSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedButtonHoldSetup(mculed_t *led_ptr, int led_index);
void prvIndicationLedErrorSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedButtonDoubleClickSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedYellowSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedGreenSetup(mculed_t *led_ptr, int led_index);
void prvIndicationLedLeftSetup(mculed_t *led_ptr, int led_index);
void prvIndicationLedRightSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedReadySetup(mculed_t *led_ptr);

/******************************************************************************/




/**
 * @brief          Initialization off led CLK, Pins, hardware, fns and init of each led fns
 */
void IndicationInit(void)
{
	mculed_ctrl_t fns = {0};

  LL_GPIO_ResetOutputPin(LED_RD_GPIO_Port, LED_RD_Pin);
  LL_GPIO_ResetOutputPin(LED_YL_GPIO_Port, LED_YL_Pin);
  LL_GPIO_ResetOutputPin(LED_GR_GPIO_Port, LED_GR_Pin);

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = LED_RD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_RD_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_YL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_YL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_GR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GR_GPIO_Port, &GPIO_InitStruct);

	mculed[LED_RD].hardware.port   =    (uint32_t) LED_RD_GPIO_Port;
	mculed[LED_RD].hardware.pin    =    (uint32_t) LED_RD_Pin;
	mculed[LED_YL].hardware.port   =    (uint32_t) LED_YL_GPIO_Port;
	mculed[LED_YL].hardware.pin    =    (uint32_t) LED_YL_Pin;
	mculed[LED_GR].hardware.port   =    (uint32_t) LED_GR_GPIO_Port;
	mculed[LED_GR].hardware.pin    =    (uint32_t) LED_GR_Pin;

	fns.turn_on = IndicationLedTurnOn;
	fns.turn_off = IndicationLedTurnOff;
	fns.toggle = IndicationLedToggle;

	McuLedInit(&mculed[LED_RD], &fns);
	McuLedInit(&mculed[LED_YL], &fns);
	McuLedInit(&mculed[LED_GR], &fns);
}
/******************************************************************************/




/**
 * @brief          Led actions fns
 */
void IndicationLedTurnOn(mculed_t *self)
{
  LL_GPIO_SetOutputPin((GPIO_TypeDef *)self->hardware.port, self->hardware.pin);
}

void IndicationLedTurnOff(mculed_t *self)
{
  LL_GPIO_ResetOutputPin((GPIO_TypeDef *)self->hardware.port, self->hardware.pin);
}

void IndicationLedToggle(mculed_t *self)
{
  LL_GPIO_TogglePin((GPIO_TypeDef *)self->hardware.port, self->hardware.pin);
}
/******************************************************************************/




/**
 * @brief          Loading led animation function
 */
void IndicationLedLoading(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedLoadingSetup(&mculed[led_index], led_index);
    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          LED YL blink 1 time
 */
void IndicationLedYellow(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedYellowSetup(&mculed[led_index], led_index);
    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          LED GR blink 1 time
 */
void IndicationLedGreen(void)
{
    for (int led_index = 0; led_index < N_LED; led_index++) {
      prvIndicationLedGreenSetup(&mculed[led_index], led_index);
      LedFunction(&mculed[led_index]);
    }
}
/******************************************************************************/




/**
 * @brief          LED left on
 */
void IndicationLedLeft(void)
{
    for (int led_index = 0; led_index < N_LED; led_index++) {
      prvIndicationLedLeftSetup(&mculed[led_index], led_index);
      LedFunction(&mculed[led_index]);
    }
}
/******************************************************************************/




/**
 * @brief          LED right on
 */
void IndicationLedRight(void)
{
    for (int led_index = 0; led_index < N_LED; led_index++) {
      prvIndicationLedRightSetup(&mculed[led_index], led_index);
      LedFunction(&mculed[led_index]);
    }
}
/******************************************************************************/




/**
 * @brief          Button led function
 */
void IndicationLedButton(void)
{
    for (int led_index = 0; led_index < N_LED; led_index++) {
      prvIndicationLedButtonSetup(&mculed[led_index], led_index);
      LedFunction(&mculed[led_index]);
    }
}
/******************************************************************************/




/**
 * @brief          Hold button led function
 */
void IndicationLedButtonHold(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedButtonHoldSetup(&mculed[led_index], led_index);
    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          Double click button led function
 */
void IndicationLedButtonDoubleClick(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedButtonDoubleClickSetup(&mculed[led_index], led_index);
    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          Error led function
 */
void IndicationLedError(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedErrorSetup(&mculed[led_index], led_index);
    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          Ready board led function
 */
void IndicationLedReady(void)
{
  for (int led_index = 0; led_index < N_LED; led_index++) {
    prvIndicationLedReadySetup(&mculed[led_index]);
    LedFunction(&mculed[led_index]);
  }
}
/******************************************************************************/




/**
 * @brief          YL led blink 1 time setup
 */
void prvIndicationLedYellowSetup(mculed_t *led_ptr, int led_index)
{
  led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

  switch (led_index) {
    case LED_RD:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    case LED_YL:
      led_ptr->hardware.mode = MCULED_ON_STATE;
      led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
      led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
      led_ptr->setup.delay_ms = ZERO_MS;
      break;
    case LED_GR:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    default:
      break;
  }

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/



/**
 * @brief          GR led blink 1 time setup
 */
void prvIndicationLedGreenSetup(mculed_t *led_ptr, int led_index)
{
  led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

  switch (led_index) {
    case LED_RD:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    case LED_YL:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    case LED_GR:
      led_ptr->hardware.mode = MCULED_ON_STATE;
      led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
      led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
      led_ptr->setup.delay_ms = ZERO_MS;
      break;
    default:
      break;
  }

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Loading led left animation setup
 */
void prvIndicationLedLeftSetup(mculed_t *led_ptr, int led_index)
{
  led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

  switch (led_index) {
    case LED_YL:
      led_ptr->hardware.mode = MCULED_ON_STATE;
      led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
      led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    default:
      break;
  }

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Loading led right animation setup
 */
void prvIndicationLedRightSetup(mculed_t *led_ptr, int led_index)
{
  led_ptr->setup.iterations_num = INDICATION_LED_LOADING_NUM;

  switch (led_index) {
    case LED_YL:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
      break;
    default:
      break;
  }

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Loading led animation setup
 */
void prvIndicationLedLoadingSetup(mculed_t *led_ptr, int led_index)
{
	led_ptr->hardware.mode = MCULED_LED_LOADING;
	led_ptr->setup.iterations_num = INDICATION_LED_LOADING_NUM;

	switch (led_index) {
		case LED_RD:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RD_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RD_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_RD_TIME_ON));
			break;
		case LED_YL:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_YL_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_YL_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_YL_TIME_ON));
			break;
		case LED_GR:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_GR_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_GR_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_GR_TIME_ON));
			break;
		default:
			break;
 	}

	led_ptr->status.iterations_counter = 0;
	led_ptr->status.on_timeout = led_ptr->setup.on_ms;
	led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Button led setup
 */
void prvIndicationLedButtonSetup(mculed_t *led_ptr, int led_index)
{
	switch (led_index) {
		case LED_RD:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RD_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RD_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_YL:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			break;
		case LED_GR:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			break;
		default:
			break;
 	}

	led_ptr->status.iterations_counter = 0;
	led_ptr->status.on_timeout = led_ptr->setup.on_ms;
	led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Button hold led setup
 */
void prvIndicationLedButtonHoldSetup(mculed_t *led_ptr, int led_index)
{
	switch (led_index) {
		case LED_RD:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_YL:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_GR:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		default:
			break;
 	}

	led_ptr->status.iterations_counter = 0;
	led_ptr->status.on_timeout = led_ptr->setup.on_ms;
	led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief         Double click button led setup
 */
void prvIndicationLedButtonDoubleClickSetup(mculed_t *led_ptr, int led_index)
{
	switch (led_index) {
		case LED_RD:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_YL:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_GR:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		default:
			break;
 	}

	led_ptr->status.iterations_counter = 0;
	led_ptr->status.on_timeout = led_ptr->setup.on_ms;
	led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Error led setup
 */
void prvIndicationLedErrorSetup(mculed_t *led_ptr, int led_index)
{
	led_ptr->setup.iterations_num = INDICATION_LED_ERROR_NUM;

  switch (led_index) {
    case LED_RD:
      led_ptr->hardware.mode = MCULED_ON_STATE;
      led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
      led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
      led_ptr->setup.delay_ms = ZERO_MS;
        break;
    case LED_YL:
        led_ptr->hardware.mode = MCULED_OFF_STATE;
        led_ptr->setup.on_ms = ZERO_MS;
        led_ptr->setup.delay_ms = ZERO_MS;
        led_ptr->setup.off_ms = ZERO_MS;
        break;
    case LED_GR:
      led_ptr->hardware.mode = MCULED_OFF_STATE;
      led_ptr->setup.on_ms = ZERO_MS;
      led_ptr->setup.delay_ms = ZERO_MS;
      led_ptr->setup.off_ms = ZERO_MS;
        break;
    default:
        break;
  }

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Ready led setup
 */
void prvIndicationLedReadySetup(mculed_t *led_ptr)
{
  led_ptr->hardware.mode = MCULED_ON_STATE;
  led_ptr->setup.iterations_num = INDICATION_LED_READY_NUM;

  led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_SLOW * LED_TIME_ON);
  led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_SLOW * LED_TIME_OFF);
  led_ptr->setup.delay_ms = ZERO_MS;

  led_ptr->status.iterations_counter = 0;
  led_ptr->status.on_timeout = led_ptr->setup.on_ms;
  led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/




/**
 * @brief          Led update from SysTick_Handler
 */
void IndicationLedsUpdate(void)
{
  for (uint8_t led_index = 0; led_index < N_LED; led_index++) {
    LedUpdate(&mculed[led_index]);
  }
}
/******************************************************************************/
