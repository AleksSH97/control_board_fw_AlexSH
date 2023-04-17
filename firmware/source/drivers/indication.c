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
static void prvIndicationLedErrorSetup(mculed_t *led_ptr);
static void prvIndicationLedButtonDoubleClickSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedTopSetup(mculed_t *led_ptr, int led_index);
static void prvIndicationLedBottomSetup(mculed_t *led_ptr, int led_index);
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

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin|LED_TOP_Pin|LED_RIGHT_Pin|LED_BOTTOM_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = LED_LEFT_Pin|LED_TOP_Pin|LED_RIGHT_Pin|LED_BOTTOM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	mculed[LED_TOP].hardware.port    =    (uint32_t) LED_TOP_GPIO_Port;
	mculed[LED_TOP].hardware.pin     =    (uint32_t) LED_TOP_Pin;
	mculed[LED_LEFT].hardware.port   =    (uint32_t) LED_LEFT_GPIO_Port;
	mculed[LED_LEFT].hardware.pin    =    (uint32_t) LED_LEFT_Pin;
	mculed[LED_BOTTOM].hardware.port =    (uint32_t) LED_BOTTOM_GPIO_Port;
	mculed[LED_BOTTOM].hardware.pin  =    (uint32_t) LED_BOTTOM_Pin;
	mculed[LED_RIGHT].hardware.port  =    (uint32_t) LED_RIGHT_GPIO_Port;
	mculed[LED_RIGHT].hardware.pin   =    (uint32_t) LED_RIGHT_Pin;

	fns.turn_on = IndicationLedTurnOn;
	fns.turn_off = IndicationLedTurnOff;
	fns.toggle = IndicationLedToggle;

	McuLedInit(&mculed[LED_TOP], &fns);
	McuLedInit(&mculed[LED_LEFT], &fns);
	McuLedInit(&mculed[LED_BOTTOM], &fns);
	McuLedInit(&mculed[LED_RIGHT], &fns);
}
/******************************************************************************/




void IndicationUpdateTask(void *argument)
{
    PrintfLogsCRLF(CLR_GR"INDICATION INIT..."CLR_DEF);
    osDelay(1000);
    IndicationLedReady();

    for (;;)
    {
        IndicationLedsUpdate();
    }
}
/******************************************************************************/




/**
 * @brief          Led actions fns
 */
void IndicationLedTurnOn(mculed_t *self)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *) self->hardware.port, self->hardware.pin, GPIO_PIN_SET);
}

void IndicationLedTurnOff(mculed_t *self)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *) self->hardware.port, self->hardware.pin, GPIO_PIN_RESET);
}

void IndicationLedToggle(mculed_t *self)
{
	HAL_GPIO_TogglePin((GPIO_TypeDef *)self->hardware.port, self->hardware.pin);
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
 * @brief          LED top on
 */
void IndicationLedTop(void)
{
	for (int led_index = 0; led_index < N_LED; led_index++) {
	    prvIndicationLedTopSetup(&mculed[led_index], led_index);
	    LedFunction(&mculed[led_index]);
	}
}
/******************************************************************************/




/**
 * @brief          LED bottom on
 */
void IndicationLedBottom(void)
{
    for (int led_index = 0; led_index < N_LED; led_index++) {
        prvIndicationLedBottomSetup(&mculed[led_index], led_index);
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
	    prvIndicationLedErrorSetup(&mculed[led_index]);
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




void indication_led_accelerometer(void)
{

}




/**
 * @brief          Loading led top animation setup
 */
void prvIndicationLedTopSetup(mculed_t *led_ptr, int led_index)
{
    led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

    switch (led_index) {
        case LED_TOP:
            led_ptr->hardware.mode = MCULED_ON_STATE;
            led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
            led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
            led_ptr->setup.delay_ms = ZERO_MS;
            break;
//        case LED_LEFT:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_BOTTOM:
            led_ptr->hardware.mode = MCULED_OFF_STATE;
            led_ptr->setup.on_ms = ZERO_MS;
            led_ptr->setup.off_ms = ZERO_MS;
            led_ptr->setup.delay_ms = ZERO_MS;
            break;
//        case LED_RIGHT:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        default:
            break;
    }

    led_ptr->status.iterations_counter = 0;
    led_ptr->status.on_timeout = led_ptr->setup.on_ms;
    led_ptr->status.off_timeout = led_ptr->setup.off_ms;
}
/******************************************************************************/



/**
 * @brief          Loading led bottom animation setup
 */
void prvIndicationLedBottomSetup(mculed_t *led_ptr, int led_index)
{
    led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

    switch (led_index) {
        case LED_TOP:
            led_ptr->hardware.mode = MCULED_OFF_STATE;
            led_ptr->setup.on_ms = ZERO_MS;
            led_ptr->setup.delay_ms = ZERO_MS;
            led_ptr->setup.off_ms = ZERO_MS;
            break;
//        case LED_LEFT:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_BOTTOM:
            led_ptr->hardware.mode = MCULED_ON_STATE;
            led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
            led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
            led_ptr->setup.delay_ms = ZERO_MS;
            break;
//        case LED_RIGHT:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
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
//        case LED_TOP:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_LEFT:
            led_ptr->hardware.mode = MCULED_ON_STATE;
            led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
            led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
            led_ptr->setup.off_ms = ZERO_MS;
            break;
//        case LED_BOTTOM:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_RIGHT:
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
 * @brief          Loading led right animation setup
 */
void prvIndicationLedRightSetup(mculed_t *led_ptr, int led_index)
{
    led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;

    switch (led_index) {
//        case LED_TOP:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_LEFT:
            led_ptr->hardware.mode = MCULED_OFF_STATE;
            led_ptr->setup.on_ms = ZERO_MS;
            led_ptr->setup.delay_ms = ZERO_MS;
            led_ptr->setup.off_ms = ZERO_MS;
            break;
//        case LED_BOTTOM:
//            led_ptr->hardware.mode = MCULED_OFF_STATE;
//            led_ptr->setup.on_ms = ZERO_MS;
//            led_ptr->setup.delay_ms = ZERO_MS;
//            led_ptr->setup.off_ms = ZERO_MS;
//            break;
        case LED_RIGHT:
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
 * @brief          Loading led animation setup
 */
void prvIndicationLedLoadingSetup(mculed_t *led_ptr, int led_index)
{
	led_ptr->hardware.mode = MCULED_LED_LOADING;
	led_ptr->setup.iterations_num = INDICATION_LED_LOADING_NUM;

	switch (led_index) {
		case LED_TOP:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TOP_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TOP_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_TOP_TIME_ON));
			break;
		case LED_LEFT:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_LEFT_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_LEFT_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_LEFT_TIME_ON));
			break;
		case LED_BOTTOM:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_BOTTOM_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_BOTTOM_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_BOTTOM_TIME_ON));
			break;
		case LED_RIGHT:
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RIGHT_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_RIGHT_TIME_OFF);
			led_ptr->setup.delay_ms = (INDICATION_LED_SPEED_VERY_FAST * (LED_RIGHT_TIME_OFF - LED_RIGHT_TIME_ON));
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
		case LED_TOP:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TOP_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TOP_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_LEFT:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			break;
		case LED_BOTTOM:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			break;
		case LED_RIGHT:
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
		case LED_TOP:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_LEFT:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_BOTTOM:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_RIGHT:
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
		case LED_TOP:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_LEFT:
			led_ptr->hardware.mode = MCULED_ON_STATE;
			led_ptr->setup.iterations_num = INDICATION_LED_BUTTON_NUM;
			led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
			led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_BOTTOM:
			led_ptr->hardware.mode = MCULED_OFF_STATE;
			led_ptr->setup.on_ms = ZERO_MS;
			led_ptr->setup.off_ms = ZERO_MS;
			led_ptr->setup.delay_ms = ZERO_MS;
			break;
		case LED_RIGHT:
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
void prvIndicationLedErrorSetup(mculed_t *led_ptr)
{
	led_ptr->hardware.mode = MCULED_ON_STATE;
	led_ptr->setup.iterations_num = INDICATION_LED_ERROR_NUM;

	led_ptr->setup.on_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_ON);
	led_ptr->setup.off_ms = (INDICATION_LED_SPEED_VERY_FAST * LED_TIME_OFF);
	led_ptr->setup.delay_ms = ZERO_MS;

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
