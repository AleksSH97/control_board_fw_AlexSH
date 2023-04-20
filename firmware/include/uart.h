/**
 ******************************************************************************
 * @file           : uart.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for uart
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H_
#define UART_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ring_buf.h"
#include "indication.h"
#include "io_uart.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define START_BYTE        0x3A
#define STOP_BYTE         0x68

#define UART_BUFF_SIZE    8u


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
extern struct uart io_uart;

//struct __attribute__ ((__packed__)) msg {
//    uint8_t start_byte;
//    uint8_t payload;
//    uint8_t stop_byte;
//};

struct uart {
    lwrb_t        lwrb_rx;
    lwrb_t        lwrb_tx;

    uint8_t       buff_rx[UART_BUFF_SIZE];
    uint8_t       buff_tx[UART_BUFF_SIZE];

    uint8_t       keyboarb_input;
    uint8_t       console_input;

    volatile bool flag;
};


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void UARTAllInit(void);
void UARTCheckMsg(void);
void UARTSendPacket(void);
//bool UARTSetupReceiveMsg(UART_HandleTypeDef *huart, struct msg *msg_ptr);
//bool UARTSetupReceiveByte(UART_HandleTypeDef *huart, uint8_t *byte);
//bool UARTSetupReceiveChar(UART_HandleTypeDef *huart, uint8_t *byte);
//bool UARTSendByte(UART_HandleTypeDef *huart, uint8_t byte);
//bool UARTSendByteTxBuff(UART_HandleTypeDef *huart);
//bool UARTReceiveByte(UART_HandleTypeDef *huart,uint8_t byte);
//bool UARTSendMsg(UART_HandleTypeDef *huart, struct msg *msg_ptr);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* UART_H_ */
