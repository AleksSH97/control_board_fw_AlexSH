/**
 ******************************************************************************
 * @file           : ring_buf.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for ring buffer
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RING_BUF_H_
#define RING_BUF_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "lwrb.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void RingBufUARTInit(void);
void RingBufMicrophoneInit(void);
void RingBufAcceleroInit(void);
void RingBufEvtCallback(struct uart *self, lwrb_evt_type_t evt, size_t bp);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* RING_BUF_H_ */
