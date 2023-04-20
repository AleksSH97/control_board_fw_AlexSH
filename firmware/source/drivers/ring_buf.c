/**
 ******************************************************************************
 * @file           : ring_buf.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Ring buffer functions
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "ring_buf.h"



void RingBufUARTInit(void)
{
    lwrb_init(&io_uart.lwrb_rx, io_uart.buff_rx, sizeof(io_uart.buff_rx));
    lwrb_init(&io_uart.lwrb_tx, io_uart.buff_tx, sizeof(io_uart.buff_tx));

    if (!lwrb_is_ready(&io_uart.lwrb_rx)) {
        PrintfLogsCRLF("Error ring buf uart_rx init");
    }

    if (!lwrb_is_ready(&io_uart.lwrb_tx)) {
        PrintfLogsCRLF("Error ring buf uart_tx init");
    }

    lwrb_set_evt_fn(&io_uart.lwrb_rx, RingBufEvtCallback);
    lwrb_set_evt_fn(&io_uart.lwrb_tx, RingBufEvtCallback);
}




void RingBufEvtCallback(struct uart *self, lwrb_evt_type_t evt, size_t bp)
{
    switch(evt) {
        case LWRB_EVT_READ:
           // log_printf_crlf("Read %d bytes from ring buff event!", (int)bp);
            break;
        case LWRB_EVT_WRITE:
           // log_printf_crlf("Write %d bytes from ring buff event!", (int)bp);
            break;
        case LWRB_EVT_RESET:
            //log_printf_crlf("Reset %d bytes from ring buff event!", (int)bp);
            break;
    }
}
