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

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/

/**
 * @brief          Initialization of any ring buffer
 */
void RingBuffInit(lwrb_t *lwrb_ptr, uint8_t *buff)
{
  lwrb_init(lwrb_ptr, buff, sizeof(buff));

  if (!lwrb_is_ready(lwrb_ptr)) {
      PrintfLogsCRLF("Error ring buf uart_rx init");
  }

  lwrb_set_evt_fn(lwrb_ptr, RingBufEvtCallback);
}
/******************************************************************************/




/**
 * @brief          Ring buffer callback
 */
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
/******************************************************************************/
