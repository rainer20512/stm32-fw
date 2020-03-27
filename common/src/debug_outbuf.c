/**
  ******************************************************************************
  * @file    debug_outbuf.c
  * @author  Rainer
  * @brief   Implementation of a circular output buffer, that will automatically
  *          be written to DebugUart
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/** @addtogroup DEBUG_OUTBUF
  * @{
  */

#include "config/config.h"
#include "debug.h"
#include "task/minitask.h"

#if DEBUG_FEATURES > 0  && DEBUG_DEBUGIO == 0

#include "hardware.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "debug_helper.h"
#include "dev/devices.h"
#include "debug_outbuf.h"
#include "circbuf.h"

#include "dev/uart_dev.h"

/* Private define ------------------------------------------------------------*/

/* size of RX/TX buffers, sizes must be power of two  */
#define OUTBUF_SIZE 16384
#define INBUF_SIZE  64

/* Private variables ---------------------------------------------------------*/
static uint8_t bOngoingTransfer = 0;    /* indicates an ongoing transfer, if != 0 */
static uint8_t bExpandCrToCrlf = 0;     /* expand LF to CRLF */
static uint8_t bDelayedFlush = 0;       /* Keep in mind an delayed flush */

static uint8_t outbuf[OUTBUF_SIZE];
static uint8_t inbuf[INBUF_SIZE];
static CircBuffT o;
static LinBuffT i; 

/* public variables */
UsartHandleT *DebugUart=NULL;

/* Private macro -------------------------------------------------------------*/

/* forward declarations  -----------------------------------------------------*/
void DebugTxCpltCallback(UsartHandleT *uhandle);
void DebugRxCpltCallback(UsartHandleT *uhandle, uint8_t ch);
void DebugErrorCallback(UsartHandleT *uhandle);

/* Private functions  ---------------------------------------------------------*/

/* Start transfer to output device */
static void Debug_OutputTransfer(uint32_t from, uint32_t to )
{
  /* Make sure, there is no other active transfer */
  assert_param(!bOngoingTransfer);
  /* Make sure, there is no wraparound between 'from' and 'to'  */
  assert_param(from <= to);

  /* Make sure, there is a assigned DebugUart  */
  assert_param(DebugUart);

  
  bOngoingTransfer = 1;
  uint32_t transfer_size = to - from;

  UsartStartTx(DebugUart, o.buf+o.rdptr, transfer_size);
}


/* Public functions  ---------------------------------------------------------*/


#if DEBUG_SLEEP_STOP > 0
  /*********************************************************************************
    * @brief  Dumps current values of outbuf read an write pointers to 
    * @param  None
    * @retval None
    * @note   NO NORMAL DEBUG OUTPUT in this function or childs, 
    *         use special functions only!*
    ********************************************************************************/
  void DebugOutbufGetPtrs(void)
  {
    store_hexXXXX(o.rdptr);
    store_chr('/');
    store_hexXXXX(o.wrptr);
    store_chr(' ');
  }
#endif


/*
 * Callback when transfer complete *
 * - reset ongoing-transfer-flag
 * - increment txptr
 * - check for wraparound and continue transfer in case of wraparound 
 */
void DebugOutputCompleteCB ( uint32_t size )
{
  bOngoingTransfer = 0;
  o.rdptr = CBUFPTR_INCR(o, rdptr, size);

  /* If we kept in mind an delayed flush, initiate it now */
  if ( bDelayedFlush ) {
    bDelayedFlush = 0;
    TaskNotifyFromISR(TASK_OUT);
  }

}


/* Copy the content of the output buffer to output device */
void task_handle_out(uint32_t arg)
{
    UNUSED(arg);
   
   /* Don't flush when empty */
   if ( CBUF_EMPTY(o) ) return;

   /* Check for ongoing transfer. If so, keep in mind to flush afterwards */ 
   if ( bOngoingTransfer ) {
     bDelayedFlush = 1;
     return;
   }

  /*
   * If the buffer content wraps around, only transfer up to the end of the circular buffer
   * and keep in mind to transfer from begin later
   */
  if ( CBUF_WRAPAROUND(o) ) {
     bDelayedFlush = 1;
     Debug_OutputTransfer(o.rdptr, OUTBUF_SIZE );
   } else {
     Debug_OutputTransfer(o.rdptr, o.wrptr );
   }
}

/*
 * redefine system function __putchar so that standard io functions can be used to produce debug output
 */
/* int __putchar(int ch ) { */
int __putchar(int ch, __printf_tag_ptr uu) {
  if ( !DebugUart ) return ch;
  if ( bExpandCrToCrlf && (char)ch == '\n' ) __putchar('\r', uu);
/*  if ( bExpandCrToCrlf && (char)ch == '\n' ) __putchar('\r'); */

  /*
   * - if there is room left in outbuf, copy character to outbuf and start transfer, if character is '\n'
   * - otherwise replace the last written character with * to indicate overflow and start transfer
   */

  if ( !CircBuff_Put(&o, ch ) ) {
      TaskNotify(TASK_OUT);
  } else { 
    if ( ch == '\n') {
      TaskNotify(TASK_OUT);
    }
  }
     
  return ch;
}


void DebugAssignUart(const HW_DeviceType *dev, void *arg)
{ 
    UsartHandleT *uart = USART_GetHandleFromDev(dev);

    DebugUart          = uart;
    bExpandCrToCrlf    = (uint32_t)(arg);

    CircBuff_Init       (&o, OUTBUF_SIZE, outbuf);
    LinBuff_Init        (&i, INBUF_SIZE,  inbuf );
    Usart_AssignBuffers ( uart, &i, &o ); 
    UsartAssignCallbacks(uart,  DebugTxCpltCallback, DebugErrorCallback, DebugRxCpltCallback );
    printf("Redirecting stdout to %s\n",dev->devName);
}

void DebugDeAssignUart(const HW_DeviceType *dev)
{ 
    UNUSED(dev);
    DebugUart=NULL;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartCOMx: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void DebugTxCpltCallback(UsartHandleT *uhandle)
{
  DebugOutputCompleteCB(uhandle->TxCount);  // or Count ????    
}

/**
* @brief  Rx Transfer completed callback
* @param  UartCOMx: UART handle
* @retval None
*/
void DebugRxCpltCallback(UsartHandleT *uhandle, uint8_t ch)
{
    /* Handle CRLF, Echo & flush*/
    if ( ch == '\r' ) DEBUG_PUTC('\n');

    DEBUG_PUTC(ch);
    TaskNotifyFromISR(TASK_OUT);

    /* Del = remove DEL and last character */
    if ( ch == 0x07f ) { LBUF_DEL(*(uhandle->in));LBUF_DEL(*(uhandle->in)); }

    /* CR(LF) or ^D: Call input handler */
    if ( ch == '\r' || ch == 0x04 ) {
        /* CR(LF) = Remove terminating \r */
        if ( ch == '\r') LBUF_DEL(*(uhandle->in));
        TaskNotifyFromISR(TASK_COM);
    }
}

/**
  * @brief  UART error callbacks
  * @param  UartCOMx: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void DebugErrorCallback(UsartHandleT *uhandle)
{
/* Immediately return on 'no error' */
if ( uhandle->last_errors == HAL_UART_ERROR_NONE ) return; 
/* Test for receive error */
if ( uhandle->last_errors & ( HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE ) ) {
  /* Restart receiver on receive error */
  DEBUG_PRINTF("Error %x on Debug input receive\n", uhandle->last_errors);
} else {
  DEBUG_PUTS("DMA error on Debug output\n");
}
}



void dbg_printf( const char* format, ... ) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end( args );
}

#endif // #if DEBUG_FEATURES > 0 && DEBUG_DEBUGIO == 0


/**
  * @}
  */
