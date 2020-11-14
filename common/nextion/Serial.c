#include "Serial.h"
#include "util/Utilities.h"
#include "NexConfig.h"
#include "NexHardware.h"

#include "dev/uart_dev.h"
#include "task/minitask.h"

#include "debug_helper.h"

#include <string.h>

#define SERIAL (&HW_COM2)
#define SERIALHANDLE (&HandleCOM2)

/* size of RX/TX buffers, sizes must be power of two  */
#define OUTBUF_SIZE 256
#define INBUF_SIZE  64

#define READ_WAIT_ADDITIONAL_MS         5

/* Private variables ---------------------------------------------------------*/

static DMAMEM uint8_t outbuf[OUTBUF_SIZE];
static DMAMEM uint8_t inbuf[INBUF_SIZE];
static CircBuffT o;
static LinBuffT i; 
static uint8_t bOngoingTransfer = 0;    /* indicates an ongoing transfer, if != 0 */
static uint32_t usPerSymbol;            /* time[us] to transfer one uint8 at st baudrate */

/* forward declarations  -----------------------------------------------------*/
void SerialTxCpltCallback(UsartHandleT *uhandle);
void SerialRxCpltCallback(UsartHandleT *uhandle, uint8_t ch);
void SerialErrorCallback (UsartHandleT *uhandle);
void Serial_Flush       (void);

void Serial_Init(long baudrate)
{
    Usart_SetCommParams(SERIAL, baudrate, false);
    /* We assume 10 bits per symbol: start, 8 data, stop */
    usPerSymbol = 1000000/(baudrate/10);
    /* add 25% as safety margin */
    usPerSymbol += usPerSymbol/4;

    CircBuff_Init       (&o, OUTBUF_SIZE, outbuf);
    LinBuff_Init        (&i, INBUF_SIZE,  inbuf );
    Usart_AssignBuffers ( SERIALHANDLE, &i, &o ); 
    UsartAssignCallbacks( SERIALHANDLE,  SerialTxCpltCallback, SerialErrorCallback, SerialRxCpltCallback );
}

void Serial_Start(void)
{
    CBUF_RESET(*(SERIALHANDLE->out));
    LinBuff_SetEmpty(SERIALHANDLE->in);
}

void Serial_Terminate(void)
{
    CircBuff_PutStr(SERIALHANDLE->out, (uint8_t *)termstr, TERMLEN );
    Serial_Flush();
    //TaskNotify(TASK_SER);
}

/******************************************************************************
 * Waits for uint8_t datum referenced by "ref" is equal or greater than limit
 * true is returned, if condition is met in estimated time
 * ref may be NULL, in which case no condition checking is done
 * false otherwise
 *****************************************************************************/
unsigned char Serial_WaitFor( uint8_t nrOfChars, uint8_t *ref, uint8_t limit)
{
    uint32_t delayms = usPerSymbol * nrOfChars / 1000 + 1;
    delayms += READ_WAIT_ADDITIONAL_MS;

    DEBUG_PRINTTS("SRW=%d\n",delayms); 
    while ( delayms-- ) {
        DEBUG_PRINTTS("RW\n"); 
        if ( ref && *ref >= limit ) return 1;
        nexDelay(1);
    }
    DEBUG_PRINTTS("EW\n"); 
    return 0;
}


unsigned char Serial_Read()
{
    unsigned char c;
    
    if ( LinBuff_Getc(SERIALHANDLE->in, &c) ) {
//        if LBUF_EMPTY(*(SERIALHANDLE->in)) LinBuff_SetEmpty(SERIALHANDLE->in);
    } else {
        c  = -1;
    }
    return c;
}

unsigned char Serial_Available()
{
   return LBUF_USED(*(SERIALHANDLE->in));
}

unsigned char Serial_ReadBytes(char *buf, unsigned char len)
{
    unsigned char cnt = 0;
    uint8_t *ret;

    /* Get ptr to inbuf and number of chars available */
    cnt = LinBuff_GetN(SERIALHANDLE->in, &ret, len );

    /* copy to return buffer */
    memcpy(buf, ret, cnt);

//    if LBUF_EMPTY(*(SERIALHANDLE->in)) LinBuff_SetEmpty(SERIALHANDLE->in);
    return cnt;
}

void Serial_Print(unsigned char *txt)
{
    CircBuff_PutStr(SERIALHANDLE->out, txt, strlen((char *)txt) );
}


void SerialRxCpltCallback(UsartHandleT *uhandle, uint8_t ch)
{
    UNUSED(uhandle);
    DEBUG_PRINTF("SerIn[%d]: %02x\n", i.wrptr-1, ch);
    if ( i.ovrrun ) { LinBuff_SetEmpty(&i); return; }
    if ( LBUF_USED(i) >= 3 ) {
        /* Check for terminating three 0xFF bytes */
        uint32_t ref = i.wrptr - TERMLEN;
        if ( memcmp(i.buf+ref, termstr, TERMLEN ) == 0 ) {
            /* Remove Terminator and set answer valid */
            i.wrptr -= TERMLEN;
            answer.bAnswerValid = 1;
            TaskNotify(TASK_SERIN);
        }
    }
}

/**
  * @brief  UART error callbacks
  * @param  UartCOMx: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void SerialErrorCallback(UsartHandleT *uhandle)
{
    /* Immediately return on 'no error' */
    if ( uhandle->last_errors == HAL_UART_ERROR_NONE ) return; 

    /* Test for receive error */
    if ( uhandle->last_errors & ( HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE ) ) {
        /* Restart receiver on receive error */
        DEBUG_PRINTF("Error %x on Serial input receive\n", uhandle->last_errors);
    } else {
        DEBUG_PUTS("DMA error on Serial output\n");
    }
}


unsigned char Serial_IsWriteActive()
{
    return bOngoingTransfer;
}

/*
 * Callback when transfer complete *
 * - reset ongoing-transfer-flag
 * - increment txptr
 */
void SerialTxCpltCallback(UsartHandleT *uhandle)
{

    bOngoingTransfer = 0;
    o.rdptr = CBUFPTR_INCR(o, rdptr, uhandle->TxCount);
}

void Serial_Flush(void)
{
   /* Don't flush when empty */
   if ( CBUF_EMPTY(o) ) return;

   /* Make sure, there is no wraparound between 'from' and 'to'  */
   assert (!CBUF_WRAPAROUND(o) );
   /* Make sure, there is no ongoing transfer */
   assert_param(!bOngoingTransfer);
  
   bOngoingTransfer = 1;
   uint32_t transfer_size = o.wrptr - o.rdptr;

   DEBUG_PRINTF("SerOut: ");
   for ( uint32_t i=0; i < transfer_size; i++ )
       DEBUG_PRINTF("%02x ", o.buf[o.rdptr+i]);
   DEBUG_PRINTF("\n");

   UsartStartTx(SERIALHANDLE, o.buf+o.rdptr, transfer_size);

    /* Wait for write to be completed */
    while ( Serial_IsWriteActive() ) {
//        DEBUG_PRINTTS("WW\n");
        nexDelay(1);
    }
    DEBUG_PRINTTS("EW\n");
}

