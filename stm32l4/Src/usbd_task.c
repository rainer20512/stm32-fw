#include "config/config.h"

#if USE_USB > 0

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#include "dev/usb_dev.h"
#include "dev/uart_dev.h"
#include "task/minitask.h"


/* size of RX/TX buffers, sizes must be power of two  */
#define OUTBUF_SIZE 2048
#define INBUF_SIZE  2048

#define HWUART      HW_COM9
#define UARTHND     (&HandleCOM9)

typedef enum {
    NO_ACTION  = 0,
    USART_RECV = 1,
    USART_XMIT = 2,
    USBD_RECV  = 3,
} U2U_ActionEnum;

static DMAMEM uint8_t outbuf[OUTBUF_SIZE];
static DMAMEM uint8_t inbuf[INBUF_SIZE];
static uint32_t usbdbuf_used;
static uint8_t* usbdbuf_ptr;

static uint8_t bOngoingTransfer = 0;    /* indicates an ongoing transfer, if != 0 */
static uint8_t bDelayedFlush = 0;       /* Keep in mind an delayed flush */

static LinBuffT  i; 
static CircBuffT o;
static U2U_ActionEnum  u2u_action;

/* Private macro -------------------------------------------------------------*/
#define TASK_NOTIFY(a)      do { u2u_action = a; TaskNotify(TASK_USBD); } while (0)

/* forward declarations  -----------------------------------------------------*/
void UartTxCpltCallback (UsartHandleT *uhandle);
void UartRxCpltCallback (UsartHandleT *uhandle, uint8_t ch);
void UartErrorCallback  (UsartHandleT *uhandle);

void UsbdRxCallback     ( uint8_t  *buf, uint32_t rxlen );


/* Start transfer to output device */
static void Uart_OutputTransfer(uint32_t from, uint32_t to )
{
  /* Make sure, there is no other active transfer */
  assert_param(!bOngoingTransfer);
  /* Make sure, there is no wraparound between 'from' and 'to'  */
  assert_param(from <= to);

  bOngoingTransfer = 1;
  uint32_t transfer_size = to - from;

  UsartStartTx(UARTHND, o.buf+o.rdptr, transfer_size);
}



void Usbd_SetComm( uint32_t  baudrate, uint32_t  stopbits, uint32_t  parity, uint32_t  wordlength )
{
    UsartCommT comm;
    comm.baudrate   = baudrate;
    comm.stopbits   = stopbits;
    comm.parity     = parity;
    comm.wordlength = wordlength;
    Usart_SetCommParamsLong(&HWUART, &comm, false );
    USBD_CDC_StartReceive();
}

/*
void usbd_QueryComm( uint32_t  *baudrate, uint32_t  *stopbits, uint32_t *parity, uint32_t  *wordlength )
{
}
*/


static const USBD_CDC_CallbacksT usbCBs = { NULL, NULL, Usbd_SetComm, NULL, UsbdRxCallback };

void U2U_InitTask(void)
{
    CircBuff_Init           ( &o, OUTBUF_SIZE, outbuf );
    LinBuff_Init            ( &i, INBUF_SIZE,  inbuf  );
    Usart_AssignBuffers     ( UARTHND, &i, NULL ); 
    UsartAssignCallbacks    ( UARTHND,  UartTxCpltCallback, NULL, UartRxCpltCallback );
    DEBUG_PRINTF            ("Connecting USBD_CDC and %s\n",HWUART.devName);
    USBD_CDC_SetCallbacks   ((USBD_CDC_CallbacksT*)&usbCBs);
    USBD_CDC_StartReceive   ();
}

void U2U_RunTask(uint32_t arg)
{
    UNUSED(arg);

    uint32_t temp;

    switch ( u2u_action ) {
        case USART_RECV:
            /* save current value of wrptr to get aware of received chars during copy */
            temp = i.wrptr;  

            /* Copy input buffer and clear usart rx input buffer */
            USBD_CDC_CopyTxBuffer(inbuf, temp);
            DEBUG_PRINTF("Got %d UART chars\n", temp);
            if ( i.wrptr != temp ) {
                DEBUG_PRINTF("Error U2U receive: %d Chars were choked \n", i.wrptr-temp);
            }
            LinBuff_SetEmpty(&i);
            USBD_CDC_Transmit(temp);  
            break;
        case USART_XMIT:
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
                Uart_OutputTransfer(o.rdptr, OUTBUF_SIZE );
            } else {
                Uart_OutputTransfer(o.rdptr, o.wrptr );
            }
            break;
        case USBD_RECV:
            DEBUG_PRINTF("Got %d USB chars\n", usbdbuf_used);
            CircBuff_PutStr(&o, usbdbuf_ptr, usbdbuf_used);

            /* ReStart USBD receiver */
            USBD_CDC_StartReceive();


            /* Start UART OUTPUT */
            TASK_NOTIFY(USART_XMIT);
            break;
        default:
            DEBUG_PRINTF("Error: U2U_RunTask: No plan for action %d\n", u2u_action);
    }
}

void UartTxCpltCallback(UsartHandleT *uhandle)
{
  bOngoingTransfer = 0;
  o.rdptr = CBUFPTR_INCR(o, rdptr, uhandle->TxCount);

  /* If we kept in mind an delayed flush, initiate it now */
  if ( bDelayedFlush ) {
    bDelayedFlush = 0;
    TASK_NOTIFY(USART_XMIT);
  }
}

void UartRxCpltCallback(UsartHandleT *uhandle, uint8_t ch)
{
    /* Del = remove DEL and last character */
    if ( ch == 0x07f ) { LBUF_DEL(*(uhandle->in));LBUF_DEL(*(uhandle->in)); }

    /* CR(LF) or ^D: Call copy op */
    if ( ch == '\r' || ch == 0x04 ) {
      if ( ch == '\r' ) LinBuff_Putc(&i,'\n'); 
      /* Notify task to start USBD output */
      TASK_NOTIFY(USART_RECV);
    }
}

void UsbdRxCallback( uint8_t  *buf, uint32_t rxlen )
{
    /* keep buffer pointer and buffer in mind */
    usbdbuf_used = rxlen;
    usbdbuf_ptr  = buf;

    /* Notify task to start USBD output */
    TASK_NOTIFY(USBD_RECV);
}

#endif // #if USE_USB > 0

