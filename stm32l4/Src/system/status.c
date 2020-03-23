#include "config/config.h"
#include "system/status.h"
#include "eeprom.h"
#include "global_flags.h"
#include "debug_helper.h"
#include "debug_outbuf.h"
#include "rtc.h"
#include "timer.h"
#include "dev/io_dev.h"
#if USE_DISPLAY > 0
    #include "ui/lcd.h"
    #include "ui/lcd_status.h"
#endif

#if DEBUG_MODE > 0
    #include "task/minitask.h"
#endif

FskStatusEnumType FSK_status=0;  
OokStatusEnumType OOK_status=0;

/* We have one additional element here to indicate "FSK mode is off" */
static const char * const FskStatusText[FSK_STATUS_NROF_ELEMENTS+1] = {
  "Sync",
  "Norm",
  "TmOu",
  "NoRF",
  "Off "
};


const char *Get_FSK_status_text(void) {
  if ( config.FSK_mode)
    return FskStatusText[FSK_status];
  else
    return FskStatusText[FSK_STATUS_NROF_ELEMENTS];
}

static const char g_status_text[] = { 'C', 'F' };
static const uint8_t g_status_bit[] = { GFLAG_CALIBRATING_BIT, GFLAG_FSK_BIT };


void StatusPrintGlobalStatus( void )
{
    uint32_t i;
    uint8_t work;
    for ( i=0; i < sizeof(g_status_bit)/sizeof(uint8_t); i++ ) {
        work = g_status_bit[i];
        if ( gflags & ( 1 << work ) )
            DEBUG_PUTC(g_status_text[i]);
        else		
            DEBUG_PUTC('-');
    }
}


void SetFSKStatus( FskStatusEnumType  newStatus, uint8_t bForce )
{
    if ( newStatus != FSK_status || bForce ) {
        FSK_status = newStatus;
        #if USE_DISPLAY > 0
            LCD_DisplayStatus(LCD_STATUS_RFM);
        #endif
    }
}

#if defined(TX18LISTENER)

    uint8_t OOK_status;

    const char x05[] = "NoTr";
    const char x04[] = "Norm";
    const char x03[] = "Init";
    const char x02[] = "Sync";
    const char x01[] = "Time";
    const char x00[] = "Inh";
    const char * const ook_status_text[] = { x00, x01, x02, x03, x04, x05 };

    const char *Get_OOK_status_text(void) {
        return ook_status_text[OOK_status];
    }

    void StatusPrintOOKStatus( void )
    {
        if ( OOK_Receiver_ForceMode() )
            DEBUG_PRINTF("Forc");
        else
            DEBUG_PRINTF("%s",Get_OOK_status_text());
    }


    void SetOOKStatus( OokStatusEnumType newStatus, uint8_t bForce )
    {
        if ( newStatus != OOK_status || bForce ) {
            OOK_status = newStatus;
            #if (USE_DISPLAY > 0)
                LCD_DisplayStatus(LCD_STATUS_RFM);
            #endif
        }
    }

#endif




/*********************************************************************************
  * @brief  Returns true, if the core can be put into Stop2-Mode
  * @param  None
  * @retval true, if Stop2 can be safely entered
  * @note   Reasons to inhibit Stop2
  *         - UART busy ( receive ) or transmit buffer not empty
  *         - Stop2 forbidden by config value
  * @note   NO DEBUG OUTPUT in this function or childs, use special functions only!
  *
  ********************************************************************************/
bool CanStop( void )
{

    /* don't stop if inhibited by global debug flag */
    #if DEBUG_MODE > 0
        if ( !bAllowStop ) return false;
    #endif

    /* don't stop, if inhibited by flag */
    if ( !config.allow_Stop ) {
        #if DEBUG_SLEEP_STOP > 1
          store_str("-c");
        #endif
        return false;
    }

    /* Don't stop, if any devices don't allow it */
    if ( DevicesInhibitStop() ) {
        #if DEBUG_SLEEP_STOP > 0
          store_str("-d");
        #endif
        return false;
    }

    #if DEBUG_SLEEP_STOP > 0
      store_time("++");
    #endif
    return true;
}
#if DEBUG_PROFILING > 0
    #include "rtc.h"
    #include "system/profiling.h"
    uint32_t start_stopmode;   /* Time we went to stop mode */
    #define GET_SYSTEMTIME()  (((uint32_t)HW_GetMinute()*60+(uint32_t)HW_GetSecond())*1000+HW_GetMillis())
    #define SYSTEMTIME_MAX    (60*60*1000)
#endif
/*********************************************************************************
  * @brief  Actions to be done just before stop2 mode will be entered
  *         - Disable USART2
  *         - Record system time to calculate the stop time on WakeUp
  *
  * @param  None
  *
  * @retval None         
  *
  ********************************************************************************/
void StopMode_Before(void)
{
    #if DEBUG_FEATURES > 0  && DEBUG_DEBUGIO == 0
        /* Disable Transmitter */
        DebugUart->Instance->CR1 &= ~USART_CR1_TE;
        /* Wait for TC bit in ISR become set
           ! check no longer neccessary: TC bit is checked as one can-sleep-condition 
           while ( !(DebugUart->Instance->ISR & USART_ISR_TC ) ) {} ;
        */

        /* Enable Wakeup Interrupt, wakeup flags have been configured to RXNE before in usart.c */
        DebugUart->Instance->CR3 |= USART_CR3_WUFIE;    

        /* Disable Whole U(s)art */
        // DebugUart->Instance->CR1 &= ~USART_CR1_UE;      

        /* Enable Wakeup From Stop, Receiver Interrupt is still active */
        DebugUart->Instance->CR1 |= USART_CR1_UESM;    
    #endif

    #if DEBUG_PROFILING > 0
       start_stopmode = GET_SYSTEMTIME();
    #endif
}

/*********************************************************************************
  * @brief  Actions to be taken just after stop2 mode has been left. Commonly, 
  *         revert all actions that have been done before stop mode was entered
  *         After a corrected value has been returned, the normal value will be
  *         - Reenable USART2
  *
  * @param  StopMode - the Stop Mode, that has been left [0,1,2]
  *         
  * @retval None
  *
  ********************************************************************************/
void StopMode_After(uint32_t StopMode)
{
    #if DEBUG_PROFILING > 0
       uint32_t end_stopmode = GET_SYSTEMTIME();

       /* Check for rollover */
       // DEBUG_PRINTF("Start=%d, End=%d\n", start_stopmode, end_stopmode);
       if ( end_stopmode < start_stopmode ) end_stopmode += SYSTEMTIME_MAX;

       /* Compute delta, that is the stoptime in ms */
       end_stopmode -= start_stopmode;

       /* Update profiler with sleep time */
      ProfilerIncrementStopTime( end_stopmode * 1000, StopMode );
    #endif

    #if DEBUG_FEATURES > 0  && DEBUG_DEBUGIO == 0

        /* Disable Wakeup Interrupt, */
        DebugUart->Instance->CR3 &= ~USART_CR3_WUFIE;    
        /* Disable Wakeup From Stop, Receiver Interrupt is still active */
        DebugUart->Instance->CR1 &= ~USART_CR1_UESM;      
        /* Reenable Transmitter */
        DebugUart->Instance->CR1 |= ( USART_CR1_UE | USART_CR1_TE );
    #endif
    // DEBUG_PUTC('b');
}

void UserPinSignal1(void)
{
  IO_UserLedOn(2);
  /*for ( uint32_t i = 0; i < 8 ; i++ )
     asm("NOP");*/
  DEBUG_PUTS("1 Step");
  IO_UserLedOff(2);
}
static uint32_t downcnt;
static int8_t cnttmr;
 
void SignalNCB ( uint32_t arg )
{
  UNUSED(arg);
  if ( downcnt == 0 ) {
    MsTimerDelete(cnttmr);
  } else {
    UserPinSignal1();
    downcnt --;
  }
}

void UserPinSignalN(uint32_t n, uint32_t period)
{
  downcnt = n;
  cnttmr =  MsTimerSetRel ( MILLISEC_TO_TIMERUNIT(period), true, SignalNCB, 0 );
  SignalNCB(0);
}