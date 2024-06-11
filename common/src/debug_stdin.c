/**
  ******************************************************************************
  * @file    debug_stdin.c
  * @author  Rainer
  * @brief   When stdin us used as debug input, do a periodical scan of
  *          stdin for availabler input characters and store the to a linear
  *          buffer until CRLF is read
  *
  ******************************************************************************
  */

/** @addtogroup DEBUG_STDIN
  * @{
  */


#include "config/config.h"

#if DEBUG_FEATURES > 0  && DEBUG_DEBUGIO > 0

#include <debugio.h>
#include "circbuf.h"
#include "hardware.h"
#include "timer.h"
#include "rtc.h"
#include "task/minitask.h"
#include "debug_outbuf.h"

/* Private define ------------------------------------------------------------*/
#define INBUF_SIZE  64                                /* Size of input buffer */
#define STDIN_SCAN_INTERVAL_MS  500                /* scan stdin every 500 ms */

/* Private variables ---------------------------------------------------------*/
static uint8_t inbuf[INBUF_SIZE];
static uint8_t stdinTimer;

/* public variables */
LinBuffT stdin_line;       /* buffer to collect characters until CRLF is read */


/******************************************************************************
 * @brief We received a character, so put it to input buffer
 *        Handle DEL and CR/LF specially
 *****************************************************************************/
void stdin_rx (uint8_t ch)
{

    // DEBUG_PRINTF("got %02x",ch);
    switch ( ch ) {
        case 0x07f: 
            /* Del = remove DEL and last character */
            LBUF_DEL(stdin_line);
            break;
        case '\n':
            /* CR(LF) : Call input handler */
            TaskNotify(TASK_COM);
            break;
        default:
            LinBuff_Putc(&stdin_line, ch );
    }
    
}



/******************************************************************************
 * @brief this functions is periodically called to check availabilty of 
 *        characters on stdin
 *****************************************************************************/
void task_handle_in( uint32_t arg )
{
    uint8_t c;
    UNUSED(arg);

    while ( debug_kbhit() ) { 
        c = (uint8_t)debug_getch();
        stdin_rx(c);
    }
}

/******************************************************************************
 * @brief callback for the std input timer: Just notify the reader task 
 *****************************************************************************/
void stdin_timer( uint32_t arg )
{
    UNUSED(arg);
    TaskNotify(TASK_STDIN);
}



/******************************************************************************
 * @brief Init the linear input buffer and set up timer for periodical scan
 *        of stdin
 *        
 *****************************************************************************/
void stdin_Init ( void )
{
    LinBuff_Init (&stdin_line, INBUF_SIZE,  inbuf );
    stdinTimer = MsTimerSetRel(MILLISEC_TO_TIMERUNIT(STDIN_SCAN_INTERVAL_MS), true, stdin_timer, 0);
}



#endif // DEBUG_FEATURES > 0  && DEBUG_DEBUGIO > 0

/**
  * @}
  */
