#include "config/config.h"

#if defined USE_PULSE_SEQUENCER > 0

#include <string.h>

#include "rtc.h"
#include "timer.h"
#include "system/status.h"
#include "rfm/rfm.h"
#include "task/minitask.h"
#include "rfm/rfm_spi_interface.h"
#include "sequencer/ook_ringbuffer.h"
#include "sequencer/pulses.h"
#include "sequencer/analyzer.h"
#include "global_flags.h"

#if DEBUG_MODE > 0 && DEBUG_PULSES > 0
    #include "debug_helper.h"
#endif

/*******************************************************************************/
/* Note: All times/time intervals in this module are Mikroseconds unless       */
/*       explicitly stated otherwise                                           */
/*******************************************************************************/ 

#define GET_MIKROSECONDS()      BASTMR_GetRawValue(&BASTIM_HANDLE)

// Max Delta between two pulses
#define MAX_PULSE_DELTA     20000
#define MIN_PULSELENGTH     200

// Variables used for computation of pulse with and space width
static volatile uint16_t uHStart;           /* timestamp (us) for rising edge  */
static volatile uint16_t uLStart;           /* timestamp (us) for falling edge */
static volatile uint16_t pLLength;          /* Length of low pulse */
static volatile uint16_t pHLength;          /* Length of High pulse */
bool bFirstPulse;           /* flag for "first pulse of sequence" */
bool bRisingEdge;           /* flag for "rising edge detected" */

static int8_t PulseTimerID;

// sequence buffer consists of MAX_SEQNUM rows, every row of size MAX_SEQLEN
// first element holds the actual length, so only MAX_SEQLEN-1 elements can be used for the sequnce itself
uint8_t sequences[MAX_SEQNUM][MAX_SEQLEN];

// temporary sequence buffer, used while receiving a sequence
uint8_t s_temp[MAX_SEQLEN];

uint8_t s_actual;

/* Forward declarations -----------------------------------------------------*/
void PulseTimeout(uint32_t arg);

// Return the next free sequence, i.e. that sequence that has been written most long time ago
uint8_t NextSequence()
{
	if ( ++s_actual == MAX_SEQNUM ) s_actual = 0;
	return s_actual;
}

// Return the actual sequence, i.e. that sequence that has been written most recently
uint8_t ActualSequence()
{
	return s_actual;
}


// Copy the actual bit sequence in the next free row in the sequence array
// This array is filled round robin 
void CopySequence()
{

	uint8_t i, s_num;
	s_num = NextSequence();
	for (i=0;i<= *s_temp; i++ )
		sequences[s_num][i]=s_temp[i];
	*s_temp=0;
}

// Store one bit in the bit buffer.
// When bit buffer is full, the sequence is marked complete automatically
// otherwise end of sequence can be indicated by passing '\n'
void Store ( char c) 
{
   (*s_temp)++;
   if ( *s_temp == MAX_SEQLEN - 1 ) c = '+';

   s_temp[*s_temp] = c;	
   if ( c == '\n' || c == '+')  {
   		CopySequence();
		TaskNotify(TASK_SEQUENCE);
   }

}	

#if DEBUG_MODE && DEBUG_PULSES
    static void LongWrite(char chr, uint16_t pLength, uint16_t pDelta )
    {
       // COM_print_time_short(chr);
       DEBUG_PUTC(chr);
       print_decXXXXX(pLength);
       DEBUG_PUTC('/');
       print_decXXXXX(pDelta);
       CRLF();
    }
#endif


unsigned char InterpretPulse( void )
{
    uint16_t pDelta, pLength;

    if ( pulse_buf_has_data() ) {
        pulse_buf_get(&pLength, &pDelta);
        if        ( pLength < MIN_PULSELENGTH ) {	
            // shorter than 100us are too short
            #if DEBUG_MODE && DEBUG_PULSES 
                if ( debuglevel > 2 ) LongWrite('.', pLength, pDelta);
            #else
                ;
            #endif
        } else if ( pLength < 1100 ) { 	
            // 100-1100 us High pulse means "logic 1"	
            #if DEBUG_MODE && DEBUG_PULSES 
                if ( debuglevel > 3 ) LongWrite('H', pLength, pDelta); 
            #endif
            Store('1');
        } else if ( pLength < 1200 ) { 
            // between 1100 an 1200 us is illegal
            #if DEBUG_MODE && DEBUG_PULSES 
                if ( debuglevel > 2 ) LongWrite('<', pLength, pDelta);
            #else
                ;
            #endif										
        } else if ( pLength < 3150 ) { 
            // between 1200 ans 3150 us is regarded as "logic 0"
            #if DEBUG_MODE && DEBUG_PULSES 
                if ( debuglevel > 3 ) LongWrite('L', pLength, pDelta); 
            #endif
            Store('0');
        } else {
            #if DEBUG_MODE && DEBUG_PULSES 
                if ( debuglevel > 2 ) LongWrite ('>', pLength, pDelta);
            #else
                ;
            #endif										
        }										

        // Check for terminating pulse
        if ( pDelta >= MAX_PULSE_DELTA ) Store('\n');
    
        return 1;
        // Disable PulseTimeout
        MsTimerDelete(PulseTimerID);
    } else {
        return 0;	
    }
}


static uint16_t old = 0;
static uint16_t tim;
void HandleOOKInterrupt(uint16_t pin, uint16_t pinvalue, void *arg)
{
    UNUSED(pin); UNUSED(arg);
    
    // Interrupt on any lvl change of Int0-Pin
    // Check for High or low value
    if ( pinvalue ) {
        // High value
        // DEBUG_PUTC('r');
        /* Dump length of previous High Pulse */
        // DEBUG_PUTC('L');DEBUG_PUTC('n');DEBUG_PUTC(' ');tim=GET_MIKROSECONDS();print_decXXXXX(tim-old);CRLF();
        old = tim;
        if ( !bRisingEdge ) {
            /* notice rising edge to avoid logging of two consecutive rising edges due to glitches */
            bRisingEdge = true;

            // High value: record time and compute length of previous low pulse
            uHStart = GET_MIKROSECONDS();
            pLLength = uHStart - uLStart;
            #if DEBUG_MODE && DEBUG_PULSES
                if (debuglevel > 3) {
                    DEBUG_PUTC('H');print_decXXXXX(uHStart);CRLF();
                }
            #endif

            // If not the first pulse, then compute the total pulse duration
            if ( !bFirstPulse ) {
                // Determine total length of high and low Pulse
                uint16_t pDelta = pHLength + pLLength;
                #if DEBUG_MODE && DEBUG_PULSES
                    if (debuglevel > 3) {
                        DEBUG_PUTC('>');print_decXXXXX(pHLength);DEBUG_PUTC('/');print_decXXXXX(pDelta);CRLF();
                    }
                #endif
                // Only store high pulses that are longer than 200us and total length > 1000
                if ( pHLength > MIN_PULSELENGTH && pDelta > 1000 ) {
                    if ( !pulse_buf_put( pHLength, pDelta ) ) {
                        // Buffer full
                        #if DEBUG_MODE && DEBUG_PULSES
                            if (debuglevel > 2) DEBUG_PUTC('-');
                        #endif
                    }
                    /* Set Timeout for next pulse to MAX_PULSE_DETA ms */
                    MsTimerReUseRel( PulseTimerID, MIKROSEC_TO_TIMERUNIT(MAX_PULSE_DELTA),0, PulseTimeout, 0 );
                    #if DEBUG_MODE && DEBUG_PULSES
                        if (debuglevel > 3) {
                            DEBUG_PUTC('L');print_decXXXXX(uLStart);CRLF();
                        }
                    #endif
                    
                    /* Request processing of pulse */
                    TaskNotify(TASK_PULSE);
                } else {
                    /* No valid pulse ( ie high or low part too short */
                    #if DEBUG_MODE && DEBUG_PULSES
                        if (debuglevel > 2 )  {
                            if ( pHLength <= MIN_PULSELENGTH )
                                DEBUG_PRINTF("%d:noH",pHLength);
                            else
                                DEBUG_PRINTF("%d:noL",pDelta);
                        }
                    #endif
                }
            } 
            bFirstPulse = false;
        } else {
            #if DEBUG_MODE && DEBUG_PULSES
            if (debuglevel > 3) DEBUG_PUTC('h');
            #endif
        }			  
    } else {
        // low value: Record Timer value and Flag Pulse
        // DEBUG_PUTC('f');
        /* Dump length of previous High Pulse */
        // DEBUG_PUTC('H');DEBUG_PUTC('n');DEBUG_PUTC(' ');tim=GET_MIKROSECONDS();print_decXXXXX(tim-old);CRLF();
        old = tim;
        if ( bRisingEdge ) {
            /* notice falliing edge to avoid logging of two consecutive falling edges due to glitches */
            bRisingEdge = false;

            // Record time of falling edge and compute time of previous high pulse
            uLStart = GET_MIKROSECONDS();
            pHLength = uLStart-uHStart;
            #if DEBUG_MODE && DEBUG_PULSES
                if (debuglevel > 3) {
                    DEBUG_PUTC('L');print_decXXXXX(uLStart);CRLF();
                }
            #endif
        } else {
            #if DEBUG_MODE && DEBUG_PULSES
                if (debuglevel > 3) DEBUG_PUTC('l');
            #endif
        }
    }
}

void PulseTimeout(uint32_t arg)
{
    UNUSED(arg);
    // When this vector is executed, the pulse wait timeout has been reached.
    // We assume that a pulse sequence ended.

    #if DEBUG_MODE && DEBUG_PULSES
        if (debuglevel > 2) DEBUG_PUTC('x');
    #endif

    // Assume maximal Pulse duration
    uint16_t pDelta = MAX_PULSE_DELTA;

    // Only Store pulse, if not the first one and duration is above minimum duration
    if ( !bFirstPulse ) {
        // Determine Time diff to previous Pulse

        // Only store pulses that are longer than 100us
        if ( pHLength > 200 ) {
            if ( !pulse_buf_put( pHLength, pDelta ) ) {
                // Buffer full
                #if DEBUG_MODE && DEBUG_PULSES
                    if (debuglevel > 2) DEBUG_PUTC('-');
                #endif
            }
        }
    }
    // Next Pulse will initiate a new sequence
    bFirstPulse = true;

    // Request processing of pulse
    TaskNotify(TASK_PULSE);
}

// Initialisation: Mark al sequnces in the sequence buffer as empty
void PulsesInit(void)
{
    uint8_t i;
    for ( i=0; i < MAX_SEQNUM; i++ )
            sequences[i][0] = 0; 

    PulseTimerID = MsTimerAllocate(NO_TIMER_ID);
    #if DEBUG_MODE > 0 && DEBUG_PULSES > 0 
        if ( PulseTimerID == NO_TIMER_ID )
            DEBUG_PUTS("PulsesInit - Error: Cannot allocate TimerID");
    #endif
}

// Task handler
void task_handle_pulse(uint32_t arg )
{
    UNUSED(arg);
    if ( InterpretPulse() )  TaskNotify(TASK_PULSE);
}

#endif //#if defined(USE_PULSE_SEQUENCER)