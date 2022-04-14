#include "config/config.h"
#include "error.h"

#if USE_PULSE_SEQUENCER > 0 

#include "timer.h"
#include "eeprom.h"
#include "rtc.h"
#include "rfm/rfm.h"
#include "sequencer/ook_ringbuffer.h"
#include "system/status.h"
#include "system/util.h"
#include "sequencer/pulses.h"
#include "sequencer/analyzer.h"
#include "global_flags.h"
#include "ui/lcd_status.h"

#include "debug_helper.h"

uint32_t aflags;                                // Flag dword we use for OOK-related flags
uint16_t ofstemp;    				// (temp+40) * 10

uint8_t  lowbatt;
uint8_t  relhum;      			   	// relative humidity in %
uint8_t  transmitter_id;  			// id of transmitter
uint8_t  OOK_retries;				// Number of retries in case of unsuccessful OOK-Reception

union CTIME_struct temp_received;               // Date & Time of last Temp transmission
union CTIME_struct rh_received;                 // Date & Time of last RH transmission

#if DEBUG_RETRY > 0
	bool bSimulateRetry;
#endif

static uint8_t RestartTimerID = NO_TIMER_ID;    // TimerID of restart timer
static uint8_t OokOffTimerID  = NO_TIMER_ID;    // TimerID of OOK timeout timer

int16_t   avg_waitms=0;                         // average time [ms} between start of receiver and reception of first correct data packet
uint16_t  OOK_RetryCount=0;                     // Counter for missed regular OOK transmissions
/*******************************************************************************
 * Return the time ( in seconds ) before OOK_Receiver will be restarted
 * SEC_TIMER_RESTART is used for this 
 ******************************************************************************/
uint32_t FreeTimeSlot(void) 
{
    uint32_t remaining_s;

    if ( !SECTimerGetRel(RestartTimerID, &remaining_s ) ) remaining_s = 0xffff;
 
    #if DEBUG_MODE > 0
        if ( console_debuglevel > 1 ) DEBUG_PRINTF("FreeTimeSlot=%d\n", remaining_s);
    #endif

    return remaining_s;
}


/*
Sendetelegrammformate TX18
 - jede Sequenz ist genau 52 bit lang. 
 - Organisation in Nibbles.
 - Das 13. Nibble enthält als CRC die Summe der 12 Nibbles davor
 - Low Batt wird nicht übertragen, kann man aber daran erkennen, daß ein Temp Wert > 100 gesendet wird
Relative Feuchte ( in %, Bereich 01 - 99 )
               +-------------------------------------------- 01 für "Rel. Feuchte" 
               |          +--------------------------------- Dieses Nibble hat bis zum nächsten Reset immer den gleichen wert
               |          |  +------------------------------ Diese drei bit haben bis zum nächsten Reset immer den gleichen wert
               |          |  |+----------------------------- für drei Sendungen 0, dann fpr eine Sendung 1 
               |          |  ||      +---------------------- 0= Initialer Dauermode, dh. senden alle 8 sec 
               |          |  ||	     |                       1= Normalbetrieb, d.h. senden ca. alle 128 sec
               |          |  ||      |        +------------- RH Zehner 
               |          |  ||      |        |          +-- RH Einer
               |          |  ||      |        |          |+- ?
              --        --------     |        |          ||
00000110 06 00010001 11 00111111 3f 11100101 e5 00101100 2c 10101101 ad 0010 OK CRC OK Rel.H 52%
--------        ----                                                    ----
    |             + Dieses Nibble hat bis zum nächten Reset               |
	|               immer den gleichen Wert                               |
	+ Dieses Byte ist immer 06, scheint eine ID für TX18 zu sein          + CRC ist die Summe der 12 Nibbles davor


Temperatur ( aktueller Wert in °C plus 40, Bereich: -40.0  .. 59.9  )
               +-------------------------------------------- 00 für "Temperatur" 
               |            +------------------------------- Zufälligs Bitmuster nach dem Einschalten, identifiziert Sender
               |            |        +---------------------- 0= Initialer Dauermode, dh. senden alle 8 sec 
	       |            |        |                       1= Normalbetrieb, d.h. senden ca. alle 128 sec
               |            |        | +-------------------- 0= Vbatt ok 1=Batteriewarnung bei kleiner 2.6V, nur bei Temp und RH gültig
               |            |        | |      +------------- Temperatur Zehner plus 4
               |            |        | |      |          +-- Temperatur Einer
               |            |        | |      |          |+- Temperatur Zehntel
              --        --------     | |      |          ||
00000110 06 00000001 01 00111101 3d 11100110 e6 00011001 19 10011110 9e 1100 OK CRC OK Temp.21.9
                                        ----    --------                ----
									   Data1   Data2Data3                + CRC ist die Summe der 12 Nibbles davor

Ungenutzt: Diese 13-Nibble Seqenzen werden häufig mitgesendet, enthalten aber keine Nutzdaten

00000110 06 01100001 61 00111111 3f 11100000 e0 00000000 00 11111111 ff 1011 OK CRC OK Unkn.
00000110 06 01110001 71 00111111 3f 11111111 ff 11100000 e0 00000001 01 1101 OK CRC OK Unkn.

Wenn VBatt unter ca. 2.1V fällt, dann wird als Temp 0xfff und als RH 0xaa ( = 126.5° und 110% ) dauerhaft gesendet
Initialer Dauermode dauert ca. 15 min

*/

#define TX18_OFS_MSGTYPE		9                       // Offset where the MSGTYPE is stored
#define TX18_OFS_MSGMASK		0x03                    // Number of Bits to mask out
#define TX18_OFS_XMIT_ID		13			// Offset of the transmitter id
#define TX18_OFS_MODEBIT		26			// Offset of "Normal Mode" bit
#define TX18_OFS_BATTWARN		28			// Offset of "Battery Warning" ( when Vbatt drops below 2.6V )

#define TX18_OFS_DATA1			29			// Offset of Data nibble 1
#define TX18_OFS_DATA2			33			// Offset of Data nibble 2
#define TX18_OFS_DATA3			37			// Offset of Data nibble 3
#define TX18_OFS_CRC			49			// Offset of CRC nibble

#define TX18_DELTA_T_INIT		8			// TX18 is transmitting every 8 sec in initial phase
#define TX18_DELTA_T_NORM		129			// TX18 is transmitting every 129 sec in normal phase
								// manual says 128, but 129 is correct

#define CORRECT_LENGTH 53


uint8_t GetBitAt ( unsigned char *s_ptr,  uint8_t ofs )
{
	return s_ptr[ofs]=='1' ? 1 : 0;
}

uint8_t GetNibbleAt ( unsigned char *s_ptr,  uint8_t ofs )
{
    uint8_t i, ret;
    ret=0;
    for ( i = 0; i < 4; i++ ) {
            ret = ( ret << 1 ) | GetBitAt(s_ptr,ofs+i);
    }
    return ret;
}

uint8_t GetByteAt ( unsigned char *s_ptr,  uint8_t ofs )
{
    return GetNibbleAt( s_ptr, ofs ) << 4 | GetNibbleAt( s_ptr, ofs+4 );
}


// CRC computation: the 13th nibble must be the sum of the 12 preceeding nibbles
uint8_t CheckCrc(uint8_t s_num)
{
    uint8_t	*ptr = sequences[s_num];
    uint8_t i;
    uint8_t nibble;
    uint8_t crc = 0;
    uint8_t ret;

    for ( i=0;i<12;i++) {
        nibble = GetNibbleAt(ptr, 1+i*4 );
        crc += nibble;
    }
    nibble = GetNibbleAt(ptr, TX18_OFS_CRC ); // crc nibble in transmission

    ret = (crc & 0x0f) == nibble;

    #if DEBUG_MODE && DEBUG_PULSES
        if ( console_debuglevel > 0 ) {
            DEBUG_PRINTF(" CRC %s", ret ? "OK" : "Fail"); 
        }
    #endif

    return ret;
}

// Return the transmitter ID ( Bit 13-20 ) 
uint8_t GetTransmitterID (uint8_t s_num)
{
    uint8_t	*ptr = sequences[s_num];
    return GetByteAt(ptr, TX18_OFS_XMIT_ID);
}

/******************************************************************************
 * Compute the average waittime from starting of ook receiver to reception of
 * first ook data packet
 *****************************************************************************/
static void ComputeAvgWaitTime(void)
{

    /* stop stopwatch and compute average time */
    uint32_t dt = RTC_StopWatch_Stop();
    avg_waitms = average4(avg_waitms, dt);

    #if DEBUG_MODE && DEBUG_PULSES 
            if ( console_debuglevel > 0 ) DEBUG_PRINTF("Waittime=%d, new Avg lead time: %dms\n", dt, avg_waitms);
    #endif
}

/*************************************************************************
 * Compute listen timeout and default waittime to next reception in dependance from OOK_status
 * also reset number of retries when in normal reception mode ( ie no previous missing
 * receptions
 ************************************************************************/
static bool Get_Default_OOK_Interval(uint16_t *timeout, uint16_t *waittime)
{
    /* reset retries, if in normal reception mode ( ie not retrying ) */
    if (OOK_Receiver_retrying() ) 
        OOK_RetryCount++;
    else
        OOK_retries = 0;

    switch (OOK_status) {
        case OOK_STATUS_WAITFORSYNC:
            /*  listen time and timeout to maximum when waiting for first ook reception */
            *timeout  = TX18_TIMEOUT_INTERVAL;
            *waittime = SECTIMER_MAX;
            return true;
        case OOK_STATUS_INITIALPHASE:
            /* listen time constant and independent from "retries" in TX18 initial phase */
            *timeout  = TX18_MAX_LISTEN_TIME;
            *waittime = TX18_DELTA_T_INIT;
            return true;
        case OOK_STATUS_NORMALPHASE:
            /* Enlarge listen time with every unsuccessful retry by 4 seconds */
            *timeout  = TX18_MAX_LISTEN_TIME +  TX18_ENLARGE * OOK_retries; 
            *waittime = TX18_DELTA_T_NORM    - (TX18_ENLARGE * (OOK_retries+1))/2;
            return true;
        default:
            #if DEBUG_MODE && DEBUG_PULSES 
                DEBUG_PUTS("Get_Default_OOK_Interval... in invalid state");
            #endif
            return false;
    }
}

/*************************************************************************
 * compute the waittime to listen to next reception after an valid data
 * packet has been received
 * This waittime regards the configured number of transmission, that shall
 * be omitted
 ************************************************************************/
static uint16_t Get_Next_OOK_Interval(void)
{

    /* reset retries, if in normal reception mode ( ie not retrying ) */
    if (!OOK_Receiver_retrying() ) OOK_retries = 0;

    switch (OOK_status) {
        case OOK_STATUS_INITIALPHASE:
           return ((uint16_t)TX18_DELTA_T_INIT)*(1+config.tx18_skip_init) - TX18_XMIT_DURATION/2 - TX18_XMIT_PREFIX;
        case OOK_STATUS_NORMALPHASE:
           return ((uint16_t)TX18_DELTA_T_NORM)*(1+config.tx18_skip_norm) - TX18_XMIT_DURATION/2 - TX18_XMIT_PREFIX;
        default:
            #if DEBUG_MODE && DEBUG_PULSES 
                DEBUG_PUTS("Get_Next_OOK_Interval... in invalid state");
            #endif
            return 0;
    }
}

bool InterpretSequence(uint8_t s_num)
{
    uint8_t  *ptr = sequences[s_num];
    uint8_t  type = GetNibbleAt(ptr, TX18_OFS_MSGTYPE ) & TX18_OFS_MSGMASK;
    uint16_t work;
    uint16_t next_transmission; 		// delta t for next transmission [s]

    if (!CheckCrc( s_num ) ) return false;
    
    if ( transmitter_id ) {
        if ( transmitter_id != GetTransmitterID( s_num ) ) {
            #if DEBUG_MODE && DEBUG_PULSES 
                    if ( console_debuglevel > 0 ) DEBUG_PUTS(" Wrong Transmitter ID");
            #endif
            return false;
        } else {
            // When reenabling after Master-Timeout, the stored transmitter_id and the one found now are equal
            // So Check this case here: If so,set state to initial
            if ( OOK_status == OOK_STATUS_WAITFORSYNC ) 
                SetOOKStatus(OOK_STATUS_INITIALPHASE, false);
        }
    } else {
       transmitter_id = GetTransmitterID( s_num );
       // Save transmitter ID
       SetOOKStatus(OOK_STATUS_INITIALPHASE, false);
    } 

    // Change to normal mode ?
    if ( OOK_status == OOK_STATUS_INITIALPHASE && GetBitAt(ptr, TX18_OFS_MODEBIT ) == 1 ) {
        SetOOKStatus(OOK_STATUS_NORMALPHASE, false);
    }
    
    // AFLAG_GOT_VALID_DATA becomes set and time of next transmission is adjusted, if one valid data packet has been received
    // This can also be an packet of type Unkonwn, not only Temp or RH
    if ( !(IsFlagBitSet(aflags,AFLAG_GOT_VALID_DATA)) ) {
        SetFlagBit(aflags,AFLAG_GOT_VALID_DATA);
        next_transmission = Get_Next_OOK_Interval();
        if (next_transmission) SecTimerReSetRel(RestartTimerID, next_transmission);
        #if DEBUG_MODE && DEBUG_PULSES 
                if ( console_debuglevel > 0 ) DEBUG_PRINTF("Next OOK-Reception in %d Secs\n", next_transmission);
        #endif
    }

    switch (type ) {
        case 0: work = GetNibbleAt(ptr,TX18_OFS_DATA1) * 10 + GetNibbleAt(ptr,TX18_OFS_DATA2);
            work = work * 10 + GetNibbleAt(ptr,TX18_OFS_DATA3);
            ofstemp = work;
            lowbatt = GetBitAt(ptr,TX18_OFS_BATTWARN);
            SetTemp( (((int16_t)ofstemp)-400)*10 );
            LCD_DisplayStatus(LCD_STATUS_TEMP);
            compress_datetime ( &temp_received, &rtc );
            SetFlagBit(aflags, AFLAG_GOT_TEMP_BIT);
            if ( RTC_StopWatch_InUse() ) ComputeAvgWaitTime();
            break;
        case 1: work = GetNibbleAt(ptr,TX18_OFS_DATA1) * 10 + GetNibbleAt(ptr,TX18_OFS_DATA2);
            relhum = work;
            lowbatt = GetBitAt(ptr,TX18_OFS_BATTWARN);
            compress_datetime ( &temp_received, &rtc );
            SetFlagBit(aflags, AFLAG_GOT_RH_BIT);
            if ( RTC_StopWatch_InUse() ) ComputeAvgWaitTime();
            break;
    }
    if ( lowbatt ) 
        CTL_error |= ERR_SENSOR_BATT;
    else
        CTL_error &= ~ERR_SENSOR_BATT;
    
    #if DEBUG_MODE && DEBUG_PULSES
        int16_t t100;
        if ( console_debuglevel > 0 ) {
            DEBUG_PUTC(' ');
            switch (type ) {
                case 0: DEBUG_PRINTF("Temp ");
                    t100 = (ofstemp - 400);
                    print_decSXX((int8_t)(t100/10));
                    DEBUG_PUTC('.');
                    int8_t frac = t100 - (t100/10)*10;
                    print_decXX( (frac < 0 ? frac * -1 : frac ) );
                    break;
                case 1: DEBUG_PRINTF("RH "); 
                    print_decXX(relhum);
                    DEBUG_PUTC('%');
                    break;
                case 2: DEBUG_PRINTF("Unkn2."); break;
                case 3: DEBUG_PRINTF("Unkn3."); break;
            }
        }
    #endif
    
    return true;
}

void AnalyzeSequence(uint8_t s_num)
{
    uint8_t *ptr;
    uint8_t i;
    uint8_t shiftbyte=0;
    unsigned char c;

    ptr = sequences[s_num];
    
    #if DEBUG_MODE && DEBUG_PULSES
        uint8_t mydebug;
        // Dump correct sequences, if console_debuglevel > 0
        // Dump seuqences of wrong length if console_debuglevel > 1
        if ( *ptr != CORRECT_LENGTH ) 
            mydebug = console_debuglevel > 1;
        else 
            mydebug = console_debuglevel > 0;
    #endif

    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
        #if DEBUG_MODE && DEBUG_PULSES 
            COM_print_time('B', false);
        #else
            COM_print_time('B', true);
        #endif
    #endif
    for ( i=1;i<= *ptr; i++ ) {
        c = ptr[i];
        if ( c == '0' || c == '1' ) {
            #if DEBUG_MODE && DEBUG_PULSES
                if ( mydebug ) DEBUG_PUTC(c);
            #endif
            shiftbyte |= (c == '0' ? 0:1 );
            if ( (i & 0x07) == 0 ) {
                #if DEBUG_MODE && DEBUG_PULSES 
                    if ( mydebug ) {
                        DEBUG_PUTC(' '); 
                        print_hexXX(shiftbyte);
                        DEBUG_PUTC(' '); 
                    }
                #endif
                shiftbyte = 0;
            } else {
                shiftbyte <<= 1;
            }
        } else {
            #if DEBUG_MODE && DEBUG_PULSES
                if ( mydebug ) {
                    if ( c == '\n' || c == '+') {
                        if ( c == '+') DEBUG_PUTC(c);
                    } else DEBUG_PUTC(c);	
                }
            #else
                ;
            #endif
        }
    }
    if ( *ptr == CORRECT_LENGTH ) {
        InterpretSequence(s_num);
    } else {
        #if DEBUG_MODE && DEBUG_PULSES
            if ( mydebug) DEBUG_PRINTF(" Bad len");
        #else
            ;
        #endif
    }
    #if DEBUG_MODE && DEBUG_PULSES
        if (mydebug) CRLF();
    #endif
}


void AnalyzerTimeout(uint32_t arg)
{
    UNUSED(arg);

    // This Routine is called by SEC_TIMER_OOK_OFF-Event or by AnalyzerCheckForShutdown
    // OOK-Receiver could have been stopped by shutdown, so don't stop again,
    // because the FSK-Transceiver could already be running
    if (OOK_Receiver_running()) Stop_OOK_Receiver();

    if ( IsFlagBitSet(aflags, AFLAG_GOT_VALID_DATA)  
            #if DEBUG_RETRY > 0
                && !bSimulateRetry )
            #else
                )
            #endif
    {
            // if valid data has been received, reset retry-mode
            ClearFlagBit(aflags, AFLAG_OOK_RETRY);
            CTL_error &= ~ERR_OOK_INOP;
            #if DEBUG_MODE && DEBUG_PULSES
                if ( console_debuglevel > 0 ) DEBUG_PRINTF("Timeout,Sleep ");
            #endif
    } else {
        /* We got no valid data until timeout */
        
        if (OOK_status == OOK_STATUS_WAITFORSYNC || OOK_status == OOK_STATUS_INHIBITED ) {
            /* If we were waiting for sync and did not get any data, assume there is no transmitter */
            /* Set global error flag and we are done */
            SetOOKStatus(OOK_STATUS_NO_PEER, false);
            CTL_error |= ERR_OOK_INOP;
            #if DEBUG_MODE && DEBUG_PULSES
                if ( console_debuglevel > 0 ) DEBUG_PRINTF("No valid signal in SYNC Phase, no transmitter error ");
            #endif
            return;
        }

        /* if we are in initial or normal sync phase increment retry counter and listen again */
        if ( ++OOK_retries <= TX18_RETRIES ) {
            /* unsuccessful OOK-Receive in this timeslot: Set Retry-Flag and try again, 
            * timer to listen again was setup before */
            SetFlagBit(aflags, AFLAG_OOK_RETRY);

            // If retries left: Restart time has already been set in Start_OOK_Receiver
            #if DEBUG_MODE && DEBUG_PULSES
                if ( console_debuglevel > 0 ) DEBUG_PRINTF("Retries=%d, Try again later\n", OOK_retries); 
            #endif
        } else {
            /* maximum number of unsuccessful retries reached, reinitialize receiver, i.e. */
            /* try to resync again */
            AnalyzerInit();
            #if DEBUG_MODE && DEBUG_PULSES
                if ( console_debuglevel > 0 ) DEBUG_PRINTF("No more retries, try to resync ");
            #endif
            /* OOK Restart is done in main */
        }
    }
    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
        if ( (aflags & AFLAG_DATA_MASK ) == AFLAG_DATA_MASK ) {
            COM_print_time('s', false);
        } else {
            COM_print_time('o', false);
            DEBUG_PUTC(' ');
            print_decXX(OOK_retries);
        }
        print_decXXXXX(FreeTimeSlot());CRLF();
    #endif
}

/**************************************************************************
 * The TX18-Listener can be shut down, if temperature and RH both have been
 * received.
 * Just shut down the receiver, the wakeup for the next transmission will be
 * scheduled by AnalyzerTimeout
 **************************************************************************/
void AnalyzerCheckForShutdown(void)
{
    // never shutdown in force mode
    if ( OOK_Receiver_ForceMode() ) return;
            
    if ( (aflags & AFLAG_DATA_MASK ) == AFLAG_DATA_MASK ) {
        Stop_OOK_Receiver();
        #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
                COM_print_time('s', true);
        #endif
        // Dont wait for regular timeout, do the neccessary steps now
        SecTimerDelete(OokOffTimerID); OokOffTimerID = NO_TIMER_ID;
        AnalyzerTimeout(0);
    } 
}



/**************************************************************************
 * Start the Receiver in OOK-Mode
 * \param do_measure - if != 0 track the time, that is spended from starting
 *                     ook receiver until reception of first correct record
 **************************************************************************/
void Start_OOK_Receiver(uint32_t do_measure)
{
    uint16_t ook_timeout;
    uint16_t ook_interval;

    /* Don't start twice */
    if ( IsFlagBitSet(gflags, GFLAG_OOK_BIT) ) return;

    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
        COM_print_time('O', false);
        print_decXXXXX(ook_timeout);
        CRLF();
    #endif

    // Clear sequence buffer
    *s_temp = 0;

    // Clear Pulse Buffer
    pulse_buf_reset();

    // assume logic level 0 at input pin, no retry mode
    bRisingEdge   = false;
    bFirstPulse     = true;

    // no data received so far
    aflags &= ~( AFLAG_DATA_MASK );

    // No Interval for next transmission has been set
    ClearFlagBit(aflags, AFLAG_GOT_VALID_DATA);

    /* No Checks in OOK receiver force mode */
    if ( !OOK_Receiver_ForceMode() ) {

        /* Get timeout for actual listen and default interval to next listen */
        if ( !Get_Default_OOK_Interval(&ook_timeout, &ook_interval) ) {
            #if DEBUG_MODE && DEBUG_PULSES > 0
                 DEBUG_PUTS("OOK receiver not started");
            #endif
            return;
        }

        OokOffTimerID = SecTimerSetRel(ook_timeout, 0, AnalyzerTimeout,0 );
        #if DEBUG_MODE && DEBUG_PULSES > 1
             DEBUG_PRINTF("Next OOK reception in %d seconds\n", ook_interval);
        #endif
        if ( NO_TIMER_ID == SecTimerReUseRel(RestartTimerID, ook_interval, 0, Start_OOK_Receiver, 1 ) )
            log_error("Restart Timer not initialized!");
    }

    /* record start time, if desired */
    if ( do_measure > 0 ) RTC_StopWatch_Start();
 
    /* Set "OOK receiver activated flag" */
    SetFlagBit(gflags, GFLAG_OOK_BIT);

    /* Start OOK receiver */
    TMR_Aquire(&BASTIM_HANDLE);
    RFM_OOK_init();
    RFM_OOK_on();
    LCD_DisplayStatus(LCD_STATUS_RADIO);
}

void Stop_OOK_Receiver(void)
{
    /* Only stop, if started before */
    if ( !IsFlagBitSet(gflags, GFLAG_OOK_BIT) ) return;

    /* Stop watch may be still running in case of timeout without reception, so stop it now */
    if ( RTC_StopWatch_InUse() ) RTC_StopWatch_Stop();
    
    TMR_Release(&BASTIM_HANDLE);
    RFM_OOK_off();

    // Receiver is deactivated
    ClearFlagBit(gflags, GFLAG_OOK_BIT);
    ClearFlagBit(aflags, AFLAG_OOK_RETRY);

    LCD_DisplayStatus(LCD_STATUS_RADIO);
}

/******************************************************************************
 * Reset to OOK Receiver to Initial (unsync'ed) state
 * change
 *****************************************************************************/
void Reset_OOK_Receiver(void)
{
    SecTimerDelete(RestartTimerID);
    SecTimerDelete(OokOffTimerID); OokOffTimerID = NO_TIMER_ID;
    Stop_OOK_Receiver();
    SetOOKStatus(OOK_STATUS_WAITFORTSYNC, false);

}

/******************************************************************************
 * Just for Test Purposes:
 * Retime the OOK receiver to a certain second
 *****************************************************************************/
bool Retime_OOK_Receiver(uint32_t sec)
{
    uint32_t delta_s = RTC_GetSecond();

    if ( delta_s >= sec ) sec += 60;
    delta_s = sec - delta_s;
    
    bool ret = NO_TIMER_ID != SecTimerReUseRel(RestartTimerID, delta_s, 0, Start_OOK_Receiver, 1 );
    if ( !ret ) log_error("Restart Timer not initialized!");

    return ret;
}

void AnalyzerInit(void)
{
    transmitter_id=0;  					// no transmitter ID so far

    temp_received.timeint=0;                                // no temp received so far
    rh_received.timeint=0;                                  // no rh received so far
    ResetMinMaxTemp((void*)0);                              // Reset min/mx of the day ***0002***
    RestartTimerID = SecTimerAllocate(RestartTimerID);      // Allocate a fixed TimerID for Restart Timer

    if ( config.OOK_mode == 0 ) {
        /* OOK mode is inhibited */
        SetOOKStatus(OOK_STATUS_INHIBITED, false);  
        CTL_error |= ERR_OOK_INOP;
    } else {
        /* wait for next master time synchronization, then start ook sync */
        SetOOKStatus(OOK_STATUS_WAITFORTSYNC, false);  
        CTL_error &= ~ERR_OOK_INOP;
    }
}

void task_handle_ana(uint32_t arg)
{
    UNUSED(arg);

    AnalyzeSequence(ActualSequence());
    AnalyzerCheckForShutdown();
}


#endif //#if defined(USE_PULSE_SEQUENCER)
