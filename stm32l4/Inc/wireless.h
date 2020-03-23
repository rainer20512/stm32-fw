/**
  ******************************************************************************
  * @file    wireless.h
  * @author  Rainer
  * @brief   All that wireless communication stuff
  *
  ******************************************************************************
  * @addtogroup Wireless
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WIRELESS_H
#define __WIRELESS_H

#include "debug.h"
#include "rtc.h"


#ifdef __cplusplus
 extern "C" {
#endif

/* 
 * One tick in the wireless section is 1/256 of a second.
 * The Macro MILLISEC_TO_TIMERUNIT does the translation from milliseconds to timer ticks
 * This macro MUST BE defined !
 */

#ifndef MILLISEC_TO_TIMERUNIT
  #error MILLISEC_TO_TIMERUNIT undefined!
#endif

#define WLTIME_SYNC_AT          (MILLISEC_TO_TIMERUNIT(750)) // Sync is broadcasted at 750ms
#define WLTIME_SYNC_TIMEOUT     (MILLISEC_TO_TIMERUNIT(62))  // Sync Timeout is set automatically so, that there 
                                                             // is 1/4 tolerance before and 3/4 after Sync time
#define WLTIME_SYNC (WLTIME_SYNC_AT \
                        - WLTIME_SYNC_TIMEOUT / 4)           // Start of Sync Window before sync, 1/4 before expected sync time 3/4 after

#define WLTIME_START            (MILLISEC_TO_TIMERUNIT(150)) 		// communication start, approx 150 ms
#define WLTIME_NEXT             (MILLISEC_TO_TIMERUNIT(40)) 		// communication follow-up during timeslot, approx 50 ms
#define WLTIME_TIMEOUT          (MILLISEC_TO_TIMERUNIT(100)) 		// slave RX timeout, approx 100 ms
//	#define WLTIME_STOP 	(MILLISEC_TO_TIMERUNIT(900)) 		// last possible communication
#ifdef ALARM
    #define WLTIME_ALARM0 	(MILLISEC_TO_TIMERUNIT(50)) 		// alarm slot#0, approx from 50-100ms 
    #define WLTIME_ALARM1 	(MILLISEC_TO_TIMERUNIT(100)) 		// alarm slot#1, approx from 100-150ms
    #define WLTIME_DELAYED 	(WLTIME_START-MILLISEC_TO_TIMERUNIT(20)) // litte shorter than WLTIME_START to allow preparation for START
#endif

#define WL_OOF_SYNC		6			// After hwo many minutes without sync we will assume that sync ist lost ?
#define WAITMIN_INITIAL 	(-10)			// How long do we initially wait after unsuccessful sync phase ?
#define WL_RESYNC_INTERVAL	1			// How long will we listen until assuming, that no sync possible?
#define WAITMIN_DELTA		10			// Increment of waittime between two tries

// #define WLTIME_LED_TIMEOUT      (MILLISEC_TO_TIMERUNIT(300))   // packet blink time

// *** Chng 050 *** changed to 5 ( was 3) - More packets not possible due to end of time slot - see xlsx in doc-dir //
#define MAX_DATA_PACKETS	5			// Max number of data packets pairs to be xchanged in one time slot
										// ie one packet from slave to master and one back
										// When increasing, you also have to increase the mask bis for the packet 
										// number in data packets, also check the time frame for enough reserve to
										// xfer additional data pairs
// *** Chng 050 *** //
#define PACKET_COUNTER_BITS 3			// Number of bits used in MSB part of first rfm12-data byte
#define PACKET_COUNTER_SHIFT (8-PACKET_COUNTER_BITS )
#define PACKET_COUNTER_MASK ((1<<PACKET_COUNTER_SHIFT) - 1 )



#define WLESS_IN_SYNC()  ( time_sync_tmo > 0 )


typedef enum {
    WL_TIMER_NONE=0,
    WL_TIMER_FIRST=1,		// first packet
    WL_TIMER_NEXT=2,		// follow-up packets in timeslot	
    WL_TIMER_RX_TMO=3,		// Timeout
    WL_TIMER_SYNC=4,            // Sync packet
    WL_TIMER_ALARM=5   
} wirelessTimerCase_t;

typedef enum {
    WL_PACKET_SYNC=0,           // Sync Packet type
    WL_PACKET_DATA=1,		// Data Packet
    WL_PACKET_ALARM=2		// Alarm Packet Type
} wirelessPacket_t;


extern int8_t time_sync_tmo;
extern uint32_t WL_BufPtr;
extern uint8_t wl_force_addr1;
extern uint8_t wl_force_addr2;
extern uint32_t wl_force_flags;
extern uint8_t wl_skip_sync;
extern uint16_t missed_syncs;	  //!< syncs missed since last reset *** Chng 052 ***
extern int16_t avg_skew_00;       // Average time skew when syncing at 00 *** Chng 052 ***
extern int16_t avg_skew_30;       // Average time skew when syncing at 30 *** Chng 052 ***


/* Exported functions ------------------------------------------------------------*/
void wirelessTimerEvent(uint32_t action );
void wirelesTimeSyncCheck(void); 
void wireless_putchar(uint8_t b);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __WIRELESS_H */

/**
  * @}
  */
