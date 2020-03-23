#pragma once

#define TX18_TIMEOUT_INTERVAL           260			// Should catch at least two TX18-Transmissions in normal mode
#define TX18_SKIP_INIT	 		3			// every 32 sec in initial phase, that is every fourth transmission
#define TX18_SKIP_NORM			0			// every 128 sec in normal phase
#define TX18_XMIT_DURATION		2 			// Total Transmission of TX18 data packet is max. 2 seconds
#define TX18_XMIT_PREFIX		0			// Start 0 seconds before the extimated transmission begins
#define TX18_MAX_LISTEN_TIME            6			// Seconds to listen when a transmission is expected
#define TX18_RETRIES			5			// Number of retries afer an unsuccessful transmission
#define TX18_ENLARGE                    4                       // Delta t in seconds to extend the listening interval after unsuccessful listening

void     AnalyzerInit();
void     task_handle_ana  (uint32_t);

void     AnalyzeSequence(uint8_t s_num);
void     AnalyzerTimeout(uint32_t);
void     AnalyzerCheckForShutdown(void);
uint32_t FreeTimeSlot(void);                                    // Get time[s] until next OOK receive is triggered
void     Start_OOK_Receiver(uint32_t);
void     Stop_OOK_Receiver(void);
void     Reset_OOK_Receiver(void);

extern int16_t	abstemp;					// temp*100
extern uint8_t  relhum;                                         // relative humidity in %    
extern uint8_t  lowbatt;					// LowBatt-Indication of sensor
extern union CTIME_struct temp_received;                        // Date & Time of last Temp transmission
extern union CTIME_struct rh_received;                          // Date & Time of last RH transmission
extern uint8_t  OOK_retries;                                    // Number of retries in case of unsuccessful OOK-Reception
extern int16_t  avg_waitms;                                     // average time [ms} between start of receiver and reception of first correct data packet
extern uint16_t OOK_RetryCount;                                 // Number of missed OOK transmissions
/* Test function(s) -------------------------------------------------------- */
bool    Retime_OOK_Receiver(uint32_t sec);

#if DEBUG_RETRY > 0
	extern bool bSimulateRetry;
#endif
