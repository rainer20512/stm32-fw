/**
  ******************************************************************************
  * @file    watch.c
  * @author  Rainer 
  * @brief   link global variables to be watched to a "watch map", whose
  *          members can be interrogated by master
  ******************************************************************************
  */
#include "config/config.h"

#include "watch.h"
#include "rtc.h"
#include "wireless.h"
#include "sequencer/analyzer.h"
#include "dev/adc_dev.h"
#include "system/profiling.h"

extern uint64_t ProfilerTimes[];

#if DEBUG_SLEEP_MODE > 0
	#include "../common/common_sleep.h"
#endif

#if USE_OPTICAL_EMETER > 0
	#include "optical_easymeter.h"
#endif

#if USE_BMP085 > 0
    #include "sensors/bmp085.h"
#endif

#define B8 0x0000
#define B16 0x80000000
#define B_MASK 0x80000000


#define WATCH_LAYOUT 0x30

#define WATCH_UNUSED 0xffff
static uint32_t stop_time;
static uint32_t sleep_time;
static uint32_t rfm_time;
static uint32_t rtc_time;
static uint32_t prof_time;

static uint32_t watch_map[WATCH_N] = {
#if USE_BMP085 > 0	
    /* 00 */ ((uint32_t) &bmp_pressure) + B16,                          // BMP085 pressure
#elif USE_OPTICAL_EMETER > 0
    /* 00 */ ((uint16_t) &SML_error_count) + B16,			// SML parser error count
#else
    /* 00 */ WATCH_UNUSED,
#endif
    #if defined(USE_ADC1)
        /* 01 */ ((uint32_t) &ADC1Handle.vdda) + B16,				// battery 
        /* 02 */ ((uint32_t) &uptime_mins) + B16,				// 32bit value LO-Word *** Chng 052 ***
    #else
        /* 01 */ WATCH_UNUSED,
        /* 02 */ WATCH_UNUSED,
    #endif
    /* 03 */ ((uint32_t) ((char*)(&uptime_mins))+2) + B16,	        // 32bit value Hi-Word *** Chng 052 ***
    /* 04 */ ((uint32_t) &missed_syncs) + B16,				// missed syncs
    /* 05 */ ((uint32_t) &avg_skew_00) + B16,				// Time skew @ Sync Second 0
    /* 06 */ ((uint32_t) &avg_skew_30) + B16,				// Time skew @ Sync Second 30
#if DEBUG_PROFILING > 0
        /* 07 */ ((uint32_t) &stop_time) + B16,                         // 32bit value LO-Word 
        /* 08 */ ((uint32_t) ((char*)(&stop_time))+2) + B16,            // 32bit value Hi-Word 
        /* 09 */ ((uint32_t) &sleep_time) + B16,			// 32bit value LO-Word 
        /* 10 */ ((uint32_t) ((char*)(&sleep_time))+2) + B16,           // 32bit value Hi-Word 
        /* 11 */ ((uint32_t) &rfm_time) + B16,                          // 32bit value LO-Word 
        /* 12 */ ((uint32_t) ((char*)(&rfm_time))+2) + B16,             // 32bit value Hi-Word 
        /* 13 */ ((uint32_t) &rtc_time) + B16,                          // 32bit value LO-Word 
        /* 14 */ ((uint32_t) ((char*)(&rtc_time))+2) + B16,             // 32bit value Hi-Word 
        /* 15 */ ((uint32_t) &prof_time) + B16,                         // 32bit value LO-Word 
        /* 16 */ ((uint32_t) ((char*)(&prof_time))+2) + B16,            // 32bit value Hi-Word 
#if USE_RFM_OOK > 0
        /* 17 */ ((uint32_t) &avg_waitms) + B16,	                // ook lead time
        /* 18 */ ((uint32_t) &OOK_RetryCount) + B16,	                // Number of missed OOK transmissions
#else
        /* 17 */ WATCH_UNUSED,
        /* 18 */ WATCH_UNUSED,
#endif
        /* 19 */ WATCH_UNUSED,
        /* 20 */ WATCH_UNUSED,
        /* 21 */ WATCH_UNUSED,
        /* 22 */ WATCH_UNUSED,
#else
        /* 07 */ WATCH_UNUSED,
        /* 08 */ WATCH_UNUSED,
        /* 09 */ WATCH_UNUSED,
        /* 10 */ WATCH_UNUSED,
        /* 11 */ WATCH_UNUSED,
        /* 12 */ WATCH_UNUSED,
        /* 13 */ WATCH_UNUSED,
        /* 14 */ WATCH_UNUSED,
        /* 15 */ WATCH_UNUSED,
        /* 16 */ WATCH_UNUSED,
        /* 17 */ WATCH_UNUSED,
        /* 18 */ WATCH_UNUSED,
        /* 19 */ WATCH_UNUSED,
        /* 20 */ WATCH_UNUSED,
        /* 21 */ WATCH_UNUSED,
        /* 22 */ WATCH_UNUSED,
#endif
        /* 23 */ WATCH_UNUSED,
        /* 24 */ WATCH_UNUSED,
};

/* 
 * web frontend expects times in 1/256 s unit, macro for conversion
 * 256/1000000 == 4/15625
 */
#define US_TO_1_256(us)     (us*4/15625)

#include <stdarg.h>
// #include <stdio.h>

#if DEBUG_PROFILING > 0
  /******************************************************************************
   * take "count" elements from "ProfilerTimes, add them up and return in 1/256s
   * units
   ****************************************** 0***********************************/
  static uint32_t add_up_times (uint32_t count, ...)
  {
    va_list ap;
    uint64_t sum = 0;
    ActiveJobEnumType jobidx;

    va_start (ap, count);         

    sum = 0;
    for (uint32_t i = 0; i < count; i++) {
      jobidx = va_arg (ap, int);
      sum += ProfilerTimes[jobidx];
    }
    va_end (ap);                  /* Clean up. */
    return (uint32_t) US_TO_1_256(sum);
  }
#endif

uint16_t Get_watch(uint8_t idx) 
{
	uint32_t p;

	if (idx >= WATCH_N) return WATCH_LAYOUT;

#if DEBUG_PROFILING > 0
        if ( idx ==7  || idx == 8  )  stop_time  = add_up_times(3, JOB_STOP0, JOB_STOP1, JOB_STOP2);
        if ( idx ==9  || idx == 10 ) sleep_time  = add_up_times(1, JOB_SLEEP);
        if ( idx ==11 || idx == 12 )   rfm_time  = add_up_times(2, JOB_TASK_RFM, JOB_IRQ_EXTI);
        if ( idx ==13 || idx == 14 )   rtc_time  = add_up_times(4, JOB_TASK_RTC, JOB_IRQ_RTC, JOB_TASK_TMR, JOB_IRQ_TMR);
        if ( idx ==15 || idx == 16 )  prof_time  = add_up_times(2, JOB_TASK_PROFILER, JOB_IRQ_PROFILER);
#endif

	p=watch_map[idx];

	// undefined watch valus
	if ( p == WATCH_UNUSED ) return 0;

	// Check for 8 or 16 bit value
	if ((p & B_MASK)==B16) return *((uint16_t *)(p & ~B_MASK));	                // 16 bit value
	else return (uint16_t)(*((uint8_t *)(p))); 					// 8 bit value
		
}

