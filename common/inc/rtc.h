/**
  ******************************************************************************
  * @file    rtc.h
  * @author  Rainer
  * @brief   Handle the RTC stuff
  *
  ******************************************************************************
  * @addtogroup RTC
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H
#define __RTC_H

#include "config/config.h"

#include "hardware.h"

#define USE_TIMER  1
#define USE_RTC    0

/* Public define ------------------------------------------------------------*/
/* 
 * Define the timer that will be used for time generation on LSE Osc. base 
 * Its granularity will be 1/256 s, or coarse: 4ms
 * Change all defines, if counter is changed 
 * Also Change Clock Selection in PeriphClockConfig in clockconfig.c
 */

// #define MILLISEC_TMR_DEBUG_STOP()     (DBGMCU->APB1FZR2 |= DBGMCU_APB1FZR2_DBG_LPTIM2_STOP)
#if USE_RTC > 0
    #define RTC_PREDIV_ASYNC  0x7f                  /* default async prescaler value */
    #define RTC_PREDIV_SYNC   0xff                  /* default sync  prescaler value */
    #define RTC_SUBSEC_MAX    (RTC_PREDIV_SYNC+1)   /* number of RTC subseconds      */
#endif

#if USE_TIMER > 0
    /* LPTIM1 use used for all timer purposes, clocked by LSE */
    #define TIMR_TICKS_PER_SEC  32768           /* LPTIM1 is clocked by LSE */
    #define TIMR_PREDIV_VALUE   16              /* prescaler value, valid are 1,4,8, ..., 128 */
    #define RTC_SUBSEC_MAX      256             /* the system heartbeat is always 256Hz       */
    uint16_t RTC_GetTimer();
    #define RTC_GetS256()       (uint8_t)(RTC_GetTimer()/TIMR_TO_RTC_DIV)
    #define CMP_FORBIDDEN_VALUE 0
#elif USE_RTC > 0
    /* RTC is used as rtc, WUCK timer of RTC is used as timer base */
    #define TIMR_TICKS_PER_SEC 32768          /* RTC is clocked by LSE */
    #define TIMR_PREDIV_VALUE  16             /* prescaler value, valid are 2,4,8, 16 */
    #define RTC_SUBSECS_MAX    256             /* the system heartbeat is always 256Hz       */
    /*
     * SSR is a downcounter and reloaded with RTC_PREDIV_SYNC, which is 0xff,
     * so we have 256 subseconds per second
     */
    #define RTC_GetS256()                 (RTC_PREDIV_SYNC - RTC->SSR)
    #define RTC_Timer()                   SUBS256_TO_TIMERUNIT(RTC_GetS256())
#endif

/* Resulting maximum Tiner ticks per second */
#define TIMR_SUBSEC_MAX         (TIMR_TICKS_PER_SEC/TIMR_PREDIV_VALUE)              

/* Convert Timer ticks to S256 */
#define TIMR_TO_RTC_DIV     (TIMR_SUBSEC_MAX/RTC_SUBSEC_MAX)

/* Bitmask of unused LSBs of timer values due to lower resolution of RTC */
#define TIMR_UNUSED_BITS        ((TIMR_SUBSEC_MAX/RTC_SUBSEC_MAX)-1)
#define TIMERVALUE_ALIGN(a)     (a & ~TIMR_UNUSED_BITS )

/* macro to subtract subsecs in the range [ 0 .. TIMR_SUBSEC_MAX-1 ] */
#define SUBSEC_DIFF(a, b)  ((a < b ? a + TIMR_SUBSEC_MAX : a) - b)

// Macros to convert Milliseconds into Timerunits
#define MILLISEC_TO_TIMERUNIT(t)      ((uint32_t)((t*TIMR_SUBSEC_MAX)/1000)) 
#define TIMERUNIT_TO_MILLISEC(t)      ((uint32_t) (((uint32_t)t)*1000/TIMR_SUBSEC_MAX) ) 

// Macros to convert Mikroseconds into Timerunits
#define MIKROSEC_TO_TIMERUNIT(t)      ((uint32_t)((t*TIMR_SUBSEC_MAX)/1000000)) 
#define TIMERUNIT_TO_MIKROSEC(t)      ((uint32_t) (((uint32_t)t)*1000000/TIMR_SUBSEC_MAX) ) 

/* Macros for 1/256 subsecs ot our Subsecs */
#define SUBS256_TO_TIMERUNIT(s256) ((uint32_t)s256*TIMR_SUBSEC_MAX/256)
#define TIMERUNIT_TO_SUBS256(mysubs) ((uint32_t)mysubs*256/TIMR_SUBSEC_MAX)


/* Macro for 1/256 to milliseconds and vice versa*/
#define SUBS256_TO_MILLIS(s256) ((uint32_t)s256*1000/256)
#define MILLIS_TO_SUBS256(mlls) ((uint32_t)mlls*256/1000)


#if USE_TIMER > 0
    #define DisableTmrIRQ()     LPTIM1->CMP = CMP_FORBIDDEN_VALUE 
    #define EnableTmrIRQ()
    #define TmrIrqIsEnabled()   (LPTIM1->CMP != CMP_FORBIDDEN_VALUE)
#elif USE_RTC > 0
    #define DisableTmrIRQ()     (RTC->CR &= ~( RTC_CR_WUTE  | RTC_CR_WUTIE ))
    #define EnableTmrIRQ()      (RTC->CR |=  ( RTC_CR_WUTE  | RTC_CR_WUTIE ))
#endif

#ifdef __cplusplus
 extern "C" {
#endif

//! day of week
typedef enum {
    MON=0, TUE, WED, THU, FRI, SAT, SUN,  
} rtc_dow_t;

/***********************************************************************************************//**
 * \struct rtc_t
 * \brief time structure for a complete date/time structure 
 *
 * may be used for encryption, in this case it must be 8 byte long identical in master and slaves 
 **************************************************************************************************/ 
typedef struct {
    uint8_t YY;         //!< \brief Date: Year (0-255) -> 2000 - 2255
    uint8_t MM;         //!< \brief Date: Month
    uint8_t DD;         //!< \brief Date: Day
    uint8_t hh;         //!< \brief Time: Hours
    uint8_t mm;         //!< \brief Time: Minutes
    uint8_t ss;         //!< \brief Time: Seconds
    uint8_t DOW;        //!< \brief Time: Day of week 
    uint8_t pkt_cnt;    //!< \brief Rolling packet counter
} rtc_t;

extern rtc_t            rtc;
extern uint32_t         uptime_mins;

void task_init_rtc(void);

#if USE_RTC > 0
    void RTC_EnableBkUpAccess(void);
    void RTC_DisableBkUpAccess(void);
    void RTC_CopytoVar(void);
#endif

void RTC_SetDateTime(uint8_t dd, uint8_t mm, uint8_t yy, uint8_t hr, uint8_t mi, uint8_t sec, uint8_t subs);
bool RTC_SetCmpMatch(uint32_t value);


char* RTC_GetStrDateTimeSubsecs(void);
char* RTC_GetStrDateTime(void);
char* RTC_GetStrTime(void);
void RTC_DumpDateTime(void);

void task_handle_rtc(uint32_t);
void task_init_rtc(void);

#define RTC_GetMillis()               SUBS256_TO_MILLIS(RTC_GetS256())
#define RTC_GetSecond()               (rtc.ss)
#define RTC_GetMinute()               (rtc.mm)
#define RTC_GetHour()                 (rtc.hh)
#define RTC_GetDay()                  (rtc.DD)
#define RTC_GetMonth()                (rtc.MM)
#define RTC_GetYearYY()               (rtc.YY)

/* Implementation of a simple stopwatch */
void     RTC_StopWatch_Start          (void);
uint32_t RTC_StopWatch_GetTime        (void);
uint32_t RTC_StopWatch_Stop           (void);
bool     RTC_StopWatch_InUse          (void);

#if DEBUG_PROFILING > 0
    /* 
     * if profiling is acive, we need functions to access the RTC registers directly 
     * because time is read when rtc variable may not be synced 
     */
    #define HW_GetMillis()            RTC_GetMillis()
    #if USE_RTC > 0
        #define HW_GetMinute()            (\
                                              (( RTC->TR & RTC_TR_MNT_Msk ) >> RTC_TR_MNT_Pos) * 10 \
                                            + (( RTC->TR & RTC_TR_MNU_Msk ) >> RTC_TR_MNU_Pos) \
                                          )
        #define HW_GetSecond()            (\
                                              (( RTC->TR & RTC_TR_ST_Msk ) >> RTC_TR_ST_Pos) * 10 \
                                            + (( RTC->TR & RTC_TR_SU_Msk ) >> RTC_TR_SU_Pos) \
                                          )
    
    #elif USE_TIMER > 0
        #define HW_GetMinute()      RTC_GetMinute()
        #define HW_GetSecond()      RTC_GetSecond()
    #endif
#endif

union CTIME_struct {
    struct {
        uint32_t   timeint;
    };
    struct {
        uint8_t    timebyte[4];
    }; 
};
void compress_datetime   ( union CTIME_struct *ctime, rtc_t *rtc );
void uncompress_datetime ( union CTIME_struct *ctime, rtc_t *rtc );



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __RTC_H */

/**
  * @}
  */
