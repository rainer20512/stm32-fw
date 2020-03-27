/**
  ******************************************************************************
  * @file    timer.h
  * @author  Rainer
  * @brief   Functions for all sorts of timers
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define NO_TIMER_ID     (-1)                     /* Indicator for "no valid timer ID   */


extern uint32_t secsFromStart;

typedef void (*TimerCBT )(uint32_t);

/* Millisecond timer ( to be exact: minimum granularity is 1/256sec, ie approx. 4ms */
int8_t   MsTimerAllocate    ( int8_t id );
int8_t   MsTimerDelete      ( int8_t id );
int8_t   MsTimerSetAbs      (            uint32_t       subsecs,                     TimerCBT myCB, uint32_t arg );
int8_t   MsTimerReUseAbs    ( int8_t id, uint32_t       subsecs,                     TimerCBT myCB, uint32_t arg );
int8_t   MsTimerSetRel      (            uint32_t delta_subsecs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg );
int8_t   MsTimerReUseRel    ( int8_t id, uint32_t delta_subsecs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg );
uint32_t MsTimerHandleCMP   ( uint16_t matchvalue );

void task_handle_tmr(uint32_t);                      /* Handle timer requests                       */

#if USE_SECONDTIMER > 0
    /* Second timer */
    #define SECTIMER_MAX         3600  /* maximum value for periodic second timer */
    void handle_sectimer_periodic (void);        /* Handle sec timer requests                   */
    int8_t SecTimerAllocate ( int8_t TimerIdx );
    int8_t SecTimerDelete   ( int8_t TimerIdx );
    int8_t SecTimerSetRel   ( uint32_t delta_secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  );
    int8_t SecTimerReUseRel ( int8_t id, uint32_t delta_secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  );
    int8_t SecTimerReSetRel ( int8_t id, uint32_t delta_secs );
    #ifdef TX18LISTENER
        bool SECTimerGetRel ( int8_t id, uint32_t *remaining_s );
    #endif // TX18LISTENER
#else
    #define handle_sectimer_periodic()
    #define SecTimerAllocate(a)         (-1)
    #define SecTimerDelete(u)           (-1)
    #define SecTimerSetRel(a,b,c,d)     (-1)
    #define SecTimerReUseRel(a,b,c,d,e) (-1)
    #define SecTimerReSetRel(a,b)       (-1)
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TIMER_H */
