/**
  ******************************************************************************
  * @file    timer.c
  * @author  Rainer 
  * @brief   Handle periodic and user defined timers
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup Timer
  * @{
  */

#define DEBUG_TIMER         0
#define DEBUG_SECTIMER      0

#include "error.h"
#include "debug.h"
#include "timer.h"
#include "task/minitask.h"
#include "rtc.h"
#include "global_flags.h"
#include "debug_helper.h"

#include "hardware.h"

#define TMR_RELATIVE_FLAG 0x8000000     /* MSB of bPeriodic indicates relative or absolute value */
#define TMR_ALL_FLAGS    TMR_RELATIVE_FLAG
#define MSTMR_GET_PERIODIC_VALUE(id)      (myMsTimers[id].bPeriodic  & ~TMR_ALL_FLAGS )
#define MSTMR_GET_FLAGS(id)               (myMsTimers[id].bPeriodic  &  TMR_ALL_FLAGS )

/* Private typedef -----------------------------------------------------------*/
typedef struct mstmr_t {
	uint32_t uTime;                 /* Timer value */
	uint32_t bPeriodic;             /* != 0 if peroidic                   */
        uint32_t delta_t;               /* the delta t when timer is periodic */
	TimerCBT cCallback;             /* Callback on expiration             */
	uint32_t arg;			/* users argument for callback        */
} MsTimer; 

/* Private define ------------------------------------------------------------*/
#define TMR_MAX 10        /* maximum number of parallel active timers, max 31 */
#define TMR_MASK ((1<<TMR_MAX)-1)              /* Mask for all useable timers */


/* Private macro -------------------------------------------------------------*/

#define MSTIMER_USED(idx)           ( msTmrBitmap  &   ( 1 << idx ) )      /* returns != 0, if Timer index "idx" is used */
#define MSTIMER_EXPIRED(idx)        ( msTmrExpired &   ( 1 << idx ) )
#define MSTIMER_ALLOC(idx)          ( msTmrBitmap  |=  ( 1 << idx ) )
#define MSTIMER_FREE(idx)           ( msTmrBitmap  &= ~( 1 << idx ) )
#define ANY_MSTIMER_USED()          ( msTmrBitmap  &   TMR_MASK )
#define MSTIMER_RESET_EXPIRED(idx)  ( msTmrExpired &= ~( 1 << idx ) )


/* Private variables ---------------------------------------------------------*/
static uint32_t msTmrBitmap=0;      /* bitmap of used ms timers    */
static uint32_t msTmrExpired=0;     /* bitmap of expired ms timers */
static uint8_t  msTmrUpper=TMR_MAX; /* upper limit of allocateable msTimers  */

static MsTimer myMsTimers[TMR_MAX];  /* neccessary data for all ms timers  */

#if USE_SECONDTIMER > 0
    #define SECTMR_GET_PERIODIC_VALUE(id)     (mySecTimers[id].bPeriodic & ~TMR_ALL_FLAGS )
    #define SECTMR_GET_FLAGS(id)              (mySecTimers[id].bPeriodic &  TMR_ALL_FLAGS )
    #define GET_LINEAR_SECS()    (secsFromStart)

    #define SECTIMER_USED(idx)          ( secTmrBitmap &    ( 1 << idx ) )     /* returns != 0, if Timer index "idx" is used */
    #define SECTIMER_EXPIRED(idx)       ( secTmrExpired &   ( 1 << idx ) )
    #define SECTIMER_ALLOC(idx)         ( secTmrBitmap |=   ( 1 << idx ) )
    #define SECTIMER_FREE(idx)          ( secTmrBitmap &=  ~( 1 << idx ) )
    #define ANY_SECTIMER_USED()         ( secTmrBitmap &    TMR_MASK )

    static uint32_t secTmrBitmap=0;     /* bitmap of used ms timers    */
    static MsTimer mySecTimers[TMR_MAX]; /* neccessary data for all sec timers */


                                         /* used for Second timer match           */
    static uint32_t secsNextMatch=0;     /* next second timer match, 0 = none     */
    static uint8_t  idxNextMatch;        /* idx of next matching sec timer        */
    static uint8_t  secTmrUpper=TMR_MAX; /* upper limit of allocateable secTimers */
#endif

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

/*********************************************************************************
  * @brief  Return next free timer ID
  *
  * @param  None
  *         
  * @retval Idx of next free Timer -or-
  *         -1  -> no free timer
  *
  ********************************************************************************/
static int8_t mstmrGetNextFree(void)
{

  int8_t i;

  for ( i=0; i < msTmrUpper; i++ ) if (! MSTIMER_USED(i) ) {
      return i;
  } // for, if


  return -1;
}

/*********************************************************************************
  * @brief  Returns true, if value is the smallest value of all active msTimer
  *         values
  *         Only if this is the case, the next timer match value has to be 
  *         recalulated
  ********************************************************************************/
static bool mstmrIsLowestValue(uint32_t val)
{
  #if DEBUG_TIMER > 2
    DEBUG_PUTS("FindLowestTmr");
  #endif

  /* if no timer is used, the new value is the lowest */
  if ( ! ANY_MSTIMER_USED()  ) {
    #if DEBUG_TIMER > 2
        DEBUG_PRINTF("%d is only\n",val);
    #endif
    return true;
  }
  register uint32_t current_subs = RTC_GetTimer();
  #if DEBUG_TIMER > 2
    DEBUG_PRINTF("current=%02x, refval=%02x\n",TIMERUNIT_TO_SUBS256(current_subs), TIMERUNIT_TO_SUBS256(val));
  #endif
  register uint32_t min = SUBSEC_DIFF(val, current_subs); 
 
  /* Check all active timers for having a shorter interval than that in 'min' */

  register int32_t i;
  register uint32_t act;
  register uint32_t dif;

  /* Iterate for all used timers */
  for (i=0;i<TMR_MAX;i++) if ( MSTIMER_USED(i) ) {
    act = myMsTimers[i].uTime;
    dif = SUBSEC_DIFF(act, current_subs );
    #if DEBUG_TIMER > 0
        DEBUG_PRINTF("ID %d: act=%02x, min=%02x, dif=%02x\n",i,TIMERUNIT_TO_SUBS256(act),TIMERUNIT_TO_SUBS256(min),TIMERUNIT_TO_SUBS256(dif));
    #endif
    /* Found a timer with a lower value -> return false */
    #if DEBUG_TIMER > 2
        DEBUG_PRINTF("TmrID %d is lower\n",i);
    #endif
    if ( dif < min ) {
        #if DEBUG_TIMER > 2
            DEBUG_PRINTF("TmrID %d is lower\n",i);
        #endif
        return false;
    }
  } // For, outer if

  /* No active timer with lower value found */
  #if DEBUG_TIMER > 2
      DEBUG_PRINTF("%d is smallest\n",val);
  #endif
  return true;
}


/*********************************************************************************
  * @brief  Find the shortest ms-timer and program its value into the compare
  *         register of MILLISEC_TMR.
  *
  * @note   Compare Match interrupt will be disabled, if no active timer is found
  * 
  * @param  None
  *         
  * @retval None
  *
  ********************************************************************************/
static void mstmrFindNext(void)
{
  /* 
   * any active ms-Timer at all? If not, stop WUCK and we are done
   */
  if ( ! ANY_MSTIMER_USED()  ) {
    DisableTmrIRQ();
    // DISABLE_CMP();
    #if DEBUG_TIMER > 2
        DEBUG_PUTS("Tmr disabled");
    #endif
    return;
  }
 
  /* 
   * Find the timer with the shortest interval and put the corresponding value into 
   * Compare register. 
   */

  
  register uint32_t current_subs = RTC_GetTimer();
  register uint32_t dif=TIMR_SUBSEC_MAX;
  register uint32_t next;
  register int32_t i;
  uint32_t n_org, n_min=0;
#if DEBUG_TIMER > 1
  int8_t id = NO_TIMER_ID;
#endif
    
  /* Iterate for all used timers */
  for (i=0;i<TMR_MAX;i++) if (MSTIMER_USED(i)) {
    #if DEBUG_TIMER > 2
        DEBUG_PRINTF("Tmr %d: %02x vs %02x", i, TIMERUNIT_TO_SUBS256(myMsTimers[i].uTime), RTC_Timer());
    #endif
    /* if the time difference of this timer is shorter than the time difference so far */
    /* then set this timer as new smallest time difference                             */
    n_org = myMsTimers[i].uTime;
    next = SUBSEC_DIFF(n_org, current_subs);
    if (next <= dif ) {
      #if DEBUG_TIMER > 2
         DEBUG_PRINTF(" -> %02x",  TIMERUNIT_TO_SUBS256(next));
      #endif
      dif   = next;
      n_min = n_org;
      #if DEBUG_TIMER > 1
        id = i;
      #endif
      #if DEBUG_TIMER > 2
         DEBUG_PRINTF(" Diff=%02x",TIMERUNIT_TO_SUBS256(dif));
      #endif
    }
    #if DEBUG_TIMER > 2
       DEBUG_PRINTF("\n");
    #endif
  } // For, outer if

    /* 
     * Timer compare match is fired, when CNT becomes CMP+1, 
     * so be sure to store the by 1 reduced values in uTime
     */
    // COM_print_time('S', false); DEBUG_PUTS("Tmr started");
    // CMP_SET(n_min);
    if ( RTC_SetCmpMatch(n_min) ) { 
        #if DEBUG_TIMER > 1
            DEBUG_PRINTF("@%04x set to %02x=%04x raw from ID %d\n", RTC_GetTimer(), TIMERUNIT_TO_SUBS256(n_min), n_min, id );
        #endif
    }
}    


/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/


/*********************************************************************************
  * @brief  permanently allocate a msTimer ID. This msTimerID will not be given
  *         to any other user. Once allocated, it will stay allocated forever
  * @param  id - NO_TIMER_ID in case of allocation of a new permanent timer
  *         id - any valid permanent msTimerID in case of ReAllocation due to 
  *              re-execution of permanent allocation part 
  * @retval TimerIdx    = SecTimerID to use
  *         NO_TIMER_ID = SecTimerID could not be allocated
  ********************************************************************************/
int8_t MsTimerAllocate(int8_t id)
{
    if ( id == NO_TIMER_ID ) {
        id = msTmrUpper - 1;
        if ( id < 0 ||  MSTIMER_USED(id) ) {
             /* Report error, if not found */
            log_error ( "MsTimerAllocate: No free TimerID!" );
            return NO_TIMER_ID;
        }
        msTmrUpper = id;
        return id;
    } else {
        /* check for a valid permanent MsTimerID */
        if ( id >= msTmrUpper && id < TMR_MAX ) 
            return id;
        else    
            return NO_TIMER_ID;
    }
}

/*********************************************************************************
  * @brief  Delete an active timer
  *
  * @note   Will be deleted immediately, scheduled event will be removed
  *
  * @param  TimerIdx - Index of timer to be deleted
  *         
  * @retval TimerIdx = Deletion successful
  *         -1       = Timer did no exist, could not be deleted 
  *
  ********************************************************************************/
int8_t MsTimerDelete ( int8_t TimerIdx )
{
  register uint32_t act = myMsTimers[TimerIdx].uTime;
  /* Check for valid Idx */ 
  if ( TimerIdx >= TMR_MAX ) return -1;

  /* Check for active timer */ 
  if ( ! MSTIMER_USED(TimerIdx ) ) return -1;

  /* mark timer idx as unused */
  MSTIMER_FREE(TimerIdx);

  /* if deleted timer was the next timer to expire, recalculate next timer to expire */
  if ( mstmrIsLowestValue(act) ) mstmrFindNext();

  return TimerIdx;
}


/*********************************************************************************
 * @brief  do the real timer setting for any absolute or relative, for any
 *         one shot or repeated timer
 *
 * @param  id          - previous allocated MsTimer-ID. 
 *         delta_ticks - relative delta of ticks in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 *         bPeriodic   - if != 0  execute timer periodically
 *                     - if == 0  execute once
           period      - if timer is periodic, the delta_ticks for every periodic repetition
 *         myCB        - Callback function when timer expires
 * @note   delta_ticks has the unit  [MSCounterTicks] 
 *         MsCounterTicks has the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 * @retval TimerIdx    = ID of new Timer
 *         NO_TIMER_ID = Timer could not be allocated 
 ********************************************************************************/
static int8_t mstmrSet( int8_t id, uint32_t delta_ticks, uint32_t bPeriodic, uint32_t period, TimerCBT myCB, uint32_t arg  )
{
    bool bNewTmrIsLowest; 

    if ( id < 0 || id >= TMR_MAX ) {
         /* Report error, if not found */
        log_error ( "MsTimerReUse: No valid Timer!" );
        return NO_TIMER_ID;
    }

    /* Check for subsec value being in allowed range [ 0 .. TIMR_SUBSEC_MAX -1 ] */
    if ( delta_ticks > TIMR_SUBSEC_MAX-1 ) {
        log_error ( "MsTmrSet: delta_ticks out of range(%d)", delta_ticks );
        return NO_TIMER_ID;
    } 
    /* Check for period being in allowed range [ 0 .. TIMR_SUBSEC_MAX ] */
    if ( bPeriodic && period > TIMR_SUBSEC_MAX ) {
        log_error ( "MsTmrSet: period out of range(%d)", period );
        return NO_TIMER_ID;
    } 

    /* If timer is still in use, then free it */
    if ( MSTIMER_USED(id) ) {
        /* mark timer idx as unused */
        MSTIMER_FREE(id);

        /* recalculate next timer event */
        mstmrFindNext();
    }

    /* Compute comparator value relative to current counter value */
    /* and limit to allowed range [0 .. TIMR_SUBSEC_MAX-1]        */
    register uint32_t cmp_value = TIMERVALUE_ALIGN((RTC_GetTimer() + delta_ticks) & (TIMR_SUBSEC_MAX-1) );

    /* if the comparator value is the "forbidden" value, add 1 */
    if ( cmp_value == CMP_FORBIDDEN_VALUE ) cmp_value++;

    /* check, whether next timer match recalculation is neccessary */
    bNewTmrIsLowest = mstmrIsLowestValue(cmp_value);  
    /* fill in neccessary data */

    MSTIMER_ALLOC(id);
    myMsTimers[id].uTime     = cmp_value;
    myMsTimers[id].bPeriodic = bPeriodic;
    myMsTimers[id].cCallback = myCB;
    myMsTimers[id].arg       = arg;
    if ( bPeriodic ) {
        /* Set the relative Flag, if periodic */
        myMsTimers[id].bPeriodic |= TMR_RELATIVE_FLAG;
        myMsTimers[id].delta_t   = period;
    }
    #if DEBUG_TIMER > 0
     DEBUG_PRINTF("@%04x MsTmr %d set to %04x with Delta_t = %d\n",RTC_GetTimer(), id, cmp_value ,delta_ticks);
    #endif
  
    /* recalculate next timer event, if new inserted timer is next to expire */
    if ( bNewTmrIsLowest ) mstmrFindNext();

    return id;
}

/*********************************************************************************
  * @brief  Allocate a new timer and start this timer ( w absolute subsecond value )
  *
  * @param  id           - prevoiusly allocated timerID
  *         abs_tmrticks - absolute subsecond value in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
  *                      - if == 0  execute once
  *         myCB         - Callback function when timer expires
  * 
  * @note   subsecs has the unit  [MSCounterTicks] 
  *         MsCounterTicks has the range [ 0 .. MILLISEC_PERIOD ]
  * @note   absolute timers cannot be periodic!
  *
  * @retval TimerIdx     = ID of Timer
  *         NO_TIMER_ID  = Timer could not be allocated 
  *
  ********************************************************************************/
int8_t MsTimerReUseAbs ( int8_t id, uint32_t abs_tmrticks, TimerCBT myCB, uint32_t arg ) 
{

  /* Check for subsec value being in allowed range [ 0 .. MILLISEC_PERIOD ] */
  if ( abs_tmrticks > TIMR_SUBSEC_MAX-1 ) {
      log_error ( "MsTmrSetAbs: %d out of range", abs_tmrticks );
      return NO_TIMER_ID;
  } 

  /* Check for subsec value being in allowed range [ 0 .. MILLISEC_PERIOD ] */
  if ( abs_tmrticks > TIMR_SUBSEC_MAX-1 ) {
      log_error ( "MsTmrSetAbs: %d out of range", abs_tmrticks );
      return NO_TIMER_ID;
  } 
  /* Compute the timer counter relative value for the passed absolute value */
  uint32_t rel_ticks = SUBSEC_DIFF(abs_tmrticks, RTC_GetTimer() );

  return mstmrSet( id, rel_ticks, false, 0, myCB, arg  );



  return id; 
}

/*********************************************************************************
  * @brief  Allocate a new MsTimer and start this timer ( w absolute subsecond value )
  * @param  subsecs   - absolute subsecond value in the range [ 0 .. MILLISEC_PERIOD ]
  *         myCB      - Callback function when timer expires
  * @note   subsecs has the unit  [MSCounterTicks] 
  *         MsCounterTicks has the range [ 0 .. MILLISEC_PERIOD ]
  * @retval TimerIdx    = ID of new Timer
  *         NO_TIMER_ID = Timer could not be allocated 
  ********************************************************************************/
int8_t MsTimerSetAbs ( uint32_t abs_tmrticks, TimerCBT myCB, uint32_t arg ) 
{
    /* Get free Timer idx and mark as used  */
    int8_t id = mstmrGetNextFree();
    if ( id == NO_TIMER_ID ) {
         /* Report error, if not found */
        log_error ( "MsTimer: No free Timer!" );
        return id;
    }
    return MsTimerReUseAbs(id, abs_tmrticks, myCB, arg );
}

/*********************************************************************************
 * @brief  Reuse/Re-Set a previous Allocated MsTimer and restart this timer
 *         or allocate and start a new MsTimer
 *
 * @param  id          - previous allocated MsTimer-ID. 
 *         delta_ticks - relative delta in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 *         bPeriodic   - if != 0  execute timer periodically
 *                     - if == 0  execute once
 *         myCB        - Callback function when timer expires
 * @note   delta_ticks has the unit  [MSCounterTicks] 
 *         MsCounterTicks has the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 * @retval TimerIdx    = ID of new Timer
 *         NO_TIMER_ID = Timer could not be allocated 
 ********************************************************************************/
int8_t MsTimerReUseRel ( int8_t id, uint32_t delta_ticks, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  )
{
    return mstmrSet( id, delta_ticks, bPeriodic, delta_ticks, myCB, arg  );
}

/*********************************************************************************
 * @brief  Allocate a new MsTimer and start this timer ( w relative subsecond value )
 *
 * @param  delta_ticks - relative delta in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 *                       
 *         bPeriodic   - if != 0  execute timer periodically
 *                     - if == 0  execute once
 *         myCB        - Callback function when timer expires
 * 
 * @note   delta_ticks has the unit  [MSCounterTicks] 
 *         MsCounterTicks has the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
 *
 * @retval TimerIdx = ID of new Timer
 *         -1       = Timer could not be allocated 
 *
 ********************************************************************************/
int8_t MsTimerSetRel ( uint32_t delta_ticks, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  )
{
    /* Get free Timer idx */
    int8_t id = mstmrGetNextFree();
    if ( id == NO_TIMER_ID ) {
         /* Report error, if not found */
        log_error ( "MsTimer: No free Timer!" );
        return id;
    }
    return MsTimerReUseRel(id, delta_ticks, bPeriodic, myCB, arg );
}


/*********************************************************************************
  * @brief  Handle a compare match interrupt
  *
  * @note   This routine is called directly by the Interrupt handler.
  *         It is executed in interrupt context, so keep it short
  *         
  * @param  id - id of expired timer
  *                       
  * @retval the bitmask of expired timers
  *
  ********************************************************************************/
uint32_t MsTimerHandleCMP(uint16_t matchvalue)
{
  
    register uint32_t mask=0;
    register uint32_t i;

    #if DEBUG_TIMER > 0
        COM_print_time('m',false ); DEBUG_PRINTF("Match @ %04x\n",matchvalue);
    #endif

    /* Iterate for all used timers */
    for (i=0;i<TMR_MAX;i++) if (MSTIMER_USED(i)) {
        /* 
        * Does the timer value match ?
        * then notice in "mask"
        */
        if ( myMsTimers[i].uTime == matchvalue ) {
            #if DEBUG_TIMER > 0
                DEBUG_PRINTF("Match TmrID %d, TMR=%04x, CMP=%04x\n", i, RTC_GetTimer(), LPTIM1->CMP );
            #endif
            mask |= ( 1 << i );
        }
    } // for, outer if
  
    /* 
    * check for any matching timer at all ( there should be one ),
    * copy all found timer ids to the expired-bitmask and set TMR taskbit to execute callback
    * in user context
    */
    if ( !mask ) {
        #if DEBUG_MODE > 0 && DEBUG_TIMER > 1
            log_error ( "MsTmrHandleCMP: No corresponding timer for value %04x\n", matchvalue );
        #endif
    } else {
        msTmrExpired |= mask;
    }

    return mask;
}

void task_handle_tmr(uint32_t arg)
{
  uint32_t i;
  
  UNUSED(arg);

  /* 1. delete all expired, non periodic timers */
  for (i=0;i<TMR_MAX;i++) if (MSTIMER_EXPIRED(i)) {
    /* if periodic timer, recalc next time, if relative, if not periodic timer, delete.  */
    if ( MSTMR_GET_PERIODIC_VALUE(i) ) {
      if ( MSTMR_GET_FLAGS(i) & TMR_RELATIVE_FLAG ) {
        uint32_t cmp_value = TIMERVALUE_ALIGN(( myMsTimers[i].uTime + myMsTimers[i].delta_t ) & (TIMR_SUBSEC_MAX-1));
        /* if the comparator value is the "forbidden" value, add 1 */
        if ( cmp_value == CMP_FORBIDDEN_VALUE ) cmp_value++;
        myMsTimers[i].uTime = cmp_value;
      }
    } else {
      MSTIMER_FREE(i);
    }
  }

  /* 2. recalculate next timer compare */
  mstmrFindNext();

  /* 3. Execute callback for all expired timers and reset expired-flag */
  for (i=0;i<TMR_MAX;i++) if (MSTIMER_EXPIRED(i)) {
    /* Reset expired flag */
    MSTIMER_RESET_EXPIRED(i);

    /* if callback specified, execute  */
    if ( myMsTimers[i].cCallback ) myMsTimers[i].cCallback( myMsTimers[i].arg );
  }
}

#if USE_SECONDTIMER > 0 

/*--------------------------------------------------------------------------------
 * ---------------------- Second timer stuff -------------------------------------
 -------------------------------------------------------------------------------*/
/*********************************************************************************
  * @brief  Return next free timer ID
  *
  * @param  None
  *         
  * @retval Idx of next free Timer -or-
  *         -1  -> no free timer
  *
  ********************************************************************************/
static int8_t sectmrGetNextFree(void)
{
  int8_t i;

  for ( i=0; i < secTmrUpper; i++ ) if (! SECTIMER_USED(i) ) {
      return i;
  } // for, if

  return NO_TIMER_ID;
}

/*********************************************************************************
  * @brief  Find the shortest ms-timer and program its value into the compare
  *         register of MILLISEC_TMR.
  *
  * @note   Compare Match interrupt will be disabled, if no active timer is found
  * 
  * @param  None
  *         
  * @retval None
  *
  ********************************************************************************/
static void sectmrFindNext(void)
{
  /* 
   * any active ms-Timer at all? If not, inhibit Compare Match interrupt 
   * and we are done
   */
  if ( ! ANY_SECTIMER_USED()  ) {
    secsNextMatch = 0;
    #if DEBUG_SECTIMER > 2
        DEBUG_PUTS("SecTmr disabled");
    #endif
    return;
  }
 
  /* 
   * Find the timer with the nearest value 
   */
  uint32_t delta_s = 0xFFFFFFFF;
  uint32_t temp;
  uint32_t i;

  /* Iterate for all used timers */
  for (i=0;i<TMR_MAX;i++) if (SECTIMER_USED(i)) {
    temp = mySecTimers[i].uTime - secsFromStart;
    #if DEBUG_SECTIMER > 2
        DEBUG_PRINTF("SecTmr %d: %d vs %d", i, mySecTimers[i].uTime, secsFromStart);
    #endif
    /* if the time difference of this timer is shorter than the time difference so far */
    /* then set this timer as new smallest time difference                             */
    if ( temp < delta_s ) {
        secsNextMatch = mySecTimers[i].uTime;
        delta_s       = temp;
        idxNextMatch  = i;
    }
    #if DEBUG_SECTIMER > 2
       DEBUG_PUTS("");
    #endif
  } // For, outer if

    #if DEBUG_SECTIMER > 0
       DEBUG_PRINTF("Next match: SecTimer %d in %d secs\n", idxNextMatch, secsNextMatch-secsFromStart);
    #endif
}    

/*********************************************************************************
  * @brief  Delete an active timer
  * @note   Will be deleted immediately, scheduled event will be removed
  * @param  TimerIdx - Index of timer to be deleted
  * @retval TimerIdx = Deletion successful
  *         -1       = Timer did no exist, could not be deleted 
  ********************************************************************************/
int8_t SecTimerDelete ( int8_t TimerIdx )
{
  /* Check for valid Idx */ 
  if ( TimerIdx >= TMR_MAX || TimerIdx == NO_TIMER_ID ) return -1;

  /* Check for active timer */ 
  if ( ! SECTIMER_USED(TimerIdx ) ) return -1;

  /* mark timer idx as unused */
  SECTIMER_FREE(TimerIdx);

  /* recalculate next timer event */
  sectmrFindNext();
  
  return TimerIdx;
}

/*********************************************************************************
  * @brief  permanently allocate a SecTimer ID. This SecTimerID will not be given
  *         to any other user. Once allocated, it will stay allocated forever
  * @param  id - NO_TIMER_ID in case of allocation of a new permanent timer
  *         id - any valid permanent SecTimerID in case of ReAllocation due to 
  *              re-execution of permanent allocation part 
  * @retval TimerIdx    = SecTimerID to use
  *         NO_TIMER_ID = SecTimerID could not be allocated
  ********************************************************************************/
int8_t SecTimerAllocate(int8_t id)
{
    if ( id == NO_TIMER_ID ) {
        id = secTmrUpper - 1;
        if ( id < 0 ||  SECTIMER_USED(id) ) {
             /* Report error, if not found */
            log_error ( "SecTimerAllocate: No free TimerID!" );
            return NO_TIMER_ID;
        }
        secTmrUpper = id;
        return id;
    } else {
        /* check for a valid permanent SecTimerID */
        if ( id >= secTmrUpper && id < TMR_MAX ) 
            return id;
        else    
            return NO_TIMER_ID;
    }
}

/*********************************************************************************
  * @brief  Reuse/Re-Set a previously allocated secTimer or allocate a new timer 
  *         and (re)start this timer ( w absolute subsecond value )
  * @param  id        - previously allocated SecTimerID 
  *                     or NO_TIMER_ID in case of new allocation
  *         secs      - absolute second value in the range [ 0 .. UINT32_MAX ]
  *         bPeriodic - if != 0  execute timer periodically
  *                   - if == 0  execute once
  *         myCB      - Callback function when timer expires
  * @note   secs has the unit [absolute seconds]. These are counted from system start
  *         within a 32 bit integer, i.e. a rollover occurs every 136 years, which makes
  *         an absolute periodic counter somewhat senseless
  * 
  * @retval TimerIdx    = ID of new Timer
  *         NO_TIMER_ID = Timer could not be allocated 
  ********************************************************************************/
static int8_t SecTimerReUseAbs( int8_t id, uint32_t secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg ) 
{
 
    if ( id < 0 || id >= TMR_MAX ) {
         /* Report error, if not found */
        log_error ( "SecTimerReUse: No valid Timer!" );
        return NO_TIMER_ID;
    }

    /* fill in neccessary data */
    SECTIMER_ALLOC(id);
    mySecTimers[id].uTime     = secs;
    mySecTimers[id].bPeriodic = bPeriodic;
    mySecTimers[id].cCallback = myCB;
    mySecTimers[id].arg       = arg;
    #if DEBUG_SECTIMER > 0
        DEBUG_PRINTF("SecTmr %d set to %d\n",id, secs);
    #endif

    /* recalculate next timer event */
    sectmrFindNext();

    return id; 
}

/*********************************************************************************
  * @brief  Allocate a new timer and start this timer ( w absolute subsecond value )
  * @param  second    - absolute second value in the range [ 0 .. UINT32_MAX ]
  *         bPeriodic - if != 0  execute timer periodically
  *                   - if == 0  execute once
  *         myCB      - Callback function when timer expires
  * @note   secs has the unit [absolute seconds]. These are counted from system start
  *         within a 32 bit integer, i.e. a rollover occurs every 136 years, which makes
  *         an absolute periodic counter somewhat senseless
  * @retval TimerIdx    = ID of new Timer
  *         NO_TIMER_ID = Timer could not be allocated 
  ********************************************************************************/
static int8_t SecTimerSetAbs ( uint32_t secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg ) 
{
    /* Get free Timer idx and mark as used  */
    int8_t id = sectmrGetNextFree();
    if ( id == NO_TIMER_ID ) {
         /* Report error, if not found */
        log_error ( "SecTimer: No free Timer!" );
        return id;
    }
    return SecTimerReUseAbs(id, secs, bPeriodic, myCB, arg );
}



/*********************************************************************************
 * @brief  Reuse/Re-Set a previous Allocated timer and restart this timer
 *         or allocate and start a new timer
 *
 * @param  id        - previous allocated secTimer-ID. If this ID is -1, 
 *                     a new timer will be allocated
 *         delta_secs- relative delta in the range [ 0 .. SECTIMER_MAX ]
 *         bPeriodic - if != 0  execute timer periodically
 *                   - if == 0  execute once
 *         myCB      - Callback function when timer expires
 * @note   delta_secs has the unit  [seconds] 
 * @retval TimerIdx    = ID of new Timer
 *         NO_TIMER_ID = Timer could not be allocated 
 ********************************************************************************/
int8_t SecTimerReUseRel ( int8_t id, uint32_t delta_secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  )
{
  /* Check for sec value being in allowed range [ 0 .. SECTIMER_MAX ] */
  if ( delta_secs > SECTIMER_MAX || delta_secs < 1 ) {
      log_error ( "SecTmrReUseRel: %d out of range", delta_secs );
      return NO_TIMER_ID;
  } 

  /* Compute absolute value */
  uint32_t abs_secs = GET_LINEAR_SECS() +  delta_secs;

  int8_t ret = SecTimerReUseAbs ( id, abs_secs, bPeriodic, myCB, arg ); 

  /* Set the relative Flag, if periodic */
  if ( bPeriodic > 0 && ret >= 0 ) {
    mySecTimers[ret].bPeriodic |= TMR_RELATIVE_FLAG;
    mySecTimers[ret].delta_t   = delta_secs;
  }

  return ret;
}
/*********************************************************************************
 * @brief  ReStart a previous Allocated timer, all values except delta_secs are
 *         unchanged
 *
 * @param  id         - previous allocated secTimer-ID. This Timer must be acive!
 *         delta_secs - new relative delta in the range [ 0 .. SECTIMER_MAX ]
 * @note   delta_secs has the unit  [seconds] 
 * @retval TimerIdx    = ID of Timer
 *         NO_TIMER_ID = TimerID not valid or Timer not running
 ********************************************************************************/
int8_t SecTimerReSetRel ( int8_t id, uint32_t delta_secs )
{
    /* Check for sec value being in allowed range [ 0 .. SECTIMER_MAX ] */
    if ( delta_secs > SECTIMER_MAX || delta_secs < 1 ) {
        log_error ( "SecTmrReSetRel: %d out of range", delta_secs );
        return NO_TIMER_ID;
    } 

    /* Compute absolute value */
    uint32_t secs = GET_LINEAR_SECS() +  delta_secs;

    /* Overwrite */
    mySecTimers[id].uTime     = secs;

    #if DEBUG_SECTIMER > 0
        DEBUG_PRINTF("SecTmr %d ReSet to %d\n",id, secs);
    #endif

    /* recalculate next timer event */
    sectmrFindNext();

  return id;
}

/*********************************************************************************
  * @brief  Allocate a new timer and start this timer ( w relative subsecond value )
  *
  * @param  delta_secs  - new relative delta in the range [ 0 .. SECTIMER_MAX ]
  *         bPeriodic   - if != 0  execute timer periodically
  *                     - if == 0  execute once
  *         myCB        - Callback function when timer expires
  * @note   delta_secs has the unit seconds
  * @retval TimerIdx    = ID of new Timer
  *         NO_TIMER_ID = Timer could not be allocated 
  *
  ********************************************************************************/
int8_t SecTimerSetRel ( uint32_t delta_secs, uint32_t bPeriodic, TimerCBT myCB, uint32_t arg  )
{
    /* Get free Timer idx and mark as used  */
    int8_t id = sectmrGetNextFree();
    if ( id == NO_TIMER_ID ) {
         /* Report error, if not found */
        log_error ( "SecTimer: No free Timer!" );
        return id;
    }
    return SecTimerReUseRel(id, delta_secs, bPeriodic, myCB, arg );
}



/*********************************************************************************
 * @brief  Handle Second timers. Should be called with every second change
 *         Will look for the next second timer to expire and -if matches-
 *         mark as expired and retrigger, if periodic
 * @note   will directly call timer callback, so do not call this routine
 *         in interrupt context
 *********************************************************************************/
void handle_sectimer_periodic(void)
{
    uint32_t i;

    /* check for next second timer match and exit if not */
    if ( !secsNextMatch || secsNextMatch != secsFromStart ) return;

    /* get matching value */
    uint32_t cnt = mySecTimers[idxNextMatch].uTime;

    /* Iterate for all used timers */
    for (i=0;i<TMR_MAX;i++) if (SECTIMER_USED(i)) {
        /* 
        * Does the timer value match the compare register?
        * then notice in "mask"
        */
        if ( mySecTimers[i].uTime == cnt ) {
            #if DEBUG_TIMER > 0
               DEBUG_PRINTF(" %d",i);
            #endif
            /* if periodic timer, recalc next time, if relative, if not periodic timer, delete.  */
            if ( SECTMR_GET_PERIODIC_VALUE(i) ) {
                if ( SECTMR_GET_FLAGS(i) & TMR_RELATIVE_FLAG ) mySecTimers[i].uTime += mySecTimers[i].delta_t;
            } else {
                SECTIMER_FREE(i);
            }
            /* if callback specified, execute  */
            if ( mySecTimers[i].cCallback ) mySecTimers[i].cCallback( mySecTimers[i].arg );

        }
    } // for, outer if


  /* recalculate next second timer match */
  sectmrFindNext();
}

#ifdef TX18LISTENER
    /*******************************************************************************
     * Return the time ( in seconds ) before timer_id will be fired
     * (Helper function for "FreeTimeSlot") 
     ******************************************************************************/
    bool SECTimerGetRel(int8_t timer_id, uint32_t *remaining_s )
    {

            if ( SECTIMER_USED(timer_id) ) {
                    *remaining_s = mySecTimers[timer_id].uTime - secsFromStart;
                    return true;
            } else {
                    return false;
            }
    }
#endif // TX18LISTENER


#endif /* -if USE_SECONDTIMER > 0 */

/**
  * @brief  Perform a system reset by watchdog reset
  * @note   function will never return
  * @param  None 
  *         
  * @retval None
  */

void TimerWatchdogReset_Internal(uint16_t num_of_ms, IWDG_TypeDef *myWD)
{
    uint8_t prescale_reg;
    uint8_t prescale_val;

    if (num_of_ms < 1)
    {
        num_of_ms = 1;
        prescale_reg = IWDG_PRESCALER_32;
        prescale_val = 1;
    }
    else if (num_of_ms <= 4096)
    {
        prescale_reg = IWDG_PRESCALER_32;
        prescale_val = 1;
    }
    else if (num_of_ms <= 8192)
    {
        prescale_reg = IWDG_PRESCALER_64;
        prescale_val = 2;
    }
    else if (num_of_ms <= 16384)
    {
        prescale_reg = IWDG_PRESCALER_128;
        prescale_val = 4;
    }
    else if (num_of_ms <= 32768)
    {
        prescale_reg = IWDG_PRESCALER_256;
        prescale_val = 8;
    }
    else
    {
        num_of_ms = 32768;
        prescale_reg = IWDG_PRESCALER_256;
        prescale_val = 8;
    }

#if 0
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    while (IWDG_GetFlagStatus(IWDG_FLAG_PVU));
    IWDG_SetPrescaler(prescale_reg);
    while (IWDG_GetFlagStatus(IWDG_FLAG_RVU));
    IWDG_SetReload(num_of_ms/prescale_val-1);
    IWDG_Enable();
#endif

  myWD->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;  /* Enable write     */
  while ( myWD->SR & IWDG_SR_PVU );         /* Wait for PVU statusbit being reset */
  myWD->PR = prescale_val;                  /* Prescaler as configured  */
  while ( myWD->SR & IWDG_SR_RVU );         /* Wait for RVU statusbit being reset */
  myWD->RLR = num_of_ms/prescale_val-1;      /* Write Reload register */
  myWD->KR = IWDG_KEY_RELOAD;               
  myWD->KR = IWDG_KEY_ENABLE;               /* Enable Watchdog  */

}


/**
  * @}
  */
