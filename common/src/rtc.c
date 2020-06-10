/**
  ******************************************************************************
  * @file    rtc.c
  * @author  Rainer 
  * @brief   Handle the RTC stuff
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup RTC
  * @{
  */
#include "config/devices_config.h"
#include "error.h"
#include "debug_helper.h"
#include "system/profiling.h"
#include "system/hw_util.h"
#include "task/minitask.h"
#include "rtc.h"
#include "timer.h"

#if USE_TIMER > 0
    #include "global_flags.h"
#endif

#define DEBUG_RTC           0

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Some checks */

#if USE_TIMER > 0
    #if !defined(RTCTIMER)
        #error "RTCTIMER not defined"
    #endif 
#endif

#if ( TIMR_SUBSEC_MAX & (TIMR_SUBSEC_MAX-1) ) != 0
    #error TIMR_SUBSEC_MAX must be a power of 2
#endif

#if ( TIMR_PREDIV_VALUE & (TIMR_PREDIV_VALUE-1) ) != 0
    #error TIMR_PREDIV_VALUE must be a power of 2
#endif

#if ( RTC_SUBSEC_MAX & (RTC_SUBSEC_MAX-1) ) != 0
    #error RTC_SUBSEC_MAX must be a power of 2
#endif

#if USE_RTC > 0
    #if   TIMR_PREDIV_VALUE ==  2
      #define RTC_WUCK_VAL      3 
    #elif TIMR_PREDIV_VALUE ==  4
      #define RTC_WUCK_VAL      2
    #elif TIMR_PREDIV_VALUE ==  8
      #define RTC_WUCK_VAL      1
    #elif TIMR_PREDIV_VALUE ==  16
      #define RTC_WUCK_VAL      0
    #else
      #error "Illegal WakeUp Clock Prescaler"
    #endif

    /* Private macro -------------------------------------------------------------*/
    #define ENABLE_WRITE()    do { RTC->WPR = 0xCA; RTC->WPR = 0x53; } while(0)
    #define DISABLE_WRITE()   (RTC->WPR = 0x00)

    /*
     * the follwing four macros require backup access to be enabled
     * and write Protection being disabled 
     */
    #define DisableIRQ()              (RTC->CR &= ~( RTC_CR_ALRAE | RTC_CR_ALRAIE ))
    #define EnableIRQ()               (RTC->CR |=  ( RTC_CR_ALRAE | RTC_CR_ALRAIE ))
#endif

/* 
 * year mod 100 = 0 is only every 400 years a leap year. 
 * we calculate only till the year 2255, so don't care
 */
#define RTC_NoLeapyear() (rtc.YY % 4) 

/* Private variables ---------------------------------------------------------*/
/* 
 * Flag for "switch back to wintertime has already been done" 
 * This is neccessary, because we have two times 03 a.m. in the night when
 * summertime ends
 */
static uint8_t rtc_DS = 0; 




/* Private function prototypes -----------------------------------------------*/
static uint32_t RTC_IsLastSunday(void);

/* Exported variables --------------------------------------------------------*/
uint32_t uptime_mins = 0;
#if USE_SECONDTIMER > 0
    uint32_t secsFromStart=0;        /* counts the seconds since system start */
#endif 

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
rtc_t rtc = { 26, 12, 12, 11, 59, 56, 1, 0 };

/* ---------------------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

#if USE_TIMER > 0
    /** day of month for each month from january to december */
    static const uint8_t RTC_DayOfMonthTable[] =
    { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    /**
     * @brief get the number of days of the actual month (1-12) and year (0-255: 2000-2255)
     *
     * @retval number of days for actual month (1-12) and year (0-255: 2000-2255*
     */
    static uint8_t RTC_DaysOfMonth()
    {
        uint8_t dom = RTC_DayOfMonthTable[rtc.MM-1];
        if ((rtc.MM == 2)&&(!RTC_NoLeapyear()))
            return 29; // leapyear feb=29
        return dom;
    }
#endif

static const uint16_t daysInYear [12] = {
	0, 
	31, 
	31+28, 
	31+28+31, 
	31+28+31+30,  
	31+28+31+30+31,
	31+28+31+30+31+30,
	31+28+31+30+31+30+31,
	31+28+31+30+31+30+31+31,
	31+28+31+30+31+30+31+31+30,
        31+28+31+30+31+30+31+31+30+31,
	31+28+31+30+31+30+31+31+30+31+30};

/******************************************************************************
 *  @brief set day of week for actual date
 *
 *  @note   valid for years from 2000 to 2255
 * 
 *  @retval 1=monday to 7=sunday, this is compatible to the RTC DOW-Coding
 *
 ******************************************************************************/
static void RTC_SetDayOfWeek(void)
{
    uint16_t day_of_year;
    uint16_t tmp_dow;

    // Day of year
    day_of_year = daysInYear[rtc.MM-1] + rtc.DD;
    if (rtc.MM > 2) { // february
        if (! RTC_NoLeapyear() ){
            day_of_year ++;
        }
    }
    // calc weekday
    tmp_dow = rtc.YY + ((rtc.YY-1) / 4) - ((rtc.YY-1) / 100) + day_of_year;
    // set DOW
    rtc.DOW = (uint8_t) ((tmp_dow + 5) % 7) +1;
    
}

/******************************************************************************
 *  @brief Check actual date/time for being a switch point to Summertime or to Wintertime
 *         i.e. last Sunday in March or last sunday in october
 * @param  None 
 *         
 * @retval None
 ******************************************************************************/
static void RTC_TimerCheckDS(void)
{
    // start of summertime: March, 2:00:00 ?
    if ((rtc.MM==3)&&(rtc.hh==2)){
        // Last Sunday ?
        if (RTC_IsLastSunday()){
            rtc.hh++; // 2:00 -> 3:00
        }
    }
    // end of summertime: October, 03:00, rtc_DS == 0
    if ((rtc.MM==10)&&(rtc.hh==3)&&(rtc_DS==0)){
        // Last Sunday ?
        if (RTC_IsLastSunday()){
            rtc.hh--; // 3:00 -> 2:00
            rtc_DS=1;
        }
    }
}


#if USE_TIMER > 0
    /*!
     *******************************************************************************
     *
     *  Add one day to date variables
     *
     *  \note
     *       calculate overflows, regarding leapyear, etc.
     *
     ******************************************************************************/
    static void RTC_AddOneDay(void)
    {
        uint8_t dom;
        // How many day has actual month
        dom = RTC_DaysOfMonth();

        if (++rtc.DD > dom) {                       // Next Month
            rtc.DD = 1;
            if (++rtc.MM > 12) {                    // Next year
                rtc.MM = 1;
                rtc.YY++;
            }
        // Clear Daylight saving Flag
        rtc_DS=0;
        }
        // next day of week
        rtc.DOW = (rtc.DOW %7)+1; // Monday = 1 Sat=7
    }
#endif

/*!
 *******************************************************************************
 *
 *  Add one second to clock variables
 *
 * \note
 *    - calculate overflows, regarding leapyear, etc.
 *    - process daylight saving
 *       - last sunday in march 1:59:59 -> 3:00:00
 *       - last sunday in october 2:59:59 -> 2:00:00
 *         ONLY ONE TIME -> set FLAG RTC_DS
 *         reset FLAG RTC_DS on next day
 * \note Is executed in interrupt context, keep it short!
 ******************************************************************************/
void RTC_AddOneSecond(void)
{
    // Paket counter starts at zero with every new slave, so reset it every second
    rtc.pkt_cnt=0;
    if (++rtc.ss >= 60) {
        rtc.ss = 0;
        uptime_mins++;
#if USE_RTC > 0
        if ( rtc.mm == 0 ) RTC_TimerCheckDS();
#elif USE_TIMER > 0
        // add one minute
        if (++rtc.mm >= 60) {
            rtc.mm = 0;
            // add one hour
            if (++rtc.hh >= 24) {
                rtc.hh = 0;
                RTC_AddOneDay();
            }
            RTC_TimerCheckDS();        
        }
#endif
    }
    #if USE_SECONDTIMER > 0
        // increment rotating counter for seconds
        secsFromStart++;
    #endif
}

#if USE_RTC > 0
    /*********************************************************************************
      * @brief  Writes 'srcval' into one BCD digit within dest
      *         the offset within dest and the width to write( 1..4 ) have to be
      *         specified, too
      *
      * @param  dest   - destination to write to
      *         srcval - value to write ( allowed range is 0 .. 9 )
      *         pos    - bit offset within dest to write the 1 digit to
      *         mask   - bit mask of the 1 digit
      *
      * @note   if dest referres to a write protected register, the write protection
      *         must have been disabled before 
      *
      * @retval None
      * 
      ********************************************************************************/
    static void RTC_WriteBCD9( __IO uint32_t *dest, uint32_t pos, uint32_t mask, uint32_t srcval )
    {
      srcval <<= pos;
   
      *dest &= ~mask;  
      *dest |= ( srcval & mask );
    }

    /*********************************************************************************
      * @brief  Writes 'srcval' into maximum two BCD digits within dest
      *         the offset within dest and the width to write( 1..4 ) have to be
      *         specified, to
      *
      * @param  dest   - destination to write to
      *         srcval - binary value to write ( allowed range is 0 .. 99 (
      *         pos1   - bit offset within dest to write the 1 digit to
      *         mask1  - bit mask for the 1 digit
      *         pos10  - bit offset within dest to write the 10 digit to
      *         mask10 - bit mask of the 10 digit
      *
      * @note   if dest referres to a write protected register, the write protection
      *         must have been disabled before 
      *
      * @retval None
      * 
      ********************************************************************************/
    static void RTC_WriteBCD99( __IO uint32_t *dest, uint32_t pos1, uint32_t mask1, uint32_t pos10, uint32_t mask10, uint32_t srcval )  
    {
       RTC_WriteBCD9(dest, pos1, mask1, srcval % 10);
       RTC_WriteBCD9(dest, pos10, mask10,  srcval / 10);
    }

    /*********************************************************************************
      * @brief  Extracts one BCD digit from srcval
      *
      * @param  pos    - bit offset within srcval to read the BCD value from
      *         mask   - bit mask to mask out the digit, allowed values are 1,3,7,0xf
      *         srcval - 32bit value from which the BCD will be masked out
      *
      * @note   returned value can be in the range 00 .. 0x0f. There is no check for
      *         allowed BCD range when mask ist 0x0f
      *
      * @retval BCD value in the range 0..9, 0x0a..0x0f are also possible and will
      *         NOT be flagged as error
      * 
      ********************************************************************************/
    static uint8_t RTC_ReadBCD9 ( uint32_t pos, uint32_t mask, uint32_t srcval )  
    {
      return ( srcval & mask ) >> pos;
    }


    /*********************************************************************************
      * @brief  Extracts two BCD digits from srcval and returns the binary value
      *
      * @param  pos1    - bit offset within srcval to read the 1 digit BCD value from
      *         mask1   - bit mask to mask out the 1 digit, allowed values are 1,3,7,0xf
      *         pos10   - bit offset within srcval to read the 10 digit BCD value from
      *         mask10  - bit mask to mask out the 10 digit, allowed values are 1,3,7,0xf
      *         srcval  - 32bit value from which the BCD digits will be masked out
      *
      * @note   returned value can be in the range 00 .. 99. There is no check for
      *         allowed BCD range when mask ist 0x0f, so values > 99 can be returned, too.
      *
      * @retval BCD value in the range 0..99, 100..255 are also possible and will
      *         NOT be flagged as error
      * 
      ********************************************************************************/
    static uint8_t RTC_ReadBCD99 ( uint32_t pos1, uint32_t mask1, uint32_t pos10, uint32_t mask10, uint32_t srcval )  
    {
      uint8_t ret;
   
       ret =  RTC_ReadBCD9(pos10, mask10, srcval) * 10;
       ret += RTC_ReadBCD9(pos1, mask1, srcval);

       return ret;
    }

    /*********************************************************************************
      * @brief  Setup RTC from internal rtc-variable
      *
      * @param  global var 'rtc' will be read
      *         
      * @note   assumes, that access to backup domain has been enabled before
      * @note   will unlock write protection an lock again when done
      *
      * @retval None
      * 
      ********************************************************************************/
    static void task_init_rtcFromVar(void)
    {
      /* Calculate Day of week */
      RTC_SetDayOfWeek();

      /* Enable write access */
      ENABLE_WRITE();

      /* Request Init Mode and wait for Init Mode becoming active, will take 2 RTC clock cycles */
      RTC->ISR = RTC_ISR_INIT;   
      while ( !(RTC->ISR & RTC_ISR_INITF) ) {};

      /*Set Date and Time */
      RTC_WriteBCD99( &RTC->DR, RTC_DR_YU_Pos, RTC_DR_YU_Msk, RTC_DR_YT_Pos, RTC_DR_YT_Msk, rtc.YY );  
      RTC_WriteBCD99( &RTC->DR, RTC_DR_MU_Pos, RTC_DR_MU_Msk, RTC_DR_MT_Pos, RTC_DR_MT_Msk, rtc.MM );  
      RTC_WriteBCD99( &RTC->DR, RTC_DR_DU_Pos, RTC_DR_DU_Msk, RTC_DR_DT_Pos, RTC_DR_DT_Msk, rtc.DD );  
      RTC_WriteBCD9( &RTC->DR,  RTC_DR_WDU_Pos, RTC_DR_WDU_Msk, rtc.DOW );

      RTC_WriteBCD99 ( &RTC->TR, RTC_TR_HU_Pos,  RTC_TR_HU_Msk,  RTC_TR_HT_Pos,  RTC_TR_HT_Msk,  rtc.hh );
      RTC_WriteBCD99 ( &RTC->TR, RTC_TR_MNU_Pos, RTC_TR_MNU_Msk, RTC_TR_MNT_Pos, RTC_TR_MNT_Msk, rtc.mm );
      RTC_WriteBCD99 ( &RTC->TR, RTC_TR_SU_Pos,  RTC_TR_SU_Msk,  RTC_TR_ST_Pos,  RTC_TR_ST_Msk,  rtc.ss );

 

      /* Clear Init bit */
      RTC->ISR &= ~RTC_ISR_INIT;
      /* Disable write access */
      DISABLE_WRITE();   
  
      /* if not bypassing shadow regs, wait for synchronization being done, will take 4 RTC clock cycles */ 
      if ( !(RTC->CR & RTC_CR_BYPSHAD) )
          while ( !(RTC->ISR & RTC_ISR_RSF) ) {};
    }

    /*********************************************************************************
     * @brief  Copy the RTC date and time to global rtc variable
     *
     * @retval None
     * 
     ********************************************************************************/
    void RTC_CopytoVar(void)
    {
      /*Read Date and Time */
      rtc.YY  = RTC_ReadBCD99( RTC_DR_YU_Pos, RTC_DR_YU_Msk, RTC_DR_YT_Pos, RTC_DR_YT_Msk, RTC->DR );  
      rtc.MM  = RTC_ReadBCD99( RTC_DR_MU_Pos, RTC_DR_MU_Msk, RTC_DR_MT_Pos, RTC_DR_MT_Msk, RTC->DR );  
      rtc.DD  = RTC_ReadBCD99( RTC_DR_DU_Pos, RTC_DR_DU_Msk, RTC_DR_DT_Pos, RTC_DR_DT_Msk, RTC->DR );  
      rtc.DOW = RTC_ReadBCD9(  RTC_DR_WDU_Pos, RTC_DR_WDU_Msk, RTC->DR );

      rtc.hh  = RTC_ReadBCD99 ( RTC_TR_HU_Pos,  RTC_TR_HU_Msk,  RTC_TR_HT_Pos,  RTC_TR_HT_Msk,  RTC->TR );
      rtc.mm  = RTC_ReadBCD99 ( RTC_TR_MNU_Pos, RTC_TR_MNU_Msk, RTC_TR_MNT_Pos, RTC_TR_MNT_Msk, RTC->TR );
      rtc.ss  = RTC_ReadBCD99 ( RTC_TR_SU_Pos,  RTC_TR_SU_Msk,  RTC_TR_ST_Pos,  RTC_TR_ST_Msk,  RTC->TR );
    }

#endif
#if USE_TIMER > 0 
    /*********************************************************************************
     * @brief  Increment second by one and activate RTC task. This routine will be
     *         called normally ( once a second ) out of the secondly interrupt handler
     *         or manually in case of negative time correction
     ********************************************************************************/
    static inline __attribute__((always_inline))
    void RTC_IncrementSecond ( void )
    {
        RTC_AddOneSecond();
        // DEBUG_PRINTF("+ %d\n",HAL_GetTick());
        TaskNotify(TASK_RTC);
        TaskNotify(TASK_PERIODIC);
    }

    /*********************************************************************************
     * @brief  Callback for second-timer when negative time correction is done
     *         Because when negative time correction is done, one second ( 0 or 30 )
     *         would miss, so a one shot timer is implemented to create this time
     *         tick manually
     ********************************************************************************/
    static void RTC_TimeCorr(uint32_t arg)
    {
        UNUSED(arg);
        // DEBUG_PRINTF("@%02x  Additional timer fired\n",  RTC_GetS256());
        RTC_IncrementSecond();
    }
    /*********************************************************************************
     * @brief  set the number of second fractions. This has to be a value between 0 
     *         and RTC_SUBSECS_MAX - 1. 
     *         As the timer value cannot be written directly, we set a special compare
     *         match which will reload the counter earlier/later than usual, depending
     *         from the delta between current and desired SubSecs
     *
     * @param  Second fraction value to set      
     * * 
     * @retval None
     ********************************************************************************/
    static void RTC_SetS256(uint32_t new_subsec)  
    {
        int32_t delta;

        uint16_t current_subsec =  RTC_GetS256();
        delta = current_subsec-new_subsec;

        #if DEBUG_RTC > 0
            DEBUG_PRINTF("current=%02x, new=%02x, delta=%d\n", current_subsec, new_subsec, delta );
        #endif

        if ( delta == 0 )  {
          // Case 3: No delta -> no corrective action
          return;
        }

        /* Temporarily disable RTCTIMER interrupts */
        HAL_NVIC_DisableIRQ(RTCTIMER_IRQn);

        if ( delta < 0 ) {
          /* 
           * Case 1: delta < 0 <=> actual time is behind new one
           * 1.increment seconds by one. This increment is not done directly,
           *   but by firing a one shot timer, that will trigger the increment
           *   This is done, because by a direct increment, one second (either 0 or 30 )
           *   would be omitted in the seconds sequence. 
           * 2. increment delta by RTC_SUBSECS_MAX
           * so we have an positive delta and execute case 2
           */
           #if DEBUG_RTC > 0
               DEBUG_PUTS("Increase Sec");
           #endif
           // DEBUG_PRINTF("@%02x  Set additional timer to %02x\n",  RTC_GetS256(), (uint8_t)delta);
           MsTimerSetAbs(SUBS256_TO_TIMERUNIT((uint8_t)delta), RTC_TimeCorr, 0);
           delta += RTC_SUBSEC_MAX;
        }
        if ( delta > 0 ) {
          /*
           * Case 2: delta > 0 <=> my actual time is in front of new one
           * - Increment ARR by delta temporarily an keep that in mind
           */
           delta += (RTC_SUBSEC_MAX);

          /* Check for unused comparator ( ARR=CMP ) and preserve this state */
          /* CMP must not be greater than ARR, so increase ARR first         */
          /* (When correcting, ARR is always greater than standard ARR )     */
           RTCTIMER->ARR = SUBS256_TO_TIMERUNIT(delta);

           SetFlagBit(gflags, GFLAG_TIMECORR_BIT);
           #if DEBUG_RTC > 0
              DEBUG_PRINTF("Add %02x to second\n", delta - RTC_SUBSEC_MAX );
           #endif
        }

        /* Reenable RTCTIMER interrupts */
        HAL_NVIC_EnableIRQ(RTCTIMER_IRQn);

    }
    /*********************************************************************************
     * @brief  return the subsec value. The subsec value is the naked CNT-register 
     *         content. 
     *
     * @Note   As the timer is operated asynchronously, the manual says to get a 
     *         reliable result, read the register twice until both readouts are 
     *         identical
     * 
     * @retval Subsec value
     ********************************************************************************/
     uint16_t RTC_GetTimer()
    {
       register uint32_t c1;
       register uint32_t c2;
       do {
        c1 = RTCTIMER->CNT;
        c2 = RTCTIMER->CNT;
       } while ( c1 != c2 );
       return (uint16_t)c1;
    }

#elif USE_RTC > 0 
    /*********************************************************************************
     * @brief  set the number of second fractions. This has to be a value between 0 
     *         and  RTC_SUBSECS_MAX - 1 ( usually 0xff ). 
     *         As the timer value cannot be written directly, we set a special compare
     *         match which will reload the counter earlier/later than usual, depending
     *         from the delta between current and desired SubSecs
     *
     * @param  Second fraction value to set      
     *
     * @Note   Executing the underlying SHIFT operation in RTC will take approx 24ms
     *         so either poll SHPF in ISR or ensure that there are no time accesses
     *         within the next 24 ms!
     * 
     * @retval None
     ********************************************************************************/
    void RTC_SetS256(uint32_t new_subsec)  
    {
        int32_t delta;
        uint32_t temp;
        #if DEBUG_RTC > 0
            uint32_t start = ProfilerGetMicrosecond();
        #endif

        uint16_t current_subsec =  RTC_GetS256();
        delta = new_subsec - current_subsec;
        #if DEBUG_RTC > 0
            DEBUG_PRINTF("current=%d, new=%d, delta=%d\n", current_subsec, new_subsec, delta );
        #endif

        if ( delta > 0 ) {
          /*
           * delta > 0 <=> actual time is behind new one
           * - decrement SSR(downcounter) by delta, Set ADD1S
           * - value = PREDIV_SYNC - delta
           */
           temp = (uint32_t)RTC_SUBSEC_MAX - delta;
           #if DEBUG_RTC > 0
               DEBUG_PRINTF("ADD1S=%d\n", delta );
           #endif
           temp |= RTC_SHIFTR_ADD1S;
        } else if ( delta < 0 ) {
          /*
           * delta < 0 <=> actual time is in front of new one
           * - Increment SSR(downcounter) by delta, Reset ADD1S
           * - value = -1 * delta;
           */
           temp = -1 * delta;
           #if DEBUG_RTC > 0
              DEBUG_PRINTF("SUB1S=%d\n", temp );
           #endif
        } else {
          // No delta -> no corrective action
          return;
        }

        /* Make sure, there are no pending corrections */

        assert ( ( RTC->ISR & RTC_ISR_SHPF) == 0);
    
        /* Do correction and wait for correction to be done */
 
        #if DEBUG_RTC > 0
            DEBUG_PRINTF("SSR=%d, SHIFT=%d\n", RTC->SSR, temp &0x7ffffff );
        #endif
        ENABLE_WRITE();
        RTC->SHIFTR = temp;
        DISABLE_WRITE();   

        /* Wait for Shift operation to be performed */
        while ( RTC->ISR & RTC_ISR_SHPF ) {};

        #if DEBUG_RTC > 0
           DEBUG_PRINTF("SSR=%d\n", RTC->SSR );
           temp = (ProfilerGetMicrosecond() - start + 500 ) / 1000;
          DEBUG_PRINTF("Dur1=%dms\n", temp);        
        #endif
 
        /* if not bypassing shadow regs, wait for synchronization being done, will take 4 RTC clock cycles */ 
        if ( !(RTC->CR & RTC_CR_BYPSHAD) ) {
            while ( !(RTC->ISR & RTC_ISR_RSF) ) {};
        
            #if DEBUG_RTC > 0
               temp = (ProfilerGetMicrosecond() - start + 500 ) / 1000;
               DEBUG_PRINTF("Dur2=%dms\n", temp);        
            #endif
        }
    }
#endif


/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
#if USE_RTC > 0
    void RTC_EnableBkUpAccess(void)
    {
        if ( !READ_BIT( PWR->CR1, PWR_CR1_DBP ) ){
            /* Enable the PWR Clock and Enable permanent access to the backup domain */
            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWR_EnableBkUpAccess();
          __HAL_RCC_PWR_CLK_DISABLE(); 
        }
    }

    void RTC_DisableBkUpAccess(void)
    {
        if ( READ_BIT( PWR->CR1, PWR_CR1_DBP ) ){
            /* Enable the PWR Clock and Disable permanent access to the backup domain */
            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWR_DisableBkUpAccess();
            __HAL_RCC_PWR_CLK_DISABLE(); 
        }
    }
#endif

/**
 *******************************************************************************
 *
 *  \returns true if actual date is last sunday in march or october
 *
 ******************************************************************************/
static uint32_t RTC_IsLastSunday(void)
{
    /* Not a sunday ? */
    if (rtc.DOW != 7) return 0;
   
    // March or October ?
    if (rtc.MM == 3){
        // last seven days of month
        return (rtc.DD > (31-7));
    } else if (rtc.MM == 10){
        // last seven days of month
        return (rtc.DD > (30-7));
    } else {
        return 0;  // not march or october
    }    
}





void RTC_SetDateTime(uint8_t dd, uint8_t mm, uint8_t yy, uint8_t hr, uint8_t mi, uint8_t sec, uint8_t s256) 
{
  bool bChanged = false;

/*local*/ void modify ( uint8_t *curval, uint8_t newval )
/*local*/ {
/*local*/    if ( *curval != newval ) {
/*local*/      bChanged = true;
/*local*/      *curval = newval;
/*local*/    }
/*local*/ }

#if USE_RTC > 0
  RTC_EnableBkUpAccess();
  DisableIRQ();
#endif

  /* store to my data structure */
  modify (&rtc.DD, dd);
  modify (&rtc.MM, mm);
  modify (&rtc.YY, yy);
  
#if USE_TIMER > 0 
  rtc.hh =hr;
  rtc.mm =mi;
  rtc.ss =sec;

  /* Update DoW, if date was changed */
  if ( bChanged ) RTC_SetDayOfWeek();

#elif USE_RTC > 0
  modify (&rtc.hh, hr);
  modify (&rtc.mm, mi);
  modify (&rtc.ss, sec);
  
  /* 
   * Update RTC vars only, if they were changed 
   * This will also reset the subsecs to 0.
   * So be sure to set them afterwards
  */
  if ( bChanged )task_init_rtcFromVar();
#endif
  RTC_SetS256(s256);

#if USE_RTC > 0
  EnableIRQ();
  // RTC_DisableBkUpAccess();
#endif

}

#if USE_TIMER > 0
    /*********************************************************************************
      * @brief  Set the milliseconds compare match value. This value MUST STRICTLY
      *         be lower than period 
      *         ( otherwise period is increased to compare match value + 1 )
      *         And value must not be the "unused value" 0x00 
      *         If this value is the Compare Match value, 1 is use instead
      *
      * @param  value - timer value in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
      *         this value is relative to <now>, the unit is <WakUpClockTicks>
      *                       
      * @retval true - if cmp value have been set ( and not kept in mind )
      *
      ********************************************************************************/
    bool RTC_SetCmpMatch(uint32_t value  )
    {

      /* 
       * The comparator does not trigger when the desired value is reached, but
       * when the value is exceeded by one, so reduce the value by one.
       * also the maximum comparator value has a special meaning, so decrement only
       * if value > 0
       */

      /* value MUST NOT exceed period */
      if ( value >= TIMR_SUBSEC_MAX ) {
        log_error( "CmpMatch subsec %d too big",value);
        return false;
      }

      /* value MUST NOT be identical to CMP_FORBIDDEN_VALUE */
      if ( value ==  CMP_FORBIDDEN_VALUE ) {
        log_error( "CmpMatch forbidden value");
        return false;
      }
        
      if ( (RTCTIMER->ISR & LPTIM_ISR_CMPOK_Msk) == 0 ) {
        log_error( "Cmp register not ready");
        return false;
      }

      RTCTIMER->CMP=value;
      #if DEBUG_RTC > 0
          uint16_t c = RTC_GetTimer();
          DEBUG_PRINTF("@%04x: RTCTIMER->CNT=%04x,RTCTIMER->CMP=%04x\n",c, c, RTCTIMER->CMP);
      #endif      
      return true;


    }

#elif USE_RTC > 0
    /*********************************************************************************
      * @brief  Set the milliseconds compare match value. This value MUST NOT exceed
      *         period ( otherwise period is increased to compare match value + 1 )
      *         And value must not be the "unused value". 
      *         In this case value is increased by 1
      *
      * @param  value - timer value in the range [ 0 .. TIMR_SUBSEC_MAX-1 ]
      *         this value is relative to <now>, the unit is <WakUpClockTicks>
      *                       
      * @retval true - if cmp value have been set ( and not kept in mind )
      *
      ********************************************************************************/
    bool RTC_SetCmpMatch(uint32_t value )
    {

      /* value MUST NOT exceed period */
      if ( value >= TIMR_SUBSEC_MAX ) {
        log_error( "Alarm subsec %d too big",value);
        return false;
      }
  
      register uint32_t current_subs = RTC_GetTimer());
 
      uint32_t rel_val = SUBSEC_DIFF(value, current_subs);

      /* If WUCk is enabled (due to a parallel running timer, temporarily disable */
      if ( RTC->CR & RTC_CR_WUTE ) {
        DisableTmrIRQ(); 
      }    

      RTC_EnableBkUpAccess();
      /* Enable write access */
      ENABLE_WRITE();
  
      /* Wait for Write Flag to become set, then update WUTR and re-enable WUCK */
      while ( !(RTC->ISR & RTC_ISR_WUTWF )) ;
      RTC->WUTR = rel_val;
      EnableTmrIRQ();
 
      /* Disable write access */
      DISABLE_WRITE();
 
      return true;
    }
#endif


static char rtcbuf[25];

char* RTC_GetStrDateTimeMillis(void)
{
  sprintf(rtcbuf, "%02d.%02d.%02d %02d:%02d:%02d.%03d", rtc.DD, rtc.MM,rtc.YY, rtc.hh, rtc.mm, rtc.ss, RTC_GetMillis() );
  return rtcbuf;
}

char* RTC_GetStrDateTime(void)
{
#if USE_RTC > 0
  RTC_CopytoVar();
#endif
  sprintf(rtcbuf, "%02d.%02d.%02d %02d:%02d:%02d", rtc.DD, rtc.MM,rtc.YY, rtc.hh, rtc.mm, rtc.ss );
  return rtcbuf;
}

char* RTC_GetStrTime(void)
{
  sprintf(rtcbuf, "%02d:%02d:%02d", rtc.hh, rtc.mm, rtc.ss );
  return rtcbuf;
}

void RTC_DumpDateTime(void)
{  
  DEBUG_PUTS(RTC_GetStrDateTime());
}

/******************************************************************************
 * Implements a simple stopwatch that counts milliseconds since start of stop
 * watch. 
 * \note The granularity is the granularity of the system timer
 * \note maximum delta-t is one hour
 *****************************************************************************/

/******************************************************************************
 * @brief  Returns the number of milliseconds since beginning of hour
 ******************************************************************************/
uint32_t RTC_GetMillisOfHour(void)
{
    register uint32_t ret;

    ret = (uint32_t)   RTC_GetMinute();
    ret = ret * 60   + RTC_GetSecond();
    ret = ret * 1000 + RTC_GetMillis();

    return ret;
}

#define RTC_MAX_MILLIS_OF_HOUR        (60*60*1000)
static uint32_t starttime;                              // Starttime of stopwatch 
static bool bStopWatchInUse = false;                    // flag for "stopwatch in use"

/******************************************************************************
 * \brief  Compute the delta-t between start of stopwatch and now
 * \note   Checks whether stopwatch is started, if not 0 will be returned
 ******************************************************************************/
static uint32_t StopWatch_GetDelta(void)
{
    /* check if stopwatch is started */
    if ( !bStopWatchInUse ) {
        DEBUG_PUTS("RTC Stopwatch not started!");
        return 0;
    }    

    /* get delta t from start of stopwatch to now */
    uint32_t tnow = RTC_GetMillisOfHour();
    if ( tnow < starttime ) tnow += RTC_MAX_MILLIS_OF_HOUR;

    return tnow-starttime;
}

/******************************************************************************
 * \brief  Start the simple stopwatch
 ******************************************************************************/
void RTC_StopWatch_Start(void)
{
    if ( bStopWatchInUse ) {
        DEBUG_PUTS("Error: RTC Stopwatch in Use!");
        return;
    }    

    starttime       = RTC_GetMillisOfHour();
    bStopWatchInUse = true;
}

/******************************************************************************
 * \brief return the milliseconds since start of stopwatch
 ******************************************************************************/
uint32_t RTC_StopWatch_GetTime(void)
{
    return StopWatch_GetDelta();
}

/******************************************************************************
 * \brief stop the stopwatch and return the milliseconds since start of stopwatch
 ******************************************************************************/
uint32_t RTC_StopWatch_Stop(void)
{
    register uint32_t ret   =   StopWatch_GetDelta();
    bStopWatchInUse         = false;

    return ret;
}

/******************************************************************************
 * \brief returns true, iff the stopwatch is in use
 ******************************************************************************/
bool RTC_StopWatch_InUse(void)
{
    return bStopWatchInUse;
}


#if USE_TIMER > 0
    /******************************************************************************
     * @brief  Initialize RTCTIMER as asynchronous counter clocked by LSE
     *         ARR is set in that way, that exactly every second an overflow
     *         occurs.
     ******************************************************************************/
    void task_init_rtc(void)
    {
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      PeriphClkInit.PeriphClockSelection = 0;

      if ( RTCTIMER == LPTIM1 ) {
          PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM1; 
          PeriphClkInit.Lptim1ClockSelection   = RCC_LPTIM1CLKSOURCE_LSE;
      } else if ( RTCTIMER == LPTIM2 ) {
          PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM2; 
          PeriphClkInit.Lptim2ClockSelection   = RCC_LPTIM2CLKSOURCE_LSE;
      #if defined( LPTIM3 )
      } else if ( RTCTIMER == LPTIM3 ) {
          PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM3; 
          PeriphClkInit.Lptim345ClockSelection = RCC_LPTIM3CLKSOURCE_LSE;
      #endif
      #if defined( LPTIM4 )
      } else if ( RTCTIMER == LPTIM4 ) {
          PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM4; 
          PeriphClkInit.Lptim345ClockSelection = RCC_LPTIM4CLKSOURCE_LSE;
      #endif
      #if defined( LPTIM5 )
      } else if ( RTCTIMER == LPTIM5 ) {
          PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM5; 
          PeriphClkInit.Lptim345ClockSelection = RCC_LPTIM5CLKSOURCE_LSE;
      #endif 
      } else {
        DEBUG_PUTS("Error: No clock assignment for RTC timer");
        return;
      }

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("Failed to set LPTIMx-Clock\n");
        return;
      }

      HW_SetHWClock(RTCTIMER, true);

      /* Enable compare match and overrun interrupt*/
      RTCTIMER->IER = LPTIM_IER_CMPMIE | LPTIM_IER_ARRMIE;
      // for tests: RTCTIMER->IER = 0;
    
      /* Set prescaler in CFGR, everything else in CFGR is ok */
      MODIFY_REG(RTCTIMER->CFGR, LPTIM_CFGR_PRESC_Msk, (HW_GetLn2(TIMR_PREDIV_VALUE) << LPTIM_CFGR_PRESC_Pos) );


      /* Preset ARR to cycle within exactly one second, timer must be enabled to set ARR */
      RTCTIMER->CR = LPTIM_CR_ENABLE; 
      RTCTIMER->ARR = TIMR_SUBSEC_MAX-1;
      /* Prest CMP with "unused" */
      DisableTmrIRQ();

      /* Enable RTCTIMER global interrupt */
      HAL_NVIC_SetPriority(RTCTIMER_IRQn, RTC_IRQ_PRIO, 0);
      HAL_NVIC_EnableIRQ(RTCTIMER_IRQn);

      /* Start timer */
      SET_BIT(RTCTIMER->CR, LPTIM_CR_CNTSTRT);
    }
#elif USE_RTC > 0
    /******************************************************************************
     * @brief  Setup the RTC initially
     *         
     * @note - The RTC clock source must have been set outside this function
     *       - The Bckup Protection in the PWR->CR1 will be permanently disabled
     *       - The Date/Time will only be overwritten, if not already set before
     *
     * @retval    None
     *
     */
    void task_init_rtc(void)
    {
  
      RTC_EnableBkUpAccess();
 
      /* Enable RTC Clock */ 
      __HAL_RCC_RTC_ENABLE(); 

      /* Prescalers will not be changed in any case, check for default value */
      assert ( (RTC->PRER & RTC_PRER_PREDIV_A_Msk ) >> RTC_PRER_PREDIV_A_Pos == RTC_PREDIV_ASYNC );
      assert ( (RTC->PRER & RTC_PRER_PREDIV_S_Msk ) >> RTC_PRER_PREDIV_S_Pos == RTC_PREDIV_SYNC );

      /* Set default date/time only if calendar has not been initialized before */
      // if ( !(RTC->ISR & RTC_ISR_INITS) ) 
     task_init_rtcFromVar();
  
  
      /* RTC Alarm Interrupt Configuration: EXTI configuration */
      __HAL_RTC_ALARM_EXTI_ENABLE_IT();
      __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();

      /* RTC Alarm Interrupt Configuration: NVIC configuration */
      HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

      /* RTC WakeUp Interrupt Configuration: EXTI configuration */
      __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
      __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_RISING_EDGE();

      /* RTC WakeUp Interrupt Configuration: NVIC configuration */
      HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

      /* Set Alarm A to be triggered at start of every second */

      /* Enable write access */
      ENABLE_WRITE();

      /* Set Date and Time and HR24-format */
      RTC->CR &= ~RTC_CR_FMT;

      /* 
       * Bypass shadow registers for easier handling, inconsitencies btw subsec, time and date cannot happen,
       * we keep these in global variable rtc
       */
      RTC->CR |= RTC_CR_BYPSHAD;

      /* Subseconds to interval start, all bits are valid */
      RTC->ALRMASSR = RTC_PREDIV_SYNC | RTC_ALRMASSR_MASKSS_Msk;   
  
      /* Ignore day,hrs, min, sec */
      RTC->ALRMAR = RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1 ;

      /* Wakeup Prescaler */  
      MODIFY_REG(RTC->CR, RTC_CR_WUCKSEL_Msk, RTC_WUCK_VAL);

      /* Enable Alarm A and Alarm A interrupt */
      EnableIRQ();

      /* Disable WUCK interrupts */
      DisableTmrIRQ();
  
      /* Disable write access */
      DISABLE_WRITE();
}
#endif


/* ---------------------------------------------------------------------------*/
/* Interrupt Handlers --------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
#if USE_TIMER > 0
    /*********************************************************************************
     * @brief  RTCTIMER global interrupt, only Compare match is enabled
     *         Used by the system for all millisecond-timers 
     *
     * @see    timer.c         
     *
     * @param  None
     *                       
     * @retval None
     *
     ********************************************************************************/
    void RTCTIMER_IRQHandler(void)
    {
        uint16_t tmrcnt;
        ProfilerPush(JOB_IRQ_RTC);
 
        if ( RTCTIMER->ISR & LPTIM_ISR_ARRM) {
            /* Overflow -> RTC second ticks */
            /* 
             * Adding one Second must be done in interrupt, because immediately after 
             * wakeup in normal program flow the profiler will read the systemtime
             * So, to actualize it before normal program flow continuation, it has to be
             * done within interrupt context
             */
            RTC_IncrementSecond();
            /* if time correction was performed, reset ARR to default value */
            if ( IsFlagBitSet(gflags, GFLAG_TIMECORR_BIT) ) {
                RTCTIMER->ARR = TIMR_SUBSEC_MAX-1;
                ClearFlagBit(gflags, GFLAG_TIMECORR_BIT);
            }
            RTCTIMER->ICR = LPTIM_ICR_ARRMCF;
        }
    
        /* Comparator is always on, we have an "unused" comparator value, that signals */
        /* that comparator is currently unused: Interrupt is triggered nevertheless    */
        /* This is due to changing the CMPIE bit would require starting and stopping   */
        /* the timer, which would infer some time skew                                 */
        if ( RTCTIMER->ISR & LPTIM_ISR_CMPM) {
            tmrcnt = RTC_GetTimer();
            ProfilerSwitchTo(JOB_IRQ_TMR);
            // uint16_t cmp = RTCTIMER->CNT - 1;
            RTCTIMER->ICR = LPTIM_ICR_CMPMCF;
            // if ( (cmp & 0xf0ff) != 0 ) {
                if ( TmrIrqIsEnabled() ) {
                     #if DEBUG_RTC > 1
                         DEBUG_PRINTF("Tmr CMP IRQ CMP=%04x, CNT=%04x  \n", RTCTIMER->CMP, tmrcnt);
                     #endif
                     /* Ignore pending interrupts from previous "unused" value */
                     if ( tmrcnt != CMP_FORBIDDEN_VALUE + 1 ) {
                            MsTimerHandleCMP(tmrcnt-1 );
                            /* Set Task bit in any case.This is neccessary to compute next timer match */
                            TaskNotify(TASK_TMR);
                     }
                }
            // } else {
            //    RTCTIMER->CMP += TIMR_SUBSEC_MAX / 4;
            // }
        }

        ProfilerPop();
    }
#elif USE_RTC > 0 
    /*********************************************************************************
      * @brief  RTC Interrupt callback is called exactly once a second. 
      *         Used to inject a 1Hz heartbeat into the system. 
      *         Used by the system to increment the uptime mins and seconds,
      *         to copy the RTC date time into the rtc-variable 
      *         and to check for Daylight saving time switching
      *
      * @see    task_handle_rtc in rtc.c         
      * @see    every_second_task in main.c
      *
      * @param  None
      *                       
      * @retval None
      *
      ********************************************************************************/
    void RTC_Alarm_IRQHandler(void)
    {
      ProfilerPush(JOB_IRQ_RTC);

      /* Clear exti interrupt bit */
      EXTI->PR1 = EXTI_PR1_PIF18;

      /* Only Alarm A and B are handeled */
      #define RTC_ALARMS ( RTC_ISR_ALRAF | RTC_ISR_ALRBF )
  
      /* Alarm A used for secondly ticks */
      if ( RTC->ISR & RTC_ISR_ALRAF ) {
        TaskNotify(TASK_RTC);
        TaskNotify(TASK_PERIODIC);
      }

      /* reset all interrupt bits by writing 0 to */
      ENABLE_WRITE();
      RTC->ISR &= ~RTC_ALARMS;
      DISABLE_WRITE();

      ProfilerPop();
    }

    void RTC_WKUP_IRQHandler(void)
    {
      ProfilerPush(JOB_IRQ_TMR);

      /* Clear exti wakeup interrupt bit */
      EXTI->PR1 = EXTI_PR1_PIF20;

      /* Only WakeUp is handeled */
      #define RTC_WKUP   ( RTC_ISR_WUTF )
  
      /* WakeUp timer used for millisecond timers */
      if ( RTC->ISR & RTC_ISR_WUTF ) {
         /* Disable WkUp interrupt, this will also enable write on WUTR */
         ENABLE_WRITE();
         DisableTmrIRQ(); 
         DISABLE_WRITE();
         // If there is an matching timer, activate Timer Task
         if ( MsTimerHandleCMP(RHB todo: Actual timer value) ) TaskNotify(TASK_TMR);      
      }

      /* reset all interrupt bits by writing 0 to */
      RTC->ISR &= ~RTC_WKUP;

      ProfilerPop();
    }
#endif



/*********************************************************************************
  * @brief  callback for every rtc event, i.e. with every second increment
  *         what todo: check the second timers         
  *         
  * @param  None
  *                       
  * @retval None
  *
  ********************************************************************************/
void task_handle_rtc(uint32_t arg)
{

    UNUSED(arg);

    #if USE_RTC > 0
        RTC_CopytoVar();
    #endif
    
    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0
        // COM_print_time('+', true); // print_hexXXXX(RTCTIMER->CNT); CRLF();
    #endif

    handle_sectimer_periodic();

}


/****************************************************************
 Compress the actual date in a 4-byte-integer
 Byte 0: yy yy yy yy yy yy mm mm
 Byte 1: mm mm dd dd dd dd dd hh
 Byte 2: hh hh hh hh mi mi mi mi
 Byte 3: mi mi ss ss ss ss ss ss
****************************************************************/
 
void compress_datetime ( union CTIME_struct *ctime, rtc_t *rtc )
{
	uint8_t i=0;
	uint8_t w1				= rtc->MM;
    ctime->timebyte[i++]	= (rtc->YY<<2) | (w1 >> 2);
    uint8_t w2 				= rtc->hh; 
    ctime->timebyte[i++]	= (w1 << 6) | (rtc->DD<<1) | (w2 >> 4);
	w1						= rtc->mm;
    ctime->timebyte[i++] 	= (w2<<4)|(w1>>2); 
    ctime->timebyte[i  ]	= (w1<<4)| rtc->ss;
}

/****************************************************************
 Uncompress a compressed datetime-value
****************************************************************/
void uncompress_datetime ( union CTIME_struct *ctime, rtc_t *rtc )
{
	uint8_t i=0;
	uint8_t w1,w2;
	
	w1 = ctime->timebyte[i++];				// Byte 0
	w2 = ctime->timebyte[i++];				// Byte 1
	rtc->YY = w1>>2;
	rtc->MM = ((w1<<2)&0b1100) | (w2>>6);
	rtc->DD = (w2>>1)&0b11111;

	w1 = ctime->timebyte[i++];				// Byte 2
	rtc->hh = ((w2&0b1)<<4) | (w1>>4);
	w2 = ctime->timebyte[i  ];				// Byte 3
	rtc->mm = ((w1&0b1111)<<2) | (w2>>6);
	rtc->ss = (w2&0b111111);
}



/**
  * @}
  */
