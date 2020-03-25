/**
  ******************************************************************************
  * @file    profiling.c
  * @author  Rainer 
  * @brief   Do some kind of profiling on a 1-us-timer/counter. 
  * @note    The minimum system frequency to sensefully use this feature is
             4MHz or Higher. At lower frequencies, the microsecond counter
             overflow interrupt tends to get chocked
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup Profiling
  * @{
  */

#include "config/config.h"
#if DEBUG_PROFILING  > 0

 /* 
  * Set > 0, if consistency checks at runtime should be done regarding the 
  * microsecond counter. If system frequency is too low ( below 4 MHz )
  * the microsecond counter interrupt tends to be congested
  */
#define ADDITIONAL_CHECKS 0         

#include "error.h"
#include "global_flags.h"
#include "debug_helper.h"
#include "system/profiling.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"


/* Private typedef --------------------------------------------------------------*/
/* Private define ---------------------------------------------------------------*/

#define MICROCOUNTER_FREQ   1000000UL
#define STACK_DEPTH         8

/* Private macro ----------------------------------------------------------------*/
#define SET_LASTTIME(a)   jLastTime = a
#define GET_LASTTIME()    (jLastTime)
#if ADDITIONAL_CHECKS == 0
  #define Increment(a,b)    ( *(a) += (b) )
#endif

/* Private variables ------------------------------------------------------------*/
static ActiveJobEnumType jActive;
static uint32_t jLastTime;
static int jStackPtr=STACK_DEPTH;
static int jStackMin=STACK_DEPTH;
static ActiveJobEnumType jStack[STACK_DEPTH];
static char * const jobnames[JOB_NROF_ELEMENTS] =  { JOBNAMES99 };

#if DEBUG_FEATURES > 0
    char ts_buffer[12];
#endif


uint64_t ProfilerTimes[JOB_NROF_ELEMENTS];

/* Private functions ------------------------------------------------------------*/

/* Exported functions -----------------------------------------------------------*/


#if  0
/* Private function prototypes --------------------------------------------------*/
void MICROCOUNTER_TMR_IRQHandler(void);

/* Exported variables -----------------------------------------------------------*/
volatile uint32_t ProfilerMicroCountHigh;  /* The upper 16 bit of the microsecond-counter */
/* Exported functions -----------------------------------------------------------*/
/**
  * @brief  Setup the Microsecond-Counter according to parameter uCoreClock
  *         in that way, that the counter will count up exactly every microsecond
  * @note   The Microsecond-timer requires an input clock of at least 1MHz
  * @param  None 
  *         
  * @retval    0 on failure
  *            1 on success
  */
uint32_t ProfilerSetupMicroCounter()
{
  TIM_HandleTypeDef    TimHandle;

  uint32_t uApb1TmrClk =  GetAPB1TimerFrequency();
  DEBUG_PRINTF("APB1 timer clock ...=%d\n", uApb1TmrClk);

  if ( uApb1TmrClk < 1000000 ) {
    return 0;
  }

  uint32_t uwPrescalerValue = (uint16_t)((uApb1TmrClk /  MICROCOUNTER_FREQ ) - 1  );    

  /* Enable Timer Clock */
  MICROCOUNTER_TMR_CLK_ENABLE();

  /* Initialize TIMx peripheral as follows:
       + Period             = 65536
       + Prescaler          = calculated before to give 1 MHz timer input frequency
       + ClockDivision      = 1
       + Counter direction  = Up
  */
  TimHandle.Instance               = MICROCOUNTER_TMR;
  TimHandle.Init.Period            = 0xffff;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  /* Init Timer */
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
    return 0;
  }

   /* 
    * Give Interrupt the highest possible priority to avoid deferred actualizations of the counter High variable 
    * No other Interrupt should have this priority 
    */
   HAL_NVIC_SetPriority(MICROCOUNTER_TMR_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(MICROCOUNTER_TMR_IRQn);

  
  /* Start Timer */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
    return 0;
  }

  return 1;
}


#if ADDITIONAL_CHECKS > 0
  static uint16_t cnt;
  static uint32_t assert_check ( uint32_t newval, uint32_t oldval )
  {
      if ( !oldval ) return 1;
      uint32_t diff = newval-oldval;
      if ( diff > 0x7ffffff ) {
        DEBUG_PRINTF("Assert failed: new=%u, old=%u, CNT=%u\n", newval, oldval, cnt);
        return 1;
      } else {
        return 0;
      }
  }
#endif

/**
  * @brief  Get the actual microsecond counter value as 32-bit value
  *         This value is assembled by the 16 counter bits as low word
            and the 16 bits from the High-Counter variable
  * @param  None 
  *         
  * @retval the actual microsecond counter value
  */
#if ADDITIONAL_CHECKS > 0
  static uint32_t last_us=0;
#endif
uint32_t ProfilerGetMicrosecond(void)
{
#if ADDITIONAL_CHECKS == 0
  uint16_t cnt;
#endif

  /* Don't use counter values near the upper limit, wait for next period */
  do {  
    cnt = MICROCOUNTER_TMR->CNT;
  } while ( cnt >= 0xfff8 );

  /* 
   * Execute interrupt, if pending. Ths can be the case, when profiler
   * is called from hihger priority interrupt routine
   */
  if (  MICROCOUNTER_TMR->SR & TIM_SR_UIF ) MICROCOUNTER_TMR_IRQHandler();

  cnt = MICROCOUNTER_TMR->CNT;
  uint32_t ret = ProfilerMicroCountHigh << 16 | cnt;
  
  #if ADDITIONAL_CHECKS > 0
    if (assert_check(ret, last_us) ) {
      DEBUG_PRINTF("cnt=%d, oldcnt=%d, OVF=%d\n", cnt, last_us, MICROCOUNTER_TMR->SR & TIM_SR_UIF); 
    }
  last_us=ret;
  #endif

  return ret;
}
#endif

#if DEBUG_FEATURES > 0
    char *ProfilerGetTS(void)
    {
        char *ptr = ts_buffer+7;
        *ptr-- = '\0';
        register uint32_t work = ProfilerGetMicrosecond();
        register uint32_t work10;
        
        for ( int32_t i = 6; i > 0; i-- ) {
            work10 = work/10;
            *ptr-- = '0' + (char)(work-work10*10);
            if ( i == 4) *ptr-- = '.';
            work = work10;
        }

        return ts_buffer;
    }
#endif

/*********************************************************************************
  * @brief  Init all profiling variables and set the active job to 'active' 
  *         
  * @param  active - the new job ID
  *         
  * @retval None
  ********************************************************************************/
void     ProfilerInitTo(ActiveJobEnumType active )
{
  for ( ActiveJobEnumType i=0; i < JOB_NROF_ELEMENTS; i++ )
     ProfilerTimes[i]=0;
  
  jStackPtr=STACK_DEPTH;
  jStackMin=STACK_DEPTH;
  jActive = active;
  SET_LASTTIME(ProfilerGetMicrosecond());
  
}
#if ADDITIONAL_CHECKS > 0
  static void Increment ( uint64_t *value, uint32_t delta )
  {
    if ( delta > 1000000 ) {
      DEBUG_PUTS("Wrong Delta!");
    }
    *value += delta;
  }
#endif

/*********************************************************************************
  * @brief  Set the active job to 'active' 
  *         the time spent from the last markup until now 
  *         is added to the current job time
  * @param  active - the new job ID
  *         
  * @retval None
  ********************************************************************************/
void     ProfilerSwitchTo(ActiveJobEnumType active )
{
  __disable_irq();
  uint32_t tempEntry = ProfilerGetMicrosecond();
  //assert_check(tempEntry, jLastTime);

  Increment(&ProfilerTimes[jActive], tempEntry - GET_LASTTIME() );
  jActive=active;

  uint32_t tempExit = ProfilerGetMicrosecond();
  Increment(&ProfilerTimes[JOB_TASK_PROFILER], tempExit-tempEntry );
  SET_LASTTIME(tempExit);
  __enable_irq();
}

/*********************************************************************************
  * @brief  Set the active job to 'active' 
  *         the time spent from the last markup until now is added to the current
  *         job time. The previous active job ID is pushed to stack
  *         see also ProfilerPop()
  * @param  active - the new job ID
  *         
  * @retval None
  ********************************************************************************/
void     ProfilerPush(ActiveJobEnumType active )
{
  __disable_irq();
  uint32_t tempEntry = ProfilerGetMicrosecond();
  //assert_check(tempEntry, jLastTime);

  if ( --jStackPtr < 0 ) {
    DEBUG_PUTS("ERROR in ProfilerPush: Stack Overflow");
  }

  /* keep track of the maximum stack depth */
  if ( jStackPtr < jStackMin ) jStackMin = jStackPtr;

  Increment(&ProfilerTimes[jActive], tempEntry - GET_LASTTIME() );
  jStack[jStackPtr] = jActive;
  jActive=active;

  uint32_t tempExit = ProfilerGetMicrosecond();
  //assert_check(tempExit,tempEntry);
  Increment(&ProfilerTimes[JOB_TASK_PROFILER], tempExit-tempEntry );
  SET_LASTTIME(tempExit);
  __enable_irq();
}

/*********************************************************************************
  * @brief  changes the active Job to the ID on top of stack 
  *         the time spent from the last markup until now is added to the current
  *         job time. 
  *         see also ProfilerPush()
  * @param  None
  *         
  * @retval None
  ********************************************************************************/
void ProfilerPop()
{
  __disable_irq();
  uint32_t tempEntry = ProfilerGetMicrosecond();
  //assert_check(tempEntry, jLastTime);

  if ( jStackPtr >= STACK_DEPTH ) {
    DEBUG_PUTS("ERROR in ProfilerPop: Stack Empty");
  }

  Increment(&ProfilerTimes[jActive], tempEntry - GET_LASTTIME() );
  jActive=jStack[jStackPtr++];

  uint32_t tempExit = ProfilerGetMicrosecond();
  //assert_check(tempExit, tempEntry);
  Increment(&ProfilerTimes[JOB_TASK_PROFILER], tempExit-tempEntry );
  SET_LASTTIME(tempExit);
  __enable_irq();
}

static char buffer[25];
void ProfilerDumpTime(uint64_t time64, char *text)
{
  /* convert to ms */
  time64  /= 1000;
  uint32_t ms    = time64 % 1000;
  /* convert to second, uint32_t is sufficient */
  uint32_t temp32 = time64 / 1000; 
  uint32_t secs = temp32 % 60;
  temp32 /= 60;
  uint32_t mins = temp32 % 60;
  temp32 /= 60;
  /* Temp contains hrs now */
  uint32_t hrs = temp32 % 24;
  temp32 /= 24;
  /* temp32 now contains days */
  sprintf(buffer, "%3dd %02d:%02d:%02d.%03ds", temp32, hrs, mins, secs,ms);
  DBG_dump_textvalue(text, buffer );
}

/*********************************************************************************
  * @brief  Increment the time, the processor spent in Stop mode. This can't be 
  *         done by normal profiling functions, because the microsecond timer is
  *         suspended while in Stop2 mode. 
  *         The stop time can be evaluated by checking a low power timer, eg
  *
  * @param  stop_us - number of microsecs to add to Stop2 time account
  *         
  * @retval None
  ********************************************************************************/
void ProfilerIncrementStopTime( uint32_t stop_us, uint32_t StopMode )
{
  Increment(&ProfilerTimes[JOB_STOP0+StopMode], stop_us );
}


bool ProfilerDump(char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  uint64_t time;
  uint64_t sum=0;
  char *text;
  DBG_printf_indent("Profiling Data\n" );
  DBG_setIndentRel(+2);
  DBG_setPadLen(18);
  for ( ActiveJobEnumType i=0; i < JOB_NROF_ELEMENTS; i++ ) {
     time = ProfilerTimes[i];
     text = jobnames[i];
     sum += time;
     ProfilerDumpTime(time, text);
  }
  ProfilerDumpTime(sum, "*** total ***");
  DBG_dump_number("Max. stack depth", STACK_DEPTH-jStackMin);
  DBG_setIndentRel(-2);

  return true;
}

/**
  * @}
  */

#endif // DEBUG_PROFILING > 0