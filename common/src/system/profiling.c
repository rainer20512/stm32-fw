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
#include <stdio.h>
/*******************************************************************************
 * @name    ProfilerFormatTime
 *
 * @brief   Format a time value to a fixed time format within a char array 
 * @param   time64 - time in milliseconds or microseconds, see "bAreUs"
 *          buffer - buffer to write to
            buflen - available chars in buffer
            bAreUs - true, if time unit are microseconds, false for milli s.
 * @note    returned time str is always 17 chars  long
 ******************************************************************************/
void ProfilerFormatTime(uint64_t time64, char *buffer, size_t buflen, bool bAreUs)
{
  /* convert to ms */
  if ( bAreUs) time64  /= 1000;

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
  snprintf(buffer, buflen, "%3dd %02d:%02d:%02d.%03ds", temp32, hrs, mins, secs,ms);
}

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

#include "hardware.h"


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

  char buffer[25];
  char *text;

  uint64_t time;
  uint64_t sum=0;

  DBG_printf_indent("Profiling Data\n" );
  DBG_setIndentRel(+2);
  DBG_setPadLen(18);
  for ( ActiveJobEnumType i=0; i < JOB_NROF_ELEMENTS; i++ ) {
     time = ProfilerTimes[i];
     text = jobnames[i];
     sum += time;
     ProfilerFormatTime(time, buffer, 25, true);
     DBG_dump_textvalue(text, buffer);

  }
  ProfilerFormatTime(sum, buffer, 25, true );
  DBG_dump_textvalue("*** total ***",buffer );
  DBG_dump_number("Max. stack depth", STACK_DEPTH-jStackMin);
  DBG_setIndentRel(-2);

  return true;
}

/**
  * @}
  */

#endif // DEBUG_PROFILING > 0