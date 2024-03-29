/**
  ******************************************************************************
  * @file    system/profiling.h
  * @author  Rainer
  * @brief   Functions for a basic profiling support on the basis of a �s-Timer
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PROFILING_H
#define __PROFILING_H

#include "config/config.h"


#ifdef __cplusplus
 extern "C" {
#endif

typedef enum ActiveJobEnum {
    JOB_TASK_TMR = 0,
    JOB_TASK_PERIODIC,
    JOB_TASK_RFM,
    JOB_TASK_DBGIO,
    JOB_TASK_RTC,
    JOB_TASK_INIT,
    JOB_TASK_MAIN,
#if USE_DISPLAY > 0
    JOB_TASK_LCD,
#endif
#if USE_EPAPER > 0
    JOB_TASK_EPD,
#endif
#if USE_ONEWIRE > 0
    JOB_TASK_ONEWIRE,
#endif
#if USE_QSPI > 0 || USE_OSPI  >0
    JOB_TASK_XSPI,
#endif
#if USE_USB > 0
    JOB_TASK_USBD,
#endif
    JOB_ADC,
    JOB_SLEEP,
    JOB_STOP0,
    JOB_STOP1,
    JOB_STOP2,
    JOB_IRQ_UART,
    JOB_IRQ_SPI,
    JOB_IRQ_I2C,
    JOB_IRQ_RTC,
    JOB_IRQ_TMR,
    JOB_IRQ_EXTI,
    JOB_IRQ_DMA,
    JOB_IRQ_PROFILER,
    JOB_TASK_PROFILER,

    JOB_NROF_ELEMENTS                        /* Keep this element as last entry in enum ! */
} ActiveJobEnumType;

#if DEBUG_PROFILING > 0

#define JOBNAMES0   \
    "TMR",          \
    "PERIODIC",     \
    "RFM",          \
    "DBGIO",        \
    "RTC",          \
    "INIT",         \
    "MAIN",       
#if USE_DISPLAY > 0 
    #define JOBNAMES1 JOBNAMES0 \
    "LCD",  
#else
    #define JOBNAMES1 JOBNAMES0
#endif              
#if USE_EPAPER > 0  
    #define JOBNAMES2 JOBNAMES1 \
    "EPD",           
#else
    #define JOBNAMES2 JOBNAMES1
#endif              
#if USE_ONEWIRE > 0 
    #define JOBNAMES3 JOBNAMES2 \
    "ONEWIRE",
#else
    #define JOBNAMES3 JOBNAMES2
#endif             
#if USE_QSPI > 0 || USE_OSPI  >0
    #define JOBNAMES4 JOBNAMES3 \
    "QUAD/OCTOSPI",
#else
    #define JOBNAMES4 JOBNAMES3
#endif             
#if USE_USB > 0
    #define JOBNAMES5 JOBNAMES4 \
    "USB-DEV",
#else
    #define JOBNAMES5 JOBNAMES4
#endif             

#define JOBNAMES99 \
    JOBNAMES5     \
    "ADC",        \
    "SLEEP",      \
    "STOP0",      \
    "STOP1",      \
    "STOP2",      \
    "IRQ_UART",   \
    "IRQ_SPI",    \
    "IRQ_I2C",    \
    "IRQ_RTC",    \
    "IRQ_TMR",    \
    "IRQ_EXTI",   \
    "IRQ_DMA",    \
    "IRQ_PROFILER",\
    "PROFILER",

#include "dev/timer_dev.h"

/* Define the timer that will be used as microsecond timer */
/* Change all defines, if counter is changed !             */
#define MICROCOUNTER_TMR              TIM7
#define MICROCOUNTER_TMR_CLK_ENABLE   __HAL_RCC_TIM7_CLK_ENABLE
#define MICROCOUNTER_TMR_IRQn         TIM7_IRQn
#define MICROCOUNTER_TMR_IRQHandler   TIM7_IRQHandler            
// #define MICROCOUNTER_TMR_IRQ_VECTOR   TIM6_DAC_IRQHandler            ; TIM6 and DAC


  /* Exported variables ------------------------------------------------------------------*/
  extern volatile uint32_t ProfilerMicroCountHigh;  /* The upper 16 bit of the microsecond-counter */
  extern uint64_t ProfilerTimes[];
   
  /* Exported functions ------------------------------------------------------------------*/
  /* Get the system uptime in mikroseconds */
  #define ProfilerGetMicrosecond()  BASTMR_GetMicrosecond(&BASTIM_HANDLE) 
  #if DEBUG_FEATURES > 0
     char *ProfilerGetTS(void);
  #endif
  void     ProfilerInitTo(ActiveJobEnumType active );
  void     ProfilerSwitchTo(ActiveJobEnumType active );
  void     ProfilerPush(ActiveJobEnumType active );
  void     ProfilerPop(void);
  void     ProfilerIncrementStopTime( uint32_t stop_us, uint32_t StopMode );
  bool     ProfilerDump(char *cmdline, size_t len, const void * arg );
  void     ProfilerDumpTime(uint64_t time64, char *text);
  void     ProfilerFormatTime(uint64_t time64, char *buffer, size_t buflen, bool bAreUs);
#else
  #define ProfilerSetupMicroCounter()
  #define ProfilerGetMicrosecond()
  #define ProfilerInitTo(a)
  #define ProfilerSwitchTo(a)
  #define ProfilerPush(a)
  #define ProfilerPop()
  #define ProfilerDump()
  #define ProfilerDumpTime(a,b);
#endif // DEBUG_PROFILING > 0

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PROFILING_H */
