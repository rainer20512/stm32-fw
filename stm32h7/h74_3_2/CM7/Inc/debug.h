/**
  ******************************************************************************
  * @file    debug.h 
  * @author  Rainer
  * @brief   Debug settings and options
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * Define different levels of logging detail depth 
 * the higher the loglevel, the more debug/log info is generated/displayed
 */

#define LOGLEVEL_NOTHING   0                    /* Don't debug/log anything at all   */
#define LOGLEVEL_ALWAYS    1                    /* Log always, independent from console_debuglevel       */
#define LOGLEVEL_FATAL     2                    /* debug/log only severe errors      */
#define LOGLEVEL_ERROR     3                    /* debug/log all kinds errors        */
#define LOGLEVEL_WARN      4                    /* debug/log also warnings           */ 
#define LOGLEVEL_INFO      5                    /* debug/log all kind of useful info */
#define LOGLEVEL_VERBOSE   6                    /* debug/log even more               */

#if defined(DEBUG)
    #define STATIC_LOGLIMIT     LOGLEVEL_VERBOSE    /* max. loglevel that will be generated in code */
    #define DEBUG_MODE          1
#else
    #define STATIC_LOGLIMIT     LOGLEVEL_ALWAYS 
    #define DEBUG_MODE          0
#endif

/* Enable/Disable different logging destinations */
#define LOGTO_FATFS         0  
#define LOGTO_CONSOLE       1   


/* Enable/Disable different debug packets */
#define DEBUG_STARTUP       0
#define DEBUG_FEATURES      1
#define DEBUG_DEBUGIO       0
#define DEBUG_DUMP_RFM      0
#define DEBUG_DUMP_KEYS     0
#define DEBUG_RFM_STATUS    1
#define DEBUG_RFM_HARDCORE  0
// DEBUG_TIMER in timer.c
// DEBUG_RTC in rtc.c
// DEBUG_QENC in qencoder.h
// DEBUG_ADC in adc_dev.c
// DEBUG_CAN in can_dev.c
// DEBUG_FMC in fmc_dev.c
// DEBUG_SDMMC in sdmmc_dev.c
#define DEBUG_QSPI          1  
#define DEBUG_PROFILING     1
#define DEBUG_PERIODIC      1
#define DEBUG_SLEEP_STOP    0
#define DEBUG_EPAPER        0
#define DEBUG_LCD           0
#define DEBUG_LCD_MENU      0
#define DEBUG_ONEWIRE       0
#define DEBUG_DS18X20       0
#define DEBUG_BMP085        0
#define DEBUG_EEPROM_EMUL   0
#define CMACDEBUG           0
#define DEBUG_PULSES        0
#define DEBUG_USB           1

#define DEBUG_PRINT_ADDITIONAL_TIMESTAMPS 0

#if DEBUG_MODE 
  #include <stdint.h>
  extern uint32_t console_debuglevel;
  extern uint32_t fatfs_debuglevel;
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_H */

