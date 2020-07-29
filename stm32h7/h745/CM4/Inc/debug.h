
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

#define DEBUG_MODE          1

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
#define DEBUG_PROFILING     0
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

#define DEBUG_PRINT_ADDITIONAL_TIMESTAMPS 0

#if DEBUG_MODE 
  #include <stdint.h>
  extern uint32_t debuglevel;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_H */

