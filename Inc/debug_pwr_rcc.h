/**
  ******************************************************************************
  * @file    debug_util.h
  * @author  Rainer
  * @brief   Miscellaneous functions for debugging
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_PWR_RCC_H
#define __DEBUG_PWR_RCC_H

#include "config/config.h"
#include "debug.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if DEBUG_FEATURES > 0
  void DBG_dump_clocksetting(void);
  void DBG_dump_powersetting(void);
  void DBG_dump_peripheralclocksetting(void);
  void DBG_dump_peripheralclocksetting_insleepmode(void);
  void DBG_dump_peripheralclockconfig(void);
  void DBG_dump_rtcclockconfig(void);
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_PWR_RCC_H */
