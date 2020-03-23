/**
  ******************************************************************************
  * @file    debug_sram.h
  * @author  Rainer
  * @brief   Miscellaneous functions for displaying SRAM Info
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_SRAM_H
#define __DEBUG_SRAM_H

#include "config/config.h"
#include "debug.h"

#include "dev/uart_dev.h"

#ifdef DEBUG_DEBUGIO
  #include <debugio.h>
#endif

#include <stdio.h>
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

void DBG_sram(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_SRAM_H */
