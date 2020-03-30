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

#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif

void DBG_sram(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_SRAM_H */
