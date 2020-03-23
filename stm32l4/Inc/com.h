/**
  ******************************************************************************
  * @file    com.h 
  * @author  Rainer
  * @brief   Debug Input handling and command interpreter 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_H
#define __COM_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

void COM_wireless_command_parse (uint8_t * buf, uint8_t bufen);
void COM_print_debug            (bool rfm_transmit);
void COM_print_version          (bool rfm_transmit);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __COM_H */
