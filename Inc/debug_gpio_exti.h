/**
  ******************************************************************************
  * @file    debug_gpio_exti.h
  * @author  Rainer
  * @brief   functions for debugging/dumping GPIO and EXTI Settings
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_GPIO_EXTI_H
#define __DEBUG_GPIO_EXTI_H

#include "config/config.h"
#include "debug.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if DEBUG_FEATURES > 0
  void DBG_dump_gpio_status(char gpio_letter );
  void DBG_dump_toggle_pin(char portletter, uint8_t portnum, bool bToggleOnce);
  void DBG_dump_exti_config(void);
  void DBG_dump_nvic_config(void);
  void DBG_init_pin(char portletter, uint8_t portnum, uint32_t speed, uint32_t pupd_status, uint32_t init);
  void DBG_deinit_pin(char portletter, uint8_t portnum);

#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_GPIO_EXTI_H */
