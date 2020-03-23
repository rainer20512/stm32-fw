/**
 ******************************************************************************
 * @file    timer_handler.c
 * @author  Rainer
 * @brief   Implements a list of TimerCapturehandlers 
 ******************************************************************************
 */

/*Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_HANDLER_H
#define __TIMER_HANDLER_H

#include "stm32l4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef void ( *TimCaptureCB )( TIM_HandleTypeDef *htim );

bool    Tim_Register_CaptureCB      ( void *hwBase, TimCaptureCB cb );
void    Tim_UnRegister_CaptureCB    ( void *hwBase );
bool    Tim_Has_CaptureCB           ( void *hwBase );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __TIMER_HANDLER_H