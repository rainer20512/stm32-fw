/**
 ******************************************************************************
 * @file    exti_handler.c
 * @author  Rainer
 * @brief   Do all the handling of EXTI pin change interrupts. 
 ******************************************************************************
 */

/*Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTI_HANDLER_H
#define __EXTI_HANDLER_H

#include "stm32l4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define EXTI_LINENUM    16      
typedef void ( *ExtIrqCB )( uint16_t pinnumber, uint16_t pinvalue, void *arg );

bool     Exti_Register_Callback      ( uint16_t GPIO_Pin, __IO uint32_t *gpioidr, ExtIrqCB cb, void *arg );
void     Exti_UnRegister_Callback    ( uint16_t GPIO_Pin );
bool     Exti_Has_Callback           ( uint16_t line );
int16_t  ExtiGetIrqNumFromPin        ( uint16_t pin );
void     Exti_ConfigIrq              ( GPIO_TypeDef *gpio, uint16_t pin, uint32_t extiTrigger );
uint16_t Exti_DisableIrq             ( uint16_t pin );
void     Exti_EnableIrq              ( uint16_t pin );
     
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __EXTI_HANDLER_H