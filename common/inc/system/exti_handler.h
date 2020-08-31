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

#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define EXTI_LINENUM    16      

/* Some macros to handle exti interrupts correctly in dual core environment */
#if defined(DUAL_CORE) && defined(CORE_CM4)
    #define EXTI_CLEAR_PR1(bitpos)      EXTI->C2PR1 = bitpos
    #define EXTI_GET_PR1()              EXTI->C2PR1
    #define EXTI_IRQ_ENABLED(bitpos)    READ_BIT(EXTI->C2IMR1,   (uint32_t)(bitpos) )
    #define EXTI_ENABLE_IRQ(bitpos)     SET_BIT(EXTI->C2IMR1,   (uint32_t)(bitpos) )
    #define EXTI_DISABLE_IRQ(bitpos)    CLEAR_BIT(EXTI->C2IMR1, (uint32_t)(bitpos) )
    #define EXTI_GET_IMR1()             EXTI->C2IMR1
#else
    #define EXTI_CLEAR_PR1(bitpos)      EXTI->PR1 = bitpos
    #define EXTI_GET_PR1()              EXTI->PR1
    #define EXTI_IRQ_ENABLED(bitpos)    READ_BIT(EXTI->IMR1,   (uint32_t)(bitpos) )
    #define EXTI_ENABLE_IRQ(bitpos)     SET_BIT(EXTI->IMR1,   (uint32_t)(bitpos) )
    #define EXTI_DISABLE_IRQ(bitpos)    CLEAR_BIT(EXTI->IMR1, (uint32_t)(bitpos) )
    #define EXTI_GET_IMR1()             EXTI->IMR1
#endif

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