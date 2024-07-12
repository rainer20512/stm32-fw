/**
  ******************************************************************************
  * @file    tmr_config.h
  * @author  Rainer
  * @brief   configuration of Timers
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TMR_CONFIG_H
#define __TMR_CONFIG_H

#include "config/devices_config.h"
#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************************/
/* TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIM*/
/* TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIM*/
/* TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIM*/
/* TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIM*/
/******************************************************************************************/

/* ----- Fixed part, don't change -------------------------------------------------- */
#if defined(USE_TIM1) && defined(TIM1)
    #if defined(USE_TIM1_ALTN1)
        /* CH1..CH4 : PA8..PA11  */
        #define TIM1_CH1                         { GPIO_PIN_8,  GPIOA, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch1" }
        #define TIM1_CH2                         { GPIO_PIN_9,  GPIOA, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch2" }
        #define TIM1_CH3                         { GPIO_PIN_10, GPIOA, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch3" }
        #define TIM1_CH4                         { GPIO_PIN_11, GPIOA, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch4" }
    #elif defined(USE_TIM1_ALTN2)
        /* CH1..CH4 : PE9, PE11, PE13, PE14 */
        #define TIM1_CH1                         { GPIO_PIN_9,  GPIOE, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch1" }
        #define TIM1_CH2                         { GPIO_PIN_11, GPIOE, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch2" }
        #define TIM1_CH3                         { GPIO_PIN_13, GPIOE, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch3" }
        #define TIM1_CH4                         { GPIO_PIN_14, GPIOE, GPIO_AF1_TIM1, GPIO_PULLDOWN, "Tim1_Ch4" }
    #else
        #error("No config for TIM1 found");
    #endif 
#endif /* TIM1 */

#if defined(USE_TIM2) && defined(TIM2)
    #if defined(USE_TIM2_ALTN1)
        /* CH1..CH4 : PA0..PA3  */
        #define TIM2_CH1                         { GPIO_PIN_0,  GPIOA, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch1" }
        #define TIM2_CH2                         { GPIO_PIN_1,  GPIOA, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch2" }
        #define TIM2_CH3                         { GPIO_PIN_2,  GPIOA, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch3" }
        #define TIM2_CH4                         { GPIO_PIN_3,  GPIOA, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch4" }
    #elif defined(USE_TIM2_ALTN2)
        /* CH1..CH4 : PA15, PB3, PB10, PB11 */
        #define TIM2_CH1                         { GPIO_PIN_15,  GPIOA, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch1" }
        #define TIM2_CH2                         { GPIO_PIN_3,   GPIOB, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch2" }
        #define TIM2_CH3                         { GPIO_PIN_10,  GPIOB, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch3" }
        #define TIM2_CH4                         { GPIO_PIN_11,  GPIOB, GPIO_AF1_TIM2, GPIO_PULLDOWN, "Tim2_Ch4" }
    #else
        #error("No config for TIM2 found");
    #endif 
#endif /* TIM2 */

#if defined(USE_TIM3) && defined(TIM3)
    #if defined(USE_TIM3_ALTN1)
        /* CH1..CH4 : PA6, PA7, PB0, PB1 */
        #define TIM3_CH1                         { GPIO_PIN_6,  GPIOA, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch1" }
        #define TIM3_CH2                         { GPIO_PIN_7,  GPIOA, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch2" }
        #define TIM3_CH3                         { GPIO_PIN_0,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch3" }
        #define TIM3_CH4                         { GPIO_PIN_1,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch4" }
    #elif defined(USE_TIM3_ALTN2)
        /* CH1..CH4 : PC6..PC9 */
        #define TIM3_CH1                         { GPIO_PIN_6,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch1" }
        #define TIM3_CH2                         { GPIO_PIN_7,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch2" }
        #define TIM3_CH3                         { GPIO_PIN_8,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch3" }
        #define TIM3_CH4                         { GPIO_PIN_9,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch4" }
    #elif defined(USE_TIM3_ALTN3)
        /* CH1..CH4 : PB4, PB5, PB0, PB1 */
        #define TIM3_CH1                         { GPIO_PIN_4,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch1" }
        #define TIM3_CH2                         { GPIO_PIN_5,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch2" }
        #define TIM3_CH3                         { GPIO_PIN_0,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch3" }
        #define TIM3_CH4                         { GPIO_PIN_1,  GPIOB, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch4" }

    #elif defined(USE_TIM3_ALTN4)
        /* CH1..CH4 : PC6, PC9 */
        #define TIM3_CH1                         { GPIO_PIN_6,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch1" }
        #define TIM3_CH2                         { GPIO_PIN_7,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch2" }
        #define TIM3_CH3                         { GPIO_PIN_8,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch3" }
        #define TIM3_CH4                         { GPIO_PIN_9,  GPIOC, GPIO_AF2_TIM3, GPIO_PULLDOWN, "Tim3_Ch4" }    #else
        #error("No config for TIM3 found");
    #endif 
#endif /* TIM3 */

#if defined(USE_TIM4) && defined(TIM4)
    #if defined(USE_TIM4_ALTN1)
        /* CH1..CH4 : PB6..PB9 */
        #define TIM4_CH1                         { GPIO_PIN_6,  GPIOB, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch1" }
        #define TIM4_CH2                         { GPIO_PIN_7,  GPIOB, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch2" }
        #define TIM4_CH3                         { GPIO_PIN_8,  GPIOB, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch3" }
        #define TIM4_CH4                         { GPIO_PIN_9,  GPIOB, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch4" }
    #elif defined(USE_TIM4_ALTN2)
        /* CH1..CH4 : PD12..PD15 */
        #define TIM4_CH1                         { GPIO_PIN_12,  GPIOD, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch1" }
        #define TIM4_CH2                         { GPIO_PIN_13,  GPIOD, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch2" }
        #define TIM4_CH3                         { GPIO_PIN_14,  GPIOD, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch3" }
        #define TIM4_CH4                         { GPIO_PIN_15,  GPIOD, GPIO_AF2_TIM4, GPIO_PULLDOWN, "Tim4_Ch4" }
    #else
        #error("No config for TIM4 found");
    #endif 
#endif /* TIM4 */

#if defined(USE_TIM5) && defined(TIM5)
    #if defined(USE_TIM5_ALTN1)
        /* CH1..CH4 : PA0..PA3  */
        #define TIM5_CH1                         { GPIO_PIN_0,  GPIOA, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch1" }
        #define TIM5_CH2                         { GPIO_PIN_1,  GPIOA, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch2" }
        #define TIM5_CH3                         { GPIO_PIN_2,  GPIOA, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch3" }
        #define TIM5_CH4                         { GPIO_PIN_3,  GPIOA, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch4" }
    #elif defined(USE_TIM5_ALTN2)
        /* CH1..CH4 : PF6..PF9 */
        #define TIM5_CH1                         { GPIO_PIN_6,  GPIOF, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch1" }
        #define TIM5_CH2                         { GPIO_PIN_7,  GPIOF, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch2" }
        #define TIM5_CH3                         { GPIO_PIN_8,  GPIOF, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch3" }
        #define TIM5_CH4                         { GPIO_PIN_9,  GPIOF, GPIO_AF2_TIM5, GPIO_PULLDOWN, "Tim5_Ch4" }
    #else
        #error("No config for TIM5 found");
    #endif 
#endif /* TIM5 */

#if defined(USE_TIM15) && defined(TIM15)
    #if defined(USE_TIM15_ALTN1)
        /* CH1,CH2,Ch1N : PA2,PA3, PA1  */
        #define TIM15_CH1                         { GPIO_PIN_2,  GPIOA, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1" }
        #define TIM15_CH2                         { GPIO_PIN_3,  GPIOA, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch2" }
        #define TIM15_CH1N                        { GPIO_PIN_1,  GPIOA, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1N" }
    #elif defined(USE_TIM15_ALTN2)
        /* CH1,CH2,Ch1N : PB14,PB15, PB13  */
        #define TIM15_CH1                         { GPIO_PIN_14, GPIOB, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1" }
        #define TIM15_CH2                         { GPIO_PIN_15, GPIOB, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch2" }
        #define TIM15_CH1N                        { GPIO_PIN_13, GPIOB, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1N" }
    #elif defined(USE_TIM15_ALTN3)
        /* CH1,CH2,Ch1N : PF9, PF10, PG9  */
        #define TIM15_CH1                         { GPIO_PIN_9,  GPIOF, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1" }
        #define TIM15_CH2                         { GPIO_PIN_10, GPIOF, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch2" }
        #define TIM15_CH1N                        { GPIO_PIN_9,  GPIOG, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1N" }
    #elif defined(USE_TIM15_ALTN3)
        /* CH1,CH2,Ch1N : PG10,PG11, PG9  */
        #define TIM15_CH1                         { GPIO_PIN_10, GPIOG, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1" }
        #define TIM15_CH2                         { GPIO_PIN_11, GPIOG, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch2" }
        #define TIM15_CH1N                        { GPIO_PIN_9,  GPIOG, GPIO_AF14_TIM15, GPIO_PULLDOWN, "Tim15_Ch1N" }
    #else
        #error("No config for TIM15 found");
    #endif 
#endif /* TIM15 */

/* RHB todo TIM8, TIM16, TIM17 */

/******************************************************************************************/
/* BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER*/
/* BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER*/
/* BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER*/
/* BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER BASE TIMER*/
/******************************************************************************************/
#if defined(USE_TIM7) && defined(TIM7)
    #define TIM7_IRQ                            { TIM7_IRQn, BASETIM_IRQ_PRIO, 0 }
    #define TIM7_IRQHandler                     TIM7_IRQHandler 
#endif /* TIM7 */

#if defined(USE_TIM6) && defined(TIM6)
    #define TIM6_IRQ                            { TIM6_IRQn, BASETIM_IRQ_PRIO, 0 }
    #define TIM6_IRQHandler                     TIM6_DAC_IRQHandler 
#endif /* TIM7 */

/******************************************************************************************/
/* Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer   */
/* Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer   */
/* Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer   */
/* Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer Peripheral Timer   */
/******************************************************************************************/
#if defined(USE_TIM2) && defined(TIM2)
    #define TIM2_IRQ                            { TIM2_IRQn, BASETIM_IRQ_PRIO, 0 }
    #define TIM2_IRQHandler                     TIM2_IRQHandler 
#endif /* TIM7 */

/* ----- End of Fixed part, change below  -------------------------------------------- */

/* ----- Configurable part, change below  -------------------------------------------- */



#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __TMR_CONFIG_H */