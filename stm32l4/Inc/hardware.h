
/*******************************************************************************
 * @file    hardware.h
 * @author  rainer
 * @brief   include all hardware specfic files
 *
 * This file has to be adopted for different STM32 hardwares. And it should be the
 * only place where hardware adoption takes place
 ******************************************************************************/

#ifndef __HARDWARE_H__
#define  __HARDWARE_H__
/*
 * customization for STM32L4xx
 */
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#define DMA_IS_LINEAR(hdma)             HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC)
#define DMA_GET_RXSIZE(hdma)            (hdma->Instance->CNDTR)

/*
 * There are no special considerations for DMA or BDMA memory on L4-family, because
 * they don't have D-cache or an MPU
 */
#define DMAMEM  
#define BDMAMEM                

/* --- Type specific setup for ADC ------------------------------------------*/
#define ADC_HAS_REFINT(inst)            ( inst == (void *)ADC1_BASE )
#if defined(STM32L476xx) || defined(STM32L496xx)
    #define ADC_HAS_CHIPTEMP(inst)          ( inst == (void *)ADC1_BASE || inst == (void *)ADC3_BASE )
#elif defined(STM32L4Pxxx) || defined(STM32L4Sxxx) || defined(STM32L43xx)
    #define ADC_HAS_CHIPTEMP(inst)          ( inst == (void *)ADC1_BASE )
#else
    #error "No Definition for ADC_HAS_CHIPTEMP"
#endif

/* --- Type specific setup for Timers ---------------------------------------*/
extern const TIM_TypeDef* apb1_timers[];    /* Timers connected to APB1 */
extern const TIM_TypeDef* apb2_timers[];    /* Timers connected to APB2 */
extern const TIM_TypeDef* bit32_timers[];   /* list of 32bit timers     */

uint32_t GetAPB1TimerFrequency  (void);     /* Get APB1 Timer input frq */
uint32_t GetAPB2TimerFrequency  (void);     /* Get APB2 Timer input frq */
void     TimerWatchdogReset     (uint16_t waitms);        /* Force reset by watchdog timeout             */

#endif /* __HARDWARE_H__ */