/*******************************************************************************
 * @file    hardware.c
 * @author  rainer
 * @brief   implement all hardware specfic things
 *
 *          STM32H7xx
 *
 * This file has to be adopted for different STM32 hardwares. And it should be the
 * only place where hardware adoption takes place
 ******************************************************************************/

#include "config/config.h"
#include "hardware.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif


/*
 * Assignment of Timers to busses / clock sources 
 */
const TIM_TypeDef* apb1_timers[9]={TIM2, TIM3, TIM4, TIM5,  TIM6,  TIM7, TIM12, TIM13, TIM14  };   /* Timers clocked by APB1 */
const TIM_TypeDef* apb2_timers[5]={TIM1, TIM8,       TIM15, TIM16, TIM17 };                        /* Timers clocked by APB2 */

/****************************************************************************** 
 * @brief  Return the APB1-Timers input frequency. If APB1 clock is prescaled 
           from             HCLK, the timer input frequency is twice the 
           APB1-Clock. If APB1 clock is equal HCLK, timer input frequency is 
           equal to APB1 clock
 * @param  None
 * @retval Actual APB1 timers input frequency
 *****************************************************************************/
uint32_t GetAPB1TimerFrequency(void)
{
  uint32_t uPclk1 =  HAL_RCC_GetPCLK1Freq();
  uint32_t uPclk1Prescale = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk ) >> RCC_D2CFGR_D2PPRE1_Pos;
  DEBUG_PRINTF("APB1 clock .........=%d\n", uPclk1);
  DEBUG_PRINTF("APB1 clock prescaler=%d\n", uPclk1Prescale);
  if (uPclk1Prescale==RCC_HCLK_DIV1)
     return uPclk1;
  else 
     return uPclk1*2;
}

uint32_t GetAPB2TimerFrequency(void)
{
  uint32_t uPclk2 =  HAL_RCC_GetPCLK2Freq();
  uint32_t uPclk2Prescale = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2_Msk ) >> RCC_D2CFGR_D2PPRE2_Pos;
  DEBUG_PRINTF("APB2 clock .........=%d\n", uPclk2);
  DEBUG_PRINTF("APB2 clock prescaler=%d\n", uPclk2Prescale);
  if (uPclk2Prescale==RCC_HCLK_DIV1)
     return uPclk2;
  else 
     return uPclk2*2;
}
/******************************************************************************
 * Get the STM32H745 peripheral clock frequency. This can be either HSI, CSI or 
 * HSE clock frequency, depending from select bits in RCC->D1CCIPR
 *****************************************************************************/
uint32_t GetPerClkFrequency(void)     
{
    uint32_t bits = (RCC->D1CCIPR & RCC_D1CCIPR_CKPERSEL_Msk) >> RCC_D1CCIPR_CKPERSEL_Pos;
    switch( bits ) {
        case 0b00: return HSI_VALUE;
        case 0b01: return CSI_VALUE;
        case 0b10: return HSE_VALUE;
        default:
            DEBUG_PUTS("Error: GetPerClkFrequency - Illegal clk source" );
    }
    return 0;
}


void TimerWatchdogReset_Internal(uint16_t num_of_ms, IWDG_TypeDef *myWD);

/******************************************************************************
 * @brief  Perform a system reset by watchdog reset
 * @param  ms - ms to delay before reset
 * @note  Implementation is hardware dependant
 * @note  for STM32H745  reset both CPUs via watchdog
 *         
 * @retval None
 *****************************************************************************/
void TimerWatchdogReset(uint16_t num_of_ms)
{
   TimerWatchdogReset_Internal(num_of_ms, IWDG1);
#if defined(IWDG2)
   TimerWatchdogReset_Internal(num_of_ms, IWDG2);
#endif
}

