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
const TIM_TypeDef* apb1_timers[9+1]={TIM2, TIM3, TIM4, TIM5,  TIM6,  TIM7, TIM12, TIM13, TIM14, NULL  };   /* Timers clocked by APB1 */
const TIM_TypeDef* apb2_timers[5+1]={TIM1, TIM8,       TIM15, TIM16, TIM17, NULL };                        /* Timers clocked by APB2 */


/****************************************************************************** 
 * @brief  Return the APB1 timers and APB2 timers clock domain prescalers
           APB1 and APB2 timer clocks are derived/prescaled from HCLK 
           - If the prescaler is 1 ( ie APB clock = HCLK ), the timer input 
             frequency is equal to APB1 clock
           - If the prescaler is > 1, the timer input clock is TWICE the
             APB clock
 * @param  None
 * @retval Actual APB1/APB2 timers prescaler value
 *****************************************************************************/
uint32_t GetAPB1TimerPrescaler(void)
{
  uint32_t bits = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk ) >> RCC_D2CFGR_D2PPRE1_Pos;
  if ( (bits & 0b100 ) == 0 )
     return 1;
  else 
     return ( 2 << ( bits & 0b011 ) ) / 2;
}

uint32_t GetAPB2TimerPrescaler(void)
{
  uint32_t bits = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2_Msk ) >> RCC_D2CFGR_D2PPRE2_Pos;
  if ( (bits & 0b100 ) == 0 )
     return 1;
  else 
     return ( 2 << ( bits & 0b011 ) ) / 2;
}

/****************************************************************************** 
 * @brief  Return the AHB clock domain prescaler.
           from             HCLK, the timer input frequency is twice the 
           APB1-Clock. If APB1 clock is equal HCLK, timer input frequency is 
           equal to APB1 clock
 * @param  None
 * @retval Actual APB1 timers input frequency
 *****************************************************************************/
uint32_t GetAHBPrescaler (void)
{
  uint32_t bits =  ( RCC->D1CFGR & RCC_D1CFGR_HPRE_Msk ) >> RCC_D1CFGR_HPRE_Pos;
  if ( ( bits & 0b1000 ) == 0 ) 
    return 1;
  else 
    return 2 << ( bits & 0b0111 );
}

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
  uint32_t uPclk1         =  HAL_RCC_GetPCLK1Freq();
  uint32_t uPclk1Prescale = GetAPB1TimerPrescaler();
  //DEBUG_PRINTF("APB1 clock .........=%d\n", uPclk1);
  //DEBUG_PRINTF("APB1 clock prescaler=%d\n", uPclk1Prescale);
  return uPclk1/uPclk1Prescale;
}

uint32_t GetAPB2TimerFrequency(void)
{
  uint32_t uPclk2         =  HAL_RCC_GetPCLK2Freq();
  uint32_t uPclk2Prescale = GetAPB2TimerPrescaler();
  //DEBUG_PRINTF("APB2 clock .........=%d\n", uPclk2);
  //DEBUG_PRINTF("APB2 clock prescaler=%d\n", uPclk2Prescale);
  return uPclk2/uPclk2Prescale;
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

