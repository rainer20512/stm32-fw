/*******************************************************************************
 * @file    hardware.c
 * @author  rainer
 * @brief   implement all hardware specfic things for
 *
 *          STM32L4xx
 *
 * This file has to be adopted for different STM32 hardwares. And it should be the
 * only place where hardware adoption takes place
 ******************************************************************************/

#include "config/config.h"
#include "hardware.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#if defined(STM32L43xx)
    const TIM_TypeDef* apb1_timers[]  ={TIM2, TIM6,  TIM7, NULL  };   /* Timers clocked by APB1 */
    const TIM_TypeDef* apb2_timers[]  ={TIM1, TIM15, TIM16, NULL };   /* Timers clocked by APB2 */
#elif defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4PLUS_FAMILY)
    const TIM_TypeDef* apb1_timers[]  ={TIM2, TIM3, TIM4, TIM5,  TIM6,  TIM7,  NULL };   /* Timers clocked by APB1 */
    const TIM_TypeDef* apb2_timers[]  ={TIM1, TIM8,       TIM15, TIM16, TIM17, NULL };   /* Timers clocked by APB2 */
#else
    #error "No timer-to-bus-assignment for selected HW type"
#endif

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
  uint32_t bits = (RCC->CFGR & RCC_CFGR_PPRE1_Msk ) >> RCC_CFGR_PPRE1_Pos;
  if ( (bits & 0b100 ) == 0 )
     return 1;
  else 
     return ( 2 << ( bits & 0b011 ) ) / 2;
}

uint32_t GetAPB2TimerPrescaler(void)
{
  uint32_t bits = (RCC->CFGR & RCC_CFGR_PPRE2_Msk ) >> RCC_CFGR_PPRE2_Pos;
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
  uint32_t bits =  (RCC->CFGR & RCC_CFGR_HPRE_Msk ) >> RCC_CFGR_HPRE_Pos;
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
  uint32_t uPclk1 =  HAL_RCC_GetPCLK1Freq();
  uint32_t uPclk1Prescale = GetAPB1TimerPrescaler();
  /*
  DEBUG_PRINTF("APB1 clock .........=%d\n", uPclk1);
  DEBUG_PRINTF("APB1 clock prescaler=%d\n", uPclk1Prescale);
  */
  return uPclk1/uPclk1Prescale;
}

uint32_t GetAPB2TimerFrequency(void)
{
  uint32_t uPclk2 =  HAL_RCC_GetPCLK2Freq();
  uint32_t uPclk2Prescale = GetAPB2TimerPrescaler();
  /*
  DEBUG_PRINTF("APB2 clock .........=%d\n", uPclk2);
  DEBUG_PRINTF("APB2 clock prescaler=%d\n", uPclk2Prescale);
  */
  return uPclk2/uPclk2Prescale;
}

void TimerWatchdogReset_Internal(uint16_t num_of_ms, IWDG_TypeDef *myWD);

/******************************************************************************
 * @brief  Perform a system reset by watchdog reset
 * @param  ms - ms to delay before reset
 * @note  Implementation is hardware dependant
 *         
 * @retval None
 *****************************************************************************/
void TimerWatchdogReset(uint16_t num_of_ms)
{
   TimerWatchdogReset_Internal(num_of_ms, IWDG);
}