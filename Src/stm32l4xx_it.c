  /**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Src/stm32l0xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"
#include "config/devices_config.h"
#include "config/spi_config.h"
#include "config/adc_config.h"
#if USE_QENCODER > 0
    #include "config/qencode_config.h"
#endif 
#include "config/system.h"

#include "error.h"
#include "system/profiling.h"
#include "rtc.h"
#include "task.h"
#include "timer.h"
#include "stm32l4xx_it.h"
#include "debug_helper.h"


#include "dev/devices.h"
   
/** @addtogroup STM32L0xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComIT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    /* If flash write in eepro emulation produeces an error, then handle this */
    #if USE_EEPROM_EMUL > 0
        extern void EEPROM_NMI_Handler(void);
        EEPROM_NMI_Handler();
    #endif
}


#if USE_EEPROM_EMUL > 0
    /* Handler for Interrupt driven page erase when using eeprom emulation */
    void FLASH_IRQHandler(void)
    {
      HAL_FLASH_IRQHandler();
    }
  
    /* @brief  This function handles PVD interrupt request.
     * @param  None
     * @retval None
     */
    void PVD_PVM_IRQHandler(void)
    {
      /* Loop inside the handler to prevent the Cortex from using the Flash,
         allowing the flash interface to finish any ongoing transfer. */
      #if DEBUG_MODE > 0 
        DEBUG_PUTS("Detected low power situation. Waiting to recover...");
      #endif
      while (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
      {
      }
      #if DEBUG_MODE > 0 
        DEBUG_PUTS("Low power situation ended - resuming ...");
      #endif
    }
#endif

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */

#if 0
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#endif

// RHB chnged: Handcrafted HardFault Handler 

// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s
void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
  debug_printf ("\n\n[Hard fault handler - all numbers in hex]\n");
  debug_printf ("R0       = %08x\n", stacked_r0);
  debug_printf ("R1       = %08x\n", stacked_r1);
  debug_printf ("R2       = %08x\n", stacked_r2);
  debug_printf ("R3       = %08x\n", stacked_r3);
  debug_printf ("R12      = %08x\n", stacked_r12);
  debug_printf ("LR [R14] = %08x  subroutine call return address\n", stacked_lr);
  debug_printf ("PC [R15] = %08x  program counter\n", stacked_pc);
  debug_printf ("PSR      = %08x\n", stacked_psr);
  debug_printf ("BFAR     = %08x\n", (*((volatile unsigned long *)(0xE000ED38))));
  debug_printf ("CFSR     = %08x\n", (*((volatile unsigned long *)(0xE000ED28))));
  debug_printf ("HFSR     = %08x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  debug_printf ("DFSR     = %08x\n", (*((volatile unsigned long *)(0xE000ED30))));
  debug_printf ("AFSR     = %08x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  debug_printf ("SCB_SHCSR= %08x\n", SCB->SHCSR);
 
  while (1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */

#if defined(USART1) && defined(USE_USART1)
    #if !defined(COM1_IRQHandler) 
        #error "IRQ names not defined for USART1" 
    #endif
    void COM1_IRQHandler(void)
  {
    ProfilerPush(JOB_IRQ_UART);
    UsartIRQHandler(&HandleCOM1);

    ProfilerPop();
  }
  #if defined(COM1_USE_TX_DMA) || defined(COM1_USE_RX_DMA) 

    #if defined(COM1_USE_TX_DMA)
        void COM1_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM1.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM1_USE_RX_DMA)
        void COM1_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM1.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

#if defined(USART2) && defined(USE_USART2)
    #if !defined(COM2_IRQHandler) 
        #error "IRQ names not defined for USART2" 
    #endif
    void COM2_IRQHandler(void)
  {
    ProfilerPush(JOB_IRQ_UART);
    UsartIRQHandler(&HandleCOM2);
    ProfilerPop();
  }
  #if defined(COM2_USE_TX_DMA) || defined(COM2_USE_RX_DMA) 

    #if defined(COM2_USE_TX_DMA)
        void COM2_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM2.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM2_USE_RX_DMA)
        void COM2_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM2.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif


#if defined(USART3) && defined(USE_USART3)
    #if !defined(COM3_IRQHandler) 
        #error "IRQ names not defined for USART3" 
    #endif
    void COM3_IRQHandler(void)

  {
    ProfilerPush(JOB_IRQ_UART);
    UsartIRQHandler(&HandleCOM3);
    ProfilerPop();
  }
  #if defined(COM3_USE_TX_DMA) || defined(COM3_USE_RX_DMA) 

    #if defined(COM3_USE_TX_DMA)
        void COM3_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM3.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM3_USE_RX_DMA)
        void COM3_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM3.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

#if defined(UART4) && defined(USE_UART4)
    #if !defined(COM4_IRQHandler) 
        #error "IRQ names not defined for UART4" 
    #endif
    void COM4_IRQHandler(void)
    {
        ProfilerPush(JOB_IRQ_UART);
        UsartIRQHandler(&HandleCOM4);
        ProfilerPop();
    }
  #if defined(COM4_USE_TX_DMA) || defined(COM4_USE_RX_DMA) 

    #if defined(COM4_USE_TX_DMA)
        void COM4_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM4.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM4_USE_RX_DMA)
        void COM4_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM4.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

#if defined(UART5) && defined(USE_UART5)
    #if !defined(COM5_IRQHandler) 
        #error "IRQ names not defined for UART5" 
    #endif
    void COM5_IRQHandler(void)
    {
        ProfilerPush(JOB_IRQ_UART);
        UsartIRQHandler(&HandleCOM5);
        ProfilerPop();
    }
  #if defined(COM5_USE_TX_DMA) || defined(COM5_USE_RX_DMA) 

    #if defined(COM5_USE_TX_DMA)
        void COM5_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM5.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM5_USE_RX_DMA)
        void COM5_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM5.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

#if defined(LPUART1) && defined(USE_LPUART1)
    #if !defined(COM6_IRQHandler) 
        #error "IRQ names not defined for LPUART1" 
    #endif
    void COM6_IRQHandler(void)
    {
        ProfilerPush(JOB_IRQ_UART);
        UsartIRQHandler(&HandleCOM6);
        ProfilerPop();
    }
  #if defined(COM6_USE_TX_DMA) || defined(COM6_USE_RX_DMA) 
    #if defined(COM6_USE_TX_DMA)
        void COM6_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM6.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM6_USE_RX_DMA)
        void COM6_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM6.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

/**
  * @brief  This function handles DMA interrupt requests for SPI devices.
  * @param  None
  * @retval None
  */
#if defined(SPI1) && defined(USE_SPI1)
    #ifdef SPI1_USE_HW_IRQ
        #if !defined(SPI1_IRQHandler) 
            #error "IRQ names not defined for SPI1" 
        #endif
        void SPI1_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_SPI);
          HAL_SPI_IRQHandler(&SPI1Handle.hw.myHalHandle);
          ProfilerPop();
        }
    #endif

    #ifdef SPI1_USE_DMA
        void SPI1_DMA_RX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI1Handle.hw.myHalHandle.hdmarx);
            ProfilerPop();
        }

        void SPI1_DMA_TX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI1Handle.hw.myHalHandle.hdmatx);
            ProfilerPop();
        }
        #endif
#endif

#if defined(SPI2) && defined(USE_SPI2)
    #ifdef SPI2_USE_HW_IRQ
        #if !defined(SPI2_IRQHandler) 
            #error "IRQ names not defined for SPI2" 
        #endif
        void SPI2_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_SPI);
          HAL_SPI_IRQHandler(&SPI2Handle.hSpi);
          ProfilerPop();
        }
    #endif

    #ifdef SPI2_USE_DMA
        void SPI2_DMA_RX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI2Handle.hSpi.hdmarx);
            ProfilerPop();
        }

        void SPI2_DMA_TX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI2Handle.hSpi.hdmatx);
            ProfilerPop();
        }
        #endif
#endif

#if defined(SPI3) && defined(USE_SPI3)
    #ifdef SPI3_USE_HW_IRQ
        #if !defined(SPI3_IRQHandler) 
            #error "IRQ names not defined for SPI3" 
        #endif
        void SPI3_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_SPI);
          HAL_SPI_IRQHandler(&SPI3Handle.hSpi);
          ProfilerPop();
        }
    #endif

    #ifdef SPI3_USE_DMA
        void SPI3_DMA_RX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI3Handle.hSpi.hdmarx);
            ProfilerPop();
        }

        void SPI3_DMA_TX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI3Handle.hSpi.hdmatx);
            ProfilerPop();
        }
        #endif
#endif

#if defined(I2C1) && defined(USE_I2C1)
    #ifdef I2C1_USE_IRQ
        #if !defined(I2C1_EV_IRQHandler) 
            #error "IRQ names not defined for I2C1" 
        #endif
        void I2C1_EV_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_I2C);
          HAL_I2C_EV_IRQHandler(&I2C1Handle.hI2c);
          ProfilerPop();
        }
        #if !defined(I2C1_ER_IRQHandler) 
            #error "IRQ names not defined for I2C1" 
        #endif
        void I2C1_ER_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_I2C);
          HAL_I2C_ER_IRQHandler(&I2C1Handle.hI2c);
          ProfilerPop();
        }
    #endif

    #ifdef I2C1_USE_DMA
        void I2C1_DMA_RX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_I2C);
            debug_putchar('r');
            HAL_DMA_IRQHandler(I2C1Handle.hI2c.hdmarx);
            ProfilerPop();
        }

        void I2C1_DMA_TX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_I2C);
            debug_putchar('t');
            HAL_DMA_IRQHandler(I2C1Handle.hI2c.hdmatx);
            ProfilerPop();
        }
        #endif
#endif

#if defined(ADC1) && defined(USE_ADC1)
    #ifdef ADC1_USE_IRQ
        #if !defined(ADC1_IRQHandler) 
            #error "IRQ names not defined for ADC1" 
        #endif
        void ADC1_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_ADC);
          HAL_ADC_IRQHandler(&ADC1Handle.hAdc);
          ProfilerPop();
        }
    #endif

    #ifdef ADC1_USE_DMA
        #if !defined(ADC1_DMA_IRQHandler)
            #error "DMA-IRQ handler not defined for ADC1"
        #endif
        void ADC1_DMA_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_ADC);
            HAL_DMA_IRQHandler(ADC1Handle.hAdc.DMA_Handle);
            ProfilerPop();
        }
    #endif
#endif

#if defined(USE_QENCODER)
    void QENC1TIM_IRQHandler(void)
    {
      HAL_TIM_IRQHandler(&QEnc1Handle.htim);
    }
#endif

void RTC_Alarm_IRQHandler(void)
{
  ProfilerPush(JOB_IRQ_RTC);
  /* Clear exti interrupt bit */
  EXTI->PR1 = EXTI_PR1_PIF18;

  /* Only Alarm A and B and WakeUp are handeled */
  #define RTC_ALARMS   ( RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_WUTF )
  
  /* Alarm A used for secondly ticks */
  if ( RTC->ISR & RTC_ISR_ALRAF ) {
     SET_TASK_BIT(TASK_RTC_BIT);
  }

  /* Alarm B used for millisecond timers */
  if ( RTC->ISR & RTC_ISR_ALRBF ) {
     // If there is an matching timer, activate Timer Task
    if ( MsTimerHandleCMP() ) SET_TASK_BIT(TASK_TMR_BIT);      
  }
  // if ( RTC->ISR & RTC_ISR_WUTF ) ...

  /* reset all interrupt bits by writing 0 to */
  RTC->ISR &= ~RTC_ALARMS;
  ProfilerPop();
}


/**
  * @brief  Handles Millisecond timer overrun interupts
  * @param  None
  * @retval None
  */
#if 0 
void MILLISEC_TMR_IRQHandler(void)
{
  /* Get Interrupt source : Auto-reload or compare match */
  if ( MILLISEC_TMR_CMP_ENABLED() && MILLISEC_TMR_CMP_IRQACTIVE() ) {

    ProfilerPush(JOB_IRQ_TMR);
    
    /* Clear Compare Match flag and activate timer task */
    MILLISEC_TMR_CMP_IRQCLEAR();

#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
    COM_print_time('!', false);
#endif

    // If there is an matching timer, activate Timer Task
    if ( MsTimerHandleCMP() ) SET_TASK_BIT(TASK_TMR_BIT);      

    ProfilerPop();
  }

  if ( MILLISEC_TMR_ARR_IRQACTIVE() ) {
    
    ProfilerPush(JOB_IRQ_RTC);

    /* Clear AutoReload done flag */
    MILLISEC_TMR_ARR_IRQCLEAR();

    /* setup Auto reload register */
    MILLISEC_TMR->ARR = RTC_GetARR();

#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
    // COM_print_time('+', false);
#endif

    /* Update rtc in interrupt context, so no other interrupt will obtain wrong time */
    RTC_TimerAddOneSecond();

    SET_TASK_BIT(TASK_RTC_BIT);
    if ( bHaveAdvance ) RTC_HandleDeferredCmp();
    
    ProfilerPop();
  }

}
#endif

/**
  * @brief  Handles Microsecond timer overrun interupts
  * @param  None
  * @retval None
  */
#if DEBUG_PROFILING > 0
    /**
      * @brief  Handles Microsecond timer overrun interupts. 
                Will occur 15,2 times a second ( exactly 1.000.000 / 65.536 per second ) 
      * @param  None
      * @retval None
      */
    void MICROCOUNTER_TMR_IRQHandler(void)
    {
      /* Reset interrupt flag by writing 0 to it */
      MICROCOUNTER_TMR->SR = ~TIM_SR_UIF;

      /* 
       * Increment has to be done before profiling, because profiling will access
       * the microsecond counter
       */
      ProfilerMicroCountHigh++;

      ProfilerPush(JOB_IRQ_PROFILER);
      /* Increment Counter Hi Word */
      ProfilerPop();
    }
#endif



/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
