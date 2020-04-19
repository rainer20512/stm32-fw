/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_AMP_Dual_RTOS/CM7/Src/stm32h7xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"

#include "config/devices_config.h"
#include "config/uart_config.h"
#include "config/spi_config.h"
#include "config/i2c_config.h"
#include "config/adc_config.h"

#if USE_QSPI > 0
    #include "config/qspi_config.h"
#endif
#if USE_QENCODER > 0
    #include "config/qencode_config.h"
#endif 

#if USE_CAN > 0
    #include "config/can_config.h"
#endif 


#if defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3)
  #include "dev/spi.h"
#endif

#if defined(USE_TIM7) || defined(USE_TIM6)
    #include "dev/timer_dev.h"
#endif

#include "error.h"
#include "rtc.h"
#include "task/minitask.h"
#include "timer.h"
#include "hardware.h" 
#include "debug_helper.h"
#include "system/profiling.h"


#include "dev/devices.h"
   

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
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

/* 
 * Hardfault handler is implemented in hardfault.s this handler will call the 
 * c routine above 
 */

#if 0
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#endif

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
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/*                     Peripherals Interrupt Handlers                         */
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
    #if !defined(COM9_IRQHandler) 
        #error "IRQ names not defined for LPUART1" 
    #endif
    void COM9_IRQHandler(void)
    {
        ProfilerPush(JOB_IRQ_UART);
        UsartIRQHandler(&HandleCOM9);
        ProfilerPop();
    }
  #if defined(COM9_USE_TX_DMA) || defined(COM9_USE_RX_DMA) 
    #if defined(COM9_USE_TX_DMA)
        void COM9_DMA_TX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM9.hTxDma);
            ProfilerPop();
        }
    #endif
    #if defined(COM9_USE_RX_DMA)
        void COM9_DMA_RX_IRQHandler(void) 
        {
            ProfilerPush(JOB_IRQ_UART);
            HAL_DMA_IRQHandler(HandleCOM9.hRxDma);
            ProfilerPop();
        }
    #endif
  #endif
#endif

/******************************************************************************
 * SPI Interrupt handlers
 *****************************************************************************/
#if defined(SPI1) && defined(USE_SPI1)
    #ifdef SPI1_USE_HW_IRQ
        #if !defined(SPI1_IRQHandler) 
            #error "IRQ names not defined for SPI1" 
        #endif
        void SPI1_IRQHandler(void)
        {
          ProfilerPush(JOB_IRQ_SPI);
          HAL_SPI_IRQHandler(&SPI1Handle.data->hw.myHalHandle);
          ProfilerPop();
        }
    #endif

    #ifdef SPI1_USE_DMA
        void SPI1_DMA_RX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI1Handle.data->hw.myHalHandle.hdmarx);
            ProfilerPop();
        }

        void SPI1_DMA_TX_IRQHandler(void)
        {
            ProfilerPush(JOB_IRQ_SPI);
            HAL_DMA_IRQHandler(SPI1Handle.data->hw.myHalHandle.hdmatx);
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

/******************************************************************************
 * I2C Interrupt handlers
 *****************************************************************************/
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

/******************************************************************************
 * ADC Interrupt handlers
 *****************************************************************************/
#if defined(ADC3) && defined(USE_ADC3)
    #ifdef ADC3_USE_IRQ
        #if !defined(ADC3_IRQHandler) 
            #error "IRQ names not defined for ADC3" 
        #endif
        void ADC3_IRQHandler(void)
        {
          ProfilerPush(JOB_ADC);
          HAL_ADC_IRQHandler(&ADC3Handle.hAdc);
          ProfilerPop();
        }
    #endif

    #ifdef ADC3_USE_DMA
        #if !defined(ADC3_DMA_IRQHandler)
            #error "DMA-IRQ handler not defined for ADC3"
        #endif
        void ADC3_DMA_IRQHandler(void)
        {
            ProfilerPush(JOB_ADC);
            HAL_DMA_IRQHandler(ADC3Handle.hAdc.DMA_Handle);
            ProfilerPop();
        }
    #endif
#endif

/******************************************************************************
 * timer in Quadrature encoder mode
 *****************************************************************************/
#if USE_QENCODER > 0
    void QENC1TIM_IRQHandler(void)
    {
      HAL_TIM_IRQHandler(&QEnc1Handle.htim);
    }
#endif

/******************************************************************************
 * basic timers
 *****************************************************************************/
#if defined(USE_TIM7)
    /**
      * @brief  Handles Microsecond timer overrun interupts. 
                Will occur 15,2 times a second ( exactly 1.000.000 / 65.536 per second ) 
      * @param  None
      * @retval None
      */
    void TIM7_IRQHandler(void)
    {
      BASTMR_IrqHandler( &TIM7Handle.MicroCountHigh, TIM7);
    }
#endif
#if defined(USE_TIM6)
    /**
      * @brief  Handles Microsecond timer overrun interupts. 
                Will occur 15,2 times a second ( exactly 1.000.000 / 65.536 per second ) 
      * @param  None
      * @retval None
      */
    void TIM6_IRQHandler(void)
    {
      BASTMR_IrqHandler( &TIM6Handle.MicroCountHigh, TIM6);
    }
#endif

/******************************************************************************
 * QSPI interrupt handlers
 *****************************************************************************/
#if USE_QSPI > 0

    #if defined(QSPI1_USE_IRQ)
        #if !defined(QSPI1_IRQHandler)
            #error "QSPI1_IRQHandler undefined!"
        #endif
        void QSPI1_IRQHandler ( void ) 
        {
            HAL_QSPI_IRQHandler(&QSpi1Handle.hqspi);
        }
    #endif

    #if defined(QSPI1_USE_DMA)
        void QSPI1_DMA_IRQHandler ( void ) 
        {
        }
    #endif
#endif

/******************************************************************************
 * CAN interrupt handlers
 *****************************************************************************/
#if USE_CAN > 0
    #if defined(CAN1) && defined(CAN1)
           void CAN1_TX_IRQHandler ( void )
           {
                if (  CAN1Handle.CanOnTx ) CAN1Handle.CanOnTx();
           }
           void CAN1_RX0_IRQHandler ( void )
           {
                CAN_RxIrqStub(&CAN1Handle, 0 );
           }
           void CAN1_RX1_IRQHandler ( void )
           {
                CAN_RxIrqStub(&CAN1Handle, 1 );
           }
           void CAN1_SCE_IRQHandler ( void )
           {
                CAN_ErrorStub( &CAN1Handle );
           }
    #endif
#endif

/**
  * @brief  This function handles EXTI1 interrupt request. This IRQ is used
  *         for IPC from CM4 To CM7
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler( void )
{
 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/******************************************************************************/
/*                 STM32H7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32h7xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
