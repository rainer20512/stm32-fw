/*******************************************************************************
 * @file    hardware.h
 * @author  rainer
 * @brief   include all hardware specfic files
 *
 * This file has to be adopted for different STM32 hardwares. And it should be the
 * only place where hardware adoption takes place
 ******************************************************************************/
 
/*
 * customization for STM32H7xx
 */
#include "config/config.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define USART_ISR_RXNE                  USART_ISR_RXNE_RXFNE
#define USART_ISR_TXE                   USART_ISR_TXE_TXFNF

#define DMA_Channel_TypeDef             DMAMUX_Channel_TypeDef
#define DMA_IS_LINEAR(hdma)             HAL_IS_BIT_CLR( ( (DMA_Stream_TypeDef *)hdma->Instance)->CR, DMA_SxCR_CIRC)
#define DMA_GET_RXSIZE(hdma)            ( ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR )

#if defined(CORE_CM7)
    #define DMAMEM                      __attribute((section(".dmamem")))
    #define IPCMEM                      __attribute((section(".ipcmem")))
    #define AXISMEM                     __attribute((section(".axismem")))    
#else
    #define DMAMEM                          
#endif

/* --- Type specific setup for ADC ------------------------------------------*/
#define ADC_HAS_REFINT(inst)            ( inst == (void *)ADC3_BASE )
#define ADC_HAS_CHIPTEMP(inst)          ( inst == (void *)ADC3_BASE )

/* --- Type specific setup for Timers ---------------------------------------*/
extern const TIM_TypeDef* apb1_timers[9];    /* Timers connected to APB1 */
extern const TIM_TypeDef* apb2_timers[5];    /* Timers connected to APB2 */
uint32_t GetAPB1TimerFrequency  (void);     /* Get APB1 Timer input frq */
uint32_t GetAPB2TimerFrequency  (void);     /* Get APB2 Timer input frq */
uint32_t GetPerClkFrequency     (void);     /* Get the peripheral clock frq */
void     TimerWatchdogReset     (uint16_t waitms);        /* Force reset by watchdog timeout             */

