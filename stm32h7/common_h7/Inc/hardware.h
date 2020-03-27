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
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define USART_ISR_RXNE                  USART_ISR_RXNE_RXFNE
#define USART_ISR_TXE                   USART_ISR_TXE_TXFNF

#define DMA_Channel_TypeDef             DMAMUX_Channel_TypeDef
#define DMA_IS_LINEAR(hdma)             HAL_IS_BIT_CLR( ( (DMA_Stream_TypeDef *)hdma->Instance)->CR, DMA_SxCR_CIRC)
#define DMA_GET_RXSIZE(hdma)            ( ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR )


uint32_t GetAPB1TimerFrequency  (void);
uint32_t GetAPB2TimerFrequency  (void);
void     TimerWatchdogReset     (uint16_t waitms);        /* Force reset by watchdog timeout             */

