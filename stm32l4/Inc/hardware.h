
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
 * customization for STM32H7xx
 */
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#define DMA_IS_LINEAR(hdma)     HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC)
#define DMA_GET_RXSIZE(hdma)    (hdma->Instance->CNDTR)

uint32_t GetAPB1TimerFrequency  (void);
uint32_t GetAPB2TimerFrequency  (void);
void     TimerWatchdogReset     (uint16_t waitms);        /* Force reset by watchdog timeout             */

#endif /* __HARDWARE_H__ */