
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

#define DMA_Channel_TypeDef     DMAMUX_Channel_TypeDef

