/**
 ******************************************************************************
 * @file    dma_handler.h
 * @author  Rainer
 * @brief   first, assign an interrupt device and channel to a given interrupt
            request ( in case of no DMAMUX present, these devices/channels are
            fixed, in case of DMAMUX present, these devices/channels will be
            assigned one by one, if no channel is statically specified
 *          second, do all the DMA interrupt handling 
 ******************************************************************************
 */

#ifndef __DMA_HANDLER_H__
#define __DMA_HANDLER_H__

#include "config/config.h"

/******************************************************************************
 * L4 family HAL layer uses DMA_Channel_TypeDef, 
 whilst H7 HAL uses DMA_Stream_TypeDef
 *****************************************************************************/
#if defined(STM32L4_FAMILY)
    typedef DMA_Channel_TypeDef DMA_ChannelT;
#elif defined(STM32H7_FAMILY)
    typedef DMA_Stream_TypeDef DMA_ChannelT;
#else
    #error "Missing DMA setup for MCU family"
#endif


#define DMA_ILLEGAL_CHANNEL     255

IRQn_Type         HW_DMA_GetChannelIrqNum        (DMA_Channel_TypeDef *channel);
DMA_HandleTypeDef *HW_DMA_RegisterDMAChannel     (const HW_DmaType* dmadata );
void              HW_DMA_HandleInit              (DMA_HandleTypeDef *hdma, const HW_DmaType *dma, void *parent );
void              HW_DMA_HandleDeInit            (DMA_HandleTypeDef *hdma);
void              HW_DMA_SetAndEnableChannelIrq (DMA_Channel_TypeDef *channel, uint8_t prio, uint8_t subprio);

 #endif /* __DMA_HANDLER_H__ */