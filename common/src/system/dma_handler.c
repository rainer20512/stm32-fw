/**
 ******************************************************************************
 * @file    dma_handler.c
 * @author  Rainer
 * @brief   first, assign an interrupt device and channel to a given interrupt
            request ( in case of no DMAMUX present, these devices/channels are
            fixed, in case of DMAMUX present, these devices/channels will be
            assigned one by one.
 *          second, do all the DMA interrupt handling 
 *          **** 004 **** Implemented to handle both devices with DMAMUX and
 *          older devices with fixed dma channels
 ******************************************************************************
 */

/** @addtogroup DMA channel handling
  * @{
  */
#include "config/config.h"
#include "error.h"
#include "system/hw_util.h"
#include "log.h"
#include "system/profiling.h"
#include "system/dma_handler.h"

#define DMA_INSTANCES 2
#define DMA_CHANNELS 7


/* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
DMA_HandleTypeDef   *handles    [DMA_INSTANCES][DMA_CHANNELS] = {0 };

/* Array of "is in use"-flags for all available DMA channels */
uint8_t             bDmaChUsed  [DMA_INSTANCES][DMA_CHANNELS] = {0 };

/* Array of all DMA channels */
static DMA_Channel_TypeDef* AllChannels[DMA_INSTANCES][DMA_CHANNELS] =
{
  { DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7 },
  { DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5, DMA2_Channel6, DMA2_Channel7 }
};

/* Array of all corresponding DMA channel interrupt numbers */
static IRQn_Type AllChannelIrqNums[DMA_INSTANCES][DMA_CHANNELS] =
{
  { DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn },
  { DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_IRQn, DMA2_Channel5_IRQn, DMA2_Channel6_IRQn, DMA2_Channel7_IRQn }
};

/*******************************************************************************************************
 * Return the DMA channel index for a given DMA Channel
 * -1 is returned in case of illegal DMA channel
 ******************************************************************************************************/
static int8_t GetDmaChannelIdx(DMA_Channel_TypeDef *ch, uint8_t DmaInstanceIdx )
{
    for ( uint32_t i = 0; i < DMA_CHANNELS; i++ )
        if ( AllChannels[DmaInstanceIdx][i] == ch ) return (int8_t)i;

    DBG_ERROR("GetDmaChannelIdx: Illegal DMA-Channel %p for DMA%d\n", ch, DmaInstanceIdx+1);  
    return -1;

    
}
/*******************************************************************************************************
 * Return the DMA instance for a given DMA Channel
 * NULL is returned in case of illegal DMA channel
 ******************************************************************************************************/
static DMA_TypeDef *GetDmaInstance(DMA_Channel_TypeDef *ch )
{
    if ( ch >= DMA1_Channel1 && ch <= DMA1_Channel7 )   return DMA1;
    if ( ch >= DMA2_Channel1 && ch <= DMA2_Channel7 )   return DMA2;
    DBG_ERROR("GetDmaInstance: Illegal DMA-Channel %p\n", ch);  
    return NULL;
}

/*******************************************************************************************************
 * Return the DMA channel IRQ number for a given DMA channel
 * DMA_ILLEGAL_CHANNEL
 ******************************************************************************************************/
IRQn_Type HW_DMA_GetChannelIrqNum(DMA_Channel_TypeDef *channel)
{
    /* get and check associated DMA instance */
    DMA_TypeDef *dma    = GetDmaInstance(channel);
    if ( !dma ) return DMA_ILLEGAL_CHANNEL;

    uint8_t DmaInstanceIdx = dma == DMA1 ? 0 : 1;

    /* get and check the Channel index ( 0 .. 6 ) */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel, DmaInstanceIdx);
    if ( DmaChannelIdx < 0 ) return DMA_ILLEGAL_CHANNEL;

    return AllChannelIrqNums[DmaInstanceIdx][DmaChannelIdx];
    
}

/*******************************************************************************************************
 * Find the next free DMA channel in case the statically specified channel is NULL
 * @param 
 *    none      
 * @retval 
 *    the first unused DMA channel, if any unused channel is left
 *    NULL in case of all DMA channels already in use
 * @note 
 *    if a free channel is found, it will NOT be allocated automatically ( by setting the used-bit )
 ******************************************************************************************************/
static DMA_Channel_TypeDef *GetFreeDmaChannel(void)
{
    for ( uint32_t i = 0; i < DMA_INSTANCES; i++ ) 
        for ( uint32_t j = 0; j < DMA_CHANNELS; j++ ) 
            if ( !bDmaChUsed[i][j] ) return AllChannels[i][j];
    
    DBG_ERROR("HW_DMA_GetFreeDmaChannel: No more free DMA channel\n");
    return NULL;
}

/*******************************************************************************************************
 * Enable the DMA hardware clock for a given DMA channel and enable the DMAMUX clock, if implemented
 ******************************************************************************************************/
static void HW_DMA_SetDmaChClock ( DMA_Channel_TypeDef *channel)
{

    #if !defined(DMA2)
        HW_SetHWClock(DMA1, 1 );
    #else
        /* Get the DMA instance from DMA channel */
        if ((uint32_t)(channel) < (uint32_t)(DMA2_Channel1)) {
            /* DMA1 */
            HW_SetHWClock(DMA1, 1 );
        } else  {
            /* DMA2 */
            HW_SetHWClock(DMA2, 1 );
        }
    #endif

    /* Enable Clock for DMAMUX1 in nay case */
    #if defined(DMAMUX1)
        HW_SetHWClock(DMAMUX1, 1 );
    #endif
}

/*******************************************************************************************************
 * register one DMA channel for the DMA specified in dmadata                                 
 * @param 
 *    dmadata [in]  -  ptr to all neccessary DMA data: associated DMA_HandleTypeDef, DMA channel, DMA request
 *                     number, DMA priority. The DMA channel in dmadata may be NULL, in which case the first
 *                     unused DMA channel will be allocated.
 *    dmadata [out] -  dmadata->dmaHandle->Instance will receive the ( manually or auotmatically ) assigned
 *                     DMA channel on successful returns
 * @retval 
 *    DMA handle is returned in case of success,
 *    NULL in case of channel already in use
 ******************************************************************************************************/
DMA_HandleTypeDef *HW_DMA_RegisterDMAChannel(const HW_DmaType* dmadata )
{
    DMA_Channel_TypeDef *channel = dmadata->dmaChannel;
    
    /* if no channel is specified, find one */
    if ( !channel ) {
        channel = GetFreeDmaChannel();
        if (!channel) return NULL;
    }

    /* get and check associated DMA instance */
    DMA_TypeDef *dma    = GetDmaInstance(channel);
    if ( !dma ) return NULL;
    uint8_t DmaInstanceIdx = dma == DMA1 ? 0 : 1;
    
    /* get and check the Channel index ( 0 .. 6 ) */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel, DmaInstanceIdx);
    if ( DmaChannelIdx < 0 ) return NULL;

    /* check for channel being available */
    if ( bDmaChUsed[DmaInstanceIdx][DmaChannelIdx] ) {
        DBG_ERROR("RegisterDMAChannel: DMA%d Channel %d already in use\n", DmaInstanceIdx+1, DmaChannelIdx+1);  
        return NULL;
    }

    /* register Channel and mark as used */
    handles[DmaInstanceIdx][DmaChannelIdx]      = dmadata->dmaHandle;
    bDmaChUsed[DmaInstanceIdx][DmaChannelIdx]   = true;

    /* store the assigned channel in handle data */
    dmadata->dmaHandle->Instance = channel;

    /* Enable the DMA ( and DMAMUX ) clock */
    HW_DMA_SetDmaChClock(channel);
    
    return dmadata->dmaHandle;
}

/******************************************************************************
 * Init an HAL DMA handle to the given parameters. This initialization is
 * hardware specific
 *****************************************************************************/
void HW_DMA_HandleInit(DMA_HandleTypeDef *hdma, const HW_DmaType *dma, void *parent )
{
//  hdma->Instance                 = dma->dmaChannel;
  /* Instance will be written by function RegisterDmaChannel */
  hdma->Parent                   = parent;
  hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma->Init.MemInc              = DMA_MINC_ENABLE;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma->Init.Mode                = DMA_NORMAL;
  hdma->Init.Priority            = dma->dmaPrio;
  hdma->Init.Request             = dma->dmaRequest;
}

/******************************************************************************
 * Set the IRQ priority for a given DMA channel and then 
 * enable the DMA channel interrupt
 *****************************************************************************/
void HW_DMA_SetAndEnableChannelIrq(DMA_Channel_TypeDef *channel, uint8_t prio, uint8_t subprio)
{
    IRQn_Type irq = HW_DMA_GetChannelIrqNum(channel);
    HAL_NVIC_SetPriority(irq, prio, subprio);
    HAL_NVIC_EnableIRQ(irq);
}

/******************************************************************************
 * Invalidates a dma channel: Disable DMA-channel interrupt, DeInit Handle,
 * mark channel as "unused"
 *****************************************************************************/
void HW_DMA_HandleDeInit(DMA_HandleTypeDef *hdma)
{
    DMA_Channel_TypeDef *channel = hdma->Instance;
    IRQn_Type irq = HW_DMA_GetChannelIrqNum(channel);
    HAL_NVIC_DisableIRQ(irq);
    HAL_DMA_DeInit(hdma);

    /* get and check associated DMA instance */
    DMA_TypeDef *dma    = GetDmaInstance(channel);
    uint8_t DmaInstanceIdx = dma == DMA1 ? 0 : 1;
    
    /* get and check the Channel index ( 0 .. 6 ) */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel, DmaInstanceIdx);

    /* unregister Channel and mark as unused */
    handles[DmaInstanceIdx][DmaChannelIdx]      = NULL;
    bDmaChUsed[DmaInstanceIdx][DmaChannelIdx]   = false;
}

/*******************************************************************************************************
 * @brief  Generate the DMA IRQ handlers for DMA1 and 2 and channel 0 .. 6 by Macro
 * @param  None
 * @retval None
 ******************************************************************************************************/
#define DMA_IRQ(dev, channel)               \
void DMA##dev##_Channel##channel##_IRQHandler(void)                 \
{                                                                  \
  ProfilerPush(JOB_IRQ_DMA);                                       \
  /* find the assigned handle, if any */                           \
  DMA_HandleTypeDef *handle = handles[dev-1][channel-1];             \
  if ( !handle) {                                                  \
    DBG_ERROR("DMA %d, Channel %d: No handler!\n", dev, channel);  \
    return;                                                        \
  }                                                                \
  HAL_DMA_IRQHandler(handle);                                      \
  ProfilerPop();                                                   \
}

DMA_IRQ(1, 1)
DMA_IRQ(1, 2)
DMA_IRQ(1, 3)
DMA_IRQ(1, 4)
DMA_IRQ(1, 5)
DMA_IRQ(1, 6)
DMA_IRQ(1, 7)

DMA_IRQ(2, 1)
DMA_IRQ(2, 2)
DMA_IRQ(2, 3)
DMA_IRQ(2, 4)
DMA_IRQ(2, 5)
DMA_IRQ(2, 6)
DMA_IRQ(2, 7)


/**
  * @}
  */

