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
 *
 * @note    Special considerations for dual cores: The channels are split
 *          between both cores, code has to be include by both cores
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

/*
 * on dual core architectures, the DMA ressources are split equally between both cores
 * DMA1 for CM7, DMA2 for CM4
 *
 * the split may be changed in other ways, if required
 */
#if defined(CORE_CM7)
    #if defined(DUAL_CORE)
        #define MAX_DMA_CHANNELS        8
        #define MAX_BDMA_CHANNELS       4

        /* Array of all DMA Streams for core CM7 */
        static DMA_Stream_TypeDef* AllChannels[MAX_DMA_CHANNELS] =
          { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7 };

        /* Array of all corresponding DMA stream interrupt numbers */
        static IRQn_Type AllChannelIrqNums[MAX_DMA_CHANNELS] =
          { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn };

        /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
        DMA_HandleTypeDef   *handles    [MAX_DMA_CHANNELS] = {0 };

        /* Array of "is in use"-flags for all available DMA channels */
        uint8_t             bDmaChUsed  [MAX_DMA_CHANNELS] = {0 };

    #else
        /* CM7 single core */
        #define MAX_DMA_CHANNELS        16
        #define MAX_BDMA_CHANNELS       8

        /* Array of all DMA Streams for core CM7 */
        static DMA_Stream_TypeDef* AllChannels[MAX_DMA_CHANNELS] =
          { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7, 
            DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 };

        /* Array of all corresponding DMA stream interrupt numbers */
        static IRQn_Type AllChannelIrqNums[MAX_DMA_CHANNELS] =
          { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
            DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn };

        /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
        DMA_HandleTypeDef   *handles    [MAX_DMA_CHANNELS] = {0 };

        /* Array of "is in use"-flags for all available DMA channels */
        uint8_t             bDmaChUsed  [MAX_DMA_CHANNELS] = {0 };
    #endif

#elif defined(DUAL_CORE) && defined(CORE_CM4)
    #define MAX_DMA_CHANNELS        8
    #define MAX_BDMA_CHANNELS       4

    /* Array of all DMA Streams for core CM4 */
    static DMA_Stream_TypeDef* AllChannels[MAX_DMA_CHANNELS] =
      { DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 };

    /* Array of all corresponding DMA stream interrupt numbers */
    static IRQn_Type AllChannelIrqNums[MAX_DMA_CHANNELS] =
      { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn };

    /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
    DMA_HandleTypeDef   *handles    [MAX_DMA_CHANNELS] = {0 };

    /* Array of "is in use"-flags for all available DMA channels */
    uint8_t             bDmaChUsed  [MAX_DMA_CHANNELS] = {0 };

#else
    #error "No DMA handler for unknown core"
#endif


/*******************************************************************************************************
 * Return the DMA channel index for a given DMA Channel
 * -1 is returned in case of illegal DMA channel
 ******************************************************************************************************/
static int8_t GetDmaChannelIdx(DMA_Stream_TypeDef *ch )
{
    for ( uint32_t i = 0; i < MAX_DMA_CHANNELS; i++ )
        if ( AllChannels[i] == ch ) return (int8_t)i;

    DBG_ERROR("GetDmaChannelIdx: Illegal DMA-Channel @0x%p for DMA%d\n", ch );  
    return -1;

    
}

/*******************************************************************************************************
 * Return the DMA channel IRQ number for a given DMA channel
 * DMA_ILLEGAL_CHANNEL
 ******************************************************************************************************/
IRQn_Type HW_DMA_GetChannelIrqNum(DMA_Stream_TypeDef *channel)
{
    /* Get the channel index */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel);
    if ( DmaChannelIdx < 0 ) return DMA_ILLEGAL_CHANNEL;

    return AllChannelIrqNums[DmaChannelIdx];
    
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
static DMA_Stream_TypeDef *GetFreeDmaChannel(void)
{
    for ( uint32_t i = 0; i < MAX_DMA_CHANNELS; i++ ) 
        if ( !bDmaChUsed[i]) return AllChannels[i];
    
    DBG_ERROR("HW_DMA_GetFreeDmaChannel: No more free DMA channel\n");
    return NULL;
}

/*******************************************************************************************************
 * Enable the DMA hardware clock for a given DMA channel and enable the DMAMUX clock, if implemented
 ******************************************************************************************************/
static void HW_DMA_SetDmaChClock ( DMA_Stream_TypeDef *channel)
{

    #if defined ( BDMA )
        if ((uint32_t)(channel) >= (uint32_t)(BDMA_Channel0) && (uint32_t)(channel) <= (uint32_t)(BDMA_Channel7) ) {
            HW_SetHWClock(BDMA, 1 );
            return; 
        }
    #endif

    #if !defined(DMA2)
        HW_SetHWClock(DMA1, 1 );
    #else
        /* Get the DMA instance from DMA channel */
        if ((uint32_t)(channel) < (uint32_t)(DMA2_Stream0)) {
            /* DMA1 */
            HW_SetHWClock(DMA1, 1 );
        } else  {
            /* DMA2 */
            HW_SetHWClock(DMA2, 1 );
        }
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
 *    dma_user [in] -  one of DMA_UserEnumT ( Either CM7 or CM4 )
 * @retval 
 *    DMA handle is returned in case of success,
 *    NULL in case of channel already in use
 ******************************************************************************************************/
DMA_HandleTypeDef *HW_DMA_RegisterDMAChannel (const HW_DmaType* dmadata)
{
    DMA_Stream_TypeDef *channel = dmadata->dmaChannel;
    
    /* if no channel is specified, find one */
    if ( !channel ) {
        channel = GetFreeDmaChannel();
        if (!channel) return NULL;
    }
        
    /* get and check the Channel index */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel);
    if ( DmaChannelIdx < 0 ) return NULL;

    /* check for channel being available */
    if ( bDmaChUsed[DmaChannelIdx] ) {
        DBG_ERROR("RegisterDMAChannel: DMA Channel %d already in use\n", DmaChannelIdx);  
        return NULL;
    }

    /* register Channel*/ 
    handles[DmaChannelIdx]  = dmadata->dmaHandle;

    /* mark as used in any case */
    bDmaChUsed[DmaChannelIdx]   = true;

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
  hdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma->Init.MemBurst            = DMA_MBURST_INC4;
  hdma->Init.PeriphBurst         = DMA_MBURST_INC4;
}

/******************************************************************************
 * Set the IRQ priority for a given DMA channel and then 
 * enable the DMA channel interrupt
 *****************************************************************************/
void HW_DMA_SetAndEnableChannelIrq(DMA_Stream_TypeDef *channel, uint8_t prio, uint8_t subprio)
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
    /* disable DMA channel interrupt */
    DMA_Stream_TypeDef *channel = hdma->Instance;
    IRQn_Type irq = HW_DMA_GetChannelIrqNum(channel);
    HAL_NVIC_DisableIRQ(irq);
    HAL_DMA_DeInit(hdma);

    /* unregister Channel and mark as unused */
    
    /* get and check the Channel index ( 0 .. 6 ) */
    int8_t DmaChannelIdx  = GetDmaChannelIdx(channel);

    /* unregister Channel and mark as unused */
    handles[DmaChannelIdx]      = NULL;
    bDmaChUsed[DmaChannelIdx]   = false;
}

/*******************************************************************************************************
 * @brief  Generate the DMA IRQ handlers for all DMA channel by macro
 * @param  dev     - 1                      for DMA1, 2 for DMA2
 * @param  channel - 0..7                   for DMAx_Channel<channel>
 * @param  idx     - 0..MAX_DMA_CHANNELS-1  corresponding index in handles[] array
 *
 ******************************************************************************************************/
#define DMA_IRQ(dev, channel,idx)                                  \
void DMA##dev##_Stream##channel##_IRQHandler(void)                 \
{                                                                  \
  ProfilerPush(JOB_IRQ_DMA);                                       \
  /* find the assigned handle, if any */                           \
  DMA_HandleTypeDef *handle = handles[idx];                        \
  if ( !handle) {                                                  \
    DBG_ERROR("DMA %d, Stream %d: No handler!\n", dev, channel);   \
    return;                                                        \
  }                                                                \
  HAL_DMA_IRQHandler(handle);                                      \
  ProfilerPop();                                                   \
}

#if defined(CORE_CM7)
    #if defined(DUAL_CORE)
        DMA_IRQ(1, 0, 0)
        DMA_IRQ(1, 1, 1)
        DMA_IRQ(1, 2, 2)
        DMA_IRQ(1, 3, 3)
        DMA_IRQ(1, 4, 4)
        DMA_IRQ(1, 5, 5)
        DMA_IRQ(1, 6, 6)
        DMA_IRQ(1, 7, 7)
    #else 
        DMA_IRQ(1, 0, 0)
        DMA_IRQ(1, 1, 1)
        DMA_IRQ(1, 2, 2)
        DMA_IRQ(1, 3, 3)
        DMA_IRQ(1, 4, 4)
        DMA_IRQ(1, 5, 5)
        DMA_IRQ(1, 6, 6)
        DMA_IRQ(1, 7, 7)
        DMA_IRQ(2, 0, 8)
        DMA_IRQ(2, 1, 9)
        DMA_IRQ(2, 2, 10)
        DMA_IRQ(2, 3, 11)
        DMA_IRQ(2, 4, 12)
        DMA_IRQ(2, 5, 13)
        DMA_IRQ(2, 6, 14)
        DMA_IRQ(2, 7, 15)
    #endif

#elif defined(CORE_CM4)
    DMA_IRQ(2, 0, 0)
    DMA_IRQ(2, 1, 1)
    DMA_IRQ(2, 2, 2)
    DMA_IRQ(2, 3, 3)
    DMA_IRQ(2, 4, 4)
    DMA_IRQ(2, 5, 5)
    DMA_IRQ(2, 6, 6)
    DMA_IRQ(2, 7, 7)
#endif

#if 0 
void DMA1_Stream4_IRQHandler(void) 
{
  ProfilerPush(JOB_IRQ_DMA);
  /* find the assigned handle, if any */
  DMA_HandleTypeDef *handle = handles[4];
  if ( !handle) {
    DBG_ERROR("DMA %d, Stream %d: No handler!\n", 1, 4);
    return;
  }
  HAL_DMA_IRQHandler(handle);
  ProfilerPop();
}
#endif

/**
  * @}
  */

