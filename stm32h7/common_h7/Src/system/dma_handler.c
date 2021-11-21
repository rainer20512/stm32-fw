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
        #define MAX_DMA_STREAMS         8
        #define MAX_BDMA_CHANNELS       4

        /* Array of all DMA Streams for core CM7 */
        static DMA_Stream_TypeDef* AllDmaStreams[MAX_DMA_STREAMS] =
          { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7 };

        /* Array of all corresponding DMA stream interrupt numbers */
        static IRQn_Type AllStreamIrqNums[MAX_DMA_STREAMS] =
          { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn };

        /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
        DMA_HandleTypeDef   *sHandles    [MAX_DMA_STREAMS] = {0 };

        /* Array of "is in use"-flags for all available DMA channels */
        uint8_t             bDmaStreamUsed  [MAX_DMA_STREAMS] = {0 };

        #if USE_BDMA > 0
            /* Array of all BDMA channels for core CM7 */
            static BDMA_Channel_TypeDef* AllBdmaChannels[MAX_BDMA_CHANNELS] =
              { BDMA_Channel0, BDMA_Channel1, BDMA_Channel2, BDMA_Channel3 };

            /* Array of all corresponding BDMA channel interrupt numbers */
            static IRQn_Type AllBdmaChannelIrqNums[MAX_BDMA_CHANNELS] =
              { BDMA_Channel0_IRQn, BDMA_Channel1_IRQn, BDMA_Channel2_IRQn, BDMA_Channel3_IRQn };

            /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
            DMA_HandleTypeDef   *bHandles    [MAX_BDMA_CHANNELS] = {0 };

            /* Array of "is in use"-flags for all available DMA channels */
            uint8_t             bBdmaChUsed  [MAX_BDMA_CHANNELS] = {0 };
        #endif /* USE_BDMA */

    #else
        /* CM7 single core */
        #define MAX_DMA_STREAMS        16
        #define MAX_BDMA_CHANNELS       8

        /* Array of all DMA Streams for core CM7 */
        static DMA_Stream_TypeDef* AllDmaStreams[MAX_DMA_STREAMS] =
          { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7, 
            DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 };

        /* Array of all corresponding DMA stream interrupt numbers */
        static IRQn_Type AllStreamIrqNums[MAX_DMA_STREAMS] =
          { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
            DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn };

        /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
        DMA_HandleTypeDef   *sHandles    [MAX_DMA_STREAMS] = {0 };

        /* Array of "is in use"-flags for all available DMA channels */
        uint8_t             bDmaStreamUsed  [MAX_DMA_STREAMS] = {0 };

        #if USE_BDMA > 0
            /* Array of all BDMA channels for core CM7 */
            static BDMA_Channel_TypeDef* AllBdmaChannels[MAX_BDMA_CHANNELS] =
              { BDMA_Channel0, BDMA_Channel1, BDMA_Channel2, BDMA_Channel3, BDMA_Channel4, BDMA_Channel5, BDMA_Channel6, BDMA_Channel7 };

            /* Array of all corresponding BDMA channel interrupt numbers */
            static IRQn_Type AllBdmaChannelIrqNums[MAX_BDMA_CHANNELS] =
              { BDMA_Channel0_IRQn, BDMA_Channel1_IRQn, BDMA_Channel2_IRQn, BDMA_Channel3_IRQn, BDMA_Channel4_IRQn, BDMA_Channel5_IRQn, BDMA_Channel6_IRQn, BDMA_Channel7_IRQn };


            /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
            DMA_HandleTypeDef   *bHandles    [MAX_BDMA_CHANNELS] = {0 };

            /* Array of "is in use"-flags for all available DMA channels */
            uint8_t             bBdmaChUsed  [MAX_BDMA_CHANNELS] = {0 };
       #endif /* USE_BDMA */
    #endif

#elif defined(DUAL_CORE) && defined(CORE_CM4)
    #define MAX_DMA_STREAMS         8
    #define MAX_BDMA_CHANNELS       4

    /* Array of all DMA Streams for core CM4 */
    static DMA_Stream_TypeDef* AllDmaStreams[MAX_DMA_STREAMS] =
      { DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 };

    /* Array of all corresponding DMA stream interrupt numbers */
    static IRQn_Type AllStreamIrqNums[MAX_DMA_STREAMS] =
      { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn };

    /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
    DMA_HandleTypeDef   *sHandles    [MAX_DMA_STREAMS] = {0 };

    /* Array of "is in use"-flags for all available DMA channels */
    uint8_t             bDmaStreamUsed  [MAX_DMA_STREAMS] = {0 };

    #if USE_BDMA > 0
        /* Array of all BDMA channels for core CM4 */
        static BDMA_Channel_TypeDef* AllBdmaChannels[MAX_BDMA_CHANNELS] =
          { BDMA_Channel4, BDMA_Channel5, BDMA_Channel6, BDMA_Channel7 };

        /* Array of all corresponding DMA stream interrupt numbers */
        static IRQn_Type AllBdmaChannelIrqNums[MAX_BDMA_CHANNELS] =
          { BDMA_Channel4_IRQn, BDMA_Channel5_IRQn, BDMA_Channel6_IRQn, BDMA_Channel7_IRQn };

        /* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
        DMA_HandleTypeDef   *bHandles    [MAX_BDMA_CHANNELS] = {0 };

        /* Array of "is in use"-flags for all available DMA channels */
        uint8_t             bBdmaChUsed  [MAX_BDMA_CHANNELS] = {0 };
    #endif /* USE_BDMA */
#else
    #error "No DMA handler for unknown core"
#endif

/*******************************************************************************************************
 * Return the DMA Stream index for a given DMA Stream
 * -1 is returned in case of illegal DMA stream
 ******************************************************************************************************/
static int8_t GetDmaStreamIdx(DMA_Stream_TypeDef *ch )
{
    for ( uint32_t i = 0; i < MAX_DMA_STREAMS; i++ )
        if ( AllDmaStreams[i] == ch ) return (int8_t)i;

    DBG_ERROR("GetDmaStreamIdx: Illegal DMA-Stream @0x%p for DMA%d\n", ch );  
    return -1;
  
}

/*******************************************************************************************************
 * Return the DMA stream IRQ number for a given DMA stream
 * DMA_ILLEGAL_CHANNEL is returned in case of illegal DMA stream
 ******************************************************************************************************/
IRQn_Type HW_DMA_GetStreamIrqNum(DMA_Stream_TypeDef *stream)
{
    /* Get the stream index */
    int8_t DmaStreamIdx  = GetDmaStreamIdx(stream);
    if ( DmaStreamIdx < 0 ) return DMA_ILLEGAL_CHANNEL;

    return AllStreamIrqNums[DmaStreamIdx];
}

/*******************************************************************************************************
 * Find the next free DMA stream in case the statically specified stream is NULL
 * @param 
 *    none      
 * @retval 
 *    the first unused DMA stream, if any unused stream is left
 *    NULL in case of all DMA streams already in use
 * @note 
 *    if a free stream is found, it will NOT be allocated automatically ( by setting the used-bit )
 ******************************************************************************************************/
static DMA_Stream_TypeDef *GetFreeDmaStream(void)
{
    for ( uint32_t i = 0; i < MAX_DMA_STREAMS; i++ ) 
        if ( !bDmaStreamUsed[i]) return AllDmaStreams[i];
    
    DBG_ERROR("GetFreeDmaStream: No more free DMA streams\n");
    return NULL;
}

/*******************************************************************************************************
 * Enable the DMA hardware clock for a given BDMA channel or DMA stream 
 * and enable the DMAMUX clock, if implemented
 ******************************************************************************************************/
static void HW_DMA_SetDmaChClock ( void *channel)
{   
    
    #if defined ( BDMA )
        if ( IS_BDMA_CHANNEL_INSTANCE(channel) ) {
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
 * register one DMA stream for the DMA specified in dmadata                                 
 * @param 
 *    dmadata [in]  -  ptr to all neccessary DMA data: associated DMA_HandleTypeDef, DMA stream, DMA request
 *                     number, DMA priority. The DMA stream in dmadata may be NULL, in which case the first
 *                     unused DMA channel will be allocated.
 *    dmadata [out] -  dmadata->dmaHandle->Instance will receive the ( manually or auotmatically ) assigned
 *                     DMA stream on successful returns
 * @retval 
 *    DMA handle is returned in case of success,
 *    NULL in case of channel already in use
 ******************************************************************************************************/
static DMA_HandleTypeDef *HW_DMA_RegisterDMAStream (const HW_DmaType* dmadata)
{
    DMA_Stream_TypeDef *stream = dmadata->dmaStream;

    /* Check passed void ptr for being a valid stream */
    #if DEBUG_MODE > 0
        if(stream && !IS_DMA_STREAM_INSTANCE(stream)){
            DBG_ERROR("RegisterDMAStream: DMA Stream @0x%p illegal\n", stream);
            return NULL;
        }
    #endif

    /* if no channel is specified, find one */
    if ( !stream ) {
        stream = GetFreeDmaStream();
        if (!stream) return NULL;
    }
        
    /* get and check the Channel index */
    int8_t DmaStreamIdx  = GetDmaStreamIdx(stream);
    if ( DmaStreamIdx < 0 ) return NULL;

    /* check for stream being available */
    if ( bDmaStreamUsed[DmaStreamIdx] ) {
        DBG_ERROR("RegisterDMAChannel: DMA Channel %d already in use\n", DmaStreamIdx);  
        return NULL;
    }

    /* register Channel*/ 
    sHandles[DmaStreamIdx]  = dmadata->dmaHandle;

    /* mark as used in any case */
    bDmaStreamUsed[DmaStreamIdx]   = true;

    /* store the assigned stream in handle data */
    dmadata->dmaHandle->Instance = stream;

    /* Enable the DMA ( and DMAMUX ) clock */
    HW_DMA_SetDmaChClock(stream);
    
    return dmadata->dmaHandle;
}

#if USE_BDMA > 0
    /*******************************************************************************************************
     * Return the BDMA channel index for a given BDMA Channel
     * -1 is returned in case of illegalB DMA channel
     ******************************************************************************************************/
    static int8_t GetBdmaChannelIdx(BDMA_Channel_TypeDef *ch )
    {
        for ( uint32_t i = 0; i < MAX_BDMA_CHANNELS; i++ )
            if ( AllBdmaChannels[i] == ch ) return (int8_t)i;

        DBG_ERROR("GetBdmaChannelIdx: Illegal BDMA-Channel @0x%p for DMA%d\n", ch );  
        return -1;
    }

    /*******************************************************************************************************
     * Return the BDMA channel IRQ number for a given BDMA channel
     * DMA_ILLEGAL_CHANNEL is returned in case of illegal BDMA channel
     ******************************************************************************************************/
    IRQn_Type HW_DMA_GetBdmaChannelIrqNum(BDMA_Channel_TypeDef *channel)
    {
        /* Get the channel index */
        int8_t BdmaChannelIdx  = GetBdmaChannelIdx(channel);
        if ( BdmaChannelIdx < 0 ) return DMA_ILLEGAL_CHANNEL;

        return AllBdmaChannelIrqNums[BdmaChannelIdx];
    }

    /*******************************************************************************************************
     * Find the next free BDMA channel in case the statically specified channel is NULL
     * @param 
     *    none      
     * @retval 
     *    the first unused BDMA channel, if any unused channel is left
     *    NULL in case of all BDMA channels already in use
     * @note 
     *    if a free channel is found, it will NOT be allocated automatically ( by setting the used-bit )
     ******************************************************************************************************/
    static BDMA_Channel_TypeDef *GetFreeBdmaChannel(void)
    {
        for ( uint32_t i = 0; i < MAX_BDMA_CHANNELS; i++ ) 
            if ( !bBdmaChUsed[i]) return AllBdmaChannels[i];
    
        DBG_ERROR("GetFreeBdmaChannel: No more free BDMA channeld\n");
        return NULL;
    }

/*******************************************************************************************************
 * register one BDMA channel for the BDMA channel specified in dmadata                                 
 * @param 
 *    dmadata [in]  -  ptr to all neccessary DMA data: associated DMA_HandleTypeDef, BDMA channel, BDMA request
 *                     number, DMA priority. The BDMA channel in dmadata may be NULL, in which case the first
 *                     unused BDMA channel will be allocated.
 *    dmadata [out] -  dmadata->dmaHandle->Instance will receive the ( manually or auotmatically ) assigned
 *                     BDMA channel on successful returns
 * @retval 
 *    DMA handle is returned in case of success,
 *    NULL in case of channel already in use
 ******************************************************************************************************/
static DMA_HandleTypeDef *HW_DMA_RegisterBDMAChannel (const HW_DmaType* dmadata)
{
    BDMA_Channel_TypeDef *channel = dmadata->dmaStream;

    /* Check passed void ptr for being a valid BDMA handle whein not NULL */
    #if DEBUG_MODE > 0
        if(channel && !IS_BDMA_CHANNEL_INSTANCE(channel)){
            DBG_ERROR("RegisterBDMAChannel: BDMA channel @0x%p illegal\n", channel);
            return NULL;
        }
    #endif

    /* if no channel is specified, find one */
    if ( !channel ) {
        channel = GetFreeBdmaChannel();
        if (!channel) return NULL;
    }
        
    /* get and check the Channel index */
    int8_t BdmaChannelIdx  = GetBdmaChannelIdx(channel);
    if ( BdmaChannelIdx < 0 ) return NULL;

    /* check for channel being available */
    if ( bBdmaChUsed[BdmaChannelIdx] ) {
        DBG_ERROR("RegisterBDMAChannel: BDMA Channel %d already in use\n", BdmaChannelIdx);  
        return NULL;
    }

    /* register Channel*/ 
    bHandles[BdmaChannelIdx]      = dmadata->dmaHandle;

    /* mark as used in any case */
    bBdmaChUsed[BdmaChannelIdx]   = true;

    /* store the assigned channel in handle data */
    dmadata->dmaHandle->Instance  = channel;

    /* Enable the DMA ( and DMAMUX ) clock */
    HW_DMA_SetDmaChClock(channel);
    
    return dmadata->dmaHandle;
}

#endif /* USE_BDMA */


/*******************************************************************************************************
 * register one DMA stream or BDMA channel for the DMA specified in dmadata                                 
 * @param 
 *    dmadata [in]  -  ptr to all neccessary DMA data: associated DMA_HandleTypeDef, DMA stream or BDMA channel, 
 *                     (B)DMA request number, DMA priority. 
 *                     The DMA stream / BDMA channel in dmadata may be NULL, in which case the first
 *                     unused DMA stream / BDMA channel will be allocated.
 *    dmadata [out] -  dmadata->dmaHandle->Instance will receive the ( manually or auotmatically ) assigned
 *                     DMA stream/BDMA channel on successful returns
 * @retval 
 *    DMA handle is returned in case of success,
 *    NULL in case of channel already in use
 ******************************************************************************************************/
DMA_HandleTypeDef *HW_DMA_RegisterDMAChannel (const HW_DmaType* dmadata)
{
    DMA_HandleTypeDef *ret;

    switch ( dmadata->dmaInstance ) 
    {
        case HW_DMA_STREAM:
            ret = HW_DMA_RegisterDMAStream (dmadata);
            break;
        #if USE_BDMA > 0
            case HW_BDMA_CHANNEL:
                ret = HW_DMA_RegisterBDMAChannel (dmadata);
                break;
        #endif
        default:
            DBG_ERROR("RegisterDMAChannel: Unknown DMA Instance %d\n", dmadata->dmaInstance);  
            ret = NULL;
    }

    return ret;
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
 * Set the IRQ priority for a given DMA stream or BDMA channel and then 
 * enable the DMA stream / BDMA channel interrupt
 *****************************************************************************/
void HW_DMA_SetAndEnableChannelIrq(void *channel, uint8_t prio, uint8_t subprio)
{
    IRQn_Type irq;

    if ( IS_BDMA_CHANNEL_INSTANCE(channel) ) {
        #if USE_BDMA > 0
            irq = HW_DMA_GetBdmaChannelIrqNum(channel);
        #else
            DBG_ERROR("HW_DMA_SetAndEnableChannelIrq: Not configured for BDMA!");
        #endif /* USE_BMDA */
    } else if ( IS_DMA_STREAM_INSTANCE(channel) ) {
        irq = HW_DMA_GetStreamIrqNum(channel);
    } else {
        DBG_ERROR("HW_DMA_SetAndEnableChannelIrq: @0x%p is no vaild channel\n", channel);
        return;
    }

    HAL_NVIC_SetPriority(irq, prio, subprio);
    HAL_NVIC_EnableIRQ(irq);
}

/******************************************************************************
 * Invalidates a dma channel: Disable DMA-channel interrupt, DeInit Handle,
 * mark channel as "unused"
 *****************************************************************************/
void HW_DMA_HandleDeInit(DMA_HandleTypeDef *hdma)
{
    IRQn_Type irq;
    void *channel = hdma->Instance;

    /* disable DMA channel interrupt */
   if ( IS_BDMA_CHANNEL_INSTANCE(channel) ) {
        #if USE_BDMA > 0
            irq = HW_DMA_GetBdmaChannelIrqNum(channel);
            /* get and check the Channel index ( 0 .. 6 ) */
            int8_t BdmaChannelIdx  = GetBdmaChannelIdx(channel);

            /* unregister Channel and mark as unused */
            bHandles[BdmaChannelIdx]      = NULL;
            bBdmaChUsed[BdmaChannelIdx]   = false;
        #else
            DBG_ERROR("HW_DMA_HandleDeInit: Not configured for BDMA!");
        #endif /* USE_BMDA */
    } else if ( IS_DMA_STREAM_INSTANCE(channel) ) {
        irq = HW_DMA_GetStreamIrqNum(channel);
        /* get and check the Channel index ( 0 .. 6 ) */
        int8_t DmaStreamIdx  = GetDmaStreamIdx(channel);

        /* unregister Channel and mark as unused */
        sHandles[DmaStreamIdx]      = NULL;
        bDmaStreamUsed[DmaStreamIdx]   = false;
    } else {
        DBG_ERROR("HW_DMA_HandleDeInit: DMA_handle @0x%p contains no vaild channel\n", channel);
        return;
    }

    HAL_NVIC_DisableIRQ(irq);
    HAL_DMA_DeInit(hdma);

}

/*******************************************************************************************************
 * @brief  Generate the DMA IRQ handlers for all DMA channel by macro
 * @param  dev     - 1                      for DMA1, 2 for DMA2
 * @param  channel - 0..7                   for DMAx_Channel<channel>
 * @param  idx     - 0..MAX_DMA_STREAMS-1  corresponding index in sHandles[] array
 *
 ******************************************************************************************************/
#define DMA_IRQ(dev, channel,idx)                                  \
void DMA##dev##_Stream##channel##_IRQHandler(void)                 \
{                                                                  \
  ProfilerPush(JOB_IRQ_DMA);                                       \
  /* find the assigned handle, if any */                           \
  DMA_HandleTypeDef *handle = sHandles[idx];                        \
  if ( !handle) {                                                  \
    DBG_ERROR("DMA %d, Stream %d: No handler!\n", dev, channel);   \
    return;                                                        \
  }                                                                \
  HAL_DMA_IRQHandler(handle);                                      \
  ProfilerPop();                                                   \
}

#if USE_BDMA > 0
    #define BDMA_IRQ(channel,idx)                                      \
    void BDMA_Channel##channel##_IRQHandler(void)                      \
    {                                                                  \
      ProfilerPush(JOB_IRQ_DMA);                                       \
      /* find the assigned handle, if any */                           \
      DMA_HandleTypeDef *handle = bHandles[idx];                       \
      if ( !handle) {                                                  \
        DBG_ERROR("BDMA Stream %d: No handler!\n", channel);           \
        return;                                                        \
      }                                                                \
      HAL_DMA_IRQHandler(handle);                                      \
      ProfilerPop();                                                   \
    }
#else
    #define BDMA_IRQ(channel,idx)
#endif /* USE_BDMA */

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
        BDMA_IRQ(0, 0)
        BDMA_IRQ(1, 1)
        BDMA_IRQ(2, 2)
        BDMA_IRQ(3, 3)
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
        BDMA_IRQ(0, 0)
        BDMA_IRQ(1, 1)
        BDMA_IRQ(2, 2)
        BDMA_IRQ(3, 3)
        BDMA_IRQ(4, 4)
        BDMA_IRQ(5, 5)
        BDMA_IRQ(6, 6)
        BDMA_IRQ(7, 7)
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
    BDMA_IRQ(4, 0)
    BDMA_IRQ(5, 1)
    BDMA_IRQ(6, 2)
    BDMA_IRQ(7, 3)
#endif

#if 0 

/* For debug and test purposes */

void DMA1_Stream4_IRQHandler(void) 
{
  ProfilerPush(JOB_IRQ_DMA);
  /* find the assigned handle, if any */
  DMA_HandleTypeDef *handle = sHandles[4];
  if ( !handle) {
    DBG_ERROR("DMA %d, Stream %d: No handler!\n", 1, 4);
    return;
  }
  HAL_DMA_IRQHandler(handle);
  ProfilerPop();
}

void BDMA_Channel4_IRQHandler(void)
{
  ProfilerPush(JOB_IRQ_DMA);
  /* find the assigned handle, if any */
  DMA_HandleTypeDef *handle = bHandles[0];
  if ( !handle) {
    DBG_ERROR("BDMA Stream %d: No handler!\n", channel);
    return;
  } 
  HAL_DMA_IRQHandler(handle);
  ProfilerPop();
}
#endif

/**
  * @}
  */

