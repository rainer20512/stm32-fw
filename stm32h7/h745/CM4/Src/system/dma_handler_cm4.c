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
#include "msg_direct.h"


#define DMA_INSTANCES       2
#define DMA_CHANNELS        8
#define BDMA_INSTANCES      1
#define BDMA_CHANNELS       8


/* Array of all DMA Streams */
static DMA_Stream_TypeDef* AllChannels[DMA_INSTANCES][DMA_CHANNELS] =
{
  { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7 },
  { DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 }
};

/* Array of all corresponding DMA stream interrupt numbers */
static IRQn_Type AllChannelIrqNums[DMA_INSTANCES][DMA_CHANNELS] =
{
  { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn },
  { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn }
};

/* Array of DMA handles for all DMA channels, initialzed with NULL, set to any handle when used */
DMA_HandleTypeDef   *handles    [DMA_INSTANCES][DMA_CHANNELS] = {0 };


/*******************************************************************************************************
 * Return the DMA channel index for a given DMA Channel
 * -1 is returned in case of illegal DMA channel
 ******************************************************************************************************/
static int8_t GetDmaChannelIdx(DMA_Stream_TypeDef *ch, uint8_t DmaInstanceIdx )
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
static DMA_TypeDef *GetDmaInstance(DMA_Stream_TypeDef *ch )
{

    if ( ch >= DMA1_Stream0 && ch <= DMA1_Stream7 )     return DMA1;
    if ( ch >= DMA2_Stream0 && ch <= DMA2_Stream7 )     return DMA2;
    DBG_ERROR("GetDmaInstance: Illegal DMA-Stream %p\n", ch);  
    return NULL;
}

/*******************************************************************************************************
 * Return the DMA channel IRQ number for a given DMA channel
 * DMA_ILLEGAL_CHANNEL
 ******************************************************************************************************/
IRQn_Type HW_DMA_GetChannelIrqNum(DMA_Stream_TypeDef *channel)
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
 * @Note    The CM4 implementation is only a stub for a remote registartion on CM7 core
 ******************************************************************************************************/
DMA_HandleTypeDef *HW_DMA_RegisterDMAChannel (const HW_DmaType* dmadata )
{
    DMA_HandleTypeDef *ret;
    
    /* Do remote registration */
    MSGD_DoRemoteDmaRegistration(dmadata, MSGTYPE6_IS_DMA);
    ret = MSGD_WaitForRemoteDmaRegisterOp();

    /* Save the handle in the array of local handles */
    if ( ret ) {
        /* get and check associated DMA instance */
        DMA_TypeDef *dma    = GetDmaInstance(ret->Instance);
        uint8_t DmaInstanceIdx = dma == DMA1 ? 0 : 1;
    
        /* get and check the Channel index ( 0 .. 6 ) */
        int8_t DmaChannelIdx  = GetDmaChannelIdx(ret->Instance, DmaInstanceIdx);
        handles[DmaInstanceIdx][DmaChannelIdx]  = dmadata->dmaHandle;

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
    DMA_Stream_TypeDef *channel = hdma->Instance;
    IRQn_Type irq = HW_DMA_GetChannelIrqNum(channel);
    HAL_NVIC_DisableIRQ(irq);
    HAL_DMA_DeInit(hdma);

     /* Do remote unregistration */
    MSGD_DoRemoteDmaUnRegistration(hdma, MSGTYPE6_IS_DMA);
    MSGD_WaitForRemoteDmaRegisterOp();
}

/*******************************************************************************************************
 * @brief  Generate the DMA IRQ handlers for DMA1 and 2 and channel 0 .. 7 by Macro
 * @param  None
 * @retval None
 ******************************************************************************************************/
#define DMA_IRQ(dev, channel)               \
void DMA##dev##_Stream##channel##_IRQHandler(void)                 \
{                                                                  \
  ProfilerPush(JOB_IRQ_DMA);                                       \
  /* find the assigned handle, if any */                           \
  DMA_HandleTypeDef *handle = handles[dev-1][channel];             \
  if ( !handle) {                                                  \
    DBG_ERROR("DMA %d, Stream %d: No handler!\n", dev, channel);  \
    return;                                                        \
  }                                                                \
  HAL_DMA_IRQHandler(handle);                                      \
  ProfilerPop();                                                   \
}

DMA_IRQ(1, 0)
DMA_IRQ(1, 1)
DMA_IRQ(1, 2)
DMA_IRQ(1, 3)
DMA_IRQ(1, 4)
DMA_IRQ(1, 5)
DMA_IRQ(1, 6)
DMA_IRQ(1, 7)

DMA_IRQ(2, 0)
DMA_IRQ(2, 1)
DMA_IRQ(2, 2)
DMA_IRQ(2, 3)
DMA_IRQ(2, 4)
DMA_IRQ(2, 5)
DMA_IRQ(2, 6)
DMA_IRQ(2, 7)

#if 0
void DMA1_Stream4_IRQHandler(void) 
{
  ProfilerPush(JOB_IRQ_DMA);
  /* find the assigned handle, if any */
  DMA_HandleTypeDef *handle = handles[0][4];
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

