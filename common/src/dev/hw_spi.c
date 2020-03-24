/**
  ******************************************************************************
  * @file    hw_spi.h
  * @author  Rainer 
  * @brief   Impelemtation of SPI functionality on Hardware SPI basis
  *
  * @note    HW-SPI, when not used in DMA mode, has great speed disadvantages
  *          compared to bit bang SPI. So, when using SPI in polling mode,
  *          choose bit bang SPI instead. Generally avoid HW SPI in IRQ mode, the
  *          overhead added by HAL layer is unbelievable. 
  *          HW SPI with DMA Mode is ok
  *          
  *          The minimum clock frequency for HW SPI in polling or IRQ mode is 16MHz
  *          at lower clock frequencies, some SPI bytes may be lost.
  * 
  ******************************************************************************
  */

#include "config/devices_config.h"

#if defined(USE_BBSPI1) || defined(USE_BBSPI2) || defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4) 

#include "dev/spi.h"

#if defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4)

#include "error.h"

#include "system/profiling.h"
#include "task/minitask.h"

#include "dev/hw_device.h"
#include "system/hw_util.h"

#include "dev/devices.h"


#include "debug_helper.h"


/*******************************************************************************************
 * Additional data that will be stored to SPI type hardware devices
 ******************************************************************************************/
  
#define SPIx_TIMEOUT_MAX                   3000

typedef enum SpiDmaDirectionEnum {
  SPI_DMA_RX=0,                       // Rx DMA
  SPI_DMA_TX,                         // Tx DMA
} SpiDmaDirectionEnumType;

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for SPI interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */

/* Forward declarations of hardware SPI Callbacks */
void OnError          (SPI_HandleTypeDef *hspi);
void OnTxComplete     (SPI_HandleTypeDef *hspi);
void OnTxRxComplete   (SPI_HandleTypeDef *hspi);



/*----------------------------------------------------------------------
 * Compute the baudrate prescaler in a way, so that the nearest frequency
 * to the desired frequency is achieved
 *--------------------------------------------------------------------*/
static uint32_t HwSpiGetBRPrescaler(SPI_TypeDef *hspi, uint32_t baudrate )
{
    #define ABS_RETURN(x)                             (((x) < 0) ? -(x) : (x))
    
    uint32_t i;
    uint32_t pclk;
    uint32_t frq_error = UINT32_MAX;
    uint32_t curr_error;
    uint32_t min_idx = 0xff;

    /* Get the bus clock, it depends from the SPI device */
    pclk = ( (uint32_t)hspi == (uint32_t)SPI1 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq() );

    for ( i=0; i < 8; i++ ) {
        curr_error = ABS_RETURN(   (int32_t)(baudrate -( pclk>>(i+1) )));
        if ( curr_error < frq_error ) {
            frq_error = curr_error;
            min_idx = i;
        }
    }

    if ( min_idx == 0xff ) DEBUG_PUTS("Error: No vaild SPI prescaler found");

    DEBUG_PRINTF("SPI prescaler=%d, resulting in baudrate %d\n", 2 << min_idx, pclk >> ( min_idx+1) ); 
    
    return min_idx;
}  

static bool HwSpiSetDefaultParams(SpiDataT *data )
{
  SpiDataHW *hndhw = &data->hw;
  SPI_TypeDef *spi    = (SPI_TypeDef*)data->mySpiDev->devBase;
  SPI_HandleTypeDef* hspi = &hndhw->myHalHandle;

  uint32_t prescaler = HwSpiGetBRPrescaler(spi, hndhw->myBaudrate);

  /* Set the SPI parameters */
  hspi->Instance               = spi;
  hspi->Init.BaudRatePrescaler = prescaler << SPI_CR1_BR_Pos;
  hspi->Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi->Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi->Init.DataSize          = ((uint16_t)data->datasize - 1) << 8;
  hspi->Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial     = 7;
  if ( !hndhw->use_nss ) 
        hspi->Init.NSS               = SPI_NSS_SOFT;

  /* if MISO is not used, then configure only for output */
  hspi->Init.Direction =  data->use_miso ? SPI_DIRECTION_2LINES : SPI_DIRECTION_1LINE ;

  if (  data->bIsMaster ) {
    hspi->Init.Mode = SPI_MODE_MASTER;
    if ( hndhw->use_nss ) {
        hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;
    }
  } else {    
    hspi->Init.Mode = SPI_MODE_SLAVE;
    if ( hndhw->use_nss ) hspi->Init.NSS = SPI_NSS_HARD_INPUT;
  }
  if(HAL_SPI_Init(hspi) != HAL_OK)
  {
    /* Initialization Error */
    #if DEBUG_MODE > 0
        DEBUG_PRINTF("%s could not be initialized\n", data->mySpiDev->devName);
    #endif
    return false;
  }

  return true;
}

static void HwSpiSetTxDMAMemInc(SPI_HandleTypeDef *hspi, bool bDoIncrement )
{
    assert(hspi->hdmatx);
    /* Must disable DMA in order to change parameters */
    __HAL_DMA_DISABLE(hspi->hdmatx);

    
    hspi->hdmatx->Init.MemInc = ( bDoIncrement ? DMA_MINC_ENABLE : DMA_MINC_DISABLE );
    /* Reconfigure */
    HAL_DMA_Init(hspi->hdmatx);
    
    /* Reenable */
    __HAL_DMA_ENABLE(hspi->hdmatx);
}

/**
  * @brief DMA UART transmit process complete callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hSpi = &((SpiDataT *)(hdma->Parent))->hw.myHalHandle;
  
  /* DMA Normal mode */
  //debug_putchar('x');
  if ( HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC) ) {  
    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
    in the UART CR3 register */
    CLEAR_BIT(hSpi->Instance->CR2, SPI_CR2_TXDMAEN);
    
    /* Enable the UART Transmit Complete Interrupt */
    SET_BIT(hSpi->Instance->CR1, SPI_CR2_TXEIE);
  } else {
    /* DMA Circular mode, not implemented */
    Error_Handler(__FILE__, __LINE__);
  }
}

/**
  * @brief DMA UART receive process complete callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hSpi = &((SpiDataT *)(hdma->Parent))->hw.myHalHandle;

  /* DMA Normal mode */
  if ( HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC) )
  {
    
    /* Disable ERR interrupts */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_RXNEIE);
    //CLEAR_BIT(hSpi->Instance->CR3, SPI_CR2_ERRIE);
    
    /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
       in the UART CR3 register */
    CLEAR_BIT(hSpi->Instance->CR2, SPI_CR2_RXDMAEN);
    
  }
  
  // RHB todo: User-callback on receive
}

void SpiStopTx(SPI_HandleTypeDef *hSpi)
{
  CLEAR_BIT(hSpi->Instance->CR2, SPI_CR2_TXEIE);
}

void SpiStopRx(SPI_HandleTypeDef *hSpi)
{
  CLEAR_BIT(hSpi->Instance->CR2, (SPI_CR2_RXNEIE | SPI_CR2_ERRIE));
}

/**
  * @brief DMA UART communication error callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SpiDataT *data = (SpiDataT *)(hdma->Parent);
  SPI_HandleTypeDef *hSpi = &data->hw.myHalHandle;
    
  /* Stop UART DMA Tx request if ongoing */
  if (HAL_IS_BIT_SET(hSpi->Instance->CR2, SPI_CR2_TXDMAEN) ) {
    SpiStopTx(hSpi);
  }
  
  /* Stop UART DMA Rx request if ongoing */
  if ( HAL_IS_BIT_SET(hSpi->Instance->CR2, SPI_CR2_RXDMAEN) ) {
    SpiStopRx(hSpi);
  }
  
  /* RHB todo
  if ( uhandle->OnError ) {
    uhandle->last_errors |= HAL_SPI_ERROR_DMA;
    uhandle->OnError(uhandle);
  }
  */
}

static void HwSpiDmaChannelInit(SpiDataT *myspi, const HW_DmaType *dma, SpiDmaDirectionEnumType dmadir )
{
  DMA_HandleTypeDef *hdma = dma->dmaHandle;

  hdma->Instance                 = dma->dmaChannel;
  hdma->Parent                   = &myspi->hw.myHalHandle;
  hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma->Init.MemInc              = DMA_MINC_ENABLE;
  if ( myspi->datasize < 9 ) {
      hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  } else {      
      hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  }
  hdma->Init.Mode                = DMA_NORMAL;
  hdma->Init.Priority            = DMA_PRIORITY_MEDIUM;
  hdma->Init.Request             = dma->dmaRequest;

  switch ( dmadir ) 
  {
    case SPI_DMA_RX:
      hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
      myspi->hw.myHalHandle.hdmarx = dma->dmaHandle;
      break;
    case SPI_DMA_TX:
      hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      myspi->hw.myHalHandle.hdmatx = dma->dmaHandle;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
  }

  HAL_DMA_Init(hdma);

  /* 
   * All callbacks have been set to NULL by Init, 
   * now set Callbacks for DMA complete and DMA error 
   */

  switch ( dmadir ) 
  {
    case SPI_DMA_RX:
      hdma->XferCpltCallback        = SPI_DMAReceiveCplt;
      hdma->XferErrorCallback       = SPI_DMAError;
      break;
    case SPI_DMA_TX:
      hdma->XferCpltCallback        = SPI_DMATransmitCplt;   
      hdma->XferErrorCallback       = SPI_DMAError;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
  }

  return;
}



bool HwSpi_DefaultInit(SpiDataT *data, const HW_DeviceType *SpiDev)
{
 
    /* Initialize handle and communication parameters */
    if ( !HwSpiSetDefaultParams( data ) ) return false;

    /* Register interrupt callbacks, if interrupt is used */
    if ( data->hw.use_hw_irq ) {
       //  HAL_SPI_RegisterCallback(&data->hw.myHalHandle, HAL_SPI_TX_COMPLETE_CB_ID,    OnTxComplete );
       //  HAL_SPI_RegisterCallback(&data->hw.myHalHandle, HAL_SPI_TX_RX_COMPLETE_CB_ID, OnTxRxComplete );
       //  HAL_SPI_RegisterCallback(&data->hw.myHalHandle, HAL_SPI_ERROR_CB_ID,          OnError );

    }

    /* Enable DMA, if specified */            
    if ( SpiDev->devDmaRx || SpiDev->devDmaTx ) {

      /* Make sure, we have interrupts where we can copy the prio from */
      if ( SpiDev->devIrqList->num == 0 ) {
        DEBUG_PUTS("Error in HwSpi DMA Init: No IRQs defined!");
        return false;
      }
        
      HW_SetDmaChClock(SpiDev->devDmaTx, SpiDev->devDmaRx);

      const HW_IrqType *irq = SpiDev->devIrqList->irq;
      const HW_DmaType *dma;
      dma = SpiDev->devDmaRx;
      if ( dma ) {
         HwSpiDmaChannelInit( data, dma, SPI_DMA_RX );
         HAL_NVIC_SetPriority(dma->dmaIrqNum, irq->irq_prio, irq->irq_subprio);
         HAL_NVIC_EnableIRQ(dma->dmaIrqNum);
      }
      
      dma = SpiDev->devDmaTx;
      if (dma ) {
         HwSpiDmaChannelInit( data ,dma, SPI_DMA_TX );
         HAL_NVIC_SetPriority(dma->dmaIrqNum, irq->irq_prio, irq->irq_subprio);
         HAL_NVIC_EnableIRQ(dma->dmaIrqNum);
      }
    } // if DMA

    return true;
}

static SpiDataT *GetMyData ( SPI_HandleTypeDef *hspi ) 
{
    const HW_DeviceType *dev = FindDevByBaseAddr(HW_DEVICE_HWSPI, hspi->Instance);
    if ( !dev ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("Cannot find SpiHandle!");
        #endif
        return NULL;
    } else {
        return SPI_GetHandleFromDev(dev)->data;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SpiDataT *data = GetMyData(hspi);
    DEBUG_PUTS("SPI Error");
    if ( data->hw.OnError ) data->hw.OnError();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SpiDataT *data = GetMyData(hspi);
    if ( data->hw.OnTxRxComplete ) data->hw.OnTxRxComplete();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SpiDataT *data = GetMyData(hspi);
    if ( data->hw.OnTxComplete ) data->hw.OnTxComplete();
}

/*******************************************************************************
 *  Hardware 8-bit SPI 
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 ******************************************************************************/
uint8_t Spi8TxRxByte_hw(SpiHandleT *self, uint8_t outval)
{
    uint8_t ret;
    
    HAL_SPI_TransmitReceive(&self->data->hw.myHalHandle, &outval, &ret, 1, SPIx_TIMEOUT_MAX);
    return ret;
}

/*******************************************************************************
 *  Bitbanged 16-bit SPI 
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 ******************************************************************************/
uint16_t Spi16TxRx_hw(SpiHandleT *self, uint16_t outval)
{
    uint8_t buf[2], retbuf[2];
    buf[0] = outval >> 8;
    buf[1] = (uint8_t)outval;

    HAL_SPI_TransmitReceive(&self->data->hw.myHalHandle, buf, retbuf, 2, SPIx_TIMEOUT_MAX);
    return ((uint16_t)retbuf[0]) << 8 | retbuf[1];
}

void Spi8TxByte_hw(SpiHandleT *self, uint8_t outval)
{
    HAL_SPI_Transmit(&self->data->hw.myHalHandle, &outval, 1, SPIx_TIMEOUT_MAX);
}

void Spi8TxVector_hw(SpiHandleT *self, uint8_t *vector, uint16_t size)
{
    HAL_SPI_Transmit(&self->data->hw.myHalHandle, vector, size, SPIx_TIMEOUT_MAX);
}

void Spi9TxByte_hw(SpiHandleT *self, uint16_t outval)
{
    /* 
     * Caution: The passed value is regarded as uint16_t value, although it is passed as uint8_t*
     * and size means 'number of uint16_t values' , not number of bytes !
     */
    HAL_SPI_Transmit(&self->data->hw.myHalHandle, (uint8_t*)&outval, 1, SPIx_TIMEOUT_MAX);
}

void Spi9TxVector_hw(SpiHandleT *self, uint16_t *vector, uint16_t size)
{
    /* 
     * Caution: The passed vector is regarded as uint16_t vector, although it is passed as uint8_t*
     * and size means 'number of uint16_t values' , not number of bytes !
     */
    HAL_SPI_Transmit(&self->data->hw.myHalHandle, (uint8_t*)vector, size, SPIx_TIMEOUT_MAX);
}

void Spi9TxConstant_hw(SpiHandleT *self, uint16_t value, uint16_t size)
{
    for ( uint16_t i = 0; i < size; i++ )
        Spi9TxByte(self, value);
}


void Spi9TxVector_IT_hw(SpiHandleT *self, uint16_t *vector, uint16_t size)
{
    /* 
     * Caution: The passed vector is regarded as uint16_t vector, although it is passed as uint8_t*
     * and size means 'number of uint16_t values' , not number of bytes !
     */
    HAL_SPI_Transmit_IT(&self->data->hw.myHalHandle, (uint8_t*)vector, size );
}

void Spi9TxVector_DMA_hw(SpiHandleT *self, uint16_t *vector, uint16_t size)
{
    /* 
     * Caution: The passed vector is regarded as uint16_t vector, although it is passed as uint8_t*
     * and size means 'number of uint16_t values' , not number of bytes !
     */
    HwSpiSetTxDMAMemInc(&self->data->hw.myHalHandle, true );
    HAL_SPI_Transmit_DMA(&self->data->hw.myHalHandle, (uint8_t*)vector, size );
}


/* 
 * value will be removed from stack upon return. As DMA works asynchronously,
 * we need to preserve this value 
 */
static uint16_t preserve_value;

void Spi9TxConstant_DMA_hw(SpiHandleT *self, uint16_t value, uint16_t size)
{
    preserve_value = value;

    HwSpiSetTxDMAMemInc(&self->data->hw.myHalHandle, false );

    // Testing
    SpiNSelHigh(self);
    SpiNSelLow(self);

    HAL_SPI_Transmit_DMA(&self->data->hw.myHalHandle, (uint8_t*)&preserve_value, size );
}


/*******************************************************************************
 *  Bitbanged 8-bit SPI with Interrupt
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 ******************************************************************************/
uint8_t Spi8TxRxByte_IT_hw(SpiHandleT *self, uint8_t outval)
{
    uint8_t ret;
    
    HAL_SPI_TransmitReceive_IT(&self->data->hw.myHalHandle, &outval, &ret, 1);
    return ret;
}

void Spi8TxVector_IT_hw(SpiHandleT *self, uint8_t *vector, uint16_t size)
{
        HAL_SPI_Transmit_IT(&self->data->hw.myHalHandle, vector, size);
}


const SpiFunctionT SpiFns_hw = {
   .Spi16TxRx          = Spi16TxRx_hw,
   .Spi9TxByte         = Spi9TxByte_hw,
   .Spi8TxByte         = Spi8TxByte_hw,
   .Spi8TxRxByte       = Spi8TxRxByte_hw,
   .Spi8TxVector       = Spi8TxVector_hw,
   .Spi8TxVector_IT    = Spi8TxVector_IT_hw,
   .Spi9TxVector       = Spi9TxVector_hw,
   .Spi9TxConstant     = Spi9TxConstant_hw,
   .Spi9TxVector_IT    = Spi9TxVector_IT_hw,
   .Spi9TxVector_DMA   = Spi9TxVector_DMA_hw,
   .Spi9TxConstant_DMA = Spi9TxConstant_DMA_hw,
};
#else
    const SpiFunctionT SpiFns_hw = { 0 };
#endif // defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3)

#endif /* #if defined(USE_BBSPI1) || defined(USE_BBSPI2) */


