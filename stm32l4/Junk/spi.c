/**
  ******************************************************************************
  * @file    spi.h
  * @author  Rainer 
  * @brief   Wrap the SPI-Hardware into Devices
  *
  * 
  ******************************************************************************
  */

#include "config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "hw_util.h"

#include "dev/spi.h"
#include "devices_config.h"
#include "dev/devices.h"
#include "system.h"

#include "debug_helper.h"



/*
/
/**
  * @brief DMA UART transmit process complete callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hSpi = &((SpiHandleT *)(hdma->Parent))->hSpi;
  
  /* DMA Normal mode */
  // debug_putchar('x');
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
  SPI_HandleTypeDef *hSpi = &((SpiHandleT *)(hdma->Parent))->hSpi;

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
  SpiHandleT *uhandle = (SpiHandleT *)(hdma->Parent);
  SPI_HandleTypeDef *hSpi = &uhandle->hSpi;
    
  /* Stop UART DMA Tx request if ongoing */
  if (HAL_IS_BIT_SET(hSpi->Instance->CR2, SPI_CR2_TXDMAEN) ) {
    SpiStopTx(hSpi);
  }
  
  /* Stop UART DMA Rx request if ongoing */
  if ( HAL_IS_BIT_SET(hSpi->Instance->CR2, SPI_CR2_RXDMAEN) ) {
    SpiStopRx(hSpi);
  }
  
  if ( uhandle->OnError ) {
    uhandle->last_errors |= HAL_SPI_ERROR_DMA;
    uhandle->OnError(uhandle);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  __HAL_SPI_DISABLE(hspi);
  //RHB todo  
 
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  __HAL_SPI_DISABLE(hspi);
  //RHB todo  
 
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  __HAL_SPI_DISABLE(hspi);
  //RHB todo  
 
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  __HAL_SPI_DISABLE(hspi);
  //RHB todo  
 
}


static void SpiResetMyHandle ( SpiHandleT *shandle ) 
{
    memset(shandle, 0, sizeof(SpiHandleT) );
}


