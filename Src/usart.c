/**
  ******************************************************************************
  * @file    usart.c
  * @author  Rainer 
  * @brief   Initialize on of the STM32L4's U(S)ARTs as debug output, assign
  *          input and output buffers and start receive and transmit
  *
  * @note    Receive is done on a character by character basis by Interrupts
  *          Transmit is done blockwise by DMA
  *          The U(S)ARTs Receive line is also used as edge triggered EXTI
  *          interrupt to wakeup from Stop.
  * 
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup USART
  * @{
  */
/* Includes ------------------------------------------------------------------*/

#include <string.h>

#include "config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "hw_util.h"

#include "dev/usart.h"
#include "dev/devices.h"
#include "system.h"

#include "debug_helper.h"

/*******************************************************************************************
 * Additional data that will be stored to UART type hardware devices
 ******************************************************************************************/

typedef struct {
    USART_TypeDef       *myUart;
    UsartHandleT        *myHandle;
    uint32_t            myBaudrate;
} USART_AdditionalDataType;

/* ------------------------------------------------------------------------------*/
/* Public device variables ------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/
/* UART declarations */

typedef enum UsartDmaDirectionEnum {
  USART_DMA_RX=0,                       // Rx DMA
  USART_DMA_TX,                         // Tx DMA
} UsartDmaDirectionEnumType;


/* ------------------------------------------------------------------------------*/
/* Private functions ------------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/

static USART_AdditionalDataType * USART_GetAdditionalData(const HW_DeviceType *self)
{
    return (USART_AdditionalDataType *)(self->devData);
}

#define TX_IDX  0
#define RX_IDX  1

bool Usart_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable clock */
    HW_SetHWClock(USART_GetAdditionalData(self)->myUart, 1);

    /*##-2- Configure Tx and Rx pins ##########################################*/  
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;

    GpioAFInitAll(self->devGpioAF, &GPIO_InitStruct);

    return true;
}

static void Usart_GPIO_DeInit(const HW_DeviceType *self)
{
    /* Disable GPIO Pins */
    GpioAFDeInitAll(self->devGpioAF);

    /* Disable Usart clock */
    HW_SetHWClock(USART_GetAdditionalData(self)->myUart, 0);
}

static bool Usart_SetCommParams(const HW_DeviceType *self)
{
  UART_HandleTypeDef huart;
  USART_TypeDef *utiny      = USART_GetAdditionalData(self)->myUart;

  /* Disable U(S)ART temporarily */
  utiny->CR1 &= ~USART_CR1_UE;

  utiny->CR1 = 0x0U;
  utiny->CR2 = 0x0U;
  utiny->CR3 = 0x0U;

  huart.Instance            = utiny;
  huart.Init.BaudRate       = USART_GetAdditionalData(self)->myBaudrate;
  huart.Init.WordLength     = UART_WORDLENGTH_8B;
  huart.Init.StopBits       = UART_STOPBITS_1;
  huart.Init.Parity         = UART_PARITY_NONE;
  huart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart.Init.Mode           = UART_MODE_TX_RX;
  huart.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

  /* Set the UART Communication parameters */
  if (UART_SetConfig(&huart) == HAL_ERROR) {
      Error_Handler(__FILE__, __LINE__);
      return false;
  }

  /* Set Wakeup Flag selection to RXNE */
  utiny->CR3 |= (USART_CR3_WUS_0 | USART_CR3_WUS_1);

  /* Enable U(S)ART  */
  utiny->CR1 |= USART_CR1_UE;

  return true;
}
/**
  * @brief DMA UART transmit process complete callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  UsartHandleT *uhandle = (UsartHandleT *)(hdma->Parent);
  
  /* DMA Normal mode */
  // debug_putchar('x');
  if ( HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC) ) {  
    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
    in the UART CR3 register */
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_DMAT);
    
    /* Enable the UART Transmit Complete Interrupt */
    SET_BIT(uhandle->Instance->CR1, USART_CR1_TCIE);
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
static void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  UsartHandleT *uhandle = (UsartHandleT *)(hdma->Parent);
  
  /* DMA Normal mode */
  if ( HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC) )
  {
    
    /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_PEIE);
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_EIE);
    
    /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
       in the UART CR3 register */
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_DMAR);
    
  }
  
  // RHB todo: User-callback on receive
}

/**
  * @brief DMA UART communication error callback.
  * @param hdma DMA handle.
  * @retval None
  */
static void UART_DMAError(DMA_HandleTypeDef *hdma)
{
  UsartHandleT *uhandle = (UsartHandleT *)(hdma->Parent);
  
  /* Stop UART DMA Tx request if ongoing */
  if (HAL_IS_BIT_SET(uhandle->Instance->CR3, USART_CR3_DMAT) ) {
    UsartStopTx(uhandle);
  }
  
  /* Stop UART DMA Rx request if ongoing */
  if ( HAL_IS_BIT_SET(uhandle->Instance->CR3, USART_CR3_DMAR) ) {
    UsartStopRx(uhandle);
  }
  
  if ( uhandle->OnError ) {
    uhandle->last_errors |= HAL_UART_ERROR_DMA;
    uhandle->OnError(uhandle);
  }
}



static void UartAssignInstance ( UsartHandleT *uhandle, USART_TypeDef *utiny ) 
{
    memset(uhandle, 0, sizeof(UsartHandleT) );
    uhandle->Instance    = utiny;
}

static void UsartDmaChannelInit(UsartHandleT *uhandle, const HW_DmaType *dma, UsartDmaDirectionEnumType dmadir )
{
  DMA_HandleTypeDef *hdma = dma->dmaHandle;

  hdma->Instance                 = dma->dmaChannel;
  hdma->Parent                   = uhandle;
  hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma->Init.MemInc              = DMA_MINC_ENABLE;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma->Init.Mode                = DMA_NORMAL;
  hdma->Init.Priority            = DMA_PRIORITY_MEDIUM;
  hdma->Init.Request             = dma->dmaRequest;

  switch ( dmadir ) 
  {
    case USART_DMA_RX:
      hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
      uhandle->hRxDma = dma->dmaHandle;
      break;
    case USART_DMA_TX:
      hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      uhandle->hTxDma = dma->dmaHandle;
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
    case USART_DMA_RX:
      hdma->XferCpltCallback = UART_DMAReceiveCplt;
      hdma->XferErrorCallback = UART_DMAError;
      break;
    case USART_DMA_TX:
      hdma->XferCpltCallback = UART_DMATransmitCplt;   
      hdma->XferErrorCallback = UART_DMAError;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
  }

  return;
}

bool COM_Init(const HW_DeviceType *self)
{
    /* Init GPIO Pins, GPIO Clocks and UART Clock */
    Usart_GPIO_Init(self);

    /* Set Communication Parameters */
    if ( !Usart_SetCommParams(self) ) return false;

     /* Assign Usart to UsartHandleT */
     UartAssignInstance( USART_GetAdditionalData(self)->myHandle, USART_GetAdditionalData(self)->myUart ); 

    /* Configure the NVIC, enable interrupts */
    HW_SetAllIRQs(self->devIrqList, true);

    /* Enable DMA, if specified */            
    if ( self->devDmaRx || self->devDmaTx ) {

      HW_SetDmaChClock(self->devDmaTx, self->devDmaRx);
      // Take the first interrupt to copy prio and subprio to dma channel interrupts
      const HW_IrqType *irq = self->devIrqList->irq;

      if ( self->devDmaRx ) {
         UsartDmaChannelInit( USART_GetAdditionalData(self)->myHandle, self->devDmaRx, USART_DMA_RX );
         HAL_NVIC_SetPriority(self->devDmaRx->dmaIrqNum, irq->irq_prio, irq->irq_subprio);
         HAL_NVIC_EnableIRQ(self->devDmaRx->dmaIrqNum);
      }
      if ( self->devDmaTx ) {
         UsartDmaChannelInit( USART_GetAdditionalData(self)->myHandle, self->devDmaTx, USART_DMA_TX );
         HAL_NVIC_SetPriority(self->devDmaTx->dmaIrqNum, irq->irq_prio, irq->irq_subprio);
         HAL_NVIC_EnableIRQ(self->devDmaTx->dmaIrqNum);
      }
    } // if DMA
    return true;
}

void COM_DeInit(const HW_DeviceType *self)
{
    HW_Reset(USART_GetAdditionalData(self)->myUart );

    Usart_GPIO_DeInit(self); 
    
    /* disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    if(self->devDmaTx) {
      /* De-Initialize the Tx part  */
      HAL_DMA_DeInit(self->devDmaTx->dmaHandle);
      HAL_NVIC_DisableIRQ(self->devDmaTx->dmaIrqNum);
    }
    if(self->devDmaRx) {
      /* De-Initialize the Rx part  */
      HAL_DMA_DeInit(self->devDmaRx->dmaHandle);
      HAL_NVIC_DisableIRQ(self->devDmaRx->dmaIrqNum);
    }
}

#if defined(USART1) && defined(USE_USART1)
    UsartHandleT HandleCOM1;

    static const HW_GpioList_AF gpio_com1 = {
        .num = 2,
        .gpio = {COM1_TX, COM1_RX} ,
    };

    static const USART_AdditionalDataType additional_com1 = {
        USART1,
        &HandleCOM1,
        DEBUG_BAUDRATE,
    };
    
    #if defined(COM1_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com1_tx;
        static const HW_DmaType dmatx_com1 = { &hdma_com1_tx, COM1_TX_DMA };
    #endif
    #if defined(COM1_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com1_rx;
        static const HW_DmaType dmarx_com1 = { &hdma_com1_rx, COM1_RX_DMA };
    #endif

    static const HW_IrqList irq_com1 = {
        .num = 1,
        .irq = { COM1_IRQ, },
    };

    static const HW_DeviceType HW_COM1 = {
        .devName        = "COM1",
        .devGpioAF        = &gpio_com1,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com1,
        .devIrqList     = &irq_com1,
        #if defined(COM1_USE_TX_DMA)
            .devDmaTx = &dmatx_com1,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM1_USE_RX_DMA)
            .devDmaRx = &dmarx_com1,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(USART2) && defined(USE_USART2)
    UsartHandleT HandleCOM2;

    static const HW_GpioList_AF gpio_com2 = {
        .num = 2,
        .gpio = {COM2_TX, COM2_RX} ,
    };

    static const USART_AdditionalDataType additional_com2 = {
        USART2,
        &HandleCOM2,
        DEBUG_BAUDRATE,
    };

    #if defined(COM2_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com2_tx;
        static const HW_DmaType dmatx_com2 = { &hdma_com2_tx, COM2_TX_DMA };
    #endif
    #if defined(COM2_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com2_rx;
        static const HW_DmaType dmarx_com2 = { &hdma_com2_rx, COM2_RX_DMA };
    #endif

    static const HW_IrqList irq_com2 = {
        .num = 1,
        .irq = { COM2_IRQ, },
    };

    static const HW_DeviceType HW_COM2 = {
        .devName        = "COM2",
        .devGpioAF        = &gpio_com2,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com2,
        .devIrqList     = &irq_com2,
        #if defined(COM2_USE_TX_DMA)
            .devDmaTx = &dmatx_com2,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM2_USE_RX_DMA)
            .devDmaRx = &dmarx_com2,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif


#if defined(USART3) && defined(USE_USART3)
    UsartHandleT HandleCOM3;

    static const HW_GpioList_AF gpio_com3 = {
        .num = 2,
        .gpio = {COM3_TX, COM3_RX} ,
    };

    static const USART_AdditionalDataType additional_com3 = {
        USART3,
        &HandleCOM3,
        DEBUG_BAUDRATE,
    };

    #if defined(COM3_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com3_tx;
        static const HW_DmaType dmatx_com3 = { &hdma_com3_tx, COM3_TX_DMA };
    #endif
    #if defined(COM3_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com3_rx;
        static const HW_DmaType dmarx_com3 = { &hdma_com3_rx, COM3_RX_DMA };
    #endif

    static const HW_IrqList irq_com3 = {
        .num = 1,
        .irq = { COM3_IRQ, },
    };

    static const HW_DeviceType HW_COM3 = {
        .devName        = "COM3",
        .devGpioAF        = &gpio_com3,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com3,
        .devIrqList     = &irq_com3,
        #if defined(COM3_USE_TX_DMA)
            .devDmaTx = &dmatx_com3,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM3_USE_RX_DMA)
            .devDmaRx = &dmarx_com3,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif


#if defined(UART4) && defined(USE_UART4)
    UsartHandleT HandleCOM4;

    static const HW_GpioList_AF gpio_com4 = {
        .num = 2,
        .gpio = {COM4_TX, COM4_RX} ,
    };

    static const USART_AdditionalDataType additional_com4 = {
        UART4,
        &HandleCOM4,
        DEBUG_BAUDRATE,
    };

    #if defined(COM4_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com4_tx;
        static const HW_DmaType dmatx_com4 = { &hdma_com4_tx, COM4_TX_DMA };
    #endif
    #if defined(COM4_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com4_rx;
        static const HW_DmaType dmarx_com4 = { &hdma_com4_rx, COM4_RX_DMA };
    #endif

    static const HW_IrqList irq_com4 = {
        .num = 1,
        .irq = { COM4_IRQ, },
    };

    static const HW_DeviceType HW_COM4 = {
        .devName        = "COM4",
        .devGpioAF        = &gpio_com4,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com4,
        .devIrqList     = &irq_com4,
        #if defined(COM4_USE_TX_DMA)
            .devDmaTx = &dmatx_com4,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM4_USE_RX_DMA)
            .devDmaRx = &dmarx_com4,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif


#if defined(UART5) && defined(USE_UART5)
    UsartHandleT HandleCOM5;

    static const HW_GpioList_AF gpio_com5 = {
        .num = 2,
        .gpio = {COM5_TX, COM5_RX} ,
    };

    static const USART_AdditionalDataType additional_com5 = {
        UART5,
        &HandleCOM5,
        DEBUG_BAUDRATE,
    };

    #if defined(COM5_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com5_tx;
        static const HW_DmaType dmatx_com5 = { &hdma_com5_tx, COM5_TX_DMA };
    #endif
    #if defined(COM5_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com5_rx;
        static const HW_DmaType dmarx_com5 = { &hdma_com5_rx, COM5_RX_DMA };
    #endif

    static const HW_IrqList irq_com5 = {
        .num = 1,
        .irq = { COM5_IRQ, },
    };

    static const HW_DeviceType HW_COM5 = {
        .devName        = "COM5",
        .devGpioAF        = &gpio_com5,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com5,
        .devIrqList     = &irq_com5,
        #if defined(COM5_USE_TX_DMA)
            .devDmaTx = &dmatx_com5,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM5_USE_RX_DMA)
            .devDmaRx = &dmarx_com5,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif


#if defined(LPUART1) && defined(USE_LPUART1)
    UsartHandleT HandleCOM6;

    static const HW_GpioList_AF gpio_com6 = {
        .num = 2,
        .gpio = {COM6_TX, COM6_RX} ,
    };

    #if defined(COM6_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com6_tx;
        static const HW_DmaType dmatx_com6 = { &hdma_com6_tx, COM6_TX_DMA };
    #endif
    #if defined(COM6_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com6_rx;
        static const HW_DmaType dmarx_com6 = { &hdma_com6_rx, COM6_RX_DMA };
    #endif

    static const USART_AdditionalDataType additional_com6 = {
        LPUART1,
        &HandleCOM6,
        DEBUG_BAUDRATE,
    };

    static const HW_IrqList irq_com6 = {
        .num = 1,
        .irq = { COM6_IRQ, },
    };

    static const HW_DeviceType HW_COM6 = {
        .devName        = "COM6",
        .devGpioAF      = &gpio_com6,
        .devGpioIO      = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com6,
        .devIrqList     = &irq_com6,
        #if defined(COM6_USE_TX_DMA)
            .devDmaTx = &dmatx_com6,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM6_USE_RX_DMA)
            .devDmaRx = &dmarx_com6,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_SetCommParams,
        .AllowSleep     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

/* ------------------------------------------------------------------------------*/
/* Exported functions -----------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/

/*********************************************************************************
  * @brief  for all selected U(S)ARTs: Assign GPIO Pins, Configure parameters,
  *         initialize UartHandleT structure, Assign and activate interrupts
  *         
  * @Note   "selected" U(S)ARTs are all U(S)ARTs defined in "dev/devices.h"
  *
  * @param  None
  *         
  * @retval None
  ********************************************************************************/
void UsartInitDev (void)
{
#if defined(USART1) && defined(USE_USART1)
    AddDevice(&HW_COM1,
        #ifdef USE_USART1_DEBUG
            true
        #else 
            false
        #endif
    );
#endif
#if defined(USART2) && defined(USE_USART2)
    AddDevice(&HW_COM2,
        #ifdef USE_USART2_DEBUG
            true
        #else 
            false
        #endif
    );
#endif
#if defined(USART3) && defined(USE_USART3)
    AddDevice(&HW_COM3,
        #ifdef USE_USART3_DEBUG
            true
        #else 
            false
        #endif
    );

#endif
#if defined(UART4) && defined(USE_UART4)
    AddDevice(&HW_COM4,
        #ifdef USE_UART4_DEBUG
            true
        #else 
            false
        #endif
    );

#endif
#if defined(UART5) && defined(USE_UART5)
    AddDevice(&HW_COM5,
        #ifdef USE_UART5_DEBUG
            true
        #else 
            false
        #endif
    );

#endif
#if defined(LPUART1) && defined(USE_LPUART1)
    AddDevice(&HW_COM6,
        #ifdef USE_LPUART1_DEBUG
            true    
        #else 
            false
        #endif
    );

#endif
} /* UsartInit */

void UsartClockInit(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  PeriphClkInit.PeriphClockSelection = 0;
#if defined(USART1) && defined(USE_USART1)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_USART1; 
    PeriphClkInit.Usart1ClockSelection = USART1_CLKSOURCE;
#endif
#if defined(USART2) && defined(USE_USART2)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_USART2; 
    PeriphClkInit.Usart2ClockSelection = USART2_CLKSOURCE;
#endif
#if defined(USART3) && defined(USE_USART3)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_USART3; 
    PeriphClkInit.Usart3ClockSelection = USART3_CLKSOURCE;
#endif
#if defined(UART4) && defined(USE_UART4)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_UART4; 
    PeriphClkInit.Uart4ClockSelection = UART4_CLKSOURCE;
#endif
#if defined(UART5) && defined(USE_UART5)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_UART5; 
    PeriphClkInit.Uart5ClockSelection = UART5_CLKSOURCE;
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_LPUART1; 
    PeriphClkInit.Lpuart1ClockSelection = LPUART1_CLKSOURCE;
#endif

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
}


void UartAssignCircBuff ( UsartHandleT *uhandle, LinBuffT *recv, CircBuffT *xmit) 
{
    assert(uhandle);
    assert(uhandle->Instance);

    uhandle->in  = recv;
    uhandle->out = xmit;

    /* If receive buffer is null, disable receiver, otherwise enable Rx */
    if ( recv == NULL ) {
        uhandle->Instance->CR1 &= ~USART_CR1_RE;
    } else {
      /* Enable the UART Receive Interrupt */
      SET_BIT(uhandle->Instance->CR1, USART_CR1_RXNEIE);

      /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      SET_BIT(uhandle->Instance->CR3, USART_CR3_EIE);
      /* Enable parity error Interrupt */
      SET_BIT(uhandle->Instance->CR1, USART_CR1_PEIE);
    }
    
    /* If transmit buffer is null, disable transmitter */
    if ( xmit == NULL ) {
        uhandle->Instance->CR1 &= ~USART_CR1_TE;
    }
}

void UsartAssignCallbacks(UsartHandleT *uhandle, UsartCB OnTx, UsartCB OnErr, UsartRCB OnRx)
{
  assert(uhandle);

  uhandle->OnTxComplete = OnTx;
  uhandle->OnError      = OnErr;
  uhandle->OnRxChar     = OnRx;
}


/**
  * @brief  Handle UART interrupt request.
  * @param  huart UART handle.
  * @retval None
  * @Note   We need no special handling of WUF interrupt, its just used to wakeup from stop
  *         WUF Interrupt bit is cleared by resetting UESM bit
  */
void UsartIRQHandler(UsartHandleT *uhandle)
{
  uint32_t isrflags   = READ_REG(uhandle->Instance->ISR);
  uint32_t cr1its     = READ_REG(uhandle->Instance->CR1);
  uint32_t errorflags;
  uint8_t ch;

  if ( isrflags & USART_ISR_WUF ) {
    uhandle->Instance->ICR = USART_ICR_WUCF;
  }

  
  /* Check for Parity, Frame, Overrun and Noise Error */
  errorflags = isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
  
  if (errorflags != RESET) {
    /* Handle error ( whatever that will be and clear all error flags ) */
    SET_BIT(uhandle->Instance->ICR, errorflags);

    /* ignore noise errors furthermore */
    CLEAR_BIT(errorflags, USART_ISR_NE );

    uhandle->last_errors = errorflags;
    if ( errorflags && uhandle->OnError ) uhandle->OnError(uhandle);
  }

  /* Check for Character Reception */
  if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
  // if ( (isrflags & USART_ISR_RXNE) != RESET ) {
      assert ( uhandle->in );
      ch = (uint8_t) (READ_REG(uhandle->Instance->RDR) & 0xFF );

      /* Store only, if there were no errors */
      if (errorflags == RESET)  {
//        DEBUG_PUTC('r');DEBUG_PUTC('=');print_hexXX((uint8_t)ch);
      
        if ( !LinBuff_Putc(uhandle->in, ch ) ) DEBUG_PUTS("Inbuf Overrun");

        /* call Callback, if specified */
        if (uhandle->OnRxChar) uhandle->OnRxChar(uhandle, ch );
      }
      return;
  }
  
  
  /* Check for next Character Transmission */
  if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
    /* More Characters to transmit ?*/
    if( uhandle->TxCount < uhandle->TxSize )
    {
      assert(CircBuff_Get_Indexed(uhandle->out, uhandle->TxCount, &ch));
      if ( ch < 0x0a || ch > 0x7f ) {
          ch ='!';
      }
      /* Transmit next character */
      uhandle->Instance->TDR = ch;
      uhandle->TxCount++;
    } else {
      /*
       * No more charcters to transmit: disable the UART Transmit Data Register Empty Interrupt 
       * and enable Transmit Complete interrupt
       */
      CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_TXEIE);
      SET_BIT(uhandle->Instance->CR1, USART_CR1_TCIE);
    }
    
    return;
  }
  
  /* Transmission end */
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
    /* Disable the UART Transmit Complete Interrupt */
    CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_TCIE);
    if ( uhandle->OnTxComplete) uhandle->OnTxComplete(uhandle);
    return;
  }
}

void UsartStartTx(UsartHandleT *uhandle, uint8_t *data, uint32_t txSize)
{  
  uhandle->TxSize  = txSize; 
  
  /* If we have an DMA-handle, activate DMA transfer */
  if ( uhandle->hTxDma ) {
    /* Enable the UART transmit DMA channel */
    HAL_DMA_Start_IT(uhandle->hTxDma, (uint32_t)data, (uint32_t)&uhandle->Instance->TDR, txSize);
    
    /* Clear the TC flag in the ICR register */
    uhandle->Instance->ICR = UART_CLEAR_TCF;
    
    /* Assume a successful transmission: all elements transferred, used on completion */
    uhandle->TxCount = uhandle->TxSize;
    /* 
     * Enable the DMA transfer for transmit request by setting the DMAT bit
     * in the UART CR3 register 
     */
    
    SET_BIT(uhandle->Instance->CR3, USART_CR3_DMAT);
  } else {
    
    /* Characters transferred will be counted up in txe interrupt, so initialize to 0 here */
    uhandle->TxCount = 0; 
    /* USART Interrupt driven transfer */
    SET_BIT(uhandle->Instance->CR1, USART_CR1_TXEIE);
  }
}

void UsartStopTx(UsartHandleT *uhandle)
{
  CLEAR_BIT(uhandle->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
}

void UsartStartRx(UsartHandleT *uhandle)
{
    UNUSED(uhandle);
}

void UsartStopRx(UsartHandleT *uhandle)
{
  CLEAR_BIT(uhandle->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_EIE);
}
