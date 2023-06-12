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

#include "config/config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "system/dma_handler.h" /**** 004 ****/
#include "system/hw_util.h"

#include "dev/uart_dev.h"
#include "dev/devices.h"
#include "config/uart_config.h"

#include "debug_helper.h"

/*
 **** 003 **** 
 * Remap new bit names in L4+ series to old ones of L4 series
 */
#if defined(STM32L4PLUS_FAMILY)
    #define USART_CR1_RXNEIE    USART_CR1_RXNEIE_RXFNEIE
    #define USART_CR1_TXEIE     USART_CR1_TXEIE_TXFNFIE
    #define USART_ISR_RXNE      USART_ISR_RXNE_RXFNE
    #define USART_ISR_TXE       USART_ISR_TXE_TXFNF
#endif

/*******************************************************************************************
 * Additional data that will be stored to UART type hardware devices
 ******************************************************************************************/

typedef struct {
//    USART_TypeDef       *myUart;
    UsartHandleT        *myHandle;
    uint32_t            myBaudrate;
    uint32_t            myStopbits;
    uint32_t            myParity;
    uint32_t            myWordlength;
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
/* Forward declarations ---------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/
void UsartStopTx            (UsartHandleT *uhandle);
void UsartStopRx            (UsartHandleT *uhandle);

/* ------------------------------------------------------------------------------*/
/* Private functions ------------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/

static USART_AdditionalDataType * USART_GetAdditionalData(const HW_DeviceType *self)
{
    #if DEBUG_MODE > 0
        if ( self->devType != HW_DEVICE_UART )
            DEBUG_PUTS("Passed device is not an u(s)art!");
    #endif
    assert(self->devType == HW_DEVICE_UART);
    return (USART_AdditionalDataType *)(self->devData);
}

UsartHandleT * USART_GetHandleFromDev(const HW_DeviceType *self)
{
    return USART_GetAdditionalData(self)->myHandle;
}


#define TX_IDX  0
#define RX_IDX  1
#if 0   
#define USE_USART2
#define USE_USART3
#define USE_UART4
#define USE_UART5
#define USE_USART6
#define USE_UART7
#define USE_UART8
#endif

#if defined(STM32L4_FAMILY)
    static bool Usart_SetClockSource(const void *hw)
    {
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      PeriphClkInit.PeriphClockSelection = 0;
      switch ( (uint32_t)hw ) {
    #if defined(USART1) && defined(USE_USART1)
        case USART1_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1; 
        PeriphClkInit.Usart1ClockSelection = USART1_CLKSOURCE;
        break;
    #endif
    #if defined(USART2) && defined(USE_USART2)
        case USART2_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2; 
        PeriphClkInit.Usart2ClockSelection = USART2_CLKSOURCE;
        break;
    #endif
    #if defined(USART3) && defined(USE_USART3)
        case USART3_BASE:    
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3; 
        PeriphClkInit.Usart3ClockSelection = USART3_CLKSOURCE;
        break;
    #endif
    #if defined(UART4) && defined(USE_UART4)
        case UART4_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4; 
        PeriphClkInit.Uart4ClockSelection  = UART4_CLKSOURCE;
        break;
    #endif
    #if defined(UART5) && defined(USE_UART5)
        case UART5_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5; 
        PeriphClkInit.Uart5ClockSelection  = UART5_CLKSOURCE;
        break;
    #endif
    #if defined(LPUART1) && defined(USE_LPUART1)
        case LPUART1_BASE:
        PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_LPUART1; 
        PeriphClkInit.Lpuart1ClockSelection = LPUART1_CLKSOURCE;
        break;
    #endif
        default:
            DEBUG_PRINTF("No Clock source set receipe for HW base 0x%08x\n", (uint32_t)hw );
            return false;
      } // switch

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PRINTF("failed to set source for HW base 0x%08x\n", (uint32_t)hw );
        return false;
      }

      return true;
    }
#elif defined(STM32H7_FAMILY)
    static bool Usart_SetClockSource(const void *hw)
    {
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      PeriphClkInit.PeriphClockSelection = 0;
      switch ( (uint32_t)hw ) {
    #if defined(USART1) && defined(USE_USART1)
        case USART1_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1; 
        PeriphClkInit.Usart16ClockSelection = USART16_CLKSOURCE;
        break;
    #endif
    #if defined(USART2) && defined(USE_USART2)
        case USART2_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(USART3) && defined(USE_USART3)
        case USART3_BASE:    
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(UART4) && defined(USE_UART4)
        case UART4_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(UART5) && defined(USE_UART5)
        case UART5_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(USART6) && defined(USE_USART6)
        case USART6_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART6; 
        PeriphClkInit.Usart16ClockSelection = USART16_CLKSOURCE;
        break;
    #endif
    #if defined(UART7) && defined(USE_UART7)
        case UART7_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART7; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(UART8) && defined(USE_UART8)
        case UART8_BASE:
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART8; 
        PeriphClkInit.Usart234578ClockSelection = USART234578_CLKSOURCE;
        break;
    #endif
    #if defined(LPUART1) && defined(USE_LPUART1)
        case LPUART1_BASE:
        PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_LPUART1; 
        PeriphClkInit.Lpuart1ClockSelection = LPUART1_CLKSOURCE;
        break;
    #endif
        default:
            DEBUG_PRINTF("No Clock source set receipe for HW base 0x%08x\n", (uint32_t)hw );
            return false;
      } // switch

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PRINTF("failed to set source for HW base 0x%08x\n", (uint32_t)hw );
        return false;
      }

      return true;
    }
#else 
    #error "No usart clock assignment defined"
#endif

/*
 * Init or DeInit Clock / clocksource 
 */
static bool Usart_Clock_Init(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !Usart_SetClockSource( self->devBase ) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock( ( USART_TypeDef*)self->devBase, bDoInit );
    return true;
}

bool Usart_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;


    /*##-2- Configure Tx and Rx pins ##########################################*/  
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;

    uint32_t devIdx = GetDevIdx(self);
    GpioAFInitAll(devIdx, self->devGpioAF, &GPIO_InitStruct);

    return true;
}

/******************************************************************************
 * Se the U(S)ARTs communication parameters
 * currently all comm parameters except baudrate are fixed!
 *****************************************************************************/
bool Usart_SetCommParamsLong(const HW_DeviceType *self, UsartCommT *comm, bool bFirstInit )
{
  UsartHandleT *myHandle = USART_GetHandleFromDev(self);
  UART_HandleTypeDef huart={0};
  USART_TypeDef *utiny  = myHandle->Instance;
 
  /* keep actual baudrate in mind ( need it if system frequancy changes */
  myHandle->baudrate = comm->baudrate;

  /* Disable U(S)ART temporarily */
  utiny->CR1 &= ~USART_CR1_UE;

  /* Reset U(S)ARTs config on first init */
  if ( bFirstInit ) {
      utiny->CR1 = 0x0U;
      utiny->CR2 = 0x0U;
      utiny->CR3 = 0x0U;
  }

  huart.Instance            = utiny;
  huart.Init.BaudRate       = comm->baudrate;
  huart.Init.WordLength     = comm->wordlength;
  huart.Init.StopBits       = comm->stopbits;
  huart.Init.Parity         = comm->parity;
  huart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart.Init.Mode           = UART_MODE_TX_RX;
  huart.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

  /* Set the UART Communication parameters */
  if (UART_SetConfig(&huart) == HAL_ERROR) {
      Error_Handler_XX(-15, __FILE__, __LINE__);
      return false;
  }

  /* Set Wakeup Flag selection to RXNE */
  utiny->CR3 |= (USART_CR3_WUS_0 | USART_CR3_WUS_1);

  /* If RX mode is not character mode, disable Receiver, must be enabled manually */
  if (!myHandle->bRxCharMode) CLEAR_BIT(utiny->CR1, USART_CR1_RE);

  /* Enable U(S)ART  */
  utiny->CR1 |= USART_CR1_UE;

  return true;
}


/******************************************************************************
 * Se the U(S)ARTs communication parameters
 * currently all comm parameters except baudrate are fixed!
 *****************************************************************************/
bool Usart_SetCommParams(const HW_DeviceType *self, uint32_t baudrate, bool bFirstInit )
{
  USART_AdditionalDataType *adt = USART_GetAdditionalData(self);
  UsartCommT comm;  

  /* stopbits, parity and wordlength are always taken from static configuration */
  comm.baudrate     = baudrate;
  comm.stopbits     = adt->myStopbits;
  comm.parity       = adt->myParity;
  comm.wordlength   = adt->myWordlength;

  return Usart_SetCommParamsLong(self, &comm, bFirstInit);
}
/******************************************************************************
 * Wrapper for "Usart_SetCommParams"
 * Thsi function will be called by device manager every time the system
 * frequency changes
 *****************************************************************************/
bool Usart_OnFrqChange( const HW_DeviceType *self )
{
    return Usart_SetCommParams(self, USART_GetAdditionalData(self)->myBaudrate, false );
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
  if ( DMA_IS_LINEAR(hdma) ) {  
    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
    in the UART CR3 register */
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_DMAT);
    
    /* Enable the UART Transmit Complete Interrupt */
    SET_BIT(uhandle->Instance->CR1, USART_CR1_TCIE);
  } else {
    /* DMA Circular mode, not implemented */
    Error_Handler_XX(-16, __FILE__, __LINE__);
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
  /* Get the number of bytes transferred (ideally the requested number of bytes ) */
  uhandle->RxCount = uhandle->RxSize - DMA_GET_RXSIZE(hdma);

  /* DMA Normal mode */
  if ( DMA_IS_LINEAR(hdma) )
  {
    
    /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_PEIE);
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_EIE);
    
    /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
       in the UART CR3 register */
    CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_DMAR);

    /* Disable Receiver */
    CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_RE );    
  }
  
  if (uhandle->OnRx) uhandle->OnRx(uhandle, 0 );
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

static void UsartDmaChannelInit(UsartHandleT *uhandle, const HW_DmaType *dma, UsartDmaDirectionEnumType dmadir )
{
  DMA_HandleTypeDef *hdma = dma->dmaHandle;
  
  HW_DMA_HandleInit(hdma, dma, uhandle );

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
        Error_Handler_XX(-17, __FILE__, __LINE__);
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
  }

  return;
}

bool COM_Init(const HW_DeviceType *self)
{
    USART_AdditionalDataType *adt   = USART_GetAdditionalData(self);
    UsartHandleT *myHandle          = adt->myHandle;

    /* Set and enable clock */
    Usart_Clock_Init(self, true);
    
    /* Init GPIO Pins, GPIO Clocks and UART Clock */
    Usart_GPIO_Init(self);

    /* Clear handle */
    memset(myHandle, 0, sizeof(UsartHandleT) );


    /* Assign U(S)ART Instance to handle */
    myHandle->Instance = (USART_TypeDef*)self->devBase; 

    /* Set RX to character mode by default */
    myHandle->bRxCharMode = true;

    /* Set Communication Parameters (baudrate) */
    if ( !Usart_SetCommParams(self, adt->myBaudrate, true) ) return false;

    /* Configure the NVIC, enable interrupts */
    HW_SetAllIRQs(self->devIrqList, true);

    /* Enable DMA, if specified */            
    if ( self->devDmaRx || self->devDmaTx ) {

        // Take the first interrupt to copy prio and subprio to dma channel interrupts
        const HW_IrqType *irq = self->devIrqList->irq;
        DMA_HandleTypeDef *hdma;

        if ( self->devDmaRx ) {
            /**** 004 ****/
            hdma = HW_DMA_RegisterDMAChannel(self->devDmaRx);
            if ( !hdma ) return false;
            UsartDmaChannelInit( myHandle, self->devDmaRx, USART_DMA_RX );
            HW_DMA_SetAndEnableChannelIrq(hdma->Instance, irq->irq_prio, irq->irq_subprio);
        }
        if ( self->devDmaTx ) {
            /**** 004 ****/
            hdma = HW_DMA_RegisterDMAChannel(self->devDmaTx);
            if ( !hdma ) return false;
            UsartDmaChannelInit( myHandle, self->devDmaTx, USART_DMA_TX );
            HW_DMA_SetAndEnableChannelIrq(hdma->Instance, irq->irq_prio, irq->irq_subprio);
        }
    } // if DMA
    return true;
}

void COM_DeInit(const HW_DeviceType *self)
{
    HW_Reset( (USART_TypeDef*)self->devBase );

    /* Disable GPIO Pins */
    uint32_t devIdx = GetDevIdx(self);
    GpioAFDeInitAll(devIdx, self->devGpioAF);
    
    /* disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    if(self->devDmaTx) {
      /**** 004 **** De-Initialize the Tx part  */
      HW_DMA_HandleDeInit(self->devDmaTx->dmaHandle);
    }
    if(self->devDmaRx) {
      /**** 004 **** De-Initialize the Rx part  */
      HW_DMA_HandleDeInit(self->devDmaRx->dmaHandle);
    }

    /* Disable clock */
    Usart_Clock_Init(self, false);
}


/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit is active
 * nor receive in character mode or receiver enabled in block mode
 *****************************************************************************/
bool COM_AllowStop(const HW_DeviceType *self)
{
    UsartHandleT *myHandle  = USART_GetAdditionalData(self)->myHandle;
    USART_TypeDef *u        = (USART_TypeDef*)self->devBase;

    return (u->ISR & USART_ISR_TC ) &&  ( u->CR1 & ( USART_CR1_TXEIE | USART_CR1_TCIE ) )  == 0  && (myHandle->bRxCharMode ? u->ISR & USART_ISR_BUSY : u->CR1 & USART_CR1_RE ) == 0;
}

#if defined(USART1) && defined(USE_USART1)
    UsartHandleT HandleCOM1;

    static const HW_GpioList_AF gpio_com1 = {
        .num = 2,
        .gpio = {COM1_TX, COM1_RX} ,
    };

    static const USART_AdditionalDataType additional_com1 = {
        &HandleCOM1,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
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

    const HW_DeviceType HW_COM1 = {
        .devName        = "COM1",
        .devBase        = USART1,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
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
        &HandleCOM2,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
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

    const HW_DeviceType HW_COM2 = {
        .devName        = "COM2",
        .devBase        = USART2,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
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
        &HandleCOM3,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
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

    const HW_DeviceType HW_COM3 = {
        .devName        = "COM3",
        .devBase        = USART3,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
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
        &HandleCOM4,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
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

    const HW_DeviceType HW_COM4 = {
        .devName        = "COM4",
        .devBase        = UART4,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
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
        &HandleCOM5,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
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

    const HW_DeviceType HW_COM5 = {
        .devName        = "COM5",
        .devBase        = UART5,
        .devGpioAF      = &gpio_com5,
        .devGpioIO      = NULL,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(USART6) && defined(USE_USART6)
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
        &HandleCOM6,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
    };

    static const HW_IrqList irq_com6 = {
        .num = 1,
        .irq = { COM6_IRQ, },
    };

    const HW_DeviceType HW_COM6 = {
        .devName        = "COM6",
        .devBase        = USART6,
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
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(UART7) && defined(USE_UART7)
    UsartHandleT HandleCOM7;

    static const HW_GpioList_AF gpio_com7 = {
        .num = 2,
        .gpio = {COM7_TX, COM7_RX} ,
    };

    static const USART_AdditionalDataType additional_com7 = {
        &HandleCOM7,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
    };

    #if defined(COM7_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com7_tx;
        static const HW_DmaType dmatx_com7 = { &hdma_com7_tx, COM7_TX_DMA };
    #endif
    #if defined(COM7_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com7_rx;
        static const HW_DmaType dmarx_com7 = { &hdma_com7_rx, COM7_RX_DMA };
    #endif

    static const HW_IrqList irq_com7 = {
        .num = 1,
        .irq = { COM7_IRQ, },
    };

    const HW_DeviceType HW_COM7 = {
        .devName        = "COM7",
        .devBase        = UART7,
        .devGpioAF      = &gpio_com7,
        .devGpioIO      = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com7,
        .devIrqList     = &irq_com7,
        #if defined(COM7_USE_TX_DMA)
            .devDmaTx = &dmatx_com7,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM7_USE_RX_DMA)
            .devDmaRx = &dmarx_com7,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
    UsartHandleT HandleCOM9;

    static const HW_GpioList_AF gpio_com9 = {
        .num = 2,
        .gpio = {COM9_TX, COM9_RX} ,
    };

    #if defined(COM9_USE_TX_DMA)
        static DMA_HandleTypeDef hdma_com9_tx;
        static const HW_DmaType dmatx_com9 = { &hdma_com9_tx, COM9_TX_DMA };
    #endif
    #if defined(COM9_USE_RX_DMA)
        static DMA_HandleTypeDef hdma_com9_rx;
        static const HW_DmaType dmarx_com9 = { &hdma_com9_rx, COM9_RX_DMA };
    #endif

    static const USART_AdditionalDataType additional_com9 = {
        &HandleCOM9,
        DEBUG_BAUDRATE,
        UART_STOPBITS_1,
        UART_PARITY_NONE,
        UART_WORDLENGTH_8B,
    };

    static const HW_IrqList irq_com9 = {
        .num = 1,
        .irq = { COM9_IRQ, },
    };

    const HW_DeviceType HW_COM9 = {
        .devName        = "COM9",
        .devBase        = LPUART1,
        .devGpioAF      = &gpio_com9,
        .devGpioIO      = NULL,
        .devType        =  HW_DEVICE_UART,
        .devData        = &additional_com9,
        .devIrqList     = &irq_com9,
        #if defined(COM9_USE_TX_DMA)
            .devDmaTx = &dmatx_com9,
        #else
            .devDmaTx = NULL,
        #endif
        #if defined(COM9_USE_RX_DMA)
            .devDmaRx = &dmarx_com9,
        #else
            .devDmaRx = NULL,
        #endif

        .Init           = COM_Init,
        .DeInit         = COM_DeInit,
        .OnFrqChange    = Usart_OnFrqChange,
        .AllowStop      = COM_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

/* ------------------------------------------------------------------------------*/
/* Exported functions -----------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/


/******************************************************************************
 * Enable Uart receiver
 * if in char mode, also enable error interrupts, otherwise disable 
 *****************************************************************************/
static void Usart_EnableRx( USART_TypeDef *Instance, bool bCharmode )
{

    /* Enable the UART Receiver */
    SET_BIT(Instance->CR1, USART_CR1_RE);

    /* Enable the UART Receive Interrupt */
    SET_BIT(Instance->CR1, USART_CR1_RXNEIE);

    /* in char mode enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    if ( bCharmode ) {
        SET_BIT(Instance->CR3, USART_CR3_EIE);
        /* Enable parity error Interrupt */
        SET_BIT(Instance->CR1, USART_CR1_PEIE);
    } else {
        CLEAR_BIT(Instance->CR3, USART_CR3_EIE);
        /* Enable parity error Interrupt */
        CLEAR_BIT(Instance->CR1, USART_CR1_PEIE);
    }
}


void Usart_AssignBuffers ( UsartHandleT *uhandle, LinBuffT *rxcharbuf, CircBuffT *xmit) 
{
    assert(uhandle);
    assert(uhandle->Instance);

    uhandle->in  = rxcharbuf;
    uhandle->out = xmit;

    /* If receive buffer is null in character mode, disable receiver, otherwise enable Rx */
    if ( uhandle->bRxCharMode && rxcharbuf == NULL ) {
        uhandle->Instance->CR1 &= ~USART_CR1_RE;
    } else {
        Usart_EnableRx(uhandle->Instance, uhandle->bRxCharMode);
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
  uhandle->OnRx         = OnRx;
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

    if ( isrflags & USART_ISR_WUF ) uhandle->Instance->ICR = USART_ICR_WUCF;
    
    /* Check for Parity, Frame, Overrun and Noise Error */
    errorflags = isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
    if (errorflags != RESET) {
        /* Handle error ( whatever that will be and clear all error flags ) */
        SET_BIT(uhandle->Instance->ICR, errorflags);

        /* ignore noise errors furthermore */
        CLEAR_BIT(errorflags, USART_ISR_NE );

        uhandle->last_errors = errorflags;
        if (errorflags) DEBUG_PRINTTS("RX err flags=0x%08x\n",errorflags);

        if ( errorflags && uhandle->OnError ) uhandle->OnError(uhandle);
    }

    /* Check for Character Reception */
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
        ch = (uint8_t) (READ_REG(uhandle->Instance->RDR) & 0xFF );
        if ( uhandle->bRxCharMode ) {
            assert ( uhandle->in );
            /* Store only, if there were no errors */
            if (errorflags == RESET)  {
                // DEBUG_PUTC('r');DEBUG_PUTC('='); print_hexXX((uint8_t)ch);
                if ( !LinBuff_Putc(uhandle->in, ch ) ) DEBUG_PUTS("Inbuf Overrun");
                /* call Callback, if specified */
                if (uhandle->OnRx) uhandle->OnRx(uhandle, ch );
            }
            return;
        } else {
            assert(uhandle->blockIn);
            /* Rx block mode: Store character and check size*/
            uhandle->blockIn[uhandle->RxCount++] = ch;
            if ( uhandle->RxCount == uhandle->RxSize ) {
                /* If expected length is received, disable receiver and signal via callback */
                CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_RE);
                if (uhandle->OnRx) uhandle->OnRx(uhandle, 0 );
            }
        }
    } /* if character reception */
  
  /* Check for next Character Transmission */
  if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
    /* More Characters to transmit ?*/
    if( uhandle->TxCount < uhandle->TxSize )
    {
      /***** 008 *****/
      if ( uhandle->out ) {
         /* If CircBuff is assigned, take char from that */
          assert(CircBuff_Get_Indexed(uhandle->out, uhandle->TxCount, &ch));
          #if 0
              if ( ch < 0x0a || ch > 0x7f ) {
                  ch ='!';
              }
          #endif

      } else { 
        /* Otherwise take it from assigned array */
        ch = *(uhandle->blockIn + uhandle->TxCount);
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

/******************************************************************************
 * Transmit and receive one Byte with active wait/timeout
 * txrx - byte to send / received byte on return
 * tmo  - timeout ( in ms )
 * will return false, if timout occured before reception
 * note: This function reset all interrupt sources 
 *****************************************************************************/
bool UsartTxRxOneByteWait(UsartHandleT *uhandle, uint8_t *txrx, uint32_t tmo) 
{
  uint32_t ticktmo = HAL_GetTick() + tmo;
  USART_TypeDef *u = uhandle->Instance;
  
  assert(u->CR1 & USART_CR1_UE);
  /* Disable all interrupts */
  CLEAR_BIT(u->CR1, USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_PEIE);
  CLEAR_BIT(u->CR3, USART_CR3_EIE );
  
 
  /* Enable Tx/Rx */
  SET_BIT(u->CR1, USART_CR1_TE | USART_CR1_RE );

  /* Wait for TX empty */
  while ( !READ_BIT(u->ISR, USART_ISR_TXE ) && HAL_GetTick() != ticktmo ) ;
  if ( HAL_GetTick() == ticktmo ) return false;

  /* dummy read to empty receive register */
  (void)u->RDR;   
  u->TDR = *txrx;

  /* Wait for RX not empty or timeout */
  while ( !READ_BIT(u->ISR, USART_ISR_RXNE) && HAL_GetTick() != ticktmo ) ;
  if ( HAL_GetTick() == ticktmo ) {
//    DEBUG_PUTS("Unexpected timeout on Tx/Rx byte");
    return false;
  }

  *txrx = u->RDR;

  /* If usart is in blockmode, disable receiver */
  if ( !uhandle->bRxCharMode )  
    CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_RE );    

  return true;
}

void UsartStartTx(UsartHandleT *uhandle, uint8_t *data, uint32_t txSize)
{  
  uhandle->TxSize  = txSize; 
  
  /* If we have an DMA-handle, activate DMA transfer */
  if ( uhandle->hTxDma) {
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

void UsartStartRx(UsartHandleT *uhandle, uint8_t *rxbuf, uint16_t rxsize)
{
    uhandle->blockIn     = rxbuf;
    uhandle->RxSize      = rxsize;
    uhandle->bRxCharMode = false;
    uhandle->RxCount     = 0;

    /* Clear the RXNE flag in the ISR register and enable U(S)ART */
    uhandle->Instance->RQR = USART_RQR_RXFRQ;
    Usart_EnableRx(uhandle->Instance, false);
    
    if ( uhandle->hRxDma ) {

        /* Enable the UART receive DMA channel */
        HAL_DMA_Start_IT(uhandle->hRxDma, (uint32_t)&uhandle->Instance->RDR, (uint32_t)rxbuf, rxsize);
    
    
        /* Assume a successful transmission: all elements transferred, used on completion */
        uhandle->TxCount = uhandle->RxSize;
        /* 
         * Enable the DMA transfer for transmit request by setting the DMAR bit
         * in the UART CR3 register 
         */
    
        SET_BIT(uhandle->Instance->CR3, USART_CR3_DMAR);
    }
}

/******************************************************************************
 * Abort asynchronous operations
 *****************************************************************************/
void UsartAbortOp (UsartHandleT *uhandle)
{
  USART_TypeDef *i = uhandle->Instance;  
  DMA_HandleTypeDef *hdma;
  /* Disable TXEIE, TCIE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(i->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_TCIE));
  CLEAR_BIT(i->CR3, USART_CR3_EIE);


  /* Disable the UART DMA Tx request if enabled */
  if (HAL_IS_BIT_SET(i->CR3, USART_CR3_DMAT))
  {
    CLEAR_BIT(i->CR3, USART_CR3_DMAT);

    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    hdma = uhandle->hTxDma;
    if ( hdma != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hdma->XferAbortCallback = NULL;

      HAL_DMA_Abort(hdma);
    }
  }

  /* Disable the UART DMA Rx request if enabled */
  if (HAL_IS_BIT_SET(i->CR3, USART_CR3_DMAR))
  {
    CLEAR_BIT(i->CR3, USART_CR3_DMAR);

    hdma = uhandle->hRxDma;
    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (hdma != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hdma->XferAbortCallback = NULL;

      HAL_DMA_Abort(hdma);
    }
  }

  /* If usart is in blockmode, disable receiver */
  if ( !uhandle->bRxCharMode ) CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_RE );    

  /* Clear the Error flags in the ICR register */
  i->ICR = UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF;

}

void UsartStopRx(UsartHandleT *uhandle)
{
  CLEAR_BIT(uhandle->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(uhandle->Instance->CR3, USART_CR3_EIE);
  if ( !uhandle->bRxCharMode ) CLEAR_BIT(uhandle->Instance->CR1, USART_CR1_RE );
}


