/*
 ******************************************************************************
 * @file    bbspi_dev.h 
 * @author  Rainer
 * @brief   bit bang spi device, only device functions, nothing else 
 *
 * @note:   ----------------------------------------------------
 *          to be performant, this file has to be compiled with 
 *          optimization on, even in DEBUG config
 *          ----------------------------------------------------
 *
 *****************************************************************************/

/** @addtogroup BitBang SPI device
  * @{
  */

#include "config/devices_config.h"

#if defined(USE_BBSPI1) || defined(USE_BBSPI2) || defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4)

#include "error.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"
#include "dev/spi.h"

#include "config/spi_config.h"
#include "dev/devices.h"
#include "system/exti_handler.h"
#include "dev/spi_dev.h"
#include "debug_helper.h"



/*******************************************************************************************
 * Additional data that will be stored to I2C type hardware devices
 ******************************************************************************************/




typedef struct {
    SpiHandleT      *mySpiHandle;   /* My associated Spi-handle */
    uint32_t        baudrate;       /* Baudrate to use (approx ) */
    /* Interrupt modes for any additional input pins: May be one of
     * GPIO_MODE_IT_RISING,  GPIO_MODE_IT_FALLING,  GPIO_MODE_IT_RISING_FALLING,
     * GPIO_MODE_EVT_RISING, GPIO_MODE_EVT_FALLING, GPIO_MODE_EVT_RISING_FALLING
     * or 0. In this case GPIO_MODE_IT_RISING_FALLING is used by default */
    uint32_t        busy_irq_mode;  /* Interrupt mode for busy line, if used */   
    uint32_t        inp_irq_mode;   /* Interrupt mode for inp line, if used */ 
    uint32_t        hw_irq_mode;    /* Interrupt mode for hw line, if used */ 
    uint8_t         use_nss;        /* Use hardware nss ( only for HW SPI */
    uint8_t         use_miso;       /* Use miso line ? */
    uint8_t         use_miso_irq;   /* let change in Miso line generate an pin change interrupt ? */
    uint8_t         use_dnc;        /* Use Data,Not Command line ? */
    uint8_t         use_rst;        /* Use Reset line ? */
    uint8_t         use_busy;       /* Use Busy Line ?  */
    uint8_t         use_busy_irq;   /* Use Busy Interrupt ?  */
    uint8_t         use_inp;        /* Use additional general purpose input line ?  */
    uint8_t         use_inp_irq;    /* Use level change Interrupt on that line ?  */
    uint8_t         use_hw_irq;     /* Is HW Interrupt configured?  */
    uint8_t         datasize;       /* SPI datasize */
} SPI_AdditionalDataType;


static SPI_AdditionalDataType * SPI_GetAdditionalData(const HW_DeviceType *self)
{
    return (SPI_AdditionalDataType *)(self->devData);
}

SpiHandleT * SPI_GetHandleFromDev(const HW_DeviceType *self)
{
    return SPI_GetAdditionalData(self)->mySpiHandle;
}

const HW_Gpio_AF_Type *SPI_GetGpio(const HW_DeviceType *self, SPI_PinEnumType idx)
{
    return &self->devGpioAF->gpio[idx];
}

uint8_t SPI_HasMisoIRQ(const HW_DeviceType *self)
{
    return SPI_GetAdditionalData(self)->use_miso_irq;
}

uint8_t SPI_HasBusyIRQ(const HW_DeviceType *self)
{
    return SPI_GetAdditionalData(self)->use_busy_irq;
}

uint8_t SPI_HasInpIRQ(const HW_DeviceType *self)
{
    return SPI_GetAdditionalData(self)->use_inp_irq;
}

/******************************************************************************
 * @brief Initialize DnC, nRST and Busy, if configured
 * @param self - hardware device of type HW_DEVICE_HWSPI or HW_DEVICE_BBSPI
 * @param gpio - List of gpio pins. MOSI, SCK and NSEL are mandatory, all
 *               other are optional
 * @note identical for HW and BB SPI
 *****************************************************************************/
static void SPI_GPIO_InitOther(uint32_t devIdx, SPI_AdditionalDataType *adt, const HW_Gpio_AF_Type *gpio, GPIO_InitTypeDef  *Init )
{
    Init->Mode = GPIO_MODE_OUTPUT_PP;
    Init->Speed = GPIO_SPEED_FREQ_LOW;   /* Up to 1MHz LO is ok */
    /* if used, D/C output also as PushPull output,initially low */
    if (adt->use_dnc ) GpioAFInitOneWithPreset( devIdx, &gpio[DNC_IDX] , Init, HW_OUTPUT_LOW);

    /* if used, reset output also as PushPull output, initially high */
    if (adt->use_rst ) GpioAFInitOneWithPreset( devIdx, &gpio[RST_IDX] , Init, HW_OUTPUT_HIGH);

    /* 
     * if used, configure MISO as Input, if interrupt trigger is selected,
     * then configure for interrupt on rising edge 
     * finally, disable the pin interrupt in exti->imr1 initially
     * it has to be enabled by user manually
     */

    if ( adt->use_busy ) {
        // Busy is active High
        Init->Pull = GPIO_PULLDOWN;
        if ( adt->use_busy_irq ) {
            if ( adt->busy_irq_mode != 0 ) 
                Init->Mode = adt->busy_irq_mode;
            else
                Init->Mode = GPIO_MODE_IT_RISING_FALLING;
        } else {
            Init->Mode = GPIO_MODE_INPUT;
        }
        GpioAFInitOne( devIdx, &gpio[BUSY_IDX] , Init);    
        
        /* 
         * finally, disable the pin interrupt in exti->imr1 
         * it has to be enabled by user manually
         */
        if ( adt->use_busy_irq ) {
            EXTI_DISABLE_IRQ( gpio[BUSY_IDX].pin );
        }
    }

    if ( adt->use_inp ) {
        // Do not Pull additional Input up or down
        Init->Pull = GPIO_NOPULL;
        if ( adt->use_inp_irq ) {
            if ( adt->inp_irq_mode != 0 ) 
                Init->Mode = adt->inp_irq_mode;
            else
                Init->Mode = GPIO_MODE_IT_RISING_FALLING;
        } else {
            Init->Mode = GPIO_MODE_INPUT;
        }
        GpioAFInitOne( devIdx, &gpio[INP_IDX] , Init);    
        
        /* 
         * finally, disable the pin interrupt in exti->imr1 
         * it has to be enabled by user manually
         */
        if ( adt->use_inp_irq ) {
            EXTI_DISABLE_IRQ( gpio[INP_IDX].pin );
        }
    }

}
 
/******************************************************************************
 * @brief Initialize all GPIO Pins for Hardware SPI operation
 * @param self - hardware device of type HW_DEVICE_HWSPI or HW_DEVICE_BBSPI
 * @param gpio - List of gpio pins. MOSI, SCK and NSEL are mandatory, all
 *               other are optional
 *****************************************************************************/
static void SPI_GPIO_InitHW(const HW_DeviceType *self, const HW_Gpio_AF_Type *gpio )
{
    GPIO_InitTypeDef  Init;

    SPI_AdditionalDataType *adt =  SPI_GetAdditionalData(self);

    uint32_t devIdx = GetDevIdx(self);

    /* Enable SPI clock */
    HW_SetHWClock((SPI_TypeDef*)self->devBase, 1);

    /* 
     * Configure the MOSI, SCK and NSS as AF Pushpull Outputs, 
     * NSS and MOSI will be initially high, SCK will be initially low 
     */
    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_LOW;   /* Up to 1MHz LO is ok */
    GpioAFInitOneWithPreset(devIdx, &gpio[MOSI_IDX] , &Init, HW_OUTPUT_HIGH);
    GpioAFInitOneWithPreset(devIdx, &gpio[SCK_IDX] , &Init,  HW_OUTPUT_LOW);
    
    if ( adt->use_miso ) {
        /* if MISO is used, configure identical to MOSI, SCK */
        GpioAFInitOne(devIdx, &gpio[MISO_IDX] , &Init);    
        if ( adt->use_miso_irq ) {
            /* If miso irq is configured, configure interrupt on rising Miso line */
            Exti_ConfigIrq(gpio[MISO_IDX].gpio, gpio[MISO_IDX].pin, EXTI_TRIGGER_RISING );
            /* Do not enable yet */
            EXTI_DISABLE_IRQ( gpio[MISO_IDX].pin );
        }
    }

    if ( adt->use_nss ) {
       /* If Hardware NSS is to be used,  the NSEL pin will be used for that */
       Init.Mode = GPIO_MODE_AF_PP;
    } else {
       /* NSEL Pin is any gpio pin, configure for OpenDrain output */
       Init.Mode = GPIO_MODE_OUTPUT_PP;
    }
    GpioAFInitOneWithPreset(devIdx, &gpio[NSEL_IDX] , &Init, HW_OUTPUT_HIGH);

     /* Configure DnC, nRST and BUSY, if specified */
    SPI_GPIO_InitOther(devIdx,  adt, gpio, &Init );
}

/******************************************************************************
 * @brief Initialize all GPIO Pins for Bitbang SPI operation
 * @param self - hardware device of type HW_DEVICE_HWSPI or HW_DEVICE_BBSPI
 * @param gpio - List of gpio pins. MOSI, SCK and NSEL are mandatory, all
 *               other are optional
 *****************************************************************************/
static void SPI_GPIO_InitBB(const HW_DeviceType *self, const HW_Gpio_AF_Type *gpio )
{
    GPIO_InitTypeDef  Init;

    SPI_AdditionalDataType *adt =  SPI_GetAdditionalData(self);

    uint32_t devIdx = GetDevIdx(self);
  
    /* 
     * Configure the MOSI, SCK and NSEL as Pushpull-Outputs, 
     * MOSI and NSEL will be initially high SCK will be initially low 
     */
    Init.Mode = GPIO_MODE_OUTPUT_PP;
    Init.Speed = GPIO_SPEED_FREQ_LOW;   /* Up to 1MHz LO is ok */
    GpioAFInitOneWithPreset( devIdx, &gpio[MOSI_IDX] , &Init, HW_OUTPUT_HIGH);
    GpioAFInitOneWithPreset( devIdx, &gpio[SCK_IDX] , &Init,  HW_OUTPUT_LOW);
    GpioAFInitOneWithPreset( devIdx, &gpio[NSEL_IDX] , &Init, HW_OUTPUT_HIGH);

    /* 
     * if used, configure MISO as Input, if interrupt trigger is selected,
     * then configure for interrupt on rising edge 
     * finally, disable the pin interrupt in exti->imr1 initially
     * it has to be enabled by user manually
     */

    if ( adt->use_miso ) {
        // Miso of RFM12 is active Low
        if ( adt->use_miso_irq ) {
            Init.Mode = GPIO_MODE_IT_RISING;
        } else {
            Init.Mode = GPIO_MODE_INPUT;
        }
        GpioAFInitOne( devIdx, &gpio[MISO_IDX] , &Init);    
        
        /* 
         * finally, disable the pin interrupt in exti->imr1 
         * it has to be enabled by user manually
         */
        if ( adt->use_miso_irq ) {
            EXTI_DISABLE_IRQ( gpio[MISO_IDX].pin );
        }
    }

     /* Configure DnC, nRST and BUSY, if specified */
     SPI_GPIO_InitOther( devIdx, adt, gpio, &Init );

}

static void SPI_GPIO_DeInit(const HW_DeviceType *self)
{
    SPI_AdditionalDataType *adt =  SPI_GetAdditionalData(self);
    const HW_Gpio_AF_Type *gpio = self->devGpioAF->gpio;

    uint32_t devIdx = GetDevIdx(self);

    /* Disable all SPI GPIO pins Pins */
    for ( uint32_t i = 0; i <= NSEL_IDX; i++ )
        GpioAFDeInitOne( devIdx, &gpio[i] );
    if ( adt->use_miso ) GpioAFDeInitOne( devIdx, &gpio[MISO_IDX] );
    if ( adt->use_dnc  ) GpioAFDeInitOne( devIdx, &gpio[DNC_IDX]  );
    if ( adt->use_rst  ) GpioAFDeInitOne( devIdx, &gpio[RST_IDX]  );
    if ( adt->use_busy ) GpioAFDeInitOne( devIdx, &gpio[BUSY_IDX]  );
    if ( adt->use_inp  ) GpioAFDeInitOne( devIdx, &gpio[INP_IDX]  );
}

extern const SpiFunctionT SpiFns_hw;
extern const SpiFunctionT SpiFns_bb;

/******************************************************************************
 * Initialize associated SPI Handle
 *****************************************************************************/
void SpiHandleInit (SpiHandleT *hnd, const HW_DeviceType *SpiDev)
{
    SpiDataT *data = hnd->data;

    data->mySpiDev = SpiDev;
    SPI_AdditionalDataType *adt = SPI_GetAdditionalData(SpiDev);
    
    data->use_miso    = adt->use_miso;
    data->use_miso_irq= adt->use_miso_irq;
    data->use_dnc     = adt->use_dnc;
    data->use_rst     = adt->use_rst;
    data->use_busy    = adt->use_busy;
    data->use_busy_irq= adt->use_busy_irq;
    data->use_inp     = adt->use_inp;
    data->use_inp_irq = adt->use_inp_irq;
    data->datasize    = adt->datasize;
    data->bIsMaster   = 1;                   /* Currently only master mode implemented */

    /* Store the GPIO_pin BSRR and bit positions for faster access*/
    data->nsel_bsrr   = &SPI_GetGpio  (SpiDev, NSEL_IDX)->gpio->BSRR;
    data->nsel_bitpos = SPI_GetGpio   (SpiDev, NSEL_IDX)->pin;
    if (data->use_miso) {
        data->miso_bitpos = SPI_GetGpio (SpiDev, MISO_IDX)->pin;
        data->miso_idr    = &SPI_GetGpio(SpiDev, MISO_IDX)->gpio->IDR;
    }
    if (data->use_busy) {
        data->busy_idr    = &SPI_GetGpio(SpiDev, BUSY_IDX)->gpio->IDR;
        data->busy_bitpos = SPI_GetGpio (SpiDev, BUSY_IDX)->pin;
    }
    if (data->use_inp) {
        data->inp_idr    = &SPI_GetGpio(SpiDev, INP_IDX)->gpio->IDR;
        data->inp_bitpos = SPI_GetGpio (SpiDev, INP_IDX)->pin;
    }
    if (data->use_dnc) {
        data->dnc_bsrr   = &SPI_GetGpio  (SpiDev, DNC_IDX)->gpio->BSRR;
        data->dnc_bitpos = SPI_GetGpio   (SpiDev, DNC_IDX)->pin;
    }
    if (data->use_rst) {
        data->rst_bsrr   = &SPI_GetGpio  (SpiDev, RST_IDX)->gpio->BSRR;
        data->rst_bitpos = SPI_GetGpio   (SpiDev, RST_IDX)->pin;
    }

    if ( SpiDev->devType == HW_DEVICE_HWSPI ) {
        /* Specific initialization for HW SPI devices */
        data->hw.myBaudrate = adt->baudrate;
        data->hw.use_nss     = adt->use_nss;
        data->hw.use_hw_irq  = adt->use_hw_irq;
        data->hw.OnError     = NULL;
        data->hw.OnTxComplete= NULL;
        data->hw.OnTxRxComplete= NULL;
        /* assign function block for hardware SPI */
        hnd->fns = &SpiFns_hw;
    } else {
        /* Specific initialization for BB SPI devices */
        data->bIsMaster = 1;
        data->bb.mosi_bsrr   = &SPI_GetGpio  (SpiDev, MOSI_IDX)->gpio->BSRR;
        data->bb.mosi_bitpos = SPI_GetGpio   (SpiDev, MOSI_IDX)->pin;
        data->bb.sck_bsrr    = &SPI_GetGpio  (SpiDev, SCK_IDX )->gpio->BSRR;
        data->bb.sck_bitpos  = SPI_GetGpio   (SpiDev, SCK_IDX)->pin;
        /* assign function block for BitBang SPI */
        hnd->fns = &SpiFns_bb;
    }
}


/******************************************************************************
 * Will be overwritten if any Hardware SPI is defined. This is just a dummy
 * implementation to make the following "SPI_Init" compileable in case of
 * only BBSPI being used
 *****************************************************************************/
__weak bool SpiClockInit(const HW_DeviceType *self, bool bDoInit)
{
    UNUSED(self); UNUSED(bDoInit);
    return true;
}
   
bool SPI_Init(const HW_DeviceType *self)
{

    const HW_Gpio_AF_Type *gpio = self->devGpioAF->gpio;
    bool bIsBbSpi = self->devType == HW_DEVICE_BBSPI;

    if ( bIsBbSpi ) {
        SPI_GPIO_InitBB( self, gpio );
    } else {
        SpiClockInit( self, true);
        SPI_GPIO_InitHW( self, gpio );
    }

    /* Enable all configured interrupts*/
    HW_SetAllIRQs ( self->devIrqList, true );

    /* Initialize the associated Handle */
    SpiHandleT *hnd = SPI_GetAdditionalData(self)->mySpiHandle;
    return SpiInit(hnd, self);

    // return SpiInit(SPI_GetAdditionalData(self)->mySpiHandle, self);
}

void SPI_DeInit(const HW_DeviceType *self)
{
    /*Reset peripherals */
    if (self->devType == HW_DEVICE_HWSPI) HW_Reset((SPI_TypeDef *)(self->devBase) );

    /* zero the associated Handle */
    SpiDeInit(SPI_GetAdditionalData(self)->mySpiHandle, self);

    SPI_GPIO_DeInit(self);

    /* Disable all IRQs */
    HW_SetAllIRQs ( self->devIrqList, false );


    if (self->devType == HW_DEVICE_HWSPI)  {
        /* Disable SPI clock */
        HW_SetHWClock((SPI_TypeDef*)self->devBase, 0);
    }

    /* Deregister all callbacks */
    const HW_IrqType *irq;
    for ( uint8_t i = 0; i < self->devIrqList->num; i++ ) {
        irq = &self->devIrqList->irq[i];
        Exti_UnRegister_Callback(irq->irq_num);
    }

    if (self->devType == HW_DEVICE_HWSPI) {
        const HW_DmaType *dma;
        /* Disable the DMA, if used */
        dma = self->devDmaRx;
        if(dma) {
          /* De-Initialize the Rx part  */
          HAL_DMA_DeInit(dma->dmaHandle);
          HAL_NVIC_DisableIRQ(dma->dmaIrqNum);
        }
        dma = self->devDmaTx;
        if(dma) {
          /* De-Initialize the Tx part  */
          HAL_DMA_DeInit(dma->dmaHandle);
          HAL_NVIC_DisableIRQ(dma->dmaIrqNum);
        }
    } 
}

/******************************************************************************
 * Callback AFTER the system frequency has been altered
 *****************************************************************************/
bool SPI_OnFrqChange(const HW_DeviceType *self)
{
  SpiHandleT *hnd = SPI_GetAdditionalData(self)->mySpiHandle;
  SpiDataHW *hndhw = &hnd->data->hw;
  SPI_TypeDef *spi    = (SPI_TypeDef*)hnd->data->mySpiDev->devBase;

  HwSpiSetPrescaler (spi, hndhw->myBaudrate );  
  return true;
}


/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool SPI_AllowStop(const HW_DeviceType *self)
{
#if defined(STM32L476xx)|| defined(STM32L496xx)
    /* Allow sleep, if not busy */
    return (((SPI_TypeDef *)self->devBase)->SR & SPI_SR_BSY_Msk ) == 0;
#elif defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    /* Allow sleep, if not enabled */
    return (((SPI_TypeDef *)self->devBase)->CR1 & SPI_CR1_SPE ) == 0;
#else
    #error "No receipe to allow stop by SPI device"
#endif
}


#if defined(USE_BBSPI1) || defined(USE_SPI1)
    SpiDataT             SPI1Data;
    SpiHandleT           SPI1Handle = {&SPI1Data, NULL };
    #if defined(USE_SPI1) && defined(SPI1_USE_DMA) 
        static DMA_HandleTypeDef hdma_spi1_tx;
        static DMA_HandleTypeDef hdma_spi1_rx;
        static const HW_DmaType dmatx_spi1 = { &hdma_spi1_tx, SPI1_TX_DMA };
        static const HW_DmaType dmarx_spi1 = { &hdma_spi1_rx, SPI1_RX_DMA };
    #endif
    static const HW_GpioList_AF gpio_spi1 = {
        /* MOSI, SCK, nSEL, MISO, DnC, nRST, BUSY */
        .gpio = { 
            SPI1_MOSI, SPI1_SCK, 
            #ifdef SPI1_USE_NSS
                SPI1_NSS,
            #else
                SPI1_NSEL,
            #endif
            #ifdef SPI1_USE_MISO
                SPI1_MISO,
            #else
                {0},
            #endif
            #ifdef SPI1_USE_DNC
                SPI1_DNC,
            #else
                {0},
            #endif
            #ifdef SPI1_USE_RST
                SPI1_RST,
            #else
                {0},
            #endif
            #ifdef SPI1_USE_BUSY
                SPI1_BUSY,
            #else
                {0},
            #endif
            #ifdef SPI1_USE_INP
                SPI1_INP,
            #else
                {0},
            #endif
        },
        .num = 8, 
    };

    static const SPI_AdditionalDataType additional_spi1 = {
        .mySpiHandle = &SPI1Handle,
        .use_nss = 
            #ifdef SPI1_USE_NSS 
                1,
            #else
                0,
            #endif
        .use_miso = 
            #ifdef SPI1_USE_MISO 
                1,
            #else
                0,
            #endif
        .use_miso_irq = 
            #ifdef SPI1_USE_MISO_IRQ 
                1,
            #else
                0,
            #endif
        .use_dnc = 
            #ifdef SPI1_USE_DNC
                1,
            #else
                0,
            #endif
        .use_rst = 
            #ifdef SPI1_USE_RST
                1,
            #else
                0,
            #endif
        .use_busy = 
            #ifdef SPI1_USE_BUSY
                1,
            #else
                0,
            #endif
        .use_busy_irq = 
            #ifdef SPI1_USE_BUSY_IRQ 
                1,
            #else
                0,
            #endif
        .busy_irq_mode = 
            #if defined(SPI1_USE_BUSY_IRQ) && defined(SPI1_BUSY_IRQ_MODE) 
                SPI1_BUSY_IRQ_MODE,
            #else
                0,
            #endif
        .use_inp = 
            #ifdef SPI1_USE_INP
                1,
            #else
                0,
            #endif
        .use_inp_irq = 
            #ifdef SPI1_USE_INP_IRQ 
                1,
            #else
                0,
            #endif
        .inp_irq_mode = 
            #if defined(SPI1_USE_INP_IRQ) && defined(SPI1_INP_IRQ_MODE) 
                SPI1_INP_IRQ_MODE,
            #else
                0,
            #endif
        .use_hw_irq = 
            #ifdef SPI1_USE_HW_IRQ 
                1,
            #else
                0,
            #endif
        .baudrate = SPI1_BAUDRATE,
        .datasize =
            #ifdef SPI1_DATASIZE 
                SPI1_DATASIZE,
            #else
                8,
            #endif
    };

    #define NUM0 0
    #define LIST0 
    #ifdef SPI1_USE_MISO_IRQ
        #define NUM1   (NUM0+1)
        #define LIST1  LIST0 SPI1_MISO_IRQ,
    #else
        #define NUM1   NUM0
        #define LIST1 LIST0
    #endif
    #ifdef SPI1_USE_BUSY_IRQ
        #define NUM2   (NUM1+1)
        #define LIST2  LIST1 SPI1_BUSY_IRQ,
    #else
        #define NUM2   NUM1
        #define LIST2 LIST1
    #endif
    #ifdef SPI1_USE_INP_IRQ
        #define NUM3   (NUM2+1)
        #define LIST3  LIST2 SPI1_INP_IRQ,
    #else
        #define NUM3   NUM2
        #define LIST3 LIST2
    #endif
    #if defined(USE_SPI1 ) && defined(SPI1_USE_HW_IRQ)
        #define NUM4   (NUM3+1)
        #define LIST4  LIST3 SPI1_HW_IRQ,
    #else
        #define NUM4   NUM3
        #define LIST4 LIST3
    #endif

    const HW_IrqList irq_spi1 = {
        .num = NUM4,
        .irq = { LIST4 },
    };
    const HW_DeviceType SPIDEV1_DEV = {
        .devName        = SPIDEV1_NAME,
        .devBase        = SPIDEV1_HARDWARE,
        .devGpioAF      = &gpio_spi1,
        .devGpioIO      = NULL,
        .devType        = SPIDEV1_TYPE,
        .devData        = &additional_spi1,
        .devIrqList     = &irq_spi1,
        #if defined(USE_SPI1) && defined(SPI1_USE_DMA) 
            .devDmaTx = &dmatx_spi1,
            .devDmaRx = &dmarx_spi1,
        #else
            .devDmaTx = NULL,
            .devDmaRx = NULL,
        #endif
        .Init           = SPI_Init,
        .DeInit         = SPI_DeInit,
#if SPIDEV1_TYPE == HW_DEVICE_HWSPI
        .OnFrqChange    = SPI_OnFrqChange,
        .AllowStop      = SPI_AllowStop,
#else
        .OnFrqChange    = NULL,
        .AllowStop      = NULL,
#endif
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };

    #undef  NUM0
    #undef  LIST0 
    #undef  NUM1
    #undef  LIST1 
    #undef  NUM2
    #undef  LIST2
    #undef  NUM3
    #undef  LIST3
    #undef  NUM4
    #undef  LIST4

#endif // #if defined(USE_BBSPI1) || defined(USE_SPI1)

#if defined(USE_BBSPI2) || defined(USE_SPI2)
    SpiDataT             SPI2Data;
    SpiHandleT           SPI2Handle = {&SPI2Data, NULL };
    #if defined(USE_SPI2) && defined(SPI2_USE_DMA) 
        static DMA_HandleTypeDef hdma_spi2_tx;
        static DMA_HandleTypeDef hdma_spi2_rx;
        static const HW_DmaType dmatx_spi2 = { &hdma_spi2_tx, SPI2_TX_DMA };
        static const HW_DmaType dmarx_spi2 = { &hdma_spi2_rx, SPI2_RX_DMA };
    #endif
    static const HW_GpioList_AF gpio_spi2 = {
        .gpio = { 
        /* MOSI, SCK, nSEL, MISO, DnC, nRST, BUSY */
            SPI2_MOSI, SPI2_SCK, 
            #ifdef SPI2_USE_NSS
                SPI2_NSS,
            #else
                SPI2_NSEL,
            #endif

            #ifdef SPI2_USE_MISO
                SPI2_MISO,
            #else
                {0},
            #endif
            #ifdef SPI2_USE_DNC
                SPI2_DNC,
            #else
                {0},
            #endif
            #ifdef SPI2_USE_RST
                SPI2_RST,
            #else
                {0},
            #endif
            #ifdef SPI2_USE_BUSY
                SPI2_BUSY,
            #else
                {0},
            #endif
            #ifdef SPI2_USE_INP
                SPI2_INP,
            #else
                {0},
            #endif
        },
        .num = 8, 
    };

    static const SPI_AdditionalDataType additional_spi2 = {
        .mySpiHandle = &SPI2Handle,
        .use_nss =
            #ifdef SPI2_USE_NSS 
                1,
            #else
                0,
            #endif
        .use_miso = 
            #ifdef SPI2_USE_MISO 
                1,
            #else
                0,
            #endif
        .use_miso_irq = 
            #ifdef SPI2_USE_MISO_IRQ 
                1,
            #else
                0,
            #endif
        .use_dnc = 
            #ifdef SPI2_USE_DNC
                1,
            #else
                0,
            #endif
        .use_rst = 
            #ifdef SPI2_USE_RST
                1,
            #else
                0,
            #endif
        .use_busy = 
            #ifdef SPI2_USE_BUSY
                1,
            #else
                0,
            #endif
        .use_busy_irq = 
            #ifdef SPI2_USE_BUSY_IRQ 
                1,
            #else
                0,
            #endif
        .use_inp = 
            #ifdef SPI2_USE_INP
                1,
            #else
                0,
            #endif
        .use_inp_irq = 
            #ifdef SPI2_USE_INP_IRQ 
                1,
            #else
                0,
            #endif
        .use_hw_irq = 
            #ifdef SPI2_USE_HW_IRQ 
                1,
            #else
                0,
            #endif
        .baudrate = SPI2_BAUDRATE,
        .datasize =
            #ifdef SPI2_DATASIZE 
                SPI2_DATASIZE,
            #else
                8,
            #endif
    };
     
    #define NUM0 0
    #define LIST0 
    #ifdef SPI2_USE_MISO_IRQ
        #define NUM1   (NUM0+1)
        #define LIST1  LIST0 SPI2_MISO_IRQ,
    #else
        #define NUM1   NUM0
        #define LIST1 LIST0
    #endif
    #ifdef SPI2_USE_BUSY_IRQ
        #define NUM2   (NUM1+1)
        #define LIST2  LIST1 SPI2_BUSY_IRQ,
    #else
        #define NUM2   NUM1
        #define LIST2 LIST1
    #endif
    #ifdef SPI2_USE_INP_IRQ
        #define NUM3   (NUM2+1)
        #define LIST3  LIST2 SPI2_INP_IRQ,
    #else
        #define NUM3   NUM2
        #define LIST3 LIST2
    #endif
    #if defined(USE_SPI2) && defined(SPI2_USE_HW_IRQ)
        #define NUM4   (NUM3+1)
        #define LIST4  LIST3 SPI2_HW_IRQ,
    #else
        #define NUM4   NUM3
        #define LIST4 LIST3
    #endif

    const HW_IrqList irq_spi2 = {
        .num = NUM4,
        .irq = { LIST4 },
    };
    const HW_DeviceType SPIDEV2_DEV = {
        .devName        = SPIDEV2_NAME,
        .devBase        = SPIDEV2_HARDWARE,
        .devGpioAF      = &gpio_spi2,
        .devGpioIO      = NULL,
        .devType        = SPIDEV2_TYPE,
        .devData        = &additional_spi2,
        .devIrqList     = &irq_spi2,
        #if defined(USE_SPI2) && defined(SPI2_USE_DMA) 
            .devDmaTx = &dmatx_spi2,
            .devDmaRx = &dmarx_spi2,
        #else
            .devDmaTx = NULL,
            .devDmaRx = NULL,
        #endif
        .Init           = SPI_Init,
        .DeInit         = SPI_DeInit,
        .OnFrqChange    = NULL,
#if SPIDEV2_TYPE == HW_DEVICE_HWSPI
        .AllowStop     = SPI_AllowStop,
#else
        .AllowStop     = NULL,
#endif
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };

    #undef  NUM0
    #undef  LIST0 
    #undef  NUM1
    #undef  LIST1 
    #undef  NUM2
    #undef  LIST2
    #undef  NUM3
    #undef  LIST3
    #undef  NUM4
    #undef  LIST5

#endif // #if defined(USE_BBSPI2) || defined(USE_SPI2)

#if defined(USE_BBSPI3) || defined(USE_SPI3)
    SpiDataT             SPI3Data;
    SpiHandleT           SPI3Handle = {&SPI3Data, NULL };
    #if defined(USE_SPI3) && defined(SPI3_USE_DMA) 
        static DMA_HandleTypeDef hdma_spi3_tx;
        static DMA_HandleTypeDef hdma_spi3_rx;
        static const HW_DmaType dmatx_spi3 = { &hdma_spi3_tx, SPI3_TX_DMA };
        static const HW_DmaType dmarx_spi3 = { &hdma_spi3_rx, SPI3_RX_DMA };
    #endif
    static const HW_GpioList_AF gpio_spi3 = {
        /* MOSI, SCK, nSEL, MISO, DnC, nRST, BUSY */
        .gpio = { 
            SPI3_MOSI, SPI3_SCK, 
            #ifdef SPI3_USE_NSS
                SPI3_NSS,
            #else
                SPI3_NSEL,
            #endif
            #ifdef SPI3_USE_MISO
                SPI3_MISO,
            #else
                {0},
            #endif
            #ifdef SPI3_USE_DNC
                SPI3_DNC,
            #else
                {0},
            #endif
            #ifdef SPI3_USE_RST
                SPI3_RST,
            #else
                {0},
            #endif
            #ifdef SPI3_USE_BUSY
                SPI3_BUSY,
            #else
                {0},
            #endif
            #ifdef SPI3_USE_INP
                SPI3_INP,
            #else
                {0},
            #endif
        },
        .num = 8, 
    };

    static const SPI_AdditionalDataType additional_spi3 = {
        .mySpiHandle = &SPI3Handle,
        .use_nss = 
            #ifdef SPI3_USE_NSS 
                1,
            #else
                0,
            #endif
        .use_miso = 
            #ifdef SPI3_USE_MISO 
                1,
            #else
                0,
            #endif
        .use_miso_irq = 
            #ifdef SPI3_USE_MISO_IRQ 
                1,
            #else
                0,
            #endif
        .use_dnc = 
            #ifdef SPI3_USE_DNC
                1,
            #else
                0,
            #endif
        .use_rst = 
            #ifdef SPI3_USE_RST
                1,
            #else
                0,
            #endif
        .use_busy = 
            #ifdef SPI3_USE_BUSY
                1,
            #else
                0,
            #endif
        .use_busy_irq = 
            #ifdef SPI3_USE_BUSY_IRQ 
                1,
            #else
                0,
            #endif
        .busy_irq_mode = 
            #if defined(SPI3_USE_BUSY_IRQ) && defined(SPI3_BUSY_IRQ_MODE) 
                SPI3_BUSY_IRQ_MODE,
            #else
                0,
            #endif
        .use_inp = 
            #ifdef SPI3_USE_INP
                1,
            #else
                0,
            #endif
        .use_inp_irq = 
            #ifdef SPI3_USE_INP_IRQ 
                1,
            #else
                0,
            #endif
        .inp_irq_mode = 
            #if defined(SPI3_USE_INP_IRQ) && defined(SPI3_INP_IRQ_MODE) 
                SPI3_INP_IRQ_MODE,
            #else
                0,
            #endif
        .use_hw_irq = 
            #ifdef SPI3_USE_HW_IRQ 
                1,
            #else
                0,
            #endif
        .baudrate = SPI3_BAUDRATE,
        .datasize =
            #ifdef SPI3_DATASIZE 
                SPI3_DATASIZE,
            #else
                8,
            #endif
    };

    #define NUM0 0
    #define LIST0 
    #ifdef SPI3_USE_MISO_IRQ
        #define NUM1   (NUM0+1)
        #define LIST1  LIST0 SPI3_MISO_IRQ,
    #else
        #define NUM1   NUM0
        #define LIST1 LIST0
    #endif
    #ifdef SPI3_USE_BUSY_IRQ
        #define NUM2   (NUM1+1)
        #define LIST2  LIST1 SPI3_BUSY_IRQ,
    #else
        #define NUM2   NUM1
        #define LIST2 LIST1
    #endif
    #ifdef SPI3_USE_INP_IRQ
        #define NUM3   (NUM2+1)
        #define LIST3  LIST2 SPI3_INP_IRQ,
    #else
        #define NUM3   NUM2
        #define LIST3 LIST2
    #endif
    #if defined(USE_SPI3 ) && defined(SPI3_USE_HW_IRQ)
        #define NUM4   (NUM3+1)
        #define LIST4  LIST3 SPI3_HW_IRQ,
    #else
        #define NUM4   NUM3
        #define LIST4 LIST3
    #endif

    const HW_IrqList irq_spi3 = {
        .num = NUM4,
        .irq = { LIST4 },
    };
    const HW_DeviceType SPIDEV3_DEV = {
        .devName        = SPIDEV3_NAME,
        .devBase        = SPIDEV3_HARDWARE,
        .devGpioAF      = &gpio_spi3,
        .devGpioIO      = NULL,
        .devType        = SPIDEV3_TYPE,
        .devData        = &additional_spi3,
        .devIrqList     = &irq_spi3,
        #if defined(USE_SPI3) && defined(SPI3_USE_DMA) 
            .devDmaTx = &dmatx_spi3,
            .devDmaRx = &dmarx_spi3,
        #else
            .devDmaTx = NULL,
            .devDmaRx = NULL,
        #endif
        .Init           = SPI_Init,
        .DeInit         = SPI_DeInit,
#if SPIDEV3_TYPE == HW_DEVICE_HWSPI
        .OnFrqChange    = SPI_OnFrqChange,
        .AllowStop      = SPI_AllowStop,
#else
        .OnFrqChange    = NULL,
        .AllowStop      = NULL,
#endif
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };

    #undef  NUM0
    #undef  LIST0 
    #undef  NUM1
    #undef  LIST1 
    #undef  NUM2
    #undef  LIST2
    #undef  NUM3
    #undef  LIST3
    #undef  NUM4
    #undef  LIST4

#endif // #if defined(USE_BBSPI3) || defined(USE_SPI3)



#endif /* #if defined(USE_BBSPI1) || defined(USE_BBSPI2) || defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4) */

/**
  * @}
  */


