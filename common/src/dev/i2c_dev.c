/**
 ******************************************************************************
 * @file    i2c.c
 * @author  Rainer
 * @brief   I2C device functions. 
 */

/** @addtogroup I2C User Functions
  * @{
  */

#include "config/config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "dev/i2c_dev.h"

#include "system/hw_util.h"
#include "system/dma_handler.h" /**** 004 ****/

#include "config/devices_config.h"
#include "config/i2c_config.h"
#include "debug_helper.h"

/*******************************************************************************************
 * Additional data that will be stored to I2C type hardware devices
 ******************************************************************************************/
#if USE_I2C > 0
#define SCL_IDX   0
#define SDA_IDX   1
      
typedef struct {
    I2cHandleT         *myI2cHandle;
    I2cSpeedModeT      mySpeed;
    I2cClockModeT      myClock; 
} I2C_AdditionalDataType;

typedef enum I2cDmaDirectionEnum {
  I2C_DMA_RX=0,                       // Rx DMA
  I2C_DMA_TX,                         // Tx DMA
} I2cDmaDirectionEnumType;


/* Structure to describe the different clock selections for I2C-devices */
typedef struct {
  I2C_TypeDef   *i2c;
  uint32_t      selector;
  uint32_t      offset;
  uint32_t      selection[I2C_MaxClock];
}I2cClockSourcesT;

/* different I2C Clock speeds we provide timing data for */
#define I2C_ILLEGAL_SPEED_NUM_IDX       0xFF
#define I2C_CORE_SPEED_NUM              5
static const uint8_t I2C_CLOCK[I2C_CORE_SPEED_NUM] = { 8, 16, 24, 48, 80 };

/* 2C Timing Register Values for different I2C modes and different core speeds */
static const uint32_t I2c_timing[I2C_CORE_SPEED_NUM][I2c_MaxMode] = {
  /*         Standard(100),  Fast(400), FastPlus(1000kHz) */
  /* 08MHz */ { 0x00201D2C, 0x0010020B, 0x00000001 },
  /* 16MHz */ { 0x00403D5A, 0x00200817, 0x00100106 },
  /* 24MHz */ { 0x00605D89, 0x00300B27, 0x0020040A },
  /* 48MHz */ { 0x00D0D2FF, 0x00601851, 0x00400819 },
  /* 80MHz */ { 0x10B0B6CF, 0x00B0298B, 0x0070122A },
};

/* Encode the RCC_PeriphCLKInitTypeDef settings for all I2C devices */
static const I2cClockSourcesT i2cClocks[] = {
#if defined(I2C1) && defined(USE_I2C1)
    { I2C1, RCC_PERIPHCLK_I2C1, offsetof(RCC_PeriphCLKInitTypeDef,I2c1ClockSelection), {  RCC_I2C1CLKSOURCE_PCLK1,  RCC_I2C1CLKSOURCE_SYSCLK, RCC_I2C1CLKSOURCE_HSI } },
#endif
#if defined(I2C2) && defined(USE_I2C2)
    { I2C2, RCC_PERIPHCLK_I2C2, offsetof(RCC_PeriphCLKInitTypeDef,I2c2ClockSelection),{  RCC_I2C2CLKSOURCE_PCLK1,  RCC_I2C2CLKSOURCE_SYSCLK, RCC_I2C2CLKSOURCE_HSI } },
#endif
#if defined(I2C3) && defined(USE_I2C3)
    { I2C3, RCC_PERIPHCLK_I2C3, offsetof(RCC_PeriphCLKInitTypeDef,I2c3ClockSelection),{  RCC_I2C3CLKSOURCE_PCLK1,  RCC_I2C3CLKSOURCE_SYSCLK, RCC_I2C3CLKSOURCE_HSI } },
#endif
#if defined(I2C4) && defined(USE_I2C4)
    { I2C4, RCC_PERIPHCLK_I2C4, offsetof(RCC_PeriphCLKInitTypeDef,I2c4ClockSelection),{  RCC_I2C4CLKSOURCE_PCLK1,  RCC_I2C4CLKSOURCE_SYSCLK, RCC_I2C4CLKSOURCE_HSI } },
#endif
};

/* forward declarations ------------------------------------------------------------*/
bool I2C_Init(const HW_DeviceType *self);
void I2C_DeInit(const HW_DeviceType *self);


static I2C_AdditionalDataType * I2C_GetAdditionalData(const HW_DeviceType *self)
{
    return (I2C_AdditionalDataType *)(self->devData);
}


/* Private or driver functions ------------------------------------------------------*/

static void I2cResetMyHandle ( I2cHandleT *ihandle ) 
{
    memset(ihandle, 0, sizeof(I2cHandleT) );
}

static void I2c_SetClockSource(const HW_DeviceType *self)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    const I2cClockSourcesT *p   = i2cClocks;
    I2C_TypeDef *myI2c          = (I2C_TypeDef *)(self->devBase);
    I2cClockModeT myClockSrc    = I2C_GetAdditionalData(self)->myClock;
    
    assert ( myClockSrc < I2C_MaxClock );
    for ( uint32_t i = 0; i < sizeof(i2cClocks); i++ ) {
        if ( myI2c == p->i2c ) {
            PeriphClkInit.PeriphClockSelection = p->selector; 
            *((uint32_t*)((uint8_t *)(&PeriphClkInit)+p->offset)) = p->selection[myClockSrc];
            if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
                DEBUG_PRINTF("Error in ClockInit of %s\n", self->devName); 
            }
        return;
       }
        p++;
    }
    DEBUG_PRINTF("Error: No clock config for %s\n", self->devName); 
}

static void I2c_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    I2c_SetClockSource(self);
    /* Enable I2C clock */
    HW_SetHWClock((I2C_TypeDef *)self->devBase, 1);

    /*##-2- Configure SCL and SDA pins ##########################################*/  

    /* I2C SCL and SDA pin configuration  */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    uint32_t devIdx = GetDevIdx(self);
    GpioAFInitAll(devIdx, self->devGpioAF, &GPIO_InitStruct);
}

static void I2c_GPIO_DeInit(const HW_DeviceType *self)
{

    /* Disable GPIO Pins */
    uint32_t devIdx = GetDevIdx(self);
    GpioAFDeInitAll(devIdx, self->devGpioAF);
    /* Disable I2C clock */
    HW_SetHWClock((I2C_TypeDef *)self->devBase, 0);
}

/* 
 * the I2C Timing Register Value is calculated with the following restrictions
 * - Analog filter enabled, digital filter index 0
 * - rise and fall time 40ns independ from i2c speed mode
 */

/* return the I2C core speed_num array index for a given pclk value */
/* 0xff is returned on error */
static uint8_t I2c_GetSpeedIdx ( uint32_t i2cclk )
{
  uint8_t s = i2cclk / 1000000;
  uint8_t i;
  for ( i = 0; i < I2C_CORE_SPEED_NUM; i++ ) {
     if (s == I2C_CLOCK[i]) return i;
  }
  
  return I2C_ILLEGAL_SPEED_NUM_IDX;
}

static uint32_t I2c_GetSpeed ( I2cClockModeT mClock )
{
    uint32_t ret;
    switch ( mClock ) {
        case I2cClock_SYSCLK:
            ret = HAL_RCC_GetSysClockFreq();
            break;
        case I2cClock_HSI:
            ret = HAL_RCC_GetHCLKFreq();
            break;
        case I2cClock_PCLK1:
        default:
            ret = HAL_RCC_GetPCLK1Freq();
    }
    return ret;
}

static uint32_t I2c_GetTiming ( const HW_DeviceType *self )
{
    I2cSpeedModeT mMode = I2C_GetAdditionalData(self)->mySpeed;
    I2cClockModeT mClockSrc = I2C_GetAdditionalData(self)->myClock;
  if ( mMode >= I2c_MaxMode ) {
      DEBUG_PRINTF("Illegal speed mode for %s\n",self->devName);
      return 0;
  }
  
  if ( mClockSrc >= I2C_MaxClock ) {
      DEBUG_PRINTF("Illegal clock source for %s\n",self->devName);
      return 0;
  }

  uint32_t i2cclk = I2c_GetSpeed(mClockSrc);
  uint8_t sidx = I2c_GetSpeedIdx(i2cclk);
  if ( sidx == I2C_ILLEGAL_SPEED_NUM_IDX ) {
      DEBUG_PRINTF("No Timing data for I2C clock of %d\n",i2cclk);
      return 0;
  }

  return I2c_timing[sidx][mMode];
}


static void I2cDmaChannelInit(I2cHandleT *myi2c, const HW_DmaType *dma, I2cDmaDirectionEnumType dmadir )
{

  DMA_HandleTypeDef *hdma = dma->dmaHandle;

  HW_DMA_HandleInit(hdma, dma, myi2c );

  switch ( dmadir ) 
  {
    case I2C_DMA_RX:
      hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
      myi2c->hI2c.hdmarx = dma->dmaHandle;
      break;
    case I2C_DMA_TX:
      hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      myi2c->hI2c.hdmatx = dma->dmaHandle;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
  }

  HAL_DMA_Init(hdma);

  return;
}



static bool I2C_MspInitInt(const HW_DeviceType *self)
{
    const HW_IrqType *irq;
    DMA_HandleTypeDef *hdma;

    /* Init GPIO and Clocks */
    I2c_GPIO_Init(self);

    /* Configure the NVIC, enable interrupts */
    HW_SetAllIRQs(self->devIrqList, true);

    /* Enable DMA, if specified */            
    if ( self->devDmaRx || self->devDmaTx ) {        

      // Take the first interrupt to copy prio and subprio to dma channel interrupts
      irq = self->devIrqList->irq;
      
      if (  self->devDmaRx ) {
        /**** 004 ****/
        hdma = HW_DMA_RegisterDMAChannel(self->devDmaRx);
        if ( !hdma ) return false;
        I2cDmaChannelInit( I2C_GetAdditionalData(self)->myI2cHandle, self->devDmaRx, I2C_DMA_RX );
        HW_DMA_SetAndEnableChannelIrq(hdma->Instance, irq->irq_prio, irq->irq_subprio);
      }
      
      if (self->devDmaTx ) {
        /**** 004 ****/
        hdma = HW_DMA_RegisterDMAChannel(self->devDmaTx);
        if ( !hdma ) return false;
        I2cDmaChannelInit( I2C_GetAdditionalData(self)->myI2cHandle, self->devDmaTx, I2C_DMA_TX );
        HW_DMA_SetAndEnableChannelIrq(hdma->Instance, irq->irq_prio, irq->irq_subprio);
      }
    } // if DMA

    return true;
}


static void I2C_MspDeInitInt(const HW_DeviceType *self)
{

    /*Reset peripherals */
    HW_Reset((I2C_TypeDef *)self->devBase );

    I2c_GPIO_DeInit(self);

    /* Disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    /**** 004 **** De-Initialize the Rx part  */
    if(self->devDmaRx) HW_DMA_HandleDeInit(self->devDmaRx->dmaHandle);

    /**** 004 **** De-Initialize the Tx part  */
    if(self->devDmaTx) HW_DMA_HandleDeInit(self->devDmaTx->dmaHandle);
}

/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, ...
 *****************************************************************************/
bool I2C_AllowStop(const HW_DeviceType *self)
{
    I2C_TypeDef *i2c = (I2C_TypeDef*)self->devBase;
    // RHB todo
    UNUSED(i2c);
    return 0;
}
/* Static configurations ---------------------------------------------------------*/

#if defined(I2C1) && defined(USE_I2C1)
    I2cHandleT I2C1Handle;
    #ifdef I2C1_USE_DMA
      static DMA_HandleTypeDef hdma_i2c1_tx;
      static DMA_HandleTypeDef hdma_i2c1_rx;
      static const HW_DmaType dmatx_i2c1 = { &hdma_i2c1_tx, I2C1_TX_DMA };
      static const HW_DmaType dmarx_i2c1 = { &hdma_i2c1_rx, I2C1_RX_DMA };
    #endif

    static const HW_GpioList_AF gpio_i2c1 = {
        .gpio = { I2C1_SCL_PIN, I2C1_SDA_PIN },
        .num = 2, 
    };

    static const I2C_AdditionalDataType additional_i2c1 = {
        &I2C1Handle,
        I2C1_SPEED,
        I2C1_CLKSOURCE,
    };

    #ifdef I2C1_USE_IRQ
        static const HW_IrqList irq_i2c1 = {
            .num = 2,
            .irq = { I2C1_EV_IRQ, I2C1_ER_IRQ, },
        };
    #endif

    const HW_DeviceType HW_I2C1 = {
        .devName        = "I2C1",
        .devBase        = I2C1,
        .devGpioAF        = &gpio_i2c1,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_I2C,
        .devData        = &additional_i2c1,
        .devIrqList     = 
        #if defined(I2C1_USE_IRQ)
            &irq_i2c1,
        #else
            NULL,
        #endif
        #if defined(I2C1_USE_DMA)
            .devDmaTx = &dmatx_i2c1,
            .devDmaRx = &dmarx_i2c1,
        #else
            .devDmaTx = NULL,
            .devDmaRx = NULL,
        #endif
        .Init           = I2C_Init,
        .DeInit         = I2C_DeInit,
        .OnFrqChange    = I2C_Init,
        .AllowStop     = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(I2C2) && defined(USE_I2C2)
    I2cHandleT I2C2Handle;
    #ifdef I2C2_USE_DMA
      static DMA_HandleTypeDef hdma_i2c2_tx;
      static DMA_HandleTypeDef hdma_i2c2_rx;
      static const HW_DmaType dmatx_i2c2 = { &hdma_i2c2_tx, I2C2_TX_DMA };
      static const HW_DmaType dmarx_i2c2 = { &hdma_i2c2_rx, I2C2_RX_DMA };
    #endif

    static const HW_GpioList_AF gpio_i2c2 = {
        .gpio = { I2C2_SCL_PIN, I2C2_SDA_PIN },
        .num = 2, 
    };

    static const I2C_AdditionalDataType additional_i2c2 = {
        &I2C2Handle,
        I2C2_SPEED,
        I2C2_CLKSOURCE,
    };

    #ifdef I2C2_USE_IRQ
        static const HW_IrqList irq_i2c2 = {
            .num = 2,
            .irq = { I2C2_EV_IRQ, I2C2_ER_IRQ, },
        };
    #endif

    const HW_DeviceType HW_I2C2 = {
        .devName        = "I2C2",
        .devBase        = I2C2,
        .devGpioAF        = &gpio_i2c2,
        .devGpioIO        = NULL,
        .devType        =  HW_DEVICE_I2C,
        .devData        = &additional_i2c2,
        .devIrqList     = 
        #if defined(I2C2_USE_IRQ)
            &irq_i2c2,
        #else
            NULL,
        #endif
        #if defined(I2C2_USE_DMA)
            .devDmaTx = &dmatx_i2c2,
            .devDmaRx = &dmarx_i2c2,
        #else
            .devDmaTx = NULL,
            .devDmaRx = NULL,
        #endif
        .Init           = I2C_Init,
        .DeInit         = I2C_DeInit,
        .OnFrqChange    = I2C_Init,
        .AllowStop      = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(I2C2) && defined(USE_I2C2)
    I2cHandleT I2C2Handle;
#endif
#if defined(I2C3) && defined(USE_I2C3)
    I2cHandleT I2C3Handle;
#endif

#if defined(I2C4) && defined(USE_I2C4)
    I2cHandleT I2C4Handle;
#endif

const HW_DeviceType *I2c_GetMySelf( I2C_HandleTypeDef *hi2c) 
{
#if defined(I2C1) && defined(USE_I2C1)
    if ( hi2c == &I2C1Handle.hI2c ) return &HW_I2C1;
#endif
#if defined(I2C2) && defined(USE_I2C2)
    if ( hi2c == &I2C2Handle.hI2c ) return &HW_I2C2;
#endif
#if defined(I2C3) && defined(USE_I2C3)
    if ( hi2c == &I2C3Handle.hI2c ) return &HW_I2C3;
#endif
#if defined(I2C4) && defined(USE_I2C4)
    if ( hi2c == &I2C4Handle.hI2c ) return &HW_I2C4;
#endif
    return NULL;
}

void I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    const HW_DeviceType *self = I2c_GetMySelf(hi2c);
    if ( self ) I2C_MspInitInt(self);
} // I2c2MspInit

void I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    const HW_DeviceType *self = I2c_GetMySelf(hi2c);
    if ( self ) I2C_MspDeInitInt(self);
}

bool I2C_Init(const HW_DeviceType *self)
{
    /* Initialize my handle to 'fresh' */
    I2cResetMyHandle(I2C_GetAdditionalData(self)->myI2cHandle);

    I2C_HandleTypeDef* hi2c = &I2C_GetAdditionalData(self)->myI2cHandle->hI2c;

    uint32_t i2c_timing = I2c_GetTiming(self);
    if ( i2c_timing == 0 ) return false;

    hi2c->Instance = (I2C_TypeDef *)self->devBase;
    hi2c->Init.Timing = i2c_timing;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    hi2c->MspInitCallback = I2C_MspInit;
    hi2c->MspDeInitCallback = I2C_MspDeInit;
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
      /* Initialization Error */
      Error_Handler(__FILE__, __LINE__);
    }
    /** Configure Analogue filter 
    */
    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
      /* Initialization Error */
      Error_Handler(__FILE__, __LINE__);
    }
    /** Configure Digital filter 
    */
    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK) {
      /* Initialization Error */
      Error_Handler(__FILE__, __LINE__);
    }
    
  return true;
}

void I2C_DeInit(const HW_DeviceType *self)
{
    I2C_MspDeInitInt(self);
}

bool Scan_I2c( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);

  printf("Scanning I2C bus:");
  HAL_StatusTypeDef result;
  uint8_t i;
  for (i=0x01; i<0x78 ; i++){
    if ( ( i & 0b111 ) == 1 ) printf("\n 0x%02X  ", i );
    /*
     * the HAL wants a left aligned i2c address
     * &hi2c1 is the handle
     * (uint16_t)(i<<1) is the i2c address left aligned
     * retries 2
     * timeout 2
     */
    result = HAL_I2C_IsDeviceReady(&USER_I2C_HANDLE.hI2c, (uint16_t)(i<<1), 2, 2);
    if (result != HAL_OK) { // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
      printf(" ...."); // No ACK received at that address
    }
    if (result == HAL_OK) {
      printf(" 0x%02X", i); // Received an ACK at that address
    }
  }

  printf("\n");

  return true;

}
#endif // USE_I2C



