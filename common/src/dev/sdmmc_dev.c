/*
 ******************************************************************************
 * @file    sdmmc_dev.c
 * @author  Rainer
 * @brief  SDMMC hardware wrapped into HW_Device
 *
 *****************************************************************************/

/** @addtogroup SDMMC
  * @{
   
*/
#include "config/config.h"

#if USE_SDMMC > 0 

/* Debug ------------------------------------------------------------------------*/
#define DSBUG_SDMMC          1

#include "config/devices_config.h"
#include "config/sdmmc_config.h"
#include "dev/sdmmc_dev.h"
#include "system/hw_util.h"

#include "error.h"
#include "task/minitask.h"

#include "debug_helper.h"


/* Private typedef --------------------------------------------------------------*/

typedef struct {
    SdmmcHandleT              *mySdmmcHandle;            /* my associated handle */
    uint8_t                   myDataLinesNum;            /* number of used Data lines */             
} Sdmmc_AdditionalDataType;

/* Private or driver functions ------------------------------------------------------*/
static  SdmmcHandleT * Sdmmc_GetMyHandle(const HW_DeviceType *self)
{
    return ((Sdmmc_AdditionalDataType *)(self->devData))->mySdmmcHandle;
}




/******************************************************************************
 * Clear the entire CanHandleT structure
 *****************************************************************************/
static void SdmmcResetMyHandle ( SdmmcHandleT *handle ) 
{
    memset(handle, 0, sizeof(SdmmcHandleT) );
}


/**************************************************************************************
 * Some Devices support different clock sources for FSMC. On configurable FMC Clock   *
 * sources, we assume that FMC Clock source is HCLK. In any case make sure, that      *   
 * Fmc_SetClockSource and Fmc_GetClockSpeed() will match                              *
 *************************************************************************************/
#if defined(STM32L4_FAMILY)
    /* STM32L4xx has no clock mux for FMC device */
    #define Fmc_SetClockSource(a)           (true)
    #define Fmc_GetClockSpeed()             HAL_RCC_GetHCLKFreq()
#elif defined(STM32H7_FAMILY)
    /* STM32H7xx: Sdmmc may be clocked by PLL1Q (default) or PLL2R */
    
    /*********************************************************************************
     * Get the SDMMC clock sourcte, which is either either PLL1Q or PLL2R
     * @returns  0 - SDMMC clk source is PLL1Q; 1 - SDMMC clk source is PLL2R
     *********************************************************************************/
    static uint32_t Sdmmc_GetClockSource(void)
    {
        return ( RCC->D1CCIPR & RCC_D1CCIPR_SDMMCSEL ) ? 1: 0 ;
    }

    /*********************************************************************************
     * Set the SDMMC clock sourcte to either PLL1Q or PLL2R
     * @param clksrc - 0 = PLL1Q; 1=PLL2R
     *********************************************************************************/
    static bool Sdmmc_SetClockSource(const void *hw, uint32_t clksrc)
    {
      UNUSED(hw);
      if ( clksrc ) 
        SET_BIT(RCC->D1CCIPR, RCC_D1CCIPR_SDMMCSEL);
      else
        CLEAR_BIT(RCC->D1CCIPR, RCC_D1CCIPR_SDMMCSEL);
      RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

      /* FSMC has to be operated with HCLK. Routines, which will set      */
      /* FMC timing constants, will call Fmc_GetClockSpeed() to determine */
      /* the current clock speed                                          */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC; 
      PeriphClkInit.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for SDMMC");
        return false;
      }

      return true;
    }


    /*********************************************************************************
     * Set the SDMMC clock speed by reprogramming the actual clock source
     * @param xxxkhz - desired SDMMC input clock speed in kHz
     * @note If you want to change the Clock source, change it by "Sdmmc_SetClockSource"
     *       before you set the speed
     *********************************************************************************/
    static bool Sdmmc_SetClockSpeed(const void *hw, uint32_t xxxkhz)
    {
      UNUSED(hw);

      return true;
    }

    /*********************************************************************************
     * Return the SDMMC clock speed 
     * The effective clock speed is affected by the SDMMC IP input speed from either 
     *********************************************************************************/
    static bool Sdmmc_GetClockSpeed(const void *hw, uint32_t xxxkhz)
    {
      UNUSED(hw);

      return true;
    }

#else 
    #error "No FMC clock assignment defined"
#endif


#if DEBUG_MODE >  0


#endif



/*
 * Init or DeInit Clock / clocksource 
 */
static bool Sdmmc_ClockInit(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !Sdmmc_SetClockSource(self->devBase, 0) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock( (void*)self->devBase, bDoInit );
    return true;
}

static void Sdmmc_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    uint32_t devIdx = GetDevIdx(self);
    GpioAFInitAll(devIdx, self->devGpioAF, &GPIO_InitStruct);
}



///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void Sdmmc_DeInit(const HW_DeviceType *self)
{
    SdmmcHandleT *myHandle = Sdmmc_GetMyHandle(self);

    /* DeInit GPIO */
    GpioAFDeInitAll(GetDevIdx(self),self->devGpioAF);
  
    /* disable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* To leave a clean state, reset hardware */
    HW_Reset((void *)self->devBase);

    /* disable SDMMC clock */
    Sdmmc_ClockInit(self, false );
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool Sdmmc_Init(const HW_DeviceType *self)
{
    SdmmcHandleT            *myHandle = Sdmmc_GetMyHandle(self);
    SD_HandleTypeDef        *hsd;
    Sdmmc_GPIO_Init(self);
    if ( !Sdmmc_ClockInit(self, true ) ) return false;

    HW_Reset((void *)self->devBase);
    SdmmcResetMyHandle(myHandle);

    /* Configure the NVIC, enable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);

    hsd                           = &myHandle->halHandle;
    hsd->Instance                 = (SD_TypeDef *)self->devBase;
    hsd->Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd->Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd->Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
#if  USE_SD_TRANSCEIVER > 0
    hsd->Init.TranceiverPresent   = SDMMC_TRANSCEIVER_PRESENT;
#endif
    #if ( USE_SD_HIGH_PERFORMANCE > 0 )
        hsd->Init.ClockDiv            = SDMMC_HSpeed_CLK_DIV;
    #else
        hsd->Init.ClockDiv            = SDMMC_INIT_CLK_DIV;
    #endif

  /* HAL SD initialization   */
  if(HAL_SD_Init(hsd) != HAL_OK) return false;

  HAL_SD_CardInfoTypeDef ci;
  HAL_SD_GetCardInfo ( hsd, &ci );
  
  DBG_dump_uint32_hex_dec ("Card type", ci.CardType );
  DBG_dump_uint32_hex_dec ("Card vers", ci.CardVersion );
  DBG_dump_uint32_hex_dec ("Class",     ci.Class );
  DBG_dump_uint32_hex_dec ("Rel. addr", ci.RelCardAdd );
  DBG_dump_uint32_hex_dec ("Num Blks",  ci.BlockNbr );
  DBG_dump_uint32_hex_dec ("Blocksize", ci.BlockSize );
  DBG_dump_uint32_hex_dec ("Capacity",  ci.LogBlockNbr );
  DBG_dump_uint32_hex_dec ("Log. blks", ci.LogBlockSize );
  DBG_dump_uint32_hex_dec ("Speed",     ci.CardSpeed );

  return true;
}

/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool Sdmmc_OnFrqChange(const HW_DeviceType *self)
{
    SdmmcHandleT *myHandle = Sdmmc_GetMyHandle(self);

    return true;
}

/******************************************************************************
 * Callback _before_ going to sleep:
 * Put all dynamic memories into self refresh mode
 *****************************************************************************/
bool Sdmmc_OnSleep( const HW_DeviceType *self)
{
    return true;
}

/******************************************************************************
 * Callback _after_ wakeup:
 * Put all dynamic memories back into normal(auto) refresh mode
 *****************************************************************************/
bool Sdmmc_OnWakeup( const HW_DeviceType *self)
{
    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if USE_SDMMC1 > 0 
    SdmmcHandleT  Sdmmc1Handle;

    #if defined( SDMMC1_USE_IRQ )
    const HW_IrqList irq_sdmmc1 = {
        .num = 1,
        .irq = {SDMMC1_IRQ },
    };
    #endif

    /* 16 or 32 Data pins, up to 26 Addr pins, FMC_CTL_MAX control pins */
    static const HW_GpioList_AF gpio_sdmmc1 = {
        .num  = 
            #if defined(USE_SDMMC1_8LINES)
                2+8,
            #else
                2+4,
            #endif
        .gpio = { 
            SDMMC1_CK, SDMMC1_CMD, 
            SDMMC1_D0, SDMMC1_D1, SDMMC1_D2, SDMMC1_D3,
            SDMMC1_D4, SDMMC1_D5, SDMMC1_D6, SDMMC1_D7,
        }
    };


    static const Sdmmc_AdditionalDataType additional_sdmmc1 = {
        .mySdmmcHandle  = &Sdmmc1Handle,
        .myDataLinesNum = 
            #if defined(USE_SDMMC1_8LINES)
                8,
            #else
                4,
            #endif
    };


    const HW_DeviceType HW_SDMMC1 = {
        .devName        = "SDMMC1",
        .devBase        = SDMMC1,
        .devGpioAF      = &gpio_sdmmc1,
        .devType        = HW_DEVICE_SDMMC,
        .devData        = &additional_sdmmc1,
        .devIrqList     = 
            #if defined(SDMMC1_USE_IRQ) 
                &irq_fmc,
            #else
                NULL,
            #endif
        // SDMMC IP has its own internal DMA controller, so these handles should be NULL
        .devDmaRx       = NULL, 
        .devDmaTx       = NULL, 
        .Init           = Sdmmc_Init,
        .DeInit         = Sdmmc_DeInit,
        .OnFrqChange    = Sdmmc_OnFrqChange,
        .AllowStop      = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
    #endif

#if 0
void SDMMC_PostInit( const HW_DeviceType *self, void *args)
{
    UNUSED(self); UNUSED(args);

}
#endif


#endif /* if USE_SDMMC > 0 */


/**
  * @}
  */


