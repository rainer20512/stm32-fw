/*
 ******************************************************************************
 * @file    xspi_dev.c
 * @author  Rainer
 * @brief   QUADSPI or OCTOSPI hardware wrapped into HW_Device
 *
 *****************************************************************************/

/** @addtogroup QUADSPI
  * @{
   
*/
#include "config/config.h"

#include "hardware.h"

#if USE_QSPI > 0 || USE_OSPI > 0

#include "config/devices_config.h"

#if USE_OSPI > 0
    #include "config/ospi_config.h"
#else
    #include "config/qspi_config.h"
#endif

#include "error.h"
#include "task/minitask.h"
#include "system/profiling.h"
#include "system/hw_util.h"
#include "system/dma_handler.h" /**** 004 ****/
#include "system/clockconfig.h"
#include "system/pll.h"
#include "dev/hw_device.h"
#include "dev/devices.h"

#include "log.h"

#include "dev/xspi/flash_interface.h"
#include "dev/xspi/xspi_lowlevel.h"
#include "dev/xspi/xspi_helper.h"

extern  NOR_FlashT flash_interface;

#if DEBUG_MODE > 0
    char* XSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize);
    void  XSpecific_DumpStatusInternal(XSpiHandleT *myHandle);
#endif



#if USE_OSPI > 0
    /* Currently the driver cannot handle both OCTOSPI devices in parallel */
    #if USE_OSPI1 > 0 && USE_OSPI2 > 0
        #error "Cannot handle OCTOSPI1 and OCTOSPI2 both active"
    #endif
    #if USE_OSPI1 > 0
        #define     XSpiStr                         "OSPI1"
        #define     XSPI_USE_IRQ                    OSPI1_USE_IRQ
        #define     XSPI_USE_DMA                    OSPI1_USE_DMA
    #else
        #define     XSpiStr                         "OSPI2"
        #define     XSPI_USE_IRQ                    OSPI2_USE_IRQ
        #define     XSPI_USE_DMA                    OSPI2_USE_DMA
    #endif
#else
        #define     XSpiStr                         "QSPI"
        #define     XSPI_USE_IRQ                    QSPI1_USE_IRQ
        #define     XSPI_USE_DMA                    QSPI1_USE_DMA
#endif


/* My macros --------------------------------------------------------------------*/

#define DEBUG_XSPI                  2

/* Private typedef --------------------------------------------------------------*/
typedef enum XSpiDmaDirectionEnum {
  XSPI_DMA_RD=0,                      // Read DMA
  XSPI_DMA_WR,                        // Write DMA
} XSpiDmaDirectionEnumType;

#if USE_OSPI > 0
    typedef enum OSpiSizeEnum {
        OSPI_QUAD=4,                     /* OCTOSPI used in QuadSpi-Mode */   
        OSPI_OCTO=8,                     /* OCTOSPI used in OctoSpi-Mode */   
    } OSpiSizeEnumType;
#endif

typedef struct {
    XSpiHandleT         *myXSpiHandle;   /* my associated handle */
    uint32_t            defaultSpeed;    /* default speed in MHz */
#if defined(STM32H7_FAMILY) || defined(STM32L4PLUS_FAMILY)           
    uint32_t            clkSource;       /* clock source as defined in stm32h7xx_hal_rcc.h */
#endif
    NOR_FlashT          *interface;      /* my flash chip interface */
#if USE_OSPI > 0 
    OSpiSizeEnumType    ospiMode;        /* currently QUADSPI and OCTOSPI mode are impelemted */
    uint8_t             bUseDQS;         /* true, iff DQS signal is active/used  */       
#endif
} XSpi_AdditionalDataType;



/* Forward declarations -------------------------------------------------------------*/


/* Private or driver functions ------------------------------------------------------*/
static XSpi_AdditionalDataType * XSpi_GetAdditionalData(const HW_DeviceType *self)
{
    return (XSpi_AdditionalDataType *)(self->devData);
}

#if 0 /* currently not used */
/*************************************************************************************
 * Initialize the dma channel. This has to be done before every read or write, because
 * QUADSPI only has one dma channel
 ************************************************************************************/
static void XSpiDmaChannelInit(const HW_DeviceType *self, XSpiDmaDirectionEnumType dmadir )
{
    const HW_DmaType *dma           = self->devDmaRx;
    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
    XSpiHandleT *myHandle           = adt->myXSpiHandle;
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
    DMA_HandleTypeDef *hdma         = dma->dmaHandle;

    hdma->Instance                 = dma->dmaChannel;
    hdma->Parent                   = myHandle;
    hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma->Init.MemInc              = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma->Init.Mode                = DMA_NORMAL;
    hdma->Init.Priority            = DMA_PRIORITY_HIGH;
    hdma->Init.Request             = dma->dmaRequest;

    switch ( dmadir ) 
    {
    case XSPI_DMA_RD:
      hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
      hxspi->hdma = dma->dmaHandle;
      break;
    case XSPI_DMA_WR:
      hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      hxspi->hdma = dma->dmaHandle;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
    }

    HAL_DMA_Init(hdma);

    return;
}
#endif

static void XSpiGpioInitAF(uint32_t devIdx, const HW_GpioList_AF *gpioaf)
{
    GPIO_InitTypeDef Init;

    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_HIGH;

    GpioAFInitAll(devIdx, gpioaf, &Init );
}

/**************************************************************************************
 * Some Devices support different clock sources for QSPI. Make sure, that             *   
  * XSpiSetClockSource and XGetClockSpeed() will match                                *
 *************************************************************************************/
#if defined(STM32L476xx) || defined(STM32L496xx)
    /* STM32L4xx has no clock mux for QUADSPI device */
    #define XSpiSetClockSource(a)           (true)
    #define XSpiGetClockSpeed(a)             HAL_RCC_GetHCLKFreq()


#elif defined(STM32H7_FAMILY)
    static bool XSpiSetClockSource(const HW_DeviceType *self)
    {
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* QSPI/OSPI has to be operaterd with HCLK. Routines, which will set */
      /* qspi/ospi speed, rely on HCLK as Clock source                     */
      #if defined(STM32L4PLUS_FAMILY)
          /* STM32L4S devices */
          PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI; 
          PeriphClkInit.OspiClockSelection   = RCC_OSPICLKSOURCE_SYSCLK;
      #else
          /* STM32H7 devices */
          PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI; 
          PeriphClkInit.QspiClockSelection   = XSpi_GetAdditionalData(self)->clkSource;
      #endif
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        LOG_FATAL("failed to set CLK source for %s", XSpiStr);
        return false;
      }

      return true;
    }
    static uint32_t XSpiGetClockSpeed(const HW_DeviceType *self)
    {
        switch (XSpi_GetAdditionalData(self)->clkSource) 
        {
            case RCC_QSPICLKSOURCE_D1HCLK:
                return HAL_RCC_GetHCLKFreq();
            case RCC_QSPICLKSOURCE_PLL:
                /* PLL1Q */
                return PLL_GetOutFrq(SYS_PLL1, PLL_LINE_Q)*1000;
#if defined(RCC_QSPICLKSOURCE_PLL2)
            case RCC_QSPICLKSOURCE_PLL2:
                /* PLL2R H7-devices*/
                return PLL_GetOutFrq(SYS_PLL2, PLL_LINE_R)*1000;
#endif
#if defined(RCC_QSPICLKSOURCE_CLKP)
            case RCC_QSPICLKSOURCE_CLKP:
                /* peripheral Clock, H7 devices */
                return GetPerClkFrequency();
#endif
#if defined(RCC_QSPICLKSOURCE_MSI)
            case RCC_QSPICLKSOURCE_MSI:
                /* MSI , L4+ devices */
                return HAL_RCC_MSI_Freq();
#endif
            default:
                LOG_FATAL("failed to retrieve CLK source for %s", XSpiStr);
                return 0;
       }
    }
#elif defined(STM32L4PLUS_FAMILY)
    static bool XSpiSetClockSource(const HW_DeviceType *self)
    {
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* QSPI/OSPI has to be operaterd with HCLK. Routines, which will set */
      /* qspi/ospi speed, rely on HCLK as Clock source                     */
      #if defined(STM32L4PLUS_FAMILY)
          /* STM32L4S devices */
          PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI; 
          PeriphClkInit.OspiClockSelection   = RCC_OSPICLKSOURCE_SYSCLK;
      #else
          /* STM32H7 devices */
          PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI; 
          PeriphClkInit.QspiClockSelection   = XSpi_GetAdditionalData(self)->clkSource;
      #endif
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        LOG_FATAL("failed to set CLK source for %s", XSpiStr);
        return false;
      }

      return true;
    }

    static uint32_t XSpiGetClockSpeed(const HW_DeviceType *self)
    {
        switch (XSpi_GetAdditionalData(self)->clkSource) 
        {
            case RCC_OSPICLKSOURCE_SYSCLK:
                return HAL_RCC_GetHCLKFreq();
            case RCC_OSPICLKSOURCE_PLL:
                /* PLL1Q */
                return PLL_GetOutFrq(SYS_PLL1, PLL_LINE_Q)*1000;
            case RCC_OSPICLKSOURCE_MSI:
                /* MSI , L4+ devices */
                return MSI_VALUE;
            default:
                LOG_FATAL("failed to retrieve CLK source for %s", XSpiStr);
                return 0;
       }
    }
#else 
    #error "No xspi clock assignment defined"
#endif

/*
 * Init or DeInit Clock / clocksource 
 */
static bool XSpiClockInit(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !XSpiSetClockSource(self) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock(( void *)self->devBase, bDoInit);
    return true;
}


/**************************************************************************************
 * Return the flash memories specific geometry data                               *
 *************************************************************************************/
void XSpi_GetGeometry(XSpiHandleT *myHandle, XSpiGeometryT *pInfo)
{
    memcpy(pInfo, &myHandle->geometry, sizeof(XSpiGeometryT) );
}


/**************************************************************************************
 * Change the Qspi clock speed on the fly                                             *
 *************************************************************************************/
bool XSpi_SetSpeed (const HW_DeviceType *self, uint32_t new_clkspeed)
{
    XSpiHandleT *myHandle = XSpi_GetAdditionalData(self)->myXSpiHandle;  
    return XSpiLL_ClockInit(myHandle, XSpiGetClockSpeed(self), new_clkspeed, myHandle->geometry.FlashSize);
}

/**************************************************************************************
 * returns value != 0, iff successfully initialized                                   *
 *************************************************************************************/
bool XSpi_IsInitialized(XSpiHandleT *myHandle)
{
    return myHandle->bIsInitialized;
}


/**************************************************************************************
 * Dump Status and restore DeepSleep-Status afterwards                                *
 *************************************************************************************/
void XSpi_DumpStatus(XSpiHandleT *myHandle)
{
    bool sleep = myHandle->bInDeepPwrDown;
    /* Wake up device, if in deep sleep */
    if ( sleep && XSpi_SetDeepPowerDown(myHandle,false)==-1 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOG_ERROR("XSpi_DumpStatus - Error: Cannot wake up flash device");
        #endif
        return;
    }

    XSpecific_DumpStatusInternal(myHandle);
    
    if ( sleep ) XSpi_SetDeepPowerDown(myHandle, true);
}

/**************************************************************************************
 * Set XSpi read, write/erase and error callbacks                                     *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
void XSpi_SetAsyncCallbacks(XSpiHandleT *myHandle, XSpiCallbackT rdDoneCB, XSpiCallbackT wrDoneCB, XSpiCallbackT errorCB)
{
    myHandle->XSpi_RdDoneCB  = rdDoneCB;
    myHandle->XSpi_WrDoneCB  = wrDoneCB;
    myHandle->XSpi_ErrorCB   = errorCB; 
}

/**************************************************************************************
 * Set the read and write mode to single, dual or quad line access
 *************************************************************************************/
void XSpi_SetRWMode( XSpiHandleT *myHandle, XSPI_RWMode xspi_mode )
{
    if ( xspi_mode >= XSPI_RW_MAXNUM ) return;
    myHandle->myRWmode = xspi_mode;
}

/**************************************************************************************
 * Set or Reset Quad data mode
 *************************************************************************************/
void XSpi_SetQuad (XSpiHandleT *myHandle, bool bEna )
{
    /* Write Enable first */
    if (!XSpiLL_WriteEnable(myHandle, true) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("XSpi_SetQuad: write enable failed");
        #endif
    }
    /* get the quad enable or disable function */
    const NOR_FlashSetterT *rs = myHandle->interface->setter.qe[bEna?0:1];
    assert(rs);
    if (!XSpiLL_Execute (myHandle, rs, NULL ) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("XSpi_SetQuad failed");
        #endif
    }
}



/******************************************************************************
 ******************************************************************************
 * Implementation of the read operation:
 * Transmit the read command ( this is device specific ) and start reading
 * in polling, irq or dma mode   
 ******************************************************************************
 *****************************************************************************/
bool XSpi_ReadOperation(XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size, uint32_t opmode)
{
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
    bool ret;

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->bInDeepPwrDown && XSpi_SetDeepPowerDown(myHandle,false)==-1 ) return false;

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        LOGU_VERBOSE("%s read, area: 0x%08x ... 0x%08x", XSpiStr, ReadAddr, ReadAddr+Size-1);
    #endif

    /* Send the read command */
    if ( !XSpiLL_ReadCMD(myHandle, ReadAddr, Size) ) return false;
    
    switch ( opmode ) {
        case XSPI_MODE_POLL:
            /* Reception of the data in polling mode with timeout */
            #if USE_OSPI > 0
                ret =  HAL_OSPI_Receive(hxspi, pData, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            #else
                ret =  HAL_QSPI_Receive(hxspi, pData, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            #endif
            break;
#if defined(XSPI_USE_IRQ)
        case XSPI_MODE_IRQ:
           /* Reception of the data in interrupt mode*/
            myHandle->bAsyncBusy = true;
            #if USE_OSPI > 0
                ret = HAL_OSPI_Receive_IT(hxspi, pData) == HAL_OK;
            #else
                ret = HAL_QSPI_Receive_IT(hxspi, pData) == HAL_OK;
            #endif
            if ( !ret) myHandle->bAsyncBusy = false;
            break;
#endif
#if defined(XSPI_USE_DMA)
        case XSPI_MODE_DMA:
           /* Reception of the data in DMA mode*/
            myHandle->bAsyncBusy = true;
            #if USE_OSPI > 0
                ret = HAL_OSPI_Receive_DMA(hxspi, pData) == HAL_OK;
            #else
                ret = HAL_QSPI_Receive_DMA(hxspi, pData) == HAL_OK;
            #endif
            if ( !ret) myHandle->bAsyncBusy = false;
            break;
#endif
        default:
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("XSpi_ReadOperation - Error: Mode %d not implemented", opmode);
            #endif
            return false;
    } // switch

    /* report errors, if configured */
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("XSpi_ReadOperation - Error: Receive error in mode %d", opmode);
        #endif
    }

    return ret;
}

bool XSpi_ReadWait  (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return XSpi_ReadOperation(myHandle, pData, ReadAddr, Size, XSPI_MODE_POLL );
}

bool XSpi_ReadIT    (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return XSpi_ReadOperation(myHandle, pData, ReadAddr, Size, XSPI_MODE_IRQ );
}

bool XSpi_ReadDMA   (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return XSpi_ReadOperation(myHandle, pData, ReadAddr, Size, XSPI_MODE_DMA );
}

/******************************************************************************
 ******************************************************************************
 * Implementation of the write operation:
 * Transmit the write command ( this is device specific ) and start writing
 * in polling, irq or dma mode   
 * Since maximum one page can be written with one write operation, the write
 * operation is repeaded as often as possible, until the whole buffer is written
 ******************************************************************************
 *****************************************************************************/
typedef bool ( *XSpi_SM_Fn ) (void);     // Typedef for Statemachine function
typedef union {
    struct {
        uint32_t wrOpmode;             // operation mode of write op
        uint32_t wrState;              // current state of write state machine
        XSpiHandleT *wrHandle;         // current QSpi handle for write op
        uint32_t currWriteAddr;        // current write Address 
        uint32_t currWriteSize;        // current write Size
        uint32_t WriteEndAddr;         // last write Address + 1
        uint8_t  *WriteSource;         // Ptr to the write source
    };
    struct {
        uint32_t erOpmode;             // operation mode of erase op
        uint32_t erState;              // current state of erase state machine
        XSpiHandleT *erHandle;         // current XSpi handle for erase op
        const NOR_FlashCmdT *ecmd;     // associated erase command
        XSpi_SM_Fn erSM;               // currently used Statemachine for erase
        uint32_t currEraseAddr;        // current erase address
        uint32_t EraseAddrInc;         // Address Increment for next erase item
        uint32_t EraseItemCnt;         // number of items to erase
        uint32_t EraseMode;            // Selected erase mode ( Sector/block/chip )
    };
} Qspi_SM_DataT;

static Qspi_SM_DataT smData;
static XSpi_SM_Fn wrSM;               // currently used Statemachine for write
static XSpi_SM_Fn erSM;               // currently used Statemachine for erase

#define GETWRDEVHANDLE()                   (smData.wrHandle)
#define GETWRHALHANDLE()                   (&(smData.wrHandle->hxspi))
#define GETERDEVHANDLE()                   (smData.erHandle)

/*-----------------------------------------------------------------------------
 * Write - Write - Write - Write - Write - Write - Write - Write - Write - Writ
 *---------------------------------------------------------------------------*/

/******************************************************************************
 * Implement a classic loop with four components
 * - Initialisation
 * - Loop body
 * - Change loop variables and check for termination
 * - terminate
 * This is done, because the loop execution is triggered direcly ( in polling mode )
 * or by interrupts ( in IRQ or DMA mode )
 *****************************************************************************/
static bool WriteInit(XSpiHandleT *myHandle,  uint8_t* pData, uint32_t WriteAddr, uint32_t Size, uint32_t opmode, XSpi_SM_Fn stateMachine)
{
    if ( opmode != XSPI_MODE_POLL ) {
        if ( myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("WriteInit - Error: Another Async Op is active");
            #endif
            return false;
        }
        myHandle->bAsyncBusy = true;
    }

    /* check for positive size */
    if ( Size == 0 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_WARN("WriteInit - Writing 0 bytes not allowed");
        #endif
        return false;
    }

    /* Calculation of the size between the write address and the end of the page */
    smData.currWriteSize = myHandle->geometry.ProgPageSize - (WriteAddr % myHandle->geometry.ProgPageSize);

    /* Check if the size of the data is less than the remaining place in the page */
    if (smData.currWriteSize > Size) smData.currWriteSize = Size;

    /* Initialize the current and final write addresses */
    smData.currWriteAddr = WriteAddr;
    smData.WriteEndAddr  = WriteAddr + Size;
    smData.WriteSource   = pData;
    smData.wrOpmode    = opmode;
    smData.wrState     = 0;
    smData.wrHandle    = myHandle;
    wrSM               = stateMachine;
    erSM               = NULL;

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        LOGU_VERBOSE("Write Init, area: 0x%08x ... 0x%08x, written in %d steps", WriteAddr,smData.WriteEndAddr-1, Size/myHandle->geometry.ProgPageSize);
    #endif

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->bInDeepPwrDown && XSpi_SetDeepPowerDown(myHandle,false)==-1 ) return false;

    return true;
}

static void WriteEraseTerminate(bool finalState, uint32_t opmode, XSpiHandleT *myHandle)
{
    if ( opmode != XSPI_MODE_POLL ) {
        if ( !myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("WriteEraseTerminate - Error: Async Op not active");
            #endif
        }
        myHandle->bAsyncBusy = false;
    }
    LOGU_VERBOSE("Write/Erase terminated %s",finalState ? "ok" : "with error");

    /* Activate Callback, if specified */
    if (myHandle->XSpi_WrDoneCB) myHandle->XSpi_WrDoneCB(myHandle);
}


static bool WriteBlock (void )
{
    bool ret;

    LOGU_VERBOSE("Write @0x%08x, Len=%d", smData.currWriteAddr, smData.currWriteSize);
    LOGU_VERBOSE("First Bytes (hex)= %02x, %02x, %02x, %02x, ...", 
                  smData.WriteSource[0], smData.WriteSource[1], smData.WriteSource[2], smData.WriteSource[3]);
    /* Enable write and send write command */
    // if ( !XSpecific_SetupWrite(GETWRHANDLE(), smData.currWriteAddr, smData.currWriteSize) ) return false;

    switch ( smData.wrOpmode ) {
        case XSPI_MODE_POLL:
            /* Transmission of the data in polling mode*/
            #if USE_OSPI > 0
                ret = HAL_OSPI_Transmit(GETWRHANDLE(), smData.WriteSource, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            #else
                ret = HAL_QSPI_Transmit(GETWRHALHANDLE(), smData.WriteSource, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            #endif
            break;
        case XSPI_MODE_IRQ:
            /* Transmission of the data in irq mode*/
            #if USE_OSPI > 0
                ret = HAL_OSPI_Transmit_IT(GETWRHALHANDLE(), smData.WriteSource) == HAL_OK;
            #else
                ret = HAL_QSPI_Transmit_IT(GETWRHALHANDLE(), smData.WriteSource) == HAL_OK;
            #endif
            break;
        case XSPI_MODE_DMA:
            /* Transmission of the data in irq mode*/
            #if USE_OSPI > 0
                ret = HAL_OSPI_Transmit_DMA(GETWRHALHANDLE(), smData.WriteSource) == HAL_OK;
            #else
                ret = HAL_QSPI_Transmit_DMA(GETWRHALHANDLE(), smData.WriteSource) == HAL_OK;
            #endif
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("WriteLoop - Error: Mode %d not implemented", smData.wrOpmode);
            #endif
            return false;
    } // switch

    /* report errors, if configured */
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("WriteLoop - Error: Transmit error in mode %d", smData.wrOpmode);
        #endif
    }

    return true;
}

static bool WaitForWriteDone(void)
{
    bool ret;

    /* Configure automatic polling mode to wait for reset of WIP bit */  
    if ( smData.wrOpmode == XSPI_MODE_POLL ) 
        ret = XSpiLL_WaitForWriteDone(GETWRDEVHANDLE(), XSPI_TIMEOUT_DEFAULT_VALUE);
    else 
        ret = XSpiLL_WaitForWriteDone_IT(GETWRDEVHANDLE());
 
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("QSpi_SpecificWriteWait - Error: Timeout while autopll");
        #endif
        return false;
    }
    
    return true;
}

static bool WriteIncement(XSpiHandleT *myHandle)
{
    /* Update the address and size variables for next page programming */
    smData.currWriteAddr += smData.currWriteSize;
    smData.WriteSource   += smData.currWriteSize;
    smData.currWriteSize = ((smData.currWriteAddr + myHandle->geometry.ProgPageSize) >smData.WriteEndAddr) ? (smData.WriteEndAddr - smData.currWriteAddr) : myHandle->geometry.ProgPageSize;
  
    return smData.currWriteAddr <smData.WriteEndAddr;
}
#define WRSTATE_START       0
#define WRSTATE_WRITEBLOCK  1
#define WRSTATE_WAITFORDONE 2
#define WRSTATE_INCREMENT   3
#define WRSTATE_TERMINATE   4
#define STATE_ERROR         99

#define WR_NEXTSTATE(a)     smData.wrState=a
#define WR_SM_PAUSE(a)      bSmPause=a
static bool WriteSM(void)
{
    bool bSmPause = false;

    while (!bSmPause ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_VERBOSE("WriteSM in state %d", smData.wrState);
        #endif
        switch ( smData.wrState ) {
            case WRSTATE_START: 
                /* Enable Write and Send Write Command - both in one step */
                if ( !XSpiLL_WriteCMD(GETWRDEVHANDLE(), smData.currWriteAddr, smData.currWriteSize) ) goto WriteSmTerminate;
                /* Continue with next state in any case*/
                WR_NEXTSTATE(WRSTATE_WRITEBLOCK);
                WR_SM_PAUSE(false);
                break;
            case WRSTATE_WRITEBLOCK:
                if (!WriteBlock() ) goto WriteSmTerminate;
                WR_NEXTSTATE(WRSTATE_WAITFORDONE);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                WR_SM_PAUSE( smData.wrOpmode != XSPI_MODE_POLL );
                break;
            case WRSTATE_WAITFORDONE:
                if ( !WaitForWriteDone() ) goto WriteSmTerminate;
                WR_NEXTSTATE(WRSTATE_INCREMENT);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                WR_SM_PAUSE( smData.wrOpmode != XSPI_MODE_POLL );
                break;
            case WRSTATE_INCREMENT:
                /* increment the loop counters and check for termination */
                if ( WriteIncement(smData.wrHandle) ) 
                    WR_NEXTSTATE(WRSTATE_WRITEBLOCK);
                else
                    WR_NEXTSTATE(WRSTATE_TERMINATE);
                /* Continue with next state right now*/
                WR_SM_PAUSE(false);
                break;
            case WRSTATE_TERMINATE:
                /* Final State reached without error */
                WriteEraseTerminate(true, smData.wrOpmode, smData.wrHandle);
                return true;
            case STATE_ERROR:
                /* Error state, reached only by error Interrupt; */
                goto WriteSmTerminate;    
            default:
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    LOGU_ERROR("WriteSM - Error: illegal state %d", smData.wrState);
                #endif
                goto WriteSmTerminate;
        } // switch
    } // while
    return true;

WriteSmTerminate:
    // All erroneous exits are handeled here
    WriteEraseTerminate(false, smData.wrOpmode, smData.wrHandle);
    return false;
}

bool XSpi_WriteWait(XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, XSPI_MODE_POLL, WriteSM) ) return false;
    return WriteSM();
}
bool XSpi_WriteIT  (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, XSPI_MODE_IRQ, WriteSM) ) return false;
    return WriteSM();
}
bool XSpi_WriteDMA  (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, XSPI_MODE_DMA, WriteSM) ) return false;
    return WriteSM();
}

/*-----------------------------------------------------------------------------
 * Erase - Erase - Erase - Erase - Erase - Erase - Erase - Erase - Erase - Eras
 *---------------------------------------------------------------------------*/


/******************************************************************************
 * Implement a classic loop with four components
 * - Initialisation
 * - Loop body
 * - Change loop variables and check for termination
 * - terminate
 * This is done, because the loop execution is triggered direcly ( in polling mode )
 * or by interrupts ( in IRQ or DMA mode )
 *****************************************************************************/
static bool EraseInit(XSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numItems, XSpi_EraseMode erasemode, uint32_t opmode, XSpi_SM_Fn stateMachine)
{
    if ( opmode != XSPI_MODE_POLL ) {
        if ( myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("EraseInit - Error: Another Async Op is active");
            #endif
            return false;
        }
        myHandle->bAsyncBusy = true;
    }

    /* check for positive size */
    if ( numItems == 0 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("EraseInit - Erasing 0 items not allowed");
        #endif
        return false;
    }

    const  NOR_EraseModeTypeT *e = erasemode != XSPI_ERASE_ALL ? myHandle->interface->erase.p_erase[erasemode] : myHandle->interface->erase.f_erase;
    if ( e == NULL ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("EraseInit - Erase mode &d not defined");
        #endif
        return false;
    }

    /* Initialize the neccessary erase data */
    smData.currEraseAddr = EraseAddr;
    smData.EraseItemCnt  = ( erasemode == XSPI_ERASE_ALL ? 1 : numItems);
    smData.EraseMode     = erasemode;
    smData.erOpmode      = opmode;
    smData.EraseAddrInc  = e->block_size;
    smData.erState       = 0;
    smData.erHandle      = myHandle;
    smData.ecmd          = &e->cmd;
    erSM                 = stateMachine;
    wrSM                 = NULL;

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        LOGU_VERBOSE("Erase Init, start: 0x%08x, itemsize=%d, items=%d, mode=%d", EraseAddr, smData.EraseAddrInc, smData.EraseItemCnt, erasemode);
    #endif

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->bInDeepPwrDown && XSpi_SetDeepPowerDown(myHandle,false)==-1 ) return false;

    return true;
}

static bool WaitForEraseDone(uint32_t timeout_ms)
{
    bool ret;

    /* Configure automatic polling mode to wait for reset of WIP bit */  
    if ( smData.erOpmode == XSPI_MODE_POLL ) 
        ret = XSpiLL_WaitForWriteDone(GETERDEVHANDLE(), timeout_ms);
    else 
        ret = XSpiLL_WaitForWriteDone_IT(GETERDEVHANDLE());
 
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_ERROR("QSpi_SpecificWait - Error: Timeout while autopoll");
        #endif
        return false;
    }
    
    return true;
}

static bool EraseIncement(void)
{
    /* Update the address and decrement count */
    if ( --smData.EraseItemCnt == 0 ) return false;

    smData.currEraseAddr += smData.EraseAddrInc;
    return true;
}


#define ERSTATE_START       0
#define ERSTATE_WAITFORDONE 1
#define ERSTATE_INCREMENT   2
#define ERSTATE_TERMINATE   3
#define STATE_ERROR         99

#define ER_NEXTSTATE(a)     smData.erState=a
#define ER_SM_PAUSE(a)      bSmPause=a

static bool EraseSM(void)
{
    bool bSmPause = false;
    uint32_t tmo_ms;

    while (!bSmPause ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOGU_VERBOSE("EraseSM in state %d", smData.erState);
        #endif
        switch ( smData.erState ) {
            case ERSTATE_START: 
                /* Enable Write and Send Erase Command - both in one step */
                LOGU_VERBOSE("Erase @0x%08x, mode=%d", smData.currEraseAddr, smData.EraseMode);
                if ( !XSpiLL_Erase(GETERDEVHANDLE(), smData.currEraseAddr, smData.ecmd) ) goto EraseSmTerminate;
                /* Continue with next state in any case*/
                ER_NEXTSTATE(ERSTATE_WAITFORDONE);
                /* Continue with waiting for completion immediately */
                ER_SM_PAUSE( false );
                break;
            case ERSTATE_WAITFORDONE:
                tmo_ms = (smData.ecmd->exec_time+500) / 1000;
                if ( !WaitForEraseDone(tmo_ms) ) goto EraseSmTerminate;
                LOGU_VERBOSE("Erase done");
                ER_NEXTSTATE(ERSTATE_INCREMENT);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                ER_SM_PAUSE( smData.erOpmode != XSPI_MODE_POLL );
                break;
            case ERSTATE_INCREMENT:
                /* increment the loop counters and check for termination */
                if ( EraseIncement() ) 
                    ER_NEXTSTATE(ERSTATE_START);
                else
                    ER_NEXTSTATE(ERSTATE_TERMINATE);
                /* Continue with next state right now*/
                ER_SM_PAUSE(false);
                break;
            case ERSTATE_TERMINATE:
                /* Final State reached without error */
                WriteEraseTerminate(true, smData.erOpmode, smData.erHandle);
                return true;
            case STATE_ERROR:
                /* Error state, reached only by error Interrupt; */
                goto EraseSmTerminate;    
            default:
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    LOGU_ERROR("EraseSM - Error: illegal state %d", smData.erState);
                #endif
                goto EraseSmTerminate;
        } // switch
    } // while
    return true;

EraseSmTerminate:
    // All erroneous exits are handeled here
    WriteEraseTerminate(false, smData.erOpmode, smData.erHandle);
    return false;
}
/******************************************************************************
 * @brief Get different erase sizes and optionally corresponding max erase times
 * @param  myHandle  - XSpi handle
 * @param  Sizes     - return array for different erase sizes, not NULL
 * @param  Max_Times - max erase times[us], may be NULL
 * @retval Number of defined erase elements 
 * @note   Does not include Full Chip erase
 * @note   a returned size of 0 means "not defined"
 * @note   Array "Sizes" and optionally "Max_Times" 
 *****************************************************************************/
uint32_t XSpi_GetEraseData      (XSpiHandleT *myHandle, uint32_t *Sizes, uint32_t *Max_Times)
{
    const  NOR_EraseModeTypeT * const *e = myHandle->interface->erase.p_erase;
    /* clear sizes array */
    memset(Sizes, 0, sizeof(uint32_t)*XSPI_MAX_ERASE_MODE);

    uint32_t i=0;
    while ( i < XSPI_MAX_ERASE_MODE && e[i] != NULL ) {
        Sizes[i] = e[i]->block_size;
        if (Max_Times) Max_Times[i] = e[i]->cmd.exec_time;
        i++;
    }
    
    return i;
}
        
/* Erase <numSect> consecutive block, first block specified by <EraseAddr> */
bool XSpi_EraseWait             (XSpiHandleT *myHandle, XSpi_EraseMode mode, uint32_t EraseAddr, uint32_t numElem)
{
    if (!EraseInit(myHandle, EraseAddr, numElem, mode, XSPI_MODE_POLL, EraseSM )) return false;
    return EraseSM();
}
bool XSpi_EraseIT               ( XSpiHandleT *myHandle, XSpi_EraseMode mode, uint32_t EraseAddr, uint32_t numElem)
{
    if (!EraseInit(myHandle, EraseAddr, numElem, mode, XSPI_MODE_IRQ, EraseSM )) return false;
    return EraseSM();
}

#if 0
    //RHB todo

    /**
      * @brief DMA UART transmit process complete callback.
      * @param hdma DMA handle.
      * @retval None
      */
    static void XSpi_DMATransmitCplt(DMA_HandleTypeDef *hdma)
    {
      XSpiHandleT *uhandle = (XSpiHandleT *)(hdma->Parent);
      (void)(uhandle);
  
      /* DMA Normal mode */
      // debug_putchar('x');
      if ( DMA_IS_LINEAR(hdma) ) {  
        // RHB tbd
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
    static void XSpi_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
    {
      XSpiHandleT *uhandle = (XSpiHandleT *)(hdma->Parent);
      (void)(uhandle);

      /* DMA Normal mode */
      if ( DMA_IS_LINEAR(hdma) ) {
        // RHB tbd
      }
  
    }

    /**
      * @brief DMA UART communication error callback.
      * @param hdma DMA handle.
      * @retval None
      */
    static void XSpi_DMAError(DMA_HandleTypeDef *hdma)
    {
      XSpiHandleT *uhandle = (XSpiHandleT *)(hdma->Parent);
      // RHB tbd
      (void)(uhandle);
    }

    static void XspiDmaChannelInit(XSpiHandleT *xhandle, const HW_DmaType *dma, XSpiDmaDirectionEnumType dmadir )
    {
      DMA_HandleTypeDef *hdma = dma->dmaHandle;
  
      HW_DMA_HandleInit(hdma, dma, xhandle );

      switch ( dmadir ) 
      {
        case XSPI_DMA_RD:
          hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
          break;
        case XSPI_DMA_WR:
          hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
          break;
        default:
            Error_Handler_XX(-18, __FILE__, __LINE__);
            return;
      }

      HAL_DMA_Init(hdma);

      /* 
       * All callbacks have been set to NULL by Init, 
       * now set Callbacks for DMA complete and DMA error 
       */

      switch ( dmadir ) 
      {
        case XSPI_DMA_RD:
          hdma->XferCpltCallback = XSpi_DMAReceiveCplt;
          hdma->XferErrorCallback = XSpi_DMAError;
          break;
        case XSPI_DMA_WR:
          hdma->XferCpltCallback = XSpi_DMATransmitCplt;   
          hdma->XferErrorCallback = XSpi_DMAError;
          break;
      }

      return;
    }
#endif

/******************************************************************************
 * Abort current operation ( i.e. MemoryMapped or DMA mode and
 * reset functional mode configuration to indirect write mode by default 
 *****************************************************************************/
bool XSpi_Abort(XSpiHandleT *myHandle)
{
    bool ret;
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
    #if USE_OSPI > 0
        ret = HAL_OSPI_Abort(hxspi) == HAL_OK;
    #else
        ret = HAL_QSPI_Abort(hxspi) == HAL_OK;
    #endif
    
    /* if device is in memory mapped mode, this will end MMM */
    if (ret && myHandle->bIsMemoryMapped ) myHandle->bIsMemoryMapped = 0;

    return ret;
}


///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void XSpi_DeInit(const HW_DeviceType *self)
{
    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
    XSpiHandleT *myHandle           = adt->myXSpiHandle;  
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;

    /* DeInit GPIO */
    uint32_t devIdx = GetDevIdx(self);
    GpioAFDeInitAll(devIdx, self->devGpioAF);
  
    /* disable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    const HW_DmaType *dma = self->devDmaRx;

    /* RHB Todo : DMA not yet implemented */
    #if 0
    if(dma) {
      /* De-Initialize the Rx part  */
      HAL_DMA_DeInit(dma->dmaHandle);
      HAL_NVIC_DisableIRQ(dma->dmaIrqNum);
    }
    #else
        UNUSED(dma);
    #endif

    /* Disable QUADSPI/OCTOSPI hardware */
    #if USE_OSPI > 0
        HAL_OSPI_DeInit(hxspi);
    #else
        HAL_QSPI_DeInit(hxspi);
    #endif

    /* To leave a clean state, reset QUAD/OCTOSPI hardware */
    HW_Reset(hxspi->Instance);

    /* disable QUAD/OCTOSPI clock and -if present- OCTOSPI manager*/
    HW_SetHWClock(hxspi->Instance, false);
    #if defined(OCTOSPIM)
        HW_SetHWClock(OCTOSPIM, false);
    #endif
}

#if USE_OSPI > 0
    /**************************************************************************************
     * In case of OSPI being used, configure the OSPIM IP                                 *  
     * The initialization is always done on that way, that there is no pin swapping       *
     * I.e. OCTOSPI1 uses the dedicated OCTOSPI1-Pins, OCTOSPI2 uses the OCTOSPI2-Pins    * 
     * consumption. So, to use the device, it has to be activated first                   *
     *************************************************************************************/
    static bool OSPI_Configure_OSPIM(const HW_DeviceType *self)
    {
        bool ret;
        XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
        XSpiHandleT *myHandle           = adt->myXSpiHandle;  
        XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
        OSPIM_CfgTypeDef OSPIM_Cfg_Struct;

        
        #if USE_OSPI1 > 0
            /* Configure the OctoSPI IO Manager for OCTOSPI1 */
            OSPIM_Cfg_Struct.ClkPort    = 1;
            OSPIM_Cfg_Struct.NCSPort    = 1;
            OSPIM_Cfg_Struct.DQSPort    = ( adt->bUseDQS ? 1 : 0 );
            OSPIM_Cfg_Struct.IOLowPort  = HAL_OSPIM_IOPORT_1_LOW;
            OSPIM_Cfg_Struct.IOHighPort = ( adt->ospiMode ==  OSPI_OCTO ? HAL_OSPIM_IOPORT_1_HIGH : HAL_OSPIM_IOPORT_NONE );
        #else
            /* Configure the OctoSPI IO Manager for OCTOSPI2 */
            OSPIM_Cfg_Struct.ClkPort    = 2;
            OSPIM_Cfg_Struct.NCSPort    = 2;
            OSPIM_Cfg_Struct.DQSPort    = ( adt->bUseDQS ? 2 : 0 );
            OSPIM_Cfg_Struct.IOLowPort  = HAL_OSPIM_IOPORT_2_LOW;
            OSPIM_Cfg_Struct.IOHighPort = ( adt->ospiMode ==  OSPI_OCTO ? HAL_OSPIM_IOPORT_2_HIGH : HAL_OSPIM_IOPORT_NONE );
        #endif

        ret = HAL_OSPIM_Config(hxspi, &OSPIM_Cfg_Struct, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
        if ( !ret ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                LOGU_ERROR("Failed to config Octospi-Manager");
            #endif
        }

        return ret;
    }
#endif

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool XSpi_Init(const HW_DeviceType *self)
{
    bool ret;
    bool bHasInitSpeed;
    uint32_t target_speed;

    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
    XSpiHandleT *myHandle           = adt->myXSpiHandle;  
    myHandle->interface             = adt->interface;
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
    #if USE_OSPI > 0
        hxspi->Instance                 = (OCTOSPI_TypeDef*)self->devBase;
    #else
        hxspi->Instance                 = (QUADSPI_TypeDef*)self->devBase;
    #endif

    myHandle->bInHPMode             = false;
    myHandle->bInDeepPwrDown        = false;
    myHandle->bAsyncBusy            = false;
    myHandle->bIsMemoryMapped       = false;
    myHandle->bIsInitialized        = false;
    myHandle->myRWmode              = XSPI_RW_FAST1;    // select 1-1-1 initially
    myHandle->clkspeed              = adt->defaultSpeed;

    XSpiClockInit(self, true);

    HW_SetHWClock(hxspi->Instance, true);
    #if defined(OCTOSPIM)
        HW_SetHWClock(OCTOSPIM, true);
    #endif

    HW_Reset(hxspi->Instance);

    uint32_t devIdx = GetDevIdx(self);
    XSpiGpioInitAF(devIdx, self->devGpioAF);

    /* First disable QUADSPI/OCTOSPI hardware */
    #if USE_OSPI > 0
        HAL_OSPI_DeInit(hxspi);
    #else
        HAL_QSPI_DeInit(hxspi);
    #endif

    XHelper_SetGeometry(myHandle, &myHandle->geometry);
    XSpiLL_DumpGeometry(&myHandle->geometry);
 
    /* if we have a separate init speed, use this first, otherwise init to normal operating speed */
    bHasInitSpeed = myHandle->interface->global.flags & FCOMMON_HAS_INIT_SPEED;
    target_speed  = bHasInitSpeed ? myHandle->interface->global.init_frq : myHandle->interface->global.operating_frq;
    ret = XSpiLL_ClockInit(myHandle, XSpiGetClockSpeed(self), target_speed,myHandle->geometry.FlashSize );
    
    #if USE_OSPI > 0
        if (ret) ret = OSPI_Configure_OSPIM(self);
    #endif

    if ( ret ) {
        if ( self->devIrqList ) {
            /* Configure the NVIC, enable interrupts */
            HW_SetAllIRQs(self->devIrqList, true);
        }

        /* Perform a software reset of Flash chip */
        if ( !XSpiLL_ResetMemory(myHandle) ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                if ( !ret ) LOGU_ERROR("Unable to perform chip reset");
            #endif
        }
        if ( !(ret=XSpiLL_GetID(myHandle)) ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                if ( !ret ) LOGU_ERROR("Unable to read XSpi chip ID");
            #endif
            #if DEBUG_MODE > 0
                XSpiLL_DumpChipInfo(myHandle->id);
                if ( console_debuglevel > 0 )  XSpi_DumpStatus(myHandle);
            #endif
        }

        /* 
         * Initialize DMA channel, if configured to use DMA
         * only regard devDmaRx, ...Tx is never used, because there is only one DMA request type
         */
        #if 0  
        // RHB tbd
        if ( self->devDmaRx ) {
        /**** 004 ****/
        DMA_HandleTypeDef *hdma;
        hdma = HW_DMA_RegisterDMAChannel(self->devDmaRx);
        ret = hdma != NULL;
        /* dma channel initiialization has to be done before every rd or wr operation
           because there is only one dma channel                                      */
        if ( ret ) {
            xhandle->hRxDma = dma->dmaHandle;
        }
        #endif

        /*
         * if we have a lower init speed, then execute the chip specific part of initialization
         * and switch to normal operating speed
         */

        if ( bHasInitSpeed ) {
            if ( myHandle->interface->init ) {
                ret = myHandle->interface->init();
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    if ( !ret ) LOGU_ERROR("driver specific init failed");
                #endif
            }
            /* now switch to normal operating speed */
            if ( ret ) {
                ret = XSpiLL_ClockInit(myHandle, XSpiGetClockSpeed(self), myHandle->interface->global.operating_frq, myHandle->geometry.FlashSize );        
            }
        }
        
    }


    /* enable quad mode */
    if ( ret ) XSpi_SetQuad(myHandle, true);

    if ( ret ) {
        /* If deep sleep is supported, put flash chip into deep sleep mode */
        if ( XHelper_HasDeepPowerDown(myHandle) ) XSpi_SetDeepPowerDown(myHandle, true);
    } else {
        /* If Initialization was unsuccessful, deactivate all */
        XSpi_DeInit(self);
    }

    myHandle->bIsInitialized = ret;
    return ret;
}

/**********************************************************************************
 * Early Init of QSPI Device to enable R/W of config values
 *********************************************************************************/
void XSpi_EarlyInit(void)
{
    int32_t dev_idx;

    /* Init QSPI device */
    dev_idx = AddDevice(&XSPI_DEV, NULL ,NULL);
    DeviceInitByIdx(dev_idx, NULL);
}


/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool XSpi_AllowStop(const HW_DeviceType *self)
{
    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
#if USE_OSPI > 0 
    return ! (adt->myXSpiHandle->hxspi.Instance->SR & OCTOSPI_SR_BUSY);
#else
    return ! (adt->myXSpiHandle->hxspi.Instance->SR & QUADSPI_SR_BUSY);
#endif
}


/******************************************************************************
 * Callback _before_ frequency changes: To be on the safe side, switch to
 * high performance mode
 *****************************************************************************/
void XSpi_BeforeFrqChange(const HW_DeviceType *self)
{
    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
    XSpiHandleT *myHandle           = adt->myXSpiHandle;  

    if ( XHelper_HasHighPerformanceMode( myHandle) ) XSpiLL_SetHPMode( myHandle, true);
}


/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool XSpi_OnFrqChange(const HW_DeviceType *self)
{
    XSpi_AdditionalDataType *adt    = XSpi_GetAdditionalData(self);
    XSpiHandleT *myHandle           = adt->myXSpiHandle;  

    return XSpiLL_ClockInit(myHandle, XSpiGetClockSpeed(self), myHandle->clkspeed, myHandle->geometry.FlashSize );

}

///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(QUADSPI) && USE_QSPI > 0

    XSpiHandleT QSpi1Handle;

    #ifdef QSPI1_USE_DMA
      static DMA_HandleTypeDef hdma_qspi1;
      static const HW_DmaType dma_qspi1 = { &hdma_qspi1, QSPI1_DMA };
    #endif

    #if defined( QSPI1_USE_IRQ )
    const HW_IrqList irq_qspi1 = {
        .num = 1,
        .irq = {QSPI1_IRQ },
    };
    #endif

    /* QUADSPI: 6 AF-Pins, NCS, CLK, SIO0, ..., SIO3 */
    static const HW_GpioList_AF gpio_qspi1 = {
        .num  = 6,
        .gpio = { 
            QSPI1_NCS, QSPI1_CLK,
            QSPI1_SI_SIO0, QSPI1_SO_SIO1,
            QSPI1_SIO2, QSPI1_SIO3,
        }
    };

    static const XSpi_AdditionalDataType additional_qspi1 = {
        .myXSpiHandle       = &QSpi1Handle,
        .interface          = &flash_interface,
        .defaultSpeed       = QSPI1_CLKSPEED,     
        #if defined(STM32H7_FAMILY)
            #if defined(QSPI1_CLKSOURCE)
                .clkSource  = QSPI1_CLKSOURCE,
            #else 
                .clkSource  = RCC_QSPICLKSOURCE_D1HCLK,
            #endif
        #endif
    };


const HW_DeviceType HW_QSPI1 = {
    .devName        = XSpiStr,
    .devBase        = QUADSPI,
    .devGpioAF      = &gpio_qspi1,
    .devGpioIO      = NULL,
    .devGpioADC     = NULL,
    .devType        = HW_DEVICE_XSPI,
    .devData        = &additional_qspi1,
    .devIrqList     = 
        #if defined(QSPI1_USE_IRQ) 
            &irq_qspi1,
        #else
            NULL,
        #endif
    /* There is only one dma channel for quadspi, so we assign that  *
     * to .devDmaRx, even it's used for both directions              */
    #if defined(QSPI1_USE_DMA)
        .devDmaRx = &dma_qspi1,
    #else
        .devDmaRx   = NULL,
    #endif
    .devDmaTx       = NULL,         /* .devDmaTx  is never used */
    .Init           = XSpi_Init,
    .DeInit         = XSpi_DeInit,
    .BeforeFrqChange= XSpi_BeforeFrqChange,
    .OnFrqChange    = XSpi_OnFrqChange,
    .AllowStop      = XSpi_AllowStop,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif /* defined(QUADSPI) && USE_QSPI > 0 */


#if defined(OCTOSPI1) && USE_OSPI1 > 0
    XSpiHandleT OSpi1Handle;

    #if defined(OSPI1_HAS_DS_MODE)
        static XSpiDeepSleepT ds_ospi1;
    #endif

    #ifdef OSPI1_USE_DMA
      static DMA_HandleTypeDef hdma_ospi1;
      static const HW_DmaType dma_ospi1 = { &hdma_ospi1, OSPI1_DMA };
    #endif

    #if defined( OSPI1_USE_IRQ )
    const HW_IrqList irq_ospi1 = {
        .num = 1,
        .irq = {OSPI1_IRQ },
    };
    #endif

    /* OCTOSPI: 11 AF-Pins, NCS, CLK, DQS, SIO0, ..., SIO3, SIO4, ..., SIO7 */
    static const HW_GpioList_AF gpio_ospi1 = {
        .gpio = { 
            OSPI1_NCS, OSPI1_CLK, 
        #if defined(OSPI1_USE_DQS)
            OSPI1_DQS, 
            #define GPIO_NUM1       3
        #else
            #define GPIO_NUM1       2
        #endif
            OSPI1_SI_SIO0, OSPI1_SO_SIO1, OSPI1_SIO2, OSPI1_SIO3,
        #if defined(OSPI1_MODE_OCTO)
            OSPI1_SI_SIO4, OSPI1_SO_SIO5, OSPI1_SIO6, OSPI1_SIO7,
            #define GPIO_NUM2       8
        #else
            #define GPIO_NUM2       4
        #endif
        },
        .num = GPIO_NUM1 + GPIO_NUM2,
    };


    static const XSpi_AdditionalDataType additional_ospi1 = {
        .myXSpiHandle       = &OSpi1Handle,
        .interface          = &flash_interface;
        .defaultSpeed       = OSPI1_CLKSPEED,                 
        .myDsInfo           =
            #if defined(OSPI1_HAS_DS_MODE)
                &ds_ospi1,
            #else
                NULL,
            #endif
        .bHasLPMode =
            #if defined(OSPI1_HAS_LP_MODE)
                1,
            #else
                0,
            #endif
    #if USE_OSPI > 0
        .ospiMode =
            #if defined(OSPI1_MODE_OCTO)
                OSPI_OCTO,
            #elif defined(OSPI1_MODE_QUAD)
                OSPI_QUAD,
            #else
                error "Operating mode for OCTOSPI1 not set"
            #endif
        .bUseDQS =
            #if defined(OSPI1_USE_DQS)
                1,
            #else
                0,
            #endif
    #endif
    };


const HW_DeviceType HW_OSPI1 = {
    .devName        = XSpiStr,
    #if USE_QSPI > 0
        .devBase        = QUADSPI,
    #else
        #if USE_OSPI2
            .devBase        = OCTOSPI2,
        #else
            .devBase        = OCTOSPI1,
        #endif
    #endif
    .devGpioAF      = &gpio_ospi1,
    .devGpioIO      = NULL,
    .devGpioADC     = NULL,
    .devType        = HW_DEVICE_XSPI,
    .devData        = &additional_ospi1,
    .devIrqList     = 
        #if defined(OSPI1_USE_IRQ) 
            &irq_ospi1,
        #else
            NULL,
        #endif
    /* There is only one dma channel for quadspi, so we assign that  *
     * to .devDmaRx, even it's used for both directions              */
    #if defined(OSPI1_USE_DMA)
        .devDmaRx = &dma_ospi1,
    #else
        .devDmaRx   = NULL,
    #endif
    .devDmaTx       = NULL,         /* .devDmaTx  is never used */
    .Init           = XSpi_Init,
    .DeInit         = XSpi_DeInit,
    .BeforeFrqChange= XSpi_BeforeFrqChange,
    .OnFrqChange    = XSpi_OnFrqChange,
    .AllowStop      = XSpi_AllowStop,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif /* ( defined(QUADSPI) && defined(USE_QSPI) ) || ( defined(OCTOSPI) && defined(USE_OSPI1) ) */

#if defined(OCTOSPI2) && USE_OSPI2 > 0
    XSpiHandleT OSpi2Handle;

    #if defined(OSPI2_HAS_DS_MODE)
        static XSpiDeepSleepT ds_ospi2;
    #endif

    #ifdef OSPI2_USE_DMA
      static DMA_HandleTypeDef hdma_ospi2;
      static const HW_DmaType dma_ospi2 = { &hdma_ospi2, OSPI2_DMA };
    #endif

    #if defined( OSPI2_USE_IRQ )
    const HW_IrqList irq_ospi2 = {
        .num = 1,
        .irq = {OSPI2_IRQ },
    };
    #endif

    /* OCTOSPI: 11 AF-Pins, NCS, CLK, DQS, SIO0, ..., SIO3, SIO4, ..., SIO7 */
    static const HW_GpioList_AF gpio_ospi2 = {
        .gpio = { 
            OSPI2_NCS, OSPI2_CLK, 
        #if defined(OSPI2_USE_DQS)
            OSPI2_DQS, 
            #define GPIO_NUM1       3
        #else
            #define GPIO_NUM1       2
        #endif
            OSPI2_SI_SIO0, OSPI2_SO_SIO1, OSPI2_SIO2, OSPI2_SIO3,
        #if defined(OSPI2_MODE_OCTO)
            OSPI2_SI_SIO4, OSPI2_SO_SIO5, OSPI2_SIO6, OSPI2_SIO7,
            #define GPIO_NUM2       8
        #else
            #define GPIO_NUM2       4
        #endif
        },
        .num = GPIO_NUM1 + GPIO_NUM2,
    };


    static const XSpi_AdditionalDataType additional_ospi2 = {
        .myXSpiHandle       = &OSpi2Handle,
        .interface          = &flash_interface;
        .defaultSpeed       = OSPI2_CLKSPEED,                 
        .myDsInfo           =
            #if defined(OSPI2_HAS_DS_MODE)
                &ds_ospi2,
            #else
                NULL,
            #endif
        .bHasLPMode =
            #if defined(OSPI2_HAS_LP_MODE)
                1,
            #else
                0,
            #endif
    #if USE_OSPI > 0
        .ospiMode =
            #if defined(OSPI2_MODE_OCTO)
                OSPI_OCTO,
            #elif defined(OSPI2_MODE_QUAD)
                OSPI_QUAD,
            #else
                error "Operating mode for OCTOSPI2 not set"
            #endif
        .bUseDQS =
            #if defined(OSPI2_USE_DQS)
                1,
            #else
                0,
            #endif
    #endif
    };


const HW_DeviceType HW_OSPI2 = {
    .devName        = XSpiStr,
    .devBase        = OCTOSPI2,
    .devGpioAF      = &gpio_ospi2,
    .devGpioIO      = NULL,
    .devGpioADC     = NULL,
    .devType        = HW_DEVICE_XSPI,
    .devData        = &additional_ospi2,
    .devIrqList     = 
        #if defined(OSPI2_USE_IRQ) 
            &irq_ospi2,
        #else
            NULL,
        #endif
    /* There is only one dma channel for quadspi, so we assign that  *
     * to .devDmaRx, even it's used for both directions              */
    #if defined(OSPI2_USE_DMA)
        .devDmaRx = &dma_ospi2,
    #else
        .devDmaRx   = NULL,
    #endif
    .devDmaTx       = NULL,         /* .devDmaTx  is never used */
    .Init           = XSpi_Init,
    .DeInit         = XSpi_DeInit,
    .BeforeFrqChange= XSpi_BeforeFrqChange,
    .OnFrqChange    = XSpi_OnFrqChange,
    .AllowStop      = XSpi_AllowStop,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif /* defined(OCTOSPI2) && defined(USE_OSPI2) */



///////////////////////////////////////////////////////////////////////////////
// Interrupt routines QUADSPI /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if USE_QSPI > 0 && defined(QSPI1_USE_IRQ)
    /**
      * @brief  Command completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("CmdCplt Callback");
        TaskNotify(TASK_XSPI);
    }

    /**
      * @brief  Rx Transfer completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hxspi)
    {
      UNUSED(hxspi);
      LOGU_VERBOSE("RxCplt Callback");
      QSpi1Handle.bAsyncBusy = false;  
      if ( QSpi1Handle.XSpi_RdDoneCB ) QSpi1Handle.XSpi_RdDoneCB(&QSpi1Handle);
    }

    /**
      * @brief  Tx Transfer completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("TxCplt Callback");
        TaskNotify(TASK_XSPI);
    }

    /**
      * @brief  Status Match callbacks
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("StatusMatch Callback");
        TaskNotify(TASK_XSPI);
    }

    void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hxspi)
    {
      UNUSED(hxspi);
      LOGU_VERBOSE("ERROR CALLBACK");
      /* Set error state and trigger next call of SM */
      smData.wrState = STATE_ERROR;
      TaskNotify(TASK_XSPI);
    }
#endif /* USE_QSPI > 0 && defined(QSPI_USE_IRQ) */



#if USE_OSPI > 0 && ( defined(OSPI1_USE_IRQ) || defined(OSPI2_USE_IRQ) )
    /**
      * @brief  Command completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("CmdCplt Callback");
        TaskNotify(TASK_XSPI);
    }

    /**
      * @brief  Rx Transfer completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hxspi)
    {
      UNUSED(hxspi);
      LOGU_VERBOSE("RxCplt Callback");
      #if USE_OSPI1 > 0
          OSpi1Handle.bAsyncBusy = false;  
          if ( OSpi1Handle.XSpi_RdDoneCB ) OSpi1Handle.XSpi_RdDoneCB(&OSpi1Handle);
      #else
          OSpi2Handle.bAsyncBusy = false;  
          if ( OSpi2Handle.XSpi_RdDoneCB ) OSpi2Handle.XSpi_RdDoneCB(&OSpi2Handle);
       #endif
    }

    /**
      * @brief  Tx Transfer completed callbacks.
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("TxCplt Callback");
        TaskNotify(TASK_XSPI);
    }

    /**
      * @brief  Status Match callbacks
      * @param  hxspi: QSPI handle
      * @retval None
      */
    void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hxspi)
    {
        UNUSED(hxspi);
        LOGU_VERBOSE("StatusMatch Callback");
        TaskNotify(TASK_XSPI);
    }

    void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hxspi)
    {
      UNUSED(hxspi);
      LOGU_VERBOSE("ERROR CALLBACK");
      /* Set error state and trigger next call of SM */
      smData.wrState = STATE_ERROR;
      TaskNotify(TASK_XSPI);
    }
#endif /* USE_OSPI > 0 && ( defined(OSPI1_USE_IRQ) || defined(OSPI2_USE_IRQ) ) */


void task_handle_xspi(uint32_t arg)
{
    UNUSED(arg);
    /* Only one SM may be active at one time, check which */
    if      (wrSM) wrSM();
    else if (erSM) erSM(); 
}

#endif /* if USE_QSPI > 0 || USE_OSPI > 0 */


/**
  * @}
  */


