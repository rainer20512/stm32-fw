/*
 ******************************************************************************
 * @file    qspi_dev.c
 * @author  Rainer
 * @brief   QUADSPI hardware wrapped into HW_Device
 *
 *****************************************************************************/

/** @addtogroup QUADSPI
  * @{
   
*/
#include "config/config.h"

#if USE_QSPI > 0 


#include "config/devices_config.h"
#include "config/qspi_config.h"
#include "error.h"
#include "task/minitask.h"
#include "system/profiling.h"
#include "system/hw_util.h"
#include "system/clockconfig.h"
#include "dev/hw_device.h"
#include "dev/devices.h"

#include "debug_helper.h"

#include "dev/qspi/qspecific.h"

/* My macros --------------------------------------------------------------------*/

/* Private typedef --------------------------------------------------------------*/
typedef enum QSpiDmaDirectionEnum {
  QSPI_DMA_RD=0,                      // Read DMA
  QSPI_DMA_WR,                        // Write DMA
} QSpiDmaDirectionEnumType;



typedef struct {
    QSpiHandleT     *myQSpiHandle;   /* my associated handle */
    uint32_t        default_speed;   /* default speed in MHz */
    QSpiDeepSleepT  *myDsInfo;       /* ptr to dsInfo iff deep sleep is supported, else NULL */
    uint8_t         bHasLPMode;      /* != 0, if device supports low power mode */
} QSpi_AdditionalDataType;


/* Forward declarations -------------------------------------------------------------*/


/* Private or driver functions ------------------------------------------------------*/
static QSpi_AdditionalDataType * QSpi_GetAdditionalData(const HW_DeviceType *self)
{
    return (QSpi_AdditionalDataType *)(self->devData);
}

#if 0 /* currently not used */
/*************************************************************************************
 * Initialize the dma channel. This has to be done before every read or write, because
 * QUADSPI only has one dma channel
 ************************************************************************************/
static void QSpiDmaChannelInit(const HW_DeviceType *self, QSpiDmaDirectionEnumType dmadir )
{
    const HW_DmaType *dma           = self->devDmaRx;
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
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
    case QSPI_DMA_RD:
      hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
      hqspi->hdma = dma->dmaHandle;
      break;
    case QSPI_DMA_WR:
      hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      hqspi->hdma = dma->dmaHandle;
      break;
    default:
        Error_Handler(__FILE__, __LINE__);
        return;
    }

    HAL_DMA_Init(hdma);

    return;
}
#endif

static void QSpiGpioInitAF(uint32_t devIdx, const HW_GpioList_AF *gpioaf)
{
    GPIO_InitTypeDef Init;

    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_HIGH;

    GpioAFInitAll(devIdx, gpioaf, &Init );
}

/**************************************************************************************
 * Some Devices support different clock sources for QSPI. Make sure, that             *   
  * QQSpiSetClockSource and QSpiGetClockSpeed() will match                            *
 *************************************************************************************/
#if defined(STM32L476xx) || defined(STM32L496xx)
    /* STM32L4xx has no clock mux for QUADSPI device */
    #define QSpiSetClockSource(a)           (true)
    #define QSpiGetClockSpeed()             HAL_RCC_GetHCLKFreq()
#elif defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    static bool QSpiSetClockSource(const void *hw)
    {
      UNUSED(hw);
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* QUADSPI has to be operaterd with HCLK. Routines, which will set */
      /* qspi speed, rely on HCLK as Clock source                        */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI; 
      PeriphClkInit.QspiClockSelection   = RCC_QSPICLKSOURCE_D1HCLK;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for QUADSPI");
        return false;
      }

      return true;
    }
    //#include "hardware.h"
    //#define  QSpiGetClockSpeed()            GetPerClkFrequency()
    #define QSpiGetClockSpeed()             HAL_RCC_GetHCLKFreq()
#else 
    #error "No qspi clock assignment defined"
#endif

/*
 * Init or DeInit Clock / clocksource 
 */
static bool QSpiClockInit(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !QSpiSetClockSource( self->devBase ) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock( (QUADSPI_TypeDef*)self->devBase, bDoInit );
    return true;
}

/**************************************************************************************
 * Initialize the flash memories specific geometry data                               *
 *************************************************************************************/
void QSpi_SetGeometry ( QSpiGeometryT *geometry, uint32_t flash_size, uint32_t page_size, uint32_t sector_size )
{
    
   #if DEBUG_MODE > 0
        /* Check validity */
        #define PWROF2(a)   ( (a & (a-1)) == 0 ) 
        if ( !PWROF2(flash_size) )
            DEBUG_PRINTF("Flash size 0x%x is not a power of 2, sure ?\n", flash_size);
        if ( !PWROF2(page_size) )
            DEBUG_PRINTF("Page size 0x%x is not a power of 2, sure ?\n", page_size);
        if ( !PWROF2(sector_size) )
            DEBUG_PRINTF("Sector size 0x%x is not a power of 2, sure ?\n", sector_size);
   #endif

   geometry->FlashSize          = flash_size;
   geometry->ProgPageSize       = page_size;
   geometry->EraseSectorSize    = sector_size;
   geometry->ProgPagesNumber    = flash_size/page_size;
   geometry->EraseSectorsNumber = flash_size/sector_size;
}


/**************************************************************************************
 * Return the flash memories specific geometry data                               *
 *************************************************************************************/
void QSpi_GetGeometry(QSpiHandleT *myHandle, QSpiGeometryT *pInfo)
{
    memcpy(pInfo, &myHandle->geometry, sizeof(QSpiGeometryT) );
}


#if DEBUG_MODE >  0
    /**************************************************************************************
     * Return the Manufacturer Name as string                                             *
     *************************************************************************************/
    const char *QSpi_GetChipManufacturer(uint8_t mf_id )
    {
        switch(mf_id) {
            case 0x20: return "Micron"; 
            case 0xc2: return "Macronix"; 
            default:
                return "Unknown Manufacturer";
        }
    }



    /**************************************************************************************
     * Read the chip ID and print chip info. Can be used to set chip specific parameters  *
     * automatically later. Now just chip info is dumped                                  *
     *************************************************************************************/
    static void QSpi_DumpChipInfo(uint8_t *idbuf)
    {
        char type[25];
        const char *mf   =  QSpi_GetChipManufacturer(idbuf[0] );
        QSpecific_GetChipTypeText(idbuf, type, 25);
        DEBUG_PRINTF("QSpi: Found %s %s\n", mf, type);
    }

    /**************************************************************************************
     * Dump the geometry data  *
     * automatically later. Now just chip info is dumped                                  *
     *************************************************************************************/
    static void QSpi_DumpGeometry(QSpiGeometryT *geo)
    {
        DEBUG_PRINTF("QSpi: Flash size is %dkiB\n", geo->FlashSize>>10);
        DEBUG_PRINTF("QSpi: %d write pages with %d bytes\n", geo->ProgPagesNumber, geo->ProgPageSize);
        DEBUG_PRINTF("QSpi: %d erase sectors with %d bytes\n", geo->EraseSectorsNumber, geo->EraseSectorSize);
    }

#endif

/**************************************************************************************
 * Change the Qspi clock speed on the fly                                             *
 *************************************************************************************/
bool QSpi_SetSpeed (QSpiHandleT *myHandle, uint32_t new_clkspeed)
{
    return QSpecific_BasicInit(myHandle, QSpiGetClockSpeed(), new_clkspeed, myHandle->geometry.FlashSize, false);
}

/**************************************************************************************
 * returns value != 0, iff successfully initialized                                   *
 *************************************************************************************/
bool QSpi_IsInitialized(QSpiHandleT *myHandle)
{
    return myHandle->bIsInitialized;
}


/**************************************************************************************
 * Dump Status and restore DeepSleep-Status afterwards                                *
 *************************************************************************************/
void QSpi_DumpStatus(QSpiHandleT *myHandle)
{
    bool sleep = myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep;
    /* Wake up device, if in deep sleep */
    if ( sleep && ! QSpecific_LeaveDeepPowerDown(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_DumpStatus - Error: Cannot wake up flash device");
        #endif
        return;
    }

    QSpecific_DumpStatusInternal(myHandle);
    
    if ( sleep ) QSpecific_EnterDeepPowerDown(myHandle);
}

/**************************************************************************************
 * Set QSpi read, write/erase and error callbacks                                     *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
void QSpi_SetAsyncCallbacks(QSpiHandleT *myHandle, QSpiCallbackT rdDoneCB, QSpiCallbackT wrDoneCB, QSpiCallbackT errorCB)
{
    myHandle->QSpi_RdDoneCB  = rdDoneCB;
    myHandle->QSpi_WrDoneCB  = wrDoneCB;
    myHandle->QSpi_ErrorCB   = errorCB; 
}

/******************************************************************************
 ******************************************************************************
 * Implementation of the read operation:
 * Transmit the read command ( this is device specific ) and start reading
 * in polling, irq or dma mode   
 ******************************************************************************
 *****************************************************************************/
bool QSpi_ReadOperation(QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size, uint32_t opmode)
{
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
    bool ret;

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep && !QSpecific_LeaveDeepPowerDown(myHandle) ) return false;

    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        DEBUG_PRINTTS("QSPI read, area: 0x%08x ... 0x%08x\n", ReadAddr, ReadAddr+Size-1);
    #endif

    /* Send the read command */
    if ( !QSpecific_ReadCMD(hqspi, ReadAddr, Size) ) return false;
    
    switch ( opmode ) {
        case QSPI_MODE_POLL:
            /* Reception of the data in polling mode with timeout */
            ret =  HAL_QSPI_Receive(hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            break;
#if defined(QSPI1_USE_IRQ)
        case QSPI_MODE_IRQ:
           /* Reception of the data in interrupt mode*/
            myHandle->bAsyncBusy = true;
            ret = HAL_QSPI_Receive_IT(hqspi, pData) == HAL_OK;
            if ( !ret) myHandle->bAsyncBusy = false;
            break;
#endif
#if defined(QSPI1_USE_DMA)
        case QSPI_MODE_DMA:
           /* Reception of the data in DMA mode*/
            myHandle->bAsyncBusy = true;
            ret = HAL_QSPI_Receive_DMA(hqspi, pData) == HAL_OK;
            if ( !ret) myHandle->bAsyncBusy = false;
            break;
#endif
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("QSpi_ReadOperation - Error: Mode %d not implemented\n", opmode);
            #endif
            return false;
    } // switch

    /* report errors, if configured */
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTF("QSpi_ReadOperation - Error: Receive error in mode %d\n", opmode);
        #endif
    }

    return ret;
}

bool QSpi_ReadWait  (QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return QSpi_ReadOperation(myHandle, pData, ReadAddr, Size, QSPI_MODE_POLL );
}

bool QSpi_ReadIT    (QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return QSpi_ReadOperation(myHandle, pData, ReadAddr, Size, QSPI_MODE_IRQ );
}

bool QSpi_ReadDMA   (QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr, uint32_t Size) {
    return QSpi_ReadOperation(myHandle, pData, ReadAddr, Size, QSPI_MODE_DMA );
}

/******************************************************************************
 ******************************************************************************
 * Implementation of the write operation:
 * Transmit the read command ( this is device specific ) and start writing
 * in polling, irq or dma mode   
 * Since maximum one page can be written with one write operation, the write
 * operation is repeaded as often as possible, until the whole buffer is written
 ******************************************************************************
 *****************************************************************************/
typedef bool ( *QSpi_SM_Fn ) (void);     // Typedef for Statemachine function
typedef union {
    struct {
        uint32_t wrOpmode;             // operation mode of write op
        uint32_t wrState;              // current state of write state machine
        QSpiHandleT *wrHandle;         // current QSpi handle for write op
        uint32_t currWriteAddr;        // current write Address 
        uint32_t currWriteSize;        // current write Size
        uint32_t WriteEndAddr;         // last write Address + 1
        uint8_t  *WriteSource;         // Ptr to the write source
    };
    struct {
        uint32_t erOpmode;             // operation mode of erase op
        uint32_t erState;              // current state of erase state machine
        QSpiHandleT *erHandle;         // current QSpi handle for erase op
        QSpi_SM_Fn erSM;               // currently used Statemachine for erase
        uint32_t currEraseAddr;        // current erase address
        uint32_t EraseAddrInc;         // Address Increment for next erase item
        uint32_t EraseItemCnt;         // number of items to erase
        uint32_t EraseMode;            // Selected erase mode ( Sector/block/chip )
    };
} Qspi_SM_DataT;

static Qspi_SM_DataT smData;
static QSpi_SM_Fn wrSM;               // currently used Statemachine for write
static QSpi_SM_Fn erSM;               // currently used Statemachine for erase

#define GETWRHANDLE()                   (&(smData.wrHandle->hqspi))
#define GETERHANDLE()                   (&(smData.erHandle->hqspi))

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
static bool WriteInit(QSpiHandleT *myHandle,  uint8_t* pData, uint32_t WriteAddr, uint32_t Size, uint32_t opmode, QSpi_SM_Fn stateMachine)
{
    if ( opmode != QSPI_MODE_POLL ) {
        if ( myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PUTS("WriteWait - Error: Another Async Op is active");
            #endif
            return false;
        }
        myHandle->bAsyncBusy = true;
    }

    /* check for positive size */
    if ( Size == 0 ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("WriteWait - Writing 0 bytes not allowed");
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

    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        DEBUG_PRINTTS("Write Init, area: 0x%08x ... 0x%08x, written in %d steps\n", WriteAddr,smData.WriteEndAddr-1, Size/myHandle->geometry.ProgPageSize);
    #endif

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep && ! QSpecific_LeaveDeepPowerDown(myHandle) ) return false;

    return true;
}

static void WriteEraseTerminate(bool finalState, uint32_t opmode, QSpiHandleT *myHandle)
{
    if ( opmode != QSPI_MODE_POLL ) {
        if ( !myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PUTS("WriteEraseTerminate - Error: Async Op not active");
            #endif
        }
        myHandle->bAsyncBusy = false;
    }
    DEBUG_PRINTTS("Write/Erase terminated %s\n",finalState ? "ok" : "with error");

    /* Activate Callback, if specified */
    if (myHandle->QSpi_WrDoneCB) myHandle->QSpi_WrDoneCB(myHandle);
}


static bool WriteBlock (void )
{
    bool ret;

    DEBUG_PRINTTS("Write @0x%08x, Len=%d\n", smData.currWriteAddr, smData.currWriteSize);
    DEBUG_PRINTTS("First Bytes (hex)= %02x, %02x, %02x, %02x, ...\n", 
                  smData.WriteSource[0], smData.WriteSource[1], smData.WriteSource[2], smData.WriteSource[3]);
    /* Enable write and send write command */
    if ( !QSpecific_WriteCMD(GETWRHANDLE(), smData.currWriteAddr, smData.currWriteSize) ) return false;

    switch ( smData.wrOpmode ) {
        case QSPI_MODE_POLL:
            /* Transmission of the data in polling mode*/
            ret = HAL_QSPI_Transmit(GETWRHANDLE(), smData.WriteSource, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            break;
        case QSPI_MODE_IRQ:
            /* Transmission of the data in irq mode*/
            ret = HAL_QSPI_Transmit_IT(GETWRHANDLE(), smData.WriteSource) == HAL_OK;
            break;
        case QSPI_MODE_DMA:
            /* Transmission of the data in irq mode*/
            ret = HAL_QSPI_Transmit_DMA(GETWRHANDLE(), smData.WriteSource) == HAL_OK;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("WriteLoop - Error: Mode %d not implemented\n", smData.wrOpmode);
            #endif
            return false;
    } // switch

    /* report errors, if configured */
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTF("WriteLoop - Error: Transmit error in mode %d\n", smData.wrOpmode);
        #endif
    }

    return true;
}

static bool WaitForWriteDone(void)
{
    bool ret;

    /* Configure automatic polling mode to wait for reset of WIP bit */  
    if ( smData.wrOpmode == QSPI_MODE_POLL ) 
        ret = QSpecific_WaitForWriteDone(GETWRHANDLE(), HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    else 
        ret = QSpecific_WaitForWriteDone_IT(GETWRHANDLE());
 
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificWriteWait - Error: Timeout while autopll");
        #endif
        return false;
    }
    
    return true;
}

static bool WriteIncement(QSpiHandleT *myHandle)
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
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTTS("WriteSM in state %d\n", smData.wrState);
        #endif
        switch ( smData.wrState ) {
            case WRSTATE_START: 
                /* Enable Write and Send Write Command - both in one step */
                if ( !QSpecific_WriteCMD(GETWRHANDLE(), smData.currWriteAddr, smData.currWriteSize) ) goto WriteSmTerminate;
                /* Continue with next state in any case*/
                WR_NEXTSTATE(WRSTATE_WRITEBLOCK);
                WR_SM_PAUSE(false);
                break;
            case WRSTATE_WRITEBLOCK:
                if (!WriteBlock() ) goto WriteSmTerminate;
                WR_NEXTSTATE(WRSTATE_WAITFORDONE);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                WR_SM_PAUSE( smData.wrOpmode != QSPI_MODE_POLL );
                break;
            case WRSTATE_WAITFORDONE:
                if ( !WaitForWriteDone() ) goto WriteSmTerminate;
                WR_NEXTSTATE(WRSTATE_INCREMENT);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                WR_SM_PAUSE( smData.wrOpmode != QSPI_MODE_POLL );
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
                #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                    DEBUG_PRINTF("WriteSM - Error: illegal state %d\n", smData.wrState);
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

bool QSpi_WriteWait(QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, QSPI_MODE_POLL, WriteSM) ) return false;
    return WriteSM();
}
bool QSpi_WriteIT  (QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, QSPI_MODE_IRQ, WriteSM) ) return false;
    return WriteSM();
}
bool QSpi_WriteDMA  (QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if ( !WriteInit(myHandle, pData, WriteAddr, Size, QSPI_MODE_DMA, WriteSM) ) return false;
    return WriteSM();
}

/*-----------------------------------------------------------------------------
 * Erase - Erase - Erase - Erase - Erase - Erase - Erase - Erase - Erase - Eras
 *---------------------------------------------------------------------------*/

/******************************************************************************
 * Get the address increment depending from the erase mode
 *****************************************************************************/
static uint32_t GetEraseAddrInc(QSpiHandleT *myHandle,uint32_t erasemode)
{
    switch(erasemode)
    {
        case QSPI_ERASE_SECTOR:
            return myHandle->geometry.EraseSectorSize;
            break;
        case QSPI_ERASE_BLOCK:
            // RHB tbd return myHandle->geometry.EraseBlockSize;
            return 1 << 15;
            break;
        case QSPI_ERASE_ALL:
            return 1 << 16;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("GetEraseAddrInc - Error: unkown erasemode %d\n", erasemode);
            #endif
    }

    return 1 << 16;
}

/******************************************************************************
 * Implement a classic loop with four components
 * - Initialisation
 * - Loop body
 * - Change loop variables and check for termination
 * - terminate
 * This is done, because the loop execution is triggered direcly ( in polling mode )
 * or by interrupts ( in IRQ or DMA mode )
 *****************************************************************************/
static bool EraseInit(QSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numItems, uint32_t opmode, uint32_t erasemode, QSpi_SM_Fn stateMachine)
{
    if ( opmode != QSPI_MODE_POLL ) {
        if ( myHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PUTS("EraseInit - Error: Another Async Op is active");
            #endif
            return false;
        }
        myHandle->bAsyncBusy = true;
    }

    /* check for positive size */
    if ( numItems == 0 ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("EraseInit - Erasing 0 items not allowed");
        #endif
        return false;
    }

    /* Initialize the neccessary erase data */
    smData.currEraseAddr = EraseAddr;
    smData.EraseAddrInc  = GetEraseAddrInc(myHandle, erasemode);
    smData.EraseItemCnt  = ( erasemode == QSPI_ERASE_ALL ? 1 : numItems);
    smData.EraseMode     = erasemode;
    smData.erOpmode      = opmode;
    smData.wrOpmode    = opmode;
    smData.erState     = 0;
    smData.erHandle    = myHandle;
    erSM               = stateMachine;
    wrSM               = NULL;

    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        DEBUG_PRINTTS("Erase Init, start: 0x%08x, itemsize=%d, items=%d, mode=%d\n", EraseAddr, smData.EraseAddrInc, smData.EraseItemCnt, erasemode);
    #endif

    /* Wake up device, if it has deep sleep capability and is in deep sleep */
    if ( myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep && ! QSpecific_LeaveDeepPowerDown(myHandle) ) return false;

    return true;
}

static bool WaitForEraseDone(uint32_t timeout_ms)
{
    bool ret;

    /* Configure automatic polling mode to wait for reset of WIP bit */  
    if ( smData.erOpmode == QSPI_MODE_POLL ) 
        ret = QSpecific_WaitForWriteDone(GETERHANDLE(), timeout_ms);
    else 
        ret = QSpecific_WaitForWriteDone_IT(GETERHANDLE());
 
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificWait - Error: Timeout while autopll");
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
    uint8_t opcode_unused;

    while (!bSmPause ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTTS("EraseSM in state %d\n", smData.erState);
        #endif
        switch ( smData.erState ) {
            case ERSTATE_START: 
                /* Enable Write and Send Write Command - both in one step */
                DEBUG_PRINTTS("Erase @0x%08x, mode=%d\n", smData.currEraseAddr, smData.EraseMode);
                if ( !QSpecific_EraseCMD(GETERHANDLE(), smData.currEraseAddr, smData.EraseMode) ) goto EraseSmTerminate;
                /* Continue with next state in any case*/
                ER_NEXTSTATE(ERSTATE_WAITFORDONE);
                /* Continue with waiting for completion immediately */
                ER_SM_PAUSE( false );
                break;
            case ERSTATE_WAITFORDONE:
                if ( !QSpecific_GetEraseParams(smData.EraseMode, &tmo_ms, &opcode_unused ) ) goto EraseSmTerminate;
                if ( !WaitForEraseDone(tmo_ms) ) goto EraseSmTerminate;
                DEBUG_PRINTTS("Erase done\n");
                ER_NEXTSTATE(ERSTATE_INCREMENT);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                ER_SM_PAUSE( smData.erOpmode != QSPI_MODE_POLL );
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
                #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                    DEBUG_PRINTF("EraseSM - Error: illegal state %d\n", smData.erState);
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




/* Erase <numSect> consecutive sectors, first sector specified by <EraseAddr> */
bool QSpi_EraseSectorWait       (QSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numSect)
{
    if (!EraseInit(myHandle, EraseAddr, numSect, QSPI_MODE_POLL, QSPI_ERASE_SECTOR, EraseSM )) return false;
    return EraseSM();
}
bool QSpi_EraseSectorIT         (QSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numSect)
{
    if (!EraseInit(myHandle, EraseAddr, numSect, QSPI_MODE_IRQ, QSPI_ERASE_SECTOR, EraseSM )) return false;
    return EraseSM();
}

/* Erase <numSect> consecutive block, first block specified by <EraseAddr> */
bool QSpi_EraseBlockWait        (QSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numBlock)
{
    if (!EraseInit(myHandle, EraseAddr, numBlock, QSPI_MODE_POLL, QSPI_ERASE_BLOCK , EraseSM )) return false;
    return EraseSM();
}
bool QSpi_EraseBlockIT          (QSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numBlock)
{
    if (!EraseInit(myHandle, EraseAddr, numBlock, QSPI_MODE_IRQ, QSPI_ERASE_BLOCK, EraseSM )) return false;
    return EraseSM();
}

/* Erase entire chip */
bool QSpi_EraseChipWait         (QSpiHandleT *myHandle)
{
    if (!EraseInit(myHandle, 0, 1, QSPI_MODE_POLL, QSPI_ERASE_ALL , EraseSM )) return false;
    return EraseSM();
}
bool QSpi_EraseChipIT           (QSpiHandleT *myHandle)
{
    if (!EraseInit(myHandle, 0, 1, QSPI_MODE_IRQ, QSPI_ERASE_ALL , EraseSM )) return false;
    return EraseSM();
}

/******************************************************************************
 * Abort current operation ( i.e. MemoryMapped or DMA mode and
 * reset functional mode configuration to indirect write mode by default 
 *****************************************************************************/
bool QSpi_Abort(QSpiHandleT *myHandle)
{
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
    return HAL_QSPI_Abort(hqspi) == HAL_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void QSpi_DeInit(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;

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

    /* Disable QUADSPI hardware */
    HAL_QSPI_DeInit(hqspi);

    /* To leave a clean state, reset QUADSPI hardware */
    HW_Reset(hqspi->Instance);

    /* disable QUADSPI clock */
    HW_SetHWClock(hqspi->Instance, false);
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool QSpi_Init(const HW_DeviceType *self)
{
    bool ret;
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
    hqspi->Instance                 = (QUADSPI_TypeDef*)self->devBase;
    /* If deep sleep is supported, setup the deep sleep information block */
    if ( adt->myDsInfo ) {
        myHandle->dsInfo            = adt->myDsInfo;
        myHandle->dsInfo->bIsDeepSleep = false;
        myHandle->dsInfo->bInDsTransit = false;
    } else {
        myHandle->dsInfo            = NULL;    
    }
    myHandle->bAsyncBusy            = false;
    myHandle->bIsMemoryMapped       = false;
    myHandle->bIsInitialized        = false;
    myHandle->clkspeed              = adt->default_speed;

    QSpiClockInit(self, true);
    HW_SetHWClock(hqspi->Instance, true);
    HW_Reset(hqspi->Instance);

    uint32_t devIdx = GetDevIdx(self);
    QSpiGpioInitAF(devIdx, self->devGpioAF);

    HAL_QSPI_DeInit(&myHandle->hqspi);
    ret = QSpi_SpecificInit(self, myHandle, QSpiGetClockSpeed() );
    if ( ret ) {
        if ( self->devIrqList ) {
            /* Configure the NVIC, enable interrupts */
            HW_SetAllIRQs(self->devIrqList, true);
        }

        #if DEBUG_MODE > 0
            QSpi_DumpChipInfo(myHandle->id);
            QSpi_DumpGeometry(&myHandle->geometry);
            if ( debuglevel > 2 )  QSpi_DumpStatus(myHandle);
        #endif

 
        /* dma channel initiialization has to be done before every rd or wr operation
           because there is only one dma channel                                      */

        /* If deep sleep is supported, put flash chip into deep sleep mode */
        if ( adt->myDsInfo ) QSpecific_EnterDeepPowerDown(myHandle);
    }


    /* If Initialization was unsuccessful, deactivate all */
    if ( !ret ) QSpi_DeInit(self);

    /* Otherwise set "initialized" flag and return with success */
    myHandle->bIsInitialized = true;
    return ret;
}

/**********************************************************************************
 * Early Init of QSPI Device to enable R/W of config values
 *********************************************************************************/
void QSPI_EarlyInit(void)
{
    int32_t dev_idx;

    /* Init QSPI device */
    dev_idx = AddDevice(&QSPI_DEV, NULL ,NULL);
    DeviceInitByIdx(dev_idx, NULL);
}


/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool QSpi_AllowStop(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    return ! (adt->myQSpiHandle->hqspi.Instance->SR & QUADSPI_SR_BUSY);
}


/******************************************************************************
 * Callback _before_ frequency changes: To be on the safe side, switch to
 * high performance mode
 *****************************************************************************/
void QSpi_BeforeFrqChange(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  

    if ( adt->bHasLPMode) QSpecific_HPerfMode( &myHandle->hqspi, true);
}


/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool QSpi_OnFrqChange(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  

    return QSpecific_BasicInit(myHandle, QSpiGetClockSpeed(), adt->default_speed, myHandle->geometry.FlashSize, false );

}

///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(QUADSPI) && defined(USE_QSPI1)
    QSpiHandleT QSpi1Handle;

    #if defined(QSPI1_HAS_DS_MODE)
        static QSpiDeepSleepT ds_qspi1;
    #endif

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

    /* 6 AF-Pins, NCS, CLK, SIO0, ..., SIO3 */
    static const HW_GpioList_AF gpio_qspi1 = {
        .num  = 6,
        .gpio = { 
            QSPI1_NCS, QSPI1_CLK,
            QSPI1_SI_SIO0, QSPI1_SO_SIO1,
            QSPI1_SIO2, QSPI1_SIO3,
        }
    };



    static const QSpi_AdditionalDataType additional_qspi1 = {
        .myQSpiHandle       = &QSpi1Handle,
        .default_speed      = QSPI1_CLKSPEED,                 
        .myDsInfo           =
            #if defined(QSPI1_HAS_DS_MODE)
                &ds_qspi1,
            #else
                NULL,
            #endif
        .bHasLPMode =
            #if defined(QSPI1_HAS_LP_MODE)
                1,
            #else
                0,
            #endif

    };


const HW_DeviceType HW_QSPI1 = {
    .devName        = "QSPI1",
    .devBase        = QUADSPI,
    .devGpioAF      = &gpio_qspi1,
    .devGpioIO      = NULL,
    .devGpioADC     = NULL,
    .devType        = HW_DEVICE_QSPI,
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
    .Init           = QSpi_Init,
    .DeInit         = QSpi_DeInit,
    .BeforeFrqChange= QSpi_BeforeFrqChange,
    .OnFrqChange    = QSpi_OnFrqChange,
    .AllowStop      = QSpi_AllowStop,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif


///////////////////////////////////////////////////////////////////////////////
// Interrupt routines /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(QSPI1_USE_IRQ)
    /**
      * @brief  Command completed callbacks.
      * @param  hqspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
    {
        UNUSED(hqspi);
        DEBUG_PRINTTS("CmdCplt Callback\n");
        TaskNotify(TASK_QSPI);
    }

    /**
      * @brief  Rx Transfer completed callbacks.
      * @param  hqspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
    {
      UNUSED(hqspi);
      DEBUG_PRINTTS("RxCplt Callback\n");
      QSpi1Handle.bAsyncBusy = false;  
      if ( QSpi1Handle.QSpi_RdDoneCB ) QSpi1Handle.QSpi_RdDoneCB(&QSpi1Handle);
    }

    /**
      * @brief  Tx Transfer completed callbacks.
      * @param  hqspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
    {
        UNUSED(hqspi);
        DEBUG_PRINTTS("TxCplt Callback\n");
        TaskNotify(TASK_QSPI);
    }

    /**
      * @brief  Status Match callbacks
      * @param  hqspi: QSPI handle
      * @retval None
      */
    void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
    {
        UNUSED(hqspi);
        DEBUG_PRINTTS("StatusMatch Callback\n");
        TaskNotify(TASK_QSPI);
    }

    void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
    {
      UNUSED(hqspi);
      DEBUG_PRINTTS("ERROR CALLBACK\n");
      /* Set error state and trigger next call of SM */
      smData.wrState = STATE_ERROR;
      TaskNotify(TASK_QSPI);
    }
#endif /* if defined(QSPI1_USE_IRQ) */


void task_handle_qspi(uint32_t arg)
{
    UNUSED(arg);
    /* Only one SM may be active at one time, check which */
    if      (wrSM) wrSM();
    else if (erSM) erSM(); 
}

#endif /* if USE_QSPI > 0 */


/**
  * @}
  */


