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

/* Debug ------------------------------------------------------------------------*/
#define DEBUG_QSPI          1

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

/* My macros --------------------------------------------------------------------*/

/* Private typedef --------------------------------------------------------------*/
typedef enum QSpiDmaDirectionEnum {
  QSPI_DMA_RD=0,                      // Read DMA
  QSPI_DMA_WR,                        // Write DMA
} QSpiDmaDirectionEnumType;



typedef struct {
    QSpiHandleT *myQSpiHandle;       /* my associated handle */
    uint32_t default_speed;          /* default speed in MHz */
} QSpi_AdditionalDataType;


/* Forward declarations -------------------------------------------------------------*/
const char *QSpi_GetChipTypeText(uint8_t *idbuf);


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

static void QSpiGpioInitAF(const HW_GpioList_AF *gpioaf)
{
    GPIO_InitTypeDef Init;
    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GpioAFInitAll(gpioaf, &Init );
}

/**************************************************************************************
 * Initialize the flash memories specific geometry data                               *
 *************************************************************************************/
static void QSpi_SetGeometry ( QSpiGeometryT *geometry, uint32_t flash_size, uint32_t page_size, uint32_t sector_size )
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
void QSPI_GetGeometry(QSpiHandleT *myHandle, QSpiGeometryT *pInfo)
{
    memcpy(pInfo, &myHandle->geometry, sizeof(QSpiGeometryT) );
}

/******************************************************************************
 * Include the hardware specific functions here
 *****************************************************************************/
#include "./qspi/mx25r6435f.c"


#if DEBUG_MODE >  0
    /**************************************************************************************
     * Return the Manufacturer Name as string                                             *
     *************************************************************************************/
    const char *QSpi_GetChipManufacturer(uint8_t mf_id )
    {
        switch(mf_id) {
            case 0xc2: return "Macronix"; 
            default:
                return "Unknown Manufacturer";
        }
    }

    /**************************************************************************************
     * Return the chip name as string                                                     *
     *************************************************************************************/
    const char *QSpi_GetChipTypeText(uint8_t *idbuf)
    {
        /* 16 bit combination of chip type and density */
        uint16_t typNdens = idbuf[1];
        typNdens = typNdens << 8 | idbuf[2];

        /* Switch by manufactureer ID first */
        switch(idbuf[0]) {
            case 0xc2:  /* Macronix */
                switch(typNdens) {
                    case 0x2817: return "MX25R6435F";
                    case 0x2018: return "MX25L12835F";
                    default:     return "Unknown Chip";
                }
            default:
                return "Unknown Chip";
        }
    
    }


    /**************************************************************************************
     * Read the chip ID and print chip info. Can be used to set chip specific parameters  *
     * automatically later. Now just chip info is dumped                                  *
     *************************************************************************************/
    static void QSpi_DumpChipInfo(uint8_t *idbuf)
    {
        const char *mf   =  QSpi_GetChipManufacturer(idbuf[0] );
        const char *type =  QSpi_GetChipTypeText(idbuf);
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
 * Dump Status and restore DeepSleep-Status afterwards                                *
 *************************************************************************************/
void QSpi_DumpStatus(QSpiHandleT *myHandle)
{
    bool sleep = myHandle->bIsDeepSleep;
    /* Wake up device, if in deep sleep */
    if ( sleep && ! QSpi_LeaveDeepPowerDown(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_DumpStatus - Error: Cannot wake up flash device");
        #endif
        return;
    }

    QSpi_DumpStatusInternal(myHandle);
    
    if ( sleep ) QSpi_EnterDeepPowerDown(myHandle);
}

/**************************************************************************************
 * Set QSpi done and/or error callback                                                *
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

    /* Wake up device, if in deep sleep */
    if ( myHandle->bIsDeepSleep && ! QSpi_LeaveDeepPowerDown(myHandle) ) return false;

    /* Send the read command */
    if ( !QSpi_ReadCMD(hqspi, ReadAddr, Size) ) return false;
    
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
typedef bool ( *QSpi_SM_T ) (void);     // Typedef for Statemachine function
static uint32_t currWriteAddr;          // current write Address 
static uint32_t currWriteSize;          // current write Size
static uint32_t WriteEndAddr;           // last write Address + 1
static uint8_t  *WriteSource;           // Ptr to the write source
static uint32_t currOpmode;             // operation mode of write op
static uint32_t currState;              // current state of state machine
static QSpiHandleT *currHandle;         // current QSpi handle
static QSpi_SM_T currSM;                // currently used Statemachine
#define currHqspi   (&currHandle->hqspi)

/******************************************************************************
 * Implement a classic loop with four components
 * - Initialisation
 * - Loop body
 * - Change loop variables and check for termination
 * - terminate
 * This is done, because the loop execution is triggered direcly ( in polling mode )
 * or by interrupts ( in IRQ or DMA mode )
 *****************************************************************************/
static bool WriteInit(QSpiHandleT *myHandle,  uint8_t* pData, uint32_t WriteAddr, uint32_t Size, uint32_t opmode, QSpi_SM_T stateMachine)
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
    currWriteSize = myHandle->geometry.ProgPageSize - (WriteAddr % myHandle->geometry.ProgPageSize);

    /* Check if the size of the data is less than the remaining place in the page */
    if (currWriteSize > Size) currWriteSize = Size;

    /* Initialize the current and final write addresses */
    currWriteAddr = WriteAddr;
    WriteEndAddr  = WriteAddr + Size;
    WriteSource   = pData;
    currOpmode    = opmode;
    currState     = 0;
    currHandle    = myHandle;
    currSM        = stateMachine;

    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        DEBUG_PRINTTS("Write Init, area: 0x%08x ... 0x%08x, written in %d steps\n", WriteAddr, WriteEndAddr-1, Size/myHandle->geometry.ProgPageSize);
    #endif

    /* Wake up device, if in deep sleep */
    if ( myHandle->bIsDeepSleep && ! QSpi_LeaveDeepPowerDown(myHandle) ) return false;

    return true;
}

static void WriteTerminate(bool finalState)
{
    if ( currOpmode != QSPI_MODE_POLL ) {
        if ( !currHandle->bAsyncBusy ) {
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PUTS("WriteTerminate - Error: Async Op not active");
            #endif
        }
        currHandle->bAsyncBusy = false;
    }
    DEBUG_PRINTTS("Write terminated %s\n",finalState ? "ok" : "with error");

    /* Activate Callback, if specified */
    if (currHandle->QSpi_WrDoneCB) currHandle->QSpi_WrDoneCB(currHandle);
}


static bool WriteBlock (void )
{
    bool ret;

    DEBUG_PRINTTS("Write @0x%08x, Len=%d\n", currWriteAddr, currWriteSize);
    /* Enable write and send write command */
    if ( !QSpi_WriteCMD(currHqspi, currWriteAddr, currWriteSize) ) return false;

    switch ( currOpmode ) {
        case QSPI_MODE_POLL:
            /* Transmission of the data in polling mode*/
            ret = HAL_QSPI_Transmit(currHqspi, WriteSource, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
            break;
        case QSPI_MODE_IRQ:
            /* Transmission of the data in irq mode*/
            ret = HAL_QSPI_Transmit_IT(currHqspi, WriteSource) == HAL_OK;
            break;
        case QSPI_MODE_DMA:
            /* Transmission of the data in irq mode*/
            ret = HAL_QSPI_Transmit_DMA(currHqspi, WriteSource) == HAL_OK;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("WriteLoop - Error: Mode %d not implemented\n", currOpmode);
            #endif
            return false;
    } // switch

    /* report errors, if configured */
    if ( !ret ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTF("WriteLoop - Error: Transmit error in mode %d\n", currOpmode);
        #endif
    }

    return true;
}

static bool WaitForWriteDone(void)
{
    bool ret;

    /* Configure automatic polling mode to wait for reset of WIP bit */  
    if ( currOpmode == QSPI_MODE_POLL ) 
        ret = QSpi_WaitForWriteDone(currHqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    else 
        ret = QSpi_WaitForWriteDone_IT(currHqspi);
 
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
    currWriteAddr += currWriteSize;
    WriteSource   += currWriteSize;
    currWriteSize = ((currWriteAddr + myHandle->geometry.ProgPageSize) > WriteEndAddr) ? (WriteEndAddr - currWriteAddr) : myHandle->geometry.ProgPageSize;
  
    return currWriteAddr < WriteEndAddr;
}
#define WRSTATE_START       0
#define WRSTATE_WRITEBLOCK  1
#define WRSTATE_WAITFORDONE 2
#define WRSTATE_INCREMENT   3
#define WRSTATE_TERMINATE   4
#define STATE_ERROR         99

#define NEXTSTATE(a)     currState=a
#define SM_PAUSE(a)      bSmPause=a
static bool WriteSM(void)
{
    bool bSmPause = false;

    while (!bSmPause ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTTS("WriteSM in state %d\n", currState);
        #endif
        switch ( currState ) {
            case WRSTATE_START: 
                /* Enable Write and Send Write Command - both in one step */
                if ( !QSpi_WriteCMD(currHqspi, currWriteAddr, currWriteSize) ) goto WriteSmTerminate;
                /* Continue with next state in any case*/
                NEXTSTATE(WRSTATE_WRITEBLOCK);
                SM_PAUSE(false);
                break;
            case WRSTATE_WRITEBLOCK:
                if (!WriteBlock() ) goto WriteSmTerminate;
                NEXTSTATE(WRSTATE_WAITFORDONE);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                SM_PAUSE( currOpmode != QSPI_MODE_POLL );
                break;
            case WRSTATE_WAITFORDONE:
                if ( !WaitForWriteDone() ) goto WriteSmTerminate;
                NEXTSTATE(WRSTATE_INCREMENT);
                /* In Async mode the next state will be triggered by interrupt, proceed directly in polling mode */
                SM_PAUSE( currOpmode != QSPI_MODE_POLL );
                break;
            case WRSTATE_INCREMENT:
                /* increment the loop counters and check for termination */
                if ( WriteIncement(currHandle) ) 
                    NEXTSTATE(WRSTATE_WRITEBLOCK);
                else
                    NEXTSTATE(WRSTATE_TERMINATE);
                /* Continue with next state right now*/
                SM_PAUSE(false);
                break;
            case WRSTATE_TERMINATE:
                /* Final State reached without error */
                WriteTerminate(true);
                return true;
            case STATE_ERROR:
                /* Error state, reached only by error Interrupt; */
                goto WriteSmTerminate;    
            default:
                #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                    DEBUG_PRINTF("WriteSM - Error: illegal state %d\n", currState);
                #endif
                goto WriteSmTerminate;
        } // switch
    } // while
    return true;

WriteSmTerminate:
    // All erroneous exits are handeled here
    WriteTerminate(false);
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


///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void QSpi_DeInit(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  
    QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;

    /* DeInit GPIO */
    GpioAFDeInitAll(self->devGpioAF);
  
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
    myHandle->bIsDeepSleep          = false;
    myHandle->bAsyncBusy            = false;
    myHandle->bIsMemoryMapped       = false;

    HW_SetHWClock(hqspi->Instance, true);
    HW_Reset(hqspi->Instance);

    QSpiGpioInitAF(self->devGpioAF);

    ret = QSpi_SpecificInit(self, adt->myQSpiHandle, adt->default_speed);
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

        /* put flash chip into deep sleep mode  */
        QSpi_EnterDeepPowerDown(myHandle);
    }


    /* If Initialization was unsuccessful, deactivate all */
    if ( !ret ) QSpi_DeInit(self);

    return ret;
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
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool QSpi_OnFrqChange(const HW_DeviceType *self)
{
    QSpi_AdditionalDataType *adt    = QSpi_GetAdditionalData(self);
    QSpiHandleT *myHandle           = adt->myQSpiHandle;  

    return QSpi_BasicInit(myHandle, adt->default_speed, myHandle->geometry.FlashSize, false );

}

///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(QUADSPI) && defined(USE_QSPI1)
    QSpiHandleT QSpi1Handle;

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
        .default_speed      = 40000000,
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
        TaskNotifyFromISR(TASK_QSPI);
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
        TaskNotifyFromISR(TASK_QSPI);
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
        TaskNotifyFromISR(TASK_QSPI);
    }

    void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
    {
      UNUSED(hqspi);
      DEBUG_PRINTTS("ERROR CALLBACK\n");
      /* Set error state and trigger next call of SM */
      currState = STATE_ERROR;
      TaskNotifyFromISR(TASK_QSPI);
    }
#endif /* if defined(QSPI1_USE_IRQ) */


void task_handle_qspi(uint32_t arg)
{
    UNUSED(arg);
    if (currSM) currSM();
}

#endif /* if USE_QSPI > 0 */


/**
  * @}
  */


