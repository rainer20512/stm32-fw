/*
 ******************************************************************************
 * @file    xspi_dev.h 
 * @author  Rainer
 * @brief   Wrapping the QSPI/OSPI hardware into my HW_DEVICE
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __XSPI_DEV_H
#define __XSPI_DEV_H

#include "config/config.h"
#include "devices.h"
#if USE_OSPI > 0
    #define     XXSPI_HandleTypeDef        OSPI_HandleTypeDef
#else
    #define     XXSPI_HandleTypeDef        QSPI_HandleTypeDef
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/* Public defines -----------------------------------------------------------------------*/
/* QSPI Error codes */
#define XSPI_OK            ((uint32_t)0x00)
#define XSPI_ERROR         ((uint32_t)0x01)
#define XSPI_BUSY          ((uint32_t)0x02)
#define XSPI_NOT_SUPPORTED ((uint32_t)0x04)
#define XSPI_SUSPENDED     ((uint32_t)0x08)

/* Different modes of QUADSPI operation */
#define XSPI_MODE_POLL      0       // Execute function in Polling/active wait mode
#define XSPI_MODE_IRQ       1       // Execute function in Interrupt/NoWalt mode
#define XSPI_MODE_DMA       2       // Execute function in DMA/NoWalt mode, makes only sense for read/write operations

/* 
   Different modes of erase Operation. Due to different naming conventions with different mamnufacturers,
   we name these units from small to big size as follows: Sector < Subblock < Block < whole chip memory
*/

#define XSPI_ERASE_SECTOR   1000    // Erase one sector
#define XSPI_ERASE_SUBBLOCK 1001    // Subblock
#define XSPI_ERASE_BLOCK    1002    // Erase one block
#define XSPI_ERASE_ALL      1009    // Erase whole chip

/* Public typedef -----------------------------------------------------------------------*/

typedef struct XSpiGeometryType {
    uint32_t FlashSize;             /* Total Size in Bytes */
    uint32_t ProgPageSize;          /* Size of a write "unit" */
    uint32_t EraseSectorSize;       /* Size of a erase "uint" */
    uint32_t ProgPagesNumber;       /* Resulting number of "write ubits" */
    uint32_t EraseSectorsNumber;    /* Resulting number of "erase units" */

} XSpiGeometryT;

typedef struct XSpiDeepSleepType {
    uint16_t            dlyToSleep;      /* min time [us] to enter deep sleep                 */
    uint16_t            dlyFmSleep;      /* min time [us] to exit from deep sleep             */   
    uint8_t             bIsDeepSleep;    /* true, iff external flash chip is in deepsleep     */
    uint8_t             bInDsTransit;    /* true, iff in transit from or to deep sleep        */
} XSpiDeepSleepT;

typedef struct XSpiHandleType XSpiHandleT;
typedef void (*XSpiCallbackT ) ( XSpiHandleT* );
typedef struct XSpiHandleType {
    XXSPI_HandleTypeDef hxspi;           /* Embedded HAL_QSPI/OSPI_HandleTypedef Structure    */
    XSpiGeometryT       geometry;        /* actual flash chip geometry                        */
    uint8_t             id[4];           /* Flash memories ID bytes ( only first 3 used )     */
    uint32_t            clkspeed;        /* Desired Clock Speed in MHz                        */
    XSpiCallbackT       XSpi_RdDoneCB;   /* Callback for any successful read                  */
    XSpiCallbackT       XSpi_WrDoneCB;   /* Callback for any successful write/erare           */
    XSpiCallbackT       XSpi_ErrorCB;    /* Callback for any Error in QSPI transaction        */
    XSpiDeepSleepT*     dsInfo;          /* if deep sleep is supported, the ds info, else NULL*/
    uint8_t             bAsyncBusy;      /* Flag for "Async operation (_IT, _DMA) ongoing     */
    uint8_t             bIsMemoryMapped; /* true, iff in memory mapped mode                   */
    uint8_t             bIsInitialized;  /* true, iff device is initialized successfully      */
} XSpiHandleT; 

void XSpi_GetGeometry           (XSpiHandleT *myHandle, XSpiGeometryT *pInfo);
void XSpi_DumpStatus            (XSpiHandleT *myHandle);
void XSpi_SetAsyncCallbacks     (XSpiHandleT *myHandle, XSpiCallbackT rdDoneCB, XSpiCallbackT wrDoneCB, XSpiCallbackT errorCB);
bool XSpi_Abort                 (XSpiHandleT *myHandle);
bool QSpecific_EnterDeepPowerDown    (XSpiHandleT *myHandle);
bool XSpi_LeaveDeepPowerDown    (XSpiHandleT *myHandle);
bool QSpecific_EnableMemoryMappedMode(XSpiHandleT *myHandle);
bool XSpi_SetSpeed              (XSpiHandleT *myHandle, uint32_t new_clkspeed);
bool XSpi_IsInitialized         (XSpiHandleT *myHandle);

bool XSpi_ReadWait              (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);
bool XSpi_ReadIT                (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);

bool XSpi_WriteWait             (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool XSpi_WriteIT               (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool XSpi_WriteDMA              (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);

/* Erase <numSect> consecutive sectors, first sector specified by <EraseAddr> */
bool XSpi_EraseSectorWait       (XSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numSect);
bool XSpi_EraseSectorIT         (XSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numSect);

/* Erase <numSect> consecutive block, first block specified by <EraseAddr> */
bool XSpi_EraseBlockWait        (XSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numBlock);
bool XSpi_EraseBlockIT          (XSpiHandleT *myHandle, uint32_t EraseAddr, uint32_t numBlock);

/* Erase entire chip */
bool XSpi_EraseChipWait         (XSpiHandleT *myHandle);
bool XSpi_EraseChipIT           (XSpiHandleT *myHandle);


void XSpi_EarlyInit             (void);
void XSpi_SetGeometry           ( XSpiGeometryT *geometry, uint32_t flash_size, uint32_t page_size, uint32_t sector_size );

/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_QSPI) || USE_OSPI
    extern const HW_DeviceType HW_XSPI1;
    extern XSpiHandleT XSpi1Handle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __XSPI_DEV_H */
