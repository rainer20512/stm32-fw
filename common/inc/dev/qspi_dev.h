/*
 ******************************************************************************
 * @file    qspi_dev.h 
 * @author  Rainer
 * @brief   Wrapping the QSpi hardware into my HW_DEVICE
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QSPI_DEV_H
#define __QSPI_DEV_H

#include "config/config.h"
#include "devices.h"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Public defines -----------------------------------------------------------------------*/
/* QSPI Error codes */
#define QSPI_OK            ((uint32_t)0x00)
#define QSPI_ERROR         ((uint32_t)0x01)
#define QSPI_BUSY          ((uint32_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint32_t)0x04)
#define QSPI_SUSPENDED     ((uint32_t)0x08)

/* Different modes of QUADSPI operation */
#define QSPI_MODE_POLL  0       // Execute function in Polling/active wait mode
#define QSPI_MODE_IRQ   1       // Execute function in Interrupt/NoWalt mode
#define QSPI_MODE_DMA   2       // Execute function in DMA/NoWalt mode, makes only sense for read/write operations

/* Public typedef -----------------------------------------------------------------------*/
typedef void ( *QEncEncCB ) ( int32_t );        /* Callback for encoder rotation         */
typedef void ( *QEncClickCB ) ( void );         /* Callbacks for Click and DblClick      */

typedef struct QSpiGeometryType {
    uint32_t FlashSize;             /* Total Size in Bytes */
    uint32_t ProgPageSize;          /* Size of a write "unit" */
    uint32_t EraseSectorSize;       /* Size of a erade "uint" */
    uint32_t ProgPagesNumber;       /* Resulting number of "write ubits" */
    uint32_t EraseSectorsNumber;    /* Resulting number of "erase units" */

} QSpiGeometryT;

typedef struct QSpiHandleType QSpiHandleT;
typedef void (*QSpiCallbackT ) ( QSpiHandleT* );
typedef struct QSpiHandleType {
    QSPI_HandleTypeDef  hqspi;           /* Embedded HAL QSpi_HandleTypedef Structure     */
    QSpiGeometryT       geometry;        /* actual flash chip geometry                    */
    uint8_t             id[4];           /* Flash memories ID bytes ( only first 3 used ) */
    QSpiCallbackT       QSpi_RdDoneCB;   /* Callback for any successful read              */
    QSpiCallbackT       QSpi_WrDoneCB;   /* Callback for any successful write/erare       */
    QSpiCallbackT       QSpi_ErrorCB;    /* Callback for any Error in QSPI transaction    */
    bool                bIsDeepSleep;    /* true, iff external flash chip is in deepsleep */
    bool                bAsyncBusy;      /* Flag for "Async operation (_IT, _DMA) ongoing */
    bool                bIsMemoryMapped; /* true, iff in memory mapped mode               */
} QSpiHandleT; 

void QSPI_GetGeometry           (QSpiHandleT *myHandle, QSpiGeometryT *pInfo);
void QSpi_DumpStatus            (QSpiHandleT *myHandle);
void QSpi_SetAsyncCallbacks     (QSpiHandleT *myHandle, QSpiCallbackT rdDoneCB, QSpiCallbackT wrDoneCB, QSpiCallbackT errorCB);
bool QSpi_ResetMemory           (QSpiHandleT *myHandle);
bool QSpi_Abort                 (QSpiHandleT *myHandle);
bool QSpi_EnterDeepPowerDown    (QSpiHandleT *myHandle);
bool QSpi_LeaveDeepPowerDown    (QSpiHandleT *myHandle);
bool QSpi_EnableMemoryMappedMode(QSpiHandleT *myHandle);

bool QSpi_ReadWait              (QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);
bool QSpi_ReadIT                (QSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);

bool QSpi_WriteWait             (QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool QSpi_WriteIT               (QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool QSpi_WriteDMA              (QSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);

bool QSpi_Erase_SectorWait      (QSpiHandleT *myHandle, uint32_t SectorAddress);

bool QSpi_EraseChipWait         (QSpiHandleT *myHandle);

/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_QSPI)
    extern const HW_DeviceType HW_QSPI1;
    extern QSpiHandleT QSpi1Handle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __QSPI_DEV_H */