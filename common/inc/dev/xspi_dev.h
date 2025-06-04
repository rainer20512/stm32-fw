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
#include "dev/xspi/flash_interface.h"

#if USE_OSPI > 0
    #define     XXSPI_HandleTypeDef             OSPI_HandleTypeDef
    #define     XSPI_CommandTypeDef             OSPI_RegularCmdTypeDef
    #define     XSPI_AutoPollingTypeDef         OSPI_AutoPollingTypeDef
    #define     XSPI_MemoryMappedTypeDef        OSPI_MemoryMappedTypeDef
    #define     XSPI_TIMEOUT_DEFAULT_VALUE      HAL_OSPI_TIMEOUT_DEFAULT_VALUE

    #define     HAL_XSPI_Init                   HAL_OSPI_Init
    #define     HAL_XSPI_Command                HAL_OSPI_Command
    #define     HAL_XSPI_Receive                HAL_OSPI_Receive
    #define     HAL_XSPI_MemoryMapped           HAL_OSPI_MemoryMapped
    #define     HAL_XSPI_AutoPolling            HAL_OSPI_AutoPolling
    #define     HAL_XSPI_AutoPolling_IT         HAL_OSPI_AutoPolling_IT
    #define     HAL_XSPI_Transmit               HAL_OSPI_Transmit
#elif USE_QSPI > 0
    #define     XXSPI_HandleTypeDef             QSPI_HandleTypeDef
    #define     XSPI_CommandTypeDef             QSPI_CommandTypeDef
    #define     XSPI_AutoPollingTypeDef         QSPI_AutoPollingTypeDef
    #define     XSPI_MemoryMappedTypeDef        QSPI_MemoryMappedTypeDef
    #define     XSPI_TIMEOUT_DEFAULT_VALUE      HAL_QSPI_TIMEOUT_DEFAULT_VALUE
 
    #define     HAL_XSPI_Init                   HAL_QSPI_Init
    #define     HAL_XSPI_Command                HAL_QSPI_Command
    #define     HAL_XSPI_Receive                HAL_QSPI_Receive
    #define     HAL_XSPI_MemoryMapped           HAL_QSPI_MemoryMapped
    #define     HAL_XSPI_AutoPolling            HAL_QSPI_AutoPolling
    #define     HAL_XSPI_AutoPolling_IT         HAL_QSPI_AutoPolling_IT
    #define     HAL_XSPI_Transmit               HAL_QSPI_Transmit
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


/* Number of defined erase block sizes */
static const uint8_t XSPI_MAX_ERASE_MODE=4;

/* 
   Different modes of erase Operation. Due to different naming conventions with different manufacturers,
   we name these units from small to big size as follows: Sector < Subblock < Block < BigBlock < whole chip memory
*/
typedef enum {
    XSPI_ERASE_SECTOR=0,            // Erase one sector
    XSPI_ERASE_SUBBLOCK =1,         // Subblock
    XSPI_ERASE_BLOCK=2,             // Erase one block
    XSPI_ERASE_BIGBLOCK=3,          // Erase one big block
    XSPI_ERASE_ALL=99,              // Erase whole chip
} XSpi_EraseMode;

/*
 * Different XSPI Read and write modes, the numbers x-y-z note: x; number of data lines for command bytes
 * y: number of data lines for address and dummy bytes, z: number of data lines for io data
 */
typedef enum {
    XSPI_RW_FAST1   = 0,            // Fast r/w 1-1-1
    XSPI_RW_DUAL2   = 1,            // Dual r/w 1-2-2
    XSPI_RW_QUAD4   = 2,            // quad r/w 1-4-4
    XSPI_RW_MAXNUM  = 3,            // last and biggest ordinary number
} XSPI_RWMode;

/* Public typedef -----------------------------------------------------------------------*/

typedef struct XSpiGeometryType {
    uint32_t FlashSize;             /* Total Size in Bytes, power of 2 value */
    uint32_t ProgPageSize;          /* Size of a write "unit" */
    uint32_t ProgPagesNumber;       /* Resulting number of "write units" */
    uint32_t EraseSize[MAX_ERASE];  /* Size of different erase units, 0 = not defined */
    uint32_t EraseNum[MAX_ERASE];   /* Resulting number of different "erase units" */

} XSpiGeometryT;


typedef struct XSpiHandleType XSpiHandleT;
typedef void (*XSpiCallbackT ) ( XSpiHandleT* );
typedef struct XSpiHandleType {
    NOR_FlashT          *interface;      /* flash chip specific flash interface               */
    XXSPI_HandleTypeDef hxspi;           /* Embedded HAL_QSPI/OSPI_HandleTypedef Structure    */
    XSpiGeometryT       geometry;        /* actual flash chip geometry                        */
    uint8_t             id[4];           /* Flash memories ID bytes ( only first 3 used )     */
    uint32_t            clkspeed;        /* Desired Clock Speed in MHz                        */
    XSpiCallbackT       XSpi_RdDoneCB;   /* Callback for any successful read                  */
    XSpiCallbackT       XSpi_WrDoneCB;   /* Callback for any successful write/erare           */
    XSpiCallbackT       XSpi_ErrorCB;    /* Callback for any Error in QSPI transaction        */
    XSPI_RWMode         myRWmode;        /* currently selected RW mode ( 1-1-1, 1-2-2, 1-4-4) */
    uint8_t             bAsyncBusy;      /* Flag for "Async operation (_IT, _DMA) ongoing     */
    uint8_t             bIsMemoryMapped; /* true, iff in memory mapped mode                   */
    uint8_t             bIsInitialized;  /* true, iff device is initialized successfully      */
    uint8_t             bInDeepPwrDown;  /* true if device currently in deep pwr down state   */
    uint8_t             bInHPMode;       /* true if device currently in high performance mode */
} XSpiHandleT; 

void XSpi_GetGeometry           (XSpiHandleT *myHandle, XSpiGeometryT *pInfo);
void XSpi_DumpStatus            (XSpiHandleT *myHandle);
void XSpi_SetAsyncCallbacks     (XSpiHandleT *myHandle, XSpiCallbackT rdDoneCB, XSpiCallbackT wrDoneCB, XSpiCallbackT errorCB);
bool XSpi_Abort                 (XSpiHandleT *myHandle);
int32_t XSpi_SetDeepPowerDown   (XSpiHandleT *myHandle, bool bEna);
bool XSpi_EnableMemoryMappedMode(XSpiHandleT *myHandle);
bool XSpi_SetSpeed              (const HW_DeviceType *self, uint32_t new_clkspeed);
bool XSpi_IsInitialized         (XSpiHandleT *myHandle);
void XSpi_SetRWMode             (XSpiHandleT *myHandle, XSPI_RWMode xspi_mode );
void XSpi_SetQuad               (XSpiHandleT *myHandle, bool bEna );
bool XSpi_ReadWait              (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);
bool XSpi_ReadIT                (XSpiHandleT *myHandle, uint8_t* pData, uint32_t ReadAddr,  uint32_t Size);

bool XSpi_WriteWait             (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool XSpi_WriteIT               (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
bool XSpi_WriteDMA              (XSpiHandleT *myHandle, uint8_t* pData, uint32_t WriteAddr, uint32_t Size);

/* Get different erase sizes and optionally corresponding max erase times */
uint32_t XSpi_GetEraseData      (XSpiHandleT *myHandle, uint32_t *Sizes, uint32_t *Max_Times);

/* Erase <numSect> consecutive block, first block specified by <EraseAddr> */
bool XSpi_EraseWait             (XSpiHandleT *myHandle, XSpi_EraseMode mode, uint32_t EraseAddr, uint32_t numBlock);
bool XSpi_EraseIT               ( XSpiHandleT *myHandle, XSpi_EraseMode mode, uint32_t EraseAddr, uint32_t numBlock);

/* Erase entire chip */
bool XSpi_EraseChipWait         (XSpiHandleT *myHandle);
bool XSpi_EraseChipIT           (XSpiHandleT *myHandle);


void XSpi_EarlyInit             (void);

/* Global variables ---------------------------------------------------------------------*/
#if defined(QUADSPI) && USE_QSPI > 0 
    extern const HW_DeviceType HW_QSPI1;
    extern XSpiHandleT QSpi1Handle;
#endif

#if defined(OCTOSPI1) && USE_OSPI1 > 0 
    extern const HW_DeviceType HW_OSPI1;
    extern XSpiHandleT OSpi1Handle;
#endif

#if defined(OCTOSPI2) && USE_OSPI2 > 0 
    extern const HW_DeviceType HW_OSPI2;
    extern XSpiHandleT OSpi2Handle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __XSPI_DEV_H */
