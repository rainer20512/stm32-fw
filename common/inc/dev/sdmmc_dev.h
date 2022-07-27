/*
 ******************************************************************************
 * @file    sdmmc_dev.h 
 * @author  Rainer
 * @brief   Wrapping the SDMMC IP into my HW_DEVICE
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDMMC_DEV_H
#define __SDMMC_DEV_H

#include "config/config.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* returncodes of SDMMC_GetCardState */
#define SDMMC_TRANSFER_OK       0
#define SDMMC_TRANSFER_BUSY     (-1)

typedef struct SdmmcHandleType {
    SD_HandleTypeDef halHandle ;    /* Associated HAL Handle */ 
    
    uint32_t uNumBlocks;            /* Number of R/W sectors */    
    uint32_t uSectorSize;           /* size of one R/W sector ( Words ) */
    uint32_t uEraseBlockSize;       /* erase block size in unit of sectors */
    HAL_StatusTypeDef lastError;    /* return code of last called HAL function */
    uint32_t AsyncErrorCode;        /* error code when performing async (DMA; IT) operations */
    uint8_t bIsInitialized;         /* true, if card is initialized and ready for Xfer */
    uint8_t bPerformingWrite;       /* true, if a write DMA is currently running */
    uint8_t bPerformingRead;        /* true, if a read DMA is currently running */

} SdmmcHandleT; 


/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_SDMMC1)
    extern const HW_DeviceType HW_SDMMC1;
    extern SdmmcHandleT Sdmmc1Handle;
#endif
#if defined(USE_SDMMC2)
    extern const HW_DeviceType HW_SDMMC2;
    extern SdmmcHandleT Sdmmc2Handle;
#endif

bool    SDMMC_CardInTransferState   (SdmmcHandleT *myHandle);
bool    SDMMC_WaitForTransferState  (SdmmcHandleT *myHandle, uint32_t tmo);
void    SDMMC_DumpGeometry          (const HW_DeviceType *self);
bool    SDMMC_ReadBlocks            (SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
bool    SDMMC_ReadBlocksDMA         (SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
bool    SDMMC_WriteBlocks           (SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
bool    SDMMC_WriteBlocksDMA        (SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
void    SDMMC_DebugInfo             (const HW_DeviceType *self);
void    SDMMC_DumpCardStatus        (const HW_DeviceType *self);
void    SDMMC_DumpStatusRegister    (const HW_DeviceType *self);
void    SDMMC_DumpCid               (const HW_DeviceType *self);
void    SDMMC_DumpCsd               (const HW_DeviceType *self);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FMC_DEV_H */
