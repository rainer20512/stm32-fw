/**
 ******************************************************************************
 * @file    storage_impl_sdmmc.c
 * @author  Rainer
  * @brief   Implementation of methods needed by USB MSC class for SDMMC
 ******************************************************************************
 *
 *****************************************************************************/


/* Includes ------------------------------------------------------------------ */
#include "config/config.h"

#if USE_SDMMC > 0

#include "dev/sdmmc_dev.h"

#include "log.h"

#include "usbd_storage.h"
#include "storage_provider.h"
#include "system/profiling.h"

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */

#define STORAGE_LUN_NBR                  1
#define SD_TIMEOUT                       10 * 1000


#if USE_SDMMC1 > 0
    #define HANDLE1                 &Sdmmc1Handle
#else
    #define HANDLE1                 NULL
#endif

#if USE_SDMMC2 > 0
    #define HANDLE2                 &Sdmmc2Handle
#else
    #define HANDLE2                 NULL
#endif

/* Public variables ---------------------------------------------------------- */
/* USB Mass storage Standard Inquiry Data */
int8_t ST_Inquirydata[36] = {  /* 36 */
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer: 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0', '1',           /* Version : 4 Bytes */
};

/* Private SDMMC driver functions ------------------------------------------ */
static SdmmcHandleT *GetHandle(uint8_t lun)
{
    return lun==0 ? HANDLE1 : HANDLE2;
}

/* Public SDMMC driver functions ------------------------------------------- */

/******************************************************************************
 * @brief  Initializes the storage unit (medium)       
 * @param  lun: Logical unit number
 * @retval Status (0 : OK / -1 : Error)
 * @Note: Nothing to do here, SDMMC card has been initialized to "trans" state
 *        on driver init
 *****************************************************************************/
int8_t ST_Init(uint8_t lun)
{
  return GetHandle(lun)-> bIsInitialized ? ST_OK : ST_FAIL;  
}

/**
  * @brief  Returns the medium capacity.      
  * @param  lun: Logical unit number
  * @param  block_num: Number of total block number
  * @param  block_size: Block size
  * @retval Status (0: OK / -1: Error)
  */
int8_t ST_GetCapacity(uint8_t lun, uint32_t * block_num, uint16_t * block_size)
{
    *block_num  = GetHandle(lun)->uNumBlocks-1;
    *block_size = GetHandle(lun)->uSectorSize;
    return *block_num > 0 ? ST_OK : ST_FAIL;
}

/**
  * @brief  Checks whether the medium is ready, if not: wait until SD_TIMEOUT is reached
  *         for card becoming ready.
  * @param  lun: Logical unit number
  * @retval Status (0: OK / -1: Error)
  */
int8_t ST_IsReady(uint8_t lun)
{
    return SDMMC_WaitForTransferState(GetHandle(lun), SD_TIMEOUT ) ? ST_OK : ST_FAIL;
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number
  * @retval Status (0: write enabled / -1: otherwise)
  */
int8_t ST_IsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return ST_OK;
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0: OK / -1: Error)
  */
int8_t ST_Read(uint8_t lun, uint32_t * buf, uint32_t blk_addr, uint32_t blk_count)
{
    int8_t ret      = ST_FAIL;
    SdmmcHandleT* h = GetHandle(lun);

#if 0
    /* Make sure, the SD-Card is ready for transfer */
    if ( !SDMMC_WaitForTransferState(h, 1000 ) ) {
        LOG_ERROR("Timeout when waiting for transfer state");
        return ret;
    }
#endif

    if ( SDMMC_ReadBlocksDMA(h, buf, blk_addr, blk_count) ) {
        uint32_t timeout = HAL_GetTick();
        ProfilerPush(JOB_TASK_WAITRD);
        while((HAL_GetTick() - timeout) < SD_TIMEOUT) {
            if (h->bPerformingRead == 0 ) {
                ret = ST_OK;
                break;
            }
        }
        ProfilerPop();
        if ( ret == ST_OK ) {
            /*
               the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
               adjust the address and the D-Cache size to invalidate accordingly.
               **** 007 ***
               No longer neccessary, as the passed buffer address from USB MSC driver is within uncached mem area
             
              uint32_t alignedAddr = (uint32_t)buf & ~0x1F;
              SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, blk_count*h->uSectorSize + ((uint32_t)buf - alignedAddr));
            */
        } else {
            LOG_ERROR("Timeout while waiting for DMA completion\n");
        }
    }
    return ret;
}

/**
  * @brief  Writes data to the medium.
  * @param  lun: Logical unit number
  * @param  buf: vector of data to be written
  * @param  blk_addr: Logical block address
  * @param  blk_count: Blocks number
  * @retval Status (0: OK / -1: Error)
  */
int8_t ST_Write(uint8_t lun, uint32_t* buf, uint32_t blk_addr, uint32_t blk_count)
{
    int8_t ret      = ST_FAIL;
    SdmmcHandleT* h = GetHandle(lun);
#if 0
    /* Make sure, the SD-Card is ready for transfer */
    if ( !SDMMC_WaitForTransferState(h, 1000 ) ) {
        LOG_ERROR("Timeout when waiting for transfer state");
        return ret;
    }
#endif    
    if ( SDMMC_WriteBlocksDMA(GetHandle(lun), buf, blk_addr, blk_count) ) {
        ProfilerPush(JOB_TASK_WAITWR);
        uint32_t timeout = HAL_GetTick();
        while((HAL_GetTick() - timeout) < SD_TIMEOUT) {
            if (h->bPerformingWrite == 0 ) {
                ret = ST_OK;
                break;
            }
        }
        ProfilerPop();
        if ( ret == ST_OK ) {
        } else {
            LOG_ERROR("Timeout while waiting for DMA completion\n");
        }
    }
    return ret;
}

/**
  * @brief  Returns the Max Supported LUNs.   
  * @param  None
  * @retval Lun(s) number
  */
int8_t ST_GetMaxLun(void)
{
  return (STORAGE_LUN_NBR - 1);
}

#endif /* USE_SDMMC > 0 */
