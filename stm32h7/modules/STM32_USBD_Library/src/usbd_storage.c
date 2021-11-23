/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_storage.c
  * @author  MCD Application Team
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#define USE_EXTMEM_QSPI

/* Includes ------------------------------------------------------------------ */
#include "config/config.h"
#if DEBUG_MODE > 0 && DEBUG_USB > 0
    #include "log.h"
#endif
#include "usbd_storage.h"

#if defined(USE_EXTMEM_QSPI)
    #include "dev/xspi_dev.h"
#else
    #include "stm32h743i_eval_sd.h"
#endif

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
/* USB Mass storage Standard Inquiry Data */
int8_t STORAGE_Inquirydata[] = {  /* 36 */
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

/* Private function prototypes ----------------------------------------------- */
int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num,
                           uint16_t * block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                    uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                     uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);

USBD_StorageTypeDef USBD_DISK_fops = {
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  STORAGE_Inquirydata,
};

/* Private functions --------------------------------------------------------- */

/**
  * @brief  Initializes the storage unit (medium)       
  * @param  lun: Logical unit number
  * @retval Status (0 : OK / -1 : Error)
  */
int8_t STORAGE_Init(uint8_t lun)
{
    UNUSED(lun);
    #if defined(USE_EXTMEM_QSPI)
    #else
      BSP_SD_Init(0);
    #endif
  DBGU_PALAVER("StorageInit");
  return 0;
}

/**
  * @brief  Returns the medium capacity.      
  * @param  lun: Logical unit number
  * @param  block_num: Number of total block number
  * @param  block_size: Block size
  * @retval Status (0: OK / -1: Error)
  */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num,
                           uint16_t * block_size)
{
    UNUSED(lun);
    int8_t ret = -1;

    #if defined(USE_EXTMEM_QSPI)
        *block_num  = QSpi1Handle.geometry.EraseSectorsNumber - 1;
        *block_size = QSpi1Handle.geometry.EraseSectorSize;
        ret = 0;
    #else
        HAL_SD_CardInfoTypeDef info;

        if (BSP_SD_IsDetected(0) != SD_NOT_PRESENT) {
            BSP_SD_GetCardInfo(0, &info);
            *block_num = info.LogBlockNbr - 1;
            *block_size = info.LogBlockSize;
            ret = 0;
        }
    #endif
  DBGU_VERBOSE("StorageCapacity: %d blocks of size %d", *block_num +1, *block_size);
  return ret;
}

/**
  * @brief  Checks whether the medium is ready.  
  * @param  lun: Logical unit number
  * @retval Status (0: OK / -1: Error)
  */
int8_t STORAGE_IsReady(uint8_t lun)
{
    UNUSED(lun);
    int8_t ret = -1;
    #if defined(USE_EXTMEM_QSPI)
        if ( !QSpi1Handle.bAsyncBusy ) ret=0;
    #else
        static int8_t prev_status = 0;

        if (BSP_SD_IsDetected(0) != SD_NOT_PRESENT) {
            if (prev_status < 0) {
                BSP_SD_Init(0);
                prev_status = 0;
            }
            if (BSP_SD_GetCardState(0) == SD_TRANSFER_OK) ret = 0;
        } else if (prev_status == 0) {
            prev_status = -1;
        }
    #endif
  DBGU_PALAVER("StorageIsReady=%d", ret);
  return ret;
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number
  * @retval Status (0: write enabled / -1: otherwise)
  */
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return 0;
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0: OK / -1: Error)
  */
int8_t STORAGE_Read(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                    uint16_t blk_len)
{
    UNUSED(lun);
    int8_t ret = -1;
    #if DEBUG_MODE > 0 && DEBUG_USB > 0
        if ( blk_len == 1 ) {
            DBGU_VERBOSE("StorageRead block %d", blk_addr);
        } else {
            DBGU_VERBOSE("StorageRead block %d...%d", blk_addr, blk_addr+blk_len-1);
        }
    #endif
    #if defined(USE_EXTMEM_QSPI)
        uint32_t byte_addr = blk_addr * QSpi1Handle.geometry.EraseSectorSize;
        uint32_t byte_size = blk_len  * QSpi1Handle.geometry.EraseSectorSize;
        ret = ! XSpi_ReadWait(&QSpi1Handle, buf, byte_addr, byte_size);
    #else
        if (BSP_SD_IsDetected(0) != SD_NOT_PRESENT)
        {
            BSP_SD_ReadBlocks(0, (uint32_t *) buf, blk_addr, blk_len);

            /* Wait until SD card is ready to use for new operation */
            while (BSP_SD_GetCardState(0) != SD_TRANSFER_OK)
            {
            }

            ret = 0;
        }
    #endif

  return ret;
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0 : OK / -1 : Error)
  */
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                     uint16_t blk_len)
{
    UNUSED(lun);
    int8_t ret = -1;

    #if DEBUG_MODE > 0 && DEBUG_USB > 0
        if ( blk_len == 1 ) {
            DBGU_VERBOSE("StorageWrite block %d", blk_addr);
        } else {
            DBGU_VERBOSE("StorageWrite block %d...%d", blk_addr, blk_addr+blk_len-1);
        }
    #endif

    #if defined(USE_EXTMEM_QSPI)
        uint32_t byte_addr = blk_addr * QSpi1Handle.geometry.EraseSectorSize;
        uint32_t byte_size = blk_len  * QSpi1Handle.geometry.EraseSectorSize;
        if ( ! XSpi_EraseSectorWait(&QSpi1Handle, byte_addr,  blk_len ) ) {
            #if DEBUG_MODE > 0 && DEBUG_USB > 0
                LOG_ERROR("Erase %d sectors beginning with sector %d failed", blk_len, blk_addr);
            #endif
            return ret;
        }
        if ( !XSpi_WriteWait(&QSpi1Handle, buf, byte_addr, byte_size) ) {
            #if DEBUG_MODE > 0 && DEBUG_USB > 0
                DBGU_ERROR("Write %d sectors beginning with sector %d failed", blk_len, blk_addr);
            #endif
            return ret;
        }
        ret = 0;
    #else
        if (BSP_SD_IsDetected(0) != SD_NOT_PRESENT)
        {
            BSP_SD_WriteBlocks(0, (uint32_t *) buf, blk_addr, blk_len);

            /* Wait until SD card is ready to use for new operation */
            while (BSP_SD_GetCardState(0) != SD_TRANSFER_OK)
            {
            }

            ret = 0;
        }
    #endif

  return ret;
}

/**
  * @brief  Returns the Max Supported LUNs.   
  * @param  None
  * @retval Lun(s) number
  */
int8_t STORAGE_GetMaxLun(void)
{
  return (STORAGE_LUN_NBR - 1);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
