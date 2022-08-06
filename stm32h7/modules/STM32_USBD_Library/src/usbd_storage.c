/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_storage.c
  * @author  MCD Application Team / Rainer
  * @brief   Implementation of an USB MSC class storage provider
  * @note    This storage provider is just a wrapper that enhances the real
  *          storage provider with debug output ( if configured )
  *          The real storage provider has to implement all methods noted in
  *          "storage_provider.h"
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------ */
#include "config/config.h"

#if USE_USB_MSC > 0

#define DEBUG_USB_STORAGE               1

#if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
    #include "log.h"
#endif

#include "usbd_storage.h"
#include "storage_provider.h"

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
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
  ST_Inquirydata,
};

/* Private functions ------------------------------------------------------- */
#if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
    static const char * get_ret_str( int8_t ret )
    {
        return ret == 0 ? "ok" : "failed";
    }
#endif

/* Neccessary USB MSC class functions -------------------------------------- */

/******************************************************************************
 * @brief  Initializes the storage unit (medium)       
 * @param  lun: Logical unit number
 * @retval Status (0 : OK / -1 : Error)
 ******************************************************************************/
int8_t STORAGE_Init(uint8_t lun)
{
    int8_t ret = ST_Init(lun);
#if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
    DBGU_PALAVER("StorageInit %s\n",get_ret_str(ret));
#endif
    return ret;
}

/******************************************************************************
 * @brief  Returns the medium capacity.      
 * @param  lun: Logical unit number
 * @param  block_num: Number of total block number
 * @param  block_size: Block size
 * @retval Status (0: OK / -1: Error)
 ******************************************************************************/
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num, uint16_t * block_size)
{
    int8_t ret = ST_GetCapacity(lun, block_num, block_size);
#if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
    DBGU_VERBOSE("StorageCapacity: %d blocks of size %d %s", *block_num +1, *block_size, get_ret_str(ret));
#endif
    return ret;
}

/******************************************************************************
 * @brief  Checks whether the medium is ready.  
 * @param  lun: Logical unit number
 * @retval Status (0: OK / -1: Error)
 ******************************************************************************/
int8_t STORAGE_IsReady(uint8_t lun)
{
    int8_t ret = ST_IsReady(lun);
#if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
    DBGU_PALAVER("StorageIsReady %d", get_ret_str(ret));
#endif
    return ret;
}

/******************************************************************************
 * @brief  Checks whether the medium is write protected.
 * @param  lun: Logical unit number
 * @retval Status (0: write enabled / -1: otherwise)
 ******************************************************************************/
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
    int8_t ret = ST_IsWriteProtected(lun);
    return ret;
}

/******************************************************************************
 * @brief  Reads data from the medium.
 * @param  lun: Logical unit number
 * @param  blk_addr: Logical block address
 * @param  blk_len: Blocks number
 * @retval Status (0: OK / -1: Error)
 ******************************************************************************/
int8_t STORAGE_Read(uint8_t lun, uint8_t * buf, uint32_t blk_addr, uint16_t blk_len)
{
    int8_t ret = ST_Read( lun, buf, blk_addr, blk_len );
    #if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
        if ( blk_len == 1 ) {
            DBGU_VERBOSE("StorageRead block %d", blk_addr);
        } else {
            DBGU_VERBOSE("StorageRead block %d...%d", blk_addr, blk_addr+blk_len-1);
        }
        DBGU_VERBOSE(" returns %s\n", get_ret_str(ret));
    #endif

    return ret;
}

/******************************************************************************
 * @brief  Writes data into the medium.
 * @param  lun: Logical unit number
 * @param  blk_addr: Logical block address
 * @param  blk_len: Blocks number
 * @retval Status (0 : OK / -1 : Error)
 ******************************************************************************/
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr, uint16_t blk_len)
{
    int8_t ret = ST_Write( lun, buf, blk_addr, blk_len );

    #if DEBUG_MODE > 0 && DEBUG_USB_STORAGE > 0
        if ( blk_len == 1 ) {
            DBGU_VERBOSE("StorageWrite block %d", blk_addr);
        } else {
            DBGU_VERBOSE("StorageWrite block %d...%d", blk_addr, blk_addr+blk_len-1);
        }
        DBGU_VERBOSE(" returns %s\n", get_ret_str(ret));
    #endif
    return ret;
}

/******************************************************************************
 * @brief  Returns the Max Supported LUNs.   
 * @param  None
 * @retval Lun(s) number
 ******************************************************************************/
int8_t STORAGE_GetMaxLun(void)
{
  int8_t ret = ST_GetMaxLun();
  return ret;
}

#endif /* USE_USB_MSC */
