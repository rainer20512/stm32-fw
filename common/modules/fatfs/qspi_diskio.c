/**
  ******************************************************************************
  * @file    qspi_diskio.c
  * @author  Rainer
  * @brief   QSPI-IODriver for FatFs
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"
#include "ff_gen_drv.h"
#include "dev/xspi_dev.h"

#include "log.h"


#if USE_FATFS > 0

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define MIN_SECTOR_SIZE         512     /* FatFs min Sector Size is 512       */
/* use the default SD timeout as defined in the platform BSP driver*/


/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
static DSTATUS XspiIo_CheckStatus(BYTE lun);

DSTATUS XspiIo_initialize (BYTE);
DSTATUS XspiIo_status (BYTE);
DRESULT XspiIo_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT XspiIo_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT XspiIo_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  FatFsXspi_Driver =
{
  XspiIo_initialize,
  XspiIo_status,
  XspiIo_read,
#if  _USE_WRITE == 1
  XspiIo_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  XspiIo_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

static DSTATUS XspiIo_CheckStatus(BYTE lun)
{
  UNUSED(lun);  
    
  Stat =  XSpi_IsInitialized(&FATFS_HND) ?  0 : STA_NOINIT;
  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS XspiIo_initialize(BYTE lun)
{
    return XspiIo_CheckStatus(lun);
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS XspiIo_status(BYTE lun)
{
  return XspiIo_CheckStatus(lun);
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT XspiIo_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    UNUSED(lun);
    DRESULT res = RES_ERROR;

    uint32_t byte_addr = sector * FATFS_HND.geometry.EraseSectorSize;
    uint32_t byte_size = count  * FATFS_HND.geometry.EraseSectorSize;
    DBGU_VERBOSE("FatFS read %d Sector(s), start=%d, Mem=%p...%p", count, sector, byte_addr, byte_addr+byte_size-1); 

    if (XSpi_ReadWait(&FATFS_HND, buff, byte_addr, byte_size)) {
        res = RES_OK;
    } else {
        #if DEBUG_MODE > 0 
            DBGU_ERROR("FatFS: Read %d sectors beginning with sector %d failed", sector, count);
        #endif
    }

    DBGU_VERBOSE("FatFS read done", count, sector, byte_addr, byte_addr+byte_size-1); 

    return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT XspiIo_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    UNUSED(lun);
    DRESULT res = RES_ERROR;

    uint32_t byte_addr = sector * FATFS_HND.geometry.EraseSectorSize;
    uint32_t byte_size = count  * FATFS_HND.geometry.EraseSectorSize;

    DBGU_VERBOSE("FatFS Write %d Sector(s), start=%d, Mem=%p...%p", count, sector, byte_addr, byte_addr+byte_size-1); 
    if ( ! XSpi_EraseSectorWait(&FATFS_HND, byte_addr, count ) ) {
        #if DEBUG_MODE > 0 
            DBGU_ERROR("FatFS: Erase %d sectors beginning with sector %d failed", sector, count);
        #endif
        return res;
    }
    if ( !XSpi_WriteWait(&FATFS_HND, (uint8_t *)buff, byte_addr, byte_size) ) {
        #if DEBUG_MODE > 0 
            DBGU_ERROR("FatFS: Write %d sectors beginning with sector %d failed", sector, count);
        #endif
        return res;
    }
    
    #if DEBUG_MODE > 0 
        DBGU_VERBOSE("FatFS: Write done\n", sector, count);
    #endif

    return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT XspiIo_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    UNUSED(lun);
    DRESULT res = RES_ERROR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {

    case CTRL_SYNC :                /* Make sure that no pending write process */
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT :             /* Get number of sectors on the disk (DWORD) */
        *(DWORD*)buff =  FATFS_HND.geometry.EraseSectorsNumber;
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE :              /* Get R/W sector size (WORD) */
        *(WORD*)buff = FATFS_HND.geometry.EraseSectorSize;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE :               /* Get erase block size in unit of sector (DWORD) */
        *(DWORD*)buff =  1;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

#endif /* USE_FATFS > 0 */
