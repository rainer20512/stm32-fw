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

struct mmc_cid {
	uint8_t 		manfid;
	char			prod_name[8];
	uint32_t		serial;
	uint16_t		oemid;
	uint16_t		year;
	unsigned char		hwrev;
	unsigned char		fwrev;
	unsigned char		month;
};

struct mmc_csd {
	unsigned char		mmca_vsn;
	unsigned short		cmdclass;
	unsigned short		tacc_clks;
	unsigned int		tacc_ns;
	unsigned int		r2w_factor;
	unsigned int		max_dtr;
	unsigned int		read_blkbits;
	unsigned int		write_blkbits;
	unsigned int		capacity;
	unsigned int		read_partial:1,
				read_misalign:1,
				write_partial:1,
				write_misalign:1;
};

struct sd_scr {
	unsigned char		sda_vsn;
	unsigned char		bus_widths;
#define SD_SCR_BUS_WIDTH_1	(1<<0)
#define SD_SCR_BUS_WIDTH_4	(1<<2)
};

typedef struct SdmmcHandleType {
    SD_HandleTypeDef halHandle ;    /* Associated HAL Handle */ 
    
    uint32_t uNumBlocks;            /* Number of R/W sectors */    
    uint32_t uSectorSize;           /* size of one R/W sector ( Words ) */
    uint32_t uEraseBlockSize;       /* erase block size in unit of sectors */
    HAL_StatusTypeDef lastError;    /* error code of last called HAL function */
    struct mmc_csd            csd;
    struct mmc_cid            cid;
    uint8_t bIsInitialized;         /* true, if card is initialized and ready for Xfer */

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

bool    SDMMC_CardReady         (SdmmcHandleT *myHandle);
void    SDMMC_DumpGeometry      (const HW_DeviceType *self);
bool    SDMMC_ReadBlocks        (SdmmcHandleT *myHandle, uint8_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
bool    SDMMC_WriteBlocks       (SdmmcHandleT *myHandle, uint8_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);
void    SDMMC_DebugInfo         (const HW_DeviceType *self);
void    SDMMC_DumpCardStatus    (const HW_DeviceType *self);
void    SDMMC_DumpStatusRegister(const HW_DeviceType *self);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FMC_DEV_H */
