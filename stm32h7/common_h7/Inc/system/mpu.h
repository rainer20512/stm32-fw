/**
  ******************************************************************************
  * @file    mpu.h
  * @author  Rainer
  * @brief   Configuration of MPU
  *          see AN4838 and AN4839 for details
  ******************************************************************************
  */

#ifndef __MPU_H__
#define __MPU_H__

/* Different types of MPU areas we can handle */
typedef enum {
    MPUTYPE_RAM_DEVICEMEM_SHARED                = 0,
    MPUTYPE_RAM_NONCACHEABLE                    = 1,
    MPUTYPE_RAM_WT                              = 2,
    MPUTPYE_FLASH_NOWRITE                       = 6,
    MPUTYPE_FLASH_WRITE                         = 7,
    MPUTYPE_FLASH___MAX                         = 8,    /* has to be the last entry */
} MPU_AreaType;

#define MPUTYPE_TEXTS { "Shared Device Memory","Uncached RAM", "Cached RAM w/ Write Thru cache", "undefined",     \
                        "undefined",           "undefined",    "Write protected flash",          "Writeable Flash", }

/* different function return values */
typedef enum {
    MPUERR_OK                                   = 0,
    MPUERR_ILLEGAL_ADDR                         = 1,
    MPUERR_ILLEGAL_TYPE                         = 2,
    MPUERR_ILLEGAL_SIZE                         = 3,
    MPUERR_ILLEGAL_NUM                          = 4,
    MPUERR_NOSPACELEFT                          = 5,
} MPU_ErrorType;

int32_t MPU_AddRegion ( uint32_t rgnAddr, MPU_AreaType rgnType, uint32_t rgnSize, uint8_t rgnNum );
void MPU_EnableAllRegions(void);
void MPU_Dump(void);

#endif /* __MPU_H__ */