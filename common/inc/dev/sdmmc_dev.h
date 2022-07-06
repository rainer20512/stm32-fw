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


typedef struct SdmmcHandleType {
    SD_HandleTypeDef halHandle ;    /* Associated HAL Handle */ 
} SdmmcHandleT; 


/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_SDMMC1)
    extern const HW_DeviceType HW_SDMMC1;
    extern SdmmcHandleT SdmmcHandle1;
#endif
#if defined(USE_SDMMC2)
    extern const HW_DeviceType HW_SDMMC2;
    extern SdmmcHandleT SdmmcHandle2;
#endif

void SDMMC_DumpGeometry(SdmmcHandleT *);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FMC_DEV_H */
