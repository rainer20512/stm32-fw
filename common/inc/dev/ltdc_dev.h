/*
 ******************************************************************************
 * @file    ltdc_dev.h 
 * @author  Rainer
 * @brief   Wrapping the LTDC hardware into my HW_DEVICE
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LTDC_DEV_H
#define __LTDC_DEV_H

#include "config/config.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif


/* 
 * Supported types of external memory
 * cannot be put into enum, is evaluated by preprocessor 
 */

#define FMC_TYPE_SRAM       0
#define FMC_TYPE_NOR        1
#define FMC_TYPE_NAND       2
#define FMC_TYPE_SDRAM      3


typedef struct LtdcHandleType {
    FmcDataT        fmcData[FMC_MAX_BLOCKS];
    uint32_t        allAddrBits;
    uint32_t        allDataBits;
    uint32_t        allCtlBits;
} LtdcHandleT; 

// void FMC_SRAM_Init              (QSpiHandleT *myHandle, XSpiGeometryT *pInfo);

/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_LTDC)
    extern const HW_DeviceType HW_LTDC;
    extern LtdcHandleT LtdcHandle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __LTDC_DEV_H */
