/*
 ******************************************************************************
 * @file    fmc_dev.h 
 * @author  Rainer
 * @brief   Wrapping the FMC hardware into my HW_DEVICE
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FMC_DEV_H
#define __FMC_DEV_H

#include "config/config.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Public defines ------------------------------------------------------------------------*/
#define FMC_MAX_BLOCKS      4           /* FMC supports max 4 external mem devices        */

typedef enum {
    FMC_TYPE_SRAM  = 0,
    FMC_TYPE_NOR   = 1,
    FMC_TYPE_NAND  = 2,
    FMC_TYPE_SDRAM = 3,
} FmcTypeE;

typedef union {
#if USE_FMC_SRAM > 0 
    SRAM_HandleTypeDef hsram;
#endif
#if USE_FMC_NOR > 0 
    NOR_HandleTypeDef hnor;
#endif
    /* todo: NAND memory */
#if USE_FMC_SDRAM > 0 
    SDRAM_HandleTypeDef hsdram;
#endif
} FmcHalHandleT;

typedef struct {
    FmcHalHandleT   hHal;               /* Associated HAL handle                          */
    FmcTypeE        fmcType;            /* Memory type                                    */
    uint32_t        fmcCtlBits;         /* Bit mask of used control bits                  */
    uint32_t        fmcAddrBits;        /* Bit mask of used address bits                  */
    uint32_t        fmcDataBits;        /* Bit mask of used data bus bits 8/16            */
    uint8_t         fmcIsMuxed;         /* true iff Addr and Data lines are multiplexed   */
    uint8_t         fmcIsUsed;          /* True, iff this FmcDataT block is config'd/used */
} FmcDataT;

typedef struct FmcHandleType {
    FmcDataT        fmcData[FMC_MAX_BLOCKS];
    uint32_t        allAddrBits;
    uint32_t        allDataBits;
    uint32_t        allCtlBits;
} FmcHandleT; 

// void FMC_SRAM_Init              (QSpiHandleT *myHandle, QSpiGeometryT *pInfo);

/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_FMC)
    extern const HW_DeviceType HW_FMC;
    extern FmcHandleT FmcHandle;
#endif

void FMC_DumpGeometry(void);

//bool Fmc_SRAM_Init(const HW_DeviceType *self, FMC_NORSRAM_InitTypeDef *Init);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FMC_DEV_H */
