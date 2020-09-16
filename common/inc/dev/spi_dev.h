/*
 ******************************************************************************
 * @file    bbspi_dev.h 
 * @author  Rainer
 * @brief   bit bang spi device, only device functions, nothing else 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_DEV_H
#define __SPI_DEV_H

#include "config/config.h"
#include <string.h>
#include "hardware.h"

#include "hw_device.h"
#include "config/devices_config.h"
#include "devices.h"
#include "circbuf.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct SpiHandleType SpiHandleT;

/* Public typedef ---------------------------------------------------------------*/

typedef enum SPI_PinEnum {  
    MOSI_IDX = 0,
    SCK_IDX  = 1,
    NSEL_IDX = 2,
    MISO_IDX = 3,
    DNC_IDX  = 4,
    RST_IDX  = 5,
    BUSY_IDX = 6,
    INP_IDX  = 7,               /* Additional input Pin */
} SPI_PinEnumType;

#if defined(USE_BBSPI1)
    #define SPIDEV1_DEV         HW_BBSPI1
    #define SPIDEV1_NAME        "BBSPI1"
    #define SPIDEV1_HARDWARE    NULL
    #define SPIDEV1_TYPE        HW_DEVICE_BBSPI
    extern const HW_DeviceType  SPIDEV1_DEV;
    extern SpiHandleT           SPI1Handle;
#endif
#if defined(USE_BBSPI2)
    #define SPIDEV2_DEV         HW_BBSPI2
    #define SPIDEV2_NAME        "BBSPI2"
    #define SPIDEV2_HARDWARE    NULL
    #define SPIDEV2_TYPE        HW_DEVICE_BBSPI
    extern const HW_DeviceType  SPIDEV2_DEV;
    extern SpiHandleT           SPI2Handle;
#endif

#if defined(SPI1) && defined(USE_SPI1)
    #define SPIDEV1_DEV         HW_SPI1
    #define SPIDEV1_NAME        "SPI1"
    #define SPIDEV1_HARDWARE    SPI1
    #define SPIDEV1_TYPE        HW_DEVICE_HWSPI

    extern const HW_DeviceType  SPIDEV1_DEV;
    extern SpiHandleT           SPI1Handle;
#endif
#if defined(SPI2) && defined(USE_SPI2)
    #define SPIDEV2_DEV         HW_SPI2
    #define SPIDEV2_NAME        "SPI2"
    #define SPIDEV2_HARDWARE    SPI2
    #define SPIDEV2_TYPE        HW_DEVICE_HWSPI

    extern const HW_DeviceType  SPIDEV2_DEV;
    extern SpiHandleT           SPI2Handle;
#endif
#if defined(SPI3) && defined(USE_SPI3)
    #define SPIDEV3_DEV         HW_SPI3
    #define SPIDEV3_NAME        "SPI3"
    #define SPIDEV3_HARDWARE    SPI3
    #define SPIDEV3_TYPE        HW_DEVICE_HWSPI

    extern const HW_DeviceType  SPIDEV3_DEV;
    extern SpiHandleT           SPI3Handle;
#endif
#if defined(SPI3) && defined(USE_SPI3)
    extern SpiHandleT SPI3Handle;
#endif

/* Public functions ---------------------------------------------------------*/
const HW_Gpio_AF_Type*  SPI_GetGpio     (const HW_DeviceType *self, SPI_PinEnumType idx);
uint8_t                 SPI_HasMisoIRQ  (const HW_DeviceType *self);
uint8_t                 SPI_HasBusyIRQ  (const HW_DeviceType *self);
uint8_t                 SPI_HasInpIRQ   (const HW_DeviceType *self);
void SpiHandleInit      (SpiHandleT *hnd, const HW_DeviceType *SpiDev);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_DEV_H */
