/*
 ******************************************************************************
 * @file    hw_util.h
 * @author  Rainer
 * @brief   Utility Functions for Hardware access
 *          This file is also platform specific, but neverthless, the header
 *          file should identical across all platforms
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_UTIL_H
#define __HW_UTIL_H

#include "config/config.h"
#include "hardware.h"
#include "dev/hw_device.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define COMBINE(a,b)    (uint16_t)( (a << 8 ) | b )

typedef struct DeviceIdType {
        uint16_t waferX;
        uint16_t waferY;
        uint8_t  waferNum;
        char lot[7];
        uint16_t devID;
        uint16_t revID;
        uint16_t flashSize;
        uint16_t package;
} DeviceIdT;



void *        HW_GetHW                ( uint16_t key );
bool          HW_GetHWClockStatus     ( void *hw );
void          HW_SetHWClock           ( void *hw, bool bOn );
void          HW_SetDmaChClock        ( const HW_DmaType *tx, const HW_DmaType *rx);
void          HW_Reset                ( void *hw );
uint16_t      HW_GetLn2               ( uint16_t pwrof2 );
uint32_t *    HW_GetPeriphBitBandAddr ( __IO uint32_t *periphAddr, uint16_t bit_number );
uint32_t      HW_GetGPIOIdx           (GPIO_TypeDef *gp);
char          HW_GetGPIOLetter        (GPIO_TypeDef *gp);


#define       HW_GetGPIO(a)           (GPIO_TypeDef *) HW_GetHW( COMBINE('G', a) );
#define       HW_GetIdxFromPin(pin)   HW_GetLn2(pin)

void          HW_ReadID               (DeviceIdT *id );
bool          HW_DumpID               (char *cmdline, size_t len, const void * arg );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HW_UTIL_H */
