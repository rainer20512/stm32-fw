/**
  ******************************************************************************
  * @file    devices.h 
  * @author  Rainer
  * @brief   Definition of all used devices. 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICES_H
#define __DEVICES_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "dev/hw_device.h"
#include "dev/uart_dev.h"
#include "dev/i2c_dev.h"
#include "dev/io_dev.h"
#include "dev/spi_dev.h"
#include "dev/adc_dev.h"
#include "dev/qencode.h"
#include "dev/qspi_dev.h"
#include "dev/timer_dev.h"
#include "dev/can_dev.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void     DBG_dump_devices               (bool bLong);
int32_t  AddDevice                      (const HW_DeviceType *dev, HW_DynInitT postInit, HW_DynDeInitT preDeInit);
void     BasicDevInit                   (void);
bool     DeviceInitByIdx                (uint32_t dev_idx, void *arg);
void     DeviceDeInitByIdx              (uint32_t dev_idx);
bool     DevicesInhibitStop             (void);
bool     DevicesInhibitFrqChange        (void);

const HW_DeviceType *FindDevByBaseAddr  (uint32_t , void *pAddress );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEVICES_H */

