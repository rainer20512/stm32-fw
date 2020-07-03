/*
 ******************************************************************************
 * @file    usb_dev.h 
 * @author  Rainer
 * @brief   USB device functions. 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEV_H
#define __USB_DEV_H

#include "config/config.h"

#if USE_USB > 0 

#include "hardware.h"
#include "hw_device.h"
#include "devices.h"

#include "usbd_def.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum USB_PinEnum {  
    ID_IDX = 0,
    DP_IDX  = 1,
    DM_IDX = 2,
    VBUS_IDX = 3,
} USB_PinEnumType;

typedef struct USBDHandleType {
	USBD_HandleTypeDef hUsb;          /* USBDevice handle structure is included */
} UsbdHandleT; 


#if defined(USB_OTG_FS) && defined(USE_USB)
    extern UsbdHandleT USBDHandle;
    extern const HW_DeviceType HW_USBD;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // USE_CAN > 0 

#endif /* __CAN_DEV_H */
