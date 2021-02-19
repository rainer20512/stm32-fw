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

typedef struct {
  void ( *USBD_CDC_OnInit )          ( void );
  void ( *USBD_CDC_OnDeInit )        ( void );
  void ( *USBD_CDC_OnChngCommParams) ( uint32_t  baudrate, uint32_t  stopbits, uint32_t  parity, uint32_t  wordlength );
  void ( *USBD_CDC_QueryCommParams)  ( uint32_t *baudrate, uint32_t *stopbits, uint32_t *parity, uint32_t *wordlength );
  void ( *USBD_CDC_OnReceive    )    ( uint8_t  *buf, uint32_t rxlen );
} USBD_CDC_CallbacksT;

typedef struct USBDHandleType {
    USBD_HandleTypeDef hUsb;                     /* USBDevice handle structure is included */
    USBD_CDC_CallbacksT cdc_callbacks;           /* All possible callbacks for CDC type device */
} UsbdHandleT; 

/* Public functions -------------------------------------------------------------------------- */
void USBD_CDC_SetCallbacks   ( USBD_CDC_CallbacksT *);
void USBD_CDC_StartReceive   ();

/* Transmit a buffer by copy to internal transmit buffer */
void USBD_CDC_CopyTxBuffer ( uint8_t  *buf, uint32_t txlen );

/* Get the internal transmit buffer an trigger a transmit of internal write buffer */
void USBD_CDC_GetTtansmitBuffer ( uint8_t **buf, uint32_t *maxlen );

/* Start transmission of internal transmit buffer */
void USBD_CDC_Transmit ( uint32_t actlen );


/* Public Variables -------------------------------------------------------------------------- */
#if defined(USB2_OTG_FS) && defined(USE_USB)
    extern UsbdHandleT USBDFSHandle;
    extern const HW_DeviceType HW_USBDFS;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // USE_CAN > 0 

#endif /* __USB_DEV_H */
