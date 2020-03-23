/*
 ******************************************************************************
 * @file    epd.h
 * @author  Rainer
 * @brief   defines a function set for an ePaper driver
 *          has to be implemented by a real driver impementation
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EPD_H
#define __EPD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "dev/hw_device.h"

typedef struct
{
  void     (*Init)(const HW_DeviceType*, bool);
  void     (*WriteOnePixel)(uint8_t);              
  void     (*StartConsecutivePixelWrite)(void);
  void     (*WriteConsecutivePixel)(uint8_t);
  void     (*StopConsecutivePixelWrite)(void);
  void     (*WriteConstantByteVector)(uint8_t, uint16_t);

  /* Optimized operation */
  void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*RefreshDisplay)(void);
  void     (*CloseChargePump)(void);
  bool     (*IsBusy) (void );
  void     (*SetBusyIrqCB)( void (*) (uint16_t, void *) );
  uint16_t (*GetEpdPixelWidth)(void);
  uint16_t (*GetEpdPixelHeight)(void);
  void     (*DrawImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*);
}
EPD_DrvTypeDef;


#ifdef __cplusplus
}
#endif

#endif /* EPD_H */

