/*
 ******************************************************************************
 * @file    io_dev.h 
 * @author  Rainer
 * @brief   plain input/output pins, w or w/o interrupt
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_DEV_H
#define __IO_DEV_H

#include "config/config.h"
#include "system/exti_handler.h"
#include "hw_device.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif


/* Public typedef ---------------------------------------------------------------*/


/* Public functions -------------------------------------------------------------*/

void IO_AssignInterrupt     (uint16_t pin, ExtIrqCB cb );

uint8_t IO_UseLedGetNum     ( void );
void IO_OutputHigh          ( uint16_t pinnum );
void IO_OutputLow           ( uint16_t pinnum );
void IO_OutputToggle        ( uint16_t pinnum );
void IO_UserLedOn           ( uint8_t idx );
void IO_UserLedOff          ( uint8_t idx );
void IO_UserLedToggle       ( uint8_t idx );
void IO_UserLedBlink        ( uint8_t idx, uint32_t toggles, uint32_t ms );

/**** 001 **** Use gn LED as "activity LED" whoen using FreeRTOS */
#define ACTIVITY_LED        0           

/* PGlobal variables ------------------------------------------------------------*/
extern const HW_DeviceType HW_IO;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __IO_DEV_H */
