/*
 ******************************************************************************
 * @file    qencode.h 
 * @author  Rainer
 * @brief   quadrature decoder on basis ofa timer, that is capable of 
 *          quadrature decoding
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QENCODE_H
#define __QENCODE_H

#include "config/config.h"
#include "devices.h"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif


/* Public typedef -----------------------------------------------------------------------*/
typedef void ( *QEncEncCB ) ( int32_t );        /* Callback for encoder rotation         */
typedef void ( *QEncClickCB ) ( void );         /* Callbacks for Click and DblClick      */

typedef struct QencHandleType {
    TIM_HandleTypeDef     htim;       /* Embedded HAL TIM_HandleTypedef Structure    */
    QEncEncCB             OnRotate;   /* User Callback for Rotation event            */
    QEncClickCB           OnClick;    /* User callback for Click Event               */  
    QEncClickCB           OnDblClick; /* User Callback for DblClick event            */
    uint16_t              last_tmr;   /* timer value at last interrupt or activation */
    uint16_t              last_btnval;/* last pushbutton value                       */
    const HW_Gpio_IO_Type *myIOPin;   /* my associated pin mask ( 0 = unused         */
    int8_t                myTimerId;  /* my Debounce-Timer ID                        */
    uint8_t               bActivated; /* flag for "device is activated"              */
    uint8_t               bDblClkItvl;/* flag for "within DblClickPeriod"            */
    uint8_t               cntPerNotch;/* Counts per Notch                            */
} QEncHandleT; 


void QEnc_SetRotateCallback     (const HW_DeviceType *self, QEncEncCB   onRotateCB   );
void QEnc_SetClickCallback      (const HW_DeviceType *self, QEncClickCB onClickCB    );
void QEnc_SetDblClickCallback   (const HW_DeviceType *self, QEncClickCB onDblClickCB );
void QEnc_SetCallbacks          (const HW_DeviceType *self, QEncEncCB   onRotateCB, QEncClickCB onClickCB, QEncClickCB onDblClickCB );

void QEnc_Activate              (const HW_DeviceType *self);
void QEnc_DeActivate            (const HW_DeviceType *self);

/* Global variables ---------------------------------------------------------------------*/
#if defined(USE_QENCODER)
    extern const HW_DeviceType HW_QENC1;
    extern QEncHandleT QEnc1Handle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __QENCODE_H */
