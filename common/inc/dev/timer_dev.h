/*
 ******************************************************************************
 * @file    timer_dev.h 
 * @author  Rainer
 * @brief   Timer device, only device functions, nothing else 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_DEV_H
#define __TIMER_DEV_H

#include "config/config.h"
#include <string.h>
#include "hardware.h"

#include "hw_device.h"
#include "config/devices_config.h"
#include "config/timer_config.h"
// #include "devices.h"
#include "circbuf.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct TimerHandleType {
    TIM_HandleTypeDef   myHalHnd;             /* my associated HAL Timer handle    */
    volatile uint32_t   MicroCountHigh;       /* extended counter for BaseTimer    */
    int32_t             reference_cnt;        /* counter for active references of BaseTimer */
    uint32_t            bTicksEnabled;        /* If Bastimer is also tick generator, this flag may inhibit tick generation */
    uint8_t             use_chX[4];           /* flag for "ChannelX is used"       */
    uint8_t             bIs32bit;             /* true, if timer is 32bit wide */
} TimerHandleT;

/* Public typedef ---------------------------------------------------------------*/

#if defined(TIM1) && defined(USE_TIM1)
    extern const HW_DeviceType  HW_TIM1;
    extern TimerHandleT         TIM11Handle;
#endif

#if defined(TIM2) && defined(USE_TIM2)
    extern const HW_DeviceType  HW_TIM2;
    extern TimerHandleT         TIM12Handle;
#endif

#if defined(TIM3) && defined(USE_TIM3)
    extern const HW_DeviceType  HW_TIM3;
    extern TimerHandleT         TIM13Handle;
#endif

/* Base Timers -----------------------------------------------------------------*/
#if defined(TIM6) && defined(USE_TIM6)
    extern const HW_DeviceType  HW_TIM6;
    extern TimerHandleT         TIM6Handle;
#endif
#if defined(TIM7) && defined(USE_TIM7)
    extern const HW_DeviceType  HW_TIM7;
    extern TimerHandleT         TIM7Handle;
#endif

/* Peripheral Timer -----------------------------------------------------------------*/
#if defined(TIM15) && defined(USE_TIM15)
    extern const HW_DeviceType  HW_TIM15;
    extern TimerHandleT         TIM15Handle;
#endif

/* Public functions for all types of Timers ------------------------------------*/
void        TMR_Aquire              (TimerHandleT *hnd);
void        TMR_Release             (TimerHandleT *hnd);
void        TMR_Start               (const HW_DeviceType *dev, bool bIntEnable);
void        TMR_Stop                (const HW_DeviceType *dev);
int32_t     TmrGetClockPrescaler    ( TIM_TypeDef *tim );
uint32_t    TmrGetClockFrq          ( TIM_TypeDef *tim );

/* Public functions for BasicTimers --------------------------------------------*/
void        BASTMR_IrqHandler       ( volatile uint32_t *CntHigh, TIM_TypeDef *htim);
uint32_t    BASTMR_GetMicrosecond   (TimerHandleT *hnd);
uint16_t    BASTMR_GetRawValue      (TimerHandleT *hnd);
void        BASTMR_EarlyInit        (void);
void        BASTMR_DelayUs          (uint32_t delta_us );

/* Public functions for PWM Timers ---------------------------------------------*/
bool TMR_SetPWMFrq          (const HW_DeviceType *self, uint32_t frq );
bool TMR_InitPWMCh          (const HW_DeviceType *self, uint32_t ch, bool invert );
void TMR_StartPWMChPromille (const HW_DeviceType *self, uint32_t ch, uint32_t promille);
void TMR_StartPWMChS256     (const HW_DeviceType *self, uint32_t ch, uint32_t s256 );
void TMR_StopPWMCh          (const HW_DeviceType *self, uint32_t ch);

void PeriphTimer_StartStop(uint32_t bStart);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_DEV_H */
