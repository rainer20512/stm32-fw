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
} TimerHandleT;

/* Public typedef ---------------------------------------------------------------*/

#if defined(TIM1) && defined(USE_TIM1)
    extern const HW_DeviceType  HW_TIM1;
    extern TimerHandleT           TIM11Handle;
#endif

#if defined(TIM3) && defined(USE_TIM3)
    extern const HW_DeviceType  HW_TIM3;
    extern TimerHandleT           TIM13Handle;
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

/* Public functions for BasicTimers --------------------------------------------*/
void        BASTMR_IrqHandler       ( volatile uint32_t *CntHigh, TIM_TypeDef *htim);
uint32_t    BASTMR_GetMicrosecond   (TimerHandleT *hnd);
uint16_t    BASTMR_GetRawValue      (TimerHandleT *hnd);
void        BASTMR_EarlyInit        (void);
void        BASTMR_Aquire           (TimerHandleT *hnd);
void        BASTMR_Release          (TimerHandleT *hnd);
void        BASTMR_DelayUs          (uint32_t delta_us );

/* Public functions for PWM Timers ---------------------------------------------*/
bool TMR_SetPWMFrq          (const HW_DeviceType *self, uint32_t frq );
bool TMR_InitPWMCh          (const HW_DeviceType *self, uint32_t ch, bool invert );
void TMR_StartPWMChPromille (const HW_DeviceType *self, uint32_t ch, uint32_t promille);
void TMR_StartPWMChS256     (const HW_DeviceType *self, uint32_t ch, uint32_t s256 );
void TMR_StopPWMCh          (const HW_DeviceType *self, uint32_t ch);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_DEV_H */
