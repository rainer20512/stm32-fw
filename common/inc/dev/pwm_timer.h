/*
 ******************************************************************************
 * @file    pwm_timer.h 
 * @author  Rainer
 * @brief  PWM timer / PWM channels 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_TIMER_H
#define __PWM_TIMER_H

#include "config/config.h"

#include "hw_device.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct UPC {                   /* One User PWM channel specification, consiting of: */     
  const HW_DeviceType *tmr;            /* Associated Timer                                  */
  uint8_t channel;                     /* Associated Timer channel [1...n]                  */
  uint8_t bInvert;                     /* Output polarity normal or inverted                */
  uint8_t bAutostart;                  /* Start PWM when initializing automatically         */
} PwmChannelT;

int                 PWM_CH_GetIdx(const HW_DeviceType *dev);
const PwmChannelT*  PWM_CH_IterateBegin(void);
const PwmChannelT*  PWM_CH_IterateNext(void);
const PwmChannelT*  PWM_CH_GetCh(uint32_t idx);
void                PWM_CH_InitTimer(const HW_DeviceType *self, void *args);
bool                PWM_CH_Init(const PwmChannelT *pwmch);

/* Public functions for PWM Timers ---------------------------------------------*/
bool     PWM_TMR_SetPWMFrq          (const HW_DeviceType *self, uint32_t frq );
bool     PWM_CH_InitPWMCh          (const HW_DeviceType *self, uint32_t ch, bool invert );
void     PWM_CH_StartPWMChPromille (const HW_DeviceType *self, uint32_t ch, uint32_t promille);
void     PWM_CH_StartPWMChS256     (const HW_DeviceType *self, uint32_t ch, uint32_t s256 );
void     PWM_CH_StopPWMCh          (const HW_DeviceType *self, uint32_t ch);
uint32_t PWM_CH_GetPWMPromille     (const HW_DeviceType *self, uint32_t ch);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /*__PWM_TIMER_H */
