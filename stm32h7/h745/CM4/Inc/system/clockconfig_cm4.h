/**
  ******************************************************************************
  * @file    clockconfig_cm4.h
  * @author  Rainer
  * @brief   This is the clock configuration part of the CM4 core in a dual core
  *          environment. The only thing to do is to react on clock change event
  *          from master and then notify own devices an callback functions
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCKCONFIG_CM4_H
#define __CLOCKCONFIG_CM4_H

#ifdef __cplusplus
 extern "C" {
#endif


/* 
 * Registration for notification on clock changes
 * ( In addition to devices, which have their own notification mechanism )
 * Registration must specify a callback fn of type "ClockChangeCB"
 */

typedef void ( *ClockChangeCB ) ( uint32_t  );
int32_t ClockRegisterForClockChange ( ClockChangeCB );
void    ClockChangePerform ( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CLOCKCONFIG_CM4_H */
