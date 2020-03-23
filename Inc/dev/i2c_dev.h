/*
 ******************************************************************************
 * @file    i2c.h 
 * @author  Rainer
 * @brief   I2C user functions. 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

#include "config/config.h"

#include "hw_device.h"
#include "config/devices_config.h"
#include "devices.h"
#include "circbuf.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct I2cHandleType I2cHandleT; 
typedef void (*I2cCB) ( I2cHandleT * );

/* Public typedef ---------------------------------------------------------------*/
typedef enum I2c_Modes {
      I2c_StandardMode = 0,
      I2c_FastMode     = 1,
      I2c_FastPlusMode = 2,
      I2c_MaxMode      = 3    /* Last Entry acts as delimiter/Array size designator */
} I2cSpeedModeT;

typedef enum I2c_CModes {
      I2cClock_PCLK1   = 0,
      I2cClock_SYSCLK  = 1,
      I2cClock_HSI     = 2,
      I2C_MaxClock     = 3    /* Last Entry acts as delimiter/Array size designator */
} I2cClockModeT;

typedef struct I2cHandleType {
	I2C_HandleTypeDef hI2c;           /* I2C handle structure is included */
        uint32_t last_errors;             /* Last error flags, only valid on call of error callback */

//        I2cCB OnMasterSendComplete;       /* Callback for Send complete */
//        I2cCB OnMasterReceiveComplete;    /* Callback for Send complete */
//        I2cCB OnError;                    /* Callback on any error */
} I2cHandleT; 


#if defined(I2C1) && defined(USE_I2C1)
    extern I2cHandleT I2C1Handle;
    extern const HW_DeviceType  HW_I2C1;
#endif
#if defined(I2C2) && defined(USE_I2C2)
    extern I2cHandleT I2C2Handle;
    extern const HW_DeviceType  HW_I2C2;
#endif
#if defined(I2C3) && defined(USE_I2C3)
    extern I2cHandleT I2C3Handle;
    extern const HW_DeviceType  HW_I2C3;
#endif

#if defined(I2C4) && defined(USE_I2C4)
    extern I2cHandleT I2C4Handle;
    extern const HW_DeviceType  HW_I2C4;
#endif


/* Public functions ---------------------------------------------------------*/

bool Scan_I2c( char *cmdline, size_t len, const void * arg );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __I2C_H */
