/**
  ******************************************************************************
  *
  * @file    arduino_wrapper.h
  * @author  Rainer
  * @brief   wrapper functions to access the hardware in "arduino style" 
  * 
  ******************************************************************************
  */
#ifndef __ARDUINO_WRAPPER_H
#define __ARDUINO_WRAPPER_H


#include "hardware.h"

/* for pinMode wrapper */
typedef enum {
    OUTPUT = 0,
    INPUT  = 1,
} ARD_PinModeT;

typedef enum {
    PULLDN = 0,
    PULLUP = 1,
    NOPULL = 2,
    AUTO   = 3,                     // Only valid during initialization: set according to initial value
} ARD_PinPullT;

/* digitalRead and digitalWrite wrappers */
typedef enum {
    LOW    = 0,                     
    HIGH   = 1,
    UNDEF  = 2,                     // Only valid during initialization: Let initial output value undefined
} ARD_PinValueT;

typedef struct {
    GPIO_TypeDef    *gpio;         //!< GPIO-Port ( GPIOA ... GPIOx )
    uint32_t        pin;           //!< Pin number 0 .. 15
    ARD_PinValueT   initVal;       //!< Initial Value, not relevant for LED pins    
} ARD_PinT;



#define LOW     0
#define HIGH    1

void            pinInitOd ( const ARD_PinT pin, const ARD_PinPullT pp, const ARD_PinValueT initVal );
void            pinInitPp ( const ARD_PinT pin, const ARD_PinPullT pp, const ARD_PinValueT initVal );
void            digitalWrite(const ARD_PinT pin, const ARD_PinValueT value);
ARD_PinValueT   digitalRead(const ARD_PinT pin);
void            pinMode(const ARD_PinT pin, const ARD_PinModeT mode );

#endif // __ARDUINO_WRAPPER_H