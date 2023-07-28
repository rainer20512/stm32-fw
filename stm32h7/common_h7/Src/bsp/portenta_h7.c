/**
  ******************************************************************************
  * 
  * @file    portenta_h7.c
  * @author  Rainer
  * @brief   minimal BSP for Portenta H7 board, mainly for debug purposes
  *          only used in the boot phase, later we access the LEDs via IO-device
  *
  ******************************************************************************
  */

#include "config/config.h"
#include "bsp/arduino_wrapper.h"
#include "bsp/portenta_h7.h"
#include "hardware.h"

static const ARD_PinT myLeds [MAX_LED]= {
    { GPIOK, 5 }, // PK5, rd
    { GPIOK, 6 }, // PK6, gn
    { GPIOK, 7 }, // PK7, bl
};

/* Set to != 0 for every LED, which is powered in inverted mode, i.e. light when pin is LOW and dark when pin is high */
static const uint8_t invertedLeds [MAX_LED]= {
    1,
    1,
    1,
};

void BSP_LED_Init(uint32_t lednum)
{
     if ( lednum < MAX_LED ) {

        /* inverted LEDs are driven by OD-Pin in order to allow higher LED voltages */ 
        if ( invertedLeds[lednum] ) {
            pinInitOd(myLeds[lednum], PULLUP);
        } else {
            pinInitPp(myLeds[lednum], NOPULL);
        }
     }
}

void BSP_LED_Toggle(uint32_t lednum)
{
     if ( lednum < MAX_LED ) {
    
        GPIO_TypeDef *g = myLeds[lednum].gpio;
        g->ODR ^= ( 1 << myLeds[lednum].pin );
     }
}
    
const ARD_PinT oscOnOff = {GPIOH,1};

void BSP_Board_Init ( void )
{
/* 25 MHz Oscillator must be started by setting PH1 to 1 */
    pinInitPp(oscOnOff, NOPULL);
    digitalWrite(oscOnOff, HIGH);
}

