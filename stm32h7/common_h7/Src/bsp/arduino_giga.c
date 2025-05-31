/**
  ******************************************************************************
  * 
  * @file    arduino_giga.c
  * @author  Rainer
  * @brief   minimal BSP for Arduino Giga board, mainly for debug purposes
  *          only used in the boot phase, later we access the LEDs via IO-device
  *
  ******************************************************************************
  */

#include "config/config.h"

#if defined(ARDUINO_GIGA_R1)

#include "system/arduino_wrapper.h"
#include "bsp/arduino_giga.h"
#include "hardware.h"

static const ARD_PinT myLeds [MAX_LED]= {
    { GPIOI, 12, UNDEF }, // PI12, rd
    { GPIOJ, 13, UNDEF }, // PJ13, gn
    { GPIOE,  3, UNDEF }, // PE3,  bl
};

/* Set to != 0 for every LED, which is powered in inverted mode, i.e. light when pin is LOW and dark when pin is high */
static const uint8_t invertedLeds [MAX_LED]= {
    1,
    1,
    1,
};

#if defined(CORE_CM4)
    #define MULTIPLIER  1024
#else
    #define MULTIPLIER  2048
#endif    
static uint32_t _wait(uint32_t duration );uint32_t _wait(uint32_t duration )
{
    uint32_t k=0;
    for ( uint32_t j=0; j < duration * MULTIPLIER; j++ ) {
        k += j;
        asm("nop");
    }
    return k;
}

void BSP_LED_Init(uint32_t lednum)
{
     if ( lednum < MAX_LED ) {

        /* inverted LEDs are driven by OD-Pin in order to allow higher LED voltages */ 
        if ( invertedLeds[lednum] ) {
            pinInitOd(myLeds[lednum], PULLUP, UNDEF);
        } else {
            pinInitPp(myLeds[lednum], NOPULL,UNDEF);
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

static void _onoff ( uint32_t lednum, uint32_t OnOffVal )
{
    /* be sure, that range of lednum has been checked */
    GPIO_TypeDef *g = myLeds[lednum].gpio;

    /* Set the corresponding Set- or Reset-Bit in BSRR */
    uint32_t bitpos = (OnOffVal ? 1 : 1 << 16) << myLeds[lednum].pin;
    g->BSRR = bitpos;
}


void BSP_PinHigh(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, 1 );    
}

void BSP_PinLow(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, 0 );    
}

void BSP_LED_On(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, invertedLeds[lednum] ? 0 : 1 );    
}

void BSP_LED_Off(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, invertedLeds[lednum] ? 1 : 0 );    
}

    

/******************************************************************************
 * Board specific initialization for Arduino Giga Board:
 * nothing to do
 ******************************************************************************/
void BSP_Board_Init ( void )
{
}

#endif // #if defined(ARDUINO_GIGA_R1)
