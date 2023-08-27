/**
  ******************************************************************************
  *
  * @file    arduino_wrapper.c
  * @author  Rainer
  * @brief   wrapper functions to access the hardware in "arduino style" 
  * 
  ******************************************************************************
  */

#include "config/config.h"
#include "system/arduino_wrapper.h"
#include "system/hw_util.h"

static void _pinOutput  ( const ARD_PinT pin, const ARD_PinPullT pp, const ARD_PinValueT initVal, uint32_t OutDrive )
{
    HW_SetHWClock(pin.gpio, 1 );
    GPIO_InitTypeDef Init;
    Init.Pin = 1 << pin.pin;
    Init.Speed = GPIO_SPEED_LOW;
    Init.Mode  = OutDrive;
    Init.Pull  = ( pp ==  PULLUP ? GPIO_PULLUP : pp ==  PULLDN ? GPIO_PULLDOWN : GPIO_NOPULL);   
    if ( initVal != UNDEF ) {
        /* If initial value is set, set pun value accordingly */
        pin.gpio->BSRR = ( initVal == HIGH ? 1  : 1L<<16 ) << pin.pin;
    } else {
        /* set pin to high, if pullup is selected, otherwise to low */
        pin.gpio->BSRR = ( pp == PULLUP ? 1  : 1L<<16 ) << pin.pin;
    }
    HAL_GPIO_Init(pin.gpio, &Init);
}

/* OD Output */
void pinInitOd ( const ARD_PinT pin, const ARD_PinPullT pp, const ARD_PinValueT initVal )
{
    _pinOutput( pin, pp, initVal, GPIO_MODE_OUTPUT_OD);
}

/* PP Output */
void pinInitPp ( const ARD_PinT pin, const ARD_PinPullT pp, const ARD_PinValueT initVal )
{
    _pinOutput( pin, pp, initVal, GPIO_MODE_OUTPUT_PP);
}


void digitalWrite(const ARD_PinT pin, const ARD_PinValueT value)
{
    if ( value == LOW || value == HIGH ) {
        uint32_t pinmask = 1 << pin.pin;
        if ( value == LOW ) pinmask <<= 16;
        pin.gpio->BSRR = pinmask;
    }
}

ARD_PinValueT digitalRead( const ARD_PinT pin)
{
    uint32_t pinmask = 1 << pin.pin;
    return ( pin.gpio->IDR & pinmask ) != 0 ? HIGH : LOW;
}

// Set pin either input or output
void pinMode( const ARD_PinT pin, const ARD_PinModeT mode )
{
    uint32_t moder;
    uint32_t mask = 0b11;
    uint32_t work;

    if ( mode == INPUT ) 
        moder = 0b00;
    else
        moder = 0b01;
    moder <<= (pin.pin * 2);
    mask  <<= (pin.pin * 2);

    work = pin.gpio->MODER;
    work &= ~mask;
    work |= moder;
    pin.gpio->MODER = work;
}

