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
#include "bsp/arduino_wrapper.h"
#include "system/hw_util.h"


/* OD Output */
void pinInitOd ( const ARD_PinT pin, const ARD_PinPullT pp )
{
    HW_SetHWClock(pin.gpio, 1 );
    GPIO_InitTypeDef Init;
    Init.Pin = 1 << pin.pin;
    Init.Speed = GPIO_SPEED_MEDIUM;
    Init.Mode  = GPIO_MODE_OUTPUT_OD;
    Init.Pull  = ( pp ==  PULLUP ? GPIO_PULLUP : pp ==  PULLDN ? GPIO_PULLDOWN : GPIO_NOPULL);   
    /* set pin to high, if pullup is selcted, otherwise to low */
    pin.gpio->BSRR = ( pp == PULLUP ? 1  : 1L<<16 ) << pin.pin;
    HAL_GPIO_Init(pin.gpio, &Init);
}

/* PP Output */
void pinInitPp ( const ARD_PinT pin, const ARD_PinPullT pp )
{
    HW_SetHWClock(pin.gpio, 1 );
    GPIO_InitTypeDef Init;
    Init.Pin = 1 << pin.pin;
    Init.Speed = GPIO_SPEED_MEDIUM;
    Init.Mode  = GPIO_MODE_OUTPUT_PP;
    Init.Pull  = ( pp ==  PULLUP ? GPIO_PULLUP : pp ==  PULLDN ? GPIO_PULLDOWN : GPIO_NOPULL);   
    /* set pin to high, if pullup is selcted, otherwise to low */
    pin.gpio->BSRR = ( pp == PULLUP ? 1  : 1L<<16 ) << pin.pin;
    HAL_GPIO_Init(pin.gpio, &Init);
}


void digitalWrite(const ARD_PinT pin, const ARD_PinValueT value)
{
    uint32_t pinmask = 1 << pin.pin;
    if ( value == LOW ) pinmask <<= 16;
    pin.gpio->BSRR = pinmask;
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

