/*
 * TM1637.h
 * A library for the 6 digit TM1637 based segment display
 *
 * Modified for 6 digits and points by: TinyTronics.nl
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Frankie.Chu
 * Create Time: 9 April,2012
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __TM1637_H__
#define __TM1637_H__

#include <inttypes.h>
#include "config/config.h"
#include "hardware.h"

typedef struct {
    GPIO_TypeDef *gpio;         //!< GPIO-Port ( GPIOA ... GPIOx )
    uint32_t pin;               //!< Pin number 0 .. 15
} TM1637PinT;


/**************definitions for brightness***********************/
#define  BRIGHT_DARKEST 0
#define  BRIGHT_TYPICAL 2
#define  BRIGHTEST      7

#define DELAY_TYPICAL 50

void TM1637_Init          (TM1637PinT Clk, TM1637PinT Data, unsigned int bitDelay);
void TM1637_displayInteger(int32_t intdisplay, bool leading_zeros, uint32_t dpAt);
void TM1637_displayHex    (uint32_t hexval,    bool leading_zeros, uint32_t dpAt);
void TM1637_clearDisplay  (void);


#endif // __TM1637_H__