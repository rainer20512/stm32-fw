/*
 * TM1637_6D.cpp
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

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "system/tm1637.h"
#include "system/hw_util.h"

/* for pinMode, digitalRead and digitalWrite wrappers */
#define INPUT   1
#define OUTPUT  0
#define LOW     0
#define HIGH    1

/************definitions for TM1637*********************/
#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define STARTADDR  0xc0

/* Arduino Framwork Wrappers for IO functions */

static void pinInit ( TM1637PinT pin )
{
    HW_SetHWClock(pin.gpio, 1 );
    GPIO_InitTypeDef Init;
    Init.Pin = 1 << pin.pin;
    Init.Speed = GPIO_SPEED_MEDIUM;
    Init.Mode  = GPIO_MODE_OUTPUT_OD;
    /* Display has PullUps mounted, so  */
    Init.Pull  = GPIO_NOPULL;   
    pin.gpio->BSRR = 1 << pin.pin;
    HAL_GPIO_Init(pin.gpio, &Init);
}

void digitalWrite(TM1637PinT pin, uint32_t value)
{
    uint32_t pinmask = 1 << pin.pin;
    if ( value == LOW ) pinmask <<= 16;
    pin.gpio->BSRR = pinmask;
}

uint32_t digitalRead(TM1637PinT pin)
{
    uint32_t pinmask = 1 << pin.pin;
    return ( pin.gpio->IDR & pinmask ) != 0;
}

// Set pin either input or output
static void pinMode(TM1637PinT pin, uint32_t mode )
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


#if 0
static uint32_t getUs(void) 
{
    uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
    register uint32_t ms, cycle_cnt;

    do {
        ms = HAL_GetTick();
        cycle_cnt = TIM6->CNT;
    } while (ms != HAL_GetTick());
 
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}


static void delayUs(uint16_t micros) 
{
    uint32_t start = getUs();
    while (getUs()-start < (uint32_t) micros) {
        asm("nop");
    }
}
#endif

uint32_t delayUs ( uint32_t micros )
{
    uint32_t k = 0;
    for ( uint32_t j=0; j < micros * 100; j++ ) {
        k += j;
        asm("nop");
    }
    return k;
}


/* End of arduiono wrappers */



static uint8_t Cmd_SetData;
static uint8_t Cmd_SetAddr;
static uint8_t Cmd_DispCtrl;
static uint16_t m_bitDelay;
static TM1637PinT Clkpin;
static TM1637PinT Datapin;



static void     displayError();
static uint32_t writeByte(uint8_t);          //write 8bit data to tm1637
static void     start(void);                //send start bits
static void     stop(void);                 //send stop bits
static void     bitDelay();
static void     displayAll(void);
//static void     displayOne(uint8_t BitAddr,uint8_t DispData,uint8_t DispPointData);
static uint8_t  code(uint8_t,uint8_t);


//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
#define  MAX_DIGIT         15                            // 16 digits, 0..9, A..F
const uint8_t digitToSegment[MAX_DIGIT+1] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

#define DISP_DP       0b10000000                        // DP is bit 8
#define DISP_MINUS    0b01000000
#define DISP_BLANK    0b00000000


void TM1637_Init(TM1637PinT Clk, TM1637PinT Data, unsigned int bitDelay)
{
    // Copy the pin numbers, verify pin number range
    Clkpin = Clk;   Clkpin.pin &= 0x0f;
    Datapin = Data; Datapin.pin &= 0x0f;
    m_bitDelay = bitDelay;

    //Init brightness
    Cmd_DispCtrl = 0x88 + BRIGHT_TYPICAL;
   
    /* Init Pins to OD outputs */
    pinInit(Clkpin);
    pinInit(Datapin);

    /*
     * Set the pin direction and default value.
     * Both pins are set as inputs, allowing the pull-up resistors to pull them up
     * -> We generate high by switching to input an rely to the on board pullups
     * requires 5V tolerant inputs or PD resistor at gpio pin to keep input level
     * below 3V3.
     */
    pinMode(Clkpin, INPUT);
    pinMode(Datapin,INPUT);
    digitalWrite(Clkpin, LOW);
    digitalWrite(Datapin, LOW);

    TM1637_clearDisplay();
}

void start()
{
  pinMode(Datapin, OUTPUT);
  bitDelay();
}

void stop()
{
    pinMode(Datapin, OUTPUT);
    bitDelay();
    pinMode(Clkpin, INPUT);
    bitDelay();
    pinMode(Datapin, INPUT);
    bitDelay();
}

static uint32_t writeByte(uint8_t data)
{
  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(Clkpin, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(Datapin, INPUT);
    else
      pinMode(Datapin, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(Clkpin, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(Clkpin, OUTPUT);
  pinMode(Datapin, INPUT);
  bitDelay();

  // CLK to high
  pinMode(Clkpin, INPUT);
  bitDelay();
  uint32_t ack = digitalRead(Datapin);
  if (ack == 0)
    pinMode(Datapin, OUTPUT);


  bitDelay();
  pinMode(Clkpin, OUTPUT);
  bitDelay();

  return ack;
}


// convert '0' .. '9', ' ' and '-' to their respective 7Seg patterns
static uint8_t code(uint8_t digit, uint8_t dp)
{
    /* handle special characters ( ' ' and '-' ) */
    if(digit >= '0' && digit <= '0' + MAX_DIGIT ) {
        /*  '0', '1', ... */
        digit -= '0';
        digit = digitToSegment[digit];
    } else if ( digit == ' ' ) 
        /* blank */
        digit = DISP_BLANK;
    else
        /* minus and all other codes are displayed as '-' */
        digit = DISP_MINUS;

    if ( dp ) digit |= DISP_DP;

    return digit;
}

#define INTSTR_SIZE 6
static int8_t digits[INTSTR_SIZE];
static int8_t dpVect[INTSTR_SIZE];

/* Display order is somewhat weird */
const uint32_t seq[INTSTR_SIZE] = {3,4,5,0,1,2};

//display function.Write to full-screen.
static void displayAll(void)
{
    int32_t i;
    uint32_t order; 

    start();          //start signal sent to TM1637 from MCU
    writeByte(ADDR_AUTO);//
    stop();           //
    start();          //
    writeByte(Cmd_SetAddr);//

    for(i=0;i < INTSTR_SIZE;i ++) {
        order = seq[i];
        writeByte( code(digits[order], dpVect[order]) );       
    }
    stop();           //
    start();          //
    writeByte(Cmd_DispCtrl);//
    stop();           //
}


/******************************************
static void displayOne(uint8_t BitAddr,uint8_t digit,uint8_t dp)
{
    uint8_t SegData = code(digit, dp);
    start();          //start signal sent to TM1637 from MCU
    writeByte(ADDR_FIXED);//
    stop();           //
    start();          //
    writeByte(BitAddr|STARTADDR);//
    writeByte(SegData);//
    stop();            //
    start();          //
    writeByte(Cmd_DispCtrl);//
    stop();           //
}
*/
// Displays 6 dashes are error marker
void displayError()
{
    memset(digits, '-', INTSTR_SIZE);
    memset(dpVect, 0,   INTSTR_SIZE);
    displayAll();
}


// convert an uint ( max 999999 ) into a sequence of 6 ascii digits
// in reversed order
static void convert( uint32_t value, bool leading_zeros, uint32_t radix )
{
    uint32_t rest;
    for ( int32_t i = 0; i < INTSTR_SIZE; i++ ) {
        rest = value % radix;
        value /= radix;
        if ( value == 0 && rest == 0 && !leading_zeros && i > 0 )  {
            digits[i] = ' ';
        } else {
            digits[i] = '0' + rest;
        }
    }
}

/******************************************************************************
 * Display integer on 6 digits, i.e. range is from -99999 to 999999
 * 
 * dpat specifies index of DP, dpAT >= INTSTR_SIZE means: no dp at all 
 *****************************************************************************/
void TM1637_displayInteger(int32_t intdisplay, bool leading_zeros, uint32_t dpAt)
{

    memset(digits, ' ', INTSTR_SIZE); // Preset with blanks
    memset(dpVect, 0,   INTSTR_SIZE); // No dp

    // if the integer is bigger than 6 characters, display an error(dashes)
    if(intdisplay > 999999 || intdisplay < -99999) {
        displayError();
        return;
    }

    if(intdisplay < 0) {
        convert(intdisplay*-1, leading_zeros, 10);
        digits[INTSTR_SIZE-1] = '-';  // add minus
    } else { 
        convert(intdisplay, leading_zeros, 10);
    }

    /* Set dp if specified */
    if ( dpAt < INTSTR_SIZE ) {
        dpVect[INTSTR_SIZE - 1 - dpAt] = 1;
    }
    displayAll();
}

/******************************************************************************
 * Display unsigned hex on max 6 digits, i.e. range is from 0 to 0xFFFFFF
 * only unsigned hex values
 * dpAt specifies index of DP, dpAT >= INTSTR_SIZE means: no dp at all 
 *****************************************************************************/
void TM1637_displayHex(uint32_t hexval, bool leading_zeros, uint32_t dpAt)
{

    memset(digits, ' ', INTSTR_SIZE); // Preset with blanks
    memset(dpVect, 0,   INTSTR_SIZE); // No dp

    // if the integer is bigger than 6 characters, display an error(dashes)
    if(hexval > 0xFFFFFF ) {
        displayError();
        return;
    }

    convert(hexval, leading_zeros, 16);

    /* Set dp if specified */
    if ( dpAt < INTSTR_SIZE ) {
        dpVect[INTSTR_SIZE - 1 - dpAt] = 1;
    }
    displayAll();
}


void TM1637_clearDisplay(void)
{
  memset(digits, ' ', INTSTR_SIZE); // Preset with blanks
  memset(dpVect, 0,   INTSTR_SIZE); // No dp
  displayAll();
}
//To take effect the next time it displays.
void TM1637_set(uint8_t brightness,uint8_t SetData,uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
}

static void bitDelay()
{
    delayUs(m_bitDelay);
}




