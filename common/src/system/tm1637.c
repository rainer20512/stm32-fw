//  Author: avishorp@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "system/tm1637.h"


#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
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

static const uint8_t minusSegments = 0b01000000;



/* private variables */
    TM1637PinT m_pinClk;
    TM1637PinT m_pinDIO;
    uint8_t m_brightness;
    unsigned int m_bitDelay;

/* forward declarations */
   void bitDelay();

   void start();

   void stop();

   uint32_t writeByte(uint8_t b);

   void showDots(uint8_t dots, uint8_t* digits);
   
   // void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
   void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint32_t leading_zero, uint8_t length, uint8_t pos);

#define INPUT   1
#define OUTPUT  0
#define LOW     0
#define HIGH    1
static void pinInit ( TM1637PinT pin )
{
    GPIO_InitTypeDef Init;
    Init.Pin = 1 << pin.pin;
    Init.Speed = GPIO_SPEED_MEDIUM;
    Init.Mode  = GPIO_MODE_OUTPUT_OD;
    // Display has PullUps mounted */
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

void TM1637Display(TM1637PinT pinClk, TM1637PinT pinDIO, unsigned int bitDelay)
{
    // Copy the pin numbers, verify pin number range
    m_pinClk = pinClk; m_pinClk.pin &= 0x0f;
    m_pinDIO = pinDIO; m_pinDIO.pin &= 0x0f;
    m_bitDelay = bitDelay;
    
    /* Init Pins to OD outputs */
    pinInit(m_pinClk);
    pinInit(m_pinDIO);

    // Set the pin direction and default value.
    // Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
    digitalWrite(m_pinClk, LOW);
    digitalWrite(m_pinDIO, LOW);
}

void setBrightness(uint8_t brightness, uint32_t on)
{
    m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
}

void setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
    start();
    writeByte(TM1637_I2C_COMM1);
    stop();

    // Write COMM2 + first digit address
    start();
    writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

    // Write the data bytes
    for (uint8_t k=0; k < length; k++)
      writeByte(segments[k]);

    stop();

    // Write COMM3 + brightness
    start();
    writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
    stop();
}

void clear()
{
    uint8_t data[] = { 0, 0, 0, 0 };
    setSegments(data,4,0);
}

void showNumberDec(int num, uint32_t leading_zero, uint8_t length, uint8_t pos)
{
  showNumberDecEx(num, 0, leading_zero, length, pos);
}

void showNumberDecEx(int num, uint8_t dots, uint32_t leading_zero, uint8_t length, uint8_t pos)
{
  showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
}

void showNumberHexEx(uint16_t num, uint8_t dots, uint32_t leading_zero, uint8_t length, uint8_t pos)
{
  showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint32_t leading_zero, uint8_t length, uint8_t pos)
{
    uint32_t negative = 0;
    if (base < 0) {
        base = -base;
        negative = 1;
    }

    uint8_t digits[4];

    if (num == 0 && !leading_zero) {
        // Singular case - take care separately
        for(uint8_t i = 0; i < (length-1); i++)
            digits[i] = 0;
        digits[length-1] = encodeDigit(0);
    }
    else {
        for(int i = length-1; i >= 0; --i)
        {
            uint8_t digit = num % base;
                
            if (digit == 0 && num == 0 && leading_zero == 0)
                // Leading zero is blank
                    digits[i] = 0;
            else
                digits[i] = encodeDigit(digit);
                    
            if (digit == 0 && num == 0 && negative) {
                digits[i] = minusSegments;
                negative = 0;
            }

            num /= base;
        }

        if(dots != 0) showDots(dots, digits);
    }
    setSegments(digits, length, pos);
}

void start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}

void stop()
{
    pinMode(m_pinDIO, OUTPUT);
    bitDelay();
    pinMode(m_pinClk, INPUT);
    bitDelay();
    pinMode(m_pinDIO, INPUT);
    bitDelay();
}

uint32_t writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();

  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint32_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);


  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();

  return ack;
}

void showDots(uint8_t dots, uint8_t* digits)
{
    for(int i = 0; i < 4; ++i)
    {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}

uint32_t getUs(void) 
{
    uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
    register uint32_t ms, cycle_cnt;

    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
 
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) 
{
    uint32_t start = getUs();
    while (getUs()-start < (uint32_t) micros) {
        asm("nop");
    }
}

void bitDelay()
{
    delayUs(m_bitDelay);
}

