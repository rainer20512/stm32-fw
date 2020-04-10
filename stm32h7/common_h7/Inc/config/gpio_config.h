/*
 *  SPI-PIN Definition for bitbanging SPI
 *  The SPI Data flow can be unidirectional, so there is no channel for MISO in that case
 */

#pragma once 

#include "config/config.h"
#include "config/devices_config.h"
#include "hardware.h"


#define IO_NO_IRQ                            {-1,0,}

#if defined(STM32H745NUCLEO)
  // LEDs  
  #define IO_00                             { GPIO_PIN_5 , GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_LOW,    IO_NO_IRQ }
  #define IO_01                             { GPIO_PIN_12, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1, } 
  // Blue pushbutton
  #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_02                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, HW_IO_NORMAL,   HW_INPUT,         IO_02_IRQ }

/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

  #define IO_NUM                            3

#else
  #error "No valid device configuration in devices_config.h"
#endif

