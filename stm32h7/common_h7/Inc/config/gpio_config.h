/*
 *  SPI-PIN Definition for bitbanging SPI
 *  The SPI Data flow can be unidirectional, so there is no channel for MISO in that case
 */

#pragma once 

#include "config/config.h"
#include "config/devices_config.h"
#include "hardware.h"


#define IO_NO_IRQ                            {-1,0,}

#if defined(STM32L476EVAL)
  // LEDs  
  #define IO_00                             { GPIO_PIN_2 , GPIOB, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,    IO_NO_IRQ }
  #define IO_01                             { GPIO_PIN_1,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   IO_NO_IRQ }
/*
  #define IO_02                             { GPIO_PIN_7,  GPIOG, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define IO_03                             { GPIO_PIN_8,  GPIOG, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define IO_04                             { GPIO_PIN_15, GPIOE, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define IO_05                             { GPIO_PIN_11, GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   IO_NO_IRQ }
*/
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1 } 
  // Blue pushbutton
  #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_02                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, HW_IO_NORMAL,   HW_INPUT,         IO_02_IRQ }

/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

  #define IO_NUM                            3

#elif defined(STM32L476NUCLEO)
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

#elif defined(STM32L476BAREMETAL)
  // LEDs  
  #define IO_01                             GPIO_PIN_2 , GPIOC, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH
  #define IO_02                             GPIO_PIN_3 , GPIOC, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH
  #define IO_03                             GPIO_PIN_4 , GPIOC, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2,}

  #define IO_NUM                            3
#elif defined(BL475IOT)

#elif defined(DRAGONFLY476)
  // LEDs  
  #define IO_01                             { GPIO_PIN_2 , GPIOB, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ }
  #define IO_02                             { GPIO_PIN_10, GPIOA, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ }
  #define IO_03                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2,}

  #define IO_NUM                            3

#else
  #error "No valid device configuration in devices_config.h"
#endif

