/*
 *  SPI-PIN Definition for bitbanging SPI
 *  The SPI Data flow can be unidirectional, so there is no channel for MISO in that case
 */

#pragma once 

#include "config/config.h"
#include "config/devices_config.h"
#include "stm32l4xx_hal.h"



#if defined(STM32L476EVAL)
  // LEDs  
  #define IO_00                             { GPIO_PIN_2 , GPIOB, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "UserLed1" }
  #define IO_01                             { GPIO_PIN_1,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "UserLed2" }
/*
  #define IO_02                             { GPIO_PIN_7,  GPIOG, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 1"  }
  #define IO_03                             { GPIO_PIN_8,  GPIOG, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 2"  }
  #define IO_04                             { GPIO_PIN_15, GPIOE, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 3"  }
  #define IO_05                             { GPIO_PIN_11, GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED,   HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 4"  }
*/
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "UserLed1" }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "UserLed2" }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1 } 
  // Blue pushbutton
  #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_02                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, HW_IO_NORMAL,   HW_INPUT,         IO_02_IRQ, "Blue PushBtn"  }

/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    GPIO_NO_IRQ, "GpioOutput" }
*/

  #define IO_NUM                            3

#elif defined(STM32L4R9DISCOVERY)
  // LEDs  
  #define IO_00                             { GPIO_PIN_4 , GPIOH, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_LOW,    GPIO_NO_IRQ, "Led LD2" }
  #define IO_01                             { GPIO_PIN_13, GPIOB, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    GPIO_NO_IRQ, "Led LD3" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 1" }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 2" }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1, } 
  // Blue pushbutton
  // #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  // #define IO_02                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, HW_IO_NORMAL,   HW_INPUT,         IO_02_IRQ, "Blue PushBtn" }

/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    GPIO_NO_IRQ, "GpioOutput" }
*/

  #define IO_NUM                            2

#elif defined(STM32L476NUCLEO)
  // LEDs  
  #define IO_00                             { GPIO_PIN_5 , GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_LOW,    GPIO_NO_IRQ, "UserLed1" }
  #define IO_01                             { GPIO_PIN_12, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "UserLed2" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 1" }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   GPIO_NO_IRQ, "GpioOut 2" }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1, } 
  // Blue pushbutton
  #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_02                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, HW_IO_NORMAL,   HW_INPUT,         IO_02_IRQ, "Blue PushBtn" }

/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    GPIO_NO_IRQ, "GpioOutput" }
*/

  #define IO_NUM                            3

#elif defined(DRAGONFLY476)
  // LEDs  
  #define IO_01                             { GPIO_PIN_2 , GPIOB, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, GPIO_NO_IRQ, "UserLed1" }
  #define IO_02                             { GPIO_PIN_10, GPIOA, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, GPIO_NO_IRQ, "UserLed2" }
  #define IO_03                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, GPIO_NO_IRQ, "UserLed3" }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2,}

  #define IO_NUM                            3

#else
  #error "No valid GPIO configuration in gpio_config.h"
#endif

