/**
  ******************************************************************************
  * @file    gpio_config.h
  * @author  Rainer
  * @brief   global definitions and configuration of all GPIO-Pins, that are 
  *          directly set or read by ODR, IDR or BSRR registers,
  *          
  *          The mandatory sequence is
  *          - first all user-LEDs
  *          - thereafter all other GPIO outputs and inputs
  *
  * @note    This is common to CM7 and CM4. In a multicore environment both
  *          cores may access these pins
  * 
  ******************************************************************************
  */


#pragma once 

#include "config/config.h"
#include "config/devices_config.h"
#include "hardware.h"


#define IO_NO_IRQ                            {-1,0,}

#if defined(STM32H745NUCLEO)
  // LEDs  
  #define IO_00                             { GPIO_PIN_0,  GPIOB, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,   IO_NO_IRQ, "UserLed gn" }
  #define IO_01                             { GPIO_PIN_1,  GPIOE, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,   IO_NO_IRQ, "UserLed ye" }
  #define IO_02                             { GPIO_PIN_14, GPIOB, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,   IO_NO_IRQ, "UserLed rd" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2} 
  // Blue pushbutton, assigned only ot CM7 core
#if defined(CORE_CM7)
  #define IO_03_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_03                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_03_IRQ, "Blue PushBtn" }
  #define IO_NUM                            4
#else
  #define IO_NUM                            3
#endif
/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/
#elif defined(STM32H747IDISCO)
  // LEDs  
  #define IO_00                             { GPIO_PIN_12, GPIOI, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed gr" }
  #define IO_01                             { GPIO_PIN_13, GPIOI, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed or" }
  #define IO_02                             { GPIO_PIN_14, GPIOI, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed rd" }
  #define IO_03                             { GPIO_PIN_15, GPIOI, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed bl" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        4
  #define USERLEDS                          { 0, 1, 2, 3} 
  // Blue pushbutton, assigned only ot CM7 core
#if defined(CORE_CM7)
  #define IO_04_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_04                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_04_IRQ, "Blue PushBtn" }
  #define IO_NUM                            5
#else
  #define IO_NUM                            4
#endif
#elif defined(PORTENTAH7)
  // LEDs  
  #define IO_00                             { GPIO_PIN_12, GPIOI, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed rd" }
  #define IO_01                             { GPIO_PIN_13, GPIOJ, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed gr" }
  #define IO_02                             { GPIO_PIN_3,  GPIOE, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed bl" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2 } 
  #define IO_NUM                            3
#elif defined(ARDUINO_GIGA_R1)
  // LEDs  
  #define IO_00                             { GPIO_PIN_5, GPIOK, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed rd" }
  #define IO_01                             { GPIO_PIN_6, GPIOK, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed gr" }
  #define IO_02                             { GPIO_PIN_7, GPIOK, GPIO_MODE_OUTPUT_PP,  GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ, "UserLed bl" }
//  #define IO_01                             { GPIO_PIN_10, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
//  #define IO_02                             { GPIO_PIN_11, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH,   IO_NO_IRQ }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2 } 
  #define IO_NUM                            3
#elif defined(STM32H7_DEVEBOX)
  // LEDs  
  #define IO_00                             { GPIO_PIN_1,  GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed gn" }
  #define USERLEDNUM                        1
  #define USERLEDS                          { 0, } 

  // Buttons K1 and K2
  #define IO_01_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_01                             { GPIO_PIN_3, GPIOE, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP, HW_IO_NORMAL, HW_INPUT,         IO_01_IRQ, "K1 PushBtn" }
  #define IO_02_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_02                             { GPIO_PIN_5, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP, HW_IO_NORMAL, HW_INPUT,         IO_02_IRQ, "K2 PushBtn" }
  #define IO_NUM                            3
/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

#elif defined(STM32H7_OPENMV)
  // LEDs  
  #define IO_00                             { GPIO_PIN_0,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "Led rd" }
  #define IO_01                             { GPIO_PIN_1,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "Led gn" }
  #define IO_02                             { GPIO_PIN_2,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "Led bl" }
  #define USERLEDNUM                        3
  #define USERLEDS                          { 0, 1, 2, } 

  #define IO_03_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_03                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_03_IRQ, "Tamper" }
  #define IO_NUM                            4


#elif defined(STM32H742REF)
  // LEDs  
  #define IO_00                             { GPIO_PIN_12, GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed gn" }
  #define IO_01                             { GPIO_PIN_5 , GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed ye" }
  #define IO_02                             { GPIO_PIN_1 , GPIOB, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed rd" }
  #define IO_03                             { GPIO_PIN_0 , GPIOB, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed bl" }
  #define USERLEDNUM                        4
  #define USERLEDS                          { 0, 1, 2, 3} 

  // Blue pushbutton
  #define IO_05_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_05                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_05_IRQ, "Blue PushBtn" }
  #define IO_NUM                            5
/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

#elif defined(STM32H743EVAL2)
  // LEDs  
  #define IO_00                             { GPIO_PIN_10,  GPIOF, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed gn" }
  #define IO_01                             { GPIO_PIN_4 ,  GPIOA, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed rd" }
//  #define IO_02                             { GPIO_PIN_5 , GPIOK, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed rd" }
//  #define IO_03                             { GPIO_PIN_6 , GPIOK, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed bl" }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1, } 

  // Blue pushbutton
  #define IO_05_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_05                             { GPIO_PIN_13, GPIOC, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_05_IRQ, "Blue PushBtn" }
  #define IO_NUM                            3
/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

#elif defined(STM32H725_WIOLITEAI)
  // LEDs  
  #define IO_00                             { GPIO_PIN_13,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL, HW_OUTPUT_LOW, IO_NO_IRQ, "UserLed rd" }
  #define IO_01                             { GPIO_PIN_0 ,  GPIOF, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL, HW_OUTPUT_LOW, IO_NO_IRQ, "UserLed ye" }
//  #define IO_02                             { GPIO_PIN_5 , GPIOK, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed rd" }
//  #define IO_03                             { GPIO_PIN_6 , GPIOK, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_INVERTED, HW_OUTPUT_HIGH, IO_NO_IRQ, "UserLed bl" }
  #define USERLEDNUM                        2
  #define USERLEDS                          { 0, 1, } 

  // Blue pushbutton
  #define IO_05_IRQ                         { BUTTON_IRQ_PRIO, 0 }
  #define IO_05                             { GPIO_PIN_1, GPIOF, GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN, HW_IO_NORMAL, HW_INPUT,         IO_05_IRQ, "Right PushBtn" }
  #define IO_NUM                            3
/* Example entry for additional IO-Pins
  #define IO_03                             { GPIO_PIN_8,  GPIOC, GPIO_MODE_OUTPUT_PP,         GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL,   HW_OUTPUT_LOW,    IO_NO_IRQ }
*/

#else
  #error "No valid device configuration in devices_config.h"
#endif

