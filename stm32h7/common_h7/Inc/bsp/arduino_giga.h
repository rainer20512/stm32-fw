/**
  ******************************************************************************
  * @file    arduino_giga.h
  * @author  Rainer
  * @brief   minimal BSP for Arduino Giga board, mainly for debug 
  *
  *                      ---
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ARDUINO_GIGA_H
#define __ARDUINO_GIGA_H

/* define the BOARD LEDS */
#define LED1    0
#define LED2    1
#define LED3    2
#define MAX_LED 3

/* Aliases for the LEDs */
#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE   2

void BSP_Board_Init         (void);
void BSP_LED_Init           (uint32_t lednum);
void BSP_LED_Toggle         (uint32_t lednum);
void BSP_LED_On             (uint32_t lednum);
void BSP_LED_Off            (uint32_t lednum);


#define BSP_PinInit(a)      BSP_LED_Init(a)
#define BSP_PinToggle(a)    BSP_LED_Toggle(a)
void BSP_PinHigh            (uint32_t lednum);
void BSP_PinLow             (uint32_t lednum);


#endif  // __ARDUINO_GIGA_H 