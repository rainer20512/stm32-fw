/**
  ******************************************************************************
  * @file    portenta_h7.h
  * @author  Rainer
  * @brief   minimal BSP for Portenta H7 board, mainly for debug 
  *
  *                      ---
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PORTENTA_H7_H
#define __PORTENTA_H7_H

/* define the BOARD LEDS */
#define LED1    0
#define LED2    1
#define LED3    2
#define MAX_LED 3

/* Aliases for the LEDs */
#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE   2

/* PMIC Operation modes, will be controlled by PJ0 GPIO */
typedef enum {
    PMIC_RUN    = 0,                     
    PMIC_STBY   = 1,
} PMIC_RunmodeT;

void PMIC_SetMode           ( PMIC_RunmodeT mode );

void BSP_Board_Init         (void);
void BSP_LAN7842_Bootstrap  (void);
void BSP_LED_Init           (uint32_t lednum);
void BSP_LED_Toggle         (uint32_t lednum);
void BSP_LED_On             (uint32_t lednum);
void BSP_LED_Off            (uint32_t lednum);


#define BSP_PinInit(a)      BSP_LED_Init(a)
#define BSP_PinToggle(a)    BSP_LED_Toggle(a)
void BSP_PinHigh            (uint32_t lednum);
void BSP_PinLow             (uint32_t lednum);


#endif  //__PORTENTA_H7_H 