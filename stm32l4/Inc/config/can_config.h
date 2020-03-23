/*
 *  GPIO and Interrupt definitions for CAN
 */

#pragma once 

#include "config/config.h"
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/************************************************************************
 * CAN 
 ***********************************************************************/
#if defined(USE_CAN1)
    
  /* Definition for CAN Pins
   * Alternatives 
   * Alternatives  RX,  TX,     AF#:
   * default       PA11,PA12    AF9
   * ALTN1         PB8, PB9     AF9
   * ALTN2         PD0, PD1     AF9
   */
  #if defined(USE_CANDEV1_ALTN1)
    /* ALTN1 PB8, PB9 */
    #define CAN1_RX_PIN                     { GPIO_PIN_8,   GPIOB, GPIO_AF9_CAN1, GPIO_PULLUP } 
    #define CAN1_TX_PIN                     { GPIO_PIN_9,   GPIOB, GPIO_AF9_CAN1, GPIO_PULLUP } 
  #elif defined(USE_CANDEV1_ALTN2)
    /* ALTN2 PD0, PD1 */
    #define CAN1_RX_PIN                     { GPIO_PIN_0,   GPIOD, GPIO_AF9_CAN1, GPIO_PULLUP } 
    #define CAN1_TX_PIN                     { GPIO_PIN_1,   GPIOD, GPIO_AF9_CAN1, GPIO_PULLUP } 
  #else
    /* default PA11, PA12 */
    #define CAN1_RX_PIN                     { GPIO_PIN_11,  GPIOA, GPIO_AF9_CAN1, GPIO_PULLUP } 
    #define CAN1_TX_PIN                     { GPIO_PIN_12,  GPIOA, GPIO_AF9_CAN1, GPIO_PULLUP } 
  #endif

  /* Definition for CAN Interrupts */
  #ifdef  CAN1_USE_IRQ
      #define CAN1_TX_IRQ                   { CAN1_TX_IRQn,  CAN_IRQ_PRIO, 0 }
      #define CAN1_RX0_IRQ                  { CAN1_RX0_IRQn, CAN_IRQ_PRIO, 0 }
      #define CAN1_RX1_IRQ                  { CAN1_RX1_IRQn, CAN_IRQ_PRIO, 0 }
      #define CAN1_SCE_IRQ                  { CAN1_SCE_IRQn, CAN_IRQ_PRIO, 0 }
  #endif
/******************************************
 Interrupt routine names are
           CAN1_TX_IRQHandler
           CAN1_RX0_IRQHandler
           CAN1_RX1_IRQHandler
           CAN1_SCE_IRQHandler
 *****************************************/
#endif // USE_CAN1 > 0



