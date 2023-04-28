/*
 *  GPIO and Interrupt definitions for USB
 */

#pragma once 

#include "config/config.h"
#include "hardware.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/************************************************************************
 * USB 
 ***********************************************************************/
#if defined(USE_USB)
    
#if   defined(STM32H747xx) || defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
  /* Definition for USB OTG Pins
   * Alternatives  ID,   DM,  DP,     AF#:
   * default       PA10, PA11,PA12    AF10
   * VBUS ( PA9 ) is a normal ( no interrupt ) input pin
   */
  #define USB_ID_PIN                     { GPIO_PIN_10,  GPIOA, GPIO_AF10_OTG2_FS, GPIO_PULLUP, "Usb2Fs Id" } 
  #define USB_DM_PIN                     { GPIO_PIN_11,  GPIOA, GPIO_AF10_OTG2_FS, GPIO_NOPULL, "Usb2Fs Dm" } 
  #define USB_DP_PIN                     { GPIO_PIN_12,  GPIOA, GPIO_AF10_OTG2_FS, GPIO_NOPULL, "Usb2Fs Dp" } 

  #define USB_VBUS                       { GPIO_PIN_9,   GPIOA, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL, HW_INPUT, GPIO_NO_IRQ, "Usb2 Vbus" }  

  /* Definition for USB Interrupts */
  #define USB_FS_IRQ                    { OTG_FS_IRQn,  USB_IRQ_PRIO, 0 }
#elif defined(STM32H723xx) || defined(STM32H733xx) || defined(STM32H725xx) || defined(STM32H735xx) || defined(STM32H730xx)
  /* Definition for USB OTG Pins
   * Alternatives  ID,   DM,  DP,     AF#:
   * default       PA10, PA11,PA12    AF10
   * VBUS ( PA9 ) is a normal ( no interrupt ) input pin
   */
  #define USB_ID_PIN                     { GPIO_PIN_10,  GPIOA, GPIO_AF10_OTG1_FS, GPIO_PULLUP, "Usb2Fs Id" } 
  #define USB_DM_PIN                     { GPIO_PIN_11,  GPIOA, GPIO_AF10_OTG1_FS, GPIO_NOPULL, "Usb2Fs Dm" } 
  #define USB_DP_PIN                     { GPIO_PIN_12,  GPIOA, GPIO_AF10_OTG1_FS, GPIO_NOPULL, "Usb2Fs Dp" } 

  #define USB_VBUS                       { GPIO_PIN_9,   GPIOA, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, HW_IO_NORMAL, HW_INPUT, GPIO_NO_IRQ, "Usb2 Vbus" }

  /* Definition for USB Interrupts */
  #define USB_FS_IRQ                    { OTG_HS_IRQn,  USB_IRQ_PRIO, 0 }
#else
    #error "No USD Pin deinitions"
#endif
#endif // USE_USB > 0



