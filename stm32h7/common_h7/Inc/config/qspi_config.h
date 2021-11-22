/**
  ******************************************************************************
  * @file    "config/qspi_config.h" 
  * @author  Rainer
  * @brief   Definition of QSPI devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QSPI_CONFIG_H
#define __QSPI_CONFIG_H

#include "hardware.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for QSPI resources
 * Function                     |   Altn1                      |  Altn2                       |  Altn3
 *------------------------------+------------------------------+------------------------------+------------------------------+
 * CLK                          : PB2/AF9                      | PF10/AF9                     | PB2/AF9
 *------------------------------+------------------------------+------------------------------+------------------------------+
 * NCS and SIO-Pins for QSPI1   : PB6/AF10                     | PB10/AF9                     | PG6(AF10)
 * SIO0(SI),SIO1(SO),SIO2, SIO3 : PC9, PC10, PE2, PA1  (AF9)   | PD11, PD12, PF7, PD1 (AF9)   | PF8,PF9(AF10), PF7,PF6(AF9)
 *------------------------------+------------------------------+------------------------------+------------------------------+
 * NCS and SIO-Pins for QSPI2   : PC11/AF9                     | PC11/AF9                     | PC11/AF9
 * SIO0(SI),SIO1(SO),SIO2, SIO3 : Pe7, PE8, PE9, PE10 (AF10)   | PH2, PH3, PG9, PG14 (AF9)    | Pe7, PE8, PE9, PE10 (AF10)
 *------------------------------+------------------------------+------------------------------+------------------------------+
 *
 */ 
#ifdef USE_QSPI1
  #if defined(USE_QSPI1_ALTN1)
    /* See above Altn1 */
    #define QSPI1_NCS                       { GPIO_PIN_6,  GPIOB, GPIO_AF10_QUADSPI, GPIO_NOPULL, "QSpi nCS"  }
    #define QSPI1_CLK                       { GPIO_PIN_2,  GPIOB, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi Clk"  }
    #define QSPI1_SI_SIO0                   { GPIO_PIN_9,  GPIOC, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO0" }
    #define QSPI1_SO_SIO1                   { GPIO_PIN_10, GPIOC, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO1" }
    #define QSPI1_SIO2                      { GPIO_PIN_2,  GPIOE, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO2" }
    #define QSPI1_SIO3                      { GPIO_PIN_1,  GPIOA, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO3" }
  #elif defined(USE_QSPI1_ALTN2)
    /* See above Altn2 */
    #define QSPI1_NCS                       { GPIO_PIN_10, GPIOB, GPIO_AF9_QUADSPI,  GPIO_PULLUP, "QSpi nCS"  }
    #define QSPI1_CLK                       { GPIO_PIN_10, GPIOF, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi Clk"  }
    #define QSPI1_SI_SIO0                   { GPIO_PIN_11, GPIOD, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO0" }
    #define QSPI1_SO_SIO1                   { GPIO_PIN_12, GPIOD, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO1" }
    #define QSPI1_SIO2                      { GPIO_PIN_7,  GPIOF, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO2" }
    #define QSPI1_SIO3                      { GPIO_PIN_13, GPIOD, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO3" }
  #elif defined(USE_QSPI1_ALTN3)
    /* See above Altn1 */
    #define QSPI1_NCS                       { GPIO_PIN_6,  GPIOG, GPIO_AF10_QUADSPI, GPIO_NOPULL, "QSpi nCS"  }
    #define QSPI1_CLK                       { GPIO_PIN_2,  GPIOB, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi Clk"  }
    #define QSPI1_SI_SIO0                   { GPIO_PIN_8,  GPIOF, GPIO_AF10_QUADSPI, GPIO_NOPULL, "QSpi SIO0" }
    #define QSPI1_SO_SIO1                   { GPIO_PIN_9,  GPIOF, GPIO_AF10_QUADSPI, GPIO_NOPULL, "QSpi SIO1" }
    #define QSPI1_SIO2                      { GPIO_PIN_7,  GPIOF, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO2" }
    #define QSPI1_SIO3                      { GPIO_PIN_6,  GPIOF, GPIO_AF9_QUADSPI,  GPIO_NOPULL, "QSpi SIO3" }
  #else
    #error "No Pin definition for QSP1"
  #endif

  /* Definition for COM1's NVIC */
  #if defined(QSPI1_USE_IRQ)
     #define QSPI1_IRQ                        { QUADSPI_IRQn, SPI_IRQ_PRIO, 0    }
     #define QSPI1_IRQHandler                  QUADSPI_IRQHandler
  #endif

  #ifdef QSPI1_USE_DMA
    /* Definition for USART1 TX DMA */
    /* NO QSPI DMA in STM32H7 */
    //#define QSPI1_DMA                      NULL, HW_DMA_STREAM, DMA_REQUEST_, DMA_PRIORITY_VERY_HIGH 
//    #define QSPI1_DMA_IRQHandler           DMA2_Stream7_IRQHandler
//    #define QSPI1_DMA                      DMA1_Channel5, DMA_REQUEST_5, DMA1_Channel5_IRQn, DMA_PRIORITY_VERY_HIGH 
//    #define QSPI1_DMA_IRQHandler           DMA1_Channel5_IRQHandler
  #endif

  /* Set a default QSPI clock if not yet set */
  #if !defined(QSPI1_CLKSPEED)
      #define  QSPI1_CLKSPEED                10000000
  #endif

#endif // USE_QSPI1 

#endif /* __QSPI_CONFIG_H */

