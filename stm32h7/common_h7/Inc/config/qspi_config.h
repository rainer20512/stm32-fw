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

#include "stm32l4xx_hal.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for QSPI1 resources
 * Function                     |   Altn1                      |  Altn2
 *------------------------------+------------------------------+------------------------------+
 * NCS, CLK                     : PB11, PB10(AF10)             | PE11, PE10 (AF10)
 * SIO0(SI),SIO1(SO),SIO2, SIO3 : PB1,  PB0, PA7,  PA6  (AF10) | PE12, PE13, PE14, PE15 (AF10)
 *------------------------------+------------------------------+------------------------------+
 *
 */
#ifdef USE_QSPI1
  #if defined(USE_QSPI1_ALTN1)
    /* PA9/PA10 with AF7 */
    #define QSPI1_NCS                       { GPIO_PIN_11, GPIOB, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_CLK                       { GPIO_PIN_10, GPIOB, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SI_SIO0                   { GPIO_PIN_1,  GPIOB, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SO_SIO1                   { GPIO_PIN_0,  GPIOB, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SIO2                      { GPIO_PIN_7,  GPIOA, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SIO3                      { GPIO_PIN_6,  GPIOA, GPIO_AF10_QUADSPI, GPIO_NOPULL }
  #elif defined(USE_QSPI1_ALTN2)
    /* PA9/PA10 with AF7 */
    #define QSPI1_NCS                       { GPIO_PIN_11, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_CLK                       { GPIO_PIN_10, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SI_SIO0                   { GPIO_PIN_12, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SO_SIO1                   { GPIO_PIN_13, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SIO2                      { GPIO_PIN_14, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL }
    #define QSPI1_SIO3                      { GPIO_PIN_15, GPIOE, GPIO_AF10_QUADSPI, GPIO_NOPULL } 
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
    #define QSPI1_DMA                      DMA2_Channel7, DMA_REQUEST_3, DMA2_Channel7_IRQn 
    #define QSPI1_DMA_IRQHandler           DMA2_Channel7_IRQHandler
//    #define QSPI1_DMA                      DMA1_Channel5, DMA_REQUEST_5, DMA1_Channel5_IRQn 
//    #define QSPI1_DMA_IRQHandler           DMA1_Channel5_IRQHandler
  #endif

#endif // USE_QSPI1 

#endif /* __QSPI_CONFIG_H */

