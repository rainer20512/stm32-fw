/**
  ******************************************************************************
  * @file    "config/ospi_config.h" 
  * @author  Rainer
  * @brief   Definition of OSPI devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  * Although there is any specific config file, there is only on driver xspi_dev.c
  * which handles both QUADSPI and OCTOSPI devices
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OSPI_CONFIG_H
#define __OSPI_CONFIG_H

#include "hardware.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for OSPI-Manager Ressources
 ************************************************************************/
#if USE_OSPI1 > 0
  #if defined(USE_OSPI1_ALTN1)
    #define OSPI1_NCS                       { GPIO_PIN_11, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 nCS"  }
    #define OSPI1_CLK                       { GPIO_PIN_10, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 Clk"  }
    #define OSPI1_DQS                       { GPIO_PIN_2,  GPIOB, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 Dqs"  }
    #define OSPI1_SI_SIO0                   { GPIO_PIN_12, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO0" }
    #define OSPI1_SO_SIO1                   { GPIO_PIN_13, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO1" }
    #define OSPI1_SIO2                      { GPIO_PIN_14, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO2" }
    #define OSPI1_SIO3                      { GPIO_PIN_15, GPIOE, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO3" }
    #define OSPI1_SIO4                      { GPIO_PIN_8,  GPIOF, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO4" }
    #define OSPI1_SIO5                      { GPIO_PIN_9,  GPIOF, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO5" }
    #define OSPI1_SIO6                      { GPIO_PIN_7,  GPIOF, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO6" }
    #define OSPI1_SIO7                      { GPIO_PIN_6,  GPIOF, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO7" }
  #elif defined(USE_OSPI1_ALTN2)
    #define OSPI1_NCS                       { GPIO_PIN_2,  GPIOA, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 nCS"  }
    #define OSPI1_CLK                       { GPIO_PIN_3,  GPIOA, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 Clk"  }
    #define OSPI1_DQS                       { GPIO_PIN_1,  GPIOA, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 Dqs"  }
    #define OSPI1_SI_SIO0                   { GPIO_PIN_1,  GPIOB, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO0" }
    #define OSPI1_SO_SIO1                   { GPIO_PIN_0,  GPIOB, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO1" }
    #define OSPI1_SIO2                      { GPIO_PIN_7,  GPIOA, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO2" }
    #define OSPI1_SIO3                      { GPIO_PIN_6,  GPIOA, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO3" } 
    #define OSPI1_SIO4                      { GPIO_PIN_4,  GPIOD, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO4" }
    #define OSPI1_SIO5                      { GPIO_PIN_5,  GPIOD, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO5" }
    #define OSPI1_SIO6                      { GPIO_PIN_6,  GPIOD, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO6" }
    #define OSPI1_SIO7                      { GPIO_PIN_7,  GPIOD, GPIO_AF10_OCTOSPIM_P1, GPIO_NOPULL, "OSpi1 SIO7" }
  #else
    #error "No Pin definition for OSPI1"
  #endif

  /* Definition for OCTOSPI1 NVIC */
  #if defined(OSPI1_USE_IRQ)
     #define OSPI1_IRQ                        { OCTOSPI1_IRQn, SPI_IRQ_PRIO, 0    }
     #define OSPI1_IRQHandler                 OCTOSPI1_IRQHandler
  #endif

  #ifdef OSPI1_USE_DMA
    /* Definition for OSPI1 DMA */
    #define OSPI1_DMA                      DMA2_Channel4, DMA_REQUEST_OCTOSPI1, DMA2_Channel4_IRQn, DMA_PRIORITY_VERY_HIGH 
    #define OSPI1_DMA_IRQHandler           DMA2_Channel4_IRQHandler
//    #define OSPI1_DMA                      DMA1_Channel5, DMA_REQUEST_OCTOSPI1, DMA1_Channel5_IRQn, DMA_PRIORITY_VERY_HIGH 
//    #define OSPI1_DMA_IRQHandler           DMA1_Channel5_IRQHandler
  #endif

  /* Set a default OCTOSPI1 clock if not yet set */
  #if !defined(OSPI1_CLKSPEED)
      #define OSPI1_CLKSPEED                10000000
  #endif

#endif // USE_OSPI1 

#if USE_OSPI2 > 0
  #if defined(USE_OSPI2_ALTN1)
    #define OSPI2_NCS                       { GPIO_PIN_3,  GPIOD, GPIO_AF10_OCTOSPIM_P2, GPIO_NOPULL, "OSpi2 nCS"  }
    #define OSPI2_CLK                       { GPIO_PIN_4,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 Clk"  }
    #define OSPI2_DQS                       { GPIO_PIN_12, GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 Dqs"  }
    #define OSPI2_SI_SIO0                   { GPIO_PIN_0,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO0" }
    #define OSPI2_SO_SIO1                   { GPIO_PIN_1,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO1" }
    #define OSPI2_SIO2                      { GPIO_PIN_2,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO2" }
    #define OSPI2_SIO3                      { GPIO_PIN_3,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO3" }
    #define OSPI2_SIO4                      { GPIO_PIN_0,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO4" }
    #define OSPI2_SIO5                      { GPIO_PIN_1,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO5" }
    #define OSPI2_SIO6                      { GPIO_PIN_9,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO6" }
    #define OSPI2_SIO7                      { GPIO_PIN_10, GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO7" }
  #elif defined(USE_OSPI2_ALTN2)
    /* PA9/PA10 with AF7 */
    #define OSPI2_NCS                       { GPIO_PIN_12, GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 nCS"  }
    #define OSPI2_CLK                       { GPIO_PIN_4,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 Clk"  }
    #define OSPI2_DQS                       { GPIO_PIN_15, GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 Dqs"  }
    #define OSPI2_SI_SIO0                   { GPIO_PIN_0,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO0" }
    #define OSPI2_SO_SIO1                   { GPIO_PIN_1,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO1" }
    #define OSPI2_SIO2                      { GPIO_PIN_2,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO2" }
    #define OSPI2_SIO3                      { GPIO_PIN_3,  GPIOF, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO3" }
    #define OSPI2_SIO4                      { GPIO_PIN_0,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO4" }
    #define OSPI2_SIO5                      { GPIO_PIN_1,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO5" }
    #define OSPI2_SIO6                      { GPIO_PIN_9,  GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO6" }
    #define OSPI2_SIO7                      { GPIO_PIN_10, GPIOG, GPIO_AF5_OCTOSPIM_P2,  GPIO_NOPULL, "OSpi2 SIO7" }
  #else
    #error "No Pin definition for OSPI2"
  #endif

  /* Definition for OCTOSPI2's NVIC */
  #if defined(OSPI2_USE_IRQ)
     #define OSPI2_IRQ                        { OCTOSPI2_IRQn, SPI_IRQ_PRIO, 0    }
     #define OSPI2_IRQHandler                 OCTOSPI2_IRQHandler
  #endif

  #ifdef OSPI2_USE_DMA
    /* Definition for OSPI2 DMA */
    #define OSPI2_DMA                      DMA2_Channel4, DMA_REQUEST_OCTOSPI2, DMA2_Channel4_IRQn, DMA_PRIORITY_VERY_HIGH 
    #define OSPI2_DMA_IRQHandler           DMA2_Channel4_IRQHandler
//    #define OSPI2_DMA                      DMA1_Channel5, DMA_REQUEST_OCTOSPI2, DMA1_Channel5_IRQn, DMA_PRIORITY_VERY_HIGH 
//    #define OSPI2_DMA_IRQHandler           DMA1_Channel5_IRQHandler
  #endif

  /* Set a default QSPI clock if not yet set */
  #if !defined(OSPI2_CLKSPEED)
      #define OSPI2_CLKSPEED                10000000
  #endif

#endif // USE_OSPI2 

#endif /* __OSPI_CONFIG_H */

