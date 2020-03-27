/**
  ******************************************************************************
  * @file    "config/i2c_config.h" 
  * @author  Rainer
  * @brief   Definition of I2C GPIO configurations
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_CONFIG_H
#define __I2C_CONFIG_H

#include "config/config.h"
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/************************************************************************
 * I2C1 
 ***********************************************************************/
#ifdef USE_I2C1
  #define I2CDEV1                                I2C1
    
  /* Definition for I2C1 Pins
   * Alternatives 
   * Alternatives  SCL, SDA     AF#:
   * default       PB6, PB7     AF4
   * ALTN1         PB8, PB9     AF4
   * ALTN2         PG14, PG13   AF4
   */
  #if defined(USE_I2CDEV1_ALTN1)
    /* PB8, PB9 */
    #define I2C1_SCL_PIN                     { GPIO_PIN_8,  GPIOB, GPIO_AF4_I2C1, GPIO_PULLUP } 
    #define I2C1_SDA_PIN                     { GPIO_PIN_9,  GPIOB, GPIO_AF4_I2C1, GPIO_PULLUP } 
  #elif defined(USE_I2CDEV1_ALTN2)
    /* PG14, PG13 */
    #define I2C1_SCL_PIN                     { GPIO_PIN_14, GPIOG, GPIO_AF4_I2C1, GPIO_PULLUP } 
    #define I2C1_SDA_PIN                     { GPIO_PIN_13, GPIOG, GPIO_AF4_I2C1, GPIO_PULLUP } 
  #else
    /* default PB6, PB7 */
    #define I2C1_SCL_PIN                     { GPIO_PIN_6,  GPIOB, GPIO_AF4_I2C1, GPIO_PULLUP } 
    #define I2C1_SDA_PIN                     { GPIO_PIN_7,  GPIOB, GPIO_AF4_I2C1, GPIO_PULLUP } 
  #endif

  /* Definition for I2C1's NVIC, only used, if no DMA for both directions is configured */
  #ifdef  I2C1_USE_IRQ
      #define I2C1_EV_IRQ                    { I2C1_EV_IRQn, I2C_IRQ_PRIO, 0 }
      #define I2C1_ER_IRQ                    { I2C1_ER_IRQn, I2C_IRQ_PRIO, 0 }
      #define I2C1_EV_IRQHandler             I2C1_EV_IRQHandler
      #define I2C1_ER_IRQHandler             I2C1_ER_IRQHandler
  #endif

  #ifdef I2C1_USE_DMA
      /* Definition for I2C1's DMA, only used if DMA useage is configured */
      #define I2C1_TX_DMA                    DMA1_Channel6, DMA_REQUEST_3, DMA1_Channel6_IRQn, DMA_PRIORITY_MEDIUM
      #define I2C1_RX_DMA                    DMA1_Channel7, DMA_REQUEST_3, DMA1_Channel7_IRQn, DMA_PRIORITY_MEDIUM
      #define I2C1_DMA_TX_IRQHandler         DMA1_Channel6_IRQHandler
      #define I2C1_DMA_RX_IRQHandler         DMA1_Channel7_IRQHandler
  #endif
#endif // I2C1

/************************************************************************
 * I2C2 
 ***********************************************************************/
#ifdef USE_I2C2
  #define I2CDEV2                              I2C2
  
  /* Definition for I2C2 Pins
   * Alternatives 
   * Alternatives  SCL, SDA     AF#:
   * default       PB10, PB11   AF4
   * ALTN1         PB13, PB14   AF4
   * ALTN2         PF1, PF0     AF4
   */
  #if defined(USE_I2CDEV2_ALTN1)
    /* PB13, PB14 */
    #define I2C2_SCL_PIN                     { GPIO_PIN_13, GPIOB, GPIO_AF4_I2C2, GPIO_PULLUP } 
    #define I2C2_SDA_PIN                     { GPIO_PIN_14, GPIOB, GPIO_AF4_I2C2, GPIO_PULLUP } 
  #elif defined(USE_I2CDEV2_ALTN2)
    /* PF1, PF0 */
    #define I2C2_SCL_PIN                     { GPIO_PIN_0,  GPIOF, GPIO_AF4_I2C2, GPIO_PULLUP } 
    #define I2C2_SDA_PIN                     { GPIO_PIN_1,  GPIOF, GPIO_AF4_I2C2, GPIO_PULLUP } 
  #else
    /* default PB10, PB11 */
    #define I2C2_SCL_PIN                     { GPIO_PIN_10, GPIOB, GPIO_AF4_I2C2, GPIO_PULLUP } 
    #define I2C2_SDA_PIN                     { GPIO_PIN_11, GPIOB, GPIO_AF4_I2C2, GPIO_PULLUP } 
  #endif

  /* Definition for I2C2's NVIC, only used, if no DMA for both directions is configured */
  #ifdef  I2C2_USE_IRQ
      #define I2C2_EV_IRQ                    { I2C2_EV_IRQn, I2C_IRQ_PRIO, 0 }
      #define I2C2_ER_IRQ                    { I2C2_ER_IRQn, I2C_IRQ_PRIO, 0 }
      #define I2C2_EV_IRQHandler             I2C2_EV_IRQHandler
      #define I2C2_ER_IRQHandler             I2C2_ER_IRQHandler
  #endif

  #ifdef I2C2_USE_DMA
      /* Definition for I2C2's DMA, only used if DMA useage is configured */
      #define I2C2_TX_DMA                    DMA1_Channel4, DMA_REQUEST_3, DMA1_Channel4_IRQn, DMA_PRIORITY_MEDIUM
      #define I2C2_RX_DMA                    DMA1_Channel5, DMA_REQUEST_3, DMA1_Channel5_IRQn, DMA_PRIORITY_MEDIUM
      #define I2C2_DMA_TX_IRQHandler         DMA1_Channel4_IRQHandler
      #define I2C2_DMA_RX_IRQHandler         DMA1_Channel5_IRQHandler
  #endif
#endif // I2C2


/************************************************************************
 * I2C3 
 ***********************************************************************/
#ifdef USE_I2C3
    // todo
#endif // I2CDEV3
#endif /* __SI2C_CONFIG_H */

