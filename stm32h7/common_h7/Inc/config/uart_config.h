/**
  ******************************************************************************
  * @file    "config/system.h" 
  * @author  Rainer
  * @brief   Definition of all devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_CONFIG_H
#define __UART_CONFIG_H

#include "hardware.h"
#include "config/devices_config.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* --- U(S)ARTS --- */ 
// Hinweis  : moegliche Pinbelegungen
//           USART1 : TX:[PA9,PB6] RX:[PA10,PB7]
//           USART2 : TX:[PA2,PD5] RX:[PA3,PD6]
//           USART3 : TX:[PB10,PC3,PC10,PD8] RX:[PB11,PC4,PC11,PD9]
//           UART4  : TX:[PA0,PC10] RX:[PA1,PC11]
//           UART5  : TX:[PC12] RX:[PD2]
//          LPUART1 : TX:[PB11,PC1,PG7] RX:[PB10,PC0,PG8]
   
/************************************************************************
 * Definition for USART1 clock resources
 * USART1 : TX:[PA9,PB6] RX:[PA10,PB7]
 */
#ifdef USE_USART1
  #define COM1                           USART1

  /* Definition for COM1 Pins 
   * Alternatives for USART1 : 
   * ALTN1   TX:PA9   RX:PA10  AF7 
   * ALTN2   TX:PB14  RX:PB15  AF4
   * DEFAULT TX:PB6   RX:PB7   AF7 
   */

  #ifdef USE_USART1_ALTN1
    /* PA9/PA10 with AF7 */
    #define COM1_TX                        { GPIO_PIN_9,  GPIOA, GPIO_AF7_USART1, GPIO_PULLUP }
    #define COM1_RX                        { GPIO_PIN_10, GPIOA, GPIO_AF7_USART1, GPIO_PULLUP }
  #elif defined(USE_USART1_ALTN2)
    /* PB14/PB15 with AF4 */
    #define COM1_TX                        { GPIO_PIN_14, GPIOB, GPIO_AF4_USART1, GPIO_PULLUP }
    #define COM1_RX                        { GPIO_PIN_15, GPIOB, GPIO_AF4_USART1, GPIO_PULLUP }
  #else
    /* PB6/PB7 with AF7 */
    #define COM1_TX                        { GPIO_PIN_6, GPIOB, GPIO_AF7_USART1, GPIO_PULLUP }
    #define COM1_RX                        { GPIO_PIN_7, GPIOB, GPIO_AF7_USART1, GPIO_PULLUP }
  #endif

  /* Definition for COM1's NVIC */
  #if defined(USE_USART1_DEBUG)
      #define COM1_IRQ                         { USART1_IRQn, DEBUG_IRQ_PRIO, 0    }
  #else
      #define COM1_IRQ                         { USART1_IRQn, USART_IRQ_PRIO, 0    }
  #endif
  #define COM1_IRQHandler                  USART1_IRQHandler

  #ifdef COM1_USE_TX_DMA
    /* Definition for USART1 TX DMA */
    #define COM1_TX_DMA                    DMA1_Stream5, DMA_REQUEST_USART1_TX, DMA1_Stream5_IRQn, DMA_PRIORITY_MEDIUM 
    #define COM1_DMA_TX_IRQHandler         DMA1_Stream5_IRQHandler
  #endif

  #ifdef COM1_USE_RX_DMA
    /* Definition for USART1 RX DMA */
    #define COM1_RX_DMA                    DMA1_Stream7, DMA_REQUEST_USART1_RX, DMA1_Stream7_IRQn, DMA_PRIORITY_LOW
    #define COM1_DMA_RX_IRQHandler         DMA1_Stream7_IRQHandler
  #endif
#endif // COM1 

/************************************************************************
 * Definition for USART2 clock resources
 * USART2 : TX:[PA2,PD5] RX:[PA3,PD6]
 */
#ifdef USE_USART2
  #define COM2                           USART2
  
  /* Definition for COM2 Pins
   * Alternatives for USART2 : 
   * ALTN1   TX:PA2  RX:PA3   AF7
   * DEFAULT TX:PD5  RX:PD6   AF7
   */
  #ifdef USE_USART2_ALTN1
    /* PA2/PA3 with AF7 */
    #define COM2_TX                        { GPIO_PIN_2, GPIOA, GPIO_AF7_USART2, GPIO_PULLUP }
    #define COM2_RX                        { GPIO_PIN_3, GPIOA, GPIO_AF7_USART2, GPIO_PULLUP }
  #else
    /* PD5/PD6 with AF7 */
    #define COM2_TX                        { GPIO_PIN_5, GPIOD, GPIO_AF7_USART2, GPIO_PULLUP }
    #define COM2_RX                        { GPIO_PIN_6, GPIOD, GPIO_AF7_USART2, GPIO_PULLUP }
  #endif

  /* Definition for COM2's NVIC */
  #if defined(USE_USART2_DEBUG)
    #define COM2_IRQ                       { USART2_IRQn, DEBUG_IRQ_PRIO, 0    }
  #else
    #define COM2_IRQ                       { USART2_IRQn, USART_IRQ_PRIO, 0    }
  #endif
  #define COM2_IRQHandler                USART2_IRQHandler

  #ifdef COM2_USE_TX_DMA
    /* Definition for USART2 TX DMA */
    #define COM2_TX_DMA                    DMA1_Stream5, DMA_REQUEST_USART2_TX, DMA1_Stream5_IRQn, DMA_PRIORITY_MEDIUM 
    #define COM2_DMA_TX_IRQHandler         DMA1_Stream5_IRQHandler
  #endif
  #ifdef COM2_USE_RX_DMA
    /* Definition for USART2 RX DMA */
    #define COM2_RX_DMA                    DMA1_Stream7, DMA_REQUEST_USART2_RX, DMA1_Stream7_IRQn, DMA_PRIORITY_LOW
    #define COM2_DMA_RX_IRQHandler         DMA1_Stream7_IRQHandler
  #endif  
#endif // COM2

/************************************************************************
 * Definition for USART3 clock resources
 * USART3 : TX:[PB10,PC10,PD8] RX:[PB11,PC11,PD9]

 */
#ifdef USE_USART3
  #define COM3                           USART3
  
  /* Definition for COM3 Pins
   * Alternatives for USART3 : 
   * ALTN1   TX:PB10  RX:PB11    AF7
   * ALTN2   TX:PC10  RX:PC11    AF7
   * DEFAULT TX:PD8  RX:PD9      AF7 
   */
  #if defined(USE_USART3_ALTN1)
    /* PB10/PB11 with AF7 */
    #define COM3_TX                        { GPIO_PIN_10, GPIOB, GPIO_AF7_USART3, GPIO_PULLUP }
    #define COM3_RX                        { GPIO_PIN_11, GPIOB, GPIO_AF7_USART3, GPIO_PULLUP }
  #elif defined(USE_USART3_ALTN2)
    /* PC10/PC11 with AF7 */
    #define COM3_TX                        { GPIO_PIN_10, GPIOC, GPIO_AF7_USART3, GPIO_PULLUP }
    #define COM3_RX                        { GPIO_PIN_11, GPIOC, GPIO_AF7_USART3, GPIO_PULLUP }
  #else
    /* PD8/PD9 with AF7 */
    #define COM3_TX                        { GPIO_PIN_8, GPIOD, GPIO_AF7_USART3, GPIO_PULLUP }
    #define COM3_RX                        { GPIO_PIN_9, GPIOD, GPIO_AF7_USART3, GPIO_PULLUP }
  #endif

  /* Definition for COM3's NVIC */
  #if defined(USE_USART3_DEBUG)
    #define COM3_IRQ                         { USART3_IRQn, DEBUG_IRQ_PRIO, 0    }
  #else
    #define COM3_IRQ                         { USART3_IRQn, USART_IRQ_PRIO, 0    }
  #endif
  #define COM3_IRQHandler                  USART3_IRQHandler

  #ifdef COM3_USE_TX_DMA
    /* Definition for USART3 TX DMA */
    #define COM3_TX_DMA                    DMA1_Stream4, DMA_REQUEST_USART3_TX, DMA1_Stream4_IRQn, DMA_PRIORITY_MEDIUM 
    #define COM3_DMA_TX_IRQHandler         DMA1_Stream4_IRQHandler
  #endif
  #ifdef COM3_USE_RX_DMA
    /* Definition for USART3 RX DMA */
    #define COM3_RX_DMA                    DMA1_Stream6, DMA_REQUEST_USART3_RX, DMA1_Stream6_IRQn, DMA_PRIORITY_LOW
    #define COM3_DMA_RX_IRQHandler         DMA1_Stream6_IRQHandler
  #endif
#endif // COM3


/************************************************************************
 * Definition for UART4 clock resources
 * UART4  : TX:[PA0,PC10] RX:[PA1,PC11]

 */
#ifdef USE_UART4
  #define COM4                           UART4
  
  /* Definition for COM4 Pins
   * Alternatives for UART4 : 
   * TX:[PA0,PC10] RX:[PA1,PC11]
   * ALTN1   TX:PC10 RX:PC11  AF8
   *         TX:PA12 RX:PA11  AF6
   *         TX:PB9  RX:PB8   AF8
   *         TX:PD1  RX:PD0   AF8
   *         TX:PH13 RX:PH14  AF8
   * DEFAULT TX:PA0  RX:PA1   AF8
   */
  #ifdef USE_UART4_ALTN1
    /* ALTN1 TX:PC10 RX:PC11 with AF8  */
    #define COM4_TX                        { GPIO_PIN_10, GPIOC, GPIO_AF8_UART4, GPIO_PULLUP }
    #define COM4_RX                        { GPIO_PIN_11, GPIOC, GPIO_AF8_UART4, GPIO_PULLUP }
  #else
    /* DEFAULT TX:PA0  RX:PA1 with AF8 */
    #define COM4_TX                        { GPIO_PIN_0, GPIOA, GPIO_AF8_UART4, GPIO_PULLUP }
    #define COM4_RX                        { GPIO_PIN_1, GPIOA, GPIO_AF8_UART4, GPIO_PULLUP }
  #endif

   /* RHB work to be done */

#endif // COM4

/************************************************************************
 * Definition for UART5 clock resources
 * UART5 : TX:[PC12] RX:[PD2]
 */
#ifdef USE_UART5
  #define COM5                           UART5
  
  /* Definition for COM5 Pins
   * Alternatives for UART5 : 
   * TX:PC12 RX:PD2  AF8
   * TX:PB6  RX:PB5  AF14
   * TX:PB13  RX:PB12  AF14
   */

  /* Only choice: TX:PC12 RX:PD2 with AF8 */
  #define COM5_TX                        { GPIO_PIN_12, GPIOC, GPIO_AF8_UART5, GPIO_PULLUP }
  #define COM5_RX                        { GPIO_PIN_2,  GPIOD, GPIO_AF8_UART5, GPIO_PULLUP }

   /* RHB work to be done */
  
#endif // COM5

/************************************************************************
 * Definition for LPUART1 clock resources
 * LPUART1 : TX:[PB11,PC1,PG7] RX:[PB10,PC0,PG8]

 */
#ifdef USE_LPUART1
  #define COM9                           LPUART1
 
  /* Definition for COM9 Pins
   * Alternatives for LPUART1 : 
   TX:[PA9, PB6] RX:[PA10, PB7]
   * ALTN1   TX:PA9  RX:PA10   AF3
   * DEFAULT TX:PB6  RX:PB7    AF8 
   */
  #ifdef USE_LPUART1_ALTN1
    /* PA9/PA10 with AF3 */
    #define COM9_TX                        { GPIO_PIN_9,  GPIOA, GPIO_AF3_LPUART, GPIO_PULLUP }
    #define COM9_RX                        { GPIO_PIN_10, GPIOA, GPIO_AF3_LPUART, GPIO_PULLUP }
  #else
    /* DEFAULT TX:PB6  RX:PB7  AF8   */
    #define COM9_TX                        { GPIO_PIN_6, GPIOB, GPIO_AF8_LPUART, GPIO_PULLUP }
    #define COM9_RX                        { GPIO_PIN_7, GPIOB, GPIO_AF8_LPUART, GPIO_PULLUP }
  #endif

  #ifdef COM9_USE_TX_DMA
    /* Definition for LPUART1 TX DMA */
    #define COM9_TX_DMA                     BDMA_Channel6, BDMA_REQUEST_LPUART1_TX,  BDMA_Channel6_IRQn, DMA_PRIORITY_MEDIUM
    #define COM9_DMA_TX_IRQHandler          BDMA_Channel6_IRQHandler
  #endif
  #ifdef COM9_USE_RX_DMA
    /* Definition for LPUART1 RX DMA */
    #define COM9_RX_DMA                     BDMA_Channel7, BDMA_REQUEST_LPUART1_RX,  BDMA_Channel7_IRQn, DMA_PRIORITY_LOW 
    #define COM9_DMA_RX_IRQHandler          BDMA_Channel7_IRQHandler
  #endif

  /* Definition for COM9's NVIC */
  #if defined(USE_LPUART1_DEBUG)
    #define COM9_IRQ                          { LPUART1_IRQn, DEBUG_IRQ_PRIO, 0    }
  #else
    #define COM9_IRQ                          { LPUART1_IRQn, USART_IRQ_PRIO, 0    }
  #endif
  #define COM9_IRQHandler                   LPUART1_IRQHandler

#endif // COM9

/* USART6, UART7, UART8 tbd */

#endif /* __UART_CONFIG_H */

