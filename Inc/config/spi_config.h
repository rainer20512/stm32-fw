/**
  ******************************************************************************
  * @file    spi_config.h
  * @author  Rainer
  * @brief   configuration of HW and BB SPI devices
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_CONFIG_H
#define __SPI_CONFIG_H

#include "config/devices_config.h"
#include "stm32l4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************************/
/* HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWS*/
/* HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWS*/
/* HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWS*/
/* HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWSPI HWS*/
/******************************************************************************************/

/* Notice: Using HW-NSS is pain in the ass 
 * You have to assure to Disable SPI manually after transfer to ensure NSS going high again 
 * So it's strongly recommended to use Software NSS
 */

/* ----- Fixed part, don't change -------------------------------------------------- */
#if defined(USE_SPI1) && defined(SPI1)
    #if defined(USE_SPI1_ALTN1)
        /* NSS: PA4/AF0, SCK: PA5/AF0, MSIO: PA6/AF0, MOSI:  PA7/AF0 */
        #define SPI1_NSS                         { GPIO_PIN_4,  GPIOA, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_SCK                         { GPIO_PIN_5,  GPIOA, GPIO_AF5_SPI1, GPIO_NOPULL }
        #define SPI1_MISO                        { GPIO_PIN_6,  GPIOA, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_MISO_IRQ                    { EXTI9_5_IRQn, SPI_IRQ_PRIO, 0 }
        #define SPI1_MOSI                        { GPIO_PIN_7,  GPIOA, GPIO_AF5_SPI1, GPIO_NOPULL } 
    #elif defined(USE_SPI1_ALTN2)
        /* NSS: PA15/AF0, SCK: PB3/AF0, MSIO: PB4/AF0, MOSI:  PB5/AF0 */
        #define SPI1_NSS                         { GPIO_PIN_15, GPIOA, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_SCK                         { GPIO_PIN_3,  GPIOB, GPIO_AF5_SPI1, GPIO_NOPULL }
        #define SPI1_MISO                        { GPIO_PIN_4,  GPIOB, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_MISO_IRQ                    { EXTI4_IRQn,  SPI_IRQ_PRIO, 0 }
        #define SPI1_MOSI                        { GPIO_PIN_5,  GPIOB, GPIO_AF5_SPI1, GPIO_NOPULL } 
    #elif defined(USE_SPI1_ALTN3)
        /* NSS: PE12/AF2, SCK: PE13/AF2, MSIO: PE14/AF2, MOSI:  PE15/AF2 */
        #define SPI1_NSS                         { GPIO_PIN_12, GPIOE, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_SCK                         { GPIO_PIN_13, GPIOE, GPIO_AF5_SPI1, GPIO_NOPULL }
        #define SPI1_MISO                        { GPIO_PIN_14, GPIOE, GPIO_AF5_SPI1, GPIO_PULLUP }
        #define SPI1_MISO_IRQ                    { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
        #define SPI1_MOSI                        { GPIO_PIN_15, GPIOE, GPIO_AF5_SPI1, GPIO_NOPULL } 
    #else
        #error("No config for SPI1 found");
    #endif 
    /* Definition for SPIx's NVIC */
    #define SPI1_HW_IRQ                          { SPI1_IRQn, SPI_IRQ_PRIO, 0 }
    #define SPI1_IRQHandler                      SPI1_IRQHandler
    #define SPI1_TX_DMA                          DMA1_Channel3, DMA_REQUEST_1, DMA1_Channel3_IRQn
    #define SPI1_RX_DMA                          DMA1_Channel2, DMA_REQUEST_1, DMA1_Channel2_IRQn
    #define SPI1_DMA_TX_IRQHandler               DMA1_Channel3_IRQHandler
    #define SPI1_DMA_RX_IRQHandler               DMA1_Channel2_IRQHandler
#endif /* SPI1 */

#if defined(USE_SPI2) && defined(SPI2)
    #if defined(USE_SPI2_ALTN1)
        /* NSS: PB12/AF0, SCK: PB13/AF0, MSIO: PB14/AF0, MOSI:  PB15/AF0 */
        #define SPI2_NSS                         { GPIO_PIN_12, GPIOB, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_SCK                         { GPIO_PIN_13, GPIOB, GPIO_AF5_SPI2, GPIO_NOPULL }
        #define SPI2_MISO                        { GPIO_PIN_14, GPIOB, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_MISO_IRQ                    { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
        #define SPI2_MOSI                        { GPIO_PIN_15, GPIOB, GPIO_AF5_SPI2, GPIO_NOPULL } 
    #elif defined(USE_SPI2_ALTN2)
        /* NSS: PB9/AF5, SCK: PB10/AF5, MSIO: PC2/AF2, MOSI:  PC3/AF2 */
        #define SPI2_NSS                         { GPIO_PIN_9,  GPIOB, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_SCK                         { GPIO_PIN_10, GPIOB, GPIO_AF5_SPI2, GPIO_NOPULL }
        #define SPI2_MISO                        { GPIO_PIN_2,  GPIOC, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_MISO_IRQ                    { EXTI2_IRQn,  SPI_IRQ_PRIO, 0 }
        #define SPI2_MOSI                        { GPIO_PIN_3,  GPIOC, GPIO_AF5_SPI2, GPIO_NOPULL } 
    #elif defined(USE_SPI2_ALTN3)
        /* NSS: PD0/AF1, SCK: PD1/AF1, MSIO: PD3/AF2, MOSI:  PD4/AF1 */
        #define SPI2_NSS                         { GPIO_PIN_0,  GPIOD, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_SCK                         { GPIO_PIN_1,  GPIOD, GPIO_AF5_SPI2, GPIO_NOPULL }
        #define SPI2_MISO                        { GPIO_PIN_3,  GPIOD, GPIO_AF5_SPI2, GPIO_PULLUP }
        #define SPI2_MISO_IRQ                    { EXTI3_IRQn,  SPI_IRQ_PRIO, 0 }
        #define SPI2_MOSI                        { GPIO_PIN_4,  GPIOD, GPIO_AF5_SPI2, GPIO_NOPULL } 
    #else
        #error("No config for SPI2 found");
    #endif 
     /* Definition for SPIx's NVIC */
    #define SPI2_HW_IRQ                          { SPI2_IRQn, SPI_IRQ_PRIO, 0 }
    #define SPI2_IRQHandler                      SPI2_IRQHandler
    /* Definition for SPIDEV2's DMA */
    #define SPI2_TX_DMA                          DMA1_Channel5, DMA_REQUEST_1, DMA1_Channel5_IRQn
    #define SPI2_RX_DMA                          DMA1_Channel4, DMA_REQUEST_1, DMA1_Channel4_IRQn
    #define SPI2_DMA_TX_IRQHandler               DMA1_Channel5_IRQHandler
    #define SPI2_DMA_RX_IRQHandler               DMA1_Channel4_IRQHandler
#endif /* SPI2 */

#if defined(USE_SPI3) && defined(SPI3)
    #if defined(USE_SPI3_ALTN1)
        /* NSS: PA4/AF6, SCK: PB3/AF6, MSIO: PB4/AF6, MOSI:  PB5/AF6 */
        #define SPI3_NSS                         { GPIO_PIN_4,  GPIOA, GPIO_AF6_SPI3, GPIO_PULLUP }
        #define SPI3_SCK                         { GPIO_PIN_3,  GPIOB, GPIO_AF6_SPI3, GPIO_NOPULL }
        #define SPI3_MISO                        { GPIO_PIN_4,  GPIOB, GPIO_AF6_SPI3, GPIO_PULLUP }
        #define SPI3_MISO_IRQ                    { EXTI4_IRQn,  SPI_IRQ_PRIO, 0 }
        #define SPI3_MOSI                        { GPIO_PIN_5,  GPIOB, GPIO_AF6_SPI3, GPIO_NOPULL } 
    #elif defined(USE_SPI3_ALTN2)
        /* NSS: PA15/AF6, SCK: PC10/AF6, MSIO: PC11/AF6, MOSI:  PC12/AF6 */
        #define SPIDEV1_NSS                      { GPIO_PIN_15, GPIOA, GPIO_AF6_SPI3, GPIO_PULLUP }
        #define SPI3_SCK                         { GPIO_PIN_10, GPIOC, GPIO_AF5_SPI3, GPIO_NOPULL }
        #define SPI3_MISO                        { GPIO_PIN_11, GPIOC, GPIO_AF5_SPI3, GPIO_PULLUP }
        #define SPI3_MISO_IRQ                    { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
        #define SPI3_MOSI                        { GPIO_PIN_12, GPIOC, GPIO_AF5_SPI3, GPIO_NOPULL } 
    #else
        #error("No config for SPI3 found");
    #endif 
    /* Definition for SPIx's NVIC */
    #define SPI3_IRQ                             { SPI3_IRQn, SPI_IRQ_PRIO, 0 }
    #define SPI3_IRQHandler                      SPI3_IRQHandler
    /* Definition for SPIDEV3's DMA */
    #define SPI3_TX_DMA                          DMA2_Channel2, DMA_REQUEST_3, DMA2_Channel2_IRQn
    #define SPI3_RX_DMA                          DMA2_Channel1, DMA_REQUEST_3, DMA2_Channel1_IRQn
    #define SPI3_DMA_TX_IRQHandler               DMA2_Channel2_IRQHandler
    #define SPI3_DMA_RX_IRQHandler               DMA2_Channel1_IRQHandler
#endif /* SPI3 */
/* ----- End of Fixed part, change below  -------------------------------------------- */

/* ----- Configurable part, change below  -------------------------------------------- */
#if defined(USE_SPI1) && defined(SPI1)
  #ifndef SPI1_USE_NSS
      /* Must define either NSS or NSEL pin, NSS goes first */
      #define SPI1_NSEL                     { GPIO_PIN_15,  GPIOA, 0, GPIO_PULLUP }
  #endif
  #ifdef SPI1_USE_DNC 
    #define SPI1_DNC                        { GPIO_PIN_5 , GPIOC, 0, GPIO_NOPULL }
  #endif  
  #ifdef SPI1_USE_RST 
    #define SPI1_RST                        { GPIO_PIN_6 , GPIOC, 0, GPIO_NOPULL }
  #endif  
  #ifdef SPI1_USE_BUSY 
    #define SPI1_BUSY                       { GPIO_PIN_8 , GPIOC, 0, GPIO_NOPULL }
    #ifdef SPI1_USE_BUSY_IRQ
      #define SPI1_BUSY_IRQ                 { EXTI9_5_IRQn, SPI_IRQ_PRIO, 0 }
    #endif
  #endif
#endif // SPIDEV1

#if defined(USE_SPI2) && defined (SPI2)
  #ifndef SPI2_USE_NSS
      /* Must define either NSS or NSEL pin, NSS goes first */
      #define SPI2_NSEL                     { GPIO_PIN_12, GPIOB, 0, GPIO_PULLUP }
  #endif
  #ifdef SPI2_USE_DNC 
    #define SPI2_DNC                        { GPIO_PIN_5 , GPIOC, 0, GPIO_NOPULL }
  #endif  
  #ifdef SPI2_USE_RST 
    #define SPI2_RST                        { GPIO_PIN_6 , GPIOC, 0, GPIO_NOPULL }
  #endif  
  #ifdef SPI2_USE_BUSY 
    #define SPI2_BUSY                       { GPIO_PIN_8 , GPIOC, 0, GPIO_NOPULL }
    #ifdef SPI2_USE_BUSY_IRQ
      #define SPI2_BUSY_IRQ                 { EXTI9_5_IRQn, SPI_IRQ_PRIO, 0 }
    #endif
  #endif
#endif // if defined(USE_SPI2) && defined (SPI2)
#if defined(USE_SPI3) && defined (SPI3)
#endif // if defined(USE_SPI3) && defined (SPI3)


/******************************************************************************************/
/* BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBS*/
/* BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBS*/
/* BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBS*/
/* BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBSPI BBS*/
/******************************************************************************************/

/* The BBSPI configuration is hardware dependent */

#if defined(STM32L476NUCLEO) || defined(STM32L476BAREMETAL)

    /*
     * We use BBSPI2 for RFM communication. RFM12 and RFM69 are tested and working
     * RFM12: uses a 4-wire-interface, consisting of MOSI, MISO, SCK and NSEL
     *        Data available interrupt is signaled on an falling edge on MOSI while
     *        NSEL is active (low)
     * RFM69: uses a 5-wire-interface, consisting of MOSI, MISO, SCK and NSEL plus
     *        a dedicated Data Available interupt ( DIO0 of RFM69 ) which is routed
     *        to PA11 and configured as "busy" interrupt in SPI-device
     */

    #if defined(USE_BBSPI2)
      #define SPI2_MOSI                           { GPIO_PIN_15, GPIOB, 0, GPIO_NOPULL }
      #define SPI2_SCK                            { GPIO_PIN_13, GPIOB, 0, GPIO_NOPULL }
      #define SPI2_NSEL                           { GPIO_PIN_12, GPIOB, 0, GPIO_NOPULL }
      #ifdef SPI2_USE_MISO 
          #define SPI2_MISO                       { GPIO_PIN_14, GPIOB, 0, GPIO_PULLUP }
          #ifdef SPI2_USE_MISO_IRQ
            #define SPI2_MISO_IRQ                 { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
          #endif
      #endif
      #ifdef SPI2_USE_INP 
        #define SPI2_INP                        { GPIO_PIN_12 , GPIOA, 0, GPIO_NOPULL }
        #ifdef SPI2_USE_INP_IRQ
          #define SPI2_INP_IRQ                  { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
       #endif
      #endif
      #ifdef SPI2_USE_BUSY 
        #define SPI2_BUSY                       { GPIO_PIN_11 , GPIOA, 0, GPIO_NOPULL }
        #ifdef SPI2_USE_BUSY_IRQ
          #define SPI2_BUSY_IRQ                 { EXTI15_10_IRQn, SPI_IRQ_PRIO, 0 }
       #endif
      #endif
    #endif
    #if defined(USE_BBSPI1)
        #define SPI1_MOSI                         { GPIO_PIN_5  , GPIOB, 0, GPIO_NOPULL }
        #define SPI1_SCK                          { GPIO_PIN_3  , GPIOB, 0, GPIO_NOPULL }
        #define SPI1_NSEL                         { GPIO_PIN_15 , GPIOA, 0, GPIO_NOPULL }
        #ifdef SPI1_USE_MISO 
          #define SPI1_MISO                       { GPIO_PIN_4, GPIOB, 0, GPIO_PULLUP }
          #ifdef SPI2_USE_MISO_IRQ
            #define SPI2_MISO_IRQ                 { EXTI4_IRQn, SPI_IRQ_PRIO, 0 }
          #endif

        #endif
        #ifdef SPI1_USE_DNC 
          #define SPI1_DNC                        { GPIO_PIN_2  , GPIOB, 0, GPIO_NOPULL }
        #endif  
        #ifdef SPI1_USE_RST 
          #define SPI1_RST                        { GPIO_PIN_6 , GPIOC, 0, GPIO_NOPULL }
        #endif  
        #ifdef SPI1_USE_BUSY 
          #define SPI1_BUSY                       { GPIO_PIN_8 , GPIOC, 0, GPIO_NOPULL }
          #ifdef SPI1_USE_BUSY_IRQ
            #define SPI1_BUSY_IRQ                 { EXTI9_5_IRQn, SPI_IRQ_PRIO, 0 }
          #endif
        #endif
    #endif

#elif defined(STM32L476EVAL)

#elif defined(BL475IOT)

  #define BBSPI1_MOSI                           { GPIO_PIN_15, GPIOB, 0, GPIO_NOPULL }
  #define BBSPI1_SCK                            { GPIO_PIN_13, GPIOB, 0, GPIO_NOPULL }
  #define BBSPI1_NSEL                           { GPIO_PIN_12, GPIOB, 0, GPIO_NOPULL }
  #ifdef BBSPI1_USE_MISO 
      #define BBSPI1_MISO                       { GPIO_PIN_9 , GPIOB, 0, GPIO_PULLUP }
      #ifdef BBSPI1_USE_MISO_IRQ
        #define BBSPI1_MISO_IRQ                      { EXTI9_5_IRQn, BBSPI_IRQ_PRIO, 0 }
      #endif
  #endif

#elif defined(DRAGONFLY476)

  #define BBSPI1_MOSI                           { GPIO_PIN_2 , GPIOC, 0, GPIO_NOPULL }
  #define BBSPI1_SCK                            { GPIO_PIN_3 , GPIOC, 0, GPIO_NOPULL }
  #define BBSPI1_NSEL                           { GPIO_PIN_7 , GPIOC, 0, GPIO_NOPULL }
  #ifdef BBSPI1_USE_MISO 
      #define BBSPI1_MISO                       { GPIO_PIN_6 , GPIOC, 0, GPIO_PULLUP }
      #ifdef BBSPI1_USE_MISO_IRQ
        #define BBSPI1_MISO_IRQ                      { EXTI9_5_IRQn, BBSPI_IRQ_PRIO, 0 }
      #endif
  #endif

#else
  #error "No valid device configuration in devices_config.h"
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __SPI_CONFIG_H */