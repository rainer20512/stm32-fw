/**
  ******************************************************************************
  * @file    devices_config.h 
  * @author  Rainer
  * @brief   Definition of all used devices. 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICES_CONFIG_H
#define __DEVICES_CONFIG_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/*******************************************************************************
 * Maximum config below 
 * choose suitable configs for your platform
 ******************************************************************************/
//#define USE_USART1
//#define USE_USART1_ALTN1
/* Undefine, if RX or TX shall be done via DMA */
//#define COM1_USE_TX_DMA
//#define COM1_USE_RX_DMA
/* Choose one */
//#define USART1_CLKSOURCE    RCC_USART1CLKSOURCE_PCLK2
//#define USART1_CLKSOURCE    RCC_USART1CLKSOURCE_SYSCLK
//#define USART1_CLKSOURCE    RCC_USART1CLKSOURCE_HSI
//#define USART1_CLKSOURCE    RCC_USART1CLKSOURCE_LSE

//#define USE_USART2
//#define USE_USART2_ALTN1
/* Undefine, if RX or TX shall be done via DMA */
//#define COM2_USE_TX_DMA
//#define COM2_USE_RX_DMA
/* Choose one */
//#define USART2_CLKSOURCE    RCC_USART2CLKSOURCE_PCLK1
//#define USART2_CLKSOURCE    RCC_USART2CLKSOURCE_SYSCLK
//#define USART2_CLKSOURCE    RCC_USART2CLKSOURCE_HSI
//#define USART2_CLKSOURCE    RCC_USART2CLKSOURCE_LSE

//#define USE_USART3
//#define USE_USART3_ALTN1
//#define USE_USART3_ALTN2
//#define USE_USART3_ALTN3
/* Undefine, if RX or TX shall be done via DMA */
//#define COM3_USE_TX_DMA
//#define COM3_USE_RX_DMA
/* Choose one */
//#define USART3_CLKSOURCE    RCC_USART3CLKSOURCE_PCLK1
//#define USART3_CLKSOURCE    RCC_USART3CLKSOURCE_SYSCLK
//#define USART3_CLKSOURCE    RCC_USART3CLKSOURCE_HSI
//#define USART3_CLKSOURCE    RCC_USART3CLKSOURCE_LSE

//#define USE_UART4
//#define USE_UART4_ALTN1
/* DMA Transfer not yet implemented for UART4 */
/* Choose one */
//#define UART4_CLKSOURCE    RCC_UART4CLKSOURCE_PCLK1
//#define UART4_CLKSOURCE    RCC_UART4CLKSOURCE_SYSCLK
//#define UART4_CLKSOURCE    RCC_UART4CLKSOURCE_HSI
//#define UART4_CLKSOURCE    RCC_UART4CLKSOURCE_LSE


//#define USE_UART5
// No alternate pin configurations for UART5
/* DMA Transfer not yet implemented for UART5 */
/* Choose one */
//#define UART5_CLKSOURCE    RCC_UART5CLKSOURCE_PCLK1
//#define UART5_CLKSOURCE    RCC_UART5CLKSOURCE_SYSCLK
//#define UART5_CLKSOURCE    RCC_UART5CLKSOURCE_HSI
//#define UART5_CLKSOURCE    RCC_UART5CLKSOURCE_LSE


//#define USE_LPUART1
   /* Alternatives for LPUART1 : 
    * TX:[PB11,PC1,PG7] RX:[PB10,PC0,PG8]
    * ALTN1   TX:PB11 RX:PB10   AF8
    * ALTN2   TX:PC1  RX:PC0    AF8
    * DEFAULT TX:PG7  RX:PG8    AF8 
    */
//#define USE_LPUART1_ALTN2
//#define USE_LPUART1_ALTN1
/* Undefine, if RX or TX shall be done via DMA */
//#define COM9_USE_TX_DMA
//#define COM9_USE_RX_DMA
/* Choose one */
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_PCLK1
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_SYSCLK
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_HSI
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_LSE




//#define USE_SPI1
  /**
   * SPI1 alternatives ( There is another one at Port G, not configured here ): 
   *          ALTN1     ALTN2     ALTN3
   * ----------------------------------
   * NSS:   PA4/AF5, PA15/AF5, PE12/AF5
   * SCK:   PA5/AF5,  PB3/AF5, PE13/AF5
   * MISO:  PA6/AF5,  PB4/AF5, PE14/AF5
   * MOSI:  PA7/AF5,  PB5/AF5, PE15/AF5
   *
   * Choose one or create a new one with mixed configuration
   */   
//#define USE_SPIDEV1_ALTN1
//#define USE_SPIDEV1_ALTN2
//#define USE_SPIDEV1_ALTN3
/* Choose none or one */
//#define SPIDEV1_USE_DMA
//#define SPIDEV1_USE_IRQ

/* Undef, if HW NSS signal is used */
//#define SPIDEV1_USE_NSS

/* Undef, if used in slave Mode */
//#define SPIDEV1_MASTER

//#define USE_SPI2
  /**
   * SPI2 alternatives: 
   *          ALTN1     ALTN2     ALTN3
   * ----------------------------------
   * NSS:  PB12/AF5,  PB9/AF5,  PD0/AF5
   * SCK:  PB13/AF5, PB10/AF5,  PD1/AF5
   * MISO: PB14/AF5,  PC2/AF5,  PD3/AF5
   * MOSI: PB15/AF5,  PC3/AF5,  PD4/AF5
   *
   * Choose one or create a new one with mixed configuration
   */
 
//#define USE_SPIDEV2_ALTN1
//#define USE_SPIDEV2_ALTN2
//#define USE_SPIDEV2_ALTN3

/* Choose none or one */
//#define SPIDEV2_USE_IRQ
//#define SPIDEV2_USE_DMA

/* Undef, if HW NSS signal is used */
//#define SPIDEV2_USE_NSS

/* Undef, if used in slave Mode */
////#define SPIDEV2_MASTER



//#define USE_I2C1    
//#define USE_I2CDEV1_ALTN1
//#define USE_I2CDEV1_ALTN2

/* Undefine, if transfer in either direction shall be done via DMA */
/* or if transfer shall be done via IRQ                            */
//#define I2CDEV1_USE_RX_DMA
//#define I2CDEV1_USE_TX_DMA
//#define I2CDEV1_USE_IRQ

//#define USE_I2C2
//#define USE_I2CDEV2_ALTN1
//#define USE_I2CDEV2_ALTN2

/* Undefine, if transfer in either direction shall be done via DMA */
/* or if transfer shall be done via IRQ                            */
//#define I2CDEV2_USE_RX_DMA
//#define I2CDEV2_USE_TX_DMA
//#define I2CDEV2_USE_IRQ


/*******************************************************************
 * Set the perpherals clock sources                                
 * Note: If the WakeUp feature of the perpheral is to be used, the 
 * clocksource has to be ...HSI
 ******************************************************************/
#if defined(BL475IOT)

  #define USE_USART1
  #define USART1_CLKSOURCE         RCC_USART1CLKSOURCE_HSI
  #define USE_USART1_DEBUG

  #define USE_I2C2 
  #define USER_I2C2
  #define DEFAULT_STOP_MODE                1

  #define USER_CLOCKCONFIG         CLK_MSI_VRNG1_08MHZ_0WS    /*  8 MHz, source MSI, Vrange1, 0 WS */

#elif defined(STM32L476EVAL)

    /* USART1 with Tx/Rx at PB6/PB7 */
    #define USE_USART1
    // #define USE_USART1_ALTN1
    // #define COM1_USE_TX_DMA
    #define USART1_CLKSOURCE       RCC_USART1CLKSOURCE_HSI
    // #define USE_USART1_DEBUG

     /* LPUART with Tx/Rx at PG7/PG8 */
    #define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    #define USE_LPUART1_DEBUG
 
    //#define USE_SPI1
    // #define SPIDEV1_USE_IRQ
    #define USE_SPIDEV1_ALTN2
    #define SPIDEV1_USE_IRQ
    #define USE_SPI1_BAUDRATE        400000
    #define USE_SPI1_MASTER
    // #define USE_SPI2
    #define SPIDEV2_USE_IRQ
    #define SPIDEV2_USE_DMA
    #define USE_SPIDEV2_ALTN1
    #define USE_SPI2_BAUDRATE        400000
    #define USE_SPI2_MASTER
    // #define USE_SPI3
    #define SPIDEV3_USE_IRQ
    #define SPIDEV3_USE_DMA
    #define USE_SPIDEV3_ALTN1
    #define USE_SPI3_BAUDRATE        400000
    #define USE_SPI3_MASTER

    #define USE_ADC1
    #define ADC1_USE_IRQ
    #define ADC1_USE_DMA 
    #define USER_ADC                HW_ADC1

    #if USE_CAN > 0
        /* CAN with Tx=PB9, Rx=PB8 */
        #define USE_CAN1
        #define CAN1_USE_IRQ
        #define USE_CANDEV1_ALTN1
    #endif

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

    #if USE_HW_PWMTIMER > 0
        #define USE_TIM3
        #define USE_TIM3_ALTN2
        #define HW_PWM_FREQUENCY 20000
        #define HW_PWMTIMER HW_TIM3
        #define LCD_BKLGHT_CH         1
    #endif

    #if USE_QSPI > 0
        #define XSPI_DEV            HW_QSPI1
        #define USE_QSPI1
        #define USE_QSPI1_ALTN1
        #define QSPI1_USE_IRQ
        #define QSPI1_USE_DMA
    #endif

    #define DEFAULT_STOP_MODE                2

#if USE_THPSENSOR > 0  
    /* User I2C is I2C1 with PB6=SCL, PB7=SDA                                  */
    #define USE_I2C1
    #define I2C
    #undef  USE_I2CDEV1_ALTN1
    #undef  USE_I2CDEV1_ALTN2
    #define I2C1_CLKSOURCE           I2cClock_HSI
    #define I2C1_SPEED               I2c_StandardMode
    #define I2C1_USE_IRQ
    #define I2C1_USE_DMA
    #define USER_I2C1
#endif

#elif defined(STM32L476NUCLEO) || defined(STM32L476BAREMETAL)

    #if USE_ONEWIRE > 0
        #define USE_USART1
        #define ONEWIRE_DEV            HW_COM1
        #define USE_USART1_ALTN1
        //#define COM1_USE_TX_DMA
        //#define COM1_USE_RX_DMA
        #define USART1_CLKSOURCE       RCC_USART1CLKSOURCE_HSI
    #endif

    // SPI for RFM12
    #if USE_RFM12 > 0 || USE_RFM69 > 0
        #define USE_BBSPI2
        // #define USE_SPI2
        #define RFM12_DEV SPIDEV2_DEV
        #define USE_SPI2_ALTN1
        #define SPI2_USE_MISO
        #if USE_RFM12 > 0
            #define SPI2_USE_MISO_IRQ
        #else
            /* "Abuse" Busy-Pin for DIO0-Interrupt */
            #define SPI2_USE_BUSY
            #define SPI2_USE_BUSY_IRQ
        #endif
        #if USE_RFM_OOK
            /* Use General purpose input as interrupt line for "ook data available" */
            #define SPI2_USE_INP
            #define SPI2_USE_INP_IRQ            
        #endif
        #define SPI2_BAUDRATE 400000
        #define SPI2_DATASIZE 8
        //#define SPI2_USE_DNC
        //#define SPI2_USE_HW_IRQ
        //#define SPI2_USE_DMA
        //#define SPI2_USE_RST
        //#define SPI2_USE_BUSY
        //#define SPI2_USE_BUSY_IRQ
    #endif

    #if USE_USB > 0
        #define USE_USART2
        #define USE_USART2_ALTN1
        // #define COM2_USE_RX_DMA

        #define USBUART      HW_COM2
        #define USBUARTHND   (&HandleCOM2)

    #endif

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

    //#define USE_USART2
    #define USE_USART2_ALTN1
    #define COM2_USE_TX_DMA
    //#define COM2_USE_RX_DMA
    #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
    //#define USE_USART2_DEBUG

    // #define USE_USART3
    //#define COM3_USE_TX_DMA
    //#define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI
    // #define USE_UART4
    #define COM4_USE_TX_DMA
    #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI
    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    #define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    #define USE_LPUART1_ALTN2
    #define USE_LPUART1_DEBUG
 
    // #define USE_SPI1
    #define SPIDEV1_USE_IRQ
    #define USE_SPIDEV1_ALTN2
    #define SPIDEV1_USE_IRQ
    #define USE_SPI1_BAUDRATE        400000
    #define USE_SPI1_MASTER
    
    //#define USE_SPI2
    #define SPIDEV2_USE_IRQ
    #define SPIDEV2_USE_DMA
    #define USE_SPIDEV2_ALTN1
    #define USE_SPI2_BAUDRATE        400000
    #define USE_SPI2_MASTER
    // #define USE_SPI3
    #define SPIDEV3_USE_IRQ
    #define SPIDEV3_USE_DMA
    #define USE_SPIDEV3_ALTN1
    #define USE_SPI3_BAUDRATE        400000
    #define USE_SPI3_MASTER

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

    // Spi for ePaper
    #if USE_EPAPER > 0 || USE_DOGM132
        #define USE_BBSPI1
        // #define USE_SPI1
        #define EPAPER_DEV SPIDEV1_DEV
        #define USE_SPI1_ALTN2
        //#define SPI1_USE_MISO
        //#define SPI1_USE_MISO_IRQ
        #define SPI1_BAUDRATE 400000
        #define SPI1_DATASIZE 8
        #define SPI1_USE_DNC
        //#define SPI1_USE_HW_IRQ
        //#define SPI1_USE_DMA
        //#define SPI1_USE_RST
        //#define SPI1_USE_BUSY
        //#define SPI1_USE_BUSY_IRQ
    #endif


    #if USE_QENCODER > 0
        #define QENC_DEV        HW_QENC1
        #define USE_QENC1_TIM2
        #define USE_QENC1_IRQ
        #define USE_QENC1_PBTN1
    #endif

  #define USE_ADC1
  #define ADC1_USE_IRQ
  #define ADC1_USE_DMA 
  #define USER_ADC              HW_ADC1

  #if USE_PERIPHTIMER > 0
      #define PERIPH_TIMER          HW_TIM15
      #define PERIPH_TIMER_TRG_ADC  ADC_EXTERNALTRIG_T15_TRGO
      #define USE_TIM15
      #define USE_TIM15_ALTN1
  #endif   

  #define HW_PWM_FREQUENCY 20000
  #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
      #define HW_PWM_CHANNELS { &HW_PWMTIMER, LCD_BKLGHT_CH, 0, 0 }, 
  #endif

  #if USE_USER_PWMTIMER > 0
      #define USE_TIM5
      #define USE_TIM5_ALTN1
      #define USER_PWM_CHANNELS { &HW_TIM5, 3, 0, 1 }, { &HW_TIM5, 4, 0, 1 }, 
  #endif

  #define ALL_PWM_CHANNELS  { HW_PWM_CHANNELS USER_PWM_CHANNELS }

  #if USE_QSPI > 0
      #define XSPI_DEV          HW_QSPI1
      #define USE_QSPI1
      #define QSPI1_CLKSPEED    500000
      #define USE_QSPI1_ALTN1
      #define QSPI1_USE_IRQ
      #define QSPI1_USE_DMA
  #endif

  #if USE_OSPI1 > 0
      #define XSPI_DEV          HW_OSPI1
      #define OSPI1_CLKSPEED    500000
      #define USE_OSPI1_ALTN1
      #define OSPI1_USE_IRQ
      #define OSPI1_USE_DMA
      #define OSPI1_MODE_QUAD
  #endif

  #if USE_CAN > 0
    #define USE_CAN1
    #define USE_CANDEV1_ALTN1  /* PB8/PB9 for Can Rx/Tx */
    #define CAN1_USE_IRQ
  #endif

  #define DEFAULT_STOP_MODE                2

  #if USE_THPSENSOR > 0  
      /* User I2C is I2C1 with PB6=SCL, PB7=SDA                                  */
      #define USE_I2C1
      #define I2C
      #undef  USE_I2CDEV1_ALTN1
      #undef  USE_I2CDEV1_ALTN2
      #define I2C1_CLKSOURCE           I2cClock_HSI
      #define I2C1_SPEED               I2c_StandardMode
      #define I2C1_USE_IRQ
      #define I2C1_USE_DMA
      #define USER_I2C1
  #endif

#elif defined(STM32L4R9DISCOVERY)

    #if USE_ONEWIRE > 0
        #define USE_USART1
        #define ONEWIRE_DEV            HW_COM1
        #define USE_USART1_ALTN1
        #define COM1_USE_TX_DMA
        #define COM1_USE_RX_DMA
        #define USART1_CLKSOURCE       RCC_USART1CLKSOURCE_HSI
    #endif

    #if USE_USB > 0
    #endif

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

    // #define USE_USART2
    // PA2, PA3 
    #if defined(USE_USART2)
        #define USE_USART2_ALTN1
        #define COM2_USE_TX_DMA
        //#define COM2_USE_RX_DMA
        #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
        #define USE_USART2_DEBUG
    #endif

    // #define USE_USART3
    #define COM3_USE_TX_DMA
    #define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI

    // #define USE_UART4
    #define COM4_USE_TX_DMA
    #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI

    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    #define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    // PC0, PC1
    #define USE_LPUART1_ALTN2
    #define USE_LPUART1_DEBUG
 
    // #define USE_SPI1
    #define SPIDEV1_USE_IRQ
    #define USE_SPIDEV1_ALTN2
    #define SPIDEV1_USE_IRQ
    #define USE_SPI1_BAUDRATE        400000
    #define USE_SPI1_MASTER
    
    //#define USE_SPI2
    #define SPIDEV2_USE_IRQ
    #define SPIDEV2_USE_DMA
    #define USE_SPIDEV2_ALTN1
    #define USE_SPI2_BAUDRATE        400000
    #define USE_SPI2_MASTER
    // #define USE_SPI3
    #define SPIDEV3_USE_IRQ
    #define SPIDEV3_USE_DMA
    #define USE_SPIDEV3_ALTN1
    #define USE_SPI3_BAUDRATE        400000
    #define USE_SPI3_MASTER

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

    #define USE_ADC1
    #define ADC1_USE_IRQ
    #define ADC1_USE_DMA 
    #define USER_ADC      HW_ADC1
  
    #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWM_FREQUENCY 20000
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
    #endif

    #if USE_QSPI > 0
      #define XSPI_DEV          HW_QSPI1
      #define USE_QSPI1
      #define QSPI1_CLKSPEED    500000
      #define USE_QSPI1_ALTN1
      #define QSPI1_USE_IRQ
      #define QSPI1_USE_DMA
    #endif

    #if USE_CAN > 0
      #define USE_CAN1
      #define USE_CANDEV1_ALTN1  /* PB8/PB9 for Can Rx/Tx */
      #define CAN1_USE_IRQ
    #endif

    #define DEFAULT_STOP_MODE                2

#elif defined(STM32L4P5BAREMETAL)

    #define USE_LPUART1
    #if defined(USE_LPUART1)
        #define COM9_USE_TX_DMA
        //#define COM9_USE_RX_DMA
        #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
        // PC0, PC1
        #define USE_LPUART1_ALTN3
        #define USE_LPUART1_DEBUG
    #endif 

    #if USE_PERIPHTIMER > 0
      #define PERIPH_TIMER      HW_TIM2
      #define USE_TIM2
      #define USE_TIM2_ALTN1
    #endif   

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

#elif defined(STM32L4S9ZXXREF)

    #if USE_ONEWIRE > 0
        #define USE_USART1
        #define ONEWIRE_DEV            HW_COM1
        // #define USE_USART1_ALTN1
        #define COM1_USE_TX_DMA
        #define COM1_USE_RX_DMA
        #define USART1_CLKSOURCE       RCC_USART1CLKSOURCE_HSI
    #endif

    #if USE_USB > 0
    #endif

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

    #define USE_USART2
    // PA2, PA3 
    #if defined(USE_USART2)
        #define USE_USART2_ALTN1
        #define COM2_USE_TX_DMA
        //#define COM2_USE_RX_DMA
        #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
        //#define USE_USART2_DEBUG
    #endif

    // #define USE_USART3
    #define COM3_USE_TX_DMA
    #define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI

    // #define USE_UART4
    #define COM4_USE_TX_DMA
    #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI

    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    #define USE_LPUART1
    #if defined(USE_LPUART1)
        #define COM9_USE_TX_DMA
        //#define COM9_USE_RX_DMA
        #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
        // PC0, PC1
        #define USE_LPUART1_ALTN2
        #define USE_LPUART1_DEBUG
    #endif 
    // #define USE_SPI1
    #define SPIDEV1_USE_IRQ
    #define USE_SPIDEV1_ALTN2
    #define SPIDEV1_USE_IRQ
    #define USE_SPI1_BAUDRATE        400000
    #define USE_SPI1_MASTER
    
    //#define USE_SPI2
    #define SPIDEV2_USE_IRQ
    #define SPIDEV2_USE_DMA
    #define USE_SPIDEV2_ALTN1
    #define USE_SPI2_BAUDRATE        400000
    #define USE_SPI2_MASTER
    // #define USE_SPI3
    #define SPIDEV3_USE_IRQ
    #define SPIDEV3_USE_DMA
    #define USE_SPIDEV3_ALTN1
    #define USE_SPI3_BAUDRATE        400000
    #define USE_SPI3_MASTER

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

    #define USE_ADC1
    #define ADC1_USE_IRQ
    #define ADC1_USE_DMA 
    #define USER_ADC      HW_ADC1
  
    #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWM_FREQUENCY 20000
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
    #endif

    #if USE_OSPI > 0
      #define XSPI_DEV          HW_OSPI2
      #define USE_OSPI2         1
      #define OSPI2_CLKSPEED    500000
      #define USE_OSPI2_ALTN1
      #define OSPI2_USE_IRQ
      #define OSPI2_USE_DMA
      #define OSPI2_MODE_QUAD
      #define OSPI2_HAS_DS_MODE
      #undef  OSPI2_USE_DQS
    #endif

    // SPI for RFM12
    #if USE_RFM12 > 0 || USE_RFM69 > 0
        #define USE_BBSPI2
        // #define USE_SPI2
        #define RFM12_DEV SPIDEV2_DEV
        #define USE_SPI2_ALTN1
        #define SPI2_USE_MISO
        #if USE_RFM12 > 0
            #define SPI2_USE_MISO_IRQ
        #else
            /* "Abuse" Busy-Pin for DIO0-Interrupt */
            #define SPI2_USE_BUSY
            #define SPI2_USE_BUSY_IRQ
        #endif
        #if USE_RFM_OOK
            /* Use General purpose input as interrupt line for "ook data available" */
            #define SPI2_USE_INP
            #define SPI2_USE_INP_IRQ            
        #endif
        #define SPI2_BAUDRATE 400000
        #define SPI2_DATASIZE 8
        //#define SPI2_USE_DNC
        //#define SPI2_USE_HW_IRQ
        //#define SPI2_USE_DMA
        //#define SPI2_USE_RST
        //#define SPI2_USE_BUSY
        //#define SPI2_USE_BUSY_IRQ
    #endif

  #if USE_THPSENSOR > 0  
      /* User I2C is I2C1 with PB6=SCL, PB7=SDA                                  */
      #define USE_I2C1
      #define I2C
      #define  USE_I2CDEV1_ALTN1
      #undef  USE_I2CDEV1_ALTN2
      #define I2C1_CLKSOURCE           I2cClock_HSI
      #define I2C1_SPEED               I2c_StandardMode
      #define I2C1_USE_IRQ
      #define I2C1_USE_DMA
      #define USER_I2C1
  #endif

    #if USE_CAN > 0
      #define USE_CAN1
      #define USE_CANDEV1_ALTN1  /* PB8/PB9 for Can Rx/Tx */
      #define CAN1_USE_IRQ
    #endif

    #define DEFAULT_STOP_MODE                2

#elif defined(DRAGONFLY476)

  #define USE_LPUART1
  #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
  #define USE_LPUART1_ALTN2
  #define USE_LPUART1_DEBUG

  #define USE_ADC1
  #define ADC1_USE_IRQ
  #define ADC1_USE_DMA 

  #if USE_QSPI > 0
      #define XSPI_DEV          HW_QSPI1
      #define USE_QSPI1
      #define USE_QSPI1_ALTN1
      #define QSPI1_USE_IRQ
      #define QSPI1_USE_DMA
  #endif

  #define DEFAULT_STOP_MODE                2
#elif defined(T61)

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM1
    #define RTCTIMER_IRQn           LPTIM1_IRQn
    #define RTCTIMER_IRQHandler     LPTIM1_IRQHandler

    #if USE_CAN > 0
        /* CAN with Tx=PA12, Rx=PA11 */
        #define USE_CAN1
        #define CAN1_USE_IRQ

    #endif

#else
  #error "No valid device configuration in devices_config.h"
#endif

/*****************************************************************************/
/* Do not configure below
 *****************************************************************************/

/* Any User I2C defined at all ? */
#if defined(USE_I2C1) || defined(USE_I2C2) || defined(USE_I2C3)
    #define USE_I2C     1
#endif

/* Any User CAN defined at all ? */
#undef USE_CAN
#if defined(USE_CAN1) || defined(USE_CAN2) 
    #define USE_CAN     1
#else
    #define USE_CAN     0
#endif

/* Determine, which I2C to use */ 
#if defined(USE_I2C)
    #if defined(USER_I2C1)
        #define USER_I2C_HANDLE I2C1Handle
        #define USER_I2CDEV     HW_I2C1
    #elif defined(USER_I2C2)
        #define USER_I2C_HANDLE I2C2Handle
        #define USER_I2CDEV     HW_I2C2
    #elif defined(USER_I2C3)
        #define USER_I2C_HANDLE I2C3Handle
        #define USER_I2CDEV     HW_I2C3
    #else
        #error "No User I2C device defined"
    #endif
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEVICES_H */
