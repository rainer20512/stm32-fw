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
  * This is the configuration file for CM7 core !!
  *                                    ---
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

/* Clock sources for USART1 / USART6 */
/* Choose one */
//#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_PCLK2
//#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_PLL2Q
//#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_PLL3Q
//#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_CSI
#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_HSI
//#define USART16_CLKSOURCE         RCC_USART16CLKSOURCE_LSE

/* Clock sources for USART2 / USART3 / UART4,5,7,8 */
/* Choose one */
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PCLK2
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PLL2Q
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PLL3Q
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_CSI
#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_HSI
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_LSE

/* Clock source for LPUART1 */
/* Choose one */
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_PCLK3
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_PLL2Q
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_PLL3Q
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_CSI
#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_HSI
//#define LPUART1_CLKSOURCE    RCC_LPUART1CLKSOURCE_LSE


/*******************************************************************************
 * Maximum config below 
 * choose suitable configs for your platform
 ******************************************************************************/
//#define USE_USART1
//#define USE_USART1_ALTN1
/* Undefine, if RX or TX shall be done via DMA */
//#define COM1_USE_TX_DMA
//#define COM1_USE_RX_DMA

//#define USE_USART2
//#define USE_USART2_ALTN1
//#define COM2_USE_TX_DMA
//#define COM2_USE_RX_DMA
//#define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
//#define USE_USART2_DEBUG

//#define USE_USART3
//#define USE_USART3_ALTN1
//#define USE_USART3_ALTN2
//#define USE_USART3_ALTN3
/* Undefine, if RX or TX shall be done via DMA */
//#define COM3_USE_TX_DMA
//#define COM3_USE_RX_DMA

//#define USE_UART4
//#define USE_UART4_ALTN1


//#define USE_UART5


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

#if defined(STM32H745NUCLEO)

    #if USE_ONEWIRE > 0
        #define USE_USART1
        #define ONEWIRE_DEV            HW_COM1
        #define USE_USART1_ALTN1
        #define COM1_USE_TX_DMA
        #define COM1_USE_RX_DMA
        #define USART1_CLKSOURCE       RCC_USART1CLKSOURCE_HSI
    #endif

    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM3
    #define RTCTIMER_IRQn           LPTIM3_IRQn
    #define RTCTIMER_IRQHandler     LPTIM3_IRQHandler

    // #define USE_USART2
    //#define USE_USART2_ALTN1
    #define COM2_USE_TX_DMA
    #define COM2_USE_RX_DMA
    #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
    //#define USE_USART2_DEBUG
    
    #define USE_USART3
    #define COM3_USE_TX_DMA
    //#define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI
    #define USE_USART3_DEBUG
 
    // #define USE_UART4
    #define COM4_USE_TX_DMA
    #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI
    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    //#define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    #define USE_LPUART1_ALTN2
    #define USE_LPUART1_DEBUG

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

    #if USE_QENCODER > 0
        #define QENC_DEV        HW_QENC1
        #define USE_QENC1_TIM2
        #define USE_QENC1_IRQ
        #define USE_QENC1_PBTN1
    #endif

  #define USE_ADC3
  #define ADC3_USE_IRQ
  #define ADC3_USE_DMA 
  #define USER_ADC      HW_ADC3

  #if USE_PERIPHTIMER > 0
      #define PERIPH_TIMER          HW_TIM15
      #define PERIPH_TIMER_TRG_ADC  ADC_EXTERNALTRIG_T15_TRGO
      #define USE_TIM15
      #define USE_TIM15_ALTN1
  #endif   

  #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
                           /* HW_Timer, PWM channel [1..4], inverted?, autostart ? */
      #define HW_PWM_CHANNELS { &HW_PWMTIMER, LCD_BKLGHT_CH, 0, 0 }, 
   #endif

  #if USE_QSPI > 0
      #define QSPI_DEV          HW_QSPI1
      #define QSPI_HND          QSpi1Handle
      #define USE_QSPI1
      #define QSPI1_CLKSPEED    300000       // Use low clk rates on breadboard
      #define QSPI1_HAS_LP_MODE
      #define USE_QSPI1_ALTN4
      #define QSPI1_USE_IRQ

      // #define QSPI1_USE_DMA Not implemented yet 
  #endif

  #if USE_ETH > 0
    #define ETH_USE_RMII
    #define ETH_DEV             HW_ETH
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

#elif defined(STM32H747IDISCO)


    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM3
    #define RTCTIMER_IRQn           LPTIM3_IRQn
    #define RTCTIMER_IRQHandler     LPTIM3_IRQHandler

    #define USE_USART1              // USART1-TX (PA9 ) and RX ( PA10 ) are tied to STLINK VCP
    #define USE_USART1_ALTN1
    #define COM1_USE_TX_DMA
    // #define COM1_USE_RX_DMA
    #define USART1_CLKSOURCE         RCC_USART1CLKSOURCE_HSI
    #define USE_USART1_DEBUG
    
    //#define USE_USART2              // USART1-TX (PD5) and RX (PD6) are tied to STLINK PMOD-Connector Pin 2 and Pin 3
    // #define USE_USART2_ALTN1
    #define COM2_USE_TX_DMA
    // #define COM2_USE_RX_DMA
    #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
    //#define USE_USART2_DEBUG

    //#define USE_USART3
    #define COM3_USE_TX_DMA
    //#define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI
    //#define USE_USART3_DEBUG
 
    // #define USE_UART4
    #define COM4_USE_TX_DMA
    #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI

    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    // UART8 TX-PJ8 and RX-PJ9 are tied to arduino connector Tx and RX, use these for CM4 debug output
    // #define USE_UART8               
    #define USE_UART8_ALTN1
    #define COM8_USE_TX_DMA
    // #define COM2_USE_RX_DMA
    #define UART8_CLKSOURCE         RCC_UART8CLKSOURCE_HSI
    // #define USE_UART8_DEBUG

    //#define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    #define USE_LPUART1_ALTN2
    #define USE_LPUART1_DEBUG

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

    #define USE_ADC3
    #define ADC3_USE_IRQ
    #define ADC3_USE_DMA 
    #define USER_ADC      HW_ADC3

  #if USE_PERIPHTIMER > 0
      #define PERIPH_TIMER          HW_TIM15
      #define PERIPH_TIMER_TRG_ADC  ADC_EXTERNALTRIG_T15_TRGO
      #define USE_TIM15
      #define USE_TIM15_ALTN1
  #endif   

  #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
                           /* HW_Timer, PWM channel [1..4], inverted?, autostart ? */
      #define HW_PWM_CHANNELS { &HW_PWMTIMER, LCD_BKLGHT_CH, 0, 0 }, 
   #endif

  #if USE_QSPI > 0
      #define QSPI_DEV          HW_QSPI1
      #define QSPI_HND          QSpi1Handle
      #define USE_QSPI1
      #define QSPI1_CLKSPEED    300000       // Use low clk rates on breadboard
      #define QSPI1_HAS_LP_MODE
      #define USE_QSPI1_ALTN4
      #define QSPI1_USE_IRQ

      // #define QSPI1_USE_DMA Not implemented yet 
  #endif

  #if USE_ETH > 0
    #define ETH_USE_RMII
    #define ETH_DEV             HW_ETH
  #endif

  #define DEFAULT_STOP_MODE                2

#elif defined(PORTENTAH7)


    /* Define the LPTimer that does timekeeping in case RTC is not used for that */
    #define RTCTIMER                LPTIM3
    #define RTCTIMER_IRQn           LPTIM3_IRQn
    #define RTCTIMER_IRQHandler     LPTIM3_IRQHandler

 
    // Uart4 is connected to UART0 on breakout board, TX=PA0, RX=PI9
    #define USE_UART4  
    #define USE_UART4_ALTN6
    // Clocksource is defined on top for all u(s)arts
    #define COM4_USE_TX_DMA
    //#define COM4_USE_RX_DMA
    #define USE_UART4_DEBUG

    // Uart6 is connected to UART2 on breakout board, TX=PG14, RX=PG9
    // #define USE_USART6  
    #define USE_USART6_ALTN1
    // Clocksource is defined on top for all u(s)arts
    #define COM6_USE_TX_DMA
    #define COM6_USE_RX_DMA
    // #define USE_USART6_DEBUG

    // UART8 TX-PJ8 and RX-PJ9 is connected to UART3 on breakout board
    // #define USE_UART8               
    #define USE_UART8_ALTN1
    #define COM8_USE_TX_DMA
    // #define COM2_USE_RX_DMA
    // Clocksource is defined on top for all u(s)arts
    // #define USE_UART8_DEBUG

    // LPUART1 TX-PA9 and RX-PA10 is connectedto UART1 on breakout board
    //#define USE_LPUART1
    #define COM9_USE_TX_DMA
    #define USE_LPUART1_ALTN1
    //#define COM9_USE_RX_DMA
    // Clocksource is defined on top for all u(s)arts
    #define USE_LPUART1_DEBUG

    // #define USE_I2C1

    #define USE_ADC3
    #define ADC3_USE_IRQ
    #define ADC3_USE_DMA 
    #define USER_ADC      HW_ADC3

  #if USE_PERIPHTIMER > 0
      #define PERIPH_TIMER          HW_TIM15
      #define PERIPH_TIMER_TRG_ADC  ADC_EXTERNALTRIG_T15_TRGO
      #define USE_TIM15
      #define USE_TIM15_ALTN1
  #endif   


  #if USE_HW_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
                           /* HW_Timer, PWM channel [1..4], inverted?, autostart ? */
      #define HW_PWM_CHANNELS { &HW_PWMTIMER, LCD_BKLGHT_CH, 0, 0 }, 
   #endif

  #if USE_QSPI > 0
      #define XSPI_DEV          HW_QSPI1
      #define QSPI_HND          QSpi1Handle
      #define USE_QSPI1
      #define QSPI1_CLKSPEED    100000000 
      #define QSPI1_CLKSOURCE   RCC_QSPICLKSOURCE_D1HCLK           
      // #define QSPI1_CLKSOURCE   RCC_QSPICLKSOURCE_PLL           // PLL1Q
      // #define QSPI1_CLKSOURCE   RCC_QSPICLKSOURCE_PLL2          // PLL2R
      // #define QSPI1_CLKSOURCE   RCC_QSPICLKSOURCE_CLKP
      #define QSPI1_HAS_LP_MODE
      #define USE_QSPI1_ALTN5
      #define QSPI1_USE_IRQ

      // #define QSPI1_USE_DMA Not implemented yet 
  #endif

  #if USE_ETH > 0
    #define ETH_USE_RMII
    #define ETH_DEV             HW_ETH
  #endif

  #define DEFAULT_STOP_MODE                2

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
#if defined(USE_CAN1) || defined(USE_CAN2) 
    #define USE_CAN     1
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
