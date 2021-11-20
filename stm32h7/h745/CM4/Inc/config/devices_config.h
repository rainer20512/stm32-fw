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
  * This is the CM4 part
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

#define SPI_SETTER(a,b)             a
#define SPI_GETTER(a,b)             b

/* Clock sources for SPI1, SPI2, SPI3 */
/* Choose one SET/GET pair */
#define SPI123_CLKSOURCE_SET         RCC_SPI123CLKSOURCE_PLL     
#define SPI123_CLKSOURCE_GET         HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)
//#define SPI123_CLKSOURCE_SET         RCC_SPI123CLKSOURCE_PLL2
//#define SPI123_CLKSOURCE_GET         HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)
//#define SPI123_CLKSOURCE_SET         RCC_SPI123CLKSOURCE_PLL3
//#define SPI123_CLKSOURCE_GET         HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)
//#define SPI123_CLKSOURCE_SET         RCC_SPI123CLKSOURCE_PIN
//#define SPI123_CLKSOURCE_GET         HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)
//#define SPI123_CLKSOURCE_SET         RCC_SPI123CLKSOURCE_CLKP
//#define SPI123_CLKSOURCE_GET         HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)

/* Clock sources for SPI4, SPI5 */
/* Choose one SET/GET pair */
//#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_D2PCLK1
//#define SPI45_CLKSOURCE_GET         HAL_RCC_GetPCLK2Freq()
//#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_PLL2
//#define SPI45_CLKSOURCE_GET         RHB ToDo
//#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_PLL3
//#define SPI45_CLKSOURCE_GET         RHB ToDo
#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_HSI
#define SPI45_CLKSOURCE_GET         HSI_VALUE
//#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_CSI
//#define SPI45_CLKSOURCE_GET       CSI_VALUE
//#define SPI45_CLKSOURCE_SET         RCC_SPI45CLKSOURCE_HSE
//#define SPI45_CLKSOURCE_GET       HSE_VALUE

/* Clock sources for SPI6 */
/* Choose one SET/GET pair */
//#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_D3PCLK1
//#define SPI6_CLKSOURCE_GET         HAL_RCCEx_GetD3PCLK1Freq()
//#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_PLL2
//#define SPI6_CLKSOURCE_GET         RHB ToDo
//#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_PLL3
//#define SPI6_CLKSOURCE_GET         RHB ToDo
#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_HSI
#define SPI6_CLKSOURCE_GET         HSI_VALUE
//#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_CSI
//#define SPI6_CLKSOURCE_GET         CSI_VALUE
//#define SPI6_CLKSOURCE_SET         RCC_SPI6CLKSOURCE_HSE
//#define SPI6_CLKSOURCE_GET         HSE_VALUE


/* Clock sources for USART2 / USART3 / UART4,5,7,8 */
/* Choose one */
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PCLK2
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PLL2Q
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_PLL3Q
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_CSI
#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_HSI
//#define USART234578_CLKSOURCE     RCC_USART234578CLKSOURCE_LSE




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
/* Undefine, if RX or TX shall be done via DMA */
//#define COM2_USE_TX_DMA
//#define COM2_USE_RX_DMA

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
    #define RTCTIMER                LPTIM4
    #define RTCTIMER_IRQn           LPTIM4_IRQn
    #define RTCTIMER_IRQHandler     LPTIM4_IRQHandler

    //#define USE_USART2
    //#define USE_USART2_ALTN1
    #define COM2_USE_TX_DMA
    // #define COM2_USE_RX_DMA
    #define USART2_CLKSOURCE         RCC_USART2CLKSOURCE_HSI
    //#define USE_USART2_DEBUG
    
    // #define USE_USART3
    #define COM3_USE_TX_DMA
    //#define COM3_USE_RX_DMA
    #define USART3_CLKSOURCE         RCC_USART3CLKSOURCE_HSI
 
    #define USE_UART4
    #define USE_UART4_ALTN1
    #define COM4_USE_TX_DMA
    // #define COM4_USE_RX_DMA
    #define UART4_CLKSOURCE          RCC_UART4CLKSOURCE_HSI
    //#define USE_UART4_DEBUG


    // #define USE_UART5  
    #define COM5_USE_TX_DMA
    #define COM5_USE_RX_DMA
    #define UART5_CLKSOURCE          RCC_UART5CLKSOURCE_HSI

    #define USE_LPUART1
    #define COM9_USE_TX_DMA
    //#define COM9_USE_RX_DMA
    #define LPUART1_CLKSOURCE        RCC_LPUART1CLKSOURCE_HSI
    //#define USE_LPUART1_ALTN2
    #define USE_LPUART1_DEBUG

    #define USE_SPI1
    #define USE_SPI1_ALTN2
    // #define SPIDEV1_USE_IRQ
    #define SPI1_USE_MISO
    #define SPI1_BAUDRATE            6000000
    #define USE_SPI1_MASTER
    #define SPI1_USE_INP             /* Interrupt pin of enc28j60 */
    #define SPI1_USE_RST             /* Hardware Rest Pin of enc28j60 */
    #define SPI1_USE_INP_IRQ
    #define SPI1_INP_IRQ_MODE        GPIO_MODE_IT_FALLING

    // #define USE_I2C1

    // #define USE_I2C2
    #define I2C2_CLKSOURCE           I2cClock_HSI
    #define I2C2_SPEED               I2c_StandardMode
    #define I2C2_USE_IRQ
    #define I2C2_USE_DMA

  #if USE_ETH > 0
    #define ETH_USE_RMII
    #define ETH_USE_IRQ
    #define ETH_DEV             HW_ETH
  #endif

    #if USE_QENCODER > 0
        #define QENC_DEV        HW_QENC1
        #define USE_QENC1_TIM2
        #define USE_QENC1_IRQ
        #define USE_QENC1_PBTN1
    #endif

  #if USE_PWMTIMER > 0
      #define USE_TIM3
      #define USE_TIM3_ALTN2
      #define PWM_FREQUENCY 20000
      #define HW_PWMTIMER HW_TIM3
      #define LCD_BKLGHT_CH         1
  #endif

  #if USE_QSPI > 0
      #define QSPI_DEV          HW_QSPI1
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
