/**
  ******************************************************************************
  * @file    config.h
  * @author  Rainer
  * @brief   global configuration and definitions
  *
  * @note    This is the CM7 config part !!!
  *                      ---
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

#include "debug.h"

/*
 ********************************************************************************
 * Board selection
 ********************************************************************************
 */ 
// #define STM32H745NUCLEO
// #define STM32H747IDISCO
#define PORTENTAH7
/*
 ********************************************************************************
 * Application selection
 ********************************************************************************
 */ 

#define UNIVERSAL 

/*
 ********************************************************************************
 * Code Versioning
 ********************************************************************************
 */ 
#define MAJOR_VERSION   1
#define MINOR_VERSION   01

/*
 ********************************************************************************
 * Application tayloring
 ********************************************************************************
 */ 
#define USE_ONEWIRE                 0
#define USE_DS18X20                 0
#define USE_EEPROM_EMUL             0
#define USE_QENCODER                0
#define USE_SECONDTIMER             1
#define USE_HW_PWMTIMER             1
#define USE_BASICTIMER              1
#define USE_QSPI                    1            // When QSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BASICTIMER_FOR_TICKS    1            // Use Basictimer to generate 1ms Ticks instead of SYSTICK-Timer
#define USE_BDMA                    0
#define USE_PERIPHTIMER             1


#define GENERAL_BAUDRATE    500000
#define CAN_BAUDRATE        500000       // default CAN Baudrate
#define DEBUG_BAUDRATE      38400        // When waking up from stop, the max baudrate is about 45kBaud
#define HW_PWM_FREQUENCY    20000

#define DEBUG_IRQ_PRIO      0x0f         // Give the debug uart the lowest priority
#define USART_IRQ_PRIO      0x0e         // Give the other uarts the next lowest priority
#define SPI_IRQ_PRIO        0x0d         // Give the SPI the next lowest priority
#define BBSPI_IRQ_PRIO      0x0c         // Give BBSPI the next lowest
#define BUTTON_IRQ_PRIO     0x0c         // Give push buttons the same prio as BBSPI input
#define ADC_IRQ_PRIO        0x09         // ADC also has high prio due to high conversion rates
#define EXTI_IRQ_PRIO       0x07         // Give Pin Change the third highest
#define I2C_IRQ_PRIO        0x06         // Give I2C a high prio
#define CAN_IRQ_PRIO        0x07         // Give CAN a very high prio
#define IPC_IRQ_PRIO        0x04         // IPC comes next
#define RTC_IRQ_PRIO        0x03         // RTC has the next higher prio
#define BASETIM_IRQ_PRIO    0x02         // Give the base timer the highest possible priority

#define DEFAULT_STOP_MODE   2

//#define USER_CLOCKCONFIG         CLK_HSI_08MHZ    /*   8 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_16MHZ    /*  16 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_32MHZ    /*  32 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_64MHZ    /*  64 MHz, source HSI, VOSrange3, 1 WS */

//#define USER_CLOCKCONFIG         CLK_PLL_50MHZ     /*  50 MHz, source PLL with HSE, Vrange3, 1 WS */
//#define USER_CLOCKCONFIG          CLK_PLL_100MHZ     /* 100 MHz, source PLL with HSE, Vrange1, 1 WS */
//#define USER_CLOCKCONFIG          CLK_PLL_200MHZ   /* 200 MHz, source PLL with HSE, Vrange1, 2 WS */
//#define USER_CLOCKCONFIG          CLK_PLL_300MHZ   /* 300 MHz, source PLL with HSE, Vrange1, 2 WS */
#define USER_CLOCKCONFIG          CLK_PLL_400MHZ   /* 400 MHz, source PLL with HSE, Vrange1, 3 WS */
//#define USER_CLOCKCONFIG          CLK_PLL_480MHZ   /* 480 MHz, source PLL with HSE, Vrange0, 4 WS, not recommended for long term useage, requires LDO equipped */
//#define USER_CLOCKCONFIG          CLK_HSE_VRNG3_xxMHZ_0WS    /* 8-45MHz, source HSE, VOSrange3, 0 WS, depends on HSE crystal, only available if HSE crystal is mounted */


/******************************************************************************
 * Choose one in case of USE_QSPI == 1 
******************************************************************************
 */
#define USE_XSPI_MX25               1
#define USE_XSPI_MT25Q              0


/******************************************************************************
 * Check and set constraints for BaseTimer
 *****************************************************************************/
#if USE_BASICTIMER_FOR_TICKS > 0
    #if defined(USE_BASICTIMER) && USE_BASICTIMER < 1
        #warning "USE_BASICTIMER redefined due to system useage"
        #undef USE_BASICTIMER
        #define USE_BASICTIMER 1
    #endif
#endif

#if DEBUG_PROFILING > 0 || USE_BASICTIMER > 0
    #if defined(USE_BASICTIMER) && USE_BASICTIMER < 1
        #warning "USE_BASICTIMER redefined due to system useage"
        #undef USE_BASICTIMER
        #define USE_BASICTIMER 1
    #endif
    #define USE_TIM7
    #define BASTIM_HANDLE TIM7Handle
    #define BASTIM_HW HW_TIM7
#endif


/******************************************************************************
 * Check and set constraints for differeht hardware types
 *****************************************************************************/
#if defined(STM32H745NUCLEO)
    #undef  HW_HAS_HSE_CRYSTAL
    #define  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       8000000
#endif

#if defined(STM32H747IDISCO)  || defined(PORTENTAH7)
    /* STM32H747I-DISCO has an external 25MHz Oscillator on board */
    #undef  HW_HAS_HSE_CRYSTAL
    #define  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       25000000
#endif

/**** !!! Note: HSE_VALUE has to be set in stm32h7xx_hal_conf.h accordingly !!! ****/

/* Can we use an HSE Clock ? */
#if defined(HW_HAS_HSE_CRYSTAL) || defined(HW_HAS_HSE_BYPASS)
    #define HW_HAS_HSE
#endif

#if defined(PORTENTAH7)
    #define HW_HAS_LSE_BYPASS
#endif

/******************************************************************************
 * Setup the MCU family
 *****************************************************************************/
#if  defined(STM32H747xx) || defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    #define STM32H7_FAMILY
#elif defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx)
    #define STM32L4_FAMILY
#else
    #error "Unkonwn MCU family!"
#endif

/*
 ********************************************************************************
 * useful macros
 ********************************************************************************
 */
  
#ifndef __cplusplus
    typedef uint32_t bool;

    #define true  1
    #define false 0
#endif


#define MINMAX_UNSET	(int16_t)0x8000     /* Indicates "Min or Max is currently unset" */

#define _CONCAT(a,b)    a##b
#define CONCAT(a,b)     _CONCAT(a,b)

#define _STR(x) #x
#define STR(x) _STR(x)

#define MIN(a,b)        ( a < b ? a : b )
#define MAX(a,b)        ( a > b ? a : b )
#define ABS(a)          ( a < 0 ? -1*a : a )
#define UCASE(x)        ( x >= 'a' && x <= 'z' ? x & ~0x20 : x )

/*
 ********************************************************************************
 * mandatory or frequent includes 
 ********************************************************************************
 */

#include "error.h"
/*
 ********************************************************************************
 * Build the config-string for version.h buildinfo
 ********************************************************************************
 */
#if BUILD_CONFIG_STR > 0
    #define PPCAT_NXE(A, B)   A " = " #B "\r\n"
    #define PPCAT_E(A)     PPCAT_NXE(#A,A)

#define MK_CONFIGSTR(a,i) \
    const char ConfigStr##i[] = #a; \
    const uint8_t ConfigVal##i = a; 

    MK_CONFIGSTR(USE_ONEWIRE,1)
    MK_CONFIGSTR(USE_DS18X20,2)
    MK_CONFIGSTR(USE_EEPROM_EMUL,3)
    MK_CONFIGSTR(USE_QENCODER,4)
    MK_CONFIGSTR(USE_SECONDTIMER,5)
    MK_CONFIGSTR(USE_HW_PWMTIMER,6)
    MK_CONFIGSTR(USE_BASICTIMER,7)
    MK_CONFIGSTR(USE_QSPI,8)
    MK_CONFIGSTR(USE_BASICTIMER_FOR_TICKS,9)
    MK_CONFIGSTR(USE_BDMA,10)

    #define MAX_CONFIGSTR   10

    const char *ConfigStrings[MAX_CONFIGSTR] = 
        {
            ConfigStr1,  ConfigStr2,  ConfigStr3,  ConfigStr4,  ConfigStr5,  ConfigStr6,  ConfigStr7,  ConfigStr8,
            ConfigStr9,  ConfigStr10, // ConfigStr11, ConfigStr12, ConfigStr13, ConfigStr14, ConfigStr15, ConfigStr16,
            // ConfigStr17, ConfigStr18, ConfigStr19, ConfigStr20, ConfigStr21, ConfigStr22, ConfigStr23, ConfigStr24,
        };
    const uint8_t ConfigValues[MAX_CONFIGSTR] = 
        {
            ConfigVal1,  ConfigVal2,  ConfigVal3,  ConfigVal4,  ConfigVal5,  ConfigVal6,  ConfigVal7,  ConfigVal8,
            ConfigVal9,  ConfigVal10, // ConfigVal11, ConfigVal12, ConfigVal13, ConfigVal14, ConfigVal15, ConfigVal16,
            // ConfigVal17, ConfigVal18, ConfigVal19, ConfigVal20, ConfigVal21, ConfigVal22, //ConfigVal23, ConfigVal24,
        };


#endif


#if 0
    #define PPCAT_NXE(A, B)   A " = " #B "\r\n"
    #define PPCAT_E(A)     PPCAT_NXE(#A,A)

    #define CONFIG_STRING  \
    PPCAT_E(USE_ONEWIRE)    \
    PPCAT_E(USE_DS18X20)    \
    PPCAT_E(USE_EEPROM_EMUL)      \
    PPCAT_E(USE_QENCODER)  \
    PPCAT_E(USE_SECONDTIMER)  \
    PPCAT_E(USE_HW_PWMTIMER)  \
    PPCAT_E(USE_BASICTIMER)  \
    PPCAT_E(USE_QSPI)  \
    PPCAT_E(USE_BASICTIMER_FOR_TICKS)  \
    PPCAT_E(USE_BDMA)  \

#endif

#endif /* __CONFIG_H */
