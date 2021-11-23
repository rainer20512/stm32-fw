/**
  ******************************************************************************
  * @file    config.h
  * @author  Rainer
  * @brief   global configuration and definitions
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

#define STM32H742REF
//#define STM32H743EVAL2

/*
 ********************************************************************************
 * Application selection
 ********************************************************************************
 */ 

#define UNIVERSAL 

/*
 ********************************************************************************
 * Versioning
 ********************************************************************************
 */ 
#define MAJOR_VERSION   1
#define MINOR_VERSION   04

/*
 ********************************************************************************
 * Application tayloring
 ********************************************************************************
 */ 
#define USE_ONEWIRE                 0
#define USE_DS18X20                 0
#define USE_EEPROM_EMUL             1
#define USE_QENCODER                0
#define USE_SECONDTIMER             1
#define USE_PWMTIMER                0
#define USE_BASICTIMER              1
#define USE_QSPI                    1            // When QSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BASICTIMER_FOR_TICKS    1            // Use Basictimer to generate 1ms Ticks instead of SYSTICK-Timer
#define USE_USB                     1   
#define USE_USB_FS                  0
#define USE_FMC_SRAM                0
#define USE_FMC_SDRAM               0
#define USE_FMC_NOR                 0
#define USE_ETH                     1
#define USE_FATFS                   1
#define USE_BDMA                    1

/* Choose one in case of USE_ETH == 1 
 * 
 * Note: As the enc28j60 does not support Checksum calculation for IP packets, lwip must compute these checksums manually when ENC28J60 is selected 
 * so, when USE_ETH_PHY_ENC28J60 = 1, in lwipopts.h the following #define's MUST be set to 1
 *  #define CHECKSUM_GEN_IP                 1
 *  #define CHECKSUM_GEN_UDP                1
 *  #define CHECKSUM_GEN_TCP                1 
 * on the ofher hand, when builtin ETH module is used, these three options MUST be set to 0
 */
#define USE_ETY_PHY_LAN8742         0
#define USE_ETH_PHY_ENC28J60        1

/* Choose one in case of USE_QSPI == 1  */
#define USE_XSPI_MX25               1
#define USE_XSPI_MT25Q              0


#define GENERAL_BAUDRATE    500000
#define CAN_BAUDRATE        500000       // default CAN Baudrate
#define DEBUG_BAUDRATE      38400        // When waking up from stop, the max baudrate is about 45kBaud

#define DEBUG_IRQ_PRIO      0x0f         // Give the debug uart the lowest priority
#define USART_IRQ_PRIO      0x0e         // Give the other uarts the next lowest priority
#define SPI_IRQ_PRIO        0x0d         // Give the SPI the next lowest priority
#define BBSPI_IRQ_PRIO      0x0c         // Give BBSPI the next lowest
#define BUTTON_IRQ_PRIO     0x0c         // Give push buttons the same prio as BBSPI input
#define ADC_IRQ_PRIO        0x09         // ADC also has high prio due to high conversion rates
#define EXTI_IRQ_PRIO       0x07         // Give Pin Change the third highest
#define I2C_IRQ_PRIO        0x06         // Give I2C a high prio
#define CAN_IRQ_PRIO        0x05         // Give CAN a very high prio
#define ETH_IRQ_PRIO        0x05         // Give ETH a very high prio
#define IPC_IRQ_PRIO        0x04         // IPC comes next
#define USB_IRQ_PRIO        0x04         // Give USB a very, very high prio
#define RTC_IRQ_PRIO        0x03         // RTC has the next higher prio
#define BASETIM_IRQ_PRIO    0x02         // Give the base timer the highest possible priority

#define DEFAULT_STOP_MODE   2

//#define USER_CLOCKCONFIG         CLK_HSI_VRNG3_08MHZ_0WS    /*   8 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_VRNG3_16MHZ_0WS    /*  16 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_VRNG3_32MHZ_0WS    /*  32 MHz, source HSI, VOSrange3, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_VRNG3_64MHZ_1WS    /*  64 MHz, source HSI, VOSrange3, 1 WS */
#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_100MHZ_1WS   /* 100 MHz, source PLL with HSE, Vrange1, 1 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_200MHZ_2WS   /* 200 MHz, source PLL with HSE, Vrange1, 2 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_300MHZ_2WS   /* 300 MHz, source PLL with HSE, Vrange1, 2 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_400MHZ_3WS   /* 400 MHz, source PLL with HSE, Vrange1, 3 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG0_480MHZ_4WS   /* 480 MHz, source PLL with HSE, Vrange0, 4 WS, not recommended for long term useage */
//#define USER_CLOCKCONFIG         CLK_HSE_VRNG3_xxMHZ_0WS    /* 8-45MHz, source HSE, VOSrange3, 0 WS, depends on HSE crystal, only available if HSE crystal is mounted */




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
 * Check and set constraints for FMC module
 *****************************************************************************/
#undef USE_FMC
#if USE_FMC_SRAM > 0 || USE_FMC_SDRAM > 0 || USE_FMC_NOR > 0 || USE_FMC_NAND > 0 
    #define USE_FMC 1
#else
    #define USE_FMC 0
#endif

/******************************************************************************
 * Check and set constraints for different hardware types
 *****************************************************************************/
#if defined(STM32H745NUCLEO)
    #undef  HW_HAS_HSE_CRYSTAL
    #define  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       8000000

#elif defined(STM32H742REF)
    #define  HW_HAS_HSE_CRYSTAL
    #undef  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       16000000

#elif defined(STM32H743EVAL2)
    #define  HW_HAS_HSE_CRYSTAL
    #undef  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       25000000

#else
     #error "Missing HSE definitions"
#endif

/* Logging to FAT filesystem requires active FAT FS support */
#if LOGTO_FATFS > 0 && USE_FATFS == 0
    #error "LOGTO_FATFS requires USE_FATFS > 0!"
#endif

/* Debug to console only supported, if console is an U(S)ART */
#if ( DEBUG_FEATURES == 0  || DEBUG_DEBUGIO > 0 ) && LOGTO_CONSOLE > 0
    #error "Console logging only when console output to UART is configured"
#endif

/*
 * FATFS is currently bound to QSPI
 */
#if USE_FATFS > 0 && USE_QSPI == 0
    #error "Must use QSPI for FATFS"
#endif

#define HSE_VALUE HW_HSE_FREQUENCY

/* Can we use an HSE Clock ? */
#if defined(HW_HAS_HSE_CRYSTAL) || defined(HW_HAS_HSE_BYPASS)
    #define HW_HAS_HSE
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
    MK_CONFIGSTR(USE_PWMTIMER,6)
    MK_CONFIGSTR(USE_BASICTIMER,7)
    MK_CONFIGSTR(USE_QSPI,8)
    MK_CONFIGSTR(USE_BASICTIMER_FOR_TICKS,9)
    MK_CONFIGSTR(USE_USB,10)
    MK_CONFIGSTR(USE_USB_FS,11)
    MK_CONFIGSTR(USE_FMC_SRAM,12)
    MK_CONFIGSTR(USE_FMC_SDRAM,13)
    MK_CONFIGSTR(USE_FMC_NOR,14)
    MK_CONFIGSTR(USE_ETH,15)
    MK_CONFIGSTR(USE_FATFS,16)
    MK_CONFIGSTR(USE_BDMA,17)

    #define MAX_CONFIGSTR   17

    const char *ConfigStrings[MAX_CONFIGSTR] = 
        {
            ConfigStr1,  ConfigStr2,  ConfigStr3,  ConfigStr4,  ConfigStr5,  ConfigStr6,  ConfigStr7,  ConfigStr8,
            ConfigStr9,  ConfigStr10, ConfigStr11, ConfigStr12, ConfigStr13, ConfigStr14, ConfigStr15, ConfigStr16,
            ConfigStr17 // , ConfigStr18, ConfigStr19, ConfigStr20, ConfigStr21, ConfigStr22, ConfigStr23, ConfigStr24,
        };
    const uint8_t ConfigValues[MAX_CONFIGSTR] = 
        {
            ConfigVal1,  ConfigVal2,  ConfigVal3,  ConfigVal4,  ConfigVal5,  ConfigVal6,  ConfigVal7,  ConfigVal8,
            ConfigVal9,  ConfigVal10, ConfigVal11, ConfigVal12, ConfigVal13, ConfigVal14, ConfigVal15, ConfigVal16,
            ConfigVal17 // , ConfigVal18, ConfigVal19, ConfigVal20, ConfigVal21, ConfigVal22, ConfigVal23, ConfigVal24,
        };

#if 0
    #define CONFIG_STRING   \
    { \
        PPCAT_E(USE_ONEWIRE),    \
        PPCAT_E(USE_DS18X20),    \
        PPCAT_E(USE_EEPROM_EMUL),      \
        PPCAT_E(USE_QENCODER),  \
        PPCAT_E(USE_SECONDTIMER),  \
        PPCAT_E(USE_PWMTIMER),  \
        PPCAT_E(USE_BASICTIMER),  \
        PPCAT_E(USE_QSPI),  \
        PPCAT_E(USE_BASICTIMER_FOR_TICKS),  \
        PPCAT_E(USE_USB),  \
        PPCAT_E(USE_USB_FS),  \
        PPCAT_E(USE_FMC_SRAM),  \
        PPCAT_E(USE_FMC_SDRAM),  \
        PPCAT_E(USE_FMC_NOR),  \
        PPCAT_E(USE_ETH),  \
        PPCAT_E(USE_FATFS),  \
        PPCAT_E(USE_BDMA),  \
    }
#endif
#endif


#endif /* __CONFIG_H */
