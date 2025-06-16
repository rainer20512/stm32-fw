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
//#define BL475IOT   
//#define DRAGONFLY476
//#define STM32L476NUCLEO
//#define STM32L4R9DISCOVERY
//#define STM32L4S9ZXXREF
//#define STM32L4P5BAREMETAL
//#define STM32L476EVAL
#define STM32L476BAREMETAL
//#define T61


/*
 ********************************************************************************
 * Application selection
 ********************************************************************************
 */ 
//#define NOEXTENSION 
#define UNIVERSAL 
//#define MULTITEMP
//#define ENVIRONMENTAL
//#define TX18LISTENER
//#define PWM_DEVICE        


/*
 ********************************************************************************
 * Code Versioning
 ********************************************************************************
 */ 
#define MAJOR_VERSION   1
#define MINOR_VERSION   08

/*
 ********************************************************************************
 * Application tayloring
 ********************************************************************************
 */ 
#define USE_RFM12           0
#define USE_RFM69           0           // When RFMxx is installed, always initialize it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BMP085          0
#define USE_BME280          0
#define USE_CCS811          0
#define USE_EPAPER          0
#define USE_ONEWIRE         0
#define USE_DS18X20         0
#define USE_EEPROM_EMUL     0
#define USE_QENCODER        0
#define USE_DISPLAY         0
#define USE_DOGM132         0
#define USE_SECONDTIMER     1
#define USE_HW_PWMTIMER     1
#define USE_BASICTIMER      1
#define USE_QSPI            0            // When QSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_OSPI1           0            // When OSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_OSPI2           0            
#define USE_CAN             0
#define USE_USB             0  
#define USE_FMC_SRAM        0
#define USE_FMC_NOR         0
#define USE_PERIPHTIMER     1
#define USE_USER_PWMTIMER   0
#define USE_LVGL            1
#define USE_LVGL_LOWLEVEL   1

#define GENERAL_BAUDRATE    500000
#define CAN_BAUDRATE        500000       // default CAN Baudrate
#define DEBUG_BAUDRATE      38400        // When waking up from stop, the max baudrate is about 45kBaud

#define DEBUG_IRQ_PRIO      0x0f         // Give the debug uart the lowest priority
#define USART_IRQ_PRIO      0x0e         // Give the other uarts the next lowest priority
#define SPI_IRQ_PRIO        0x0d         // Give the SPI the next lowest priority
#define BBSPI_IRQ_PRIO      0x0c         // Give BBSPI the next lowest
#define BUTTON_IRQ_PRIO     0x0c         // Give push buttons the same prio as BBSPI input
#define ADC_IRQ_PRIO        0x08         // ADC also has high prio due to high conversion rates
#define EXTI_IRQ_PRIO       0x05         // Give Pin Change the third highest
#define I2C_IRQ_PRIO        0x04         // Give I2C a high prio
#define CAN_IRQ_PRIO        0x03         // Give CAN a very high prio
#define USB_IRQ_PRIO        0x02         // Give USB a very, very high prio
#define RTC_IRQ_PRIO        0x01         // RTC has the next higher prio
#define BASETIM_IRQ_PRIO    0x00         // Give the base timer the highest possible priority

#define DEFAULT_STOP_MODE   2

#define FSK_CARRIER_FREQUENCY	433.450

//#define USER_CLOCKCONFIG         CLK_MSI_VRNG2_08MHZ   /*  8 MHz, source MSI, Vrange2, 1 WS */
//#define USER_CLOCKCONFIG         CLK_HSE_VRNG1         /*  8 MHz, source HSE, Vrange1, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_VRNG1_16MHZ   /* 16 MHz, source HSI16, Vrange1, 0 WS */
//#define USER_CLOCKCONFIG         CLK_HSI_VRNG1_16MHZ   /* 16 MHz, source HSI16, Vrange1, 0 WS */
#define USER_CLOCKCONFIG         CLK_HSI_VRNG2_16MHZ   /* 16 MHz, source HSI16, Vrange2, 2 WS */
//#define USER_CLOCKCONFIG         CLK_MSI_VRNG1_24MHZ   /* 24 MHz, source MSI, Vrange1, 1 WS */
//#define USER_CLOCKCONFIG         CLK_MSI_VRNG1_48MHZ   /* 48 MHz, source MSI, Vrange1, 2 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_48MHZ   /* 48 MHz, source PLL, Vrange1, 2 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_64MHZ   /* 64 MHz, source PLL, Vrange1, 3 WS */
//#define USER_CLOCKCONFIG         CLK_PLL_VRNG1_80MHZ   /* 80 MHz, source PLL, Vrange1, 4 WS */

/******************************************************************************
 * Choose one in case of USE_QSPI == 1 
******************************************************************************
 */
#define USE_XSPI_MX25               1
#define USE_XSPI_MT25Q              0

/******************************************************************************
 * Check and set constraints for Temperarure, Humidity and Pressure sensors
 ******************************************************************************
 */
#if USE_BMP085 > 0 || USE_BME280 > 0 || USE_CCS811 > 0
    #define USE_THPSENSOR   1
#endif


/******************************************************************************
 * Check and set constraints for TX18LISTENER
 ******************************************************************************
 */
#if defined(TX18LISTENER) 

	/* 434.000 for the new TX18 with La Crosse Label on it               */
	#define OOK_CARRIER_FREQUENCY0        434.000
	/* 433,850 for the old Tx18                                          */
	#define OOK_CARRIER_FREQUENCY1        433.850                     
        /* Use 434.000 as default                                            */
        #define TX_OOK_FRQ                    0

        #if USE_RFM12 == 0 && USE_RFM69 == 0
            #error "TX18LISTENER requers useage of any RFM chip"
        #endif

        /* TX18Listener requires BMP085                                      */
        #if USE_THPSENSOR == 0 
            #error "TX18LISTENER requers useage of any pressure sensor"
        #endif


        /* Include Pulse Sequencer for TX18 Listener                         */
        #define USE_PULSE_SEQUENCER          1

        /* Compile OOK-Part of RFM Code                                      */
        #define USE_RFM_OOK                  1

        /* TX18Listener requires second timers                               */
        #if  USE_SECONDTIMER < 1
            #error "TX18LISTENER requires second timers"
        #endif

	/* Must not use DS18X20 module                                       */
	#if USE_DS18X20 > 0
		#error "TX18LISTENER cannot use DS18X20"
	#endif

	/* Must not use LSM303D magnetometer                                 */
	#if USE_LSM303D > 0
		#error "TX18LISTENER cannot use LSM303D"
	#endif

	/* Must not use Optical equipment                                    */
	#if USE_OPTICAL > 0
		#error "TX18LISTENER cannot use OPTICAL"
	#endif
#endif

/******************************************************************************
 * Check and set constraints for BaseTimer
 *****************************************************************************/
#if defined(TX18LISTENER) || DEBUG_PROFILING > 0
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
#if defined(STM32L476EVAL) || defined(DRAGONFLY476) || defined(STM32L4R9DISCOVERY) || defined(STM32L4S9ZXXREF)
    #define HW_HAS_HSE
    #define HW_HAS_HSE_CRYSTAL
    #undef  HW_HAS_HSE_BYPASS
    #if defined(STM32L476EVAL) || defined(DRAGONFLY476)
        #define HSE_FREQUENCY   8000000UL
    #else
        #define HSE_FREQUENCY   16000000UL
    #endif
#endif

#if defined(HW_HAS_HSE_CRYSTAL) || defined(HW_HAS_HSE_BYPASS)
    #define HW_HAS_HSE
    #define HSE_VALUE           HSE_FREQUENCY
#else
    #undef  HW_HAS_HSE
#endif


/******************************************************************************
 * Check and set constraints for FMC module
 *****************************************************************************/
#undef USE_FMC 
#if USE_FMC_SRAM > 0 || USE_FMC_NOR > 0 || USE_FMC_NAND > 0
    #define USE_FMC 1
#else
    #define USE_FMC 0
#endif

/******************************************************************************
 * Check and set constraints for OSPI module
 *****************************************************************************/
#undef USE_OSPI 
#if USE_OSPI1 > 0 || USE_OSPI2 > 0
    #define USE_OSPI 1
#else
    #define USE_OSPI 0
#endif


/******************************************************************************
 * Check and set constraints for MULTITEMP
 *****************************************************************************/
#if defined(MULTITEMP)
    #if !defined(USE_DS18X20) || USE_DS18X20 < 1
        #error "MULTITEMP requires USE_DS18X20"
    #endif
#endif

/******************************************************************************
 * Setup the MCU family
 *****************************************************************************/
#if  defined(STM32H747xx) || defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    #define STM32H7_FAMILY
#elif defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx) || defined(STM32L43xx) || defined(STM32L4Pxxx) || defined(STM32L4Qxxx)
    #define STM32L4_FAMILY
    #if defined(STM32L4Sxxx) || defined(STM32L4Rxxx) || defined(STM32L4Pxxx) || defined(STM32L4Qxxx)
        #define STM32L4PLUS_FAMILY
    #endif
#else
    #error "Unkonwn MCU family!"
#endif


/*
 ********************************************************************************
 * useful macros
 ********************************************************************************
 */
  
#ifndef __cplusplus
    #include <stdbool.h>
#endif


#define MINMAX_UNSET	(int16_t)0x8000     /* Indicates "Min or Max is currently unset" */

#define _CONCAT(a,b)    a##b
#define CONCAT(a,b)     _CONCAT(a,b)

#define _STR(x) #x
#define STR(x) _STR(x)

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#define ABS(a)          (((a) < 0  ) ? -1*(a) : (a) )
#define UCASE(x)        (((x) >= 'a' && (x) <= 'z') ? (x) & ~0x20 : (x) )

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

    MK_CONFIGSTR(USE_RFM12,1)
    MK_CONFIGSTR(USE_RFM69,2)
    MK_CONFIGSTR(USE_BMP085,3)
    MK_CONFIGSTR(USE_BME280,4)
    MK_CONFIGSTR(USE_CCS811,5)
    MK_CONFIGSTR(USE_EPAPER,6)
    MK_CONFIGSTR(USE_ONEWIRE,7)
    MK_CONFIGSTR(USE_DS18X20,8)
    MK_CONFIGSTR(USE_EEPROM_EMUL,9)
    MK_CONFIGSTR(USE_QENCODER,10)

    MK_CONFIGSTR(USE_DISPLAY,11)
    MK_CONFIGSTR(USE_DOGM132,12)
    MK_CONFIGSTR(USE_SECONDTIMER,13)
    MK_CONFIGSTR(USE_HW_PWMTIMER,14)
    MK_CONFIGSTR(USE_BASICTIMER,15)
    MK_CONFIGSTR(USE_QSPI,16)
    MK_CONFIGSTR(USE_OSPI1,17)
    MK_CONFIGSTR(USE_OSPI2,18)
    MK_CONFIGSTR(USE_CAN,19)
    MK_CONFIGSTR(USE_USB,20)
    MK_CONFIGSTR(USE_FMC_SRAM,21)

    MK_CONFIGSTR(USE_FMC_NOR,22)
    MK_CONFIGSTR(USE_PERIPHTIMER,23)
    MK_CONFIGSTR(USE_USER_PWMTIMER,24)
    MK_CONFIGSTR(USE_LVGL,25)
    MK_CONFIGSTR(USE_LVGL_LOWLEVEL,26)

    
    #define MAX_CONFIGSTR   25

    const char *ConfigStrings[MAX_CONFIGSTR] = 
        {
            ConfigStr1,  ConfigStr2,  ConfigStr3,  ConfigStr4,  ConfigStr5,  ConfigStr6,  ConfigStr7,  ConfigStr8,
            ConfigStr9,  ConfigStr10, ConfigStr11, ConfigStr12, ConfigStr13, ConfigStr14, ConfigStr15, ConfigStr16,
            ConfigStr17, ConfigStr18, ConfigStr19, ConfigStr20, ConfigStr21, ConfigStr22, ConfigStr23, ConfigStr24,
            ConfigStr25, ConfigStr26, // ConfigStr27, ConfigStr28, ConfigStr29, ConfigStr30, ConfigStr31, ConfigStr32,
        };
    const uint8_t ConfigValues[MAX_CONFIGSTR] = 
        {
            ConfigVal1,  ConfigVal2,  ConfigVal3,  ConfigVal4,  ConfigVal5,  ConfigVal6,  ConfigVal7,  ConfigVal8,
            ConfigVal9,  ConfigVal10, ConfigVal11, ConfigVal12, ConfigVal13, ConfigVal14, ConfigVal15, ConfigVal16,
            ConfigVal17, ConfigVal18, ConfigVal19, ConfigVal20, ConfigVal21, ConfigVal22, ConfigVal23, ConfigVal24,
            ConfigVal25, ConfigVal26, // ConfigVal27, ConfigVal28, ConfigVal29, ConfigVal30, ConfigVal31, ConfigVal32,
        };

#endif

#endif /* __CONFIG_H */
