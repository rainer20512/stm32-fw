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
#define STM32L476NUCLEO
//#define STM32L476EVAL
//#define STM32L476BAREMETAL

#define HW_HAS_HSE
#define HW_HAS_HSE_BYPASS
#define HW_HSE_FREQ         8000000
/*
 ********************************************************************************
 * Application selection
 ********************************************************************************
 */ 
// #define NOEXTENSION 
#define UNIVERSAL 
//#define TX18LISTENER        

/*
 ********************************************************************************
 * Application tayloring
 ********************************************************************************
 */ 
#define USE_RFM12                   0
#define USE_RFM69                   0           // When RFMxx is installed, always initialize it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BMP085                  0
#define USE_EPAPER                  0
#define USE_ONEWIRE                 0
#define USE_DS18X20                 0
#define USE_EEPROM_EMUL             0
#define USE_QENCODER                0
#define USE_DISPLAY                 0
#define USE_DOGM132                 0
#define USE_SECONDTIMER             1
#define USE_PWMTIMER                0
#define USE_BASICTIMER              1
#define USE_QSPI                    0            // When QSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BASICTIMER_FOR_TICKS    1            // Use Basictimer to generate 1ms Ticks instead of SYSTICK-Timer

#define GENERAL_BAUDRATE    500000
#define CAN_BAUDRATE        500000       // default CAN Baudrate
#define DEBUG_BAUDRATE      38400        // When waking up from stop, the max baudrate is about 45kBaud

#define DEBUG_IRQ_PRIO      0x0f         // Give the debug uart the lowest priority
#define USART_IRQ_PRIO      0x0e         // Give the other uarts the next lowest priority
#define SPI_IRQ_PRIO        0x0d         // Give the SPI the next lowest priority
#define BBSPI_IRQ_PRIO      0x0c         // Give BBSPI the next lowest
#define BUTTON_IRQ_PRIO     0x0c         // Give push buttons the same prio as BBSPI input
#define ADC_IRQ_PRIO        0x08         // ADC also has high prio due to high conversion rates
#define EXTI_IRQ_PRIO       0x06         // Give Pin Change the third highest
#define I2C_IRQ_PRIO        0x05         // Give I2C a high prio
#define CAN_IRQ_PRIO        0x04         // Give CAN a very high prio
#define RTC_IRQ_PRIO        0x03         // RTC has the next higher prio
#define BASETIM_IRQ_PRIO    0x02         // Give the base timer the highest possible priority

#define DEFAULT_STOP_MODE   2

#define FSK_CARRIER_FREQUENCY	433.450

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
 * Check and set constraints for Temperarure, Humidity and Pressure sensors
 ******************************************************************************
 */
#if USE_BMP085 > 0
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
#if USE_BASICTIMER_FOR_TICKS > 0
    #if defined(USE_BASICTIMER) && USE_BASICTIMER < 1
        #warning "USE_BASICTIMER redefined due to system useage"
        #undef USE_BASICTIMER
        #define USE_BASICTIMER 1
    #endif
#endif

#if defined(TX18LISTENER) || DEBUG_PROFILING > 0 || USE_BASICTIMER > 0
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
#if defined(STM32L476EVAL) || defined(DRAGONFLY476)
    #define HW_HAS_LSE_CRYSTAL
    #undef  HW_HAS_LSE_BYPASS
    #define LSE_FREQUENCY   8000000
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

#endif /* __CONFIG_H */
