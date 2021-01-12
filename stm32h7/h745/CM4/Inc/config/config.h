/**
  ******************************************************************************
  * @file    config.h
  * @author  Rainer
  * @brief   global configuration and definitions
  *
  * @note    This is the CM4 config part !!!
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
#define STM32H745NUCLEO

/*
 ********************************************************************************
 * Application selection
 ********************************************************************************
 */ 

#define UNIVERSAL 

/*
 ********************************************************************************
 * Application tayloring
 ********************************************************************************
 */ 
#define USE_ONEWIRE                 0
#define USE_DS18X20                 0
#define USE_EEPROM_EMUL             0
#define USE_QENCODER                0
#define USE_DISPLAY                 0
#define USE_SECONDTIMER             1
#define USE_PWMTIMER                0
#define USE_BASICTIMER              1
#define USE_QSPI                    0            // When QSPI flash is installed, always USE it, otherwise it will consume roundabout 2mA in uninitialized state!
#define USE_BASICTIMER_FOR_TICKS    1            // Use Basictimer to generate 1ms Ticks instead of SYSTICK-Timer
#define USE_ETH                     1

/* Choose one in case of USE_ETH == 1 
 * 
 * Note: As the enc28j60 does not support Checksum calculation for IP packets, lwip must compute these checksums manually when ENC28J60 is selected 
 * so, when USE_ETH_PHY_ENC28J60 = 1, in lwipopts.h the following #define's MUST be set to 1
 *  #define CHECKSUM_GEN_IP                 1
 *  #define CHECKSUM_GEN_UDP                1
 *  #define CHECKSUM_GEN_TCP                1 
 * on the ofher hand, when builtin ETH module is used, these three options MUST be set to 0
 */
#define USE_ETY_PHY_LAN8742         1
#define USE_ETH_PHY_ENC28J60        0


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
#define RTC_IRQ_PRIO        0x03         // RTC has the next higher prio
#define BASETIM_IRQ_PRIO    0x02         // Give the base timer the highest possible priority

#define DEFAULT_STOP_MODE   2

#define FSK_CARRIER_FREQUENCY	433.450


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
    #define USE_TIM6
    #define BASTIM_HANDLE TIM6Handle
    #define BASTIM_HW HW_TIM6
#endif


/******************************************************************************
 * Check and set constraints for differeht hardware types
 *****************************************************************************/
#if defined(STM32H745NUCLEO)
    #undef  HW_HAS_HSE_CRYSTAL
    #define  HW_HAS_HSE_BYPASS
    #define  HW_HSE_FREQUENCY       8000000
#endif

/* Can we use an HSE Clock ? */
#if defined(HW_HAS_HSE_CRYSTAL) || defined(HW_HAS_HSE_BYPASS)
    #define HW_HAS_HSE
#endif


/* Exactly one eth phy defined ? */
#if USE_ETH > 0
    #if USE_ETY_PHY_LAN8742 + USE_ETH_PHY_ENC28J60 == 0 || USE_ETY_PHY_LAN8742 + USE_ETH_PHY_ENC28J60 > 1
        #error "must define exactly one Eth Phy!"
    #endif
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
