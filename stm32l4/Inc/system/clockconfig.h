/**
  ******************************************************************************
  * @file    clockconfig.h
  * @author  Rainer
  * @brief   Several Clock Configurations with different sources, SYSCLK 
  *          frequencies and correct Vcore settings and NVM Wait states
  *
  *          Available configurations
  *          1.  MSI-Clock based with parameterized clk value between 65kHz
  *              and 4 MHz, Vcore       Range 3, 0 WS
  *          2.  HSE Bypass mode, 8MHz, Range 2, 0 WS
  *              ( this is specially for Nucleo-Boards ) 
  *          3a. HSI based, 16MHz,      Range 2, 1 WS
  *          3b. HSI based, 16MHz,      Range 1, 0 WS
  *          4.  PLL/HSI based, 32MHz,  Range 1, 1 WS
  *         
  * @note    In every configuration there are only identical clock values for
  *          all types of sore clocks ( SYSCLK, HCLK, APB1 and APB2 CLK ) 
  *
  ******************************************************************************
  * Coarse Performance estimation
  * MSI  2MHZ Clocks, Scale 3, 0WS :  63700 Increments per second
  * MSI  4MHZ Clocks, Scale 3, 0WS : 129200 Increments per second
  * HSE  8MHZ Clocks, Scale 2, 0WS : 248100 Increments per second
  * HSI 16MHZ Clocks, Scale 2, 1WS : 430300 Increments per second
  * HSI 16MHZ Clocks, Scale 1, 0WS : 498000 Increments per second
  * PLL 32MHZ Clocks, Scale 1, 0WS : 862200 Increments per second
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCKCONFIG_H
#define __CLOCKCONFIG_H

#include <stdio.h>

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/*
 *****************************************************************************
 *****************************************************************************
 */

/*
 ******************************************************************************
 Define some common clock configurations, so all the neccessary clock settings
 can be done by configuration byte
 When USB is used, all configs w/o PLL, except MSI48 are not allowed
 ******************************************************************************
 */
typedef enum {
/* 
 * The first block is common to all L4 or L4+ derivates
 *
 *                              Frq    clk src  Vcore    WS L4    WS L4+
 *                              -------------------------------------------  */
  CLK_MSI_VRNG2_02MHZ =  0, /*  2 MHz, MSI,     Vrange4, 0        0          */
  CLK_MSI_VRNG2_04MHZ =  1, /*  4 MHz, MSI,     Vrange4, 0        0          */
  CLK_MSI_VRNG1_08MHZ =  2, /*  8 MHz, MSI,     Vrange1, 0        0          */
  CLK_MSI_VRNG2_08MHZ =  3, /*  8 MHz, MSI,     Vrange2, 1        0          */
  CLK_HSE_VRNG1       =  4, /*  HSE,   HSE,     Vrange1, 0        0          */
  CLK_HSE_VRNG2       =  5, /*  HSE,   HSE,     Vrange2, depends from HSE    */
  CLK_HSI_VRNG1_16MHZ =  6, /* 16 MHz, HSI16,   Vrange1, 0        0          */
  CLK_HSI_VRNG2_16MHZ =  7, /* 16 MHz, HSI16,   Vrange2, 2        1          */
  CLK_MSI_VRNG1_16MHZ =  8, /* 16 MHz, MSI,     Vrange1, 0        0          */
  CLK_MSI_VRNG2_16MHZ =  9, /* 16 MHz, MSI,     Vrange2, 2        1          */
  CLK_MSI_VRNG1_24MHZ = 10, /* 24 MHz, MSI,     Vrange1, 1        0          */
  CLK_MSI_VRNG2_24MHZ = 11, /* 24 MHz, MSI,     Vrange2, 3        2          */
  CLK_MSI_VRNG1_32MHZ = 12, /* 32 MHz, MSI,     Vrange1, 1        1          */
  CLK_MSI_VRNG1_48MHZ = 13, /* 48 MHz, MSI,     Vrange1, 2        2          */

/* 
 * The second block is specific for certain L4 or L4+ derivates, Clock source is
 * always PLL, Vcore is always Range1 and PLL input is HSE, when equipped,
 * otherwise HSI16
 *
 * Note: the following clock configuration enum values MUST be consecutive 
 *

 * PLL input clock is either HSE ( if equipped ) or HSI16
 *                              Frq    clk src      WS L4    WS L4+
 *                              --------------------------------------  */
  /* 
   */   
  CLK_PLL_VRNG1_16MHZ  = 18, /* 16 MHz,  HSE/HSI16, 0        0           */
  CLK_PLL_VRNG1_24MHZ  = 19, /* 24 MHz,  HSE/HSI16, 1        1           */
  CLK_PLL_VRNG1_32MHZ  = 20, /* 32 MHz,  HSE/HSI16, 1        1           */
  CLK_PLL_VRNG1_48MHZ  = 21, /* 48 MHz,  HSE/HSI16, 2        2           */
  CLK_PLL_VRNG1_64MHZ  = 22, /* 64 MHz,  HSE/HSI16, 3        3           */
  CLK_PLL_VRNG1_80MHZ  = 23, /* 80 MHz,  HSE/HSI16, 4        3           */
  CLK_PLL_VRNG1_100MHZ = 24, /* 100 MHz, HSE/HSI16, n/a      4           */
  CLK_PLL_VRNG1_120MHZ = 25, /* 120 MHz, HSE/HSI16, n/a      5           */
} CLK_CONFIG_T;

#if defined(STM32L476xx) || defined(STM32L496xx)
    #define ALLOWED_PLL_CLOCKS  {16,24,32,48,64,80,}
#elif defined(STM32L4Sxxx)
    #define ALLOWED_PLL_CLOCKS  {16,24,32,48,64,80,100, 120}
#else
    #error "No clock configuration table for selected MCU"
#endif
bool ClockMustReconfiguredAfterStop ( void );
void ClockReconfigureAfterStop(void);
void SystemClock_SetConfiguredClock(void);
void LSEClockConfig(bool bLSEon, bool bUseAsRTCClock);
void HSIClockConfig(bool bHSIon);

uint32_t Get_SysClockFrequency  ( void );
bool     HSIClockCalibrate      ( void );
void     EnableMCO              ( uint32_t mcoSource );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CLOCKCONFIG_H */
