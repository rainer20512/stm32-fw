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
  CLK_MSI_VRNG1_08MHZ_0WS =  0, /*  8 MHz, source MSI, Vrange1, 0 WS */
  CLK_MSI_VRNG2_08MHZ_1WS =  1, /*  8 MHz, source MSI, Vrange2, 1 WS */
  CLK_HSE_VRNG1_08MHZ_0WS =  2, /*  8 MHz, source HSE, Vrange1, 0 WS */
  CLK_HSE_VRNG2_08MHZ_1WS =  3, /*  8 MHz, source HSE, Vrange2, 1 WS */
  CLK_HSI_VRNG1_16MHZ_0WS =  4, /* 16 MHz, source HSI16, Vrange1, 0 WS */
  CLK_HSI_VRNG2_16MHZ_2WS =  5, /* 16 MHz, source HSI16, Vrange2, 2 WS */
  CLK_MSI_VRNG1_16MHZ_0WS =  6, /* 16 MHz, source MSI, Vrange1, 0 WS */
  CLK_MSI_VRNG2_16MHZ_2WS =  7, /* 16 MHz, source MSI, Vrange2, 2 WS */
  CLK_MSI_VRNG1_24MHZ_1WS =  8, /* 24 MHz, source MSI, Vrange1, 1 WS */
  CLK_MSI_VRNG2_24MHZ_3WS =  9, /* 24 MHz, source MSI, Vrange2, 3 WS */
  CLK_MSI_VRNG1_32MHZ_1WS = 10, /* 32 MHz, source MSI, Vrange1, 1 WS */
  CLK_MSI_VRNG1_48MHZ_2WS = 16, /* 48 MHz, source MSI, Vrange1, 2 WS */

  /* 
   * Note: the follwing PLL based clock configuration values MUST be consecutive 
   * and have matching entries in array "pll_clk_rates" in clockconfig.c
   * PLL input clock is either HSE ( if equipped ) or HSI16
   */   
  CLK_PLL_VRNG1_16MHZ_0WS = 18, /* 16 MHz, source PLL with HSI, Vrange1, 0 WS */
  CLK_PLL_VRNG1_24MHZ_1WS = 19, /* 24 MHz, source PLL with HSI, Vrange1, 1 WS */
  CLK_PLL_VRNG1_32MHZ_1WS = 20, /* 32 MHz, source PLL with HSI, Vrange1, 1 WS */
  CLK_PLL_VRNG1_48MHZ_2WS = 21, /* 48 MHz, source PLL with HSI, Vrange1, 2 WS */
  CLK_PLL_VRNG1_64MHZ_3WS = 22, /* 64 MHz, source PLL with HSI, Vrange1, 3 WS */
  CLK_PLL_VRNG1_80MHZ_4WS = 23, /* 80 MHz, source PLL with HSI, Vrange1, 4 WS */

} CLK_CONFIG_T;

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
