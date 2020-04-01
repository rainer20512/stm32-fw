/**
  ******************************************************************************
  * @file    clockconfig.h
  * @author  Rainer
  * @brief   Several Clock Configurations with different sources, SYSCLK 
  *          frequencies and correct Vcore settings and NVM Wait states
  *
  *          Available configurations
  *          1.  HSI-Clock based with 8,16,32,64 MHZ
  *              VOS range3        0 WS ( 8,16,32 ) 1 WS(64)
  *          2.  HSE configurable, Range 3, 0 WS
  *              ( this is specially for Nucleo-Boards ) 
  *          4.  PLL/HSI or HSE based, 
  *              64-480MHz,  Range 0 or 1, 0 .. 5 WS
  *         
  * @note    As long as SYSCLK is below 120MHz, sysclk and all perpheral clocks are equal
  *          As long as SYSCLK is below 240MHz, sysclk and AHB clock are equal,
  *             ABP clocks are Sysclk/2
  *          When SYSCLK > 240MHz, AHB clocks are SYSCLK/2, APB clocks are SYSCLK/4
  *
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
 Note: Do not use PLL with HSI as input at higher frequencies, this will lead
       to unpredictable results / hanging system
 ******************************************************************************
 */
typedef enum {
  CLK_HSI_VRNG3_08MHZ_0WS =  0, /*  8 MHz, source HSI, VOSrange3, 0 WS */
  CLK_HSI_VRNG3_16MHZ_0WS,      /* 16 MHz, source HSI, VOSrange3, 0 WS */
  CLK_HSI_VRNG3_32MHZ_0WS,      /* 32 MHz, source HSI, VOSrange3, 0 WS */
  CLK_HSI_VRNG3_64MHZ_1WS,      /* 64 MHz, source HSI, VOSrange3, 1 WS */
  CLK_PLL_VRNG1_100MHZ_1WS,     /* 100 MHz,source PLL with HSE, VOSrange1, 1 WS */
  CLK_PLL_VRNG1_200MHZ_2WS,     /* 200 MHz,source PLL with HSE, VOSrange1, 2 WS */
  CLK_PLL_VRNG1_300MHZ_2WS,     /* 300 MHz,source PLL with HSE, VOSrange1, 2 WS */
  CLK_PLL_VRNG1_400MHZ_3WS,     /* 300 MHz,source PLL with HSE, VOSrange1, 3 WS */
  CLK_PLL_VRNG0_480MHZ_4WS,     /* 480 MHz,source PLL with HSE, VOSrange0, 4 WS */
#if defined(HW_HAS_HSE)
  CLK_HSE_VRNG3_xxMHZ_0WS,      /*  xx MHz, source HSE, Vrange1, 0 WS */
#endif

} CLK_CONFIG_T;

bool ClockMustReconfiguredAfterStop ( void );
void ClockReconfigureAfterStop(void);

/* Only use "SystemClock_SetConfiguredClock" to change system clock */
/* THis will ensure, that all devices will be noticed about clock   */
/* changes                                                          */
/*
void SystemClock_MSI_Vrange_2(uint32_t msi_range);
void SystemClock_MSI_Vrange_1(uint32_t msi_range);
void SystemClock_HSE_8MHz_Vrange_2_1WS(bool bSwitchOffMSI);
void SystemClock_HSE_8MHz_Vrange_1_0WS(bool bSwitchOffMSI);
void SystemClock_HSI_16MHz_Vrange_2_2WS(bool bSwitchOffMSI);
void SystemClock_HSI_16MHz_Vrange_1_0WS(bool bSwitchOffMSI);
void SystemClock_PLL_xxMHz_Vrange_1(uint32_t xxmhz, bool bSwitchOffMSI);
void SystemClock_Set(CLK_CONFIG_T clk_config_byte, bool bSwitchOffMSI );
*/

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
