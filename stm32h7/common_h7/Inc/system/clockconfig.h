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
  CLK_HSI_08MHZ  = 0,   /*  8 MHz, source HSI */ 
  CLK_HSI_16MHZ  = 1,   /* 16 MHz, source HSI */ 
  CLK_HSI_32MHZ  = 2,   /* 32 MHz, source HSI */
  CLK_HSI_64MHZ  = 3,   /* 64 MHz, source HSI */
  CLK_PLL_50MHZ  = 4,   /* 50 MHz, source PLL with HSE */
  CLK_PLL_100MHZ = 5,   /* 100 MHz,source PLL with HSE */
  CLK_PLL_200MHZ = 6,   /* 200 MHz,source PLL with HSE */
  CLK_PLL_300MHZ = 7,   /* 300 MHz,source PLL with HSE */
  CLK_PLL_400MHZ = 8,   /* 300 MHz,source PLL with HSE */
  CLK_PLL_480MHZ = 9,   /* 480 MHz,source PLL with HSE */

#if  defined(STM32H723xx) || defined(STM32H733xx) || defined(STM32H725xx) || defined(STM32H735xx) || defined(STM32H730xx)
  CLK_PLL_500MHZ = 10,  /* 500 MHz,source PLL with HSE */
  CLK_PLL_550MHZ = 11,  /* 550 MHz,source PLL with HSE */
#endif

#if defined(HW_HAS_HSE)
  CLK_HSE_xxMHZ  = 20,  /*  xx MHz, source HSE, Vrange1, 0 WS */
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
void LSIClockConfig(bool bLSIon, bool bUseAsRTCClock);
void HSIClockConfig(bool bHSIon);

uint32_t Get_SysClockFrequency  ( void );
bool     HSIClockCalibrate      ( void );
void     EnableMCO              ( uint32_t mcoSource );

/* Allowed clock sources for SetPeripheralClkSource */
enum {
    CLKP_HSI = 0,
    CLKP_CSI = 1,
    CLKP_HSE = 2,
} ;
void SetPeripheralClkSource( uint32_t src );

#if USE_USB > 0
    void stm32h7_enable_hsi48(void);
    void stm32h7_disable_hsi48(void);
#endif

/* 
 * Registration for notification on clock changes
 * ( In addition to devices, which have their own notification mechanism )
 * Registration must specify a callback fn of type "ClockChangeCB"
 **** C001 ****
 */

typedef void ( *ClockChangeCB ) ( uint32_t  );
int32_t ClockRegisterForClockChange ( ClockChangeCB );


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CLOCKCONFIG_H */
