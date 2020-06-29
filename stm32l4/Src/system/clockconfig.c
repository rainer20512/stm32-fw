/**
  ******************************************************************************
  * @file    clockconfig.c
  * @author  Rainer
  * @brief   Several Clock Configurations with different sources, SYSCLK 
  *          frequencies and correct Vcore settings and NVM Wait states
  *
  *          Available configurations
  *          1a. MSI-Clock based with parameterized clk value between 100kHz
  *              and 24 MHz,                Vcore Range 2, 0-3 WS
  *          1b. MSI-Clock based with parameterized clk value between 100kHz
  *              and 48 MHz,                Vcore Range 1, 0-2 WS
  *          2.  HSE Bypass mode, 8MHz,     Range 2, 0 WS
  *              ( this is specially for Nucleo-Boards ) 
  *          3a. HSI based, 16MHz,          Range 2, 1 WS
  *          3b. HSI based, 16MHz,          Range 1, 0 WS
  *          4.  PLL/HSI based, 32-80 MHz,  Range 1, 1-4 WS
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


/** @addtogroup CLOCK_CONFIG
  * @{
  */

#include "stm32l4xx_hal.h"
#include "config/config.h"
#include "dev/devices.h"
#include "eeprom.h"
#include "system/clockconfig.h"
#include "system/timer_handler.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#if DEBUG_PROFILING > 0
    #include "system/profiling.h"
#endif

/* Can we use an LSE Clock */
#if defined(HW_HAS_LSE_CRYSTAL) || defined(HW_HAS_LSE_BYPASS)
    #define HW_HAS_LSE
#endif

/* set the Output frq. of the PLL's M-stage to a fixed value when PLL is used */
/* This is not mandatory, but alleviates other setup                          */
#define PLL_BASE_FRQ      8000000

/*
 *************************************************************************************
 * As in Stop2 mode the wakeup clock is either MSI or HSI, we have to safe 
 * all neccessary parameters to be able to restore the clock settings when
 * normal clock source is not HSI16 or MSI.
 * When normal clock source is HSI16 or MSI, the same clock is selected as wakeup
 * clock source and we can continue as before sleep without any reconfiguration
 * All the following variables are set in "System_Clock_Set"
 *************************************************************************************/

static uint32_t saved_mhz;                 /* The last selected PLL frequency        */
static bool saved_bSwitchOffMSI;           /* the last selected bSwitchOffMSI value  */
static void (*RestoreFn)( uint32_t, bool); /* the restore function to call           */
static bool bClockSettingVolatile;         /* true, if the clock settings will be    */
                                           /* lost in Stop mode                      */


/******************************************************************************
 * returns true, iff the clock configuration is volatile in STOP mode
 *****************************************************************************/
bool ClockMustReconfiguredAfterStop ( void )
{
    return bClockSettingVolatile;
}

/******************************************************************************
 * Call this function to reconfigure clock after STOP mode
 *****************************************************************************/
void ClockReconfigureAfterStop(void)
{
    /* Make sure, clock setting is volatile and restoreFn is set */
    assert(bClockSettingVolatile);
    assert(RestoreFn);
    RestoreFn( saved_mhz, saved_bSwitchOffMSI );
}

/*
 *******************************************************************************
 Flash wait states in dependency of Vcore and HCLK
 Wait states                            HCLK (MHz) 
 (WS)(LATENCY)               VCORE Range 1(1)   VCORE Range 2(2)
     0 WS (1 CPU cycles)         = 16                = 6
     1 WS (2 CPU cycles)         = 32                = 12
     2 WS (3 CPU cycles)         = 48                = 18
     3 WS (4 CPU cycles)         = 64                = 26
     4 WS (5 CPU cycles)         = 80                = 26
 maximum HCLK at Vcore Range 2 is 26 MHz!
 *******************************************************************************
 */
static uint32_t SystemClock_GetFlashLatency( uint8_t vrange, uint32_t khz )
{  
  uint32_t flash_latency;
  switch ( vrange ) {
    case 1:
          if ( khz > 80000 ) {
              Error_Handler(__FILE__, __LINE__); 
              flash_latency = FLASH_LATENCY_4;
          }
          if      ( khz > 64000 ) flash_latency = FLASH_LATENCY_4;
          else if ( khz > 48000 ) flash_latency = FLASH_LATENCY_3;
          else if ( khz > 32000 ) flash_latency = FLASH_LATENCY_2;
          else if ( khz > 16000 ) flash_latency = FLASH_LATENCY_1;
          else                    flash_latency = FLASH_LATENCY_0;
        break;
    case 2:
          if ( khz > 26000 ) {
              Error_Handler(__FILE__, __LINE__); 
              flash_latency = FLASH_LATENCY_4;
          }
          if      ( khz > 18000 ) flash_latency = FLASH_LATENCY_3;
          else if ( khz > 12000 ) flash_latency = FLASH_LATENCY_2;
          else if ( khz > 6000  ) flash_latency = FLASH_LATENCY_1;
          else                    flash_latency = FLASH_LATENCY_0;
        break;
     default:
        Error_Handler(__FILE__, __LINE__); 
        flash_latency = FLASH_LATENCY_4;
  }

  return flash_latency;
}

#if USE_USB > 0
    static inline __attribute__((always_inline)) void SC_SetUsbClockSource(uint32_t source )
    {
        MODIFY_REG(RCC->CCIPR, RCC_CCIPR_CLK48SEL_Msk, source );
    }
#endif

/* Public functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration on MSI-Base
  *         The system Clock is configured as follow : 
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 100kHz ... 48MHz, depending from range
  *            HCLK(Hz)                       = 100kHz ... 48MHz, depending from range
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = depending from SYSCLK
  *            Main regulator output voltage  = Range 2, preferred, Range 1 when SYSCLK > 26 Mhz
  * @param range - One of RCC_MSIRANGE0 ... RCC_MSIRANGE_11
  *            RCC_MSIRANGE_0: range 0 around 100 kHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_1: range 1 around 200 kHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_2: range 2 around 400 kHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_3: range 3 around 800 kHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_4: range 4 around 1 MHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_5: range 5 around 2 MHz, 0 WS, Vcore Range 2  
  *            RCC_MSIRANGE_6: range 6 around 4 MHz, 0 WS, Vcore Range 2   (reset value)
  *            RCC_MSIRANGE_7: range 6 around 8 MHz, 1 WS, Vcore Range 2  
  *            RCC_MSIRANGE_8: range 6 around 16 MHz, 2 WS, Vcore Range 2  
  *            RCC_MSIRANGE_9: range 6 around 24 MHz, 3 WS, Vcore Range 2   
  *            RCC_MSIRANGE_10: range 6 around 32 MHz, 1 WS, Vcore Range 1, NOT ALLOWED HERE
  *            RCC_MSIRANGE_11: range 6 around 48 MHz, 2 WS  Vcore Range 1, NOT ALLOWED HERE
  * @retval None
  */
static void SystemClock_MSI_Vrange_2(uint32_t msi_range)
{
  uint32_t flash_latency=FLASH_LATENCY_0;

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = msi_range;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  switch ( msi_range ) {
    case RCC_MSIRANGE_7: flash_latency=FLASH_LATENCY_1; break;
    case RCC_MSIRANGE_8: flash_latency=FLASH_LATENCY_2; break;
    case RCC_MSIRANGE_9: flash_latency=FLASH_LATENCY_3; break;
    case RCC_MSIRANGE_10:
    case RCC_MSIRANGE_11:
      Error_Handler(__FILE__, __LINE__);
      break;
  }
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
  
  /* Select MSI as WakeUp clock */
  CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);

}

/**
  * @brief  System Clock Configuration on MSI-Base
  *         The system Clock is configured as follow : 
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 100kHz ... 48MHz, depending from range
  *            HCLK(Hz)                       = 100kHz ... 48MHz, depending from range
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = depending from SYSCLK
  *            Main regulator output voltage  = always Range 1
  * @param range - One of RCC_MSIRANGE0 ... RCC_MSIRANGE_11
  *            RCC_MSIRANGE_0: range 0 around 100 kHz, 0 WS
  *            RCC_MSIRANGE_1: range 1 around 200 kHz, 0 WS
  *            RCC_MSIRANGE_2: range 2 around 400 kHz, 0 WS
  *            RCC_MSIRANGE_3: range 3 around 800 kHz, 0 WS
  *            RCC_MSIRANGE_4: range 4 around 1 MHz, 0 WS
  *            RCC_MSIRANGE_5: range 5 around 2 MHz, 0 WS
  *            RCC_MSIRANGE_6: range 6 around 4 MHz, 0 WS  (reset value)
  *            RCC_MSIRANGE_7: range 7 around 8 MHz, 0 WS
  *            RCC_MSIRANGE_8: range 8 around 16 MHz, 0 WS
  *            RCC_MSIRANGE_9: range 9 around 24 MHz, 1 WS
  *            RCC_MSIRANGE_10: range 10 around 32 MHz, 1 WS
  *            RCC_MSIRANGE_11: range 11 around 48 MHz, 2 WS
  * @retval None
  */
static void SystemClock_MSI_Vrange_1(uint32_t msi_range)
{
  uint32_t flash_latency;

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable Power Control clock and switch to Range 1 before doing any other things */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = msi_range;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select MSI as WakeUp clock */
  CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);

  flash_latency=FLASH_LATENCY_0;

  switch ( msi_range ) {
    case RCC_MSIRANGE_9: 
    case RCC_MSIRANGE_10: 
        flash_latency=FLASH_LATENCY_1; break;
    case RCC_MSIRANGE_11: 
        flash_latency=FLASH_LATENCY_2; break;
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }


  #if USE_USB > 0
      /* Set USB clock also to MSI 48 MHz */
      if ( msi_range == RCC_MSIRANGE_11 ) {
          SC_SetUsbClockSource(RCC_USBCLKSOURCE_MSI);
      }
  #endif
}


/******************************************************************************
 * Switch MSI clock off
 *****************************************************************************/
static void SwitchOffMSI(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
    {
      /* Initialization Error */
      while(1); 
    }
}

/* HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- 
 * The following #ifdef-block requires an HSE oscillator being functional, either
 * as original crystal oscillator or being supplied as external clock
 * HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --*/

 #if defined(HW_HAS_LSE)
    /******************************************************************************
     * Switch HSE clock on
     *****************************************************************************/
    static void SwitchOnHSE(void)
    {
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  
      /* Enable HSE Oscillator */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      #if defined(HW_HAS_LSE_CRYSTAL)
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
      #elif defined(HW_HAS_LSE_BYPASS)
        RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
      #else
        #error "Undefined LSE oscillator type"
      #endif
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
      {
        /* Initialization Error */
        while(1); 
      }
    }
    /*
     * The initialization part that has to be restored after wakeup from stop 
     * only available, if an HSE oscillator is euqipped
     */
    static void SystemClock_HSE_8MHz_Vrange_2_1WS_restore ( uint32_t xxmhz, bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      UNUSED(xxmhz);
      SwitchOnHSE();

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
      }
    }

    /**
      * @brief  System Clock Configuration on 8MHz Clock input
      *         The system Clock is configured as follow : 
      *            System Clock source            = 8 MHz Signal at OSC_IN pin HSE Bypass mode 
      *            SYSCLK(Hz)                     = 8MHz
      *            HCLK(Hz)                       = 8MhZ
      *            AHB Prescaler                  = 1
      *            APB1 Prescaler                 = 1
      *            APB2 Prescaler                 = 1
      *            Flash Latency(WS)              = 1
      *            Main regulator output voltage  = Scale2 mode
      * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
      *         MUST be activated again, BEFORE going to sleep
      * @note the WakeUp Clock will be MSI, so user has to take actions to switch off to LSE after wakeup
      * @note only available, if HSE oscillator is euqipped
      * @retval None
      */

    static void SystemClock_HSE_8MHz_Vrange_2_1WS(bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      SwitchOnHSE();
      /* Select HSE as system clock source and configure the HCLK, PCLK1 and PCLK2 
         clocks dividers */
      RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
      {
        /* Initialization Error */
        while(1); 
      }

      /* Enable Power Control clock */
      __HAL_RCC_PWR_CLK_ENABLE();
  
      /* Select Voltage Scale2  */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
  
      /* Disable Power Control clock */
      __HAL_RCC_PWR_CLK_DISABLE();

      /* Disable MSI Oscillator, if desired */
      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
        /* HSI16 as Wakeup clock */
        SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      } else {
        /* MSI as Wakeup clock */
        CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      }
    }

    /*
     * The initialization part that has to be restored after wakeup from stop 
     * only available, if an HSE oscillator is euqipped
     */
    static void SystemClock_HSE_8MHz_Vrange_1_0WS_restore ( uint32_t xxmhz, bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      UNUSED(xxmhz);
      SwitchOnHSE();

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
      }
    }

    /**
      * @brief  System Clock Configuration on 8MHz Clock input
      *         The system Clock is configured as follow : 
      *            System Clock source            = 8 MHz Signal at OSC_IN pin HSE Bypass mode 
      *            SYSCLK(Hz)                     = 8MHz
      *            HCLK(Hz)                       = 8MhZ
      *            AHB Prescaler                  = 1
      *            APB1 Prescaler                 = 1
      *            APB2 Prescaler                 = 1
      *            Flash Latency(WS)              = 0
      *            Main regulator output voltage  = Scale1 mode
      * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
      *         MUST be activated again, BEFORE going to sleep
      * @note only available, if HSE oscillator is euqipped
      * @retval None
      */
    static void SystemClock_HSE_8MHz_Vrange_1_0WS(bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
      SwitchOnHSE();
  
      /* Enable Power Control clock */
      __HAL_RCC_PWR_CLK_ENABLE();
  
      /* Select Voltage Scale1  */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  
      /* Disable Power Control clock */
      __HAL_RCC_PWR_CLK_DISABLE();

      /* Select HSE as system clock source and configure the HCLK, PCLK1 and PCLK2 
         clocks dividers */
      RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)!= HAL_OK)
      {
        /* Initialization Error */
        while(1); 
      }

      /* Disable MSI Oscillator, if desired */
      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
        /* HSI16 as Wakeup clock */
        SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      } else {
        /* MSI as Wakeup clock */
        CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      }
    }
    /**
      * @brief  Switch on LSE an select as system clock 
      *
      * @note Caller must assure, that core voltage and wait srtats are selected properly before!
      * 
      */
#endif

/**
  * @brief  System Clock Configuration on 16MhZ HSI with one NVM-Waitstate and PowerScale 2
  *         The system Clock is configured as follow : 
  *            System Clock source            = 16MHz HSI RC-oscillator
  *            SYSCLK(Hz)                     = 16MHz
  *            HCLK(Hz)                       = 16MhZ
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 2
  *            Main regulator output voltage  = Scale2 mode
  *
  *         Alternatively, Voltage can be switched to Scale1, after that flash can operate with 0 WS
  * 
  * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
  *         MUST be activated again, BEFORE going to sleep
  * @retval None
  */

static void SystemClock_HSI_16MHz_Vrange_2_2WS(bool bSwitchOffMSI)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable HSI RC-Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  
  /* Select HSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Select Voltage Scale2  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Disable MSI Oscillator, if desired */
  if ( bSwitchOffMSI ) {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
    {
      /* Initialization Error */
      while(1); 
    }
  }
  /* Select HSI16 also as WakeUp clock */
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);

}

/**
  * @brief  Switch on HSI an select as system clock 
  *
  * @note Caller must assure, that core voltage and wait srtats are selected properly before!
  * 
  * allowed are - Vrange 1, min 0 flash WS
  *             - Vrange 2, min 2 flash WS
  *
  */
static void SC_SwitchOnHSI(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* Enable HSI RC-Oscillator */
    RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState              = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT; 
    RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
    {
        /* Initialization Error */
        while(1); 
    }

    /* Select HSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
    RCC_ClkInitStruct.ClockType             = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource          = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider         = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider        = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider        = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)!= HAL_OK)
    {
        /* Initialization Error */
        while(1); 
    }
}

/**
  * @brief  System Clock Configuration on 16MhZ HSI with no NVM-Waitstates and Power Scale 1
  *         The system Clock is configured as follow : 
  *            System Clock source            = 16MHz HSI RC-oscillator
  *            SYSCLK(Hz)                     = 16MHz
  *            HCLK(Hz)                       = 16MhZ
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale1 mode
  *
  *         Alternatively, Voltage can be switched to Scale1, after that flash can operate with 0 WS
  * 
  * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
  *         MUST be activated again, BEFORE going to sleep
  * @retval None
  */

static void SystemClock_HSI_16MHz_Vrange_1_0WS(bool bSwitchOffMSI)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Select Voltage Scale1  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  SC_SwitchOnHSI();

  /* Disable MSI Oscillator, if desired */
  if ( bSwitchOffMSI ) {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
    {
      /* Initialization Error */
      while(1); 
    }
  }
  /* Select HSI16 also as WakeUp clock */
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
}

#if USE_USB > 0
    /**************************************************************************
     * PLL clock has been set up as system clock. During setup, output of the
     * PLL's M-divider has been set to  PLL_BASE_FRQ
     *************************************************************************/
    static void SetupUSB48Clk(void)
    {
    }
#endif

/******************************************************************************
 * configure PLL on HSI16 bas to xxmhz Mhz and switch on PLL
 *****************************************************************************/
static void SwitchOnPLL ( uint32_t xxmhz )
{
  uint32_t pllsrc;
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* 
   * if PLL is already selected as system clock source, 
   * it must be disabled before reconfiguration 
   */
  if (  __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL ) {
    /* if so, select HSI as temporary system clock source       */
    /* Vrange and Wait states have been set by caller */
    SC_SwitchOnHSI();
  }

  #if USE_USB > 0
      CLEAR_BIT(RCC->CR, RCC_CR_PLLSAI1ON_Msk);
  #endif

  /* if HSE oscillator is equipped, use as PLL input, otherwise use HSI16 */
  #if defined(PLL_INP_FRQ) 
    #undef PLL_INP_FRQ
  #endif

  #if defined(HW_HAS_LSE)
    SwitchOnHSE();
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    pllsrc = RCC_PLLSOURCE_HSE;
    #define PLL_INP_FRQ  LSE_FREQUENCY
  #else
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; 
      pllsrc = RCC_PLLSOURCE_HSI;
      #define PLL_INP_FRQ 16000000
  #endif

  /* 
   * PLLN is computed so, that the output after M-Divider is at 8MHz.
   * Check, whether this can be achieved
   */
  #define PLL_DIVM          (PLL_INP_FRQ/PLL_BASE_FRQ) 
  #if PLL_DIVM * PLL_BASE_FRQ != PLL_INP_FRQ
    #error "PLLM is not an integer value - cannot set PLLM"
  #endif  
  /* Enable  PLL */

  RCC_OscInitStruct.PLL.PLLSource = pllsrc;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;   
  RCC_OscInitStruct.PLL.PLLM = PLL_DIVM;
  RCC_OscInitStruct.PLL.PLLN = xxmhz / 2 ;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLP = 17;
  RCC_OscInitStruct.PLL.PLLQ = 8;
   
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  #if USE_USB > 0
    /* PLLSAI1 Q output to 48 MHz */
    RCC->PLLSAI1CFGR = 
        24 << RCC_PLLSAI1CFGR_PLLSAI1N_Pos |       /* PLLSAI1N = 24          */
      0b01 << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos |       /* PLLSAI1Q = 4           */
      0b11 << RCC_PLLSAI1CFGR_PLLSAI1R_Pos |       /* PLLSAI1R = 8,  unused  */
         1 << RCC_PLLSAI1CFGR_PLLSAI1P_Pos |       /* PLLSAI1P = 17, unused  */
         1 << RCC_PLLSAI1CFGR_PLLSAI1QEN_Pos ;     /* Enable Q output        */
    
    /* Switch PLLSAI1 on and wait for ready */
    SET_BIT(RCC->CR, RCC_CR_PLLSAI1ON_Msk );
    while ( (RCC->CR & RCC_CR_PLLSAI1RDY_Msk ) == 0 );

    SC_SetUsbClockSource(RCC_USBCLKSOURCE_PLLSAI1);
  #endif
}

/*
 * The initialization part that has to be restored after wakeup from stop 
 */
static void SystemClock_PLL_xxMHz_Vrange_1_restore (uint32_t xxmhz, bool bSwitchOffMSI)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
  uint32_t flash_latency = SystemClock_GetFlashLatency(1, xxmhz * 1000);
  SwitchOnPLL(xxmhz);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency );

  if ( bSwitchOffMSI ) {
    SwitchOffMSI();
  }
}

/**
  * @brief  System Clock Configuration on 32MhZ PLL-Clock, basing on 16MhZ HSI
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL Clock
  *            PLL input                      = 16MHz HSI RC-oscillator
  *            SYSCLK(Hz)                     = as desired, between 16 and 80 MHz
  *            HCLK(Hz)                       = as desired, between 16 and 80 MHz
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = depending from xxmhz, between 0 and 4
  *            Main regulator output voltage  = Scale1 mode
  *
  * 
  * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
  *         MUST be activated again, BEFORE going to sleep
  * @retval None
  */

static void SystemClock_PLL_xxMHz_Vrange_1(uint32_t xxmhz, bool bSwitchOffMSI)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  uint32_t flash_latency = SystemClock_GetFlashLatency(1, xxmhz * 1000);

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Select Voltage Scale1  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  SwitchOnPLL(xxmhz);
  
  /* Select Pll as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency )!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  #if USE_USB > 0
    SetupUSB48Clk();
  #endif

  /* Disable MSI Oscillator, if desired */
  if ( bSwitchOffMSI ) {
    SwitchOffMSI();
  }

  /* Select HSI16 also as WakeUp clock */
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
}

static const uint16_t pll_clk_rates[] = {16,24,32,48,64,80};
/*******************************************************************************
 * Return the clock spped in MHz for a given CLK_CONFIG_T with PLL useage
 *******************************************************************************/
static uint16_t GetPLLClockRate( CLK_CONFIG_T clk_config_byte )
{
    if ( clk_config_byte < CLK_PLL_VRNG1_16MHZ_0WS || clk_config_byte > CLK_PLL_VRNG1_80MHZ_4WS ) 
        return 0;
    else   
        return pll_clk_rates[clk_config_byte - CLK_PLL_VRNG1_16MHZ_0WS];
}



/* Only use "SystemClock_SetConfiguredClock" to change system clock            */
/* THis will ensure, that all devices will be noticed about clock              */
/* changes                                                                     */
/*******************************************************************************
 * Do all the clock setting on a configuration byte basis
 *******************************************************************************/
void SystemClock_Set(CLK_CONFIG_T clk_config_byte, bool bSwitchOffMSI )
{
    /* 
     * When using USB, we need a 48MHz clock. This is generated from PLLSAI1
     * in any case. So we only allow clock settings, which are derived from PLL
     * This is not mandatory, but easier to handle
     * Only if the System clock is MSI48, we use this as USB clock, too
     */
    #if USE_USB > 0
        /* 
         * any clocksetting without PLL and != MSI48 will be changed to
         * MSI48, if USB is configured.
         */
        if ( clk_config_byte < CLK_MSI_VRNG1_48MHZ_2WS ) {
            DEBUG_PRINTF("Error: Clock config #%d not allowed with USB active. Setting SYSCLK to MSI48\n", clk_config_byte); 
            clk_config_byte = CLK_MSI_VRNG1_48MHZ_2WS;
        }
    #endif
    /* Save desired settings for restoration after Stop */
    saved_bSwitchOffMSI   = bSwitchOffMSI;

    /* 
     * Initially assume, that there is no restoration needed
     * In general, all clock settings on basis of HSI16 or MSI are non volatile
     * and do not need a restoration, whereas all other clock settings 
     * ( eg. HSE or PLL are volatile and need a restauration 
     */

    if (  clk_config_byte <= CLK_MSI_VRNG1_48MHZ_2WS ) {
        /* First, handle all clock settings, that are not PLL based */
        bClockSettingVolatile = false;         
        RestoreFn             = NULL;

        switch ( clk_config_byte ) {
            case CLK_MSI_VRNG1_08MHZ_0WS:
                SystemClock_MSI_Vrange_1(RCC_MSIRANGE_7);
                break;
            case CLK_MSI_VRNG2_08MHZ_1WS:
                SystemClock_MSI_Vrange_2(RCC_MSIRANGE_7);
                break;
            case CLK_HSE_VRNG1_08MHZ_0WS:
                #if defined(HW_HAS_LSE)
                    bClockSettingVolatile   = true; 
                    RestoreFn               = SystemClock_HSE_8MHz_Vrange_1_0WS_restore;
                    SystemClock_HSE_8MHz_Vrange_1_0WS(bSwitchOffMSI);
                #else
                    DEBUG_PRINTF("Clock option not avialable - No LSE oscillator equipped");
                #endif
                break;
            case CLK_HSE_VRNG2_08MHZ_1WS:
                #if defined(HW_HAS_LSE)
                    bClockSettingVolatile   = true;         
                    RestoreFn               = SystemClock_HSE_8MHz_Vrange_2_1WS_restore;
                    SystemClock_HSE_8MHz_Vrange_2_1WS(bSwitchOffMSI);
                #else
                    DEBUG_PRINTF("Clock option not avialable - No LSE oscillator equipped");
                #endif
                break;
            case CLK_HSI_VRNG1_16MHZ_0WS:
                SystemClock_HSI_16MHz_Vrange_1_0WS(bSwitchOffMSI);
                break;
            case CLK_HSI_VRNG2_16MHZ_2WS:
                SystemClock_HSI_16MHz_Vrange_2_2WS(bSwitchOffMSI);
                break;
            case CLK_MSI_VRNG1_16MHZ_0WS:
                SystemClock_MSI_Vrange_1(RCC_MSIRANGE_8);
                break;
            case CLK_MSI_VRNG2_16MHZ_2WS:
                SystemClock_MSI_Vrange_2(RCC_MSIRANGE_8);
                break;
            case CLK_MSI_VRNG1_24MHZ_1WS:
                SystemClock_MSI_Vrange_1(RCC_MSIRANGE_9);
                break;
            case CLK_MSI_VRNG2_24MHZ_3WS:
                SystemClock_MSI_Vrange_2(RCC_MSIRANGE_9);
                break;
            case CLK_MSI_VRNG1_32MHZ_1WS:
                SystemClock_MSI_Vrange_1(RCC_MSIRANGE_10);
                break;
            case CLK_MSI_VRNG1_48MHZ_2WS:
                SystemClock_MSI_Vrange_1(RCC_MSIRANGE_11);
                break;
            default:
                DEBUG_PRINTF("Error: Clockconfig - Unknown config #%d\n", clk_config_byte);       
        } // switch
    } else {
        /* 
         * Thereafter, handle all PLL-based clock settings: All these clock settings are
         * volatile and require a restore function after wakeup from stop
         */
        bClockSettingVolatile = true;         
        RestoreFn             = SystemClock_PLL_xxMHz_Vrange_1_restore;
        saved_mhz             = GetPLLClockRate(clk_config_byte );
        if ( saved_mhz == 0 ) { 
            DEBUG_PRINTF("Error: Clockconfig - Unknown config #%d, setting 48Mhz PLL clk\n", clk_config_byte);       
            saved_mhz = 48;
        }                
        SystemClock_PLL_xxMHz_Vrange_1(saved_mhz, bSwitchOffMSI);
    } // if

    /* Notice all devices about frequency change                             */
    /* Momentarily, the frq change is not reverted, if any device refuses to */
    /* change frq. this has to be changed for correct implementation         */
    /* Eg by asking before frq change takes place                            */
    if ( DevicesInhibitFrqChange() ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("ERR: One or more devices did not agree to new frequency!");
        #endif
    }

    #if DEBUG_MODE > 0
        DEBUG_PRINTF("SYSCLK nom. %d\n", HAL_RCC_GetSysClockFreq());
        DEBUG_PRINTF("SYSCLK real  %d\n",Get_SysClockFrequency());
    #endif
}

/******************************************************************************
 * Set the system clock configuration as configured in config-variable
 *****************************************************************************/
void SystemClock_SetConfiguredClock(void)
{
  // Set clcock configuration, switch MSI off, if no more used
  SystemClock_Set(config.clk_config, true );
}

void LSEClockConfig(bool bLSEon, bool bUseAsRTCClock)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /* Enable the PWR Clock and Enable access to the backup domain */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /*Switch LSE on or OFF */
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = bLSEon ? RCC_LSE_ON : RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      Error_Handler(__FILE__, __LINE__);
  
  /* Set to lowest driving strngth */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
 
  /* Set LSE as RTC clock source, if desired */
  if ( bUseAsRTCClock )  {
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
      PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
          Error_Handler(__FILE__, __LINE__);
  }

  /* IF MSI is running, set the MSIPLL bit for automatic synchronization of MSI to LSE clock */
  if ( RCC->CR & RCC_CR_MSION ) {
    SET_BIT(RCC->CR, RCC_CR_MSIPLLEN );
  }
   
  /*Disable Backup Access and PWR Clock */ 
  HAL_PWR_DisableBkUpAccess();
  __HAL_RCC_PWR_CLK_DISABLE(); 
}

void HSIClockConfig(bool bHSIon)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  
  /*Switch HSI on or OFF */
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;            /* Reset value */
  RCC_OscInitStruct.HSIState = bHSIon ? RCC_HSI_ON : RCC_HSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      Error_Handler(__FILE__, __LINE__);
  
  /* Set HSI Autostart from Stop mode */
  RCC->CR |= RCC_CR_HSIASFS;
}


/******************************************************************************
 * Do a Calibration of the HSI16 clock 
 * To do this, the LSE clock has to be on. SYSCLK will be switched to
 * HSI16 temporarily
 * TMR15 is used to perform the calibration
 *****************************************************************************/

#include "debug_helper.h"

#define __TIMx_CLK_ENABLE()        __HAL_RCC_TIM15_CLK_ENABLE()
#define __TIMx_CLK_DISABLE()       __HAL_RCC_TIM15_CLK_DISABLE()
#define  TIMx                       TIM15
#define  TIM_CHANNEL_y              TIM_CHANNEL_1
#define  HAL_TIM_ACTIVE_CHANNEL_y   HAL_TIM_ACTIVE_CHANNEL_1
#define  TIM_TIMx_GPIO              TIM_TIM15_TI1_GPIO
#define  TIM_TIMx_LSE               TIM_TIM15_TI1_LSE
#define  TIMx_IRQn                  TIM1_BRK_TIM15_IRQn

#define CAPTURE_START              ((uint32_t) 0x00000001)
#define CAPTURE_ONGOING            ((uint32_t) 0x00000002)
#define CAPTURE_COMPLETED          ((uint32_t) 0x00000003)

/* No timer input prescaler */
#define HSI_TIMx_COUNTER_PRESCALER   ((uint32_t)0)
/* The signal in input capture is divided by 8 */
#define HSI_TIMx_IC_DIVIDER          TIM_ICPSC_DIV8

/* The LSE is divided by 8 => LSE/8 = 32768/8 = 4096 */
#define REFERENCE_FREQUENCY         ((uint32_t)4096) /*!< The reference frequency value in Hz */

/* Timeout to avoid endless loop */
#define HSI_TIMEOUT                    ((uint32_t)0xFFFFFF)

/* Number of measurements in the loop */
#define HSI_NUMBER_OF_LOOPS            ((uint32_t)10)

#define INITIAL_ERROR              ((uint32_t)99999000)

TIM_HandleTypeDef  TimHandle; /* Timer handler declaration */
uint32_t __IO       CaptureState;
uint32_t __IO       Capture;
uint32_t            StartCalibration;
uint32_t            IC1ReadValue1 = 0, IC1ReadValue2 = 0;
/* Exported macro ------------------------------------------------------------*/
#define __HAL_GET_TIM_PRESCALER(__HANDLE__)       ((__HANDLE__)->Instance->PSC)
#define ABS_RETURN(x)                             (((x) < 0) ? -(x) : (x))
#define __HAL_GET_HSI_CALIBRATIONVALUE() \
                  ((RCC->ICSCR & RCC_ICSCR_HSITRIM_Msk) >> RCC_ICSCR_HSITRIM_Pos)

/******************************************************************************
  * @brief  This function handles TIM15 interrupt request 
  * (for HSI16 calibration).
  * @param  None
  * @retval None
  ****************************************************************************/
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

/******************************************************************************
  * @brief  Conversion complete callback in non blocking mode
  * @param  htim : hadc handle
  * @retval None
  ****************************************************************************/
void TimerCaptureCallback(TIM_HandleTypeDef *htim)
{

  if ((htim->Channel) == HAL_TIM_ACTIVE_CHANNEL_y)
  {
    if (CaptureState == CAPTURE_START)
    {
      /* Get the 1st Input Capture value */
      IC1ReadValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_y);
      CaptureState = CAPTURE_ONGOING;
    }
    else if (CaptureState == CAPTURE_ONGOING)
    {
      /* Get the 2nd Input Capture value */
      IC1ReadValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_y);

      /* Capture computation */
      if (IC1ReadValue2 > IC1ReadValue1)
      {
        Capture = (IC1ReadValue2 - IC1ReadValue1);
      }
      else if (IC1ReadValue2 < IC1ReadValue1)
      {
        Capture = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2);
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
        measures */
        Error_Handler(__FILE__, __LINE__);
      }

      CaptureState = CAPTURE_COMPLETED;
    }
  }
}

#if 0
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    Error_Handler(__FILE__, __LINE__);
}
void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    Error_Handler(__FILE__, __LINE__);
}
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    Error_Handler(__FILE__, __LINE__);
}
#endif

/******************************************************************************
  * @brief Measures actual value of HSI
  * @param  None.
  * @retval Actual HSI frequency
  ****************************************************************************/
uint32_t HSI_FreqMeasure(void)
{
  uint32_t  measured_frequency;
  uint32_t  loop_counter = 0;
  uint32_t  timeout = HSI_TIMEOUT;

  /* Start frequency measurement for current trimming value */
  measured_frequency = 0;
  loop_counter = 0;
  /* Start measuring Internal Oscillator frequency */
  while (loop_counter <= HSI_NUMBER_OF_LOOPS)
  {
    CaptureState = CAPTURE_START;

    /* Enable capture 1 interrupt */
    HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_y);

    /* Enable the TIMx IRQ channel */
    HAL_NVIC_EnableIRQ(TIMx_IRQn);

    /* Wait for end of capture: two consecutive captures */
    while ((CaptureState != CAPTURE_COMPLETED) && (timeout != 0))
    {
      if (--timeout == 0)
      {
        return ERROR;
      }
    }

    /* Disable IRQ channel */
    HAL_NVIC_DisableIRQ(TIMx_IRQn);

    /* Disable TIMx */
    HAL_TIM_IC_Stop_IT(&TimHandle, TIM_CHANNEL_y);

    if (loop_counter != 0)
    {
      /* Compute the frequency (the Timer prescaler isn't included) */
      measured_frequency += (uint32_t) (REFERENCE_FREQUENCY * Capture);
    }

    /* Increment loop counter */
    loop_counter++;
  }
  /* END of Measurement */

  /* Compute the average value corresponding the current trimming value */
  measured_frequency = (uint32_t)((__HAL_GET_TIM_PRESCALER(&TimHandle) + 1) * (measured_frequency / HSI_NUMBER_OF_LOOPS));
  return measured_frequency;
}


/**
  * @brief  Calibrates internal oscillators HSI to the minimum computed error.
  *         The system clock source is checked:
  *           - If HSI oscillator is used as system clock source, HSI is calibrated
  *             and the new HSI value is returned.
  *           - Otherwise function returns 0.
  * @param  None.
  * @retval The optimum computed frequency of HSI oscillator.
  *         Returning 0 means that the system clock source is not HSI.
  */
uint32_t HSI_CalibrateMinError(void)
{
  uint32_t  measured_frequency = 0;
  uint32_t  sys_clock_frequency = 0;
  uint32_t  optimum_frequency = 0;
  uint32_t  frequency_error = 0;
  uint32_t  optimum_frequency_error = INITIAL_ERROR; /* Large value */
  uint32_t  steps_number = 0;         /* Number of steps: size of trimming bits */
  uint32_t  trimming_value = 0;
  uint32_t  optimum_calibration_value = 0;
  uint32_t  calibrated_frequency = 0;

  /* Get system clock frequency */
  sys_clock_frequency = HAL_RCC_GetSysClockFreq();

  if (StartCalibration != 0)
  {
    /* 
     * HSI16TRIM is 5-bit or 7 bits ( on SXM32L49x ) wide. The Calibration default value from 
     * HAL driver is the mid value in that range. So the total number of possible steps is
     * twice the calibration default value 
     */
    steps_number = RCC_HSICALIBRATION_DEFAULT * 2; /* number of steps is twice the default value */
  }
  else
  {
    /* Without Calibration */
    steps_number = 1;
  }

  /* Internal Osc frequency measurement for steps_number */
  for (trimming_value = 0; trimming_value < steps_number; trimming_value++)
  {
    if (StartCalibration != 0)
    {
      /* Set the Intern Osc trimming bits to trimming_value */
      __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(trimming_value);
    }

    /* Get actual frequency value */
    measured_frequency = HSI_FreqMeasure();

    if (StartCalibration != 0)
    {
      /* Compute current frequency error corresponding to the current trimming value:
      measured value is subtracted from the typical one */
      frequency_error = ABS_RETURN((int32_t) (measured_frequency - sys_clock_frequency));

      /* Get the nearest frequency value to typical one */
      if (optimum_frequency_error > frequency_error)
      {
        optimum_frequency_error = frequency_error;
        optimum_calibration_value = trimming_value;
        optimum_frequency = measured_frequency;
      }
    }
  }

  if (StartCalibration != 0)
  {
    /* Set trimming bits corresponding to the nearest frequency */
    __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(optimum_calibration_value);
    /* Return the intern oscillator frequency after calibration */
    calibrated_frequency = optimum_frequency;
  }
  else
  {
    /* Return the intern oscillator frequency before calibration */
    calibrated_frequency = measured_frequency;
  }

  return calibrated_frequency;
}

/******************************************************************************
  * @brief Configures the TIMx in input capture to measure HSI frequency.
  * @param  None.
  * @retval None.
  ****************************************************************************/
static void HSI_TIMx_ConfigForCalibration(void)
{
  TIM_IC_InitTypeDef      ic_config; /* Timer Input Capture Configuration Structure declaration */

  /* Enable TIMx clock */
  __TIMx_CLK_ENABLE();

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Reset TIMx registers */
  HAL_TIM_IC_DeInit(&TimHandle);

    /* Connect LSE clock to TIMx Input Capture 1 */
    HAL_TIMEx_RemapConfig(&TimHandle, TIM_TIMx_LSE);     

  /* Initialize TIMx peripheral as follows:
       + Period = 0xFFFF
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 0xFFFF;
  TimHandle.Init.Prescaler         = HSI_TIMx_COUNTER_PRESCALER;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(__FILE__, __LINE__);
  }

  /* Register capture Callback */
  Tim_Register_CaptureCB(TimHandle.Instance, TimerCaptureCallback);

  /* Configure the Input Capture of channel y */
  ic_config.ICPolarity  = TIM_ICPOLARITY_RISING;
  ic_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic_config.ICPrescaler = HSI_TIMx_IC_DIVIDER;
  ic_config.ICFilter    = 0;
  if (HAL_TIM_IC_ConfigChannel(&TimHandle, &ic_config, TIM_CHANNEL_y) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler(__FILE__, __LINE__);
  }

  /* Configure the NVIC for TIMx */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0, 1);

  /* Disable the TIMx global Interrupt */
  HAL_NVIC_DisableIRQ(TIMx_IRQn);

}
/******************************************************************************
  * @brief DeInitializes TIMx
  * @param  None.
  * @retval None.
  ****************************************************************************/
static void HSI_TIMx_DeInit(void)
{

  /* Disable the TIMx global Interrupt */
  HAL_NVIC_DisableIRQ(TIMx_IRQn);

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* UnRegister capture Callback */
  Tim_UnRegister_CaptureCB(TimHandle.Instance);

  /* Reset TIMx registers */
  HAL_TIM_IC_DeInit(&TimHandle);

  /* Disable TIMx clock */
  __TIMx_CLK_DISABLE();


}



bool HSIClockCalibrate ( void )
{
    /* check for LSE being on and ready */
    if ( (RCC->BDCR & ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) != ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) 
      return false;

    /* switch to HSI16, Vrange1, 0 WS*/
    SystemClock_Set(CLK_HSI_VRNG1_16MHZ_0WS, false); 

    /* Prepare TIM15 for calibration */
    HSI_TIMx_ConfigForCalibration();


    uint32_t old_freq = HSI_FreqMeasure();
    uint8_t old_trim = __HAL_GET_HSI_CALIBRATIONVALUE();

    /* Do the calibration */
    StartCalibration = 1;
    uint32_t act_freq = HSI_CalibrateMinError();

    /* DeInit all the timer stuff */
    HSI_TIMx_DeInit();

    /* Switch back to default clock source */
    SystemClock_SetConfiguredClock();

    DEBUG_PUTS("Performing HSI16 Calibration");
    DEBUG_PRINTF("HSI-Cal before=%d, Freq=%d\n", old_trim, old_freq);
    DEBUG_PRINTF("HSI-Cal  after=%d, Freq=%d\n", __HAL_GET_HSI_CALIBRATIONVALUE(), act_freq);

    return true;
}

uint32_t Get_SysClockFrequency ( void )
{
    /* check for LSE being on and ready */
    if ( (RCC->BDCR & ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) != ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) 
      return 0;

    /* Prepare TIM15 for calibration */
    HSI_TIMx_ConfigForCalibration();


    uint32_t _freq = HSI_FreqMeasure();

    /* DeInit all the timer stuff */
    HSI_TIMx_DeInit();

    return _freq;
}

/******************************************************************************
 * enable the MCO output at PA8. Output clock is undivided.
 * The MCO source can be selected by parameter
 * 0 - Off & DeInit PA8
 * 1 - SYSCLK
 * 2 - MSI
 * 3 = HSI
 * 4 = HSE
 * 5 = PLL
 * 6 = LSE
 * 7 = LSI
 *****************************************************************************/
void EnableMCO ( uint32_t mcoSource )
{
    uint32_t halParam;
    switch ( mcoSource ) {
        case 1: halParam = RCC_MCO1SOURCE_SYSCLK; break;
        case 2: halParam = RCC_MCO1SOURCE_MSI; break;
        case 3: halParam = RCC_MCO1SOURCE_HSI; break;
        case 4: halParam = RCC_MCO1SOURCE_HSE; break;
        case 5: halParam = RCC_MCO1SOURCE_PLLCLK; break;
        case 6: halParam = RCC_MCO1SOURCE_LSE; break;
        case 7: halParam = RCC_MCO1SOURCE_LSI; break;
        default:
            halParam = RCC_MCO1SOURCE_NOCLOCK;
    }
    HAL_RCC_MCOConfig(RCC_MCO1, halParam, RCC_MCODIV_1);
    if ( halParam == RCC_MCO1SOURCE_NOCLOCK ) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
    }
}

/**
  * @}
  */


