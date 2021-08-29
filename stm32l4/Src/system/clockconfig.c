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

#include "config/config.h"

#include "stm32l4xx_hal.h"
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

/*
 *************************************************************************************
 * MSI frequencies for the 12 MSI ranges and MSI_Range codings for the 12 MSI Ranges
 *
 *************************************************************************************/
static const uint16_t msi_range_frq_khz[] = 
{   100,   200,   400,   800, 
    1000,  2000,  4000,  8000,
    16000, 24000, 32000, 48000,
};


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

#if 0
    old style wait state selection algorithm
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
#endif


#if defined(STM32L476xx) || defined(STM32L496xx)
    /*
     *******************************************************************************
     Flash wait states in dependency of Vcore and HCLK for STM32L4 devices
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
/* Maximum allowable HCLK for Vcore Range1 and Range2 */
/* minimum # of flash WS                    0               1                   2                   3                   4 */       
static const uint8 hclk_range_1[]  = {16,                32,                 48,                 64,                 80, };
static const uint8 hclk_range_2[]  = { 6,                12,                 18,                 26, };
static const uint32_t flash_latency[] = {FLASH_LATENCY_0,   FLASH_LATENCY_1,    FLASH_LATENCY_2,    FLASH_LATENCY_3,    FLASH_LATENCY_4, };
#elif defined(STM32L4Sxxx)
    /*
     *******************************************************************************
     Flash wait states in dependency of Vcore and HCLK for STM32L4+ devices
     (WS)(LATENCY)               VCORE Range 1(1)   VCORE Range 2(2)
         0 WS (1 CPU cycles)         = 20                = 8
         1 WS (2 CPU cycles)         = 40                = 16
         2 WS (3 CPU cycles)         = 60                = 26
         3 WS (4 CPU cycles)         = 80                = 26
         4 WS (5 CPU cycles)         = 100               = 26
         5 WS (6 CPU cycles)         = 120               = 26
     maximum HCLK at Vcore Range 2 is 26 MHz!
     *******************************************************************************
     */
/* Maximum allowable HCLK for Vcore Range1 and Range2 */
/* minimum # of flash WS                    0               1                   2                   3                   4                5                   */       
static const uint8_t hclk_range_1[]  = {20,                40,                 60,                 80,                 100,             120, };
static const uint8_t hclk_range_2[]  = { 8,                16,                 26, };
static const uint8_t flash_latency[] = {FLASH_LATENCY_0,   FLASH_LATENCY_1,    FLASH_LATENCY_2,    FLASH_LATENCY_3,    FLASH_LATENCY_4, FLASH_LATENCY_5, };

#else
    #error "No Flash Waitstate table for selected MCU"
#endif

#define      MAX_LATENCY()                   flash_latency[ARRSIZE(uint32_t, flash_latency)-1];
#define      ARRSIZE(basetype,array)         (sizeof(array)/sizeof(basetype))

static const uint8_t* range_tables[] = {hclk_range_1,      hclk_range_2, };
static const uint32_t  size_tables[]  = {ARRSIZE(uint8_t, hclk_range_1),      ARRSIZE(uint8_t, hclk_range_2),  };
/*
 *******************************************************************************
 Flash wait states in dependency of Vcore and HCLK for STM32L4 devices
 (WS)(LATENCY)               VCORE Range 1(1)   VCORE Range 2(2)
     0 WS (1 CPU cycles)         = 16                = 6
     1 WS (2 CPU cycles)         = 32                = 12
     2 WS (3 CPU cycles)         = 48                = 18
     3 WS (4 CPU cycles)         = 64                = 26
     4 WS (5 CPU cycles)         = 80                = 26
 maximum HCLK at Vcore Range 2 is 26 MHz!
 *******************************************************************************
 */
static uint32_t SystemClock_GetFlashLatency( uint8_t vrange, uint8_t mhz )
{  
  const uint8_t *act_range_table;
  uint32_t act_table_size;
  uint32_t i;
 
  /* Ensure, we have a table for the passed range */
  if ( vrange > ARRSIZE(uint32_t *, range_tables) ) {
      Error_Handler(__FILE__, __LINE__); 
      return MAX_LATENCY();
  } else {
      act_range_table = range_tables[vrange-1];
      act_table_size  = size_tables [vrange-1];
  }

  /* Find the apprpriate entry in the selected frequency table, ie that antry that is equal or higher than the desired HCLK frq */
  i = 0;
  while ( i < act_table_size && act_range_table[i] < mhz ) {
    i++;
  }

  /* Check, whether desired frq is within allowed range */
  if ( i >= act_table_size ) {
      Error_Handler(__FILE__, __LINE__); 
      return MAX_LATENCY();
   } else {
      return flash_latency[i];
   }
}

#if USE_USB > 0
    static inline __attribute__((always_inline)) void SC_SetUsbClockSource(uint32_t source )
    {
        MODIFY_REG(RCC->CCIPR, RCC_CCIPR_CLK48SEL_Msk, source );
    }
#endif


#if defined(STM32L4S9xx)
    /*
     *************************************************************************************************
     * @brief  Set the Vcore voltage to Vrange1, Vrange2 or VRange1 Boost
     * @param   vrange - 1,2 or 3 according to Vrange1, Vrange2 and  VRange1 Boost
     */
    static void SC_SetVrange ( uint8_t vrange )
    {
        uint32_t pwrbit = __HAL_RCC_PWR_IS_CLK_ENABLED();
        if ( !pwrbit ) __HAL_RCC_PWR_CLK_ENABLE();

        HAL_PWREx_ControlVoltageScaling( vrange == 1 ? PWR_REGULATOR_VOLTAGE_SCALE1 :  vrange == 2 ? PWR_REGULATOR_VOLTAGE_SCALE2 : PWR_REGULATOR_VOLTAGE_SCALE1_BOOST );

        /* Switch PWR domain clock off again, if it was off before */
        if ( !pwrbit ) __HAL_RCC_PWR_CLK_DISABLE();
    }
#else
    /*
     *************************************************************************************************
     * @brief  Set the Vcore voltage to Vrange1 or Vrange2
     * @param   vrange - 1or 2 according to Vrange1 and Vrange2
     */
    static void SC_SetVrange ( uint8_t vrange )
    {
        uint32_t pwrbit = __HAL_RCC_PWR_IS_CLK_ENABLED();
        if ( !pwrbit ) __HAL_RCC_PWR_CLK_ENABLE();

        HAL_PWREx_ControlVoltageScaling( vrange == 1 ? PWR_REGULATOR_VOLTAGE_SCALE1 : PWR_REGULATOR_VOLTAGE_SCALE2 );

        /* Switch PWR domain clock off again, if it was off before */
        if ( !pwrbit ) __HAL_RCC_PWR_CLK_DISABLE();
    }
#endif
/*
 *************************************************************************************************
 * @brief  System Clock Configuration on MSI-Base
 *         The system Clock is configured as follow : 
 *            System Clock source            = MSI
 *            SYSCLK(Hz)                     = 100kHz ... 48MHz, depending from range
 *            HCLK(Hz)                       = 100kHz ... 48MHz, depending from range
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            Flash Latency(WS)              = depending from SYSCLK
 *            Main regulator output voltage  = Range 2
 * @param range - any value between 0 .. 11 
 *            0: around 100 kHz
 *            1: around 200 kHz
 *            2: around 400 kHz
 *            3: around 800 kHz
 *            4: around 1 MHz
 *            5: around 2 MHz
 *            6: around 4 MHz
 *            7: around 8 MHz
 *            8: around 16 MHz
 *            9: around 24 MHz
 *           10: around 32 MHz NOT ALLOWED with Vrange2
 *           11: around 48 MHz NOT ALLOWED with Vrange2
 * @param vrange - 1 or 2 according to Vrange1 or Vrange2
 *
 * @retval None
 */
static void SystemClock_MSI(uint8_t range, uint8_t vrange)
{
  /* Check validity of range and vrange */  
  if ( range >= ARRSIZE(uint16_t, msi_range_frq_khz) ) {
      Error_Handler(__FILE__, __LINE__);
      return;
  }

  if ( vrange > ARRSIZE(uint8_t*, range_tables) ) {
      Error_Handler(__FILE__, __LINE__);
      return;
  }

  uint32_t frq_mhz       = msi_range_frq_khz[range]/1000;
  uint32_t flash_latency = SystemClock_GetFlashLatency( vrange, frq_mhz );
  uint32_t msi_range     = ( ((uint32_t)range) << RCC_CR_MSIRANGE_Pos ) & RCC_CR_MSIRANGE_Msk;

  /* maximum system frq with Vrange2 is 26 Mhz, check it*/
  if( vrange == 2 && frq_mhz > 26 ) {
      Error_Handler(__FILE__, __LINE__);
      return;
  }

  /* if Vrange = 1 then set it first to avoid overclocking when changing clock parameters */ 
  if ( vrange == 1 ) SC_SetVrange(vrange);

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

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* 
   * if Vrange = 2 then set it after all other clock parameters have been set 
   * to avoid overclocking
   */
  if ( vrange == 2 ) SC_SetVrange(vrange);
  
  
  /* Select MSI as WakeUp clock */
  CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);

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

#if defined(HW_HAS_HSE)
    /******************************************************************************
     * Switch HSE clock on
     *****************************************************************************/
    static void SwitchOnHSE(void)
    {
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  
      /* Enable HSE Oscillator */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      #if defined(HW_HAS_HSE_CRYSTAL)
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
      #elif defined(HW_HAS_HSE_BYPASS)
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
    static void SystemClock_HSE_Vrange2_restore ( uint32_t xxmhz, bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      UNUSED(xxmhz);
      SwitchOnHSE();

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency( 2, HSE_FREQUENCY/1000000 ));

      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
      }
    }

 /*
  ************************************************************************************************************
  * @brief  System Clock Configuration on HSE crystal or HSE clock input with Vrange2
  *         The system Clock is configured as follow : 
  *            System Clock source            = HSE crystal clock or HSE bypass clock input
  *            SYSCLK(Hz)                     = depends from HSE input clock or HSE crystal resp.
  *            HCLK(Hz)                       = depends from HSE input clock or HSE crystal resp.
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = set according to HSE frequency
  *            Vrange                         = 2
  * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
  *         MUST be activated again, BEFORE going to sleep
  * @note only available, if HSE oscillator is equipped
  * @note the HCLK frq depends from the HSE crystal frq or the HSE clock input signal in bypass mode
  *
  * @retval None
  */
  static void SystemClock_HSE_Vrange2(bool bSwitchOffMSI)
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
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(2, HSE_FREQUENCY/1000000) )!= HAL_OK)
      {
        /* Initialization Error */
        while(1); 
      }

      /* Select core voltage range 2 after all other changes to prevent overclocking */
      SC_SetVrange(2);

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
    static void SystemClock_HSE_Vrange1_restore ( uint32_t xxmhz, bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      UNUSED(xxmhz);
      SwitchOnHSE();

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(1, HSE_FREQUENCY/1000000) );
      if ( bSwitchOffMSI ) {
        SwitchOffMSI();
      }
    }

 /*
  ************************************************************************************************************
  * @brief  System Clock Configuration on HSE crystal or HSE clock input with Vrange1
  *         The system Clock is configured as follow : 
  *            System Clock source            = HSE crystal clock or HSE bypass clock input
  *            SYSCLK(Hz)                     = depends from HSE input clock or HSE crystal resp.
  *            HCLK(Hz)                       = depends from HSE input clock or HSE crystal resp.
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = set according to HSE frequency
  *            Vrange                         = 1
  * @param  bSwitchOffMSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
  *         MUST be activated again, BEFORE going to sleep
  * @note only available, if HSE oscillator is equipped
  * @note the HCLK frq depends from the HSE crystal frq or the HSE clock input signal in bypass mode
  *
  * @retval None
  */
    static void SystemClock_HSE_Vrange1(bool bSwitchOffMSI)
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
      SwitchOnHSE();
  
      /* Select core voltage range 1 before any other changes */
      SC_SetVrange(1);

      /* Select HSE as system clock source and configure the HCLK, PCLK1 and PCLK2 
         clocks dividers */
      RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(1, HSE_FREQUENCY/1000000) )!= HAL_OK)
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
#endif /* HW_HAS_HSE */


/**
  * @brief  Switch on HSI an select as system clock 
  *
  * @note Caller must assure, that core voltage and wait srtats are selected properly before!
  * 
  * allowed are - Vrange 1, min 0 flash WS
  *             - Vrange 2, min 2 flash WS
  *
  */
static void SC_SwitchOnHSI16(uint8_t vrange)
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
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(vrange, 16))!= HAL_OK)
    {
        /* Initialization Error */
        while(1); 
    }
}

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

static void SystemClock_HSI_16MHz(bool bSwitchOffMSI, uint8_t vrange)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Select core voltage range 1 before any other clock changes */
  if ( vrange == 1 ) SC_SetVrange(vrange);

  SC_SwitchOnHSI16(vrange);
  
  /* Select core voltage range 2 after all other clock changes */
  if ( vrange == 2 ) SC_SetVrange(vrange);

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
    SC_SwitchOnHSI16(1);
  }

  #if USE_USB > 0
      CLEAR_BIT(RCC->CR, RCC_CR_PLLSAI1ON_Msk);
  #endif

  /* if HSE oscillator is equipped, use as PLL input, otherwise use HSI16 */
  #if defined(PLL_INP_FRQ) 
    #undef PLL_INP_FRQ
  #endif

  #if defined(HW_HAS_HSE)
    SwitchOnHSE();
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    pllsrc = RCC_PLLSOURCE_HSE;
    #define PLL_INP_FRQ  HSE_FREQUENCY
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
static void SystemClock_PLL_xxMHz_Vrange1_restore (uint32_t xxmhz, bool bSwitchOffMSI)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
  uint32_t flash_latency = SystemClock_GetFlashLatency(1, xxmhz );
  SwitchOnPLL(xxmhz);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency );

  if ( bSwitchOffMSI ) {
    SwitchOffMSI();
  }
}
#if defined ( STM32L4Sxxx )
    /*
     * Switch system Clock to PLL is somewhat complicated on L4+ devices:
     *
     * When Vrange1 Boost mode is selected the first time, scaling up has
     * to be done in two steps: first with HCLK divided by 2, then
     * with desired HCLK in second step
     *
     * Vice versa, if Boost mode is deselected the first time, also
     * frq decrease has also to be done in two steps: first, set
     * HCLK prescaler to 2, then disabel boost, then set desired clk frq
     */

    static void SC_Switch_to_PLL ( uint32_t xxmhz )
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      uint32_t flash_latency;
      uint8_t vrange = xxmhz > 80 ? 3:1;
      
      /* Activate PWR, if not already activated */
      uint32_t pwrbit = __HAL_RCC_PWR_IS_CLK_ENABLED();
      if ( !pwrbit ) __HAL_RCC_PWR_CLK_ENABLE();

      uint32_t is_boosted = ( (PWR->CR5 & PWR_CR5_R1MODE) == 0 );

      /* switch off boost mode -> first step: divide HCLK by 2 */
      if ( is_boosted && vrange == 1 ) {
          RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
          RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
          /* We use the intermediate flash latency for 60MHz HCLK, which is always on the safe side */
          if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(1, 60 )) != HAL_OK) {
            /* Initialization Error */
            while(1);
          }
      }
      
      if ( !is_boosted && vrange == 3 ) {
          /* switch to boost mode: first step: divide HCLK by 2 */
          RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
          flash_latency = SystemClock_GetFlashLatency(1, xxmhz/2);
      } else {
          RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
          flash_latency = SystemClock_GetFlashLatency(1, xxmhz);
      }

      /* Select core voltage range 1 as desired */
      SC_SetVrange(vrange);

      /* configure and switch on PLL with desired frquency */
      SwitchOnPLL(xxmhz);
  
      /* 
       * Select Pll as system clock source and configure PCLK1 and PCLK2 clocks dividers,
       * the HCLK divider has been set above
       */
      RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      // see above RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency )!= HAL_OK)
      {
        /* Initialization Error */
        while(1); 
      }

      /* last step: If boost mode is enabled the first time, now set HCLK prescaler to 1 */
      if ( !is_boosted && vrange == 3 ) {  
          RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
          RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
          if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SystemClock_GetFlashLatency(1, xxmhz )) != HAL_OK) {
            /* Initialization Error */
            while(1);
          }
      }

      if ( !pwrbit ) __HAL_RCC_PWR_CLK_DISABLE();
    }
#else
    /*
     * Switch system Clock to PLL is easy on L4 devices: Just switch 
     */
    static void SC_Switch_to_PLL ( uint32_t xxmhz )
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

      uint32_t flash_latency = SystemClock_GetFlashLatency(1, xxmhz);
      uint8_t vrange = 1;

      /* Select core voltage range 1 before any other clock changes */
      SC_SetVrange(vrange);

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
    }
#endif
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

static void SystemClock_PLL_xxMHz_Vrange1(uint32_t xxmhz, bool bSwitchOffMSI)
{ 
 
  SC_Switch_to_PLL(xxmhz);
     
  #if USE_USB > 0
    SetupUSB48Clk();
  #endif

  /* Disable MSI Oscillator, if desired */
  if ( bSwitchOffMSI ) {
    SwitchOffMSI();
  }

  /* Select HSI16 as WakeUp clock */
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
}

static const uint16_t pll_clk_rates[] = ALLOWED_PLL_CLOCKS;
/*******************************************************************************
 * Return the clock spped in MHz for a given CLK_CONFIG_T with PLL useage
 *******************************************************************************/
static uint16_t GetPLLClockRate( CLK_CONFIG_T clk_config_byte )
{
    /* check for clk_config_byte being in the PLL range of configurations */
    if ( clk_config_byte < CLK_PLL_VRNG1_16MHZ ) return 0;

    uint8_t clk_idx = clk_config_byte - CLK_PLL_VRNG1_16MHZ;

    if ( clk_idx >= ARRSIZE(uint16_t, pll_clk_rates) ) 
        return 0; 
    else   
        return pll_clk_rates[clk_idx];
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
        if ( clk_config_byte < CLK_MSI_VRNG1_48MHZ ) {
            DEBUG_PRINTF("Error: Clock config #%d not allowed with USB active. Setting SYSCLK to MSI48\n", clk_config_byte); 
            clk_config_byte = CLK_MSI_VRNG1_48MHZ;
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

    if (  clk_config_byte <= CLK_MSI_VRNG1_48MHZ ) {
        /* First, handle all clock settings, that are not PLL based */
        bClockSettingVolatile = false;         
        RestoreFn             = NULL;

        switch ( clk_config_byte ) {
            case CLK_MSI_VRNG2_02MHZ:
                SystemClock_MSI(5, 2);
                break;
            case CLK_MSI_VRNG2_04MHZ:
                SystemClock_MSI(6, 2);
                break;
            case CLK_MSI_VRNG1_08MHZ:
                SystemClock_MSI(7, 1);
                break;
            case CLK_MSI_VRNG2_08MHZ:
                SystemClock_MSI(7, 2);
                break;
            case CLK_HSE_VRNG1:
                #if defined(HW_HAS_HSE)
                    bClockSettingVolatile   = true; 
                    RestoreFn               = SystemClock_HSE_Vrange1_restore;
                    SystemClock_HSE_Vrange1(bSwitchOffMSI);
                #else
                    DEBUG_PRINTF("Clock option not avialable - No LSE oscillator equipped");
                #endif
                break;
            case CLK_HSE_VRNG2:
                #if defined(HW_HAS_HSE)
                    bClockSettingVolatile   = true;         
                    RestoreFn               = SystemClock_HSE_Vrange2_restore;
                    SystemClock_HSE_Vrange2(bSwitchOffMSI);
                #else
                    DEBUG_PRINTF("Clock option not avialable - No LSE oscillator equipped");
                #endif
                break;
            case CLK_HSI_VRNG1_16MHZ:
                SystemClock_HSI_16MHz(bSwitchOffMSI, 1);
                break;
            case CLK_HSI_VRNG2_16MHZ:
                SystemClock_HSI_16MHz(bSwitchOffMSI, 2);
                break;
            case CLK_MSI_VRNG1_16MHZ:
                SystemClock_MSI(8, 1);
                break;
            case CLK_MSI_VRNG2_16MHZ:
                SystemClock_MSI(8, 2);
                break;
            case CLK_MSI_VRNG1_24MHZ:
                SystemClock_MSI(9,1);
                break;
            case CLK_MSI_VRNG2_24MHZ:
                SystemClock_MSI(9,2);
                break;
            case CLK_MSI_VRNG1_32MHZ:
                SystemClock_MSI(10, 1);
                break;
            case CLK_MSI_VRNG1_48MHZ:
                SystemClock_MSI(11, 1);
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
        RestoreFn             = SystemClock_PLL_xxMHz_Vrange1_restore;
        saved_mhz             = GetPLLClockRate(clk_config_byte );
        if ( saved_mhz == 0 ) { 
            DEBUG_PRINTF("Error: Clockconfig - Unknown config #%d, setting 48Mhz PLL clk\n", clk_config_byte);       
            saved_mhz = 48;
        }                
        SystemClock_PLL_xxMHz_Vrange1(saved_mhz, bSwitchOffMSI);
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

/*
 *****************************************************************************
 * Set the system clock configuration as configured in config-variable
 *****************************************************************************/
void SystemClock_SetConfiguredClock(void)
{
  // Set clcock configuration, switch MSI off, if no more used
  SystemClock_Set(config.clk_config, true );
}

/*
 *****************************************************************************
 * Configure LSE clock
 *****************************************************************************/
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

/*
 *****************************************************************************
 * Configure HSI clock
 *****************************************************************************/
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
#define  TIMx_IRQHandler            TIM1_BRK_TIM15_IRQHandler
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

static TIM_HandleTypeDef  CalTimHandle; /* Timer handler declaration */
static uint32_t __IO       CaptureState;
static uint32_t __IO       Capture;
static uint32_t            StartCalibration;
static uint32_t            IC1ReadValue1 = 0, IC1ReadValue2 = 0;
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
void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&CalTimHandle);
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
    HAL_TIM_IC_Start_IT(&CalTimHandle, TIM_CHANNEL_y);

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
    HAL_TIM_IC_Stop_IT(&CalTimHandle, TIM_CHANNEL_y);

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
  measured_frequency = (uint32_t)((__HAL_GET_TIM_PRESCALER(&CalTimHandle) + 1) * (measured_frequency / HSI_NUMBER_OF_LOOPS));
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
  CalTimHandle.Instance = TIMx;

  /* Reset TIMx registers */
  HAL_TIM_IC_DeInit(&CalTimHandle);

    /* Connect LSE clock to TIMx Input Capture 1 */
    HAL_TIMEx_RemapConfig(&CalTimHandle, TIM_TIMx_LSE);     

  /* Initialize TIMx peripheral as follows:
       + Period = 0xFFFF
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up
  */
  CalTimHandle.Init.Period            = 0xFFFF;
  CalTimHandle.Init.Prescaler         = HSI_TIMx_COUNTER_PRESCALER;
  CalTimHandle.Init.ClockDivision     = 0;
  CalTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  CalTimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_IC_Init(&CalTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(__FILE__, __LINE__);
  }

  /* Register capture Callback */
  Tim_Register_CaptureCB(CalTimHandle.Instance, TimerCaptureCallback);

  /* Configure the Input Capture of channel y */
  ic_config.ICPolarity  = TIM_ICPOLARITY_RISING;
  ic_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic_config.ICPrescaler = HSI_TIMx_IC_DIVIDER;
  ic_config.ICFilter    = 0;
  if (HAL_TIM_IC_ConfigChannel(&CalTimHandle, &ic_config, TIM_CHANNEL_y) != HAL_OK)
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
  CalTimHandle.Instance = TIMx;

  /* UnRegister capture Callback */
  Tim_UnRegister_CaptureCB(CalTimHandle.Instance);

  /* Reset TIMx registers */
  HAL_TIM_IC_DeInit(&CalTimHandle);

  /* Disable TIMx clock */
  __TIMx_CLK_DISABLE();


}



bool HSIClockCalibrate ( void )
{
    /* check for LSE being on and ready */
    if ( (RCC->BDCR & ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) != ( RCC_BDCR_LSEON_Msk | RCC_BDCR_LSERDY_Msk ) ) 
      return false;

    /* switch to HSI16, Vrange1, 0 WS*/
    SystemClock_Set(CLK_HSI_VRNG1_16MHZ, false); 

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


