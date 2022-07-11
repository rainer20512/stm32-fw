/**
  ******************************************************************************
  * @file    clockconfig.c
  * @author  Rainer
  * @brief   Several Clock Configurations with different sources, SYSCLK 
  *          frequencies and correct Vcore settings and NVM Wait states
  *
  *          Available configurations
  *          1a. HSI-Clock based with parameterized clk value of 8,16,32,64 MHz
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

#include "hardware.h"
#include "dev/devices.h"
#include "eeprom.h"
#include "system/pll.h"
#include "config/pll_config.h"
#include "system/clockconfig.h"
#include "system/timer_handler.h"


#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#if DEBUG_PROFILING > 0
    #include "system/profiling.h"
#endif

/*
 *************************************************************************************
 * All the stuff for clock change notification callback management
 * ( aside of callbacks in devices ) 
 **** C0001 ****
 ************************************************************************************/
#define MAX_CLKCHG_CB                4

static ClockChangeCB clkCB[MAX_CLKCHG_CB];/* Array of registered clk chng callbacks */
static int32_t numClkchangeCB = 0;      /* Number of "    "    "      "      "      */
static void ClockNotifyCallbacks(uint32_t);  /* forward declaration                 */


/*
 *************************************************************************************
 * As in Stop2 mode the wakeup clock is either MSI or HSI, we have to safe 
 * all neccessary parameters to be able to restore the clock settings when
 * normal clock source is not HSI16 or MSI.
 * When normal clock source is HSI16 or MSI, the same clock is selected as wakeup
 * clock source and we can continue as before sleep without any reconfiguration
 * All the following variables are set in "System_Clock_Set"
 *************************************************************************************/

static uint32_t saved_khz;                 /* The last selected PLL frequency        */
static uint32_t saved_latency;             /* last flash latency used w PLL clk src  */
static uint32_t saved_vosrange;            /* last VOS range used w PLL clk src      */
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
    RestoreFn( saved_khz, saved_bSwitchOffMSI );
}

/*
 *******************************************************************************
 * Flash wait states in dependency of Vcore and HCLK
 * see STM32H7 RefMan pg 166 / Table 16
 * Flash programming delay will never be altered from reset value, 
 * which is FLASH_PROGRAMMING_DELAY_3
 * @note flash_khz is the AXI clock, which is limited to 240MHz
 *******************************************************************************
 */
static uint32_t GetFlashLatency( uint32_t vrange, uint32_t flash_khz )
{  
    uint32_t flash_latency;

    if ( vrange > 0 && flash_khz > 240000 ) {
        Error_Handler_XX(-2, __FILE__, __LINE__); 
        flash_latency = FLASH_LATENCY_4;
    } else {
        switch ( vrange ) {
        case PWR_REGULATOR_VOLTAGE_SCALE0:
        case PWR_REGULATOR_VOLTAGE_SCALE1:
              if      ( flash_khz > 225000 ) flash_latency = FLASH_LATENCY_4;
              else if ( flash_khz > 210000 ) flash_latency = FLASH_LATENCY_3;
              else if ( flash_khz > 140000 ) flash_latency = FLASH_LATENCY_2;
              else if ( flash_khz > 70000  ) flash_latency = FLASH_LATENCY_1;
              else                     flash_latency = FLASH_LATENCY_0;
            break;
        case PWR_REGULATOR_VOLTAGE_SCALE2:
              if      ( flash_khz > 165000 ) flash_latency = FLASH_LATENCY_3;
              else if ( flash_khz > 110000 ) flash_latency = FLASH_LATENCY_2;
              else if ( flash_khz > 55000  ) flash_latency = FLASH_LATENCY_1;
              else                     flash_latency = FLASH_LATENCY_0;
            break;
        case PWR_REGULATOR_VOLTAGE_SCALE3:
              if      ( flash_khz > 180000 ) flash_latency = FLASH_LATENCY_4;
              else if ( flash_khz > 135000 ) flash_latency = FLASH_LATENCY_3;
              else if ( flash_khz > 90000  ) flash_latency = FLASH_LATENCY_2;
              else if ( flash_khz > 45000  ) flash_latency = FLASH_LATENCY_1;
              else                     flash_latency = FLASH_LATENCY_0;
            break;
         default:
            Error_Handler_XX(-2, __FILE__, __LINE__); 
            flash_latency = FLASH_LATENCY_4;
        }
    }
    return flash_latency;
}
/******************************************************************************
 * Set all peripheral clocks to their maximum values with respect to current
 * sysclk rate 
 * @return the selected AHB clock frequency
 *****************************************************************************/
static uint32_t SetPredividers( RCC_ClkInitTypeDef *ClkInit, uint32_t sysclk_khz )
{
    if ( sysclk_khz > 480000 ) {
        Error_Handler_XX(-3, __FILE__, __LINE__); 
    }  

    /* Set all types of peripheral clocks */
    ClkInit->ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);
    
    /* D1CPrescaler is always 1 */
    ClkInit->SYSCLKDivider = RCC_SYSCLK_DIV1;

    /* sysclk_khz <= 480000 here */
    if ( sysclk_khz > 240000 ) {
        ClkInit->AHBCLKDivider = RCC_HCLK_DIV2;
        sysclk_khz /= 2;
    } else {
        ClkInit->AHBCLKDivider = RCC_HCLK_DIV1;
    }   
    /* sysclk_khz is now the AHB clock frequency */

    /* AHB clk <= 240000 here */
    if ( sysclk_khz > 120000 ) {
        ClkInit->APB3CLKDivider = RCC_APB3_DIV2;
        ClkInit->APB1CLKDivider = RCC_APB1_DIV2;
        ClkInit->APB2CLKDivider = RCC_APB2_DIV2;
        ClkInit->APB4CLKDivider = RCC_APB4_DIV2;
   } else {
        ClkInit->APB3CLKDivider = RCC_APB3_DIV1;
        ClkInit->APB1CLKDivider = RCC_APB1_DIV1;
        ClkInit->APB2CLKDivider = RCC_APB2_DIV1;
        ClkInit->APB4CLKDivider = RCC_APB4_DIV1;
   }

   /* return AHB clock frquency */
   return sysclk_khz;
}


/**********************************************************************************
 * Setup the selected SYSCLK provider, which is one OF HSI, HSE or PLL
 * HSI and HSE are handled by HAL Layer, PLL is handeled by hancrafted code
 *
 * In case of PLL, the PLL Clock provider, which can be HSI or HSE has to be
 * started by HAL _before_ PLL is activated
 * @returns != 0 on success
 **********************************************************************************/
static uint32_t SetupSysClkProvider ( RCC_OscInitTypeDef *o, RCC_ClkInitTypeDef *clk )
{
    bool success;
    /* 
     * Configure SYSCLK provider first. That can be HSI, HSE or PLL
     * If SYSCLK Is provided by PLL, the start is no longer done by
     * HAL Layer, but by handcrafted code in pll.c
     */
    /* Set PLL state to none to prevent HAL from configuring/starting PLL again */
    
    success = HAL_RCC_OscConfig(o) == HAL_OK;
    
    /* now handle the setup of the PLL and start it */
    if ( success && clk->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK ) { 
      o->PLL.PLLState = RCC_PLL_ON;
      success = Pll_Set( &o->PLL, SYSCLK_PLL) == PLL_CONFIG_OK;
      if ( success ) success = Pll_Start(SYSCLK_PLL) == PLL_CONFIG_OK;
    }
    
    return success;
}

/**********************************************************************************
 * Do a clock frequency transition. Depending from rising or lowering the clock frq,
 * the steps hasve to be done in different order
 * - Switching On the desired oscillator/PLL is done first
 * - when rising the clock frequency
 *    1. Increase Vcore if neccessary
 *    2. Change Clock, Prescalers and flash waitstates ( consistency is guaranteed by HAL )
 * - when decreasing
 *    1. Change Clock, Prescalers and flash waitstates ( consistency is guaranteed by HAL )
 *    2. Decrease Vcore
 **********************************************************************************/
static void   DoClockTransition ( uint32_t new_khz, RCC_OscInitTypeDef *o, RCC_ClkInitTypeDef *clk, uint32_t flash_latency, uint32_t reg_scale, int32_t err_code)
{
    uint32_t old_khz = HAL_RCC_GetSysClockFreq()/1000;

    int32_t from, to, step;

    /* Configure new SYSCLK provider first. */
    if (!SetupSysClkProvider(o, clk)) Error_Handler_XX(err_code, __FILE__, __LINE__); 
  
    /* Then do the transition */
    if ( old_khz < new_khz ) {
        from = 1; to = 3; step = 1;
    } else {
        from = 2; to = 0; step = -1;
    }
    
    for ( int32_t i = from; i != to; i = i+step ) {
        switch ( i ) {
           case 1:
              /* Adjust Vcore */
              __HAL_PWR_VOLTAGESCALING_CONFIG(reg_scale);
              while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY));
              break;
           case 2:
              if (HAL_RCC_ClockConfig(clk, flash_latency)!= HAL_OK) 
                 Error_Handler_XX(err_code, __FILE__, __LINE__); 
              break;
        }
    }

  /*activate CSI clock mandatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE() ;

  step = __HAL_RCC_SYSCFG_IS_CLK_DISABLED();
  /* Enable SYSCFG clock temporarily for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();

  /* Switch of SYSCFG clock agian, if it was off before */
  if ( step )   __HAL_RCC_SYSCFG_CLK_DISABLE() ;

}

#if defined(HW_HAS_HSE)
    /******************************************************************************
     * Configure HSE clock on
     * Be sure RCC_OscInitStruct has been zeroed before call
     *****************************************************************************/
    static void ConfigureHSE(RCC_OscInitTypeDef *RCC_OscInitStruct)
    {
      /* Enable HSE Oscillator */
      RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSE;
      #if defined(HW_HAS_HSE_CRYSTAL)
        RCC_OscInitStruct->HSEState = RCC_HSE_ON;
      #elif defined(HW_HAS_HSE_BYPASS)
        RCC_OscInitStruct->HSEState = RCC_HSE_BYPASS;
      #else
        #error "Undefined HSE oscillator type"
      #endif
      RCC_OscInitStruct->PLL.PLLState = RCC_PLL_NONE;
    }
#endif

/******************************************************************************
 * Configure HSI clock
 * Be sure RCC_OscInitStruct has been zeroed before call
 *****************************************************************************/
static void ConfigureHSI(RCC_OscInitTypeDef *RCC_OscInitStruct, uint32_t hsi_khz)
{
    RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct->PLL.PLLState = RCC_PLL_NONE;

    RCC_OscInitStruct->HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;  

    switch ( hsi_khz ) {
    case 8000: 
        RCC_OscInitStruct->HSIState = RCC_HSI_DIV8;
        break;
    case 16000: 
        RCC_OscInitStruct->HSIState = RCC_HSI_DIV4;
        break;
    case 32000: 
        RCC_OscInitStruct->HSIState = RCC_HSI_DIV2;
        break;
    case 64000: 
        RCC_OscInitStruct->HSIState = RCC_HSI_DIV1;
        break;
    default:
        Error_Handler_XX(-9, __FILE__, __LINE__); 
        return; 
    }
}

/******************************************************************************
 * Switch HSI clock off
 *****************************************************************************/
static void SwitchOffHSI(RCC_OscInitTypeDef *osc)
{
    memset(osc, 0, sizeof(RCC_OscInitTypeDef));
    
    osc->OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc->HSIState = RCC_HSI_OFF;
    if (HAL_RCC_OscConfig(osc)!= HAL_OK)
    {
        Error_Handler_XX(-6, __FILE__, __LINE__ );
    }
}

/******************************************************************************
 * Switch PLL  off
 *****************************************************************************/
static void SwitchOffSysClkPll(void)
{
    if ( Pll_Stop(SYSCLK_PLL) != PLL_CONFIG_OK ) {
        Error_Handler_XX(-4, __FILE__, __LINE__ );
    }
}


/* Public functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration on MSI-Base
  *         The system Clock is configured as follow : 
  *            System Clock source            = HSI
  *            SYSCLK(Hz)                     = 8, 16, 32 or 64 MHz
  *            HCLK(Hz)                       = 8, 16, 32 or 64 MHz
  *            AHB Prescaler                  = 1
  *            APBx Prescaler                 = 1
  *            Flash Latency(WS)              = depending from SYSCLK
  *            Main regulator output voltage  = Range 3
  * @param hsi_khz -  one of 8000,16000,32000,640000 
  * @retval None
  */
static void SystemClock_HSI_VOSrange_3(uint32_t hsi_khz)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* 
    * if HSI is current CLK source, it has to be deactivated before changes may be applied
    * so switch to HSE temporarily. This can be done without changing VOS range
    * or flash latency, because max HSE frq will meet the VOS and latency settings of current HSI
    */
    if ( __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI ) {
      ConfigureHSE(&RCC_OscInitStruct);
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
          Error_Handler_XX(-5, __FILE__, __LINE__); 
     RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
     RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
     RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
     if ( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK )
          Error_Handler_XX(-5, __FILE__, __LINE__); 
      memset(&RCC_OscInitStruct,0, sizeof(RCC_OscInitTypeDef));
      memset(&RCC_ClkInitStruct,0, sizeof(RCC_ClkInitTypeDef));
    }
    /* 
     * At this "low" frequencies AHB clock frq = sysclk, so we can safely use sysclk 
     * as flash clock parameter for GetFlashLatency()
     */
    uint32_t flash_latency=GetFlashLatency(PWR_REGULATOR_VOLTAGE_SCALE3, hsi_khz);

    /* configure HSI Oscillator */
    ConfigureHSI(& RCC_OscInitStruct, hsi_khz);

    SetPredividers( &RCC_ClkInitStruct, hsi_khz );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    DoClockTransition ( hsi_khz, &RCC_OscInitStruct, &RCC_ClkInitStruct, flash_latency, PWR_REGULATOR_VOLTAGE_SCALE3, -6);
  
    /* Switch off PLL in any case */
    SwitchOffSysClkPll();

   /* Select HSI as WakeUp clock */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
}




/* HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- 
 * The following #ifdef-block requires an HSE oscillator being functional, either
 * as original crystal oscillator or being supplied as external clock
 * HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --- HSE --*/

#if defined(HW_HAS_HSE)
    /**************************************************************************
     * The initialization part that has to be restored after wakeup from stop 
     * only available, if an HSE oscillator is euqipped
     *************************************************************************/
    static void SystemClock_HSE_xxMHz_VOSrange_3_0WS_restore ( uint32_t xxmhz, bool bSwitchOffHSI)
    {
      UNUSED(xxmhz);
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};

      /* Restart HSE after Stop */
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      #if defined(HW_HAS_HSE_CRYSTAL)
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
      #else 
        /* Bypass */
        RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
      #endif
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
         Error_Handler_XX(-12, __FILE__, __LINE__); 

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
      RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
      if ( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, saved_latency ) != HAL_OK )
         Error_Handler_XX(-12, __FILE__, __LINE__); 

      if ( bSwitchOffHSI ) {
        SwitchOffHSI(&RCC_OscInitStruct);
      }
    }

    /**************************************************************************
     * @brief  System Clock Configuration on 8MHz Clock input
     *         The system Clock is configured as follow : 
     *            System Clock source            = xx MHz Signal HSE Bypass mode or oscillator mode, depending from crystal
     *                                              allowed range for LSE is 4 .. 45 MHz
     *            SYSCLK(Hz)                     = xxMHz
     *            HCLK(Hz)                       = xxMhZ
     *            AHB Prescaler                  = 1
     *            APB1 Prescaler                 = 1
     *            APB2 Prescaler                 = 1
     *            Flash Latency(WS)              = 1
     *            Main regulator output voltage  = VOS range 3
     * @param  bSwitchOffHSI - Set to TRUE, if MSI Clock should be switched off. When going to sleep, MSI clock
     *         MUST be activated again, BEFORE going to sleep
     * @note the WakeUp Clock will be HSI, so user has to take actions to switch off to LSE after wakeup
     * @note only available, if HSE oscillator is euqipped
     * @retval None
     *************************************************************************/
    static void SystemClock_HSE_xxMHz_VOSrange_3_0WS( bool bSwitchOffHSI )
    {
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
      RCC_OscInitTypeDef osc = {0};

      ConfigureHSE(&osc);

      SetPredividers( &RCC_ClkInitStruct, HW_HSE_FREQUENCY/1000 );
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;

      /* 
       * At this "low" frequencies AHB clock frq = sysclk, so we can safely use sysclk 
       * as flash clock parameter for GetFlashLatency()
       */
      uint32_t flash_latency=GetFlashLatency(PWR_REGULATOR_VOLTAGE_SCALE3, HW_HSE_FREQUENCY/1000);
      DoClockTransition(HW_HSE_FREQUENCY/1000, &osc, &RCC_ClkInitStruct, flash_latency, PWR_REGULATOR_VOLTAGE_SCALE3, -5);

      /* Disable HSI Oscillator, if desired */
      if ( bSwitchOffHSI ) {
        SwitchOffHSI(&osc);
        /* CSI as Wakeup clock */
        SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      } else {
        /* HSI as Wakeup clock */
        CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
      }

      /* Switch off PLL in any case */
      SwitchOffSysClkPll();

      /* LSE has to be restored after stop */
      bClockSettingVolatile = true;         
      saved_khz             = HW_HSE_FREQUENCY/1000;
      saved_latency         = flash_latency;
      saved_vosrange        = 3;
      RestoreFn             = SystemClock_HSE_xxMHz_VOSrange_3_0WS_restore;
    }

    static void SwitchOffHSE(void)
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};

        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSIState = RCC_HSE_OFF;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
        {
          Error_Handler_XX(-5, __FILE__, __LINE__ );
        }
    }
#endif




/*
 * The initialization part that has to be restored after wakeup from stop 
 */
static void SystemClock_PLL_xxMHz_Vrange_01_restore (uint32_t xxkhz, bool bSwitchOffHSI)
{
  UNUSED(xxkhz);
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Restart PLL1 after Stop */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
     Error_Handler_XX(-12, __FILE__, __LINE__); 

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  if ( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, saved_latency ) != HAL_OK )
     Error_Handler_XX(-12, __FILE__, __LINE__); 

  if ( bSwitchOffHSI ) {
    SwitchOffHSI(&RCC_OscInitStruct);
  }
}

/**
  * @brief  System Clock Configuration on 64-480 PLL-Clock, basing on 16MhZ HSI or 8MHz HSE
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL Clock
  *            PLL input                      = 16MHz HSI RC-oscillator
  *            SYSCLK(Hz)                     = as desired, between 64 and 480 MHz ( maximum specified )
  *            HCLK(Hz)                       = as desired, between 32 and 240 MHz ( maximum specified )
  *            AHB Prescaler                  = so, that resulting AHB frq is at or below 240MHz
  *            APBx Prescaler                 = so, that resulting APB frq is at or below 120MHz
  *            Flash Latency(WS)              = depending from xxmhz, between 0 and 4
  *            Main regulator output voltage  = VOS range1 or VOS range0, when xxmhz > 225MHz
  *
  * 
  * @param  bUseHSE, true when HSE is used as PLL input, false for HSI input
  * @param  bSwitchOffOther true, if other HSx source is to be switched off
  *         Notice: User has to take care to possibly switch on HSx again before going to sleep
  * @note   Algorithm only works for desired SYSCLK frq between 64 and 480 MHz!
  */
static void SystemClock_PLL_xxxMHz_Vrange_01(uint32_t pll_khz, bool bUseHSE, bool bSwitchOffOther)
{
  #define PLL_HSI_BASE_FRQ_KHZ      16000
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pll_inp_khz;
  bool bBaseClkSet = false;         /* flag for "PLL base clock has been set"   */

  /* Check constraints */
  if ( pll_khz > 480000 || pll_khz < 64000) {
    Error_Handler_XX(-9, __FILE__, __LINE__);
  }

  /* 
   * if PLL is currently in use, it has to be deactivated before changes may be applied
   * so select PLL input clock as SYSCLK temporarily. No constraint violation to VOS range 
   * or flash latency, because PLL inp clock is always lower or equal to previous PLL output frq
   */
  if ( __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL1 ) {
     if ( bUseHSE ) {
        #if defined(HW_HAS_HSE)
            ConfigureHSE(&RCC_OscInitStruct);
            pll_inp_khz = HW_HSE_FREQUENCY/1000;
            RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
        #else
            Error_Handler_XX(-5, __FILE__, __LINE__);       
        #endif
     } else { 
        ConfigureHSI(&RCC_OscInitStruct, PLL_HSI_BASE_FRQ_KHZ);
        pll_inp_khz = PLL_HSI_BASE_FRQ_KHZ;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
     }

     if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) Error_Handler_XX(-6, __FILE__, __LINE__); 

     /* Set all clocks and predividers according to that intermediate low and safe frequewncy */
     SetPredividers(&RCC_ClkInitStruct, pll_inp_khz);
     if ( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK ) Error_Handler_XX(-6, __FILE__, __LINE__); 
     
     /* Stop PLL */
     Pll_Stop(SYSCLK_PLL);

     /* clear Osc and Clk init structures after use */
     memset(&RCC_OscInitStruct,0, sizeof(RCC_OscInitTypeDef));
     memset(&RCC_ClkInitStruct,0, sizeof(RCC_ClkInitTypeDef));
     /* PLL base clock is set already to final values */
     bBaseClkSet = true;
  }

  uint32_t ahb_clock_khz = SetPredividers( &RCC_ClkInitStruct, pll_khz );

  /* Set VOS range to 0 when ahb clock > 225 MHz */
  /* Since nucleo board has SMPS only support, VOS scale 0 is inibited */
  /* For lower frequencies, select scale 2, scake 1 only for high requencies */
  #if defined(STM32H745xx)
      /* Set VOS range to 0 when ahb clock > 225 MHz */
      /* Since nucleo board has SMPS only support, VOS scale 0 is inibited */
      /* For lower frequencies, select scale 2, scake 1 only for high requencies */
      uint32_t vosrange = ( ahb_clock_khz > 225000 ? PWR_REGULATOR_VOLTAGE_SCALE1 : PWR_REGULATOR_VOLTAGE_SCALE2 );
  #elif defined(STM32H742xx) || defined(STM32H743xx)
      /* Set VOS range to 0 when ahb clock > 225 MHz */
      /* Since nucleo board has SMPS only support, VOS scale 0 is inibited */
      /* For lower frequencies, select scale 2, scale 1 only for high requencies */
      uint32_t vosrange = ( ahb_clock_khz > 225000 ? PWR_REGULATOR_VOLTAGE_SCALE0 :  ahb_clock_khz > 140000 ? PWR_REGULATOR_VOLTAGE_SCALE1 : PWR_REGULATOR_VOLTAGE_SCALE2 );
  #else
      #error "No VOS range determination defined"
  #endif
  /* calculate neccessary minimum flash latency */
  uint32_t flash_latency = GetFlashLatency(vosrange, ahb_clock_khz);

  /* configure PLL input clock */
  if ( bUseHSE ) {
    #if defined(HW_HAS_HSE)
        /* Don't set base clock again, if already set */
        if (!bBaseClkSet) ConfigureHSE(&RCC_OscInitStruct);
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        pll_inp_khz = HW_HSE_FREQUENCY / 1000;
    #else
        Error_Handler_XX(-5, __FILE__, __LINE__);       
    #endif    
  } else {
        /* Don't set base clock again, if already set */
        if (!bBaseClkSet) ConfigureHSI(&RCC_OscInitStruct, PLL_HSI_BASE_FRQ_KHZ);
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        pll_inp_khz = PLL_HSI_BASE_FRQ_KHZ;
  }

  PLL_Configure_SYSCLK(&RCC_OscInitStruct, pll_khz, pll_inp_khz);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  
  DoClockTransition(pll_khz,&RCC_OscInitStruct, &RCC_ClkInitStruct, flash_latency,vosrange, -4);


  /* Disable MSI Oscillator, if desired */
  if ( bSwitchOffOther ) {
    if ( bUseHSE ) {
        SwitchOffHSI(&RCC_OscInitStruct);
        /* CSI as Wakeup clock */
        SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);
    } else {
        #if defined(HW_HAS_HSE)
            SwitchOffHSE();
        #endif
    }
  }

  /* Select HSI also as WakeUp clock */
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK_Msk);

  bClockSettingVolatile = true;         
  saved_khz             = pll_khz;
  saved_latency         = flash_latency;
  saved_vosrange        = vosrange;
  RestoreFn             = SystemClock_PLL_xxMHz_Vrange_01_restore;
}

/* Only use "SystemClock_SetConfiguredClock" to change system clock            */
/* THis will ensure, that all devices will be noticed about clock              */
/* changes                                                                     */
/*******************************************************************************
 * Do all the clock setting on a configuration byte basis
 *******************************************************************************/
void SystemClock_Set(CLK_CONFIG_T clk_config_byte, bool bSwitchOffMSI )
{
    /* Save desired settings for restoration after Stop */
    saved_bSwitchOffMSI   = bSwitchOffMSI;

    /* 
     * Initially assunm, that there is no restoration needed
     * In general, all clock settings on basis of HSI are non volatile
     * and do not need a restoration, whereas all other clock settings 
     * ( eg. HSE or PLL are volatile and need a restauration 
     */
    bClockSettingVolatile = false;         
    RestoreFn             = NULL;

    /* Inform about outstanding frq change */
    DevicesBeforeFrqChange();

    switch ( clk_config_byte ) {
        case  CLK_HSI_VRNG3_08MHZ_0WS:
            SystemClock_HSI_VOSrange_3(8000);
            break;
        case  CLK_HSI_VRNG3_16MHZ_0WS:
            SystemClock_HSI_VOSrange_3(16000);
            break;
        case  CLK_HSI_VRNG3_32MHZ_0WS:
            SystemClock_HSI_VOSrange_3(32000);
            break;
        case  CLK_HSI_VRNG3_64MHZ_1WS:
            SystemClock_HSI_VOSrange_3(64000);
            break;
/*------------------------------------------------------------------------------*/ 
/* Note: Do not use PLL with HSI as input at higher frequencies, this will lead */
/*       to unpredictable results / hanging system                              */
/*------------------------------------------------------------------------------*/ 
        case  CLK_PLL_VRNG1_100MHZ_1WS:
            SystemClock_PLL_xxxMHz_Vrange_01(100000, true, false);
            break;
        case  CLK_PLL_VRNG1_200MHZ_2WS:
            SystemClock_PLL_xxxMHz_Vrange_01(200000, true, false);
            break;
        case  CLK_PLL_VRNG1_300MHZ_2WS:
            SystemClock_PLL_xxxMHz_Vrange_01(300000, true, false);
            break;
        case  CLK_PLL_VRNG1_400MHZ_3WS:
            SystemClock_PLL_xxxMHz_Vrange_01(400000, true, false);
            break;
        case  CLK_PLL_VRNG0_480MHZ_4WS:
            SystemClock_PLL_xxxMHz_Vrange_01(480000, true, false);
            break;
#if defined(HW_HAS_HSE)
        case  CLK_HSE_VRNG3_xxMHZ_0WS:
            SystemClock_HSE_xxMHz_VOSrange_3_0WS(false);
#endif
    } // Case

    /* Update the system clock variables */
    SystemCoreClockUpdate();

    /* Notice all devices about frequency change                             */
    /* Momentarily, the frq change is not reverted, if any device refuses to */
    /* change frq. this has to be changed for correct implementation         */
    /* Eg by asking before frq change takes place                            */
    if ( DevicesInhibitFrqChange() ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("ERR: One or more devices did not agree to new frequency!");
        #endif
    }

    /* Notify all registered callbacks **** C001 **** */
    ClockNotifyCallbacks(HAL_RCC_GetSysClockFreq());

    #if DEBUG_MODE > 0
        uint32_t sys_real = HAL_RCC_GetSysClockFreq();
        DEBUG_PRINTF("SYSCLK nom. %d.%06dMHz\n",sys_real/1000000,sys_real%1000000 );
        uint32_t sysclk = Get_SysClockFrequency();
        DEBUG_PRINTF("SYSCLK real %d.%06dMHz\n",sysclk/1000000,sysclk%1000000);
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

/* LSE --- LSE --- LSE --- LSE --- LSE --- LSE --- LSE --- LSE --- LSE --- LS*/
 /*****************************************************************************
  * @brief  Configure LSE
  * @param  bLSEon - switch LSE on or off
  * @param  bUseAsRTCClock - configure RTC to use LSE as clocksource
  ****************************************************************************/
void LSEClockConfig(bool bLSEon, bool bUseAsRTCClock)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /*Switch LSE on or OFF */
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = bLSEon ? RCC_LSE_ON : RCC_LSE_OFF;

  /*
   * All platform except DevEBox H7 will run with lowest driving strength
   */
  if ( bLSEon ) {
      #if defined(STM32H7_DEVEBOX )
          __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
      #else
          /* Set to lowest driving strngth */
          __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
      #endif
  }

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      Error_Handler_XX(-11, __FILE__, __LINE__);
  
 
  /* Set LSE as RTC clock source, if desired */
  if ( bUseAsRTCClock )  {
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
      PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
          Error_Handler_XX(-11, __FILE__, __LINE__);
  }

}

void SetPeripheralClkSource( uint32_t src )
{
    if ( src < 3 ) {
        MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_CKPERSEL_Msk, src << RCC_D1CCIPR_CKPERSEL_Pos);
    } else {
        DEBUG_PUTS("Error: SetPeripheralClkSource -illegal source");
    }
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
  
}

/******************************************************************************
 * the following two functions are added due to **** C001 ****
 *****************************************************************************/
/******************************************************************************
 * @brief Register a clock change callback
 * @param changeCB callback function to be notified on clock changes
 * @returns 0  on success
 *          -1 ERR_: no more room to register callback
 *          -2 error: callback must no be NULL
 * @note Once registered a function, it can never be unregistered again
 *****************************************************************************/
int32_t ClockRegisterForClockChange ( ClockChangeCB changeCB )
{
    /* Space left in array ? */
    if ( numClkchangeCB >= MAX_CLKCHG_CB - 1 ) return -1;

    /* CB must not be NULL */
    if ( changeCB == 0 ) return -2;

    clkCB[numClkchangeCB++] = changeCB;
    return 0;
}

/******************************************************************************
 * @brief Notify all registered callbacks on Clock change
 * @param newclk  new clock frequncy in Hz
 *****************************************************************************/
void   ClockNotifyCallbacks(uint32_t newclk)
{
    for ( int32_t i = 0; i < numClkchangeCB; i++ ) 
        clkCB[i](newclk);
}



/******************************************************************************
 * Do a Calibration of the HSI clock 
 * To do this, the LSE clock has to be on. SYSCLK will be switched to
 * HSI temporarily
 * TMR15 is used to perform the calibration
 *****************************************************************************/

#include "debug_helper.h"

#define __TIMx_CLK_ENABLE()        __HAL_RCC_TIM15_CLK_ENABLE()
#define __TIMx_CLK_DISABLE()       __HAL_RCC_TIM15_CLK_DISABLE()
#define  TIMx                       TIM15
#define  TIM_CHANNEL_y              TIM_CHANNEL_1
#define  HAL_TIM_ACTIVE_CHANNEL_y   HAL_TIM_ACTIVE_CHANNEL_1
#define  TIM_TIMx_GPIO              TIM_TIM15_TI1_GPIO
#define  TIM_TIMx_LSE               TIM_TIM15_TI1_RCC_LSE
#define  TIMx_IRQn                  TIM15_IRQn

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
                  ((RCC->HSICFGR & RCC_HSICFGR_HSITRIM_Msk) >> RCC_HSICFGR_HSITRIM_Pos)

/******************************************************************************
  * @brief  This function handles TIM15 interrupt request 
  * (for HSI16 calibration).
  * @param  None
  * @retval None
  ****************************************************************************/
void TIM15_IRQHandler(void)
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

  /* 
   * up to here we measured the timer clock domain frequency ( either APB1 or APB2 domain )
   * to get the SYSCLK frequency, we have to take the APBx ad AHB prescaler values into consideration, too
   */
  return measured_frequency * TmrGetClockPrescaler(TIMx) * GetAHBPrescaler();
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
    //HAL_TIMEx_RemapConfig(&TimHandle, TIM_TIMx_LSE);     
    HAL_TIMEx_TISelection(&TimHandle, TIM_TIMx_LSE, TIM_CHANNEL_y);

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

    /* switch to HSI 64MHz, Vrange1, 1 WS*/
    SystemClock_Set(CLK_HSI_VRNG3_64MHZ_1WS, false);

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
 * 1 = HSI
 * 2 = LSE
 * 3 = HSE
 * 4 = PLL1Q
 * 5 = HSI48
 * 7 = LSI
 *****************************************************************************/
void EnableMCO ( uint32_t mcoSource )
{
    uint32_t halParam;
    switch ( mcoSource ) {
        case 0:
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
            return;
        case 1: halParam = RCC_MCO1SOURCE_HSI; break;
        case 2: halParam = RCC_MCO1SOURCE_LSE; break;
        case 3: halParam = RCC_MCO1SOURCE_HSE; break;
        case 4: halParam = RCC_MCO1SOURCE_PLL1QCLK; break;
        case 5: halParam = RCC_MCO1SOURCE_HSI48; break;
        default:
            return;
    }
    HAL_RCC_MCOConfig(RCC_MCO1, halParam, RCC_MCODIV_1);
}

#if USE_USB > 0
  /* Enable the HSI48 clock and synchronize with LSE clock
   * HSI48 will be used as source 
   * The HSI48 RC can be switched on and off using the HSI48ON bit in the
   * Clock control register (RCC_CRRCR).
   *
   * The USB clock may be derived from either the PLL clock or from the
   * HSI48 clock.  This oscillator will be also automatically enabled (by
   * hardware forcing HSI48ON bit to one) as soon as it is chosen as a clock
   * source for the USB and the peripheral is
   * enabled.
   */
void stm32h7_enable_hsi48(void)
{
  uint32_t regval;

  /* Enable the HSI48 clock.
   *
   * The HSI48 RC can be switched on and off using the HSI48ON bit in the
   * Clock control register (RCC_CRRCR).
   *
   * The USB clock may be derived from either the PLL clock or from the
   * HSI48 clock.  This oscillator will be also automatically enabled (by
   * hardware forcing HSI48ON bit to one) as soon as it is chosen as a clock
   * source for the USB and the peripheral is
   * enabled.
   */

  SET_BIT(RCC->CR, RCC_CR_HSI48ON);

  /* Wait for the HSI48 clock to stabilize */
  while ( READ_BIT(RCC->CR, RCC_CR_HSI48RDY) == 0 );


  while ((RCC->CR & RCC_CR_HSI48RDY) == 0);

  #define CRS_CFGR_SYNCSRC_USB2SOF        0b00
  #define CRS_CFGR_SYNCSRC_LSE            (0b01 << CRS_CFGR_SYNCSRC_Pos)
  #define CRS_CFGR_SYNCSRC_OTGHS1SOF      (0b10 << CRS_CFGR_SYNCSRC_Pos)

  __HAL_RCC_CRS_CLK_ENABLE();

  /* Set RELOAD and FELIM value according to Refman 9.5.4 */
  /* Ftarget = 48 MHz, Fsync = 32768Hz */
  #define QUOTIENT ((48000000+16384)/32788)
  #define RELOAD_VAL (QUOTIENT - 1)
  /* Trimming step size is 0,14% */
  #define FELIM_VAL  (QUOTIENT*140 + 10000 ) / 20000

  /* Enable synchronization with LSE */
  regval = READ_REG(CRS->CFGR); 
  MODIFY_REG(regval, CRS_CFGR_SYNCSRC_Msk,  CRS_CFGR_SYNCSRC_LSE);
  MODIFY_REG(regval, CRS_CFGR_RELOAD_Msk,  RELOAD_VAL << CRS_CFGR_RELOAD_Pos );
  MODIFY_REG(regval, CRS_CFGR_FELIM_Msk,   FELIM_VAL << CRS_CFGR_FELIM_Pos );
  WRITE_REG(CRS->CFGR, regval);

  /* Set the AUTOTRIMEN bit the CRS_CR register to enables the automatic
   * hardware adjustment of TRIM bits according to the measured frequency
   * error between the selected SYNC event. Also enable CEN bit to enable
   * frequency error counter and SYNC events.
   */

  SET_BIT(CRS->CR, CRS_CR_AUTOTRIMEN | CRS_CR_CEN );  
}

/****************************************************************************
 * Name: stm32l4_disable_hsi48
 *
 * Description:
 *   Disable the HSI48 clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32h7_disable_hsi48(void)
{

  /* Disable the HSI48 clock and autotrim */
  CLEAR_BIT(CRS->CR, CRS_CR_AUTOTRIMEN | CRS_CR_CEN );  
  __HAL_RCC_CRS_CLK_DISABLE();
  CLEAR_BIT(RCC->CR, RCC_CR_HSI48ON);

}

#endif

/**
  * @}
  */


