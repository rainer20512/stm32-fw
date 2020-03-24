/*
 ******************************************************************************
 * @file    timer_dev.h 
 * @author  Rainer
 * @brief   timer device, only device functions, nothing else 
 *
 *
 *****************************************************************************/

/** @addtogroup Timer device
  * @{
  */

#include "config/config.h"

#if USE_PWMTIMER > 0 || USE_BASICTIMER > 0

#include "error.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"

#include "config/devices_config.h"
#include "config/timer_config.h"
#include "dev/devices.h"
#include "debug_helper.h"

/*******************************************************************************************
 * Additional data that will be stored to I2C type hardware devices
 ******************************************************************************************/

static uint32_t idxToTimCh[6] = 
    { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
      TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6, 
    };

typedef struct {
    TimerHandleT    *myTmrHandle;   /* My associated Spi-handle */
    uint32_t        base_frq; /* Timer Base Frequency */
} TMR_AdditionalDataType;

static TMR_AdditionalDataType * TMR_GetAdditionalData(const HW_DeviceType *self)
{
    return (TMR_AdditionalDataType *)(self->devData);
}


/******************************************************************************
 * Returns true, if any PWM channel is active
 *****************************************************************************/
static bool IsAnyChnActive( const HW_DeviceType *self)
{
    TIM_TypeDef *htim   = (TIM_TypeDef *)self->devBase;
        
    /* Check, if any of the enable bits in CCER are set */
    uint32_t mask = 0xbfff;
    return htim->CCER & mask;
}     

#if USE_BASICTIMER > 0 
    void BASTMR_IrqHandler( volatile uint32_t *CntHigh, TIM_TypeDef *htim)
    {
      /* Reset interrupt flag by writing 0 to it */
      htim->SR = ~TIM_SR_UIF;

      /* 
       * Increment has to be done before profiling, because profiling will access
       * the microsecond counter
       */
      (*CntHigh)++;
      ProfilerPush(JOB_IRQ_PROFILER);
      /* Increment Counter Hi Word */

      /* Increment Counter Hi Word */
      ProfilerPop();
    }

    uint16_t BASTMR_GetRawValue(TimerHandleT *hnd)
    {
        register TIM_TypeDef *htim = hnd->myHalHnd.Instance;
        register uint16_t cnt;
        /* Don't use counter values near the upper limit, wait for next period */
        do {  
            cnt = htim->CNT;
        } while ( cnt >= 0xfff8 );

        /* 
        * Execute interrupt, if pending. Ths can be the case, when profiler
        * is called from hihger priority interrupt routine
        */
        if (  htim->SR & TIM_SR_UIF ) BASTMR_IrqHandler(&hnd->MicroCountHigh, htim);

        return htim->CNT;
    }

    /**********************************************************************************
     * return the number of microseconds since powerup
     * \note rollover every 71 minutes, use only for delta-measurements
     *********************************************************************************/
    uint32_t BASTMR_GetMicrosecond(TimerHandleT *hnd)
    {
      register uint32_t ret;
      /* 
       * Dont change sequence: First get Raw Counter value ( this might change the 
       * counter high value, thereafter get CounterHigh value
       */
      ret = BASTMR_GetRawValue(hnd);
      return ret | hnd->MicroCountHigh << 16;
    }

    /**********************************************************************************
     * Early Init of BasicTimer Device to enable Profiling even during initialization
     *********************************************************************************/
    void BASTMR_EarlyInit(void)
    {
      int32_t dev_idx;

      /* Init BASTMR device */
      dev_idx = AddDevice(&BASTIM_HW,NULL ,NULL);
      DeviceInitByIdx(dev_idx, NULL);
    }

    /**********************************************************************************
     * Aquire the basic timer instance
     *********************************************************************************/
    void BASTMR_Aquire(TimerHandleT *timhandle)
    {
        timhandle->reference_cnt++;
    }

    /**********************************************************************************
     * Release the basic timer instance
     *********************************************************************************/
    void BASTMR_Release(TimerHandleT *timhandle)
    {
        if ( --(timhandle->reference_cnt) < 0 ) log_error("BASTMR Semaphore Release w/o Aquire");
    }
#endif /* #if USE_BASICTIMER > 0 */

/******************************************************************************
 * DeInit all assigned GPIO PIns
 *****************************************************************************/
static void TMR_GPIO_DeInit(const HW_DeviceType *self)
{
    GpioAFDeInitAll(self->devGpioAF); 
}

/******************************************************************************
 * DeInitialize Timer Handle
 *****************************************************************************/
static void TmrHandleDeInit (TimerHandleT *hnd)
{
    memset(hnd, 0, sizeof(TimerHandleT) );
}
/******************************************************************************
 * Initialize associated Timer Handle
 *****************************************************************************/
static void TmrHandleInit (TimerHandleT *hnd, const HW_DeviceType *self)
{    
    UNUSED(self);
    TmrHandleDeInit(hnd);
    hnd->use_chX[0]         = 0;
    hnd->use_chX[1]         = 0;
    hnd->use_chX[2]         = 0;
    hnd->use_chX[3]         = 0; 
    hnd->reference_cnt      = 0;
    hnd->MicroCountHigh     = 0;
}

/****************************************************************************** 
 * @brief  Return the APB1-Timers input frequency. If APB1 clock is prescaled 
           from             HCLK, the timer input frequency is twice the 
           APB1-Clock. If APB1 clock is equal HCLK, timer input frequency is 
           equal to APB1 clock
 * @param  None
 * @retval Actual APB1 timers input frequency
  *****************************************************************************/
static uint32_t GetAPB1TimerFrequency(void)
{
  
  uint32_t uPclk1 =  HAL_RCC_GetPCLK1Freq();
  uint32_t uPclk1Prescale = (RCC->CFGR & RCC_CFGR_PPRE1_Msk ) >> RCC_CFGR_PPRE1_Pos;
  DEBUG_PRINTF("APB1 clock .........=%d\n", uPclk1);
  DEBUG_PRINTF("APB1 clock prescaler=%d\n", uPclk1Prescale);
  if (uPclk1Prescale==RCC_HCLK_DIV1)
     return uPclk1;
  else 
     return uPclk1*2;
}
static uint32_t GetAPB2TimerFrequency(void)
{
  
  uint32_t uPclk2 =  HAL_RCC_GetPCLK2Freq();
  uint32_t uPclk2Prescale = (RCC->CFGR & RCC_CFGR_PPRE2_Msk ) >> RCC_CFGR_PPRE2_Pos;
  DEBUG_PRINTF("APB2 clock .........=%d\n", uPclk2);
  DEBUG_PRINTF("APB2 clock prescaler=%d\n", uPclk2Prescale);
  if (uPclk2Prescale==RCC_HCLK_DIV1)
     return uPclk2;
  else 
     return uPclk2*2;
}


static const TIM_TypeDef* apb1_timers[]={TIM2, TIM3, TIM4, TIM5,  TIM6,  TIM7  };   /* Timers clocked by APB1 */
static const TIM_TypeDef* apb2_timers[]={TIM1, TIM8,       TIM15, TIM16, TIM17 };   /* Timers clocked by APB2 */
/****************************************************************************** 
 * Return a timers input clock frequency. This can either be the APB1 or
 * APB2 clock.
 * Does not work for LPTIM timers
 * 0 is returned, if the base clock cannot be determined
  *****************************************************************************/
static uint32_t TmrGetClockFrq ( TIM_TypeDef *tim )
{
    uint32_t i;
    for ( i=0; i < sizeof(apb1_timers)/sizeof(TIM_TypeDef*); i++ ) {
        if ( tim == apb1_timers[i] ) return GetAPB1TimerFrequency();
    }
    for ( i=0; i < sizeof(apb2_timers)/sizeof(TIM_TypeDef*); i++ ) {
        if ( tim == apb2_timers[i] ) return GetAPB2TimerFrequency();
    }

    return 0;
}

/******************************************************************************
 * Set the timers prescaler to achieve the desired base frequency
 * false is reported, if sysclk frq is too low to achieve this or
 * if sysclk is not dividable to achieve base frq when bExactMatch = true
 *****************************************************************************/
static bool SetBaseFrq( TIM_TypeDef *htim, uint32_t base_frq, uint32_t *new_psc, bool bExactMatch )
{
    bool ret            = true;
    uint32_t new_TmrClk =  TmrGetClockFrq(htim);
    uint32_t prediv     = 1;

    if ( new_TmrClk < base_frq ) {
        DEBUG_PRINTF("TMR ERROR: Input clock of %d lower than base frq of %d\n", new_TmrClk, base_frq );
        ret =  false;
    } else {
        prediv = (new_TmrClk+(base_frq>>1))/base_frq;
        if ( prediv * base_frq != new_TmrClk ) {
            DEBUG_PRINTF("TMR%s: Unable to prescale TmrClk %d to base frq %d\n", (bExactMatch?" ERROR:":""), new_TmrClk, base_frq );
            DEBUG_PRINTF("TMR%s: Resulting base frq is %d\n", (bExactMatch?" ERROR:":""), new_TmrClk/prediv );
            ret =  !bExactMatch;
        }
    }
    *new_psc = prediv-1;
    return ret;
}

/******************************************************************************
 * Initialization of Timer Device. We use the HAL function for this,
 * because later on some other HAL functions will also be used. These require
 * an initialized HAL handle.
 *****************************************************************************/
bool TMR_Init(const HW_DeviceType *self)
{
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
    TIM_TypeDef *htim       = (TIM_TypeDef *)self->devBase;
    TIM_HandleTypeDef *myHnd  =  &adt->myTmrHandle->myHalHnd;

    /* Enable TMR clock */
    HW_SetHWClock((TIM_TypeDef*)self->devBase, 1);

    /* Initialize the associated Handle */
    TmrHandleInit(adt->myTmrHandle, self);

    /* Set base frequency */
    myHnd->Instance = htim;

#if USE_PWMTIMER > 0
    if ( self->devType == HW_DEVICE_PWMTIMER ) {
        SetBaseFrq(htim, adt->base_frq, &myHnd->Init.Prescaler, false);
        /* default as configured */
        myHnd->Init.Period            = adt->base_frq / PWM_FREQUENCY;  
        myHnd->Init.ClockDivision     = 0;
        myHnd->Init.CounterMode       = TIM_COUNTERMODE_UP;
        myHnd->Init.RepetitionCounter = 0;
        return HAL_TIM_PWM_Init(myHnd) == HAL_OK;
    }
#endif
#if USE_BASICTIMER > 0
    bool ret;
    if ( self->devType == HW_DEVICE_BASETIMER ) {
        /* Initialize TIMx peripheral as follows:
            + Period             = 65536
            + Prescaler          = calculated dynamically to give 1 MHz timer input frequency
            + ClockDivision      = 1
            + Counter direction  = Up
        */
        myHnd->Init.Period            = 0xffff;
        if (!SetBaseFrq(htim, adt->base_frq, &myHnd->Init.Prescaler, true) ) return false;
        myHnd->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        myHnd->Init.CounterMode       = TIM_COUNTERMODE_UP;
        myHnd->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

        /* Init Timer */
        ret = HAL_TIM_Base_Init(myHnd) == HAL_OK;
        if (ret && self->devIrqList) {
            HW_SetAllIRQs(self->devIrqList, true);
            ret = HAL_TIM_Base_Start_IT(myHnd) == HAL_OK;
        }

        return ret;
    }
#endif
    return false;
}

/******************************************************************************
 * DeInitialization of timer device
 *****************************************************************************/
void TMR_DeInit(const HW_DeviceType *self)
{
    /*Reset peripherals */
    HW_Reset((TIM_TypeDef *)(self->devBase) );

    /* Disable TMR clock */
    HW_SetHWClock((TIM_TypeDef*)self->devBase, 0);

    /* Disable GPIOs */
    if ( self->devType == HW_DEVICE_PWMTIMER ) TMR_GPIO_DeInit(self);

    /* Disable Interrupts */
    if ( self->devType == HW_DEVICE_BASETIMER && self->devIrqList ) HW_SetAllIRQs ( self->devIrqList, false );

    /* zero the associated Handle */
    TmrHandleDeInit(TMR_GetAdditionalData(self)->myTmrHandle);


}

/******************************************************************************
 * Set the PWM frequency 
 * an error is returned, if the PWM frq is greater than Base Frq / 20 
 * this is due to shrinking duty cycle options
 *****************************************************************************/
bool TMR_SetPWMFrq (const HW_DeviceType *self, uint32_t frq ) 
{
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
    bool ret = true;
     
    if ( frq > adt->base_frq/2 ) {
        DEBUG_PRINTF("Error: TMR PWM frq must not be greater than %d, no change\n", adt->base_frq/2 );
        return false;
    }

 
    uint32_t arr = adt->base_frq / frq;
    #if DEBUG_MODE > 0
        if (arr < 20 ) {
            DEBUG_PRINTF("Warning: TMR PWM frq allows only %d Duty cycle steps\n", arr);
        }
    #endif

    ((TIM_TypeDef *)self->devBase)->ARR = arr-1;
    return ret;
}


/******************************************************************************
 * Init one PWM channel by call of the corresponding HAL function
 * seemed easier to implement than grabbing together all bits by hand 
 *****************************************************************************/
bool TMR_InitPWMCh(const HW_DeviceType *self, uint32_t ch, bool invert )
{
    if ( ch < 1 || ch > 4 ) return false;

   /* configure/enable the corresponding gpio */
    GPIO_InitTypeDef  Init;
    Init.Mode  = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_HIGH;   
    GpioAFInitOne(&self->devGpioAF->gpio[ch-1], &Init );

    TIM_OC_InitTypeDef sConfig;
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self); 
    TIM_TypeDef *inst = (TIM_TypeDef *)adt->myTmrHandle->myHalHnd.Instance;

    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = ( invert ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH );
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    /* PWM initially to 50% ( PWM is not started yet! ) */
    sConfig.Pulse        = inst->ARR / 2;
    
    return HAL_TIM_PWM_ConfigChannel(&adt->myTmrHandle->myHalHnd, &sConfig, idxToTimCh[ch-1]) == HAL_OK;

}

/******************************************************************************
 * Start the PWM with an absolute value for the channels CCR
 * the allowed value for CCR is [ 0 .. ARR ]
 *****************************************************************************/
static void StartPWMAbs( TIM_HandleTypeDef *htim, uint32_t ch, uint32_t absval)
{
    TIM_TypeDef *inst = htim->Instance;

    __IO uint32_t *ccr_reg = &inst->CCR1;
    *(ccr_reg+ch-1) = absval;

    HAL_TIM_PWM_Start( htim, idxToTimCh[ch-1]);
}


/******************************************************************************
 * Start the PWM with a duty cycle of promille
 *****************************************************************************/
void TMR_StartPWMChPromille(const HW_DeviceType *self, uint32_t ch, uint32_t promille)
{
    if ( promille > 1000 )  return;
    if ( ch < 1 || ch > 4 ) return;
  
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self); 
    TIM_HandleTypeDef *hnd = &adt->myTmrHandle->myHalHnd;
    TIM_TypeDef *inst = (TIM_TypeDef *)hnd->Instance;

    uint32_t abs = (inst->ARR+1) * promille;
    /* Round before division */
    abs =  (abs+500) /1000 ;
    if ( abs > 0 ) abs--;
    /* for 100% on, CCR must be greater than ARR */
    if ( promille == 1000 ) abs = (inst->ARR+1);
    StartPWMAbs(hnd, ch, abs );
}

/******************************************************************************
 * Start the PWM with a duty cycle of s256/256
 *****************************************************************************/
void TMR_StartPWMChS256(const HW_DeviceType *self, uint32_t ch, uint32_t s256 )
{
    if ( s256 > 256 ) return;
    if ( ch < 1 || ch > 4 ) return;

    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self); 
    TIM_HandleTypeDef *hnd = &adt->myTmrHandle->myHalHnd;
    TIM_TypeDef *inst = (TIM_TypeDef *)hnd->Instance;

    uint32_t abs = (inst->ARR+1) * s256;
    /* Round before division */
    abs =  (abs+128) /256 ;
    if ( abs > 0 ) abs--;
    /* for 100% on, CCR must be greater than ARR */
    if ( s256 == 256 ) abs = (inst->ARR+1);

    StartPWMAbs(hnd, ch, abs );
}

/******************************************************************************
 * Stop one PWM channel and deactivate the GPIO
 *****************************************************************************/
void TMR_StopPWMCh(const HW_DeviceType *self, uint32_t ch)
{
    if ( ch < 1 || ch > 4 ) return;

    TIM_HandleTypeDef *htim = &TMR_GetAdditionalData(self)->myTmrHandle->myHalHnd;
    HAL_TIM_PWM_Stop(htim, idxToTimCh[ch-1]);

    /* Deactivate GPIO */
    GpioAFDeInitOne(&self->devGpioAF->gpio[ch-1]);

    /* If no other PWM channel is active, disable TMR */
    if ( !IsAnyChnActive(self) ) CLEAR_BIT( htim->Instance->CR1, TIM_CR1_CEN );
}

/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool TMR_AllowStop(const HW_DeviceType *self)
{
    if ( self->devType == HW_DEVICE_BASETIMER ) {
        const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
        return adt->myTmrHandle->reference_cnt == 0;
    } else
        return (!IsAnyChnActive(self));
}




/******************************************************************************
 * React on SYSCLK frequency changes
 *****************************************************************************/
bool TMR_OnFrqChange(const HW_DeviceType *self)
{
    bool ret;
    uint32_t new_psc;
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
    TIM_HandleTypeDef *hnd = &adt->myTmrHandle->myHalHnd;
    TIM_TypeDef *inst = (TIM_TypeDef *)hnd->Instance;

    ret = SetBaseFrq(inst, adt->base_frq, &new_psc, self->devType==HW_DEVICE_BASETIMER);
    if ( ret ) inst->PSC = new_psc;
    return ret;
}

// RHB todo TIM6 as basetimer in analogy to TIM7

#if defined(USE_TIM7) && defined(TIM7)
    /**************************************************************************
     * This timer is primarily used for Profilimg and pulse capture of OOK
     * sequencer. Its timer frequency is fixed to exactly 1 MHz
     *************************************************************************/
    TimerHandleT         TIM7Handle;

    static const TMR_AdditionalDataType additional_tim7 = {
        .myTmrHandle = &TIM7Handle,
        .base_frq   = 1000000,          // Base Timers always run with 1MHz
    };

    const HW_IrqList irq_tim7 = {
        .num = 1,
        .irq = { TIM7_IRQ },
    };

    const HW_DeviceType HW_TIM7 = {
        .devName        = "BASETIM7",
        .devBase        = TIM7,
        .devGpioAF      = NULL,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_BASETIMER,
        .devData        = &additional_tim7,
        .devIrqList     = &irq_tim7,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = TMR_Init,
        .DeInit         = TMR_DeInit,
        .OnFrqChange    = TMR_OnFrqChange,
        .AllowStop      = TMR_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif /* TIM7 */

#if defined(USE_TIM1) && defined(TIM1)
    TimerHandleT         TIM1Handle;

    static const HW_GpioList_AF gpio_tim1 = {
        /* CH1 .. CH4 must be specified, will only be initialized when used */
        .gpio = { 
             TIM1_CH1,
             TIM1_CH2,
             TIM1_CH3,
             TIM1_CH4,
        },
        .num = 4, 
    };

    static const TMR_AdditionalDataType additional_tim1 = {
        .myTmrHandle = &TIM1Handle,
        .base_frq   = 8000000,
    };

    const HW_DeviceType HW_TIM1 = {
        .devName        = "TIM1",
        .devBase        = TIM1,
        .devGpioAF      = &gpio_tim1,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PWMTIMER,
        .devData        = &additional_tim1,
        .devIrqList     = NULL,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = TMR_Init,
        .DeInit         = TMR_DeInit,
        .OnFrqChange    = TMR_OnFrqChange,
        .AllowStop      = TMR_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif /* TIM1 */

#if defined(USE_TIM3) && defined(TIM3)
    TimerHandleT         TIM3Handle;

    static const HW_GpioList_AF gpio_tim1 = {
        /* CH1 .. CH4 must be specified, will only be initialized when used */
        .gpio = { 
             TIM3_CH1,
             TIM3_CH2,
             TIM3_CH3,
             TIM3_CH4,
        },
        .num = 4, 
    };

    static const TMR_AdditionalDataType additional_tim1 = {
        .myTmrHandle = &TIM3Handle,
        .base_frq   = 8000000,
    };

    const HW_DeviceType HW_TIM3 = {
        .devName        = "TIM3",
        .devBase        = TIM3,
        .devGpioAF      = &gpio_tim1,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PWMTIMER,
        .devData        = &additional_tim1,
        .devIrqList     = NULL,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = TMR_Init,
        .DeInit         = TMR_DeInit,
        .OnFrqChange    = TMR_OnFrqChange,
        .AllowStop      = TMR_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif /* TIM3 */

#endif /* USE_PWMTIMER > 0 || USE_BASICTIMER > 0 */
/**
  * @}
  */

