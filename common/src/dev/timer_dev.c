/*
 ******************************************************************************
 * @file    timer_dev.c 
 * @author  Rainer
 * @brief   timer device, only device functions, nothing else 
 *
 *
 *****************************************************************************/

/** @addtogroup Timer device
  * @{
  */

#include "config/config.h"

#if USE_HW_PWMTIMER > 0 || USE_BASICTIMER > 0 || USE_PERIPH_TIMER > 0 || USE_USER_PWMTIMER > 0

#include "hardware.h"
#include "error.h"
#include "system/profiling.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"

#include "config/devices_config.h"
#include "config/timer_config.h"
#include "dev/devices.h"
#include "debug_helper.h"

/*******************************************************************************************
 * Additional data that will be stored to timer type hardware devices
 ******************************************************************************************/

typedef struct TUPC {
  uint8_t num;                         /* number of PWM channels used, 0...n */
  uint8_t chn_idx[];                   /* channel idx of used channel        */
} TmrUsdPwmChnls;

typedef struct {
    TimerHandleT    *myTmrHandle;     /* My associated Timer-handle */
    uint32_t        base_frq;         /* Timer Base Frequency       */
    uint32_t        pwm_frq;          /* default pwm frequency      */
    const TmrUsdPwmChnls* pwm_used;   /* list of used pwm channels  */
} TMR_AdditionalDataType;

uint32_t idxToTimCh[6] = 
    { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
      TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6, 
    };


static TMR_AdditionalDataType * TMR_GetAdditionalData(const HW_DeviceType *self)
{
    return (TMR_AdditionalDataType *)(self->devData);
}

TimerHandleT * TMR_GetHandleFromDev(const HW_DeviceType *self)
{
    return TMR_GetAdditionalData(self)->myTmrHandle;
}

/******************************************************************************
 * Returns the predefined values for base and pwm frq from eeprom
 *****************************************************************************/
void TMR_GetPredefBaseAndPwmFrq(const HW_DeviceType *self, uint32_t *base_frq, uint32_t *pwm_frq)
{
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
    *base_frq = adt->base_frq;
    *pwm_frq  = adt->pwm_frq;
}

/******************************************************************************
 * Returns the actual values for base and pwm frq for Timer "self"
 *****************************************************************************/
void TMR_GetActualBaseAndPwmFrq(const HW_DeviceType *self, uint32_t *base_frq, uint32_t *pwm_frq)
{
    TIM_TypeDef *htim = (TIM_TypeDef *)self->devBase;
    uint32_t inp_clk = TmrGetClockFrq(htim);
    *base_frq = inp_clk / (htim->PSC +1);
    uint32_t arr = htim->ARR;
    *pwm_frq = arr ? *base_frq / arr : 0;
}

/******************************************************************************
 * Returns true, if any PWM channel is active
 *****************************************************************************/
bool TMR_IsAnyChnActive( const HW_DeviceType *self)
{
    TIM_TypeDef *htim   = (TIM_TypeDef *)self->devBase;
        
    /* Check, if any of the enable bits in CCER are set */
    return htim->CCER & ( TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC5E | TIM_CCER_CC6E );
}    

/******************************************************************************
 * Returns true, if "ch" is a valid channel number[1..4}] 
 * AND channel is configured for use 
 *****************************************************************************/
bool TMR_CheckValidChn( const HW_DeviceType *self, uint32_t ch )
{
    if ( ch < 1 || ch > 4 ) return false;

    return TMR_GetHandleFromDev(self)->use_chX[ch];
    
}

/******************************************************************************
 * Returns a value ! 0, if channel "ch" is active
 *****************************************************************************/
bool TMR_IsChnActive( const HW_DeviceType *self, uint32_t ch )
{
    if (!TMR_CheckValidChn(self, ch)) return false;

    TIM_TypeDef *inst = TMR_GetHandleFromDev(self)->myHalHnd.Instance;

    return inst->CCER & (TIM_CCER_CC1E << (idxToTimCh[ch-1] & 0x1FU));
}


/**********************************************************************************
 * Aquire a timer instance
 *********************************************************************************/
void TMR_Aquire(TimerHandleT *timhandle)
{
    timhandle->reference_cnt++;
}

/**********************************************************************************
 * Release a timer instance
 *********************************************************************************/
void TMR_Release(TimerHandleT *timhandle)
{
    if ( --(timhandle->reference_cnt) < 0 ) log_error("BASTMR Semaphore Release w/o Aquire");
}

/******************************************************************************
 * Start a timer
 *****************************************************************************/
void TMR_Start(const HW_DeviceType *dev, bool bIntEnable)
{
    TIM_TypeDef *htim         = (TIM_TypeDef*)dev->devBase;

    /* Enable Update interrupt, if requested */
    if ( bIntEnable) {
        htim->SR   = 0;
        htim->DIER = TIM_DIER_UIE;
    }

    /* Start_timer */
    htim->CR1 |= TIM_CR1_CEN;
}

/******************************************************************************
 * Stop a timer
 *****************************************************************************/
void TMR_Stop(const HW_DeviceType *dev)
{
    TIM_TypeDef *htim = (TIM_TypeDef*)dev->devBase;

    /* Stop_timer */
    htim->CR1 &= ~TIM_CR1_CEN;

    /* Disable all interrupts and pending interrupt flags */
    htim->DIER = 0;
    htim->SR   = 0;
}


#if USE_BASICTIMER > 0 

    /* 
     * If Basictimer is used for tick generation, too then set the period to 1ms
     * Otherwise set the period to maximum to ensure maximum sleep time
     */
    #if USE_BASICTIMER_FOR_TICKS > 0
        #define BASTIMER_UPPER      (1000 - 1)
    #else
        #define BASTIMER_UPPER      0xffff
    #endif
    /* 
     * When reading the CNT value near the upper limit, wait for timer reload 
     * interrupt, because it could be chocked otherwise
     */
    #define BASTIMER_HOLDLIMIT      ( BASTIMER_UPPER - 7 )

    void BASTMR_IrqHandler( volatile uint32_t *CntHigh, TIM_TypeDef *htim)
    {
      /* Reset interrupt flag by writing 0 to it */
      htim->SR = ~TIM_SR_UIF;

      /* 
       * Increment has to be done before profiling, because profiling will access
       * the microsecond counter
       */
      #if USE_BASICTIMER_FOR_TICKS > 0
        /* Increment counter by 1000 */
        (*CntHigh) += 1000;
      #else
        /* Increment Counter Hi Word */
        (*CntHigh)++;
      #endif
      ProfilerPush(JOB_IRQ_PROFILER);
      #if USE_BASICTIMER_FOR_TICKS > 0
        /* If also used as tick generator, increment system tick, if not inhibited */
        if (BASTIM_HANDLE.bTicksEnabled ) HAL_IncTick();
      #endif  
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
        } while ( cnt >= BASTIMER_HOLDLIMIT );

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
    
      #if USE_BASICTIMER_FOR_TICKS > 0
        /* When using also as tick generator, MicroCountHigh counts in 1000us steps */
        return ret + hnd->MicroCountHigh;
      #else
        /* When using standalone, MicroCountHigh counts 65536 us steps */
        return ret | hnd->MicroCountHigh << 16;
      #endif
    }

    /**********************************************************************************
     * Do an delay of some microseconds
     *********************************************************************************/
    void BASTMR_DelayUs ( uint32_t delta_us )
    {
        uint32_t start = BASTMR_GetMicrosecond(&BASTIM_HANDLE);

        while ((BASTMR_GetMicrosecond(&BASTIM_HANDLE) - start) < delta_us);
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

#endif /* #if USE_BASICTIMER > 0 */

/******************************************************************************
 * DeInit all assigned GPIO PIns
 *****************************************************************************/
static void TMR_GPIO_DeInit(const HW_DeviceType *self)
{
    uint32_t devIdx = GetDevIdx(self);
    GpioAFDeInitAll(devIdx, self->devGpioAF); 
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
    const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
    uint8_t ch;
    TmrHandleDeInit(hnd);

    /* Set the flags for all used pwm channels */
    if ( self->devGpioAF && adt->pwm_used ) {
        for ( int i=0; i < adt->pwm_used->num; i++ ) {
            ch = adt->pwm_used->chn_idx[i];    
            if ( ch > 0 && ch < 5 ) {
                hnd->use_chX[ ch ] = 1;
            } else { 
                DEBUG_PRINTF("Illegal PWM channel %d for %s\n", ch, self->devName);
            }
        }
    }

    hnd->MicroCountHigh     = 0;
    hnd->bTicksEnabled      = 1; /* Only ues, if basetimer is also system tick generator */
}


/****************************************************************************** 
 * Return a timers clock domain ( ie APB1 or APB2 ) prescaler 
 * Does not work for LPTIM timers
 * -1 is returned, if the base clock cannot be determined
  *****************************************************************************/
int32_t TmrGetClockPrescaler ( TIM_TypeDef *tim )
{
    const TIM_TypeDef *iter;
    uint32_t i;
    for ( i=0; iter=apb1_timers[i], iter; i++ ) {
        if ( tim == iter ) return GetAPB1TimerPrescaler();
    }
    for ( i=0; iter=apb2_timers[i], iter; i++ ) {
        if ( tim == iter ) return GetAPB2TimerPrescaler();
    }

    return -1;
}

/****************************************************************************** 
 * Return a timers input clock frequency. This can either be the APB1 or
 * APB2 clock.
 * Does not work for LPTIM timers
 * 0 is returned, if the base clock cannot be determined
  *****************************************************************************/
uint32_t TmrGetClockFrq ( TIM_TypeDef *tim )
{
    const TIM_TypeDef *iter;
    uint32_t i;
    for ( i=0; iter=apb1_timers[i], iter; i++ ) {
        if ( tim == iter ) return GetAPB1TimerFrequency();
    }
    for ( i=0; iter=apb2_timers[i], iter; i++ ) {
        if ( tim == iter ) return GetAPB2TimerFrequency();
    }

    return 0;
}



/******************************************************************************
 * Set the timers prescaler to achieve the desired base frequency
 * false is reported, if sysclk frq is too low to achieve this or
 * if sysclk is not dividable to achieve base frq when bExactMatch = true
 *****************************************************************************/
bool TMR_SetBaseFrq( TIM_TypeDef *htim, uint32_t base_frq, uint32_t *new_psc, bool bExactMatch )
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
 * Returns the current counter frequency, ie system clock divided by prescaler+1
 *****************************************************************************/
uint32_t TMR_GetBaseFrq(const HW_DeviceType *self )
{
    TIM_TypeDef *htim       = (TIM_TypeDef *)self->devBase;
    return TmrGetClockFrq(htim) / (htim->PSC+1);
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

#if USE_PERIPHTIMER > 0 || USE_BASICTIMER > 0
    if ( self->devType == HW_DEVICE_BASETIMER || self->devType == HW_DEVICE_PERIPHTIMER ) {
        /* Initialize TIMx peripheral as follows:
            + Period of basetimer     =65536 or 1000, depends from USE_BASTIMER_FOR_TICKS
            + Period of periph. timer = 50000 or 500000 ie 20 or 2Hz, depending from timer bitwidth
            + Prescaler               = calculated dynamically to give 1 MHz timer input frequency
            + ClockDivision           = 1
            + Counter direction       = Up
        */
        if ( self->devType == HW_DEVICE_BASETIMER ) {
            myHnd->Init.Period            = BASTIMER_UPPER;
        } else {
            /* peripheral timer */
            if ( IS_TIM_32B_COUNTER_INSTANCE(htim) )
                myHnd->Init.Period            = 1000000-1;
            else
                myHnd->Init.Period            = 50000-1;
        }

        if (!TMR_SetBaseFrq(htim, adt->base_frq, &myHnd->Init.Prescaler, true) ) return false;

        /* Init Timer */
        htim->PSC = myHnd->Init.Prescaler;      // Set prescaler
	htim->EGR = TIM_EGR_UG;                 // load prescaler
        htim->CR1 = 0;                          // upcount
	htim->CR2 = TIM_CR2_MMS_1;              // master mode 010: update
	htim->ARR = myHnd->Init.Period;         // set period
	htim->SR = 0;                           // clear update status caused by TIM_EGR_UG

        /* Enable all configured interrupts */
        if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);
        
        /* Basictimer will be started immediately */
        if ( self->devType == HW_DEVICE_BASETIMER ) TMR_Start(self, true);

        return true;
   }
#endif
    return true;
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
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool TMR_AllowStop(const HW_DeviceType *self)
{
    if ( self->devType == HW_DEVICE_BASETIMER || self->devType == HW_DEVICE_PERIPHTIMER ) {
        const TMR_AdditionalDataType *adt = TMR_GetAdditionalData(self);
        return adt->myTmrHandle->reference_cnt == 0;
    } else
        return (!TMR_IsAnyChnActive(self));
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

    ret = TMR_SetBaseFrq(inst, adt->base_frq, &new_psc, self->devType==HW_DEVICE_BASETIMER);
    if ( ret ) inst->PSC = new_psc;
    return ret;
}


#if defined(USE_TIM6) && defined(TIM6)
    /**************************************************************************
     * This timer is primarily used for Profilimg and pulse capture of OOK
     * sequencer. Its timer frequency is fixed to exactly 1 MHz
     *************************************************************************/
    TimerHandleT         TIM6Handle;

    static const TMR_AdditionalDataType additional_tim6 = {
        .myTmrHandle = &TIM6Handle,
        .base_frq    = 1000000,          // Base Timers always run with 1MHz
        .pwm_frq     = HW_PWM_FREQUENCY,
    };

    const HW_IrqList irq_tim6 = {
        .num = 1,
        .irq = { TIM6_IRQ },
    };

    const HW_DeviceType HW_TIM6 = {
        .devName        = "BASETIM6",
        .devBase        = TIM6,
        .devGpioAF      = NULL,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_BASETIMER,
        .devData        = &additional_tim6,
        .devIrqList     = &irq_tim6,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = TMR_Init,
        .DeInit         = TMR_DeInit,
        .OnFrqChange    = TMR_OnFrqChange,
        .AllowStop      = TMR_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif /* TIM6 */

#if defined(USE_TIM7) && defined(TIM7)
    /**************************************************************************
     * This timer is primarily used for Profilimg and pulse capture of OOK
     * sequencer. Its timer frequency is fixed to exactly 1 MHz
     *************************************************************************/
    TimerHandleT         TIM7Handle;

    static const TMR_AdditionalDataType additional_tim7 = {
        .myTmrHandle = &TIM7Handle,
        .base_frq    = 1000000,          // Base Timers always run with 1MHz
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
        /* CH1 .. CH4 must be specified, even if not used for pwm*/
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
        .base_frq    = 8000000,
        .pwm_frq     = HW_PWM_FREQUENCY,
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

#if defined(USE_TIM2) && defined(TIM2)
    TimerHandleT         TIM2Handle;

    static const HW_GpioList_AF gpio_tim2 = {
        /* CH1 .. CH4 must be specified, even if not used for pwm*/
        .gpio = {
             NULL,
             NULL,
             NULL,
             NULL,
        },
*/
        .num = 0, 
        // .num = 4, 
    };

    static const TMR_AdditionalDataType additional_tim2 = {
        .myTmrHandle = &TIM2Handle,
        .base_frq    = 1000000,
        .pwm_frq     = HW_PWM_FREQUENCY,
    };

    static const HW_IrqList irq_tim2 = {
        .num = 1,
        .irq = { TIM2_IRQ },
    };

    const HW_DeviceType HW_TIM2 = {
        .devName        = "TIM2",
        .devBase        = TIM2,
        .devGpioAF      = &gpio_tim2,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PERIPHTIMER,
        .devData        = &additional_tim2,
        .devIrqList     = &irq_tim2,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = TMR_Init,
        .DeInit         = TMR_DeInit,
        .OnFrqChange    = TMR_OnFrqChange,
        .AllowStop      = TMR_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif /* TIM2 */

#if defined(USE_TIM3) && defined(TIM3)
    TimerHandleT         TIM3Handle;

    static const HW_GpioList_AF gpio_tim3 = {
        .gpio = { 
             TIM3_CH1,
             TIM3_CH2,
             TIM3_CH3,
             TIM3_CH4,
        },
        .num = 4, 
    };

    static const TmrUsdPwmChnls chUsed3 = {
        .chn_idx = { LCD_BKLGHT_CH, },   /* specify channel numbers here [1 ..4] ! */
        .num     = 1,
    };

    static const TMR_AdditionalDataType additional_tim3 = {
        .myTmrHandle = &TIM3Handle,
        .base_frq    = 8000000,
        .pwm_frq     = HW_PWM_FREQUENCY,
        .pwm_used    = &chUsed3,
    };

    const HW_DeviceType HW_TIM3 = {
        .devName        = "TIM3",
        .devBase        = TIM3,
        .devGpioAF      = &gpio_tim3,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PWMTIMER,
        .devData        = &additional_tim3,
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

#if defined(USE_TIM5) && defined(TIM5)
    TimerHandleT         TIM5Handle;

    static const HW_GpioList_AF gpio_tim5 = {
        /* CH1 .. CH4 must be specified, even if not used for pwm*/
        .gpio = { 
             TIM5_CH1,
             TIM5_CH2,
             TIM5_CH3,
             TIM5_CH4,
        },
        .num = 4, 
    };

    static const TmrUsdPwmChnls chUsed5 = {
        .chn_idx = { 3, 4, },   /* specify channel numbers here [1 ..4] ! */
        .num     = 2,
    };

    static const TMR_AdditionalDataType additional_tim5 = {
        .myTmrHandle = &TIM5Handle,
        .base_frq    = 8000000,
        .pwm_frq     = HW_PWM_FREQUENCY,
        .pwm_used    = &chUsed5,
    };

    const HW_DeviceType HW_TIM5 = {
        .devName        = "TIM5",
        .devBase        = TIM5,
        .devGpioAF      = &gpio_tim5,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PWMTIMER,
        .devData        = &additional_tim5,
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
#endif /* TIM5 */

#if defined(USE_TIM15) && defined(TIM15)
    TimerHandleT         TIM15Handle;

    static const HW_GpioList_AF gpio_tim15 = {
        /* CH1, CH2, CH1N must be specified, set NULL if not used for PWM */
        .gpio = { 
             TIM15_CH1,
             TIM15_CH2,
             TIM15_CH1N,
        },
        .num = 3, 
    };

    static const TMR_AdditionalDataType additional_tim15 = {
        .myTmrHandle = &TIM15Handle,
        .base_frq    = 8000000,
        .pwm_frq     = HW_PWM_FREQUENCY,
    };

    const HW_DeviceType HW_TIM15 = {
        .devName        = "TIM15",
        .devBase        = TIM15,
        .devGpioAF      = &gpio_tim15,
        .devGpioIO      = NULL,
        .devType        = HW_DEVICE_PWMTIMER,
        .devData        = &additional_tim15,
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
#endif /* TIM15 */

#if USE_BASICTIMER_FOR_TICKS > 0
    /*
     * If Basictimer is used for tick generation, the following
     * functions hasve to be implemented. They are used/called
     * by the HAL layer
     */
    HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
    {
      if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
          uwTickPrio = TickPriority;
      } else {
          return HAL_ERROR;
      }
  
      /* DeInit and Init Basetimer device */
      if ( BASTIM_HW.OnFrqChange ) BASTIM_HW.OnFrqChange(&BASTIM_HW);
//      BASTIM_HW.DeInit(&BASTIM_HW );  
//      BASTIM_HW.Init(&BASTIM_HW );  

      return HAL_OK;
    }

    /**
      * @brief  Suspend Tick increment.
      * @note   Disable the tick increment by disabling TIM6 update interrupt.
      * @param  None
      * @retval None
      */
    void HAL_SuspendTick(void)
    {
        /* hold tick counter */
        BASTIM_HANDLE.bTicksEnabled = false;
    }

    /**
      * @brief  Resume Tick increment.
      * @note   Enable the tick increment by Enabling TIM6 update interrupt.
      * @param  None
      * @retval None
      */
    void HAL_ResumeTick(void)
    {
        /* relese tick counter */
        BASTIM_HANDLE.bTicksEnabled = true;
    }
#endif /* USE_BASICTIMER_FOR_TICK > 0 */

#if USE_PERIPHTIMER > 0
    /******************************************************************************
     * Start/Stop for peripheral timer
     *****************************************************************************/
    void PeriphTimer_StartStop(uint32_t bStart)
    {
        TimerHandleT* myTmrHandle = TMR_GetHandleFromDev(&PERIPH_TIMER);

        if ( bStart ) {
            TMR_Start(&PERIPH_TIMER, true);
            /* Set reference count to 1 ( a timer can be only started once ) */
            myTmrHandle->reference_cnt = 1;
        } else {
            TMR_Stop(&PERIPH_TIMER);
            myTmrHandle->reference_cnt = 0;
        }
    }
#endif /* USE_PERIPHTIMER > 0 */ 

#endif /* USE_HW_PWMTIMER > 0 || USE_BASICTIMER > 0 || USE_PERIPH_TIMER > 0 || USE_USER_PWMTIMER > 0 */
/**
  * @}
  */


