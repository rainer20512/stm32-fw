/*
 ******************************************************************************
 * @file    pwm_timer.c 
 * @author  Rainer
 * @brief   All funtions to use a timer as PWM timer 
 *
 *
 *****************************************************************************/

/** @addtogroup Timer device
  * @{
  */

#include "config/config.h"

#include "config/devices_config.h"
#include "debug_helper.h"
#include "dev/devices.h"
#include "dev/pwm_timer.h"

#if USE_HW_PWMTIMER > 0 || USE_USER_PWMTIMER > 0

static uint32_t idxToTimCh[6] = 
    { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
      TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6, 
    };


static const PwmChannelT PwmChannelList[] =  ALL_PWM_CHANNELS;
static uint32_t bChnListIterator;


/******************************************************************************
 * Overwrite the Hardcoded presets of PWM Timers for base and PWM frequency
 * Parameters IN
 *   self - Timer Device
 *   ch   - PWM channel [1..4], may also be 0, which means: ignore
 * Parameters IN/OUT
 *   base_frq - Timer base frequency, ie input frequency to counters in Hz
 *   pwm_frq    PWM freqency in HZ
 *   duty_cycle PWM duty cycle in %
 * Any of the IN/OUT parameters may be NULL, in wich case theyy are ignored 
 *****************************************************************************/
void Pwm_Ch_UpdatePresets(const HW_DeviceType *self, uint8_t ch, uint32_t *base_frq, uint32_t *pwm_frq, uint8_t *duty_cycle)
{
    UNUSED(base_frq);
    if ( self == &HW_TIM5 ) {
        if (pwm_frq) *pwm_frq = 10000;
        if ( ch ) switch (ch) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                if (duty_cycle) *duty_cycle = 25;
                break;
            case 4:
                if (duty_cycle) *duty_cycle = 25;
                break;
        } /* switch, if */
         
    } else if ( self == &HW_TIM3 ) {

    }
}

/******************************************************************************
 * Returns the index in PwmChannelList, if "dev" is a PWM-Timer
 * Returns DEV_DOTFOUND ist not in the list
 *****************************************************************************/
int PWM_CH_GetIdx(const HW_DeviceType *dev)
{
    for ( uint32_t i=0; i < sizeof(PwmChannelList)/sizeof(PwmChannelT); i++ ) {
        if ( dev == PwmChannelList[i].tmr ) return i;
    }

    return DEV_NOTFOUND;
}

/******************************************************************************
 * Returns the idx'th PWM channel [ 0 .. n ]
 * NULL if invalid index
 *****************************************************************************/
const PwmChannelT*  PWM_CH_GetCh(uint32_t idx)
{
    if ( idx < sizeof(PwmChannelList)/sizeof(PwmChannelT) )
        return &PwmChannelList[idx];
    else
        return NULL;
}

/******************************************************************************
 * Returns next element of PWM channel ist or NULL if no more elements
 *****************************************************************************/
const PwmChannelT* PWM_CH_IterateNext(void)
{
    if ( bChnListIterator < sizeof(PwmChannelList)/sizeof(PwmChannelT) )
        return &PwmChannelList[bChnListIterator++];
    else    
        return NULL;
}

/******************************************************************************
 * Start the PWM channel list iterator 
 * Returns first element or NULL if list is empty
 *****************************************************************************/
const PwmChannelT* PWM_CH_IterateBegin(void)
{
    bChnListIterator = 0;
    return PWM_CH_IterateNext();
}

/******************************************************************************
 * Check the validity of PWM frequency 
 * an error is flagged, if the PWM frq is greater than Base Frq / 20 
 * this is due to shrinking duty cycle options
 * function will return false, if pwm frequency is illegal
 * in all other cases the correct value for the ARR-Register is 
 * optionally written to arr_ret, if arr_ret is not NULL

 *****************************************************************************/
static bool Pwm_Ch_CheckPwmFrq (uint32_t base_frq, uint32_t pwm_frq, uint32_t *arr_ret ) 
{
    bool ret = true;
     
    if ( pwm_frq > base_frq/2 ) {
        DEBUG_PRINTF("Error: TMR PWM frq must not be greater than %d, no change\n", base_frq/2 );
        return false;
    }

 
    uint32_t arr = base_frq / pwm_frq;
    #if DEBUG_MODE > 0
        if (arr < 20 ) {
            DEBUG_PRINTF("Warning: TMR PWM frq allows only %d Duty cycle steps\n", arr);
        }
    #endif

    if ( arr_ret) *arr_ret = arr;
    return ret;
}


/******************************************************************************
 * Initialize the underlying timer of a PWM channel
 * This functions is called automatically once per timer device after HW Init
 *****************************************************************************/
void PWM_CH_InitTimer ( const HW_DeviceType *self, void *args)
{
    UNUSED(args);
    TIM_TypeDef *htim   = (TIM_TypeDef *)self->devBase;
    TIM_HandleTypeDef *myHnd  =  &TMR_GetHandleFromDev(self)->myHalHnd;
    uint32_t base_frq;         /* Timer Base Frequency       */
    uint32_t pwm_frq;          /* default pwm frequency      */

    /* Check for timer device being a PWM Timer */
    if ( self->devType != HW_DEVICE_PWMTIMER ) {
        DEBUG_PRINTF("Error: Timer %s is not a PWM timer!\n", self->devName);
        return;
    }

    /* Get presets for timer base frequency and pwm frequeny from additional data record */
    TMR_GetBaseAndPwmFrq(self, &base_frq, &pwm_frq);
    
    /* Update with program specific settings */
    Pwm_Ch_UpdatePresets(self, 0, &base_frq, &pwm_frq, NULL);

    /* Check validity */
    if ( !Pwm_Ch_CheckPwmFrq(base_frq, pwm_frq, NULL) ) return;

    myHnd->Instance = htim;

    /* Set PWM base and PWM frequency */
    TMR_SetBaseFrq(htim, base_frq, &myHnd->Init.Prescaler, false);
    /* default as configured */
    myHnd->Init.Period            = base_frq / pwm_frq;  
    myHnd->Init.ClockDivision     = 0;
    myHnd->Init.CounterMode       = TIM_COUNTERMODE_UP;
    myHnd->Init.RepetitionCounter = 0;
    
    HAL_TIM_PWM_Init(myHnd);
    
}

static bool TmrCheckValidChn( const HW_DeviceType *self, uint32_t ch )
{
    if ( ch < 1 || ch > 4 ) return false;

    return TMR_GetHandleFromDev(self)->use_chX[ch];
    
}


/******************************************************************************
 * Initialize one PWM channel
 *****************************************************************************/
bool PWM_CH_Init(const PwmChannelT *pwmch)
{
    const HW_DeviceType *self = pwmch->tmr;
    /* Check for valid channel */
    if (!TmrCheckValidChn(self, pwmch->channel) ) return false;

    uint32_t devIdx = GetDevIdx(self);

   /* configure/enable the corresponding gpio */
    GPIO_InitTypeDef  Init;
    Init.Mode  = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_HIGH;   
    GpioAFInitOne(devIdx, &self->devGpioAF->gpio[pwmch->channel-1], &Init );

    TIM_OC_InitTypeDef sConfig;
    TIM_TypeDef *inst = (TIM_TypeDef *)TMR_GetHandleFromDev(self)->myHalHnd.Instance;

    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = (pwmch-> bInvert ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH );
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    /* PWM initially to 50% ( PWM is not started yet! ) */
    sConfig.Pulse        = inst->ARR / 2;
    
    if ( HAL_TIM_PWM_ConfigChannel(&TMR_GetHandleFromDev(self)->myHalHnd, &sConfig, idxToTimCh[pwmch->channel-1]) != HAL_OK ) return false;

    /* if autostart is set, then start the PWM channel immediately */
    if ( pwmch->bautostart ) {
        /* if no other assignments are made, set a default duty cycle of 50% */
        uint8_t duty = 50;
        /* Update presets */
        Pwm_Ch_UpdatePresets(self, pwmch->channel, NULL, NULL, &duty);

        PWM_CH_StartPWMChPromille(self, pwmch->channel, ((uint32_t)(duty)) * 10);
    }

    return true;
}




/******************************************************************************
 * Start the PWM with an absolute value for the channels CCR
 * the allowed value for CCR is [ 0 .. ARR ]
 *****************************************************************************/
static void Pwm_Tmr_StartPWMAbs( TIM_HandleTypeDef *htim, uint32_t ch, uint32_t absval)
{
    TIM_TypeDef *inst = htim->Instance;

    __IO uint32_t *ccr_reg = &inst->CCR1;
    *(ccr_reg+ch-1) = absval;

    HAL_TIM_PWM_Start( htim, idxToTimCh[ch-1]);
}


/******************************************************************************
 * Start the PWM with a duty cycle of promille
 *****************************************************************************/
void PWM_CH_StartPWMChPromille(const HW_DeviceType *self, uint32_t ch, uint32_t promille)
{
    if ( promille > 1000 )  return;

    /* check for valid pwm channel */
    if (!TmrCheckValidChn(self, ch) ) return;
    

    TIM_HandleTypeDef *hnd = &TMR_GetHandleFromDev(self)->myHalHnd;

    TIM_TypeDef *inst = (TIM_TypeDef *)hnd->Instance;

    uint32_t abs = (inst->ARR+1) * promille;
    /* Round before division */
    abs =  (abs+500) /1000 ;
    if ( abs > 0 ) abs--;
    /* for 100% on, CCR must be greater than ARR */
    if ( promille == 1000 ) abs = (inst->ARR+1);
    Pwm_Tmr_StartPWMAbs(hnd, ch, abs );
}

/******************************************************************************
 * Start the PWM with a duty cycle of s256/256
 *****************************************************************************/
void PWM_CH_StartPWMChS256(const HW_DeviceType *self, uint32_t ch, uint32_t s256 )
{
    if ( s256 > 256 ) return;

    /* check for valid pwm channel */
    if (!TmrCheckValidChn(self, ch) ) return;

    TIM_HandleTypeDef *hnd = &TMR_GetHandleFromDev(self)->myHalHnd;
    TIM_TypeDef *inst = (TIM_TypeDef *)hnd->Instance;

    uint32_t abs = (inst->ARR+1) * s256;
    /* Round before division */
    abs =  (abs+128) /256 ;
    if ( abs > 0 ) abs--;
    /* for 100% on, CCR must be greater than ARR */
    if ( s256 == 256 ) abs = (inst->ARR+1);

    Pwm_Tmr_StartPWMAbs(hnd, ch, abs );
}

/******************************************************************************
 * Stop one PWM channel and deactivate the GPIO
 *****************************************************************************/
void PWM_CH_StopPWMCh(const HW_DeviceType *self, uint32_t ch)
{
    /* check for valid pwm channel */
    if (!TmrCheckValidChn(self, ch) ) return;

    uint32_t devIdx = GetDevIdx(self);
    TIM_HandleTypeDef *htim = &TMR_GetHandleFromDev(self)->myHalHnd;

    HAL_TIM_PWM_Stop(htim, idxToTimCh[ch-1]);

    /* Deactivate GPIO */
    GpioAFDeInitOne(devIdx, &self->devGpioAF->gpio[ch-1]);

    /* If no other PWM channel is active, disable TMR */
    if ( !TMR_IsAnyChnActive(self) ) CLEAR_BIT( htim->Instance->CR1, TIM_CR1_CEN );
}


/******************************************************************************
 * Set the PWM frequency 
 * an error is returned, if the PWM frq is greater than Base Frq / 20 
 * this is due to shrinking duty cycle options
 * NOTE: This setting affects ALL PWM outputs!
 *****************************************************************************/
bool PWM_TMR_SetPWMFrq (const HW_DeviceType *self, uint32_t pwm_frq ) 
{
    bool ret = true;
    uint32_t arr;
    
    if (!Pwm_Ch_CheckPwmFrq(TMR_GetBaseFrq(self), pwm_frq, &arr) ) return false;

    ((TIM_TypeDef *)self->devBase)->ARR = arr-1;
    return ret;
}


#endif /*  USE_HW_PWMTIMER > 0 || USE_USER_PWMTIMER > 0 */
/**
  * @}
  */


