/*
 ******************************************************************************
 * @file    adc_dev.c 
 * @author  Rainer
 * @brief   ADC hardware wrapped into device type
 *
 *****************************************************************************/

/** @addtogroup ADC device
  * @{
  */

#include "config/devices_config.h"

#if defined(USE_ADC1) || defined(USE_ADC2) || defined(USE_ADC3)

#include "error.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"
#include "system/dma_handler.h"

#include "dev/devices.h"
#include "config/adc_config.h"
#include "task/minitask.h"
#include "system/periodic.h"


#define DEBUG_ADC 1

#if DEBUG_MODE > 0 && DEBUG_ADC > 0
    #include "debug_helper.h"
    #include "system/profiling.h"
#else
    #include <stdio.h>
#endif

#if defined(STM32L4_FAMILY)
    #define ADC_SAMPLETIME_FAST     ADC_SAMPLETIME_24CYCLES_5
    #define ADC_SAMPLETIME_SLOW     ADC_SAMPLETIME_47CYCLES_5
    #define ADC_CLOCKSOURCE         RCC_ADCCLKSOURCE_SYSCLK
#elif defined(STM32H7_FAMILY)
    #define ADC_SAMPLETIME_FAST     ADC_SAMPLETIME_32CYCLES_5
    #define ADC_SAMPLETIME_SLOW     ADC_SAMPLETIME_64CYCLES_5
    #define ADC_CLOCKSOURCE         RCC_ADCCLKSOURCE_CLKP
#else
    #error "No setup for ADC sample times"
#endif

#define ADC_DEFAULT_OVRSAMPLE       4           /* Default will be 16x Oversampling / 4 bits right shift */

/* My defines -------------------------------------------------------------------*/
/* Size of requierd DMA buffer for max 8 DMA channels, MUST reside in noncached area! */
#define ADC_DMABUF_SIZE         17

/* My macros --------------------------------------------------------------------*/
#define GET_SEQ_LEN(hadc)       ( ( hadc->Instance->SQR1 & ADC_SQR1_L_Msk ) + 1 )

/* Public typedef ---------------------------------------------------------------*/


typedef struct {
     AdcHandleT *myAdcHandle;       /* Ptr to my associated handle */
     uint16_t   *myDmaBuf;          /* Ptr to associated dmabuf    */
} ADC_AdditionalDataType;


/* Forward declarations -------------------------------------------------------------*/

static ADC_AdditionalDataType * ADC_GetAdditionalData(const HW_DeviceType *self)
{
    return (ADC_AdditionalDataType *)(self->devData);
}



/* Private or driver functions ------------------------------------------------------*/

/******************************************************************************
 * Check for driver being initialized 
 *****************************************************************************/
static bool Adc_AssertInitialized(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0
        if (!adt->myAdcHandle->bInitialized ) DEBUG_PRINTF("Error: %s not initialized\n", self->devName);
    #endif
    return adt->myAdcHandle->bInitialized;
}


/******************************************************************************
 * Check for driver being initialized 
 *****************************************************************************/
static bool Adc_AssertChiptemp(const HW_DeviceType *self)
{
    bool ret = ADC_HAS_CHIPTEMP(self->devBase);
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0
        if (!ret ) DEBUG_PRINTF("Error: %s has no Chip temp capability\n", self->devName);
    #endif
    return ret;
}
static const uint32_t ovrsampleCode[9] = { 0, 
    ADC_OVERSAMPLING_RATIO_2,  ADC_OVERSAMPLING_RATIO_4,  ADC_OVERSAMPLING_RATIO_8,   ADC_OVERSAMPLING_RATIO_16, 
    ADC_OVERSAMPLING_RATIO_32, ADC_OVERSAMPLING_RATIO_64, ADC_OVERSAMPLING_RATIO_128, ADC_OVERSAMPLING_RATIO_256 };
static const uint32_t rshiftCode[9] = { ADC_RIGHTBITSHIFT_NONE,
    ADC_RIGHTBITSHIFT_1, ADC_RIGHTBITSHIFT_2, ADC_RIGHTBITSHIFT_3, ADC_RIGHTBITSHIFT_4, 
    ADC_RIGHTBITSHIFT_5, ADC_RIGHTBITSHIFT_6, ADC_RIGHTBITSHIFT_7, ADC_RIGHTBITSHIFT_8 };

/******************************************************************************
 * Setup Oversampling according to nOvrSample
 *  - nOvrSample must be in the range 0 .. 8, which means 0 .. 256 x oversampling,
 
 *  - RightBitShift is alway set accordingly
 *  - No Reset on Injected conversions
 *  - All samples to be done in one step
 *****************************************************************************/
static void Adc_SetupOvrSampling ( ADC_InitTypeDef *Init, uint8_t nOvrSample  )
{
    ADC_OversamplingTypeDef *o  = &Init->Oversampling;

    if ( nOvrSample > 8 ) nOvrSample = 8;
    o->OversamplingStopReset    = ADC_REGOVERSAMPLING_CONTINUED_MODE;   /* Preserve Oversampling buffer during injected conversions */
    o->TriggeredMode            = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;     /* All samples done in one step */
    o->Ratio                    = ovrsampleCode[nOvrSample];            /* OvrSampling and Rshift according to nOvrSample */
    o->RightBitShift            = rshiftCode[nOvrSample];
    Init->OversamplingMode      = nOvrSample ? ENABLE : DISABLE;        /* enable/disable oversampling, according to nOvrsample */
}

/******************************************************************************
 * Setup the Init-parameters, that will never be altered
 * which is:
 * - Right data alignment ( which is due to possible oversampling )
 * - No wait during low power modes
 * - No continuous conversion
 * - Enable data overrun mode
 *****************************************************************************/
static void Adc_SetupStdParams( ADC_InitTypeDef *Init )
{
    Init->DataAlign             = ADC_DATAALIGN_RIGHT;              /* Right-alignment for converted data */
    Init->DMAContinuousRequests = DISABLE;                          /* ADC DMA for only one sequence */
    Init->LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
    Init->ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled (one shot conversion) */
    Init->DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because continuous mode is disabled */
    Init->NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
    Init->Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
}

/******************************************************************************
 * fill the ADC_InitTypeDef structure with the "variable" values
 * which are:
 * - resolution: One of  ADC_RESOLUTION_xyB
 * - prescaler:  One of the ADC_CLOCK_(A)SYNC_yyyyyy constants
 * - trigger source: One of ADC group regular trigger source
 * - external start/stop function: In case of external triggers: A function
 *       to start/stop the external trigger source ( Timer, eg )
 * - number of channels used in one conversion cycle
 *****************************************************************************/
static void Adc_SetupVariableParams ( const HW_DeviceType *self, uint32_t adcResolution, uint32_t adcPrescaler, 
                                        uint32_t extTriggerSrc, ExtStartStopFn startstopFn, uint8_t nrofChannels )
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_InitTypeDef *Init       = &adt->myAdcHandle->hAdc.Init;

    Init->Resolution            = adcResolution;
    Init->ClockPrescaler        = adcPrescaler;
    #if defined(STM32H7_FAMILY)
        if ( nrofChannels < 2 ) 
            Init->ConversionDataManagement = ADC_CONVERSIONDATA_DR  ;       /* ADC Data register of inly one channel */
        else
            Init->ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;/* ADC DMA for only one sequence */
    #endif
    if ( nrofChannels < 2 ) {
        Init->ScanConvMode          = DISABLE;                   /* Sequencer disabled, only one channel */
        Init->EOCSelection          = ADC_EOC_SINGLE_CONV;       /* EOC flag picked-up to indicate end of one conversion */
        adt->myAdcHandle->bSequence = false;                     /* remember that we configured single channel */   
    } else {
        Init->ScanConvMode          = ENABLE;                    /* Sequencer enabled  when more than one channel */
        Init->EOCSelection          = ADC_EOC_SEQ_CONV;          /* EOC flag picked-up to indicate end of whole sequence */
        adt->myAdcHandle->bSequence = true;                      /* remember that we configured a sequence */ 
   }
    Init->NbrOfConversion       = nrofChannels;                  /* As passed */
    Init->ExternalTrigConv      = extTriggerSrc;      
    if ( extTriggerSrc == ADC_SOFTWARE_START ) {
        Init->ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;   /* Parameter discarded because software trigger chosen */
        adt->myAdcHandle->ExtStartStop = NULL;                            /* No external trigger start function                  */
    } else {
        Init->ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING; /* Parameter discarded because software trigger chosen */
        adt->myAdcHandle->ExtStartStop = startstopFn;                     /* external trigger start function as passed           */
    }
}

/******************************************************************************
 * Setup the ADC_InitTypeDef structure for 
 * - 12 bit resolution, 16x oversampling, right aligned result, 
 * - ADC start by software, number of sequencer channels as specified,
 * - EOC at end of sequence,
 * 0 on failure 
 *****************************************************************************/
bool ADC_SetupStd(const HW_DeviceType *self, uint8_t nrofChannels )
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    ADC_InitTypeDef *Init       = &hAdc->Init;


    /* 
     * Do the neccessary ADC settings. Theses settings are done only once and are kept 
     * in Init-Struct as long as the variable is in scope.
     */

    /* Setup Oversampling */
    Adc_SetupOvrSampling(Init, PeriphTimer_StartStop);

    /*Set the standard (unchanged) init parameters */
    Adc_SetupStdParams(Init);

    /* Setup all the variable parameters with commonly used values */
    Adc_SetupVariableParams( 
        self, 
        ADC_RESOLUTION_12B,               /* 12-bit resolution for converted data */
    #if defined(STM32L4_FAMILY)
        ADC_CLOCK_SYNC_PCLK_DIV1,         /* Synchronous clock mode, input ADC clock = SYSCLK */
    #elif defined(STM32H7_FAMILY)
        ADC_CLOCK_ASYNC_DIV16,            /* Peripheral clock of 64 MHz divied by 16*/
    #else
        #error "NO ADC setup for selected STM32 type"
    #endif
        ADC_SOFTWARE_START,               /* Manual start by software */  
        NULL,                             /* No Startup function */
        nrofChannels                     /* As passed */
    );
    
    /* Initialize ADC peripheral according to the passed parameters */
    if (HAL_ADC_Init(hAdc) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("Init of %s failed\n", self->devName);
        #endif
        return false;
    } 

    return true;
}

/******************************************************************************
 * Setup the ADC_InitTypeDef structure with a relevant parameters set indiviually 
 *****************************************************************************/
bool ADC_SetupSpecial ( const HW_DeviceType *self, uint32_t adcResolution, uint32_t adcPrescaler, 
                                        uint32_t extTriggerSrc, ExtStartStopFn startstopFn, uint8_t nrofChannels, uint8_t nOvrSample )
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    ADC_InitTypeDef *Init       = &hAdc->Init;


    /* 
     * Do the neccessary ADC settings. Theses settings are done only once and are kept 
     * in Init-Struct as long as the variable is in scope.
     */
    /* Setup Oversampling */
    Adc_SetupOvrSampling(Init, nOvrSample);

    /*Set the standard (unchanged) init parameters */
    Adc_SetupStdParams(Init);

    /* Setup all the variable parameters */
    Adc_SetupVariableParams( self, adcResolution,adcPrescaler, extTriggerSrc, startstopFn, nrofChannels );
    
    /* Initialize ADC peripheral according to the passed parameters */
    if (HAL_ADC_Init(hAdc) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("Init of %s failed\n", self->devName);
        #endif
        return false;
    } 

    return true;
}

/******************************************************************************
 * Return the Rank bitmask for Rank idx
 * 0 on failure 
 *****************************************************************************/
static uint32_t Adc_GetRankFromIdx( uint32_t idx )
{
    static const uint32_t ADC_ranks[] = { 
        ADC_REGULAR_RANK_1,  ADC_REGULAR_RANK_2,  ADC_REGULAR_RANK_3,  ADC_REGULAR_RANK_4,
        ADC_REGULAR_RANK_5,  ADC_REGULAR_RANK_6,  ADC_REGULAR_RANK_7,  ADC_REGULAR_RANK_8,
        ADC_REGULAR_RANK_9,  ADC_REGULAR_RANK_10, ADC_REGULAR_RANK_11, ADC_REGULAR_RANK_12,
        ADC_REGULAR_RANK_13, ADC_REGULAR_RANK_14, ADC_REGULAR_RANK_15, ADC_REGULAR_RANK_16,
    };
    
    idx--;

    if ( idx > sizeof(ADC_ranks)/sizeof(uint32_t) ) 
        return 0;
    else
        return ADC_ranks[idx];
}

/******************************************************************************
 * Setup a channel with Rank1 for Vrefint conversion
 * caller has to make sure, that associated ADC has an refint channel!
 *****************************************************************************/
 static void Adc_SetupVrefintChannel(ADC_ChannelConfTypeDef *c, uint32_t rankidx) 
{
    c->Channel      = ADC_CHANNEL_VREFINT;         /* Internal Vrefint channel ( only valid for certain ADC instances ) */
    c->Rank         = Adc_GetRankFromIdx(rankidx); /* Rank of sampled channel number ADCx_CHANNEL */
    c->SamplingTime = ADC_SAMPLETIME_SLOW;         /* Sampling time (number of clock cycles unit), according to datasheet: minimum 4uS */
    c->SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    c->OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
    c->Offset       = 0;                           /* Parameter discarded because offset correction is disabled */
}

/**************************************************************************************
 * Start a single conversion 
 * ADC must have been initialized for one channel
 * and the channel must have been configured before
 *************************************************************************************/
static bool ADC_SingleConversion(ADC_HandleTypeDef *hAdc, uint32_t *raw )
{
    bool ret = true;
    /* Start conversion  */
    if (HAL_ADC_Start(hAdc) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("ADC_Start failed");
        #endif
        return false;
    }
  
    /* Wait for conversion being done */
    if (HAL_ADC_PollForConversion(hAdc, 10) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("ADC Polling failed");
        #endif
        ret = false;
        goto single_end;
    }

    /* Read the converted value and convert to mV by HAL macro*/
    *raw = HAL_ADC_GetValue(hAdc);

single_end:
    HAL_ADC_Stop(hAdc);
    return ret;
}

static void AdcDmaChannelInit(AdcHandleT *myAdc, const HW_DmaType *dma )
{

  DMA_HandleTypeDef *hdma = dma->dmaHandle;

  HW_DMA_HandleInit(hdma, dma, &myAdc->hAdc );

  /* Overwrite Data alignment, which is initialized to 'byte' */
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  
  hdma->Init.Direction           = DMA_PERIPH_TO_MEMORY;  
  /* Overwrite mode to circular mode */
  hdma->Init.Mode               = DMA_CIRCULAR;
  
  myAdc->hAdc.DMA_Handle     = dma->dmaHandle;

  HAL_DMA_Init(hdma);

  return;
}

static void Adc_UpdateVdda( AdcHandleT *myAdcHandle, uint32_t raw_result )
{
    ADC_HandleTypeDef *hAdc  = &myAdcHandle->hAdc;
    myAdcHandle->vdda        = __HAL_ADC_CALC_VREFANALOG_VOLTAGE( raw_result, hAdc->Init.Resolution );
    myAdcHandle->bVdda_valid = true;
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF("Vdda=%dmV\n", myAdcHandle->vdda);
    #endif
}

static AdcHandleT *Adc_GetMyHandleFromHalHandle(ADC_HandleTypeDef *hadc)
{
#if defined(ADC1) && defined(USE_ADC1)
    if ( &ADC1Handle.hAdc == hadc ) return &ADC1Handle;
#endif
#if defined(ADC2) && defined(USE_ADC2)
    if ( &ADC2Handle.hAdc == hadc ) return &ADC2Handle;
#endif
#if defined(ADC3) && defined(USE_ADC3)
    if ( &ADC3Handle.hAdc == hadc ) return &ADC3Handle;
#endif
    return NULL;
}

/* Interrupt Callbacks, executed in interrupt context --------------------------------*/

/**************************************************************************************
 * Callback for "Conversion of sequence complete"
 * Inform the task scheduler, that ADC actions have to be taken
 *
 * Moreover, if Refint capability is availabel, it has been added as first channel:
 * As we added measurement of Vrefint in the first position, we will reomve the first
 * result from the sequence here and use it to update the Vdda value
 * 
 * 
 *************************************************************************************/
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{   
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 && DEBUG_PROFILING > 0
        DEBUG_PRINTF("%d>",ProfilerGetMicrosecond());
    #endif

    /* If measurement was started manually, disable ADC manually after measurement */
    if ( hadc->Init.ExternalTrigConv == ADC_SOFTWARE_START ) {
        HAL_ADC_Stop_DMA(hadc);
    } else {
        /* Reset end of conversion and end of sequence flag */
        hadc->Instance->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
        /* And restart conversion */
        hadc->Instance->CR |= ADC_CR_ADSTART;
    }
    
    AdcHandleT *myHandle = Adc_GetMyHandleFromHalHandle(hadc);
    if (!myHandle ) return;

    uint8_t seqlen = GET_SEQ_LEN(hadc);
    if ( ADC_HAS_REFINT(hadc->Instance) ) {
        uint32_t raw = myHandle->dmabuf[0];
        Adc_UpdateVdda(myHandle, raw);
        myHandle->seqLen = seqlen-1;
        myHandle->seqResultPtr = myHandle->dmabuf+1;
    } else {
        myHandle->seqLen = seqlen;
        myHandle->seqResultPtr = myHandle->dmabuf;
    }

    myHandle->bSequenceDone = true;
    TaskNotify(TASK_ADC);
}

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc)
{   
    puts("error");
    HAL_ADC_Stop_DMA(hadc);
}



/* Public or driver functions --------------------------------------------------------*/

/**************************************************************************************
 * Retun the result of the latest Vdda measurement
 * conversions may be performed
 *************************************************************************************/
uint16_t ADC_GetVdda(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    AdcHandleT *myAdcHandle     = adt->myAdcHandle;
    if (myAdcHandle->bVdda_valid)
        return myAdcHandle->vdda;
    else
        return 0;
}    

#if defined(STM32L4_FAMILY)
static inline __attribute__((always_inline))
bool AdcCalibrate ( ADC_HandleTypeDef *hAdc )
{
  return    
       (HAL_ADCEx_Calibration_Start(hAdc, ADC_SINGLE_ENDED      ) ==  HAL_OK) 
    && (HAL_ADCEx_Calibration_Start(hAdc, ADC_DIFFERENTIAL_ENDED) ==  HAL_OK)
  ;
}
#elif defined(STM32H7_FAMILY)
bool AdcCalibrate ( ADC_HandleTypeDef *hAdc )
{
  return    
       (HAL_ADCEx_Calibration_Start(hAdc, ADC_CALIB_OFFSET,           ADC_SINGLE_ENDED      ) ==  HAL_OK) 
    && (HAL_ADCEx_Calibration_Start(hAdc, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED      ) ==  HAL_OK)
    && (HAL_ADCEx_Calibration_Start(hAdc, ADC_CALIB_OFFSET,           ADC_DIFFERENTIAL_ENDED) ==  HAL_OK)
    && (HAL_ADCEx_Calibration_Start(hAdc, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) ==  HAL_OK)
  ;
}
#else
    #error "No ADC calibration preocedure defined"
#endif

/**************************************************************************************
 * Do an ADC calibration. This is mandatory as rthe first step before any other
 * conversions may be performed
 *************************************************************************************/
bool ADC_Calibrate(const HW_DeviceType *self)
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return false;

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;

    /* Setup for one channel, mandatory before conversion */
    ADC_SetupStd(self, 1);

    /* Wait for ADC voltage sources to stablize */
    HAL_Delay(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

    /* do the mandatory ADC Calibration, MUST be done after ADC_Init! */
    if (  !AdcCalibrate(hAdc) ) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("Calibration of %s failed\n", self->devName);
        #endif
        return false;
    }
    /* Store both calibration factors */
    adt->myAdcHandle->adc_calfact = hAdc->Instance->CALFACT;
    return true;
}
 

/******************************************************************************
 * Measure a single channel. The parameter channel_idx refers to the
 * "channel_idx"-th channel in the gpio list of the device
 *****************************************************************************/
uint32_t ADC_MeasureChannel ( const HW_DeviceType *self, uint32_t channel_idx )
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return 0xFFFFFFFF;

    ADC_AdditionalDataType *adt         = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc             = &adt->myAdcHandle->hAdc;
    const HW_GpioList_ADC *gpioadclist  = self->devGpioADC;

    const HW_Gpio_ADC_Type *gpio;
    ADC_ChannelConfTypeDef c;  
    uint32_t raw;

    if ( channel_idx >= gpioadclist->num ) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("Channel %d not configured for %s\n", channel_idx, self->devName);
        #endif
        return 0xFFFFFFFF;
    }

    /* Check for single channel being initialized */
    if ( adt->myAdcHandle->bSequence ) ADC_SetupStd(self, 1);

    gpio           = &gpioadclist->gpio[channel_idx];
    c.OffsetNumber = ADC_OFFSET_NONE;                           /* No offset subtraction */ 
    c.Offset       = 0;                                         /* Parameter discarded because offset correction is disabled */
    c.SingleDiff   = ADC_SINGLE_ENDED;                          /* Single-ended input channel */
    c.Channel      = gpio->channelID;                           /* Channel ID as configured */
    c.Rank         = Adc_GetRankFromIdx(1);                     /* Rank in sequence of definition */
    c.SamplingTime = gpio->sampleTime;                          /* Sample time as configured */

    if (HAL_ADC_ConfigChannel(hAdc, &c) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("unable to configure channel %d of %s\n",channel_idx, self->devName);
        #endif
        return 0xFFFFFFFF;
    } 

    if ( !ADC_SingleConversion(hAdc, &raw) ) return 0xFFFFFFFF;
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF("Raw value of Ch %d is %d\n", channel_idx, raw);
    #endif
    return raw;
}

/**************************************************************************************
 * Do an ADC conversion of Vrefint to compute Vdda, which is also Vref
 * Then Vref is stored internally 
 *************************************************************************************/
static bool Adc_MeasureVdda (const HW_DeviceType *self)
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return false;

    /* make sure that the selected ADC has a REFINT channel */
    if ( !ADC_HAS_REFINT(self->devBase) ) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0
            DEBUG_PRINTF("Error: %s has no Refint capability\n", self->devName);
        #endif
        return false;
    }

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    uint32_t raw;

    /* Check for single channel being initialized */
    if ( adt->myAdcHandle->bSequence ) ADC_SetupStd(self, 1);

    ADC_ChannelConfTypeDef c;  
    Adc_SetupVrefintChannel(&c,1);

    /* Configure channel, will also set ADC_CCR_VREFEN */
    if (HAL_ADC_ConfigChannel(hAdc, &c) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("Failed to configure Vrefint-Channel");
        #endif
        return false;
    }
    HAL_Delay(LL_ADC_DELAY_VREFINT_STAB_US);
      
    if ( !ADC_SingleConversion(hAdc, &raw) ) return false;
 
    Adc_UpdateVdda( adt->myAdcHandle, raw );
    return true;
}

static inline __attribute__((always_inline))
ADC_Common_TypeDef *GetCommonInstance(const HW_DeviceType *self)
{
    UNUSED(self);
    #if defined(STM32L4_FAMILY)
        return __LL_ADC_COMMON_INSTANCE(self->devBase);
    #elif defined(STM32H7_FAMILY)
        return __LL_ADC_COMMON_INSTANCE(self->devBase);
    #else
        #error "No implementation to determine ADC common instance"
    #endif
}

void ADC_DisableRefintCh(const HW_DeviceType *self)
{
    CLEAR_BIT(GetCommonInstance(self)->CCR, ADC_CCR_VREFEN );
}

void ADC_DisableAllInternalCh(const HW_DeviceType *self)
{
    CLEAR_BIT(GetCommonInstance(self)->CCR, ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_VBATEN );
}


bool ADC_MeasureChipTemp (const HW_DeviceType *self)
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return false;

    /* make sure that the ADC is capable of measuring Chip temp */
    if (!Adc_AssertChiptemp(self) ) return false;

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    uint32_t raw;

    /* Check for single channel being initialized */
    if ( adt->myAdcHandle->bSequence ) ADC_SetupStd(self, 1);
    
    ADC_ChannelConfTypeDef c;  
    c.Channel      = ADC_CHANNEL_TEMPSENSOR;      /* Internal TempSensor ch. ( valid for certain channels ) */
    c.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
    c.SamplingTime = ADC_SAMPLETIME_FAST;         /* Sampling time (number of clock cycles unit) */
    c.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    c.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
    c.Offset       = 0;                           /* Parameter discarded because offset correction is disabled */

    /* Configure channel */
    if (HAL_ADC_ConfigChannel(hAdc, &c) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("Failed to configure TempSensor-Channel");
        #endif
        return false;
    }
  
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 && DEBUG_PROFILING > 0
        DEBUG_PRINTF(">%d",ProfilerGetMicrosecond());
    #endif
    
    if ( !ADC_SingleConversion(hAdc, &raw) ) return false;
    
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 && DEBUG_PROFILING > 0 
        DEBUG_PRINTF(",%d>\n",ProfilerGetMicrosecond());
    #endif

    adt->myAdcHandle->chiptemp  = __HAL_ADC_CALC_TEMPERATURE(adt->myAdcHandle->vdda, raw, ADC_RESOLUTION_12B );
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF("Chiptemp=%d degC\n", adt->myAdcHandle->chiptemp);
    #endif
    return true;
}

/******************************************************************************
 * Setup the ADC to convert all defined channels in one group
 * - if ADC has a RefInt channel, Vrefint is inserted as first channel
 * - ADC_SetupStd or ADC_SetupSpecial MUST HAVE been called before
 * - DMA MUST have been configured
 *
 *****************************************************************************/
void ADC_SetupGroup (const HW_DeviceType *self, uint32_t bStandard)
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return;

    ADC_AdditionalDataType *adt         = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc             = &adt->myAdcHandle->hAdc;
    const HW_GpioList_ADC *gpioadclist  = self->devGpioADC;
    const HW_Gpio_ADC_Type *gpio;

    ADC_ChannelConfTypeDef c;  
    uint32_t rank = 1;
    uint32_t i = 0;

    /* Make sure, that DMA is supported, it won't work without */
    if ( !self->devDmaRx ) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("Converting sequences requires DMA to be configured");
        #endif
        return;
    }

    uint8_t nrofch = gpioadclist->num; 

    /* if Instance has Refint channel, it is always inserted as the first channel */
    if ( ADC_HAS_REFINT(hAdc->Instance) ) nrofch++;
    if ( bStandard) {
        ADC_SetupStd(self, nrofch);
    } else {
       ADC_SetupSpecial ( 
         self, 
         ADC_RESOLUTION_12B, 
    #if defined(STM32L4_FAMILY)
        ADC_CLOCK_SYNC_PCLK_DIV1,         /* Synchronous clock mode, input ADC clock = SYSCLK */
    #elif defined(STM32H7_FAMILY)
        ADC_CLOCK_ASYNC_DIV16,            /* Peripheral clock of 64 MHz divied by 16*/
    #else
        #error "NO ADC setup for selected STM32 type"
    #endif
        ADC_EXTERNALTRIG_T2_TRGO,        /* Start by TRGO of TIM2 */  
        PeriphTimer_StartStop,            /* Start/Stop of TIM2 */
        nrofch,
        ADC_DEFAULT_OVRSAMPLE
     );
    }
    /* if Instance has Refint channel, it is always inserted as the first channel */
    if ( ADC_HAS_REFINT(hAdc->Instance) ) {
        Adc_SetupVrefintChannel(&c, rank);
        /* Configure channel */
        if (HAL_ADC_ConfigChannel(hAdc, &c) != HAL_OK) {
            #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
                DEBUG_PUTS("Failed to configure Vrefint-Channel");
            #endif
        } else {
            rank++;
        }
    }

    /* Now process all other channels */
    /* We only support single ended with no offset */
    c.OffsetNumber = ADC_OFFSET_NONE;                           /* No offset subtraction */ 
    c.Offset       = 0;                                         /* Parameter discarded because offset correction is disabled */
    c.SingleDiff   = ADC_SINGLE_ENDED;                          /* Single-ended input channel */

    while ( i < gpioadclist->num && rank <= 16 ) {
        gpio           = &gpioadclist->gpio[i];
        c.Channel      = gpio->channelID;                       /* Channel ID as configured */
        c.Rank         = Adc_GetRankFromIdx(rank);              /* Rank in sequence of definition */
        c.SamplingTime = gpio->sampleTime;                      /* Sample time as configured */
        if (HAL_ADC_ConfigChannel(hAdc, &c) != HAL_OK) {
            #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
                DEBUG_PRINTF("unable to configure channel %d of %s\n",i, self->devName);
            #endif
        } else {
            rank++;
        }
        i++;
    }

    if ( hAdc->Init.ExternalTrigConv != ADC_SOFTWARE_START ) {
        /* 
         * When ADC is not started by software, then start it now, which then
         * is an arming. The real start is done by the external trigger
         */
        if (HAL_ADC_Start_DMA(hAdc, (uint32_t *)adt->myAdcHandle->dmabuf, GET_SEQ_LEN(hAdc) ) != HAL_OK) {
            #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
                DEBUG_PUTS("Measuring Via DMA failed");
            #endif
        }
    }
}

bool ADC_MeasureGroup(const HW_DeviceType *self)
{
    /* make sure that the handle is initialized */
    if (!Adc_AssertInitialized(self) ) return false;

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;

    if ( !adt->myAdcHandle->bSequence ) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("MeasureGroup: No Sequence configured");
        #endif
        return false;
    }
  
    /* Invalidate old results */
    adt->myAdcHandle->bSequenceDone = false;
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 && DEBUG_PROFILING > 0 
        DEBUG_PRINTF(">%d",ProfilerGetMicrosecond());
    #endif

    if ( hAdc->Init.ExternalTrigConv == ADC_SOFTWARE_START ) {
        /* Start conversion manually */
        uint32_t length = GET_SEQ_LEN(hAdc);
        /* Start conversion  */
        if (HAL_ADC_Start_DMA(hAdc, (uint32_t *)adt->myAdcHandle->dmabuf, length ) != HAL_OK) {
            #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
                DEBUG_PUTS("Measuring Via DMA failed");
            #endif
            return false;
        }
     } else {
        /* Start the trigger generator */
        assert ( adt->myAdcHandle->ExtStartStop );
        adt->myAdcHandle->ExtStartStop(true);
     }

    return true;

}

/* 
 * There is one clocksource, that is common to all ADCs.
 * currently, we only support SYSCLK as clocksoure. This is configured  once
 * at this point

 */
static void ADC_ClockInit (const HW_DeviceType *self, bool bIsInit)
{
    /* Set clock source */
    if ( bIsInit ) {
      __HAL_RCC_ADC_CONFIG(ADC_CLOCKSOURCE);
    }

    /* enable ADC clock */
    HW_SetHWClock((void *)self->devBase, 1);

}

void Adc_Dump_CR( uint32_t cr )
{
    int oldpadlen = DBG_setPadLen ( 12 );
    DBG_dump_uint32_binary("CR=", cr);
    DBG_setIndentRel(+2);
    DBG_dump_setresetvalue("ADCAL", cr, ADC_CR_ADCAL_Msk );
    DBG_dump_setresetvalue("ADCALDIF", cr, ADC_CR_ADCALDIF_Msk );
    DBG_dump_setresetvalue("DEEPPWD", cr, ADC_CR_DEEPPWD_Msk );
    DBG_dump_setresetvalue("ADVREGEN", cr, ADC_CR_ADVREGEN_Msk );
    DBG_dump_setresetvalue("JADSTP", cr, ADC_CR_JADSTP_Msk );
    DBG_dump_setresetvalue("ADSTP", cr, ADC_CR_ADSTP_Msk );
    DBG_dump_setresetvalue("JADSTART", cr, ADC_CR_JADSTART_Msk );
    DBG_dump_setresetvalue("ADSTART", cr, ADC_CR_ADSTART_Msk );
    DBG_dump_setresetvalue("ADDIS", cr, ADC_CR_ADDIS_Msk );
    DBG_dump_setresetvalue("ADEN", cr, ADC_CR_ADEN_Msk );
    DBG_setPadLen ( oldpadlen );
    DBG_setIndentRel(-2);
}

const char * const exten_txt[]={"Disabled", "Rising Edge", "Falling Edge", "Both edges" };
static const char* adc_cfgr_get_exten_txt(uint32_t sel)
{
  if ( sel < sizeof(exten_txt)/sizeof(char *) ) 
    return exten_txt[sel];
  else
    return "Illegal";
}

const char * const res_txt[]={"12bit", "10bit", "8bit", "6bit" };
static const char* adc_cfgr_get_res_txt(uint32_t sel)
{
  if ( sel < sizeof(res_txt)/sizeof(char *) ) 
    return res_txt[sel];
  else
    return "Illegal";
}


void Adc_Dump_CFGR( uint32_t cfgr )
{
    int oldpadlen = DBG_setPadLen ( 25 );
    DBG_dump_uint32_binary("CFGR=", cfgr);
    DBG_setIndentRel(+2);
    DBG_dump_endisvalue("Injected Queue", ~cfgr, ADC_CFGR_JQDIS_Msk );
    DBG_dump_endisvalue("Auto injected group conv.", cfgr, ADC_CFGR_JAUTO );
    DBG_dump_endisvalue("AWD on inj. channels", cfgr, ADC_CFGR_JAWD1EN_Msk );
    DBG_dump_endisvalue("AWD in reg. channels", cfgr, ADC_CFGR_AWD1EN_Msk );
    if ( cfgr & ( ADC_CFGR_JAWD1EN_Msk |  ADC_CFGR_AWD1EN_Msk ) ) {
        DBG_dump_bitvalue("AWD on single channel", cfgr, ADC_CFGR_AWD1SGL_Msk );
        if ( cfgr & ADC_CFGR_AWD1SGL_Msk ) DBG_dump_number("AWD monitored channel", (cfgr & ADC_CFGR_AWD1CH_Msk) >> ADC_CFGR_AWD1CH_Pos );
    }
    DBG_dump_number("JQSR queue mode", ( cfgr & ADC_CFGR_JQM_Msk) >> ADC_CFGR_JQM_Pos );
    DBG_dump_endisvalue("discont. mode on inj. ch", cfgr, ADC_CFGR_JDISCEN_Msk );
    DBG_dump_endisvalue("discont. mode on reg. ch", cfgr, ADC_CFGR_DISCEN_Msk );
    if (cfgr & ( ADC_CFGR_JDISCEN_Msk | ADC_CFGR_DISCEN_Msk) ) DBG_dump_number("discont. mode ch cnt", ( cfgr &ADC_CFGR_DISCNUM_Msk ) >> ADC_CFGR_DISCNUM_Pos );
    DBG_dump_onoffvalue("Delayed conversion", cfgr, ADC_CFGR_AUTDLY_Msk );
    DBG_dump_endisvalue("Continuous conversion", cfgr, ADC_CFGR_CONT_Msk );
    DBG_dump_endisvalue("Overrun mode", cfgr, ADC_CFGR_OVRMOD_Msk );
    DBG_dump_textvalue("External trigger mode", adc_cfgr_get_exten_txt((cfgr &ADC_CFGR_EXTEN_Msk )>>ADC_CFGR_EXTEN_Pos ));
    if ( (cfgr &ADC_CFGR_EXTEN_Msk )>>ADC_CFGR_EXTEN_Pos ) DBG_dump_number("Trigger event #", (cfgr &ADC_CFGR_EXTSEL_Msk )>>ADC_CFGR_EXTSEL_Pos );
    DBG_dump_endisvalue("Right alignment", ~cfgr, ADC_CFGR_ALIGN_Msk );
    DBG_dump_textvalue("Resolution", adc_cfgr_get_res_txt((cfgr &ADC_CFGR_RES_Msk )>>ADC_CFGR_RES_Pos ));
    DBG_setPadLen ( oldpadlen );
    DBG_setIndentRel(-2);
}

void ADC_ShowStatus ( const HW_DeviceType *self )
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    ADC_TypeDef *adc            = hAdc->Instance;
   
    DEBUG_PRINTF("Debug of %s\n", self->devName);
    DBG_setIndentRel(+2);
    Adc_Dump_CR( adc->CR );
    Adc_Dump_CFGR( adc->CFGR );
    DBG_setIndentRel(-2);
}


bool ADC_Init(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt     = ADC_GetAdditionalData(self);
    AdcHandleT *myHandle            = adt->myAdcHandle;  
    ADC_HandleTypeDef *hAdc         = &myHandle->hAdc;
    hAdc->Instance                  = (ADC_TypeDef *)self->devBase;

    myHandle->bVdda_valid   = false;
    myHandle->bSequence     = false;
    myHandle->dmabuf        = adt->myDmaBuf;
  
    ADC_ClockInit(self, true);

    uint32_t devIdx = GetDevIdx(self);
    GpioADCInitAll(devIdx, self->devGpioADC);

    /* DeInit before Init */
    if (HAL_ADC_DeInit(hAdc) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("DeInit before Init of %s failed\n", self->devName);
        #endif
        return false;
    }

    if ( !ADC_SetupStd(self, 1) ) return false;

    /* 
     * Do the mandatory calibration, or in case of ADC1
     * do the calibration & sample of Vrefint in order to 
     * determine Vdda ( which is also Vref+ ) 
     * Calibration will also init ADC for one channel, 
     * no sequence, no injected channels 
     */
    myHandle->bInitialized  = true;
    if (!ADC_Calibrate(self) ) {
        myHandle->bInitialized  = false;
        return false;
    }
    
    if ( ADC_HAS_REFINT(self->devBase) ) ADC_MeasureVdda((void *)self);


    /* Configure the NVIC, enable interrupts */
    HW_SetAllIRQs(self->devIrqList, true);

    /* Enable DMA, if specified, ADC has only Rx dma */            
    if ( self->devDmaRx ) {

      // Take the first interrupt to copy prio and subprio to dma channel interrupts
      const HW_IrqType *irq = self->devIrqList->irq;

      /*make sure, that also interrupts are configured, DMA won't work without */
      if ( self->devIrqList->num == 0 ) {
         #if DEBUG_MODE > 0 && DEBUG_ADC > 0
             DEBUG_PRINTF("Init of %s: Use of DMA requires interrupts \n", self->devName);
         #endif
         myHandle->bInitialized  = false;
         return false;
      }

      /**** 004 ****/
      DMA_HandleTypeDef *hdma = HW_DMA_RegisterDMAChannel(self->devDmaRx);
      if ( !hdma ) return false;
      AdcDmaChannelInit( myHandle, self->devDmaRx);
      HW_DMA_SetAndEnableChannelIrq(hdma->Instance, irq->irq_prio, irq->irq_subprio);
    }

    return true;
}

void ADC_DeInit(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;

    HAL_ADC_Stop(hAdc);

    uint32_t devIdx = GetDevIdx(self);
    GpioADCDeInitAll(devIdx, self->devGpioADC);
  
    /* disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    if(self->devDmaRx) {
      HW_DMA_HandleDeInit(self->devDmaRx->dmaHandle);
    }
 
    /* disable ADC clock */
    ADC_ClockInit(self, false);

    adt->myAdcHandle->bInitialized  = false;
}

/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool ADC_AllowStop(const HW_DeviceType *self)
{
    return !LL_ADC_REG_IsConversionOngoing((ADC_TypeDef *)self->devBase);
}



static void Adc_Handle(AdcHandleT *myHandle)
{
    /* reset "conversion doen" flag, to prevent another processing */
    myHandle->bSequenceDone = false;

    ADC_ShowStatus(&HW_ADC1);
    for ( uint32_t i = 0; i<myHandle->seqLen; i++ )
        printf("Ch %d:Raw %d is %dmV\n", i, myHandle->seqResultPtr[i], __HAL_ADC_CALC_DATA_TO_VOLTAGE(myHandle->vdda,myHandle->seqResultPtr[i], myHandle->hAdc.Init.Resolution) );
    puts("");
}

/******************************************************************************
 * Trigger an Vdda measuerment. The void * parameter should be
 * the ptr to the ADC which is capable of Vdda measurement
 *****************************************************************************/
void ADC_MeasureVdda ( void *arg )
{
        Adc_MeasureVdda((const HW_DeviceType *)arg);
}

/******************************************************************************
 * Actions to be taken when adc task is initialized
 * - trigger periodic conversions at seond 56
 *****************************************************************************/
void task_init_adc(void)
{
    // AtSecond ( 56, ADC_MeasureVdda, (void *)&USER_ADC, "Measure Vdda");
}

/******************************************************************************
 * handler for ADC-Tasks, is only called, if a sequence is converted via DMA
 * Single conversions are always handeled vie polling
 *****************************************************************************/
void task_handle_adc(uint32_t arg)
{
    UNUSED(arg);
#if defined(ADC1) && defined(USE_ADC1)
    if ( ADC1Handle.bSequenceDone ) Adc_Handle(&ADC1Handle);
#endif
#if defined(ADC2) && defined(USE_ADC2)
    if ( ADC2Handle.bSequenceDone ) Adc_Handle(&ADC2Handle);
#endif
#if defined(ADC3) && defined(USE_ADC3)
    if ( ADC3Handle.bSequenceDone ) Adc_Handle(&ADC3Handle);
#endif
}



#if defined(ADC1) && defined(USE_ADC1)
    AdcHandleT ADC1Handle;
    static uint16_t DMAMEM adc1_dmabuf[ADC_DMABUF_SIZE];
    static DMA_HandleTypeDef hdma_adc1_rx;
    static const HW_DmaType dmarx_adc1 = { &hdma_adc1_rx, ADC1_RX_DMA };

    static const HW_GpioList_ADC gpio_adc = {
        .num  = 4,
        .gpio = { 
            ADC1CH_REFINT,
            ADC1CH_TEMPSENSOR,
/* example*/ ADC123_CH_3,
/* example*/ ADC123_CH_4, 
        }
    };

    static const ADC_AdditionalDataType additional_adc1 = {
        .myAdcHandle = &ADC1Handle,
        .myDmaBuf    = adc1_dmabuf,
    };

    const HW_IrqList irq_adc1 = {
        .num = 1,
        .irq = { ADC1_IRQ },
    };

    const HW_DeviceType HW_ADC1 = {
        .devName        = "ADC1",
        .devBase        = ADC1,
        .devGpioAF      = NULL,
        .devGpioIO      = NULL,
        .devGpioADC     = &gpio_adc,
        .devType        = HW_DEVICE_ADC,
        .devData        = &additional_adc1,
        .devIrqList     = 
            #if defined(USE_ADC1) && defined(ADC1_USE_IRQ) 
                &irq_adc1,
            #else
                NULL,
            #endif
        .devDmaRx       = 
            #if defined(USE_ADC1) && defined(ADC1_USE_DMA) 
                &dmarx_adc1,
             #else
                NULL,
             #endif
        .devDmaTx       = NULL,
        .Init           = ADC_Init,
        .DeInit         = ADC_DeInit,
        .OnFrqChange    = NULL,
        .AllowStop      = ADC_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#if defined(ADC3) && defined(USE_ADC3)
    AdcHandleT ADC3Handle;
    static uint16_t DMAMEM adc3_dmabuf[ADC_DMABUF_SIZE];
    static DMA_HandleTypeDef hdma_adc3_rx;
    static const HW_DmaType dmarx_adc3 = { &hdma_adc3_rx, ADC3_RX_DMA };

    static const HW_GpioList_ADC gpio_adc = {
        .num  = 2,
        .gpio = { 
            ADC3CH_REFINT,
            ADC3CH_TEMPSENSOR,
/*example */  ADC123_CH_3,
/*example */  ADC123_CH_4,
        }
    };

    static const ADC_AdditionalDataType additional_adc3 = {
        .myAdcHandle = &ADC3Handle,
        .myDmaBuf    = adc3_dmabuf,
    };

    const HW_IrqList irq_adc3 = {
        .num = 1,
        .irq = { ADC3_IRQ },
    };

    const HW_DeviceType HW_ADC3 = {
        .devName        = "ADC3",
        .devBase        = ADC3,
        .devGpioAF      = NULL,
        .devGpioIO      = NULL,
        .devGpioADC     = &gpio_adc,
        .devType        = HW_DEVICE_ADC,
        .devData        = &additional_adc3,
        .devIrqList     = 
            #if defined(USE_ADC3) && defined(ADC3_USE_IRQ) 
                &irq_adc3,
            #else
                NULL,
            #endif
        .devDmaRx       = 
            #if defined(USE_ADC3) && defined(ADC3_USE_DMA) 
                &dmarx_adc3,
             #else
                NULL,
             #endif
        .devDmaTx       = NULL,
        .Init           = ADC_Init,
        .DeInit         = ADC_DeInit,
        .OnFrqChange    = NULL,
        .AllowStop      = ADC_AllowStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif

#endif // #if defined(USE_ADC1) || defined(USE_ADC2) || defined(USE_ADC3)



/**
  * @}
  */


