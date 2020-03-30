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
#include "dev/devices.h"
#include "config/adc_config.h"
#include "task/minitask.h"
#include "system/periodic.h"


#define DEBUG_ADC 0


#if DEBUG_MODE > 0 && DEBUG_ADC > 0
    #include "debug_helper.h"
    #include "system/profiling.h"
#else
    #include <stdio.h>
#endif

#if defined(STM32L476xx) || defined(STM32L496xx)
    #define ADC_SAMPLETIME_FAST     ADC_SAMPLETIME_24CYCLES_5
    #define ADC_SAMPLETIME_SLOW     ADC_SAMPLETIME_47CYCLES_5
    #define ADC_CLOCKSOURCE         RCC_ADCCLKSOURCE_SYSCLK
#elif defined(STM32H745xx)
    #define ADC_SAMPLETIME_FAST     ADC_SAMPLETIME_32CYCLES_5
    #define ADC_SAMPLETIME_SLOW     ADC_SAMPLETIME_64CYCLES_5
    #define ADC_CLOCKSOURCE         RCC_ADCCLKSOURCE_CLKP
#else
    #error "No setup for ADC sample times"
#endif

/* My macros --------------------------------------------------------------------*/
#define GET_SEQ_LEN(hadc)       ( ( hadc->Instance->SQR1 & ADC_SQR1_L_Msk ) + 1 )

/* Public typedef ---------------------------------------------------------------*/


typedef struct {
     AdcHandleT *myAdcHandle;       /* Ptr to my associated handle */
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
 * Setup the ADC_InitTypeDef structure for 
 * - 12 bit resolution, 16x oversampling, right aligned result, 
 * - ADC start by software, number of sequencer channels as specified,
 * - EOC at end of sequence,
 * 0 on failure 
 *****************************************************************************/
static bool Adc_SetupInit(const HW_DeviceType *self, uint8_t nrofChannels )
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    ADC_InitTypeDef *Init       = &hAdc->Init;
    ADC_OversamplingTypeDef *o  = &Init->Oversampling;


    /* 
     * Do the neccessary ADC settings. Theses settings are done only once and are kept 
     * in Init-Struct as long as the variable is in scope.
     */
    Init->ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;         /* Synchronous clock mode, input ADC clock divided by 2*/
    Init->Resolution            = ADC_RESOLUTION_12B;               /* 12-bit resolution for converted data */
    Init->OversamplingMode      = ENABLE;                           /* enable oversampling */
    #if defined(STM32L476xx) || defined(STM32L496xx)
        Init->DataAlign             = ADC_DATAALIGN_RIGHT;              /* Right-alignment for converted data */
        Init->DMAContinuousRequests = DISABLE;                          /* ADC DMA for only one sequence */
        o->Ratio                    = ADC_OVERSAMPLING_RATIO_16;        /* 16x oversampling */
        o->RightBitShift            = ADC_RIGHTBITSHIFT_4;              /* shift result, so we have again a 12bit result after oversampling */
    #elif defined(STM32H745xx)
        o->Ratio                    = 16;                               /* 16x oversampling */
        o->RightBitShift            = ADC_RIGHTBITSHIFT_4;              /* shift result, so we have again a 12bit result after oversampling */
        Init->ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;/* ADC DMA for only one sequence */
    #else
        #error "NO ADC setup for selected STM32 type"
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
    Init->LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
    Init->ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled (one shot conversion) */
    Init->NbrOfConversion       = nrofChannels;                  /* As passed */
    Init->DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because continuous mode is disabled */
    Init->NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
    Init->ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    Init->ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
    Init->Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */

    o->OversamplingStopReset    = ADC_REGOVERSAMPLING_CONTINUED_MODE;   /* Preserve Oversampling buffer during injected conversions */
    o->TriggeredMode            = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;     /* All samples done in one step */

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
 *****************************************************************************/
 static void Adc_SetupVrefintChannel(ADC_ChannelConfTypeDef *c, uint32_t rankidx) 
{
    c->Channel      = ADC_CHANNEL_VREFINT;         /* Internal Vrefint channel ( only valid for ADC1 ) */
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
 * Moreover, ADC1 requires a special handling: 
 * As we added measurement of Vrefint in the first position, we will reomve the first
 * result from the sequence here and use it to update the Vdda value
 * 
 * 
 *************************************************************************************/
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{   
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0
        DEBUG_PRINTF("%d>",ProfilerGetMicrosecond());
    #endif

    HAL_ADC_Stop_DMA(hadc);
    AdcHandleT *myHandle = Adc_GetMyHandleFromHalHandle(hadc);
    if (!myHandle ) return;

    uint8_t seqlen = GET_SEQ_LEN(hadc);
    if ( hadc->Instance == ADC1 ) {
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

#if defined(STM32L476xx) || defined(STM32L496xx)
static inline __attribute__((always_inline))
bool AdcCalibrate ( ADC_HandleTypeDef *hAdc )
{
  return    
       (HAL_ADCEx_Calibration_Start(hAdc, ADC_SINGLE_ENDED      ) ==  HAL_OK) 
    && (HAL_ADCEx_Calibration_Start(hAdc, ADC_DIFFERENTIAL_ENDED) ==  HAL_OK)
  ;
}
#elif defined(STM32H745xx)
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
    Adc_SetupInit(self, 1);

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
    if ( adt->myAdcHandle->bSequence ) Adc_SetupInit(self, 1);

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

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    uint32_t raw;

    /* Vrefint can be measured via ADC1 only */
    if ( self->devBase != (void *)ADC1_BASE ) return false;

    /* Check for single channel being initialized */
    if ( adt->myAdcHandle->bSequence ) Adc_SetupInit(self, 1);

    ADC_ChannelConfTypeDef c;  
    Adc_SetupVrefintChannel(&c,1);

    /* Configure channel */
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
    #if defined(STM32L476xx) || defined(STM32L496xx)
        UNUSED(self);
        return __LL_ADC_COMMON_INSTANCE(self->devBase);
    #elif defined(STM32H745xx)
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

    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    uint32_t raw;

    /* Chiptemp can be measured via ADC1 or ADC3 */
    if ( self->devBase != (void *)ADC1_BASE && self->devBase != (void *)ADC3_BASE ) return false;

    /* Check for single channel being initialized */
    if ( adt->myAdcHandle->bSequence ) Adc_SetupInit(self, 1);
    
    ADC_ChannelConfTypeDef c;  
    c.Channel      = ADC_CHANNEL_TEMPSENSOR;      /* Internal TempSensor ch. ( valid for ADC1/ADC3 ) */
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
  
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF(">%d",ProfilerGetMicrosecond());
    #endif
    
    if ( !ADC_SingleConversion(hAdc, &raw) ) return false;
    
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF(",%d>\n",ProfilerGetMicrosecond());
    #endif

    adt->myAdcHandle->chiptemp  = __HAL_ADC_CALC_TEMPERATURE(adt->myAdcHandle->vdda, raw, ADC_RESOLUTION_12B );
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF("Chiptemp=%d degC\n", adt->myAdcHandle->chiptemp);
    #endif
    return true;
}

/******************************************************************************
 * Put all defined channels for tha ADC in one big channel group
 *****************************************************************************/
void ADC_SetupGroup (const HW_DeviceType *self)
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
    if ( hAdc->Instance == ADC1 ) nrofch++;
    Adc_SetupInit(self, nrofch);

    /* if Instance is ADC1, the Vrefint channel is always inserted as the first channel */
    if ( hAdc->Instance == ADC1 ) {
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
    #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
        DEBUG_PRINTF(">%d",ProfilerGetMicrosecond());
    #endif
    uint32_t length = GET_SEQ_LEN(hAdc);
    /* Start conversion  */
    if (HAL_ADC_Start_DMA(hAdc, (uint32_t *)adt->myAdcHandle->dmabuf, length ) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PUTS("Measuring Via DMA failed");
        #endif
        return false;
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




bool ADC_Init(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt     = ADC_GetAdditionalData(self);
    AdcHandleT *myHandle            = adt->myAdcHandle;  
    ADC_HandleTypeDef *hAdc         = &myHandle->hAdc;
    hAdc->Instance                  = (ADC_TypeDef *)self->devBase;

    myHandle->bVdda_valid   = false;
    myHandle->bSequence     = false;
  
    ADC_ClockInit(self, true);

    GpioADCInitAll(self->devGpioADC);

    /* DeInit before Init */
    if (HAL_ADC_DeInit(hAdc) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_ADC > 0 
            DEBUG_PRINTF("DeInit before Init of %s failed\n", self->devName);
        #endif
        return false;
    }

    if ( !Adc_SetupInit(self, 1 ) ) return false;

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
    
    if ( self->devBase == (void *)ADC1_BASE ) ADC_MeasureVdda((void *)self);

    const HW_IrqType *irq;
    const HW_DmaType *dma;

    /* Configure the NVIC, enable interrupts */
    HW_SetAllIRQs(self->devIrqList, true);

    /* Enable DMA, if specified, ADC has only Rx dma */            
    if ( self->devDmaRx ) {
      /*make sure, that also interrupts are configured, MA won't work without */
      if ( self->devIrqList->num == 0 ) {
         #if DEBUG_MODE > 0 && DEBUG_ADC > 0
             DEBUG_PRINTF("Init of %s: Use of DMA requires interrupts \n", self->devName);
         #endif
         myHandle->bInitialized  = false;
         return false;
      }

      HW_SetDmaChClock(NULL, self->devDmaRx);
      // Take the first interrupt to copy prio and subprio to dma channel interrupts
      irq = self->devIrqList->irq;
      dma = self->devDmaRx;
      if (dma ) {
         AdcDmaChannelInit( myHandle ,dma);
         HAL_NVIC_SetPriority(dma->dmaIrqNum, irq->irq_prio, irq->irq_subprio);
         HAL_NVIC_EnableIRQ(dma->dmaIrqNum);
      }
    }

    return true;
}

void ADC_DeInit(const HW_DeviceType *self)
{
    ADC_AdditionalDataType *adt = ADC_GetAdditionalData(self);
    ADC_HandleTypeDef *hAdc     = &adt->myAdcHandle->hAdc;
    const HW_DmaType *dma;

    HAL_ADC_Stop(hAdc);

    GpioADCDeInitAll(self->devGpioADC);
  
    /* disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
    dma = self->devDmaRx;
    if(dma) {
       HAL_DMA_DeInit(dma->dmaHandle);
       HAL_NVIC_DisableIRQ(dma->dmaIrqNum);
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

    for ( uint32_t i = 0; i<myHandle->seqLen; i++ )
        printf("%d ", myHandle->seqResultPtr[i]);
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
    AtSecond ( 56, ADC_MeasureVdda, (void *)&USER_ADC, "Measure Vdda");
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
    static DMA_HandleTypeDef hdma_adc1_rx;
    static const HW_DmaType dmarx_adc1 = { &hdma_adc1_rx, ADC1_RX_DMA };

    static const HW_GpioList_ADC gpio_adc = {
        .num  = 2,
        .gpio = { 
            ADC1CH_REFINT,
            ADC1CH_TEMPSENSOR,
// example  ADC123_CH_3,
// example  ADC123_CH_4,
        }
    };

    static const ADC_AdditionalDataType additional_adc1 = {
        .myAdcHandle = &ADC1Handle,
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
    static DMA_HandleTypeDef hdma_adc3_rx;
    static const HW_DmaType dmarx_adc3 = { &hdma_adc3_rx, ADC3_RX_DMA };

    static const HW_GpioList_ADC gpio_adc = {
        .num  = 2,
        .gpio = { 
            ADC3CH_REFINT,
            ADC3CH_TEMPSENSOR,
// example  ADC323_CH_3,
// example  ADC323_CH_4,
        }
    };

    static const ADC_AdditionalDataType additional_adc3 = {
        .myAdcHandle = &ADC3Handle,
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


