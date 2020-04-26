/**
  ******************************************************************************
  * @file    adc_config.h
  * @author  Rainer
  * @brief   configuration of ADC channels and input pins
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_CONFIG_H
#define __ADC_CONFIG_H

#include "config/devices_config.h"
#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************************/
/* ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC*/
/* ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC*/
/* ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC*/
/* ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC*/
/******************************************************************************************/


/* ----- Fixed part, don't change ------------------------------------------------------- */
/* ----- Neccessary pin selections have to be made in "adc_dev.c" ----------------------- */
/* ----- This file should be regarded "read only" --------------------------------------- */

#if defined(USE_ADC1) && defined(ADC1)
#endif

#if defined(USE_ADC2) && defined(ADC2)
    /* ADC2 has internal channels DAC1 CH1 and CH2 */
    #define ADC2CH_DAC1CH1      { ADC_CHANNEL_DAC1CH1_ADC2, ADC_SAMPLETIME_47CYCLES_5, NULL, 0 }
    #define ADC2CH_DAC1CH2      { ADC_CHANNEL_DAC1CH2_ADC2, ADC_SAMPLETIME_47CYCLES_5, NULL, 0 }

    /* Definition ADC2 NVIC */
    #define ADC2_IRQ                             { ADC2_2_IRQn, ADC_IRQ_PRIO, 0 }
    #define ADC2_IRQHandler                      ADC2_2_IRQHandler
    #define ADC2_RX_DMA                          DMA1_Channel2, DMA_REQUEST_0, DMA1_Channel2_IRQn
    #define ADC2_DMA_IRQHandler                  DMA1_Channel2_IRQHandler
#endif

#if defined(USE_ADC3) && defined(ADC3)
    /* ADC1 has internal channels Vrefint, Vts, Vbat/3 */
    #define ADC3CH_REFINT       { ADC_CHANNEL_VREFINT,      ADC_SAMPLETIME_64CYCLES_5, NULL, 0, "Refint" }
    #define ADC3CH_TEMPSENSOR   { ADC_CHANNEL_TEMPSENSOR,   ADC_SAMPLETIME_32CYCLES_5, NULL, 0, "ChipTemp" }
    #define ADC3CH_VBAT         { ADC_CHANNEL_VBAT,         ADC_SAMPLETIME_32CYCLES_5, NULL, 0, "Vbat" }
 
    /* Definition ADC1 NVIC */
    #define ADC3_IRQ                             { ADC_IRQn, ADC_IRQ_PRIO, 0 }
    #define ADC3_IRQHandler                      ADC_IRQHandler
    #define ADC3_RX_DMA                          DMA1_Stream6, DMA_REQUEST_ADC3,  DMA1_Stream6_IRQn
    #define ADC3_DMA_IRQHandler                  DMA1_Stream6_IRQHandler
                                                    
#endif

#if defined(USE_ADC1) || defined(USE_ADC2) || defined(USE_ADC3)
    /* The following external channels are common to ADC1 .. ADC3 */
    #define ADC123_CH_1         { ADC_CHANNEL_1,  ADC_SAMPLETIME_32CYCLES_5, GPIOC, GPIO_PIN_0, "ADC Ch.1" }
    #define ADC123_CH_2         { ADC_CHANNEL_2,  ADC_SAMPLETIME_32CYCLES_5, GPIOC, GPIO_PIN_1, "ADC Ch.2" }
    #define ADC123_CH_3         { ADC_CHANNEL_3,  ADC_SAMPLETIME_32CYCLES_5, GPIOC, GPIO_PIN_2, "ADC Ch.3" }
    #define ADC123_CH_4         { ADC_CHANNEL_4,  ADC_SAMPLETIME_32CYCLES_5, GPIOC, GPIO_PIN_3, "ADC Ch.4" }
    #if defined(USE_ADC1) || defined(USE_ADC2)
    /* The following external channels are common to ADC1 and ADC2 */
        #define ADC12_CH_5      { ADC_CHANNEL_5,  ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_0, "ADC Ch.5" }
        #define ADC12_CH_6      { ADC_CHANNEL_6,  ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_1, "ADC Ch.6" }
        #define ADC12_CH_7      { ADC_CHANNEL_7,  ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_2, "ADC Ch.7" }
        #define ADC12_CH_8      { ADC_CHANNEL_8,  ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_3, "ADC Ch.8" }
        #define ADC12_CH_9      { ADC_CHANNEL_9,  ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_4, "ADC Ch.9" }
        #define ADC12_CH_10     { ADC_CHANNEL_10, ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_5, "ADC Ch.10" }
        #define ADC12_CH_11     { ADC_CHANNEL_11, ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_6, "ADC Ch.11" }
        #define ADC12_CH_12     { ADC_CHANNEL_12, ADC_SAMPLETIME_12CYCLES_5, GPIOA, GPIO_PIN_7, "ADC Ch.12" }
        #define ADC12_CH_13     { ADC_CHANNEL_13, ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_4, "ADC Ch.13" }
        #define ADC12_CH_14     { ADC_CHANNEL_14, ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_5, "ADC Ch.14" }
        #define ADC12_CH_15     { ADC_CHANNEL_15, ADC_SAMPLETIME_12CYCLES_5, GPIOB, GPIO_PIN_0, "ADC Ch.15" }
        #define ADC12_CH_16     { ADC_CHANNEL_16, ADC_SAMPLETIME_12CYCLES_5, GPIOB, GPIO_PIN_1, "ADC Ch.16" }
    #else
        /* The dollowing channels are specific to ADC3 in larger package types only */
        #define ADC3_CH_6       { ADC_CHANNEL_6,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_3, "ADC Ch.3" }
        #define ADC3_CH_7       { ADC_CHANNEL_7,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_4, "ADC Ch.4" }
        #define ADC3_CH_8       { ADC_CHANNEL_8,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_5, "ADC Ch.5" }
        #define ADC3_CH_9       { ADC_CHANNEL_9,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_6, "ADC Ch.6" }
        #define ADC3_CH_10      { ADC_CHANNEL_10, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_7, "ADC Ch.7" }
        #define ADC3_CH_11      { ADC_CHANNEL_11, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_8, "ADC Ch.8" }
        #define ADC3_CH_12      { ADC_CHANNEL_12, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_9, "ADC Ch.9" }
        #define ADC3_CH_13      { ADC_CHANNEL_13, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_10, "ADC Ch.10" }
    #endif
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __ADC_CONFIG_H */