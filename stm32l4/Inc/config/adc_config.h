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
#include "stm32l4xx.h"

/*
 **** 003 **** 
 * Remap "new" DMAMUX input lines to "old" fixed DMA-Request numbers 
 */
#if defined(STM32L476xx) || defined(STM32L496xx)
    #define DMA_REQUEST_ADC1       DMA_REQUEST_0
    #define DMA_REQUEST_ADC2       DMA_REQUEST_0
#endif

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
    /* ADC1 has internal channels Vrefint, Vts, Vbat/3 */
    #define ADC1CH_REFINT       { ADC_CHANNEL_VREFINT,      ADC_SAMPLETIME_47CYCLES_5, NULL, 0,"Vrefint" }
    #define ADC1CH_TEMPSENSOR   { ADC_CHANNEL_TEMPSENSOR,   ADC_SAMPLETIME_24CYCLES_5, NULL, 0, "ChipTemp" }
    #define ADC1CH_VBAT         { ADC_CHANNEL_VBAT,         ADC_SAMPLETIME_24CYCLES_5, NULL, 0, "Vbat" }
 
    /* Definition ADC1 NVIC */
    #define ADC1_IRQ                             { ADC1_2_IRQn, ADC_IRQ_PRIO, 0 }
    #define ADC1_IRQHandler                      ADC1_2_IRQHandler
    #define ADC1_RX_DMA                          DMA1_Channel1, DMA_REQUEST_ADC1, DMA1_Channel1_IRQn, DMA_PRIORITY_HIGH
    #define ADC1_DMA_IRQHandler                  DMA1_Channel1_IRQHandler
                                                    
#endif

#if defined(USE_ADC2) && defined(ADC2)
    /* ADC2 has internal channels DAC1 CH1 and CH2 */
    #define ADC2CH_DAC1CH1      { ADC_CHANNEL_DAC1CH1_ADC2, ADC_SAMPLETIME_47CYCLES_5, NULL, 0, "Dac1Ch1" }
    #define ADC2CH_DAC1CH2      { ADC_CHANNEL_DAC1CH2_ADC2, ADC_SAMPLETIME_47CYCLES_5, NULL, 0, "Dac1Ch2" }

    /* Definition ADC2 NVIC */
    #define ADC2_IRQ                             { ADC2_2_IRQn, ADC_IRQ_PRIO, 0 }
    #define ADC2_IRQHandler                      ADC2_2_IRQHandler
    #define ADC2_RX_DMA                          DMA1_Channel2, DMA_REQUEST_ADC2, DMA1_Channel2_IRQn, DMA_PRIORITY_HIGH
    #define ADC2_DMA_IRQHandler                  DMA1_Channel2_IRQHandler
#endif

#if defined(USE_ADC3) && defined(ADC3)
    /* ADC2 has internal channels DAC1 CH1 and CH2, Vts, Vbat/3*/
    #define ADC3CH_DAC1CH1      { ADC_CHANNEL_DAC1CH1_ADC3, ADC_SAMPLETIME_47CYCLES_5, NULL, 0, "Dac1Ch1" }
    #define ADC3CH_DAC1CH2      { ADC_CHANNEL_DAC1CH2_ADC3, ADC_SAMPLETIME_47CYCLES_5, NULL, 0, "Dac1Ch2" }
    #define ADC3CH_TEMPSENSOR   { ADC_CHANNEL_TEMPSENSOR,   ADC_SAMPLETIME_24CYCLES_5, NULL, 0, "ChipTemp" }
    #define ADC3CH_VBAT         { ADC_CHANNEL_VBAT,         ADC_SAMPLETIME_24CYCLES_5, NULL, 0, "Vbat" }

    /* Definition ADC3 NVIC */
    #define ADC3_IRQ                             { ADC3_IRQn, ADC_IRQ_PRIO, 0 }
    #define ADC3_IRQHandler                      ADC3_IRQHandler
    #define ADC3_RX_DMA                          DMA1_Channel3, DMA_REQUEST_ADC3, DMA1_Channel3_IRQn, DMA_PRIORITY_HIGH
    #define ADC3_DMA_IRQHandler                  DMA1_Channel3_IRQHandler
#endif

#if defined(USE_ADC1) || defined(USE_ADC2) || defined(USE_ADC3)
    /* The following external channels are common to ADC1 .. ADC3 */
    #define ADC123_CH_1         { ADC_CHANNEL_1,  ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_0, "ADC Ch.1" }
    #define ADC123_CH_2         { ADC_CHANNEL_2,  ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_1, "ADC Ch.2" }
    #define ADC123_CH_3         { ADC_CHANNEL_3,  ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_2, "ADC Ch.3" }
    #define ADC123_CH_4         { ADC_CHANNEL_4,  ADC_SAMPLETIME_12CYCLES_5, GPIOC, GPIO_PIN_3, "ADC Ch.4" }
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
        /* The following channels are specific to ADC3 in larger package types only */
        #define ADC3_CH_6       { ADC_CHANNEL_6,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_3, "ADC Ch.6" }
        #define ADC3_CH_7       { ADC_CHANNEL_7,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_4, "ADC Ch.7" }
        #define ADC3_CH_8       { ADC_CHANNEL_8,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_5, "ADC Ch.8" }
        #define ADC3_CH_9       { ADC_CHANNEL_9,  ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_6, "ADC Ch.9" }
        #define ADC3_CH_10      { ADC_CHANNEL_10, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_7, "ADC Ch.10" }
        #define ADC3_CH_11      { ADC_CHANNEL_11, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_8, "ADC Ch.11" }
        #define ADC3_CH_12      { ADC_CHANNEL_12, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_9, "ADC Ch.12" }
        #define ADC3_CH_13      { ADC_CHANNEL_13, ADC_SAMPLETIME_12CYCLES_5, GPIOF, GPIO_PIN_10, "ADC Ch.13" }
    #endif
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __ADC_CONFIG_H */