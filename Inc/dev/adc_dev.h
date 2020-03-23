/*
 ******************************************************************************
 * @file    io_dev.h 
 * @author  Rainer
 * @brief   plain input/output pins, w or w/o interrupt
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_DEV_H
#define __ADC_DEV_H

#include "config/config.h"
#include "system/exti_handler.h"
#include "hw_device.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Public typedef -----------------------------------------------------------------------*/
typedef struct AdcHandleType {
	ADC_HandleTypeDef hAdc;           /* Embedded HAL ADC_HandleTypedef Structure    */
        uint32_t adc_calfact;             /* dynamic calibration factor (for single ended and differential) */
        uint16_t dmabuf[17];              /* internal DMA buffer for sequence conversion */
        uint16_t chiptemp;                /* actual chiptemp value                       */
        uint16_t vdda;                    /* actual Vref value                           */
        uint8_t bVdda_valid;              /* indicates a valid vref value                */
        uint8_t bInitialized;             /* flag for "device is initialized"            */
        uint8_t bSequence;                /* flag for "a sequence is configured"         */ 
        uint8_t bSequenceDone;            /* flag for "sequence is converted fully"      */
        uint8_t seqLen;                   /* length of converted sequence (in words)     */
        uint16_t *seqResultPtr;           /* Ptr to array of converted values            */
 } AdcHandleT; 


/* Public functions -------------------------------------------------------------*/
bool     ADC_Calibrate           (const HW_DeviceType *self);
void     ADC_MeasureVdda         (void *arg);
bool     ADC_MeasureChipTemp     (const HW_DeviceType *self);
uint32_t ADC_MeasureChannel      ( const HW_DeviceType *self, uint32_t channel_idx );

void     ADC_SetupGroup          (const HW_DeviceType *self);
bool     ADC_MeasureGroup        (const HW_DeviceType *self);

void     ADC_DisableRefintCh     (const HW_DeviceType *self);
void     ADC_DisableAllInternalCh(const HW_DeviceType *self);

uint16_t ADC_GetVdda             (const HW_DeviceType *self);

void     task_init_adc           (void);
void     task_handle_adc         (uint32_t);

/* Global variables ------------------------------------------------------------*/
#if defined(ADC1) && defined(USE_ADC1)
    extern const HW_DeviceType HW_ADC1;
    extern AdcHandleT ADC1Handle;
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ADC_DEV_H */
