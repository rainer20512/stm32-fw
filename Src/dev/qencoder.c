/*
 ******************************************************************************
 * @file    qencode.c
 * @author  Rainer
 * @brief   Quadrature decoder on basis of a STM32 timer that is capable 
 *          of quadrature decoding
 *          Select a proper timer by configuration
 *
 *****************************************************************************/

/** @addtogroup Quadrature Decoder
  * @{
   
*/
#include "config/config.h"

#if USE_QENCODER > 0 

/* Debug ------------------------------------------------------------------------*/
#define DEBUG_QENC          0

#include "config/devices_config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"
#include "system/timer_handler.h"
#include "system/exti_handler.h"
#include "dev/devices.h"
#include "config/qencode_config.h"
#include "timer.h"
#include "rtc.h"

#include "debug_helper.h"

/* My macros --------------------------------------------------------------------*/
#define CNT_STARTVAL        8192    /* initial CNT value to avoid zero crossing  */
#define QENC_DEBOUNCE_MS    12      /* Debounce time for push button in ms       */
#define QENC_DBLCLICK_MS    200     /* Max time to handle two clicks as DblClick */
#define QENC_CH1_IDX        0       /* Idx of Encoder Channel 1 in gpiolist      */
#define QENC_CH2_IDX        1       /* Idx of Encoder Channel 2 in gpiolist      */
#define QENC_BTN_IDX        2       /* Idx of optional Encoder PushBtn in gpiol. */ 


/* Public typedef ---------------------------------------------------------------*/


typedef struct {
    QEncHandleT *myQEncHandle;       /* my associated handle */
    TIM_TypeDef *myTimer;            /* my associated timer */
    uint8_t     myCountsPerNotch;    /* as the name says */
} QEnc_AdditionalDataType;


/* Forward declarations -------------------------------------------------------------*/



/* Private or driver functions ------------------------------------------------------*/
static QEnc_AdditionalDataType * QEnc_GetAdditionalData(const HW_DeviceType *self)
{
    return (QEnc_AdditionalDataType *)(self->devData);
}


static void QEncGpioInitAF(const HW_GpioList_AF *gpioaf)
{
    GPIO_InitTypeDef Init;
    
    /* Init both encoder GPIO pins */
    if ( gpioaf->num  <= QENC_CH2_IDX ) {
        DEBUG_PUTS("Qencoder Init: Error: Need at least two inputs");
        return;
    }
    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_LOW;

    GpioAFInitOne( gpioaf->gpio+QENC_CH1_IDX, &Init );
    GpioAFInitOne( gpioaf->gpio+QENC_CH2_IDX, &Init );
}


static void QEncGpioInit(const HW_GpioList_AF *gpioaf, const HW_GpioList_IO *gpioio)
{
    /* Init both encoder input pins */
    QEncGpioInitAF(gpioaf);

    /* Init optional PushButton Pin as EXTI Pin, pulled up with interrupt on falling edge */
    if ( gpioio ) GpioIOInitAll(gpioio);
}


static bool QEncTmrInit(TIM_HandleTypeDef *htim)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 65535;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK) {
     DEBUG_PUTS("QencTmrInit: Cannot init timer");
     return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
  {
     DEBUG_PUTS("QencTmrInit: Cannot init timer trigger");
     return false;
  }

  return true;
}


static QEncHandleT *QEnc_GetMyHandle ( TIM_HandleTypeDef *htim )
{
    if ( htim == &QEnc1Handle.htim ) return &QEnc1Handle;
    /* Check for more QEncHandleT variables here */
    DEBUG_PRINTF("QEncGetMyHandle: No Handle found");
    return NULL;
}
/* Timer Capture Callback ------------------------------------------------------------*/
void QEnc_CaptureCallback (TIM_HandleTypeDef *htim)
{
    int32_t delta;
    #if DEBUG_MODE > 0 && DEBUG_QENC > 2
        DEBUG_PRINTF("Cnt=%d, SR=%04x\r\n",htim->Instance->CNT, htim->Instance->SR);
    #endif
    QEncHandleT *myHandle = QEnc_GetMyHandle(htim);
    if ( myHandle ) {
        if ( myHandle->OnRotate ) {
            delta = ((int32_t)htim->Instance->CNT) - myHandle->last_tmr;
            delta = ( delta / myHandle->cntPerNotch ) * myHandle->cntPerNotch;
            if ( ABS(delta) >= myHandle->cntPerNotch ) { 
                #if DEBUG_MODE > 0 && DEBUG_QENC > 0
                    DEBUG_PRINTF("OnRotate(%d)\n", delta/ myHandle->cntPerNotch);
                #endif
                myHandle->OnRotate ( delta / myHandle->cntPerNotch );
                myHandle->last_tmr += delta;
            }
        }
    }
}

/* Encoder dblclick timer Callback --------------------------------------------------*/
void QEnc_OnDblClickTimer( uint32_t arg )
{
    #if DEBUG_MODE > 0 && DEBUG_QENC > 1
        DEBUG_PUTS("DblClickTmo");
    #endif
    /* Just reset the "in DblClick Interval" flag */
    QEncHandleT *myHandle = (QEncHandleT *)arg;
    myHandle->bDblClkItvl = 0;
}


/* Encoder debounce timer Callback ---------------------------------------------------*/
void QEnc_OnClickTimer( uint32_t arg )
{
    QEncHandleT *myHandle = (QEncHandleT *)arg;

    #if DEBUG_MODE > 0 && DEBUG_QENC > 2
        DEBUG_PUTS("OnClickTimer");
    #endif

    /* Get input pin value */
    uint16_t inp = myHandle->myIOPin->gpio->IDR & myHandle->myIOPin->pin;

    /* Reenable Pin Change interrupt on PushButton */
    Exti_EnableIrq(myHandle->myIOPin->pin);

    /* if input value differs from value, that triggered the interrupt,           */
    /* regard as jitter and do nothing. Also, on low->high transition, do nothing */
    if ( inp != myHandle->last_btnval ) return;
    if ( inp ) return;

    /* If within DblClick interval, then activate DblClick Callback */
    if ( myHandle->bDblClkItvl ) {
        if ( myHandle->OnDblClick ) myHandle->OnDblClick();
        #if DEBUG_MODE > 0 && DEBUG_QENC > 0
            DEBUG_PUTS("OnDblClick");
        #endif
     } else {
        /* Otherwise start DblClickInterval timer and call Click Callback */
        myHandle->bDblClkItvl = 1;
        MsTimerSetRel ( MILLISEC_TO_TIMERUNIT(QENC_DBLCLICK_MS), 0, QEnc_OnDblClickTimer, arg );
        if ( myHandle->OnClick ) myHandle->OnClick();
        #if DEBUG_MODE > 0 && DEBUG_QENC > 0
            DEBUG_PUTS("OnClick");
        #endif
     }
}


/* Encoder button press Callback -----------------------------------------------------*/
void QEnc_OnClick ( uint16_t pin, uint16_t pinvalue, void *arg )
{
    UNUSED(pin);
    
    #if DEBUG_MODE > 0 && DEBUG_QENC > 1
        DEBUG_PRINTF("Qenc_OnClick(%d)\n", pinvalue);
    #endif

    QEncHandleT *myHandle = (QEncHandleT *)arg;
    myHandle->last_btnval = pinvalue;

    /* temporarily deactivate further Interrupts due to bouncing */
    Exti_DisableIrq(myHandle->myIOPin->pin);
    
    /* Start Debounce timer */
    MsTimerReUseRel ( myHandle->myTimerId, MILLISEC_TO_TIMERUNIT(QENC_DEBOUNCE_MS), 0, QEnc_OnClickTimer, (uint32_t)arg );
}

/* Public or driver functions --------------------------------------------------------*/


/*****************************************************************************
 * Set the rotate-callbacks 
 * NULL as parameter means: Deactivate callback
 ****************************************************************************/
void QEnc_SetRotateCallback  (const HW_DeviceType *self, QEncEncCB onRotateCB ) 
{
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  
    myHandle->OnRotate      = onRotateCB;
}

/*****************************************************************************
 * Set the button press callback
 * NULL as parameter means: Deactivate callback
 ****************************************************************************/
void QEnc_SetClickCallback  (const HW_DeviceType *self, QEncClickCB onClickCB )
{
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  
    myHandle->OnClick       = onClickCB;
}

/*****************************************************************************
 * Set the button doubleclick callback
 * NULL as parameter means: Deactivate callback
 * @note: Before double click event, a single click event is generated!
 ****************************************************************************/
void QEnc_SetDblClickCallback  (const HW_DeviceType *self, QEncClickCB onDblClickCB )
{
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  
    myHandle->OnDblClick    = onDblClickCB;
}

/*****************************************************************************
 * Set ALL callbacks 
 * NULL as parameter means: Deactivate callback
 * @note: Before double click event, a single click event is generated!
 ****************************************************************************/
void QEnc_SetCallbacks (const HW_DeviceType *self, QEncEncCB   onRotateCB, QEncClickCB onClickCB, QEncClickCB onDblClickCB )
{
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  
    myHandle->OnRotate      = onRotateCB;
    myHandle->OnClick       = onClickCB;
    myHandle->OnDblClick    = onDblClickCB;
}



/*****************************************************************************
 * Deactivate the quadrature decoder device 
 * - The two encoder input Pins are set Input with pullup
 * - The timer clock is started again
 ****************************************************************************/
void QEnc_Activate(const HW_DeviceType *self)
{    
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  

    /* Activate Encoder GPIO pins */
    QEncGpioInit(self->devGpioAF, self->devGpioIO);


    /* Enable TMR clock */
    HW_SetHWClock(myHandle->htim.Instance, true);

    /* Initialize CNT to avoid zero crossing */
    myHandle->htim.Instance->CNT = CNT_STARTVAL;
    myHandle->last_tmr = CNT_STARTVAL;
    
    HAL_TIM_Encoder_Start_IT(&myHandle->htim, TIM_CHANNEL_1);

    /* Notice, that Encoder is activated */
    myHandle->bActivated = true;
}

/*****************************************************************************
 * Deactivate the quadrature decoder device to reduce power dissipation
 * - The two encoder input Pins are reset to analog input,
 * - The timer clock is stopped
 * - If a rotary callback is set, it is called with a delta of 0. This can
 *   be used by the callback as an indicator for deactivation of rotary
 ****************************************************************************/
void QEnc_DeActivate(const HW_DeviceType *self)
{    
    QEncHandleT *myHandle = QEnc_GetAdditionalData(self)->myQEncHandle;  
    
    HAL_TIM_Encoder_Stop_IT(&myHandle->htim, TIM_CHANNEL_1);
 
    myHandle->bActivated  = false;

    /* DeInit GPIO */
    GpioAFDeInitAll(self->devGpioAF);

    /* disable Timer clock */
    HW_SetHWClock(myHandle->htim.Instance, false);

    /* Indicate termination */
    if ( myHandle->OnRotate ) { 
        /* Inhibit multiple calls */
        register QEncEncCB cb = myHandle->OnRotate;
        myHandle->OnRotate = NULL;
        cb(0);
    }
}

void QEnc_DeInit(const HW_DeviceType *self)
{
    QEnc_AdditionalDataType *adt    = QEnc_GetAdditionalData(self);
    QEncHandleT *myHandle           = adt->myQEncHandle;  
    TIM_HandleTypeDef *htim         = &myHandle->htim;

    /* Deactivate handlers */
    myHandle->OnRotate              = NULL;
    myHandle->OnClick               = NULL;
    myHandle->OnDblClick            = NULL;

    /* DeInit GPIO */
    GpioAFDeInitAll(self->devGpioAF);
  
    /* disable interrupts */
    HW_SetAllIRQs(self->devIrqList, false);

    /* Reset Timer registers */
    HAL_TIM_IC_DeInit(htim);

    /* Unregister Timer Capture Callback */
    Tim_UnRegister_CaptureCB(htim->Instance);

    /* disable Timer clock */
    HW_SetHWClock(htim->Instance, false);

    myHandle->bActivated = false;
    myHandle->myIOPin = NULL;

    /* Quadrature encode does not use DMA */
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool QEnc_Init(const HW_DeviceType *self)
{
    bool ret;
    QEnc_AdditionalDataType *adt    = QEnc_GetAdditionalData(self);
    QEncHandleT *myHandle           = adt->myQEncHandle;  
    TIM_HandleTypeDef *htim         = &myHandle->htim;
    htim->Instance                  = adt->myTimer;
    myHandle->cntPerNotch           = adt->myCountsPerNotch;
    myHandle->OnRotate              = NULL;
    myHandle->OnClick               = NULL;
    myHandle->OnDblClick            = NULL;
    // RHB todo: This will allocate a new timer with every init. Has to be fixed !
    myHandle->myTimerId             = MsTimerAllocate(NO_TIMER_ID);
    myHandle->bActivated            = false;
    myHandle->bDblClkItvl           = 0;

    QEncGpioInit(self->devGpioAF, self->devGpioIO);

    HW_SetHWClock(htim->Instance, true);
    ret = QEncTmrInit(htim);
    if ( ret ) {
        /* Configure the NVIC, enable interrupts */
        HW_SetAllIRQs(self->devIrqList, true);
        Tim_Register_CaptureCB(htim->Instance, QEnc_CaptureCallback);
    }

    /* Set Button Press CB */
    myHandle->myIOPin = NULL;
    if ( self->devGpioIO ) {
        /* Only regard the first devGpioIO element! */
        myHandle->myIOPin = self->devGpioIO->gpio;
        Exti_Register_Callback(myHandle->myIOPin->pin, &myHandle->myIOPin->gpio->IDR, QEnc_OnClick, myHandle);
    } 

    /* Leave encoder in deactivated state initially */
    QEnc_DeActivate(self);

    /* If Initialization was unsuccessful, deactivate all */
    if ( !ret ) QEnc_DeInit(self);

    return ret;
}




/******************************************************************************
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, if neither transmit nor receive are active
 *****************************************************************************/
bool QEnc_AllowStop(const HW_DeviceType *self)
{
    return ! QEnc_GetAdditionalData(self)->myQEncHandle->bActivated;  
}


QEncHandleT QEnc1Handle;

/* Two encoder Inputs */
static const HW_GpioList_AF gpio_qenc1 = {
    .num  = 2,
    .gpio = { 
        QENC1TIM_CH1,
        QENC1TIM_CH2,
    }
};

/* One optional PushButton Input */
#if defined(USE_QENC1_PBTN1)
    static const HW_GpioList_IO gpio_qbp1 = {
        .num  = 1,
        .gpio = { 
            QENC1_PBTN1
        }
    };
#endif

static const QEnc_AdditionalDataType additional_qenc1 = {
    .myQEncHandle       = &QEnc1Handle,
    .myTimer            = QTIM1,
    .myCountsPerNotch   =
        #if defined(QENC1_DIVIDER)
            QENC1_DIVIDER,
        #else
            1,
        #endif
};

#if defined( USE_QENC1_IRQ )
const HW_IrqList irq_qenc1 = {
    .num = 1,
    .irq = {QENC1_IRQ },
};
#endif

const HW_DeviceType HW_QENC1 = {
    .devName        = "QENC1",
    .devBase        = QTIM1,
    .devGpioAF      = &gpio_qenc1,
    .devGpioIO      = 
        #if defined(QENC1_PBTN1)
            &gpio_qbp1,
        #else
            NULL,
        #endif
    .devGpioADC     = NULL,
    .devType        = HW_DEVICE_QENC,
    .devData        = &additional_qenc1,
    .devIrqList     = 
        #if defined(USE_QENC1_IRQ) 
            &irq_qenc1,
        #else
            NULL,
        #endif
    .devDmaRx       = NULL,
    .devDmaTx       = NULL,
    .Init           = QEnc_Init,
    .DeInit         = QEnc_DeInit,
    .OnFrqChange    = NULL,
    .AllowStop      = QEnc_AllowStop,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif






/**
  * @}
  */


