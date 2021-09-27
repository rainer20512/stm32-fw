/**
 ******************************************************************************
 * @file    usb_dev.c
 * @author  Rainer
 * @brief   USB Devidefunctions. 
 */

/** @addtogroup USB Device Functions
  * @{
  */

#include "config/devices_config.h"

#if USE_USB > 0 

#if DEBUG_MODE > 0
    #define DEBUG_USB               
#endif

#include "error.h"
#include "dev/hw_device.h"
#include "dev/usb_dev.h"

#include "system/hw_util.h"
#include "config/usb_config.h"
#include "debug_helper.h"

#include "usbd_cdc.h"
#include "usbd_desc.h"


/* forward declarations ------------------------------------------------------------*/
bool USB_InitDev(const HW_DeviceType *self);
void USB_DeInitDev(const HW_DeviceType *self);

/**************************************************************************************
 * Some Devices support different clock sources for QSPI. Make sure, that             *   
  * UsbSetClockSource and UsbGetClockSpeed() will match                            *
 *************************************************************************************/
#if defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx)
    /* STM32L4xx has no clock mux for USB device */
    #define UsbSetClockSource(a)           (true)
    #define UsbGetClockSpeed()             HAL_RCC_GetHCLKFreq()
#elif defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    static bool UsbSetClockSource(const void *hw)
    {
      UNUSED(hw);
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* USB will always be clocked by HSI48 */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB; 
      PeriphClkInit.UsbClockSelection   = RCC_USBCLKSOURCE_HSI48;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for USB");
        return false;
      }

      return true;
    }
    //#include "hardware.h"
    //#define  QSpiGetClockSpeed()            GetPerClkFrequency()
    #define QSpiGetClockSpeed()             HAL_RCC_GetHCLKFreq()
#else 
    #error "No qspi clock assignment defined"
#endif

/******************************************************************************
 * Clear the entire UsbHandleT structure
 *****************************************************************************/
static void UsbResetMyHandle ( UsbdHandleT *handle ) 
{
    memset(handle, 0, sizeof(UsbdHandleT) );
}

static void Usbd_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    const HW_Gpio_AF_Type *gpio = self->devGpioAF->gpio;
    uint32_t devIdx     = (uint32_t)GetDevIdx(self);

    /* ID pin as open drain */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GpioAFInitOne(devIdx, &gpio[ID_IDX], &GPIO_InitStruct);

    /* DP and DM as pushpull */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GpioAFInitOne(devIdx, &gpio[DP_IDX], &GPIO_InitStruct);
    GpioAFInitOne(devIdx, &gpio[DM_IDX], &GPIO_InitStruct);

    /* Vbus as simple input */
    GpioIOInitAll(devIdx, self->devGpioIO);
}

static void Usbd_GPIO_DeInit(const HW_DeviceType *self)
{
    uint32_t devIdx     = (uint32_t)GetDevIdx(self);

    /* Disable GPIO Pins */
    GpioAFDeInitAll(devIdx, self->devGpioAF);
    GpioIODeInitAll(devIdx, self->devGpioIO);
}

/*******************************************************************************
 *                      PCD BSP Routines
 ******************************************************************************/

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Low level Initializatio, called from the HAL layer by "HAL_PCD_Init"
 * so procedure signature is fixed here!
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    const HW_DeviceType *self = &HW_USBDFS;

    HW_SetHWClock( hpcd->Instance, true );

    /* Init GPIO and Clocks */
    Usbd_GPIO_Init(self);

    /* Configure the NVIC, enable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Low level DeInit, called from the HAL layer by "HAL_PCD_DeInit"
 * so procedure signature is fixed here!
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    const HW_DeviceType *self = &HW_USBDFS;
    /* disable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* DeInit GPIO */
    Usbd_GPIO_DeInit(self);

  /* Disable USB FS Clock */
    HW_SetHWClock( hpcd->Instance, false );
}


void USBD_CDC_SetCallbacks   ( USBD_CDC_CallbacksT *usb_cbs)
{
    USBDFSHandle.cdc_callbacks = *usb_cbs;
}

void USBD_CDC_StartReceive(void)
{
    USBD_CDC_ReceivePacket(&USBDFSHandle.hUsb);
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, whenever CAN is in SLEEP or INIT Mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool USBD_FrqChange(const HW_DeviceType *self)
{
    UsbdHandleT* me = (UsbdHandleT*)self->devData;
    UNUSED(me);
    return true;
}


/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, whenever CAN is in SLEEP or INIT Mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool USBD_CanStop(const HW_DeviceType *self)
{
    UsbdHandleT* me = (UsbdHandleT*)self->devData;
    UNUSED(me);
    return false;
}

extern USBD_CDC_ItfTypeDef USBD_CDC_fops;
//extern USBD_StorageTypeDef USBD_DISK_fops;

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Can Device initialization: Reset handle, init GPIO pins and interrupts,
 * set default baudrate and normal bus mode, CAN will remain in sleep mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool USBD_InitDev(const HW_DeviceType *self)
{
    UsbdHandleT* me             = (UsbdHandleT*)self->devData;
    // PCD_TypeDef *inst           = (PCD_TypeDef *)self->devBase;
    USBD_HandleTypeDef* hUsbd   = &me->hUsb; 
    
    /* Initialize my handle to 'fresh' */
    UsbResetMyHandle(me);

    UsbSetClockSource(self);

#if defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx)
    /* Get clock status of PWR domain and switch on, if not already on*/
    uint32_t pwrbit = __HAL_RCC_PWR_IS_CLK_ENABLED();
    if ( !pwrbit ) __HAL_RCC_PWR_CLK_ENABLE();
#endif
 
#if defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx)
    /* enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();
#elif defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    HAL_PWREx_EnableUSBVoltageDetector();
#else 
    #error "No suitable USB Initialization for selected MCU"  
#endif

#if defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4Sxxx)
    /* Switch PWR domain clock off again, if it was off before */
    if ( !pwrbit ) __HAL_RCC_PWR_CLK_DISABLE();
#endif


    /* Init Device Library */
    USBD_Init(hUsbd, &VCP_Desc, 0);

    /* Add Supported Class */
    USBD_RegisterClass(hUsbd, USBD_CDC_CLASS);

    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(hUsbd, &USBD_CDC_fops);

    /* Start Device Process */
    USBD_Start(hUsbd);

#if 0 
  /* Init Device Library */
  USBD_Init(hUsbd, &MSC_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(hUsbd, USBD_MSC_CLASS);

  /* Add Storage callbacks for MSC Class */
  USBD_MSC_RegisterStorage(hUsbd, &USBD_DISK_fops);

  /* Start Device Process */
  USBD_Start(hUsbd);
#endif

    return true;
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Reset CAN peripheral, deassign interrupts and DeInit GPIO Pins
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
void USBD_DeInitDev(const HW_DeviceType *self)
{
    /*Reset peripherals */
    HW_Reset((USB_OTG_GlobalTypeDef *)self->devBase );

    Usbd_GPIO_DeInit(self);

    /* Disable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
}

/* Static configurations ---------------------------------------------------------*/

#if defined(USB_HARDWARE) && defined(USE_USB)
    UsbdHandleT USBDFSHandle;

    static const HW_GpioList_AF gpioaf_usb = {
        .gpio = { USB_ID_PIN, USB_DP_PIN, USB_DM_PIN },
        .num = 3, 
    };

    static const HW_GpioList_IO gpioio_usb = {
        .num  = 1,
        .gpio = { USB_VBUS }, 
    };

    static const HW_IrqList irq_usb = {
        .num = 1,
        .irq = { USB_FS_IRQ  },
    };

    const HW_DeviceType HW_USBDFS = {
        .devName        = "USBFS_MSC",
        .devBase        = USB_HARDWARE,
        .devGpioAF      = &gpioaf_usb,
        .devGpioIO      = &gpioio_usb,
        .devType        = HW_DEVICE_USBD,
        .devData        = &USBDFSHandle,
        .devIrqList     = &irq_usb,
        /* No DMA for USBD */
        .devDmaTx = NULL,
        .devDmaRx = NULL,
        .Init           = USBD_InitDev,
        .DeInit         = USBD_DeInitDev,
        .OnFrqChange    = USBD_FrqChange,
        .AllowStop      = USBD_CanStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif



#endif // USE_CAN



