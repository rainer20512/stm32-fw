/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_conf.c
  * @author  MCD Application Team
  * @brief   This file implements the USB Device library callbacks and MSP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


/* Max time to wait for UsbSema to become available */
#define USB_SEMA_WAITMS         2000

/* Includes ------------------------------------------------------------------ */
#include "config/config.h"
#include "usbd_msc.h"

#if USE_FREERTOS > 0
    #include "FreeRTOS.h"
    #include "semphr.h"
    #include "cmsis_os.h"
    #include "task/minitask.h"
    #include "log.h"
    #include "dev/usb_dev.h"
#endif

#include "log.h"

/* Private typedef ----------------------------------------------------------- */

/********************************************************************************
 * Enumeration for all different Callback types
 *******************************************************************************/
typedef enum {
    CB_SETUP            =0,
    CB_DATAOUT          =1,
    CB_DATAIN           =2,
    CB_SOF              =3,
    CB_RESET            =4,
    CB_SUSPEND          =5,
    CB_RESUME           =6,
    CB_ISOOUTINCOMPLETE =7,
    CB_ISOININCOMPLETE  =8,
    CB_CONNECT          =9,
    CB_DISCONNECT       =10,
} CbEnumT;

/* Private define ------------------------------------------------------------ */

/* Private macro ------------------------------------------------------------- */
#define INLINE          inline __attribute__((always_inline))

/* Private variables --------------------------------------------------------- */
PCD_HandleTypeDef hpcd;

#if USE_FREERTOS > 0
    typedef struct {
        SemaphoreHandle_t CBSem;        /* Semaphore to access this structure      */
        SemaphoreHandle_t USBTaskSem;   /* Semaphore for "USB task is running"     */
        PCD_HandleTypeDef *hpcd;        /* passed PCD handle                       */
        uint8_t epnum;                  /* passed epnum parameter (not always used)*/
        void *arg;                      /* Argument of that function call          */
    } USBD_XferT;
    StaticSemaphore_t xCBSemBuffer;    
    StaticSemaphore_t xUsbTaskSemBuffer;    
    static USBD_XferT    UsbSema = {0};
#endif

/* Private function prototypes ----------------------------------------------- */
/* Private functions --------------------------------------------------------- */

#if USE_FREERTOS > 0
    static void UsbTaskNotify( PCD_HandleTypeDef *hpcd, uint8_t epnum, CbEnumT cbnum )
    {
        LOGT_INFO("Notify USB(%d): %d", epnum, cbnum);
        /* Wait for the UsbSema structure becoming free */
        if ( osSemaphoreWait ( UsbSema.CBSem, USB_SEMA_WAITMS )  == osOK ) {
            UsbSema.hpcd    = hpcd;
            UsbSema.arg     = (void *)cbnum;
            UsbSema.epnum   = epnum;
            TaskNotify(TASK_USB);
/*
            if ( osSemaphoreWait ( UsbSema.USBTaskSem, USB_SEMA_WAITMS )  != osOK ) {
                LOGT_ERROR("UsbTaskNotify: Timeout when waiting for USB-task to complete\n");
            }
*/
        } else {
            LOGT_ERROR("UsbTaskNotify: Timeout when waiting for UsbSema\n");
        }
    }
#endif    

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
  * @brief  SetupStage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void SetupStageCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *) hpcd->Setup);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 2
    UsbTaskNotify(hpcd, 0, CB_SETUP);
  #else
    SetupStageCore(hpcd);
  #endif
}

/**
  * @brief  DataOut Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
static INLINE void DataOutStageCore(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, epnum, CB_DATAOUT);
  #else
    DataOutStageCore(hpcd, epnum);
  #endif
}

/**
  * @brief  DataIn Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
static INLINE void DataInStageCore(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, epnum, CB_DATAIN);
  #else
    DataInStageCore(hpcd, epnum);
  #endif
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void SOFCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_SOF(hpcd->pData);
}


void HAL_PCD_SOFCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, 0, CB_SOF);
  #else
    SOFCore(hpcd);
  #endif
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void ResetCore(PCD_HandleTypeDef * hpcd)
{
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  /* Set USB Current Speed */
  switch (hpcd->Init.speed)
  {
  case PCD_SPEED_HIGH:
    speed = USBD_SPEED_HIGH;
    break;

  case PCD_SPEED_FULL:
    speed = USBD_SPEED_FULL;
    break;

  default:
    speed = USBD_SPEED_FULL;
    break;
  }

  /* Reset Device */
  USBD_LL_Reset(hpcd->pData);

  USBD_LL_SetSpeed(hpcd->pData, speed);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 2
    UsbTaskNotify(hpcd, 0, CB_RESET);
  #else
    ResetCore(hpcd);
  #endif
}

/**
  * @brief  Suspend callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void SuspendCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_Suspend(hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 2
    UsbTaskNotify(hpcd, 0, CB_SUSPEND);
  #else
    SuspendCore(hpcd);
  #endif
}

/**
  * @brief  Resume callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void ResumeCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_Resume(hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, 0, CB_RESUME);
  #else
    ResumeCore(hpcd);
  #endif
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
static INLINE void ISOOUTIncompleteCore(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}


void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, epnum, CB_ISOOUTINCOMPLETE);
  #else
    ISOOUTIncompleteCore(hpcd, epnum);
  #endif
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
static INLINE void ISOINIncompleteCore(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, epnum, CB_ISOININCOMPLETE);
  #else
    ISOINIncompleteCore(hpcd, epnum);
  #endif
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void ConnectCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, 0, CB_CONNECT);
  #else
    ConnectCore(hpcd);
  #endif
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
static INLINE void DisconnectCore(PCD_HandleTypeDef * hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef * hpcd)
{
  #if USE_FREERTOS > 0
    UsbTaskNotify(hpcd, 0, CB_DISCONNECT);
  #else
    DisconnectCore(hpcd);
  #endif
}


/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef * pdev)
{


#if USE_USB_FS > 0
  /* Set LL Driver parameters */
  hpcd.Instance = USB2_OTG_FS;
  hpcd.Init.dev_endpoints = 8;
  hpcd.Init.use_dedicated_ep1 = 0;
  hpcd.Init.low_power_enable = 0;
  hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd.Init.Sof_enable = 0;
  hpcd.Init.speed = PCD_SPEED_FULL;
  hpcd.Init.vbus_sensing_enable = 0;
  hpcd.Init.lpm_enable = 0;

  /* Link The driver to the stack */
  hpcd.pData = pdev;
  pdev->pData = &hpcd;
  /* Initialize LL Driver */
  HAL_PCD_Init(&hpcd);

  HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x80);

#endif

#ifdef USE_USB_HS
  /* Set LL Driver parameters */
  hpcd.Instance = USB1_OTG_HS;
  hpcd.Init.dev_endpoints = 8;
  hpcd.Init.use_dedicated_ep1 = 0;

  /* Be aware that enabling DMA mode will result in data being sent only by
   * multiple of 4 packet sizes. This is due to the fact that USB DMA does not
   * allow sending data from non word-aligned addresses. For this specific
   * application, it is advised to not enable this option unless required. */
  hpcd.Init.dma_enable = 0;
  hpcd.Init.low_power_enable = 0;
  hpcd.Init.lpm_enable = 0;
  hpcd.Init.phy_itface = PCD_PHY_ULPI;
  hpcd.Init.Sof_enable = 0;
  hpcd.Init.speed = PCD_SPEED_HIGH;
  hpcd.Init.vbus_sensing_enable = 0;

  /* Link The driver to the stack */
  hpcd.pData = pdev;
  pdev->pData = &hpcd;

  /* RHB Added with H7FW V1.9 */
  if (hpcd.Init.dma_enable == 1U)
  {
    SCB_DisableDCache();
  }

  /* Initialize LL Driver */
  HAL_PCD_Init(&hpcd);

  HAL_PCDEx_SetRxFiFo(&hpcd, 0x200);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x100);
#endif

  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef * pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef * pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef * pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef * pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

  return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef * pdev,
                                        uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if ((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef * pdev,
                                         uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef * pdev,
                                    uint8_t ep_addr,
                                    uint8_t * pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef * pdev,
                                          uint8_t ep_addr,
                                          uint8_t * pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Returns the last transferred packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Received Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  Static single allocation.
  * @param  size: Size of allocated memory
  * @retval None
  */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_MSC_BOT_HandleTypeDef)/4)+1] AXISDMAMEM;/* On 32-bit boundary */
  UNUSED(size);
  return mem;
}

/**
  * @brief  Dummy memory free
  * @param  p: Pointer to allocated  memory address
  * @retval None
  */
void USBD_static_free(void *p)
{
    UNUSED(p);
}

#if USE_FREERTOS > 0

/******************************************************************************
 * - Create an blocked semaphore for callback functions 
 *   and release that lock immeditaley 
 * - Create an blocked Semaphore for "usb task running". This semaphore
 *   will be unlocked when the USB task has responded to any USB request
 * - Start the USB device
 *****************************************************************************/
void task_init_usb(void)
{
    UsbSema.CBSem = xSemaphoreCreateBinaryStatic( &xCBSemBuffer );
    osSemaphoreRelease(UsbSema.CBSem);

    UsbSema.USBTaskSem = xSemaphoreCreateBinaryStatic( &xUsbTaskSemBuffer );

    /* Start the USB device */
    USBD_Start(&USBDFSHandle.hUsb);
    LOGT_INFO("Init USB");

}

/******************************************************************************
 * USB main task: act upon USB messages 
 *****************************************************************************/
void task_handle_usb(uint32_t arg)
{
    UNUSED(arg);

    PCD_HandleTypeDef *hpcd = UsbSema.hpcd;
    LOGT_INFO("Handle USB: %d", (CbEnumT)UsbSema.arg);
    switch ( (CbEnumT)UsbSema.arg ) { 
        case CB_SETUP:
            SetupStageCore(UsbSema.hpcd);
            break;
        case CB_DATAOUT:
            DataOutStageCore(hpcd,UsbSema.epnum);
            break;
        case CB_DATAIN:
            DataInStageCore(hpcd,UsbSema.epnum);
            break;
        case CB_SOF:
            SOFCore(UsbSema.hpcd);
            break;
        case CB_RESET:
            ResetCore(UsbSema.hpcd);
            break;
        case CB_SUSPEND:
            SuspendCore(UsbSema.hpcd);
            break;
        case CB_RESUME:
            ResumeCore(UsbSema.hpcd);
            break;
        case CB_ISOOUTINCOMPLETE:
            ISOOUTIncompleteCore(hpcd,UsbSema.epnum);
            break;
        case CB_ISOININCOMPLETE:
            ISOINIncompleteCore(hpcd,UsbSema.epnum);
            break;
        case CB_CONNECT:
            ConnectCore(hpcd);
            break;
        case CB_DISCONNECT:
            DisconnectCore(hpcd);
            break;
        default:
            LOG_ERROR("task_handle_usb: Undefined callback #%d\n",(CbEnumT)UsbSema.arg);
    } // switch

    /* Release the UsbSema - lock */
    osSemaphoreRelease(UsbSema.CBSem);
    /* Release the USB-Task semaphore */
    // osSemaphoreRelease(UsbSema.USBTaskSem);

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
