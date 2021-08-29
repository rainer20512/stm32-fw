/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "config/config.h"

#if USE_USB > 0


/* Includes ------------------------------------------------------------------*/

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h" 
#include "usbd_cdc_interface.h"
#include "dev/usb_dev.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/*************************************************************************************************************
 * USB device class library expects two array as receive and transmit buffer. According to the doc, the
 * minimal size should be 2kByte each
 ************************************************************************************************************/
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

uint32_t BuffLength;
uint32_t UserTxBufPtrIn  = 0;  /* Increment this pointer or roll it back to
                                  start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0;  /* Increment this pointer or roll it back to
                                  start address when data are sent over USB */

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;

#define MyDevice        ( &USBDHandle.hUsb )

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

static void LineCoding_To_UartParms(void);
static void UartParms_To_LineCoding(void);

/* Exported / global variables ----------------------------------------------*/
USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(MyDevice, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(MyDevice, UserRxBuffer);
  
  /* Call callback, if set */
  if ( USBDHandle.cdc_callbacks.USBD_CDC_OnInit ) USBDHandle.cdc_callbacks.USBD_CDC_OnInit();

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  /* Call callback, if set */
  if ( USBDHandle.cdc_callbacks.USBD_CDC_OnDeInit ) USBDHandle.cdc_callbacks.USBD_CDC_OnDeInit();

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  UNUSED(length);
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    
    /* Save the comm settings internally */
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    
    /* Decode and call callback, if set */
    if ( USBDHandle.cdc_callbacks.USBD_CDC_OnChngCommParams ) {
        /* Set the new configuration */
        LineCoding_To_UartParms();
    }
    break;

  case CDC_GET_LINE_CODING:
    /* if Query-Callback is set then get comm params and copy to LineCoding */
    if ( USBDHandle.cdc_callbacks.USBD_CDC_QueryCommParams ) {
        /* Set the new configuration */
        UartParms_To_LineCoding();
    }

    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;     
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }
  
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* buf, uint32_t *len)
{
  #if 0  
  HAL_UART_Transmit_DMA(&UartHandle, Buf, *Len);
  #endif
  /* Call callback, if set */
  if ( USBDHandle.cdc_callbacks.USBD_CDC_OnReceive ) USBDHandle.cdc_callbacks.USBD_CDC_OnReceive( buf, *len);
  
  return (USBD_OK);
}

/* Get the internal transmit buffer an trigger a transmit of internal write buffer */
void USBD_CDC_GetTtansmitBuffer ( uint8_t **buf, uint32_t *maxlen )
{
    *buf    = UserTxBuffer;
    *maxlen = APP_TX_DATA_SIZE;
}

void USBD_CDC_Transmit ( uint32_t actlen )
{
    USBD_CDC_SetTxBuffer(MyDevice, UserTxBuffer, actlen);
    USBD_CDC_TransmitPacket(MyDevice);
}

/* Transmit a buffer by copy to internal transmit buffer */
void USBD_CDC_CopyTxBuffer ( uint8_t  *buf, uint32_t txlen )
{
    /* check for tx buffer overflow */
    #if DEBUG_MODE > 0
        if ( txlen > APP_TX_DATA_SIZE ) USBD_ErrLog("CopyTxBuffer: Buffer overflow, truncated");
    #endif
    if ( txlen > APP_TX_DATA_SIZE ) txlen = APP_TX_DATA_SIZE;

    memcpy(UserTxBuffer, buf, txlen );
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
static void LineCoding_To_UartParms(void)
{
  uint32_t baudrate, stopbits, parity, wordlength;

  baudrate = LineCoding.bitrate;

  /* set the Stop bit */
  switch (LineCoding.format)
  {
  case 0:
    stopbits = UART_STOPBITS_1;
    break;
  case 2:
    stopbits = UART_STOPBITS_2;
    break;
  default :
    stopbits = UART_STOPBITS_1;
    break;
  }
  
  /* set the parity bit*/
  switch (LineCoding.paritytype)
  {
  case 0:
    parity = UART_PARITY_NONE;
    break;
  case 1:
    parity = UART_PARITY_ODD;
    break;
  case 2:
    parity = UART_PARITY_EVEN;
    break;
  default :
    parity = UART_PARITY_NONE;
    break;
  }
  
  /*set the data type : only 8bits and 9bits is supported */
  switch (LineCoding.datatype)
  {
  case 0x07:
    /* With this configuration a parity (Even or Odd) must be set */
    wordlength = UART_WORDLENGTH_8B;
    break;
  case 0x08:
    if(parity == UART_PARITY_NONE) {
      wordlength = UART_WORDLENGTH_8B;
    } else {
      wordlength = UART_WORDLENGTH_9B;
    }
    break;
  default :
    wordlength = UART_WORDLENGTH_8B;
    break;
  }

  /* The presence of setter callback has been checked before, so just call here w/o check */
  USBDHandle.cdc_callbacks.USBD_CDC_OnChngCommParams(baudrate, stopbits, parity, wordlength);  
}


static void UartParms_To_LineCoding(void)
{
  uint32_t baudrate, stopbits, parity, wordlength;
  uint32_t temp;
  /* The presence of query callback has been checked before, so just call here w/o check */
  USBDHandle.cdc_callbacks.USBD_CDC_QueryCommParams(&baudrate, &stopbits, &parity, &wordlength);  

  LineCoding.bitrate    = baudrate;
  LineCoding.format     = ( stopbits == UART_STOPBITS_2      ? 2 : 0 );
  LineCoding.paritytype = ( parity ==  UART_PARITY_EVEN      ? 2 : ( parity ==  UART_PARITY_ODD ? 1 : 0 ) );
  temp                  = ( wordlength == UART_WORDLENGTH_9B ? 9 : 8 );
  LineCoding.datatype   = ( parity == UART_PARITY_NONE       ? temp : temp - 1 );
}    

/***********************************************************************************
 * Handcrafted malloc ( to avoid the "real" malloc of STM USB class implementation
 ***********************************************************************************/
void *USBD_StaticMalloc( size_t size )
{
    /* Will be used by the STM implemetation and will be allocated by USBD_malloc  */ 
    static USBD_CDC_HandleTypeDef usbdcdcHandle;     

    /* Just ensure, we have enough memory */
    assert( size <= sizeof(USBD_CDC_HandleTypeDef) );

    return ( void *)&usbdcdcHandle;
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



#endif // #if USE_USB > 0

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

