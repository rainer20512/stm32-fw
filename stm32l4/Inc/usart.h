/**
  ******************************************************************************
  * @file    usart.h
  * @author  Rainer 
  * @brief   Initialize on of the STM32L4's U(S)ARTs as debug output, assign
  *          input and output buffers and start receive and transmit
  *
  * @note    Receive is done on a character by character basis by Interrupts
  *          Transmit is done blockwise by DMA
  *          The U(S)ARTs Receive line is also used as edge triggered EXTI
  *          interrupt to wakeup from Stop.
  *
  * @note    This is a lean and mean alternative to HAL USART stuff    
  * 
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#include "stm32l4xx.h"

#include "dev/hw_device.h"
#include "config.h"
#include "circbuf.h"
#include "devices_config.h"

#ifdef __cplusplus
 extern "C" {
#endif
typedef struct UsartHandleType UsartHandleT; 
typedef void (*UsartCB) ( UsartHandleT * );
typedef void (*UsartRCB) ( UsartHandleT *, uint8_t  );

/* Public typedef ---------------------------------------------------------------*/
typedef struct UsartHandleType {
	USART_TypeDef *Instance;          /* Pointer to HAL USART_Typedef Structure */
        CircBuffT *out;                   /* Output buffer  */
        LinBuffT *in;                     /* Input buffer  */
        uint32_t TxCount;                 /* Bytes transmitted since last Tx Start */
        uint32_t TxSize;                  /* Bytes to transmit within actual transmission */
        UsartCB OnTxComplete;             /* Callback for transmission (of block ) complete */
        UsartCB OnError;                  /* Callback on any Receive error */
        UsartRCB OnRxChar;                /* Callback on any character reception */
        uint32_t last_errors;             /* Last error flags, only valid on call of error callback */
        DMA_HandleTypeDef *hRxDma;        /* DMA-Handle for Receive */
        DMA_HandleTypeDef *hTxDma;        /* DMA-Handle for Transmit */
} UsartHandleT; 

/* ------------------------------------------------------------------------------*/
/* Public device variables ------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/
/* UART declarations */
#if defined(USART1) && defined(USE_USART1)
  extern HW_DeviceType HW_Com1;
  extern UsartHandleT HandleCOM1;
#endif
#if defined(USART2) && defined(USE_USART2)
  extern HW_DeviceType HW_Com2;
  extern UsartHandleT HandleCOM2;
#endif
#if defined(USART3) && defined(USE_USART3)
  extern HW_DeviceType HW_Com3;
  extern UsartHandleT HandleCOM3;
#endif
#if defined(UART4) && defined(USE_UART4)
  extern HW_DeviceType HW_Com4;
  extern UsartHandleT HandleCOM4;
#endif
#if defined(UART5) && defined(USE_UART5)
  extern HW_DeviceType HW_Com5;
  extern UsartHandleT HandleCOM5;
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
  extern HW_DeviceType HW_Com6;
  extern UsartHandleT HandleCOM6;
#endif


/* Public functions ---------------------------------------------------------*/
void UsartInitDev ( void );
void UsartClockInit(void);
void UartAssignCircBuff ( UsartHandleT *uhandle, LinBuffT *recv, CircBuffT *xmit); 
void UsartAssignCallbacks(UsartHandleT *uhandle, UsartCB OnTx, UsartCB OnErr, UsartRCB OnRx);
void UsartIRQHandler(UsartHandleT *uhandle);
void UsartStartTx(UsartHandleT *uhandle, uint8_t *data, uint32_t txSize);
void UsartStartTxDMA(UsartHandleT *uhandle, uint32_t txSize);
void UsartStopTx(UsartHandleT *uhandle);
void UsartStartRx(UsartHandleT *uhandle);
void UsartStopRx(UsartHandleT *uhandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __USART_H */
