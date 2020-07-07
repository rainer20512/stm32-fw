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

#include "hardware.h"

#include "dev/hw_device.h"
#include "config/config.h"
#include "circbuf.h"
#include "config/devices_config.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    uint32_t baudrate;      /* plain baudrate */
    uint32_t stopbits;      /* stopbits as decoded in ..._hal_uart.h */
    uint32_t parity;        /* parity as decoded in ..._hal_uart.h */
    uint32_t wordlength;    /* wordlength as decoded in ..._hal_uart.h */
} UsartCommT;

typedef struct UsartHandleType UsartHandleT; 
typedef void (*UsartCB) ( UsartHandleT * );
typedef void (*UsartRCB) ( UsartHandleT *, uint8_t  );

/* Public typedef ---------------------------------------------------------------*/
typedef struct UsartHandleType {
	USART_TypeDef *Instance;          /* Pointer to HAL USART_Typedef Structure */
        uint32_t baudrate;                /* baudrate to use (initially as configured */
        CircBuffT *out;                   /* Output buffer  */
        LinBuffT *in;                     /* Input buffer  */
        uint8_t  *blockIn;                /* Input buffer for block transfer */
        uint32_t TxCount;                 /* Bytes transmitted since last Tx Start */
        uint32_t TxSize;                  /* Bytes to transmit within actual transmission */
        uint16_t RxCount;                 /* Bytes received so far in block mode */
        uint16_t RxSize;                  /* Bytes to be received in block mode */
        UsartCB OnTxComplete;             /* Callback for transmission (of block ) complete */
        UsartCB OnError;                  /* Callback on any Receive error */
        UsartRCB OnRx;                    /* Callback on any character or block reception */
        uint32_t last_errors;             /* Last error flags, only valid on call of error callback */
        DMA_HandleTypeDef *hRxDma;        /* DMA-Handle for Receive */
        DMA_HandleTypeDef *hTxDma;        /* DMA-Handle for Transmit */
        uint8_t bRxCharMode;              /* indicates character mode on receive channel (default) */
} UsartHandleT; 

/* ------------------------------------------------------------------------------*/
/* Public device variables ------------------------------------------------------*/
/* ------------------------------------------------------------------------------*/
/* UART declarations */
#if defined(USART1) && defined(USE_USART1)
  extern const HW_DeviceType HW_COM1;
  extern UsartHandleT HandleCOM1;
#endif
#if defined(USART2) && defined(USE_USART2)
  extern const HW_DeviceType HW_COM2;
  extern UsartHandleT HandleCOM2;
#endif
#if defined(USART3) && defined(USE_USART3)
  extern const HW_DeviceType HW_COM3;
  extern UsartHandleT HandleCOM3;
#endif
#if defined(UART4) && defined(USE_UART4)
  extern const HW_DeviceType HW_COM4;
  extern UsartHandleT HandleCOM4;
#endif
#if defined(UART5) && defined(USE_UART5)
  extern const HW_DeviceType HW_COM5;
  extern UsartHandleT HandleCOM5;
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
  extern const HW_DeviceType HW_COM9;
  extern UsartHandleT HandleCOM9;
#endif

/* Find out debug usart assignment */
#if   defined(USE_USART1_DEBUG)
    #define HW_DEBUG_UART    HW_COM1
#elif defined(USE_USART2_DEBUG)
    #define HW_DEBUG_UART    HW_COM2
#elif defined(USE_USART3_DEBUG)
    #define HW_DEBUG_UART    HW_COM3
#elif defined(USE_USART4_DEBUG)
    #define HW_DEBUG_UART    HW_COM4
#elif defined(USE_UART5_DEBUG)
    #define HW_DEBUG_UART    HW_COM5
#elif defined(USE_LPUART1_DEBUG)
    #define HW_DEBUG_UART    HW_COM9
#else
    #undef HW_DEBUG_UART
#endif

/* Public functions ---------------------------------------------------------*/

UsartHandleT *USART_GetHandleFromDev(const HW_DeviceType *self);

bool Usart_SetCommParams    (const HW_DeviceType *self, uint32_t baudrate, bool bFirstInit);
bool Usart_SetCommParamsLong(const HW_DeviceType *self, UsartCommT *comm,  bool bFirstInit );

void Usart_AssignBuffers    (UsartHandleT *uhandle, LinBuffT *rxcharbuf, CircBuffT *xmit); 
void UsartAssignCallbacks   (UsartHandleT *uhandle, UsartCB OnTx, UsartCB OnErr, UsartRCB OnRx);
void UsartIRQHandler        (UsartHandleT *uhandle);
bool UsartTxRxOneByteWait   (UsartHandleT *uhandle, uint8_t *txrx, uint32_t tmo);
void UsartStartTx           (UsartHandleT *uhandle, uint8_t *data, uint32_t txSize);
void UsartStartRx           (UsartHandleT *uhandle, uint8_t *data, uint16_t rxSize);
void UsartAbortOp           (UsartHandleT *uhandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __USART_H */
