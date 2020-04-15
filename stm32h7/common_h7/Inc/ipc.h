/*
 ******************************************************************************
 * @file    ipc.h 
 * @author  rainer
 *
 * @brief  interprocess communication for FreeRTOS AMP Dual RTOS on STM32H745
 *         
 ******************************************************************************
 */
#ifndef __IPC_H__
#define __IPC_H__

#include "message_buffer.h"
#include "FreeRTOS.h"

#define mbaDONT_BLOCK				0
#define MAX_AMP_CTRL                            16
#define AMP_ID                                  0x4354524C

typedef struct {
    uint32_t ID;
    MessageBufferHandle_t ctrl;
    StaticStreamBuffer_t  ctrl_stream;
    uint32_t num_xfer_used;
    MessageBufferHandle_t xfer       [MAX_AMP_CTRL];
    StaticStreamBuffer_t  xfer_stream[MAX_AMP_CTRL];
} AMPCtrl_t;


/* Used by CM7 */
extern AMPCtrl_t AMPCtrl_Block;


#define ControlMessageBuffer    (AMPCtrl_Block.ctrl)
#define ControlStreamBuffer     (AMPCtrl_Block.ctrl_stream)
#define DataMessageBuffer(i)    (AMPCtrl_Block.xfer[i])
#define DataStreamBuffer(i)     (AMPCtrl_Block.xfer_stream[i])

/* Used by CM4 */
typedef AMPCtrl_t *      AMP_Ctrl_ptr;
extern  AMP_Ctrl_ptr    AMPCtrl_Ptr;

#define ControlMessageBufferRef    (AMPCtrl_Ptr->ctrl)
#define ControlStreamBufferRef     (AMPCtrl_Ptr->ctrl_stream)
#define DataMessageBufferRef(i)    (AMPCtrl_Ptr->xfer[i])
#define DataStreamBufferRef(i)     (AMPCtrl_Ptr->xfer_stream[i])

void WakeUp_CM4  ( void );
void Ipc_Init    ( void );
#endif /* __IPC_H__ */