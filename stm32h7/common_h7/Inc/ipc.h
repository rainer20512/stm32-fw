/*
 ******************************************************************************
 * @file    ipc.h 
 * @author  rainer
 *
 * @brief  interprocess communication for FreeRTOS AMP Dual RTOS on STM32H745
 *         
 * @note   Any changes in AMP control block requires recompile of both parts!
 * 
 ******************************************************************************
 */
#ifndef __IPC_H__
#define __IPC_H__

#include "msg_direct.h"
#include "message_buffer.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define mbaDONT_BLOCK				0

/* Initial transfer on Control block and direct message buffer */
#define CTRL_HOOK_ENABLE_ACCESS()      __HAL_RCC_RTC_CLK_ENABLE()
#define CTRL_HOOK_DISABLE_ACCESS()      __HAL_RCC_RTC_CLK_DISABLE()
#define CTRL_BLOCK_HOOK_GET()           ((AMPCtrl_t* )RTC->BKP0R)
#define MSGBUF_HOOK_GET()               ((AMPDctBuf_t* )RTC->BKP1R)
#define CTRL_BLOCK_HOOK_PUT(ctrl)       RTC->BKP0R = (uint32_t)(&ctrl)
#define MSGBUF_HOOK_PUT(msgbuf)         RTC->BKP1R = (uint32_t)(&msgbuf)



/* Number of different message buffer paths between CM7 and CM4 */
#define MAX_AMP_CTRL                            8
/* Magic number to indicate a valid AMP_Ctrl buffer             */
#define AMP_ID                                  0x4354524C

typedef struct {
    uint32_t ID;
    MessageBufferHandle_t ctrl_cm7;
    StaticStreamBuffer_t  ctrl_cm7_stream;
    SemaphoreHandle_t     ctrl_cm7_sem;
    StaticSemaphore_t     ctrl_cm7_sem_buf;
    MessageBufferHandle_t ctrl_cm4;
    StaticStreamBuffer_t  ctrl_cm4_stream;
    SemaphoreHandle_t     ctrl_cm4_sem;
    StaticSemaphore_t     ctrl_cm4_sem_buf;

    uint32_t num_xfer_used;
    MessageBufferHandle_t xfer       [MAX_AMP_CTRL];
    StaticStreamBuffer_t  xfer_stream[MAX_AMP_CTRL];
    SemaphoreHandle_t     xfersem    [MAX_AMP_CTRL];
    StaticSemaphore_t     xfersem_buf[MAX_AMP_CTRL];
} AMPCtrl_t;


/* Used by CM7 */
extern AMPCtrl_t AMPCtrl_Block;


/* CM7 to CM4 */
#define Control74MessageBuffer    (AMPCtrl_Block.ctrl_cm7)
#define Control74StreamBuffer     (AMPCtrl_Block.ctrl_cm7_stream)
#define Control74Sem              (AMPCtrl_Block.ctrl_cm7_sem)
/* CM4 to CM7  */
#define Control47MessageBuffer    (AMPCtrl_Block.ctrl_cm4)
#define Control47StreamBuffer     (AMPCtrl_Block.ctrl_cm4_stream)
#define Control47Sem              (AMPCtrl_Block.ctrl_cm4_sem)

/* Data Buffers and associated semaphores */
#define DataMessageBuffer(i)      (AMPCtrl_Block.xfer[i])
#define DataStreamBuffer(i)       (AMPCtrl_Block.xfer_stream[i])
#define DataMessageSem(i)         (AMPCtrl_Block.xfersem[i]) 

/* Used by CM4 */
typedef AMPCtrl_t *         AMP_Ctrl_ptr;
extern  AMP_Ctrl_ptr        AMPCtrl_Ptr;

/* CM7 to CM4 */

#define Control74MessageBufferRef    (AMPCtrl_Ptr->ctrl_cm7)
#define Control74StreamBufferRef     (AMPCtrl_Ptr->ctrl_cm7_stream)
#define Control74SemRef              (AMPCtrl_Ptr->ctrl_cm4_sem)

/* CM4 to CM7 */
#define Control47MessageBufferRef    (AMPCtrl_Ptr->ctrl_cm4)
#define Control47StreamBufferRef     (AMPCtrl_Ptr->ctrl_cm4_stream)
#define Control47SemRef              (AMPCtrl_Ptr->ctrl_cm4_sem)
#define DataMessageBufferRef(i)      (AMPCtrl_Ptr->xfer[i])
#define DataStreamBufferRef(i)       (AMPCtrl_Ptr->xfer_stream[i])
#define DataMessageSemRef(i)         (AMPCtrl_Ptr->xfersem[i]) 

#if defined(CORE_CM7)
    void     Ipc_CM7_SendDirect     ( void );
    void     Ipc_CM7_WakeUp_CM4     ( void );
    void     Ipc_CM7_Init           ( void );
#endif

#if defined(CORE_CM4)
    #define INIT_RESTRICTED 1
    #define INIT_FULLY      0
    bool Ipc_CM4_Init ( uint32_t  bRestricted );   
    void Ipc_CM4_SendDirect     ( void );
#endif


#endif /* __IPC_H__ */