/*
 ******************************************************************************
 * @file    ipc.c 
 * @author  rainer
 *
 * @brief  interprocess communication for FreeRTOS AMP Dual RTOS on STM32H745
 *         
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "config/config.h"
#include "hardware.h"
#include "task/minitask.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "ipc.h"
#include "msg_direct.h"

#if defined(CORE_CM4)
    #include "MessageBufferAMP.h"
#endif

#include "debug_helper.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define HSEM_CM7_to_CM4_Send        (0U)            /* HW semaphore 0 - signal incoming msg from CM7 to CM4  */
#define HSEM_CM7_to_CM4_Recvd       (1U)            /* HW semaphore 1 - signal msg reception from CM7 to CM4 */
#define HSEM_CM7_to_CM4_Wkup        (2U)            /* HW semaphore 2 - used to wake up CM4 */
#define HSEM_CM4_to_CM7_Send        (3U)            /* HW semaphore 3 - signal incoming msg from CM4 to CM7  */
#define HSEM_CM4_to_CM7_Recvd       (4U)            /* HW semaphore 4 - signal msg reception from CM4 to CM7 */

#define HSEM_CM7_to_CM4_Msg         (5U)            /* HW sem. 5 - direct CM7 to CM4 messaging ( w/o RTOS )  */
#define HSEM_CM4_to_CM7_Msg         (6U)            /* HW sem. 6 - direct CM4 to CM7 messaging ( w/o RTOS )  */
 
/* The static message buffer size for the ctrl buffer
overhead of message buffers. */
#define CONTROL_MESSAGE_BUFFER_SIZE         24



/* Private functions ---------------------------------------------------------*/
#if defined(CORE_CM7)

    /* The control block is allocated by CM7 and referenced by CM4 */
    IPCMEM AMPCtrl_t AMPCtrl_Block={0};
    static IPCMEM uint8_t StorageBuffer_ctrl74 [ CONTROL_MESSAGE_BUFFER_SIZE ] ;
    static IPCMEM uint8_t StorageBuffer_ctrl47 [ CONTROL_MESSAGE_BUFFER_SIZE ] ;
    IPCMEM AMPDctBuf_t AMP_DirectBuffer;

#endif


/******************************************************************************
 * inter-CPU signaling by taking and releasing a semaphore, this will raise an
 * interrupt on the other CPU
 *****************************************************************************/
static void IPC_Signal( uint32_t HW_sem )
{
    HAL_HSEM_FastTake(HW_sem);
    HAL_HSEM_Release(HW_sem,0);
}

/* Used to dimension the array used to hold the streams.*/
/* Defines the memory that will actually hold the streams within the stream buffer.*/
#if defined(CORE_CM7)
    /**************************************************************************
     * Initialize the IPC buffer and allocate the Control message buffer 
     * This is done by CM7 core
     *************************************************************************/
    void Ipc_CM7_Init ( void )
    {
        /*HW semaphore Clock enable*/
        __HAL_RCC_HSEM_CLK_ENABLE();

        /* Enable HSEM interrupts */
        HAL_NVIC_SetPriority( HSEM1_IRQn, IPC_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ( HSEM1_IRQn);

        // Is done by initialization code in thumb_crt0_cm7.s 
        // memset(&AMPCtrl_Block, 0, sizeof(AMPCtrl_t));
        AMPCtrl_Block.ID = AMP_ID;
        DEBUG_PRINTF("IPC control block of size %d initialized\n", sizeof(AMPCtrl_t) );

        /* Create and initialize direct message buffer */
        memset(&AMP_DirectBuffer, 0, sizeof(AMPDctBuf_t));
        AMP_DirectBuffer.id = DIRECTMSG_ID;
        DEBUG_PRINTF("IPC direct msg buffer of size %d initialized\n", sizeof(AMPDctBuf_t) );


        /* Create control message buffer and buffer semaphore for CM7 to CM4 communication */
        Control74MessageBuffer = xMessageBufferCreateStatic( CONTROL_MESSAGE_BUFFER_SIZE,StorageBuffer_ctrl74 ,&Control74StreamBuffer);  
        Control74Sem           = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.ctrl_cm7_sem_buf );
        xSemaphoreGive(Control74Sem);
        
        /* Create control message buffer and buffer semaphore for CM4 to CM7 communication*/
        Control47MessageBuffer = xMessageBufferCreateStatic( CONTROL_MESSAGE_BUFFER_SIZE,StorageBuffer_ctrl47 ,&Control47StreamBuffer);  
        Control47Sem           = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.ctrl_cm4_sem_buf );
        xSemaphoreGive(Control47Sem);

        /* create all data buffer semaphores */
        for ( uint32_t i = 0; i < MAX_AMP_CTRL; i++ ) {
            DataMessageSem(i) = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.xfersem_buf[i] );
            xSemaphoreGive(DataMessageSem(i));

        }

        /* Activate notification on all messages from CM4 */
        HSEM->C1IER |= (1 << HSEM_CM4_to_CM7_Send) | (1 << HSEM_CM4_to_CM7_Recvd ) | ( 1 << HSEM_CM4_to_CM7_Msg ) ;
    }

    /**************************************************************************
     * Returns true, if the IPC buffer has been initialized by CMz
     *************************************************************************/
    uint32_t Ipc_Is_Initialized     ( void )
    {
        return AMPCtrl_Block.ID == AMP_ID;
    }

    /**************************************************************************
     * CM4 goes to sleep after reset and waits for Semaphore HSEM_CM4_WKUP 
     * This CM7 routine is used to start CM4 again
     *************************************************************************/
    void Ipc_CM7_WakeUp_CM4 ( void )
    {
        /* Cortex-M7 will release Cortex-M4  by means of HSEM notification */
        IPC_Signal(HSEM_CM7_to_CM4_Wkup);
    }


    /**************************************************************************
     * Reimplementation of sbSEND_COMPLETED(), defined as follows in FreeRTOSConfig.h:
     * #define sbSEND_COMPLETED( pxStreamBuffer ) vCore1SignalControlBufToSend( pxStreamBuffer )
     * This is the implementation on the CM7 core 
     *
     * Called from within xMessageBufferSend().  As this function also calls
     * xMessageBufferSend() itself it is necessary to guard against a recursive
     * call.  If the message buffer just updated is the message buffer written to
     * by this function, then this is a recursive call, and the function can just
     * exit without taking further action.
     *************************************************************************/
    void vCore1SignalControlBufToSend( void * xUpdatedMessageBuffer )
    {
        MessageBufferHandle_t xUpdatedBuffer = ( MessageBufferHandle_t ) xUpdatedMessageBuffer;

        if( xUpdatedBuffer != Control74MessageBuffer ) {
            /* Use xControlMessageBuffer to pass the handle of the message buffer
            written to by core 1 to the interrupt handler about to be generated in
            core 2. */
            if ( xSemaphoreTake(Control74Sem, pdMS_TO_TICKS(10000)) == pdTRUE ) {
                xMessageBufferSend( Control74MessageBuffer, &xUpdatedBuffer, sizeof( xUpdatedBuffer ), mbaDONT_BLOCK );
                /* Signal core2 */
                IPC_Signal(HSEM_CM7_to_CM4_Send);
            } else {
                DEBUG_PUTS("Cannot allocate ctrl74");
            }
        }
    }

    /**************************************************************************
     * Interrupt hander for the software interrupt from core2
     * A message buffer for core1 has been delivered from core2
     * The message buffer is passed within the control buffer.
     * So unpack the control buffer and deliver the message buffer to
     * the corresponding core1 task
     * Note: Routine is called in ISR context only!
     *************************************************************************/
    static void prvCore1ReceiveHandler( void )
    {
      MessageBufferHandle_t xUpdatedMessageBuffer;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
      /* Control47MessageBuffer contains the handle of the message buffer that
      contains data. Ensure to drain the buffer before returning. */
      while ( xMessageBufferReceiveFromISR( Control47MessageBuffer,
                                       &xUpdatedMessageBuffer,
                                       sizeof( xUpdatedMessageBuffer ),
                                       &xHigherPriorityTaskWoken ) == sizeof( xUpdatedMessageBuffer ) )
      {
        /* Call the API function that sends a notification to any task that is
        blocked on the xUpdatedMessageBuffer message buffer waiting for data to
        arrive. */
        xMessageBufferSendCompletedFromISR( xUpdatedMessageBuffer, &xHigherPriorityTaskWoken );
      }
  
      /* Signal reception to core2 */
      IPC_Signal(HSEM_CM7_to_CM4_Recvd);

      /* Normal FreeRTOS yield from interrupt semantics, where
      xHigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
      pdTRUE if the interrupt safe API unblocks a task that has a priority above
      that of the currently executing task. */
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * A control buffer reception has been acknowledged by CM4
     * So release the control buffer semaphore
     * Note: Routine is called in ISR context only!
     *************************************************************************/
    static void prvCore1SendAckHandler( void )
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(Control74Sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * Send a direct message ( w/o RTOS ) to CM4 core
     * Message must have been setup before in AMP_DirectBuffer
     *************************************************************************/
    void Ipc_CM7_SendDirect( void )
    {
        IPC_Signal(HSEM_CM7_to_CM4_Msg);
    }


    /**************************************************************************
     * Semaphore free handler for CM7
     * It has to react on incoming messages from CM4
     * and on receive acknowledge from CM4 as reaction on CM7 data sent
     *************************************************************************/
    void HSEM1_IRQHandler ( void )
    {
        /* Get activated Interrupt bits */
        uint32_t SemMask = HSEM->C1ISR & HSEM->C1IER;
        
        /*Clear all activated bits */
        HSEM->C1ICR = SemMask;

        /* Handle all "free" interrupts */
        if ( SemMask & (1 << HSEM_CM4_to_CM7_Send ) ) prvCore1ReceiveHandler();
        if ( SemMask & (1 << HSEM_CM4_to_CM7_Recvd) ) prvCore1SendAckHandler();
        if ( SemMask & (1 << HSEM_CM4_to_CM7_Msg  ) ) CM7_handle_remote_direct();

    }

#endif /* CORE_CM7 */

#if defined(CORE_CM4)
    
    #include "debug_helper.h"

    /* Ptr to the AMP control block, allocated by CM/ core and passed as reference to CM4 */
    AMP_Ctrl_ptr AMPCtrl_Ptr;

    /**************************************************************************
     * Check IPC block for proper initialization by CM7 core
     *************************************************************************/
    void Ipc_CM4_Check(void)
    {
        bool bNullPtr;

        if ( !AMPCtrl_Ptr ) {
            DEBUG_PUTS("AMP Control Block not assigned");
        }

        
        if ( AMPCtrl_Ptr->ID != AMP_ID ) {
            DEBUG_PUTS("AMP Control Block invalid");
        }

        if ( AMP_DctBuf_Ptr->id != DIRECTMSG_ID ) {
            DEBUG_PUTS("AMP direct message buffer invalid");
        }

        
        bNullPtr = Control74MessageBufferRef == NULL;
        for ( uint32_t i = 0; i < AMPCtrl_Ptr->num_xfer_used; i++ ) {
            bNullPtr = bNullPtr || DataMessageBufferRef(i) == NULL || DataMessageSemRef(i) == NULL;
        }
        if ( bNullPtr ) 
            DEBUG_PUTS("One or more IPC message buffers could not be allocated");

    }

    /**************************************************************************
     * Initialize the IPC buffer and allocate the Control message buffer 
     * This is done by CM7 core
     *************************************************************************/
    void Ipc_CM4_Init ( uint32_t uRestricted )
    {
        /*HW semaphore Clock enable*/
        __HAL_RCC_HSEM_CLK_ENABLE();

        HAL_NVIC_SetPriority( HSEM2_IRQn, IPC_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ( HSEM2_IRQn);

        /* in first phase, only allow wakeup from CM7 */
        if ( uRestricted==INIT_RESTRICTED) {
            /* Activate notification on only Wakepu */
            HSEM->C2IER = 1 << HSEM_CM7_to_CM4_Wkup;
            return;
        }

        /* 
         * Normal Init: Get the adress of the control message buffer. It has been written 
         * to RCT Bkup ram offset 0 by CM7 before waking up this core
         */
        CTRL_HOOK_ENABLE_ACCESS();
        AMPCtrl_Ptr     = CTRL_BLOCK_HOOK_GET();
        AMP_DctBuf_Ptr  = MSGBUF_HOOK_GET(); 
        CTRL_HOOK_DISABLE_ACCESS();

        /* Check consitency of AMP control block */
        Ipc_CM4_Check();

        /* finally activate notification on all messages from CM4 */
        HSEM->C2IER |= ( (1 << HSEM_CM7_to_CM4_Wkup) | (1 << HSEM_CM7_to_CM4_Send) | ( 1 << HSEM_CM7_to_CM4_Recvd) | (1 << HSEM_CM7_to_CM4_Msg) );
    }


    /**************************************************************************
     * Interrupt hander for the software interrupt from core1
     * A message buffer for core2 has been delivered from core1
     * The message buffer is passed within the control buffer.
     * So unpack the control buffer and deliver the message buffer to
     * the corresponding core2 task
     *************************************************************************/
    static void prvCore2ReceiveHandler( void )
    {
      MessageBufferHandle_t xUpdatedMessageBuffer;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
      /* xControlMessageBuffer contains the handle of the message buffer that
      contains data. Ensure to drain the buffer before returning. */
      while ( xMessageBufferReceiveFromISR( Control74MessageBufferRef,
                                       &xUpdatedMessageBuffer,
                                       sizeof( xUpdatedMessageBuffer ),
                                       &xHigherPriorityTaskWoken ) == sizeof( xUpdatedMessageBuffer ) )
      {
        /* Call the API function that sends a notification to any task that is
        blocked on the xUpdatedMessageBuffer message buffer waiting for data to
        arrive. */
        xMessageBufferSendCompletedFromISR( xUpdatedMessageBuffer, &xHigherPriorityTaskWoken );
      }
  
      /* Signal reception to core1  */
      IPC_Signal(HSEM_CM4_to_CM7_Recvd);

      /* Normal FreeRTOS yield from interrupt semantics, where
      xHigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
      pdTRUE if the interrupt safe API unblocks a task that has a priority above
      that of the currently executing task. */
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * Reimplementation of sbSEND_COMPLETED(), defined as follows in FreeRTOSConfig.h:
     * #define sbSEND_COMPLETED( pxStreamBuffer ) vGenerateCore2Interrupt( pxStreamBuffer )
     * This is the implementation on the CM4 core 
     *
     * Called from within xMessageBufferSend().  As this function also calls
     * xMessageBufferSend() itself it is necessary to guard against a recursive
     * call.  If the message buffer just updated is the message buffer written to
     * by this function, then this is a recursive call, and the function can just
     * exit without taking further action.
     *************************************************************************/
    void vCore2SignalControlBufToSend( void * xUpdatedMessageBuffer )
    {
        MessageBufferHandle_t xUpdatedBuffer = ( MessageBufferHandle_t ) xUpdatedMessageBuffer;

        if( xUpdatedBuffer != Control47MessageBufferRef ) {
            /* Use xControlMessageBuffer to pass the handle of the message buffer
            written to by core 1 to the interrupt handler about to be generated in
            core 2. */
            if ( xSemaphoreTake(Control47SemRef, pdMS_TO_TICKS(10000)) == pdTRUE ) {
                xMessageBufferSend( Control47MessageBufferRef, &xUpdatedBuffer, sizeof( xUpdatedBuffer ), mbaDONT_BLOCK );
                /* Signal core1 */
                IPC_Signal(HSEM_CM4_to_CM7_Send);
            } else {
                DEBUG_PUTS("Cannot allocate ctrl47");
            }
        }
    }

    /**************************************************************************
     * A control buffer reception has been acknowledged by CM4
     * So release the control buffer semaphore
     *************************************************************************/
    static void prvCore2SendAckHandler( void )
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(Control47SemRef, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * Send a direct message ( w/o RTOS ) to CM7 core
     * Message must have been setup before in AMP_DirectBuffer
     *************************************************************************/
    void Ipc_CM4_SendDirect( void )
    {
        IPC_Signal(HSEM_CM4_to_CM7_Msg);
    }

    void HSEM2_IRQHandler ( void )
    {
        /* Get activated Interrupt bits */
        uint32_t SemMask = HSEM->C2ISR & HSEM->C2IER;
        
        /*Clear all activated bits */
        HSEM->C2ICR = SemMask;

        /* Handle all "free" interrupts */
        if ( SemMask & __HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_to_CM4_Send) )  prvCore2ReceiveHandler();
        if ( SemMask & __HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_to_CM4_Recvd) ) prvCore2SendAckHandler();
        if ( SemMask & __HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_to_CM4_Msg) )   CM4_handle_remote_direct();
    }
#endif
