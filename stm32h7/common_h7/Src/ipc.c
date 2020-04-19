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
#include "hardware.h"
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "ipc.h"

#include "debug_helper.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define HSEM_ID_0               (0U)            /* HW semaphore 0*/
#define CM7_TO_CM4_LINE         EXTI_LINE0      /* Signaling CM7 to CM4 */
#define CM4_TO_CM7_LINE         EXTI_LINE1      /* Signaling CM7 to CM4 */
  
/* The static message buffer size for the ctrl buffer
overhead of message buffers. */
#define CONTROL_MESSAGE_BUFFER_SIZE         24



/* Private functions ---------------------------------------------------------*/
#if defined(CORE_CM7)

    /* The control block is allocated by CM7 and referenced by CM4 */
    IPCMEM AMPCtrl_t AMPCtrl_Block;
    static IPCMEM uint8_t StorageBuffer_ctrl74 [ CONTROL_MESSAGE_BUFFER_SIZE ] ;
    static IPCMEM uint8_t StorageBuffer_ctrl47 [ CONTROL_MESSAGE_BUFFER_SIZE ] ;
#endif

#if defined(CORE_CM4)
#endif

/* Used to dimension the array used to hold the streams.*/
/* Defines the memory that will actually hold the streams within the stream buffer.*/
#if defined(CORE_CM7)
    /**************************************************************************
     * Initialize the IPC buffer and allocate the Control message buffer 
     * This is done by CM7 core
     *************************************************************************/
    void Ipc_Init ( void )
    {
        memset(&AMPCtrl_Block, 0, sizeof(AMPCtrl_t));
        AMPCtrl_Block.ID = 0x4354524C;
        DEBUG_PRINTF("IPC control block of size %d initialized\n", sizeof(AMPCtrl_t) );

        /* Create control message buffer and buffer semaphore for CM7 to CM4 communication */
        Control74MessageBuffer = xMessageBufferCreateStatic( CONTROL_MESSAGE_BUFFER_SIZE,StorageBuffer_ctrl74 ,&Control74StreamBuffer);  
#if USE_SEMAPHORES > 0
        Control74Sem           = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.ctrl_cm7_sem_buf );
        xSemaphoreGive(Control74Sem);
#endif
        
        /* Create control message buffer and buffer semaphore for CM4 to CM7 communication*/
        Control47MessageBuffer = xMessageBufferCreateStatic( CONTROL_MESSAGE_BUFFER_SIZE,StorageBuffer_ctrl47 ,&Control47StreamBuffer);  
#if USE_SEMAPHORES > 0
        Control47Sem           = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.ctrl_cm4_sem_buf );
        xSemaphoreGive(Control47Sem);
#endif

#if USE_SEMAPHORES > 0
        /* create all data buffer semaphores */
        for ( uint32_t i = 0; i < MAX_AMP_CTRL; i++ ) {
            DataMessageSem(i) = xSemaphoreCreateBinaryStatic( &AMPCtrl_Block.xfersem_buf[i] );
            xSemaphoreGive(DataMessageSem(i));

        }
#endif
    }


    /**************************************************************************
     * CM4 goes waits for Semaphore 0 and to sleep after reset
     * This CM7 routine is used to start CM4 again
     *************************************************************************/
    void WakeUp_CM4 ( void )
    {
        /* Cortex-M7 will release Cortex-M4  by means of HSEM notification */
        /*HW semaphore Clock enable*/
        __HAL_RCC_HSEM_CLK_ENABLE();
        /*Take HSEM */
        HAL_HSEM_FastTake(HSEM_ID_0);
        /*Release HSEM in order to wakeup the CPU2(CM4) from stop mode*/
        HAL_HSEM_Release(HSEM_ID_0,0);
    }


    /**************************************************************************
     * Reimplementation of sbSEND_COMPLETED(), defined as follows in FreeRTOSConfig.h:
     * #define sbSEND_COMPLETED( pxStreamBuffer ) vGenerateCore2Interrupt( pxStreamBuffer )
     * This is the implementation on the CM7 core 
     *
     * Called from within xMessageBufferSend().  As this function also calls
     * xMessageBufferSend() itself it is necessary to guard against a recursive
     * call.  If the message buffer just updated is the message buffer written to
     * by this function, then this is a recursive call, and the function can just
     * exit without taking further action.
     *************************************************************************/
    void vGenerateCore2Interrupt( void * xUpdatedMessageBuffer )
    {
      MessageBufferHandle_t xUpdatedBuffer = ( MessageBufferHandle_t ) xUpdatedMessageBuffer;
  
      if( xUpdatedBuffer != Control74MessageBuffer )
      {
        /* Use xControlMessageBuffer to pass the handle of the message buffer
        written to by core 1 to the interrupt handler about to be generated in
        core 2. */
        xMessageBufferSend( Control74MessageBuffer, &xUpdatedBuffer, sizeof( xUpdatedBuffer ), mbaDONT_BLOCK );
    
        /* Signal core2 by Software interrupt on Core2. */
        HAL_EXTI_D1_EventInputConfig(CM7_TO_CM4_LINE , EXTI_MODE_IT,  DISABLE);
        HAL_EXTI_D2_EventInputConfig(CM7_TO_CM4_LINE , EXTI_MODE_IT,  ENABLE);
        HAL_EXTI_GenerateSWInterrupt(CM7_TO_CM4_LINE);
      }
    }

    /**************************************************************************
     * Interrupt hander for the software interrupt from core2
     * A message buffer for core1 has been delivered from core2
     * The message buffer is passed within the control buffer.
     * So unpack the control buffer and deliver the message buffer to
     * the corresponding core1 task
     *************************************************************************/
    static void prvCore1InterruptHandler( void )
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
  
      /* Normal FreeRTOS yield from interrupt semantics, where
      xHigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
      pdTRUE if the interrupt safe API unblocks a task that has a priority above
      that of the currently executing task. */
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * Interrupt hander for the software interrupt from core1
     * see prvCore2InterruptHandler
     *************************************************************************/
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
      prvCore1InterruptHandler();
      HAL_EXTI_D1_ClearFlag(CM4_TO_CM7_LINE);
    }


#endif /* CORE_CM7 */

#if defined(CORE_CM4)
    
    #include "debug_helper.h"

    /**************************************************************************
     * Check IPC block for proper initialization by CM7 core
     *************************************************************************/
    void Ipc_Check(void)
    {
        bool bNullPtr;

        if ( AMPCtrl_Ptr->ID != AMP_ID ) {
            DEBUG_PUTS("AMP Control Block invalid");
        }

        
        bNullPtr = Control74MessageBufferRef == NULL;
        for ( uint32_t i = 0; i < AMPCtrl_Ptr->num_xfer_used; i++ ) {
            bNullPtr = bNullPtr || DataMessageBufferRef(i) == NULL || DataMessageSemRef(i) == NULL;
        }
        if ( bNullPtr ) 
            DEBUG_PUTS("One or more IPC message buffers could not be allocated");

    }

    /**************************************************************************
     * Interrupt hander for the software interrupt from core1
     * A message buffer for core2 has been delivered from core1
     * The message buffer is passed within the control buffer.
     * So unpack the control buffer and deliver the message buffer to
     * the corresponding core2 task
     *************************************************************************/
    static void prvCore2InterruptHandler( void )
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
  
      /* Normal FreeRTOS yield from interrupt semantics, where
      xHigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
      pdTRUE if the interrupt safe API unblocks a task that has a priority above
      that of the currently executing task. */
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /**************************************************************************
     * Interrupt hander for the software interrupt from core1
     * see prvCore2InterruptHandler
     *************************************************************************/
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
      prvCore2InterruptHandler();
      HAL_EXTI_D2_ClearFlag(CM7_TO_CM4_LINE);
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
    void vGenerateCore1Interrupt( void * xUpdatedMessageBuffer )
    {
      MessageBufferHandle_t xUpdatedBuffer = ( MessageBufferHandle_t ) xUpdatedMessageBuffer;
  
      if( xUpdatedBuffer != Control47MessageBufferRef )
      {
        /* Use xControlMessageBuffer to pass the handle of the message buffer
        written to by core 1 to the interrupt handler about to be generated in
        core 2. */
        xMessageBufferSend( Control47MessageBufferRef, &xUpdatedBuffer, sizeof( xUpdatedBuffer ), mbaDONT_BLOCK );
    
        /* Signal core2 by Software interrupt on Core2. */
        HAL_EXTI_D2_EventInputConfig(CM4_TO_CM7_LINE , EXTI_MODE_IT,  DISABLE);
        HAL_EXTI_D1_EventInputConfig(CM4_TO_CM7_LINE , EXTI_MODE_IT,  ENABLE);
        HAL_EXTI_GenerateSWInterrupt(CM4_TO_CM7_LINE);
      }
    }
#endif
