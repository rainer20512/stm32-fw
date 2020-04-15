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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define HSEM_ID_0 (0U) /* HW semaphore 0*/

/* The static message buffer size for the ctrl buffer
overhead of message buffers. */
#define CONTROL_MESSAGE_BUFFER_SIZE         24



/* Private functions ---------------------------------------------------------*/

IPCMEM AMPCtrl_t AMPCtrl_Block;

#if 0
IPCMEM MessageBufferHandle_t xControlMessageBuffer __attribute__((section(".RAM_D3_Z1")));
IPCMEM MessageBufferHandle_t xDataMessageBuffers[ mbaNUMBER_OF_CORE_2_TASKS ] __attribute__ ((section (".RAM_D3_Z2")));
static IPCMEM uint32_t ulCycleCounters[ mbaNUMBER_OF_CORE_2_TASKS ] __attribute__ ((section (".RAM_D3_Z3")));
/* The variable used to hold the stream buffer structure.*/
StaticStreamBuffer_t IPCMEM xStreamBufferStruct_ctrl  __attribute__ ((section (".RAM_D3_Z4")));
StaticStreamBuffer_t IPCMEM xStreamBufferStruct[mbaNUMBER_OF_CORE_2_TASKS] __attribute__ ((section (".RAM_D3_Z5")));
#endif
/* Used to dimension the array used to hold the streams.*/
/* Defines the memory that will actually hold the streams within the stream buffer.*/
static IPCMEM uint8_t StorageBuffer_ctrl [ CONTROL_MESSAGE_BUFFER_SIZE ] ;

void Ipc_Init ( void )
{
    memset(&AMPCtrl_Block, 0, sizeof(AMPCtrl_t));
    AMPCtrl_Block.ID = 0x4354524C;

    /* Create control message buffer */
    ControlMessageBuffer = xMessageBufferCreateStatic( CONTROL_MESSAGE_BUFFER_SIZE,StorageBuffer_ctrl ,&ControlStreamBuffer);  
}


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

/*-----------------------------------------------------------*/

/* Reimplementation of sbSEND_COMPLETED(), defined as follows in FreeRTOSConfig.h:
   #define sbSEND_COMPLETED( pxStreamBuffer ) vGenerateCore2Interrupt( pxStreamBuffer )

  Called from within xMessageBufferSend().  As this function also calls
  xMessageBufferSend() itself it is necessary to guard against a recursive
  call.  If the message buffer just updated is the message buffer written to
  by this function, then this is a recursive call, and the function can just
  exit without taking further action.
*/
void vGenerateCore2Interrupt( void * xUpdatedMessageBuffer )
{
  MessageBufferHandle_t xUpdatedBuffer = ( MessageBufferHandle_t ) xUpdatedMessageBuffer;
  
  if( xUpdatedBuffer != ControlMessageBuffer )
  {
    /* Use xControlMessageBuffer to pass the handle of the message buffer
    written to by core 1 to the interrupt handler about to be generated in
    core 2. */
    xMessageBufferSend( ControlMessageBuffer, &xUpdatedBuffer, sizeof( xUpdatedBuffer ), mbaDONT_BLOCK );
    
    /* This is where the interrupt would be generated. */
    HAL_EXTI_D1_EventInputConfig(EXTI_LINE0 , EXTI_MODE_IT,  DISABLE);
    HAL_EXTI_D2_EventInputConfig(EXTI_LINE0 , EXTI_MODE_IT,  ENABLE);
    HAL_EXTI_GenerateSWInterrupt(EXTI_LINE0);
  }
}

