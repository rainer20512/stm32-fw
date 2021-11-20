/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_AMP_Dual_RTOS/CM4/Src/main.c
  * @author  MCD Application Team
  *          This is the main program for Cortex-M4 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Standard includes. */
#include "stdio.h"
#include "string.h"
#include "task/minitask.h"
#include "log.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "MessageBufferAMP.h"
#include "ipc.h"

#include "eeprom.h"
#include "dev/devices.h"
#include "task/minitask.h"
#include "debug_helper.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HSEM_CM4_WKUP (0U) /* HW semaphore 0*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
uint32_t gflags;

#if DEBUG_MODE 
  uint32_t console_debuglevel;
  uint32_t fatfs_debuglevel;
//  uint32_t debuglevel;
#endif

/* Set to pdFALSE if any errors are detected.  Used to inform the check task
that something might be wrong. */
BaseType_t xDemoStatus = pdPASS;

/* Forward declarations for external initialization functions -----------------*/
void Init_DumpAndClearResetSource(void);
void Init_OtherDevices(void);
void Init_DefineTasks(void);
void LwIP_Start ( void );

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void prvCore2Tasks( void *pvParameters );
static void prvCore2InterruptHandler( void );
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, uint32_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


uint32_t LedToggle ( uint32_t duration, uint32_t num )
{   uint32_t k = 0;
    for ( uint32_t i = 0; i < num; i++ ) {
        BSP_LED_Toggle(LED1);
        for ( uint32_t j=0; j < duration * 1000; j++ ) {
            k += j;
            asm("nop");
        }
        BSP_LED_Toggle(LED1);
        for ( uint32_t j=0; j < duration * 1000; j++ ) {
            k += j;
            asm("nop");
        }
    }
    return k;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
l   */
int main(void)
{

    BaseType_t x;
    BSP_LED_Init(LED1);
    LedToggle(250, 2);  
    AMPCtrl_t *ref;

    /* Activate wakeup from CM7 */
    Ipc_CM4_Init(INIT_RESTRICTED);

    /* 
     * Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
     * perform system initialization (system clock config, external memory configuration.. )   
     */
    HAL_PWREx_ClearPendingEvent();
    HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);


    /* STM32H7xx HAL library initialization:
        - Systick timer is configured by default as source of time base, but user 
        can eventually implement his proper time base source (a general purpose 
        timer for example or other time source), keeping in mind that Time base 
        duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
        handled in milliseconds basis.
        - Set NVIC Group Priority to 4
        - Low Level Initialization
    */

    /* initialize the ipc communication */
    Ipc_CM4_Init(INIT_FULLY);


    #if USE_BASICTIMER > 0
        /* 
         * Start microsecond counter , must be done before Profiler is initialized 
         * and before HAL_Init, in case of USE_BASICTIMER_FOR_TICKS == 1
         */
        BASTMR_EarlyInit();
    #endif


    HAL_Init();

    /* configure "simulated EEPROM" in flash and read config settings */
    Config_Init();
    Log_SetDebugLevels((uint32_t)config.dbg_level);
//    debuglevel = config.dbg_level;


    /* Set neccessary peripheral clocks and initialize IO_DEV and debug u(s)art */
    BasicDevInit();
    Init_DumpAndClearResetSource();
 
    DEBUG_PRINTF("SYSCLK = %dMHz\n", HAL_RCC_GetHCLKFreq()/1000000 ); 
    DEBUG_PRINTF("AHBCLK = %dMHz\n", HAL_RCC_GetHCLKFreq()/1000000 );
    DEBUG_PRINTF("APBCLK = %dMHz\n", HAL_RCC_GetPCLK1Freq()/1000000 );
    Init_OtherDevices();
  
    Init_DefineTasks();
    #if DEBUG_PROFILING > 0
        ProfilerSwitchTo(JOB_TASK_MAIN);  
    #endif

    TaskInitAll();
    
    TaskNotify(TASK_LOG);    

    LwIP_Start();

    for( x = 0; x < mbaNUMBER_OF_CORE_2_TASKS; x++ ) {    
        /* 
         * Pass the loop counter into the created task using the task's
         * parameter.  The task then uses the value as an index into the
         * ulCycleCounters and xDataMessageBuffers arrays. 
         */
        xTaskCreate( prvCore2Tasks,
                    "AMPCore2",
                    256,
                    ( void * ) x,
                    tskIDLE_PRIORITY + 1,
                    NULL );
    }
   
    

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);

}

static void prvCore2Tasks( void *pvParameters )
{
  BaseType_t x;
  size_t xReceivedBytes;
  uint32_t ulNextValue = 0;
  uint32_t retval;
  char cExpectedString[ 15 ];
  char cReceivedString[ 15 ];
  
  /* The index into the xDataMessageBuffers and ulLoopCounter arrays is
  passed into this task using the task's parameter. */
  x = ( BaseType_t ) pvParameters;
  configASSERT( x < mbaNUMBER_OF_CORE_2_TASKS );
  
  for( ;; )
  {
    /* Create the string that is expected to be received this time round. */
    sprintf( cExpectedString, "%lu", ( unsigned long ) ulNextValue );
    
    /* Wait to receive the next message from core 1. */
    memset( cReceivedString, 0x00, sizeof( cReceivedString ) );
    xReceivedBytes = xMessageBufferReceive( DataMessageBufferRef( x ),
                                            cReceivedString,
                                            sizeof( cReceivedString ),
                                            portMAX_DELAY );
    
    /* Check the number of bytes received was as expected. */
    if ( xReceivedBytes != strlen( cExpectedString ) ) {
        DEBUG_PRINTF("wrong str: expected:%s, got:%s\n", cExpectedString, cReceivedString);
        xDemoStatus = pdFAIL;
    } else {
        /* If the received string matches that expected then increment the loop
        counter so the check task knows this task is still running. */
        if( strcmp( cReceivedString, cExpectedString ) == 0 ) {
            DEBUG_PRINTF("Ok %d\n", ulNextValue);
            retval = 42+x;
            xMessageBufferSend( DataMessageBufferRef(2), 
                         ( void * )&retval,
                         sizeof(uint32_t),
                         mbaDONT_BLOCK );

        } else {
          DEBUG_PRINTF("Fail %d\n", ulNextValue);
          xDemoStatus = pdFAIL;
        }
    }
    /* Expect the next string in sequence the next time around. */
    ulNextValue++;
  }
}


/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, uint32_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  /* If the buffers to be provided to the Idle task are declared inside this
  function then they must be declared static - otherwise they will be allocated on
  the stack and so not exists after this function exits. */
  static StaticTask_t xIdleTaskTCB;
  static uint32_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

      /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
      state will be stored. */
      *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

      /* Pass out the array that will be used as the Idle task's stack. */
      *ppxIdleTaskStackBuffer = uxIdleTaskStack;

      /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
      Note that, as the array is necessarily of type StackType_t,
      configMINIMAL_STACK_SIZE is specified in words, not bytes. */
      *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
