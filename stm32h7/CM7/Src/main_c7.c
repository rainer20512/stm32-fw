/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_AMP_Dual_RTOS/CM7/Src/main.c
  * @author  MCD Application Team
  *          This is the main program for Cortex-M7 
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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "MessageBufferAMP.h"
#include "ipc.h"
#include "system/hw_util.h"
#include "system/clockconfig.h"
#include "eeprom.h"
#include "dev/devices.h"
#include "task/minitask.h"
#include "debug_helper.h"

#include "system/tm1637.h"
#include "cmsis_os.h"

/* external variables --------------------------------------------------------*/
extern uint32_t __SRAMUNCACHED_segment_start__;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define mainCHECK_TASK_PRIORITY			( configMAX_PRIORITIES - 2 )

/* Private variables ---------------------------------------------------------*/
static IPCMEM uint8_t StorageBuffer_data[mbaNUMBER_OF_CORE_2_TASKS][ mbaTASK_MESSAGE_BUFFER_SIZE ];
static IPCMEM uint8_t StorageBuffer_back[ mbaTASK_MESSAGE_BUFFER_SIZE ];

osSemaphoreId osSemaphore;
/* Exported variables --------------------------------------------------------*/
uint32_t gflags;

#if DEBUG_MODE 
  uint32_t debuglevel;
#endif

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MPU_Setup(void);
static void MPU_Config(void);
static void MPU_Dump(void);


/* Forward declarations for external initialization functions -----------------*/
void HW_InitJtagDebug(void);
void Init_DumpAndClearResetSource(void);
void Init_OtherDevices(void);
void Init_DefineTasks(void);


static void prvCheck2Task   ( void *pvParameters );
static void prvCore1Task    ( void *pvParameters );
static void prvCheckTask    ( void *pvParameters );
static void prvCore1InitTask( void *pvParameters );
static void prvCore1ModifiedTask( void *pvParameters );

static BaseType_t xAreMessageBufferAMPTasksStillRunning( void );
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, uint32_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
/* Private functions ---------------------------------------------------------*/



uint32_t LedToggle ( uint32_t duration, uint32_t num )
{   uint32_t k = 0;
    for ( uint32_t i = 0; i < num; i++ ) {
        BSP_LED_Toggle(LED2);
        for ( uint32_t j=0; j < duration * 2000; j++ ) {
            k += j;
            asm("nop");
        }
        BSP_LED_Toggle(LED2);
        for ( uint32_t j=0; j < duration * 2000; j++ ) {
            k += j;
            asm("nop");
        }
    }
    return k;
}


void PB_CB ( uint16_t u, uint16_t pinvalue, void * arg )
{  
    UNUSED(u);UNUSED(arg);
    osSemaphoreRelease (osSemaphore);
    DEBUG_PRINTF("User Button=%d\n", pinvalue);
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
#define STATUS(i)   TM1637_displayInteger(i,0,99)
int main(void)
{

    int32_t timeout;
    BaseType_t x;
   
    /* 
     * STM32H745 Nucleo does only support SMPS. So select once at startup 
     * and do not touch again. Due to this, VOS scale0 is inhibited and
     * max SYSCLK is 400MHZ.
     */
    HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

    TM1637PinT clk = { GPIOA, 3 };
    TM1637PinT dio = { GPIOC, 0 };

    /* configure SWDIO and SWCLK pins, configure DBG and clear software reset flag in RCC */
    HW_InitJtagDebug();  


    /* 
     * MPU Configuration: Define Flash ReadOnly (to detect faulty flash write accesses) 
     * Define SRAM3 as not cacheable and not bufferable ( used as DMA buffers & IPC mem )
     */
    MPU_Setup();
    MPU_Config();

    /* Enable the D- and I- Cache for M7  */
    CPU_CACHE_Enable();

    BSP_LED_Init(LED2); 
    LedToggle(250,2);

    TM1637_Init( clk, dio, DELAY_TYPICAL);


    /* Wait until CPU2 boots and enters in stop mode or timeout*/
    timeout = 0xFFFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
    if ( timeout < 0 )
    {
        Error_Handler_XX(-1, __FILE__, __LINE__);
    }


    /* STM32H7xx HAL library initialization:
       - TIM6 timer is configured by default as source of HAL time base, but user
         can eventually implement his proper time base source (another general purpose
         timer for application or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
         This application uses FreeRTOS, the RTOS initializes the systick to generate an interrupt each 1ms. 
         The systick is then used for FreeRTOS time base.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
    */


    #if USE_BASICTIMER > 0
        /* 
         * Start microsecond counter , must be done before Profiler is initialized 
         * and before HAL_Init, in case of USE_BASICTIMER_FOR_TICKS == 1
         */
        BASTMR_EarlyInit();
    #endif

    HAL_Init();

    STATUS(0);

    /* configure "simulated EEPROM" in flash and read config settings */
    Config_Init();

    /* 
    * Select clock source as configured in config variable, 
    * switch off MSI clock if no longer used
    */
    SystemClock_SetConfiguredClock();
    STATUS(11);

    // Switch LSE Clock on 
    LSEClockConfig(true, true);

    /* Configure the system clock to 400 MHz */
    // SystemClock_Config();

    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);
  

    STATUS(1);

    #if DEBUG_PROFILING > 0
        ProfilerInitTo(JOB_TASK_INIT);
    #endif

    STATUS(2);

    /* Set neccessary peripheral clocks and initialize IO_DEV and debug u(s)art */
    BasicDevInit();
    STATUS(3);
    Init_DumpAndClearResetSource();
    STATUS(4);

#if 0
    /* AIEC Common configuration: make CPU1 and CPU2 SWI line0
    sensitive to rising edge : Configured only once */
    HAL_EXTI_EdgeConfig(EXTI_LINE0 , EXTI_RISING_EDGE);
    HAL_EXTI_EdgeConfig(EXTI_LINE1 , EXTI_RISING_EDGE);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0xFU, 0U);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
#endif
    
    MPU_Dump();
    DEBUG_PRINTF("SYSCLK = %d\n", Get_SysClockFrequency() ); 
    DEBUG_PRINTF("SYSCLK = %dMHz\n", HAL_RCC_GetSysClockFreq()/1000000 ); 
    DEBUG_PRINTF("AHBCLK = %dMHz\n", HAL_RCC_GetHCLKFreq()/1000000 );
    DEBUG_PRINTF("APBCLK = %dMHz\n", HAL_RCC_GetPCLK1Freq()/1000000 );
    Init_OtherDevices();
    STATUS(5);

    Init_DefineTasks();
    STATUS(6);

    #if DEBUG_PROFILING > 0
        ProfilerSwitchTo(JOB_TASK_MAIN);  
    #endif

    
    /* 
     * Write address of control message buffer to RTC backup ram address 0 
     * from there it will be read upon startup from M4 core 
     */
    CTRL_HOOK_ENABLE_ACCESS();
    CTRL_BLOCK_HOOK_PUT(AMPCtrl_Block);
    MSGBUF_HOOK_PUT(AMP_DirectBuffer);
    CTRL_HOOK_DISABLE_ACCESS();

    /* Define used semaphore */
    osSemaphoreDef(SEM);  
    /* Create the semaphore */
    osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);

    IO_AssignInterrupt(GPIO_PIN_13, PB_CB );


    /* Start the Core 1 init task, this task will start all other tasks */
    xTaskCreate( prvCore1InitTask, "Core1Init", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);		

    STATUS(7);

    /* Start scheduler */
    vTaskStartScheduler();
    /* We should never get here as control is now taken by the scheduler */
    for (;;);
}

/**
  * @brief  BSP Configuration
  * @param  None
  * @retval None
  */

/* Initialization of CM7 Tasks and RTOS environment */
static void prvCore1InitTask( void *pvParameters )
{
  
    UNUSED(pvParameters);

    Ipc_CM7_Init();

    /* Create control message buffer */
    /* Create data message buffer */
    for( uint32_t x = 0; x < mbaNUMBER_OF_CORE_2_TASKS; x++ ) {
        DataMessageBuffer(x) = xMessageBufferCreateStatic( mbaTASK_MESSAGE_BUFFER_SIZE, StorageBuffer_data[x], &DataStreamBuffer(x));
        configASSERT( DataMessageBuffer(x) );
    }

    DataMessageBuffer(2) = xMessageBufferCreateStatic( mbaTASK_MESSAGE_BUFFER_SIZE, StorageBuffer_back, &DataStreamBuffer(2));
    configASSERT( DataMessageBuffer(2) );

    AMPCtrl_Block.num_xfer_used = 3;

    /* Start system tasks */
    TaskInitAll();
    STATUS(8);
    
    TaskNotify(TASK_OUT);

    /* Wake up CM4 from initial stop */
    Ipc_CM7_WakeUp_CM4 ();

#if 0
    /* Start the check task */
    xTaskCreate( prvCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

    /* Start the check2 task */
    xTaskCreate( prvCheck2Task, "Check2", configMINIMAL_STACK_SIZE, (void *)2, mainCHECK_TASK_PRIORITY, NULL );

    /* Start the Core 1 task */
    xTaskCreate( prvCore1Task, "AMPCore1", 256, NULL, tskIDLE_PRIORITY, NULL );		

    /* Start the Modified Core 1 task */
    xTaskCreate( prvCore1ModifiedTask, "Mod1", 256, NULL, tskIDLE_PRIORITY, NULL );		
#endif

    STATUS(44);

    /* Terminate initialization task */ 
    vTaskDelete(NULL);
}
/* This task will periodically send data to tasks running on Core 2
   via message buffers. */
static void prvCore1Task( void *pvParameters )
{
  BaseType_t x;
  uint32_t ulNextValue = 0;
  const TickType_t xDelay = pdMS_TO_TICKS( 250 );
  char cString[ 15 ];
  
  /* Remove warning about unused parameters. */
  ( void ) pvParameters;

  for( ;; )
  {
    /* Create the next string to send.  The value is incremented on each
    loop iteration, and the length of the string changes as the number of
    digits in the value increases. */
    sprintf( cString, "%lu", ( unsigned long ) ulNextValue );

    /* Send the value from this Core to the tasks on the Core 2 via the message 
    buffers.  This will result in sbSEND_COMPLETED()
    being executed, which in turn will write the handle of the message
    buffer written to into xControlMessageBuffer then generate an interrupt
    in core 2. */
    for( x = 0; x < mbaNUMBER_OF_CORE_2_TASKS; x++ )
    {
      DEBUG_PRINTF("Send %d on task#%d\n", ulNextValue, x);
      if ( xSemaphoreTake(DataMessageSem(x), pdMS_TO_TICKS(100000)) == pdTRUE ) {
          xMessageBufferSend( DataMessageBuffer(x), 
                             ( void * ) cString,
                             strlen( cString ),
                             mbaDONT_BLOCK );
          xSemaphoreGive(DataMessageSem(x));
      } else {
          DEBUG_PRINTF("Failed to obtain DataSemaphore %d\n", x );
      }
      /* Delay before repeating */
      vTaskDelay( xDelay );
    }

    ulNextValue++;
    DEBUG_PRINTF("Send packet %d\n", ulNextValue);
  }
}


/* This task will periodically send data to tasks running on Core 2
   via message buffers. */
static void prvCore1ModifiedTask( void *pvParameters )
{
  BaseType_t x;
  uint32_t ulNextValue = 0;
  const TickType_t xDelay = pdMS_TO_TICKS( 250 );
  char cString[ 15 ];
  
  /* Remove warning about unused parameters. */
  ( void ) pvParameters;

  while(1) {
      if ( osSemaphoreWait(osSemaphore , 100000) != osOK ) {
        DEBUG_PUTS("Giving up waiting for semaphore");
        vTaskDelete(NULL);
      }
      for( uint32_t loops = 0; loops < 40 ; loops++ )
      {
        /* Create the next string to send.  The value is incremented on each
        loop iteration, and the length of the string changes as the number of
        digits in the value increases. */
        sprintf( cString, "%lu", ( unsigned long ) ulNextValue );
    
        /* Send the value from this Core to the tasks on the Core 2 via the message 
        buffers.  This will result in sbSEND_COMPLETED()
        being executed, which in turn will write the handle of the message
        buffer written to into xControlMessageBuffer then generate an interrupt
        in core 2. */
        for( x = 0; x < mbaNUMBER_OF_CORE_2_TASKS; x++ )
        {
          if ( xSemaphoreTake(DataMessageSem(x), pdMS_TO_TICKS(100000)) == pdTRUE ) {
              xMessageBufferSend( DataMessageBuffer(x), 
                                 ( void * ) cString,
                                 strlen( cString ),
                                 mbaDONT_BLOCK );
              xSemaphoreGive(DataMessageSem(x));
          } else {
              DEBUG_PRINTF("Failed to obtain DataSemaphore %d\n", x );
          }
          /* Delay before repeating */
          // vTaskDelay( xDelay );
        }

        ulNextValue++;
        DEBUG_PRINTF("Send packet %d\n", ulNextValue);
      }
  }
}


/* 
  Check if the application still running
*/
static uint32_t ulLastCycleCounters[ mbaNUMBER_OF_CORE_2_TASKS ] = { 0 };
static BaseType_t xAreMessageBufferAMPTasksStillRunning( void )
{
  BaseType_t xDemoStatus = pdPASS;
  BaseType_t x;
  
  /* Called by the check task to determine the health status of the tasks
  implemented in this demo. */
  for( x = 0; x < mbaNUMBER_OF_CORE_2_TASKS; x++ )
  {
    if( ulLastCycleCounters[ x ] == ulCycleCounters[ x ] )
    {
      xDemoStatus = pdFAIL;
    }
    else
    {
      ulLastCycleCounters[ x ] = ulCycleCounters[ x ];
    }
  }
  
  return xDemoStatus;
}
static void prvCheck2Task( void *pvParameters )
{
  BaseType_t x;
  uint32_t val;
  uint32_t xReceivedBytes;
  
  /* The index into the xDataMessageBuffers and ulLoopCounter arrays is
  passed into this task using the task's parameter. */
  x = ( BaseType_t ) pvParameters;
  for( ;; )
  {
    xReceivedBytes = xMessageBufferReceive( DataMessageBuffer( x ),
                                            &val,
                                            sizeof( uint32_t ),
                                            portMAX_DELAY );
    
    /* Check the number of bytes received was as expected. */
    if (xReceivedBytes != sizeof(uint32_t) ) {
        DEBUG_PRINTF("Check2: Unexpected number of bytes received:%d vs %d\n", xReceivedBytes, sizeof(uint32_t));
    } else {
        /* If the received string matches that expected then increment the loop
        counter so the check task knows this task is still running. */
        if( val == 42 || val == 43  ) {
           ulLastCycleCounters[ val-42 ] ++; 
        } else {
          DEBUG_PRINTF("Checker got wrong return value %d\n",val);
        }
    }
    /* Expect the next string in sequence the next time around. */
  }
}

/* Check task fonction 
   */
static void prvCheckTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  const TickType_t xCycleFrequency = pdMS_TO_TICKS( 2000UL );
  
  /* Just to remove compiler warning. */
  ( void ) pvParameters;
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again. */
    vTaskDelayUntil( &xNextWakeTime, xCycleFrequency );

    
    if( xAreMessageBufferAMPTasksStillRunning() != pdPASS )
    {
      /* Application fail */
      BSP_LED_Off(LED1);
      BSP_LED_On(LED2);
    }
    else
    {
      /* Application still running */
      BSP_LED_Off(LED2);
      BSP_LED_Toggle(LED1);
    }
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

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (Cortex-M4 CPU, Bus matrix Clocks)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
#if 0
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  /*activate CSI clock mondatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE() ;

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();

}
#endif
/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/* Definition of non cached memory areas */
typedef struct {
    uint32_t baseAddress;
    uint32_t regionSize;
} MPURegionT;

#define MAX_MPU_REGIONS     1                       /* Number of defined MPU regions */
static MPURegionT mpuRegions[MAX_MPU_REGIONS]; 

/******************************************************************************
 * define MPU regions
 *****************************************************************************/
static void MPU_Setup(void)
{
    mpuRegions[0].baseAddress = (uint32_t)&__SRAMUNCACHED_segment_start__;
    mpuRegions[0].regionSize  = MPU_REGION_SIZE_16KB;
}


/******************************************************************************
 * configure the previously defined MPU regions
 *****************************************************************************/
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU "constant" attributes once */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  /* Now configure the "dynamic attributes for every region and setup MPU */
  for ( uint32_t i = 0; i < MAX_MPU_REGIONS; i++ ) {
      MPU_InitStruct.BaseAddress = mpuRegions[i].baseAddress;
      MPU_InitStruct.Size = mpuRegions[i].regionSize;
      HAL_MPU_ConfigRegion(&MPU_InitStruct);
  }

  /* Configure the MPU attributes as RO for CM7 Flash */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = FLASH_BANK1_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/******************************************************************************
 * dump info about every configured MPU region
 *****************************************************************************/
static void MPU_Dump(void)
{
  for ( uint32_t i = 0; i < MAX_MPU_REGIONS; i++ ) {
      DEBUG_PRINTF("Uncached RAM at 0x%p of size 0x%08x\n", 
        mpuRegions[i].baseAddress, 
        2 << mpuRegions[i].regionSize 
      );
  }
}



/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
