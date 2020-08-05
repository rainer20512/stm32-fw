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

osSemaphoreId osSemaphore;
/* Exported variables --------------------------------------------------------*/
uint32_t gflags;

#if DEBUG_MODE 
  uint32_t debuglevel;
#endif

/* Private function prototypes -----------------------------------------------*/
static void CPU_CACHE_Enable(void);
static void MPU_Setup(void);
static void MPU_Config(void);
static void MPU_Dump(void);


/* Forward declarations for external initialization functions -----------------*/
void HW_InitJtagDebug(void);
void Init_DumpAndClearResetSource(void);
void Init_OtherDevices(void);
void Init_DefineTasks(void);


static void prvCore1InitTask( void *pvParameters );

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, uint32_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
/* Private functions ---------------------------------------------------------*/



uint32_t LedToggle ( uint32_t duration, uint32_t num )
{   uint32_t k = 0;
    for ( uint32_t i = 0; i < num; i++ ) {
        IO_UserLedToggle(num);
        for ( uint32_t j=0; j < duration * 2000; j++ ) {
            k += j;
            asm("nop");
        }
        IO_UserLedToggle(num);
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

    /* 
     * STM32H742 starts up with internal LDO enabled and VOS3 scale 
     * ie max core clock at this point is 200Mhz / AHB clk 100MHz
     */

    TM1637PinT clk = { GPIOA, 3 };
    TM1637PinT dio = { GPIOC, 0 };

    /* configure SWDIO and SWCLK pins, configure DBG and clear software reset flag in RCC */
    HW_InitJtagDebug();  

    /* PWR-CR3 must be written initially once to be able to change the VOS selection */
    PWR->CR3 = ( PWR_CR3_SCUEN | PWR_CR3_LDOEN );
  
    /* 
     * MPU Configuration: Define Flash ReadOnly (to detect faulty flash write accesses) 
     * Define SRAM3 as not cacheable and not bufferable ( used as DMA buffers & IPC mem )
     */
    MPU_Setup();
    MPU_Config();

    /* Enable the D- and I- Cache for M7  */
    CPU_CACHE_Enable();


    TM1637_Init( clk, dio, DELAY_TYPICAL);

    /* Init variables and structures for device handling */
    DevicesInit();

    #if USE_BASICTIMER > 0
        /* 
         * Start microsecond counter , must be done before Profiler is initialized 
         * and before HAL_Init, in case of USE_BASICTIMER_FOR_TICKS == 1
         */
        BASTMR_EarlyInit();
    #endif

    #if USE_QSPI  > 0 && USE_EEPROM_EMUL > 0
        QSPI_EarlyInit();
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

    /* Peripheral clock to HSE ( 8Mhz ) */
    SetPeripheralClkSource(CLKP_HSE);

    /* Configure the system clock to 400 MHz */
    // SystemClock_Config();

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

    /* Toggle LEDs to indicate system up and running with correct SYSTICK and frquency */
    for ( uint32_t i = 0; i < IO_UseLedGetNum(); i++ )
        IO_UserLedBlink(i, 2, 100);

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

    
    /* Define used semaphore */
    osSemaphoreDef(SEM);  
    /* Create the semaphore */
    osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);

    IO_AssignInterrupt(GPIO_PIN_13, PB_CB );


    /* Start the Core 1 init task, this task will start all other tasks */
    xTaskCreate( prvCore1InitTask, "Core1Init", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);		

    STATUS(7);

    /* Start scheduler */
    IO_UserLedOn(0);
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

    /* Start system tasks */
    TaskInitAll();
    STATUS(8);
    
    TaskNotify(TASK_OUT);

    STATUS(44);

    /* Terminate initialization task */ 
    vTaskDelete(NULL);
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
