/**
  ******************************************************************************
  * @file    main_c7.c
  * @author  Rainer
  * @brief   Main routine for Cortex M7-Core in single and dual core environments
  *
  *          1. Setup MPU and enable D- and I-caches
  *          2. Init all devices and start profiling timer ( if configured )
  *          3. Initialize / Read simulated eeprom
  *          4. Set configured clock source/speed, 
  *          5. start LSE-clock, start USB-clock ( if configured )
  *          6. start basic devices ( IO-DEV, DEBUG, FATFS (if configured) )
  *          7. Start profiler ( if configured ) 
  *          8. start all other configured devices
  *          9. define initial set of FreeRTOS tasks
  *         10. Start the task-creator-task
  *         11. Start FreeRTOS scheduler
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"
//#include "main.h"

/* Standard includes. */
#include "stdio.h"
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "system/hw_util.h"
#include "system/mpu.h"
#include "system/clockconfig.h"
#include "eeprom.h"
#include "log.h"
#include "dev/devices.h"
#include "task/minitask.h"
#include "debug_helper.h"

#include "system/tm1637.h"
#include "cmsis_os.h"

/* external variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define mainCHECK_TASK_PRIORITY			( configMAX_PRIORITIES - 2 )

/* Private variables ---------------------------------------------------------*/

osSemaphoreId osSemaphore;
/* Exported variables --------------------------------------------------------*/
uint32_t gflags;

#if DEBUG_MODE > 0
  uint32_t console_debuglevel;
  uint32_t fatfs_debuglevel;
#endif

/* Private function prototypes -----------------------------------------------*/
void CPU_CACHE_Enable(void);
static void MPU_Setup(void);


/* Forward declarations for external initialization functions -----------------*/
void HW_InitJtagDebug(void);
void Init_DumpAndClearResetSource(void);
void Init_OtherDevices(void);
void Init_DefineTasks(void);
void LwIP_Start(void);

#define TESTSIZE    16384
#if USE_FMC > 0 
    uint32_t psram[TESTSIZE]  __attribute__((section(".psram3")));
    uint32_t sdram[TESTSIZE]  __attribute__((section(".sdram2")));
    uint32_t dtcm[TESTSIZE]  __attribute__((section(".non_init")));
    uint32_t axi[TESTSIZE]   __attribute__((section(".axismem")));
    uint32_t sram2[TESTSIZE]   __attribute__((section(".srammem")));
    uint32_t sram4[TESTSIZE]   __attribute__((section(".sram4")));
#endif
 
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

    ARD_PinT clk = { GPIOA, 3 };
    ARD_PinT dio = { GPIOC, 3 };

    /* configure SWDIO and SWCLK pins, configure DBG and clear software reset flag in RCC */
    HW_InitJtagDebug();  

    /* PWR-CR3 must be written initially once to be able to change the VOS selection */
    #if defined(STM32H745xx) || defined(STM32H747xx) || defined(STM32H742xx) || defined(STM32H743xx)
        PWR->CR3 = ( PWR_CR3_SCUEN | PWR_CR3_LDOEN );
    #elif defined(STM32H723xx) || defined(STM32H733xx) || defined(STM32H725xx) || defined(STM32H735xx) || defined(STM32H730xx)
        /*Internal SMPS on, internal LDO off */
        PWR->CR3 = PWR_CR3_SMPSEN;
        /* Select Vrange0 and wait for Vrange0 ready   */
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
        while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY));

    #endif
  
    /* 
     * MPU Configuration: Define Flash ReadOnly (to detect faulty flash write accesses) 
     * Define SRAM3 as not cacheable and not bufferable ( used as DMA buffers & IPC mem )
     */

    MPU_Setup();

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

    #if ( USE_QSPI  > 0 && USE_EEPROM_EMUL > 0 )
        XSpi_EarlyInit();
    #endif
    
    HAL_Init();

    STATUS(0);

    /* configure "simulated EEPROM" in flash and read config settings */
    Config_Init();
    Log_SetDebugLevels((uint32_t)config.dbg_level);

    /* 
    * Select clock source as configured in config variable, 
    * switch off MSI clock if no longer used
    */
    SystemClock_SetConfiguredClock();
    STATUS(11);

#if HAS_NO_LSE > 0
    // Switch LSI Clock on 
    LSIClockConfig(true, true);
#else
    // Switch LSE Clock on 
    LSEClockConfig(true, true);
#endif

    #if USE_USB > 0
        stm32h7_enable_hsi48();
    #endif

    /* Peripheral clock to HSE ( 8Mhz ) */
    SetPeripheralClkSource(CLKP_HSE);

    /* Configure the system clock to 400 MHz */
    // SystemClock_Config();

    STATUS(1);

    #if DEBUG_PROFILING > 0
        ProfilerInitTo(JOB_TASK_INIT);
    #endif

    STATUS(2);

    /* Set neccessary peripheral clocks and initialize IO_DEV, debug u(s)art and FAT file system */
    BasicDevInit();

    STATUS(3);
    Init_DumpAndClearResetSource();
    STATUS(4);

    /* Toggle LEDs to indicate system up and running with correct SYSTICK and frquency */
    for ( uint32_t i = 0; i < IO_UseLedGetNum(); i++ )
        IO_UserLedBlink(i, 2, 100);

    #if USE_EEPROM_EMUL > 0
        if ( Config_GetEmulationStatus() == false )
            DEBUG_PUTS("EEPROM emulation: Corrupted data"); 
    #endif

    MPU_Dump();
    DEBUG_PRINTF("SYSCLK = %d\n", Get_SysClockFrequency() ); 
    DEBUG_PRINTF("SYSCLK = %dMHz\n", HAL_RCC_GetSysClockFreq()/1000000 ); 
    DEBUG_PRINTF("AHBCLK = %dMHz\n", HAL_RCC_GetHCLKFreq()/1000000 );
    DEBUG_PRINTF("APBCLK = %dMHz\n", HAL_RCC_GetPCLK1Freq()/1000000 );
    Init_OtherDevices();
    STATUS(5);

    Init_DefineTasks();

#if USE_LWIP > 0
    LwIP_Start();
#endif

    STATUS(6);

    #if DEBUG_PROFILING > 0
        ProfilerSwitchTo(JOB_TASK_MAIN);  
    #endif

    


    /* Start the Core 1 init task, this task will start all other tasks */
    xTaskCreate( prvCore1InitTask, "Core1Init", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);		

    STATUS(7);

    /* Start scheduler */
    IO_UserLedOn(0);
    vTaskStartScheduler();

    /* We should never get here as control is now taken by the scheduler */
    for (;;);
}

/* Initialization of CM7 Tasks and RTOS environment */
static void prvCore1InitTask( void *pvParameters )
{
  
    UNUSED(pvParameters);

    /* Define used semaphore */
    osSemaphoreDef(SEM);  
    /* Create the semaphore */
    osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);

    IO_AssignInterrupt(GPIO_PIN_13, PB_CB );

    /* Start system tasks */
    TaskInitAll();
    STATUS(8);
    
    TaskNotify(TASK_LOG);

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
void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

void D_CACHE_Disable(void)
{
  /* Disable D-Cache */
  SCB_DisableDCache();
}

#define IS_PWROF2(a)           ( (a & (a-1)) == 0 ) 

/******************************************************************************
 * define MPU regions and enable them
 *  - all flash as RO memory ( to detect faulty flash writes )
 *  - DMA buffers as uncached
 *  - LWIP heap as uncached
 *  - ETH DMA descriptors as device memory
 *****************************************************************************/
static void MPU_Setup(void)
{
   extern uint32_t __SRAMUNCACHED_segment_start__,__SRAMUNCACHED_segment_size__;
   extern uint32_t __devicemem_start__, __devicemem_size__;
   extern uint32_t __axisdmamem_start__, __axisdmamem_size__; /**** 007 ****/
   extern uint32_t __lwipheap_start__, __lwipheap_size__;
   uint32_t flashSize = *(uint16_t*)FLASHSIZE_BASE * 1024;
   uint32_t size;


    MPU_AddRegion ( FLASH_BANK1_BASE,                          MPUTPYE_FLASH_NOWRITE,        flashSize,                                0 );
    MPU_AddRegion ( (uint32_t)&__SRAMUNCACHED_segment_start__, MPUTYPE_RAM_NONCACHEABLE,     (uint32_t)&__SRAMUNCACHED_segment_size__, 1 );


    /* find next greater or equal power of 2 for lwipheap size */
    size = (uint32_t)&__lwipheap_size__;
    if ( !IS_PWROF2(size ) ) {
       size = 1 << ( HW_GetLn2(size)+1);
    }
    MPU_AddRegion ( (uint32_t)&__lwipheap_start__,             MPUTYPE_RAM_NONCACHEABLE,     size,                                     2 );

    /* find next greater or equal power of 2 for devicemem size */
    size = (uint32_t)&__devicemem_size__;
    if ( !IS_PWROF2(size ) ) {
       size = 1 << ( HW_GetLn2(size)+1);
    }
    
    /* devicemem is part of uncached mem, a higher region ID will cause the devicemem area setting to supersede that of the uncached mem  */
    MPU_AddRegion ( (uint32_t)&__devicemem_start__,            MPUTYPE_RAM_DEVICEMEM_SHARED, size,                                     3 );

    /*** 007 **** find next greater or equal power of 2 for axisdmamem size */
    size = (uint32_t)&__axisdmamem_size__;
    if ( !IS_PWROF2(size ) ) {
       size = 1 << ( HW_GetLn2(size)+1);
    }
    
    /**** 007 **** axisdmamem is placed within AXIS MEM, used for SDMMC MDMA in conjunction with USB MSC   */
    MPU_AddRegion ( (uint32_t)&__axisdmamem_start__,            MPUTYPE_RAM_DEVICEMEM_SHARED, size,                                     4 );

    MPU_EnableAllRegions();
    MPU_Dump();

}
