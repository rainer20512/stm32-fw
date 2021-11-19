/**
 ******************************************************************************
 * @file    init_c4.c
 * @author  rainer
 * @brief   Initialization and configuration tasks
 *          exclusively called by main
 ******************************************************************************
 */
#include "hardware.h"

#include "config/devices_config.h"
#include "dev/devices.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/* - Add additional conditional #includes here ------------------------------*/


#if USE_ONEWIRE > 0
    #include "onewire.h"
#endif


/******************************************************************************
 * Find, dump and clear the most recent reset reason in PWR-SRx
 *****************************************************************************/
void Init_DumpAndClearResetSource(void)
{
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET) {    
    #if DEBUG_MODE > 0
        DEBUG_PUTS("Start from RESET");
    #endif
  } else {
    /* Clear the Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    #if DEBUG_MODE > 0
        DEBUG_PUTS("Restart from Standby");
    #endif
  }
}

/******************************************************************************
 * Init all other devices except those, that are initialized early ( IODEV and
 * debug uart ). This code portion is heavily contaminated by #ifdef's
 * if you add addtional devices, THIS is the place to add the initialization
 * code 
 *****************************************************************************/
void Init_OtherDevices(void)
{
  int32_t dev_idx;
  #if defined(USE_ADC3)
      dev_idx = AddDevice(&HW_ADC3, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init ADC3-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
       
  #endif
  #if defined(USE_USART2)
      dev_idx = AddDevice(&HW_COM2, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init COM2-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
       
  #endif
  #if defined(USE_USART1)
      dev_idx = AddDevice(&HW_COM1, NULL, NULL );
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init USART-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }

  #endif
  #if USE_QENCODER > 0
      /* Do this before Display Init */  
      dev_idx = AddDevice(&QENC_DEV, NULL , NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init Quadrature Decoder-device %s\n", QENC_DEV.devName );
      } else {
        /* Init device */
        DeviceInitByIdx(dev_idx, NULL );
      }
  #endif
  #if USE_PWMTIMER > 0
      /* Do this before Display Init */  
     dev_idx = AddDevice(&HW_PWMTIMER, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init Timer-device %s\n",HW_PWMTIMER.devName );
      } else {
        DeviceInitByIdx(dev_idx, NULL );
      }
      /* Assign PWM device and channel to LCD */
      LCD_SetPWMDev(&HW_PWMTIMER, LCD_BKLGHT_CH);
  #endif
  #if USE_DISPLAY > 0
      dev_idx = AddDevice(&EPAPER_DEV, LCD_PostInit, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init LCD-device %s\n", EPAPER_DEV.devName );
      } else {
        /* Landscape mode */
        DeviceInitByIdx(dev_idx, (void *)1 );
      }
  #endif
  #if USE_ONEWIRE > 0
      dev_idx = AddDevice(&ONEWIRE_DEV, OW_PostInit, OW_PreDeInit);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init OneWire-device %s\n", ONEWIRE_DEV.devName );
      } else {
        /* Init Oowire device */
        DeviceInitByIdx(dev_idx, NULL );
      }
  #endif
  #if USE_QSPI > 0 && USE_EEPROM_EMUL == 0 
      dev_idx = AddDevice(&QSPI_DEV, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init QuadSpi device %s\n", QSPI_DEV.devName );
      } else {
        /* Init QuadSpi device */
        DeviceInitByIdx(dev_idx, NULL );
      }
  #endif
  #if defined(USER_I2CDEV)
      dev_idx = AddDevice(&USER_I2CDEV, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init I2C device %s\n", USER_I2CDEV.devName );
      } else {
        /* Init I2c device */
        if ( DeviceInitByIdx(dev_idx, NULL ) ) 
            SENSOR_IO_Init(&USER_I2C_HANDLE, NULL);
      }
  #endif
  #if defined(USE_CAN1)
      dev_idx = AddDevice(&HW_CAN1, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to add CAN device %s\n", HW_CAN1.devName );
      } else {
        /* Init I2c device */
        if ( !DeviceInitByIdx(dev_idx, NULL ) ) 
            DEBUG_PRINTF("Failed to init CAN device %s\n", HW_CAN1.devName );
      }
  #endif

}

#include "task/minitask.h"
#include "timer.h"
#include "rtc.h"
#include "interpreters.h"
#include "system/periodic.h"
#include "system/profiling.h"

#include "FreeRTOS.h"
#include "task.h"

#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif

#if USE_DS18X20 > 0
    #include "ds18xxx20.h"
#endif

#if USE_DS18X20 > 0
    #include "system/util.h"
#endif

void task_handle_out  (uint32_t);
void task_handle_qspi (uint32_t);
void task_handle_ser  (uint32_t);
void CM7_handle_remote(uint32_t);
void task_init_ser_in (void);
void task_handle_serin(uint32_t arg);
void task_handle_ser  (uint32_t);


/* Stack sizes are in 32bit words */
#define TMR_STACK_SIZE  256
#define RTC_STACK_SIZE  256
#define CMD_STACK_SIZE  384
#define OUT_STACK_SIZE  256
#define PER_STACK_SIZE  256
#define ADC_STACK_SIZE  128
#define RMT_STACK_SIZE  256
#define QSP_STACK_SIZE  128
#define SER_STACK_SIZE  256

static StackType_t tmrStack[TMR_STACK_SIZE];
static StackType_t rtcStack[RTC_STACK_SIZE];
static StackType_t cmdStack[CMD_STACK_SIZE];
static StackType_t outStack[OUT_STACK_SIZE];
static StackType_t perStack[PER_STACK_SIZE];
static StackType_t adcStack[ADC_STACK_SIZE];
static StackType_t rmtStack[RMT_STACK_SIZE];
#if USE_QSPI > 0
    static StackType_t qspStack[QSP_STACK_SIZE];
#endif
static StackType_t sinStack[SER_STACK_SIZE];


/******************************************************************************
 * Initially put all tasks to task list. 
 * This code portion is heavily contaminated by #ifdef's. If you add 
 * addtional devices, THIS is the place to add the initialization code 
 *****************************************************************************/
void Init_DefineTasks(void)
{
  /* The priority is defined by the sequence: The higher in the list, the higher the priority */
#if USE_QSPI > 0
  TaskRegisterTask(NULL,            task_handle_qspi, TASK_QSPI,      JOB_TASK_QSPI,     qspStack, QSP_STACK_SIZE, "QSPI task");
#endif
  TaskRegisterTask(task_init_rtc,   task_handle_tmr,  TASK_TMR,        JOB_TASK_TMR,      tmrStack, TMR_STACK_SIZE, "Timer task");
  TaskRegisterTask(NULL,            task_handle_rtc,  TASK_RTC,        JOB_TASK_RTC,      rtcStack, RTC_STACK_SIZE, "RTC task");
  TaskRegisterTask(NULL,            CM7_handle_remote,TASK_REMOTE_CM7,JOB_TASK_REMOTE,   rmtStack, RMT_STACK_SIZE, "CM4 remote task");
  TaskRegisterTask(NULL,            task_periodic,    TASK_PERIODIC,   JOB_TASK_PERIODIC, perStack, PER_STACK_SIZE, "periodic task");
  TaskRegisterTask(task_init_adc,   task_handle_adc,  TASK_ADC,        JOB_ADC,           adcStack, ADC_STACK_SIZE, "ADC task");
  TaskRegisterTask(task_init_ser_in,task_handle_serin,TASK_SERIN,      JOB_SER,           sinStack, SER_STACK_SIZE, "Display in task");
#if DEBUG_FEATURES > 0  && DEBUG_DEBUGIO == 0
  TaskRegisterTask(CMD_Init,        task_handle_com,  TASK_COM,        JOB_TASK_DBGIO,    cmdStack, CMD_STACK_SIZE, "Debug input");
  TaskRegisterTask(NULL,            task_handle_out,  TASK_LOG,        JOB_TASK_DBGIO,    outStack, OUT_STACK_SIZE, "Debug output");  
#endif
#if USE_THPSENSOR > 0
  TaskRegisterTask(task_init_thp,   task_handle_thp, TASK_THP,      JOB_TASK_MAIN,     "THP sensor task");
#endif
#if USE_DISPLAY > 0
  TaskRegisterTask(task_init_lcd,   task_handle_lcd, TASK_LCD,      JOB_TASK_LCD,      "LCD task");
#endif
#if USE_DS18X20 > 0
  TaskRegisterTask(task_init_ds,    task_handle_ds,   TASK_OW,      JOB_TASK_ONEWIRE,  "OneWire task");
#endif
#if USE_DS18X20 > 0
    AtHour(0,ResetMinMaxTemp, (void*)0, "ResetMinMaxTemp");
#endif
}