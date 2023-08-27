/**
 ******************************************************************************
 * @file    INIT.c
 * @author  rainer
 * @brief   Initialization and configuration tasks
 *          exclusively called by main
 ******************************************************************************
 */
#include "stm32l4xx.h"

#include "config/config.h"
#include "config/devices_config.h"
#include "system/profiling.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/* - Add additional conditional #includes here ------------------------------*/

#if USE_RFM12 > 0 || USE_RFM69 > 0
    #include "rfm/rfm.h"
#endif

#if USE_DISPLAY > 0 || USE_PWMTIMER > 0
    #include "ui/lcd.h"
#endif

#if USE_ONEWIRE > 0
    #include "onewire.h"
#endif

#if defined(ADC1) && defined(USE_ADC1)
    #include "dev/adc_dev.h"
#endif

#if USE_USB > 0
    #include "dev/usb_dev.h"
#endif

#if USE_CAN > 0
    #include "dev/can_dev.h"
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
    
    /* Check and Clear the Wakeup flag */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF1) != RESET)
    {
       #if DEBUG_MODE > 0
           DEBUG_PUTS("Wakeup by external event");
       #endif
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
    }
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
  #if defined(ADC1) && defined(USE_ADC1)
      dev_idx = AddDevice(&HW_ADC1, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init ADC1-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
       
  #endif
  #if defined(USE_USART2) && !defined(USE_USART2_DEBUG)
      dev_idx = AddDevice(&HW_COM2, NULL, NULL );
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init USART2-device");
      } else {
        DeviceInitByIdx(dev_idx, (void *)0 );
      }

  #endif
  #if defined(USE_LPUART1) && !defined(USE_LPUART1_DEBUG)
      dev_idx = AddDevice(&HW_COM9, NULL, NULL );
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init LPUART1");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
  #endif
  #if USE_RFM12 > 0 || USE_RFM69 > 0
      #include "rfm/rfm.h"
      dev_idx = AddDevice(&RFM12_DEV, RFM_PostInit, RFM_PreDeInit);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init RFM-device %s\n", RFM12_DEV.devName );
      } else {
        DeviceInitByIdx(dev_idx, NULL);
        /* If no RFM device is found, DeInit again to reduce power consumption */
        if (!RFM_IsPresent() ) DeviceDeInitByIdx(dev_idx);
      }
  #endif
  #if USE_EPAPER > 0
      dev_idx = AddDevice(&EPAPER_DEV, EPD_PostInit, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init Epaper-device %s\n", EPAPER_DEV.devName );
      } else {
        /* Landscape mode */
        DeviceInitByIdx(dev_idx, (void *)1 );
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
      #if USE_DISPLAY > 0
          /* Assign PWM device and channel to LCD */
          LCD_SetPWMDev(&HW_PWMTIMER, LCD_BKLGHT_CH);
      #endif
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
  #if USE_QSPI > 0 || USE_OSPI > 0
      dev_idx = AddDevice(&XSPI_DEV, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to init Quad/OctoSpi device %s\n", XSPI_DEV.devName );
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
  #if USE_PERIPHTIMER > 0
      dev_idx = AddDevice(&PERIPH_TIMER, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PRINTF("Failed to add peripheral timer device %s\n", PERIPH_TIMER.devName );
      } else {
        /* Init peripheral timer */
        if ( !DeviceInitByIdx(dev_idx, NULL ) ) 
            DEBUG_PRINTF("Failed to init peripheral timer device %s\n", PERIPH_TIMER.devName );
      }
  #endif
  #if defined(USB_OTG_FS) && USE_USB > 0 
      void USBD_PostInit( const HW_DeviceType *self, void *args);
      dev_idx = AddDevice(&HW_USBDFS, NULL, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init USBDFS-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
       
  #endif
  #if defined(FMC_Bank1_R) && USE_FMC > 0 
      void FMC_PostInit( const HW_DeviceType *self, void *args);
      dev_idx = AddDevice(&HW_FMC, FMC_PostInit, NULL);
      if ( dev_idx < 0 ) {
        DEBUG_PUTS("Failed to init FMC-device");
      } else {
        DeviceInitByIdx(dev_idx, NULL);
      }
       
  #endif

}

#include "task/minitask.h"
#include "timer.h"
#include "rtc.h"
#include "interpreters.h"
#include "system/periodic.h"

#if USE_RFM12 > 0 || USE_RFM69 > 0
    #include "rfm/rfm.h"
#endif

#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif

#if USE_DS18X20 > 0
    #include "ds18xxx20.h"
#endif

#ifdef TX18LISTENER
    #include "sequencer/analyzer.h"
    #include "sequencer/pulses.h"
#endif
#if defined(TX18LISTENER) || USE_DS18X20 > 0
    #include "system/util.h"
#endif
#if USE_EPAPER > 0
    #include "disp/epaper.h"
    #include "disp/ssdxxxx_spi.h"
#endif

#if USE_USB > 0
    void U2U_InitTask(void);
    void U2U_RunTask(uint32_t arg);
#endif

#if DEBUG_FEATURES > 0 && DEBUG_DEBUGIO > 0
    void stdin_Init ( void );
    void task_handle_in( uint32_t arg );
#endif

void task_handle_out  (uint32_t);
void task_handle_xspi (uint32_t);

/******************************************************************************
 * Initially put all tasks to task list. 
 * This code portion is heavily contaminated by #ifdef's. If you add 
 * addtional devices, THIS is the place to add the initialization code 
 *****************************************************************************/
void Init_DefineTasks(void)
{
 TaskRegisterTask(task_init_rtc,  task_handle_tmr, TASK_TMR,      JOB_TASK_TMR,      "Timer task");
  TaskRegisterTask(NULL,          task_handle_rtc, TASK_RTC,      JOB_TASK_RTC,      "RTC task");
#if USE_USB > 0
  TaskRegisterTask(U2U_InitTask,  U2U_RunTask,     TASK_USBD,     JOB_TASK_USBD,     "U2U Traffic");
#endif
#if DEBUG_FEATURES > 0  
  TaskRegisterTask(CMD_Init,      task_handle_com, TASK_COM,      JOB_TASK_DBGIO,    "Debug input");
  #if DEBUG_DEBUGIO == 0
      TaskRegisterTask(NULL,      task_handle_out, TASK_LOG,      JOB_TASK_DBGIO,    "Debug output");  
 #else
      TaskRegisterTask(stdin_Init,task_handle_in,  TASK_STDIN,    JOB_TASK_DBGIO,    "Std input watch");  
 #endif
#endif
  TaskRegisterTask(NULL,          task_periodic,   TASK_PERIODIC, JOB_TASK_PERIODIC, "periodic task");

#if defined(USE_ADC1) || defined(USE_ADC2) || defined(USE_ADC3)
  TaskRegisterTask(task_init_adc, task_handle_adc, TASK_ADC,      JOB_ADC,           "ADC task");
#endif

#if USE_RFM12 > 0 || USE_RFM69 > 0
  TaskRegisterTask(task_init_rfm, task_handle_rfm, TASK_RFM,      JOB_TASK_RFM,      "RFM task");
#endif
#if USE_THPSENSOR > 0
  TaskRegisterTask(task_init_thp, task_handle_thp, TASK_THP,      JOB_TASK_MAIN,     "THP sensor task");
#endif
#if USE_DISPLAY > 0
  TaskRegisterTask(task_init_lcd, task_handle_lcd, TASK_LCD,      JOB_TASK_LCD,      "LCD task");
#endif
#if USE_DS18X20 > 0
  TaskRegisterTask(task_init_ds,  task_handle_ds,   TASK_OW,      JOB_TASK_ONEWIRE,  "OneWire task");
#endif
#if USE_QSPI > 0 || USE_OSPI > 0

  TaskRegisterTask(NULL,          task_handle_xspi, TASK_XSPI,    JOB_TASK_XSPI,     "QSPI task");
#endif
#ifdef TX18LISTENER
    TaskRegisterTask(PulsesInit,  task_handle_pulse,TASK_PULSE,   JOB_TASK_MAIN,     "Pulse sequencer");
    TaskRegisterTask(AnalyzerInit,task_handle_ana,  TASK_SEQUENCE,JOB_TASK_MAIN,     "Sequence analyzer");
#endif
#if USE_EPAPER > 0
    TaskRegisterTask(NULL,        task_handle_epd,TASK_EPD,       JOB_TASK_EPD,     "EPD task");
#endif
#if defined(TX18LISTENER) || USE_DS18X20 > 0
    AtHour(0,ResetMinMaxTemp, (void*)0, "ResetMinMaxTemp");
#endif
}