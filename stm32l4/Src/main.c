/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          IT transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <debugio.h>

#include "config/config.h"
#include "debug_helper.h"
#include "system/clockconfig.h"
#include "system/status.h"
#include "eeprom.h"
#include "task/minitask.h"
#include "system/exti_handler.h"
#include "dev/io_dev.h"

#include "system/profiling.h"

#include "system/tm1637.h"
#define  STATUS(i)          TM1637_displayInteger(i,0,99)
ARD_PinT clk = { GPIOB, 0,0 };
ARD_PinT dio = { GPIOB, 1, 0 };


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */


/* Exported variables --------------------------------------------------------*/
uint32_t gflags;

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h" 
USBD_HandleTypeDef USBD_Device;

uint8_t CTL_error           =0;
uint8_t general_error_code  =0;

uint16_t missed_syncs=0;    //!< syncs missed since last reset *** Chng 052 ***
int16_t avg_skew_00=0;      // Average time skew when syncing at 00 *** Chng 052 ***
int16_t avg_skew_30=0;      // Average time skew when syncing at 30 *** Chng 052 ***

#if DEBUG_MODE 
  uint32_t console_debuglevel;
  uint32_t fatfs_debuglevel;
#endif


/* Forward declarations for external initialization functions -----------------*/
void HW_InitJtagDebug(void);
void Init_DumpAndClearResetSource(void);
void Init_OtherDevices(void);
void Init_DefineTasks(void);


#if DEBUG_STARTUP > 0

/******************************************************************************
 * @brief  Initialuize Pin C13 as output. Will be used to signel the progress
 *         of initialization by increasing number of pin toggles
 *         may be overwritten and no longer useable with DevicesInit
 *****************************************************************************/
void TogglePinInit(void)
{
    GPIO_InitTypeDef g;

    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pin  = GPIO_PIN_13;
    g.Pull = GPIO_PULLUP;
    g.Speed= GPIO_SPEED_FREQ_VERY_HIGH;

    __HAL_RCC_GPIOC_CLK_ENABLE();
   GpioInitHW(GPIOC, &g );
    GPIOC->BSRR = GPIO_PIN_13;
}

/******************************************************************************
 * @brief  Toggle a GPIO pin
 * @param  nCnt - number of toggles
 *         
 * @note   GPIO Clock MUST BE ENABLED BEFORE
 *****************************************************************************/
void TogglePin(uint32_t nCnt )
{
    while ( nCnt -- ) {
        GPIOC->BRR = GPIO_PIN_13;
        __asm("nop");
        GPIOC->BSRR = GPIO_PIN_13;
    }
}
#else
    #define TogglePinInit()
    #define TogglePin(a)
#endif



/******************************************************************************
 * Will be called exactly at he begin of every minute ( second == 0 )
 * \note As at second 0 a time correction may occur, don't use neither 
 *       the Milliseconds nor the millisecond timer in this routine
 *****************************************************************************/
 void every_minute_task ( void )
{
/* migrated
#if USE_DISPLAY > 0
    LCD_DisplayStatus(LCD_STATUS_TIME);
#endif
*/
#if 0 // RHB todo
    // Recalibrate OSCCAL every 15 min
    if ( RTC_GetMinute() % 15 == 0 ) {
            #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
                    COM_print_time('C', true);
            #endif
            Enable_TMR1();
            SetFlagBit(gflags, GFLAG_CALIBRATING_BIT);
    }
#endif
}



void PB_CB ( uint16_t u, uint16_t pinvalue, void * arg )
{  
    UNUSED(u);UNUSED(arg);
    UserButtonStatus = 1;
    DEBUG_PRINTF("User Button=%d\n", pinvalue);
}




static inline __attribute__((always_inline))
void CheckForSleep(void)
{
    if ( !TaskIsRunableTask() ){
        store_time("??");
        /* 
        * Nothing to do, goto sleep
        */

        /*
        * Suspend Tick increment to prevent wakeup by Systick interrupt. 
        * Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)
        */
        // ProfilerPush(JOB_PREP_SLEEP);
        HAL_SuspendTick();

        /* Enable Power Control clock */
        __HAL_RCC_PWR_CLK_ENABLE();

        /*{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{*/
        /* No debug output after this line. It will prevent STOP mode !               */
        /* Use special functions for debugging, which store debug output in temp.     */
        /* buffer until next wakeup                                                   */ 
        /*{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{*/
        if ( CanStop() ) {
            /* 
            * Enter STOP 2 mode, profiling does not work, because microsecond timer
            * is suspended in Stop mode. There is a workaround in StopMode_Before()
            * and StopMode_After(). But nevertheless, do a ProfilerPush here to
            * be able to Pop afterwards
            */
            StopMode_Before();
            IO_UserLedOff(0);
            IO_UserLedOff(1);
            switch ( config.StopMode ) {
                case 0:
                    ProfilerPush(JOB_STOP0);
                    store_time("S0");
                    HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
                    break;
                case 1:
                    ProfilerPush(JOB_STOP1);
                    store_time("S1");
                    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
                    break;
                case 2:
                    ProfilerPush(JOB_STOP2);
                    store_time("S2");
                    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
                    break;
                default:
                    Error_Handler(__FILE__, __LINE__);
            }
            StopMode_After(config.StopMode);
            /* Restore volatile clock settings after stop mode */
            if ( ClockMustReconfiguredAfterStop() ) ClockReconfigureAfterStop();
        } else {
            /* Enter Sleep Mode , wake up is done once User push-button is pressed */
            IO_UserLedOff(1);
            ProfilerPush(JOB_SLEEP);
            store_time("Sl");
            HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        }
        /*}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}*/
        /* Resume with normal debug output                                            */
        /*}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}*/

        /* Resume Tick interrupt if disabled prior to sleep mode entry*/
        HAL_ResumeTick();
        ProfilerPop();
        IO_UserLedOn(0);
        IO_UserLedOn(1);
        store_chr(' ');
        store_time("Wk");
        // ProfilerPop();
    } /* if nothing to do */
}


#include "../../../../lvgl/lv_conf.h"
#include "../../../../lvgl/src/display/lv_display.h"
#include "../../../../lvgl/src/drivers/lv_drivers.h"
void lv_example_get_started_1(void);
int GC9A01_init(void);
void gc9a01_send_cmd(lv_display_t * disp, const uint8_t * cmd, size_t cmd_size, 
                       const uint8_t * param, size_t param_size);

static void Init_Lvgl(void)
{
    lv_init();
    lv_display_t * disp = lv_gc9a01_create(240, 240, LV_LCD_FLAG_NONE, gc9a01_send_cmd, gc9a01_send_cmd );
    lv_example_get_started_1();
}

int main(void)
{
    /* configure SWDIO and SWCLK pins, configure DBG and clear software reset flag in RCC */
    HW_InitJtagDebug();  

    TogglePinInit();
    TogglePin(1);

    HAL_Init();
    TogglePin(2);

    /* configure "simulated EEPROM" in flash and read config settings */
    Config_Init();

    // TM1637_Init( clk, dio, DELAY_TYPICAL);

    /* Init variables and structures for device handling */
    DevicesInit();

    // Switch LSE Clock on, if LSE is equipped
    LSEClockConfig(true, true);

    TogglePin(3);

    /*
    * Attention: When using the RFM12 Module, the minimum required SYSCLK frequency is 8Mhz
    * otherwise the Receive buffer will overflow, producing tons of erroneous results
    */

    /* 
    * Select clock source as configured in config variable, 
    * switch off MSI clock if no longer used
    * Note: SystemClock_SetConfiguredClock needs a running LSE clock to measure real system clock
    */
    SystemClock_SetConfiguredClock();

    TogglePin(4);

    // HSI16 Clock always on , it may be used by some peripherals
    HSIClockConfig(true);
    TogglePin(5);

    #if USE_BASICTIMER > 0
        /* Start microsecond counter , must be done before Profiler is initialized */
        BASTMR_EarlyInit();
    #endif

    ProfilerInitTo(JOB_TASK_INIT);

    #if DEBUG_MODE > 0
        console_debuglevel = config.dbg_level;
    #endif

    TogglePin(6);

    /* Set neccessary peripheral clocks and initialize IO_DEV and debug u(s)art */
    BasicDevInit();

    TogglePin(7);

    Init_DumpAndClearResetSource();

    #if DEBUG_EEPROM_EMUL > 1  
        Config_Test();
    #endif

    /* Toggle LEDs to indicate system up and running with correct SYSTICK and frquency */
    for ( uint32_t i = 0; i < IO_UseLedGetNum(); i++ )
        IO_UserLedBlink(i, 2, 100);

    IO_AssignInterrupt(GPIO_PIN_13, PB_CB );

    #if DEBUG_FEATURES > 1
        DBG_dump_powersetting();
        DBG_dump_clocksetting();
        HAL_Delay(100);
    #endif

    DEBUG_PRINTF("SYSCLK = %d\n", Get_SysClockFrequency() );

    Init_OtherDevices();

    Init_DefineTasks();

    ProfilerSwitchTo(JOB_TASK_MAIN);  

    TaskInitAll();

    #if defined(PWM_DEVICE)
        void Init_StartUserPWM(void);
        Init_StartUserPWM();
    #endif

    Init_Lvgl();

    /* Run forever */
    while(1) { 
        CheckForSleep();
        TaskRunAll();
    }  
}




