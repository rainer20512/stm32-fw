/**
  ******************************************************************************
  * @file    interpreters.c
  * @author  Rainer 
  * @brief   Commmand line interpreters and execution function
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup CMDLINE
  * @{
  */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <inttypes.h>

#include "config/devices_config.h"
#include "interpreters.h"
#include "debug_helper.h"
#if DEBUG_PROFILING > 0
    #include "system/profiling.h"
#endif
#include "timer.h"
#include "task/minitask.h"
#include "rtc.h"
#include "dev/i2c_dev.h"
#include "system/hw_util.h"
#include "cmdline.h"
#include "dev/devices.h"

#if USE_DS18X20  > 0
    #include "onewire.h"
    #include "ds18xxx20.h"
#endif

#if USE_QENCODER
    #include "dev/qencode.h"
#endif

/** @addtogroup CmdLine
  * @{
  */ 


/* Private macro -------------------------------------------------------------*/

#define VOID(a)  (const void *)(a)

#define ADD_SUBMODULE(name) \
const InterpreterModuleT mdl##name =  {pmt##name, cmd##name, sizeof(cmd##name) / sizeof(CommandSetT) }


/* Private types---- ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/*
 * A "stack" of interpreter modules, only the first level is initialized to a
 * basic commandline interpreter
 */

/* Private functions ---------------------------------------------------------*/

#if DEBUG_FEATURES > 0 


#include "debug_sram.h"
#include "debug_gpio_exti.h"
 
#include "debug_pwr_rcc.h"


/*********************************************************************************
  * @brief  Submenu to dump system config
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
static bool Config_Menu ( char *cmdline, size_t len, const void * arg )
{
  char *word;
  size_t wordlen;
  uint32_t ret;

  UNUSED(cmdline); UNUSED(len);
  switch( (uint32_t)arg )  {
    case 0:
      DEBUG_PUTS("Not implemented here");
      break;
    case 1:
      DBG_dump_rtcclockconfig();
      break;
    case 2:
      DBG_dump_powersetting();
      break;
    case 3:
      if ( CMD_argc() < 1 ) {
          printf("Usage: 'PeriClk <0|1> - dump only active or all periph. clk states\n");
          return false;
      }
      CMD_get_one_word( &word, &wordlen );
      ret = CMD_to_number ( word, wordlen );
      DBG_dump_peripheralclocksetting(ret!=0);
      break;
    case 4:
      DBG_dump_peripheralclockconfig();
      break;
    case 5:
      if ( CMD_argc() < 1 ) {
          printf("Usage: 'SleepClk <0|1> - dump only active or all periph. clk states in LP mode\n");
          return false;
      }
      CMD_get_one_word( &word, &wordlen );
      ret = CMD_to_number ( word, wordlen );
      DBG_dump_peripheralclocksetting_insleepmode(ret!=0);
      break;
    case 6:
      DBG_dump_all_DMA ();
      break;
    default:
      DEBUG_PUTS("Clock Menu: command not implemented");
  } /* end switch */

  return true;
} /* Config_menu */

static const char *pmtClkCfg (void)
{
  return "Clock";
}

static const CommandSetT cmdClkCfg[] = {
  { "Clock",     ctype_fn, {Config_Menu},  VOID(0), "Show clocksettings" },
  { "RTCClock",  ctype_fn, {Config_Menu},  VOID(1), "Show RTC clocksettings" },
  { "Power",     ctype_fn, {Config_Menu},  VOID(2), "Show powersettings" },
  { "PeriClk",   ctype_fn, {Config_Menu},  VOID(3), "Show peripheral clock settings"  },
  { "PeriCfg",   ctype_fn, {Config_Menu},  VOID(4), "Show peripheral clock configuration"  },
  { "SleepClk",  ctype_fn, {Config_Menu},  VOID(5), "Show peripheral clock settings in sleep mode"  },
};
ADD_SUBMODULE(ClkCfg);

  void DBG_init_pin(char portletter, uint8_t portnum, uint32_t speed, uint32_t pupd_status, uint32_t init);
  void DBG_deinit_pin(char portletter, uint8_t portnum);


/*********************************************************************************
  * @brief  Init or Deinit GPIO Pin
  * @retval true on success, false otherwise
  ********************************************************************************/
static bool Init_GPIO ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;
  char portletter;   
  uint8_t portnum;
  uint32_t fnarg = (uint32_t)arg;
  uint32_t speed, pupd, outval;
  switch ( fnarg ) {
    case 0:
      if ( CMD_argc() < 5 ) {
        printf("Usage: Pin Init portletter portnum speed pupd outval\n");
        return false;
      }

      CMD_get_one_word( &word, &wordlen);
      portletter = *word;

      CMD_get_one_word( &word, &wordlen);
      portnum = CMD_to_number(word, wordlen) & 0x0f;
      CMD_get_one_word( &word, &wordlen);
      speed = CMD_to_number(word, wordlen) & 0x03;
      CMD_get_one_word( &word, &wordlen);
      pupd = CMD_to_number(word, wordlen) & 0x03;
      CMD_get_one_word( &word, &wordlen);
      outval = CMD_to_number(word, wordlen) & 0x01;

        DBG_init_pin(portletter, portnum, speed, pupd, outval);
        break;
    case 1:
      if ( CMD_argc() < 2 ) {
        printf("Usage: Pin DeInit portletter portnum1 [portnum2 [...]]\n");
        return false;
      }

      CMD_get_one_word( &word, &wordlen);
      portletter = *word;

      while (CMD_get_one_word( &word, &wordlen) ) {
          portnum = CMD_to_number(word, wordlen) & 0x0f;
          DBG_deinit_pin(portletter, portnum);
      }
      break;
    case 2:
        break;
    default:
        DEBUG_PUTS("Not implemented");
  }
  return true;
}

/*********************************************************************************
  * @brief  Toggle GPIO ports
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as mamy port numbers as possible
  ********************************************************************************/
static bool Toggle_GPIO ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;
  char portletter;   
  uint8_t portnum;
  
  if ( CMD_argc() < 2 ) {
    printf("Usage: Toggle portletter [portnum1 [portnum2[...]]\n");
    return false;
  }

  CMD_get_one_word( &word, &wordlen);
  portletter = *word;

  while (CMD_get_one_word( &word, &wordlen) ) {
      portnum = CMD_to_number(word, wordlen) & 0x0f;
      DBG_dump_toggle_pin(portletter, portnum,(uint32_t)arg==1);
  }
  return true;
}

/*********************************************************************************
  * @brief  Dump GPIO ports
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many GPIO port names as needed
  ********************************************************************************/
static bool Dump_GPIO ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;

  if ( CMD_argc() < 1 ) {
    printf("Usage: GPIO letter [letter [...]]\n");
    return false;
  }

  while (CMD_get_one_word( &word, &wordlen) ) {
    for ( uint32_t i = 0; i < wordlen; i++ ) {
      DBG_dump_gpio_status(*(word+i));
    }
  }
  return true;
}

static bool Devices_Menu ( char *cmdline, size_t len, const void * arg )
{
  char *word;
  size_t wordlen;
  uint8_t idx;
  uint32_t initarg;
    
  UNUSED(cmdline);UNUSED(len);

  switch( (uint32_t)arg )  {
    case 0:
      DBG_sram();
      break;
    case 1:
      DBG_dump_exti_config(0);
      DBG_dump_exti_config(1);
      break;
    case 2:
      DBG_dump_nvic_config();
      break;
    case 3:
      DBG_dump_devices(true);
      break;
    case 4:
    case 5:
      if ( CMD_argc() < 1 ) {
        printf("Usage: 'De/ReInit <devicenum>\n");
        return false;
      }
      CMD_get_one_word( &word, &wordlen );
      idx = CMD_to_number ( word, wordlen );
      if ( (uint32_t)arg == 5) {
        initarg = 0;
        if ( CMD_argc() > 1 ) {
          CMD_get_one_word( &word, &wordlen );
          initarg = CMD_to_number ( word, wordlen );
        }
        DeviceInitByIdx(idx, (void *)initarg);
      } else {
        DeviceDeInitByIdx(idx);
      }
      break;
    default:
      DEBUG_PUTS("RFM_Menu: command not implemented");
  } /* end switch */

  return true;
} /* Devices_menu */


static const char *pmtDevices (void)
{
  return "Devices";
}

static const CommandSetT cmdDevices[] = {
  { "SRAM",      ctype_fn, {Devices_Menu},    VOID(0), "Show SRAM sections and usage" },
  { "EXTI",      ctype_fn, {Devices_Menu},    VOID(1), "Show EXTI settings" },
  { "NVIC",      ctype_fn, {Devices_Menu},    VOID(2), "Show NVIC settings" },
  { "GPIO",      ctype_fn, {Dump_GPIO},       VOID(0), "Show GPIO settings"  },
  { "Devices",   ctype_fn, {Devices_Menu},    VOID(3), "Dump Devices"  },
  { "Dev Init",  ctype_fn, {Devices_Menu},    VOID(4), "DeInit Device i"  },
  { "Dev DeInit",ctype_fn, {Devices_Menu},    VOID(5), "ReInit Device i"  },
  { "Toggle 8x", ctype_fn, {Toggle_GPIO},     VOID(0), "Toggle GPIO pin 8x"  },
  { "Alter",     ctype_fn, {Toggle_GPIO},     VOID(1), "Alter output value of pin"  },
  { "Pin Init",  ctype_fn, {Init_GPIO},       VOID(0), "Set Output Pin"  },
  { "Pin DeInit",ctype_fn, {Init_GPIO},       VOID(1), "Reset Output Pin"  },
};
ADD_SUBMODULE(Devices);


#endif  // DEBUG_FEATURES >0



static bool Test_WD_reset ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;

  if ( CMD_argc() < 1 || ( CMD_get_one_word( &word, &wordlen ), *word != 'X' && *word != 'x') ) {
      printf("Usage: 'Reset X\n");
      return false;
  }

  DEBUG_PUTS("Watchdog reset");

  /* the following call will not return */
  TimerWatchdogReset(500);

  return true;
}
 
#if DEBUG_MODE > 0
    static bool Test_Sleep ( char *cmdline, size_t len, const void * arg )
    {
      UNUSED(cmdline);UNUSED(len);UNUSED(arg);
      char *word;
      size_t wordlen;
      char c;

      if ( CMD_argc() < 1 || ( CMD_get_one_word( &word, &wordlen ), c = *word,  c != '0' && c != '1') ) {
          printf("Usage: 'Sleep {0|1}\n");
          return false;
      }

      if ( c == '0') {
          bAllowStop = false;
          DEBUG_PUTS("Inhibit Stop");
      } else {
          bAllowStop = true;
          DEBUG_PUTS("Allow Stop");
      }
  
      return true;
    } 
#endif

static uint8_t tmrID;
void TimerCB ( uint32_t arg)
{
   UNUSED(arg);
   COM_print_time('*', true);
}

static bool Test_TmrAbs ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;

  if ( CMD_argc() < 1 ) {
      printf("Usage: 'TmrAbs nn [1] \n");
      return false;
  }

  CMD_get_one_word( &word, &wordlen );
  uint32_t ret = CMD_to_number ( word, wordlen );

  COM_print_time('?', true);
  tmrID = MsTimerSetAbs ( MILLISEC_TO_TIMERUNIT(ret), TimerCB, 0 );

  return true;
}

static bool Test_TmrRel ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;
  bool bPeriodic;

  if ( CMD_argc() < 1 ) {
      printf("Usage: 'TmrRel nn [1] \n");
      return false;
  }

  CMD_get_one_word( &word, &wordlen );
  uint32_t ret = CMD_to_number ( word, wordlen );
  uint32_t work = 0;

  if ( CMD_argc() >= 1 ) {
      CMD_get_one_word( &word, &wordlen );
      work = CMD_to_number ( word, wordlen );
      bPeriodic = work > 0;
      DEBUG_PRINTF("Periodic Rel Timer to %d\n", ret );
  } else {
      DEBUG_PRINTF("One Shot Rel Timer to %d\n", ret );
      bPeriodic = false;
  }
  COM_print_time('?', true);
  tmrID = MsTimerSetRel ( MILLISEC_TO_TIMERUNIT(ret), bPeriodic, TimerCB, 0 );

  return true;
}

static bool Test_TmrDel ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  MsTimerDelete(tmrID);
  return true;
}

void TestCB(uint32_t arg)
{
    DEBUG_PRINTF("@%02x Tmr activation by iteration %d\n", RTC_GetS256(), arg);
}

static bool Test_TmrMulti ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;

  if ( CMD_argc() < 1 ) {
      printf("Usage: 'TmrMulti nn \n");
      return false;
  }

  CMD_get_one_word( &word, &wordlen );
  uint32_t ret = CMD_to_number ( word, wordlen );

  int8_t TmrID = MsTimerAllocate(NO_TIMER_ID);
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("Could not allocate TimerID");
    return false;
  }

  for ( uint32_t i = 0; i < ret; i++ ) {
    if ( MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(20), 0, TestCB, i) == NO_TIMER_ID ) 
        DEBUG_PUTS("Could not ReSet Timer");
    uint32_t cnt = RTC_GetS256()+2;
    while ( RTC_GetS256() != cnt );
  }
  return true;
}

static bool Test_TmrKill ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
 
  int8_t TmrID;
  uint16_t cnt;
  uint32_t i = 0;

#define DELTA_CNT   4

  TmrID=MsTimerSetRel(MILLISEC_TO_TIMERUNIT(15), 0, TestCB, i++ );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not Set Timer 0");
    return false;
  }
   cnt = DELTA_CNT + RTC_GetTimer();
   while ( RTC_GetS256() == cnt );
  
  TmrID=MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(20), 0, TestCB, i++ );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not Set Timer 1");
    return false;
  }
   cnt = DELTA_CNT + RTC_GetTimer();
   while ( RTC_GetS256() == cnt );
  
  TmrID=MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(25), 0, TestCB, i++ );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not set timer 2");
    return false;
  }
   cnt = DELTA_CNT + RTC_GetTimer();
   while ( RTC_GetS256() == cnt );
  
  TmrID=MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(30), 0, TestCB, i++ );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not set timer 3");
    return false;
  }
   cnt = DELTA_CNT + RTC_GetTimer();
   while ( RTC_GetS256() == cnt );
  
  TmrID=MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(35), 0, TestCB, i++ );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not set timer 4");
    return false;
  }

   cnt = DELTA_CNT + RTC_GetTimer();
   while ( RTC_GetS256() == cnt );
  
  TmrID=MsTimerReUseRel(TmrID, MILLISEC_TO_TIMERUNIT(40), 0, TestCB, 0 );
  if ( TmrID == NO_TIMER_ID ) {
    DEBUG_PUTS("TmrKill: Could not set timer 5");
    return false;
  }


  return true;
}

void UserPinSignal1(void)
{
  IO_UserLedOn(2);
  /*for ( uint32_t i = 0; i < 8 ; i++ )
     asm("NOP");*/
  DEBUG_PUTS("1 Step");
  IO_UserLedOff(2);
}
static uint32_t downcnt;
static int8_t cnttmr;
 
void SignalNCB ( uint32_t arg )
{
  UNUSED(arg);
  if ( downcnt == 0 ) {
    MsTimerDelete(cnttmr);
  } else {
    UserPinSignal1();
    downcnt --;
  }
}

void UserPinSignalN(uint32_t n, uint32_t period)
{
  downcnt = n;
  cnttmr =  MsTimerSetRel ( MILLISEC_TO_TIMERUNIT(period), true, SignalNCB, 0 );
  SignalNCB(0);
}

static bool Test_Toggle ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  char *word;
  size_t wordlen;

  if ( CMD_argc() < 2 ) {
      printf("Usage: 'Toggle <steps> <period> \n");
      return false;
  }

  CMD_get_one_word( &word, &wordlen );
  uint32_t p1 = CMD_to_number ( word, wordlen );

  CMD_get_one_word( &word, &wordlen );
  uint32_t p2 = CMD_to_number ( word, wordlen );

  DEBUG_PRINTF("%d Steps, Period %d\n", p1,p2  );
  UserPinSignalN(p1, p2);
  return true;
}

#if USE_QENCODER > 0 || 1
/*********************************************************************************
  * @brief  Submenu for FM24 Access functions
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
bool eeprom_update_config_byte(uint8_t cfg_idx, uint8_t newval);
static const char usart2[]="The quick brown fox jumps over the lazy dog";
static bool Test_Menu ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);
  char *word;
  uint8_t c;
  size_t wordlen;
  uint8_t pattern;

  switch((uint32_t)arg) {
  #if USE_QENCODER > 0
      case 2:
        if ( CMD_argc() < 1 ) {
            printf("Usage: 'QEnc Test 0 | 1 \n");
            return false;
        }
       CMD_get_one_word( &word, &wordlen );
       pattern = CMD_to_number ( word, wordlen );
       if ( pattern ) {
          QEnc_SetCallbacks(&HW_QENC1, TestOnRotate, TestOnClick, TestOnDblClick );
          QEnc_Activate(&HW_QENC1);
       } else {
          QEnc_DeActivate(&HW_QENC1);
       }
       break;
  #endif
  #if USE_EEPROM_EMUL > 0
      case 3:
        if ( CMD_argc() < 1 ) {
            printf("Usage: 'Write config[31] <x> times\n");
            return false;
        }
       CMD_get_one_word( &word, &wordlen );
       pattern = CMD_to_number ( word, wordlen );
       printf("Writing %d times to config[31]\n", pattern);
       for ( uint32_t i = 0; i < pattern; i++ )
            eeprom_update_config_byte(31, (uint8_t)i);
       break;
#endif
#if defined(USE_LPUART1) 
      case 4:
        if ( CMD_argc() < 1 ) {
            printf("Usage: 'LPUART Write/read char  <x> times\n");
            return false;
        }
       CMD_get_one_word( &word, &wordlen );
       pattern = CMD_to_number ( word, wordlen );
       c = '?';
       for ( uint32_t i = 0; i < pattern; i++ )
            UsartTxRxOneByteWait   (&HandleCOM9, &c, 3000);
       break;
#endif
#if defined(USE_USART2)
    case 5:
        UsartStartTx(& HandleCOM2, (uint8_t *)usart2,sizeof(usart2));
        break;
#endif
    default:
      DEBUG_PUTS("Test_Menu: command not implemented");
  } /* end switch */

  return true;

} /* Test_Menu */
#endif
#include "system/periodic.h"
void stats_display(void);

#if USE_ETH > 0
  void ethernetif_statistic ( void );
#endif

/*********************************************************************************
  * @brief  Submenu for system functions
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
static bool System_Menu ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);

  switch((uint32_t)arg) {
    case 0:
        TaskDumpList();
        break;
    case 1:
        PeriodicDumpList();
        break;
    case 2:
        stats_display();
        break;
#if USE_ETH > 0 && USE_ETH_PHY_ENC28J60 > 0
    case 3:
        ethernetif_statistic();
        break;
#endif
    /* sample entry
    case 1:
        if ( CMD_argc() < 1 ) {
        printf("Usage: 'ePaper Init <pattern> \n");
        return false;
        }
        CMD_get_one_word( &word, &wordlen );
        pattern = CMD_to_number ( word, wordlen );
        // BSP_EPD_Init();
        EPD_Test(pattern);
    break;
    sample entry */
    default:
      DEBUG_PUTS("System_Menu: command not implemented");
  } /* end switch */

  return true;

} /* System_Menu */


static const char *pmtTest (void)
{
  return "T&S";
}

static const CommandSetT cmdTest[] = {
  { "Reset",           ctype_fn, .exec.fn = Test_WD_reset,  VOID(0), "Perform a watchdog reset" },
#if DEBUG_MODE > 0
  { "Sleep",           ctype_fn, .exec.fn = Test_Sleep,     VOID(0), "Allow/Inhibit sleep mode" },
#endif
  { "Task list",       ctype_fn, .exec.fn = System_Menu,    VOID(0), "Show task list" },
  { "Periodic lists",  ctype_fn, .exec.fn = System_Menu,    VOID(1), "Show periodic lists" },
  { "LwIP Stats",      ctype_fn, .exec.fn = System_Menu,    VOID(2), "Show LwIP stats" },

  { "TmrAbs",          ctype_fn, .exec.fn = Test_TmrAbs,    VOID(0), "Set Abs Timer" },
  { "TmrRel",          ctype_fn, .exec.fn = Test_TmrRel,    VOID(0), "Set Rel Timer" },
  { "TmrDel",          ctype_fn, .exec.fn = Test_TmrDel,    VOID(0), "Delete priodic timer" },
  { "Multi Tmr",       ctype_fn, .exec.fn = Test_TmrMulti,  VOID(0), "Lots of timers" }, 
  { "Kill Tmr",        ctype_fn, .exec.fn = Test_TmrKill,   VOID(0), "Generates Error" }, 
  { "Toggle",          ctype_fn, .exec.fn = Test_Toggle,    VOID(0), "Toggle UserLed2" },
#if defined(USE_LPUART1) 
  { "LPUART1 test",     ctype_fn, .exec.fn = Test_Menu,     VOID(4), "Test LPUART1"}, 
#endif
#if USE_EEPROM_EMUL > 0
  { "Write config ",   ctype_fn, .exec.fn = Test_Menu,      VOID(3), "Write config[31] x times"}, 
#endif
#if defined(USE_ETH)
  { "ETH IF statistic",ctype_fn, .exec.fn = System_Menu,    VOID(3), "Show ETH Interface stats"},   
#endif
};
ADD_SUBMODULE(Test);






#if USE_QSPI > 0

    #include "dev/qspi_dev.h"
    /*********************************************************************************
     * @brief  Submenu for QuadSpi
     *         
     * @retval true on success, false otherwise
     *
     * @note  will try to read as many parameters as needed
     ********************************************************************************/

    #define PGSIZE  10240
    static uint8_t PageBuffer[PGSIZE];
#if defined(QSPI1_USE_IRQ)

    void rddoneCB(QSpiHandleT *hnd )
    {
        UNUSED(hnd);
        DEBUG_PRINTTS("read terminated ok\n");
    }
    void wrdoneCB(QSpiHandleT *hnd )
    {
        UNUSED(hnd);
        DEBUG_PRINTTS("write/erase terminated ok\n");
    }
    void errorCB(QSpiHandleT *hnd )
    {
        UNUSED(hnd);
        DEBUG_PRINTTS("Async Op terminated with error\n");
    }
#endif
    static bool QSPI_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t cnt;
      uint32_t addr;
      uint32_t i,j;
      bool ret;
      
      UNUSED(cmdline);UNUSED(len);
#if defined(QSPI1_USE_IRQ)
      QSpi_SetAsyncCallbacks(&QSpi1Handle, rddoneCB, wrdoneCB, errorCB);
#endif

      switch((uint32_t)arg) {
        case 0:
            printf("Flash Size= ...... %d kByte\n", QSpi1Handle.geometry.FlashSize/1024);
            printf("Erase sector size= %d Byte\n", QSpi1Handle.geometry.EraseSectorSize);
            printf("   sector count= . %d\n", QSpi1Handle.geometry.EraseSectorsNumber);
            printf("Write page size= . %d Byte\n", QSpi1Handle.geometry.ProgPageSize);
            printf("   page count= ... %d\n", QSpi1Handle.geometry.ProgPagesNumber);
            printf("pages per sector=  %d\n", QSpi1Handle.geometry.EraseSectorSize/QSpi1Handle.geometry.ProgPageSize);            
            break;
        case 1:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Erase <sector#> [<cnt>] - erase sector  +<cnt> following\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( CMD_argc() >= 1 ) {
                CMD_get_one_word( &word, &wordlen );
                cnt = CMD_to_number ( word, wordlen );
            } else {
                cnt = 0;
            }
            addr =  QSpi1Handle.geometry.EraseSectorSize * num;
            for ( i=0; i <=cnt; i++ ) {
                printf("Erase sector %d (startaddr=0x%08x) - ", num+i, addr );
                ret = QSpi_Erase_SectorWait(&QSpi1Handle, addr);
                printf ( "%s\n", ret ? "ok": "fail");
                addr += QSpi1Handle.geometry.EraseSectorSize;
            }
            break;
        case 2:
            if ( QSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", QSpi1Handle.geometry.ProgPageSize);
                return false;
            }
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Write <page#> [<cnt>] - write page  +<cnt> following\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( CMD_argc() >= 1 ) {
                CMD_get_one_word( &word, &wordlen );
                cnt = CMD_to_number ( word, wordlen );
            } else {
                cnt = 0;
            }
            addr =  QSpi1Handle.geometry.ProgPageSize * num;
            for ( i=0; i <=cnt; i++ ) {
                for ( j  = 0; j < QSpi1Handle.geometry.ProgPageSize; j++ )
                    PageBuffer[j]=i+j;
                printf("Write page %d (startaddr=0x%08x) - ", num+i, addr );
                ret = QSpi_WriteWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize );
                printf ( "%s\n", ret ? "ok": "fail");
                addr += QSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 3:
            if ( QSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", QSpi1Handle.geometry.ProgPageSize);
                return false;
            }
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Compare <page#> [<cnt>] - write page  +<cnt> following\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( CMD_argc() >= 1 ) {
                CMD_get_one_word( &word, &wordlen );
                cnt = CMD_to_number ( word, wordlen );
            } else {
                cnt = 0;
            }
            addr =  QSpi1Handle.geometry.ProgPageSize * num;
            for ( i=0; i <=cnt; i++ ) {
                printf("compare page %d (startaddr=0x%08x) - Read ", num+i, addr );
                #if defined(QSPI1_USE_IRQ)
                    ret = QSpi_ReadIT(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
                #else
                    ret = QSpi_ReadWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
                #endif
                printf ( "%s\n", ret ? "ok": "fail");
                #if defined(QSPI1_USE_IRQ)
                    DEBUG_PUTS("Waiting for Async op to be done");
                    uint32_t start = HAL_GetTick();
                    while ( QSpi1Handle.bAsyncBusy );
                    DEBUG_PRINTF("Wait terminated after %d ticks\n", HAL_GetTick() - start);
                #endif
                for ( j  = 0; j < QSpi1Handle.geometry.ProgPageSize; j++ ) {
                    if ( PageBuffer[j] != (uint8_t)(i+j) ) {
                        printf("Compare failed Page offset 0x%02x: read 0x%02x, expected 0x%02x\n", j, PageBuffer[j], i+j);
                        break;
                    }
                }
                addr += QSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 4:
            if ( QSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", QSpi1Handle.geometry.ProgPageSize);
                return false;
            }
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Dump <page#> [<cnt>] - write page  +<cnt> following\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( CMD_argc() >= 1 ) {
                CMD_get_one_word( &word, &wordlen );
                cnt = CMD_to_number ( word, wordlen );
            } else {
                cnt = 0;
            }
            addr =  QSpi1Handle.geometry.ProgPageSize * num;
            if ( QSpi1Handle.bIsMemoryMapped ) {
            } else {
                
            }
            for ( i=0; i <=cnt; i++ ) {
                printf("Read page %d (startaddr=0x%08x) in %s mode - ", num+i, addr,  QSpi1Handle.bIsMemoryMapped ? "MemoryMapped" : "QSPI" );
                if ( QSpi1Handle.bIsMemoryMapped ) {
                    memmove(PageBuffer,(void *)(QSPI_BASE+addr), QSpi1Handle.geometry.ProgPageSize);
                    /*
                    for ( j  = 0; j < QSpi1Handle.geometry.ProgPageSize; j++ ) {
                        PageBuffer[j] = *((uint8_t *)(QSPI_BASE+addr+j));
                    }
                    */
                    ret = true;
                } else {
                    ret = QSpi_ReadWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
                }
                printf ( "%s\n", ret ? "ok": "fail");
                for ( j  = 0; j < QSpi1Handle.geometry.ProgPageSize; j++ ) {
                    if ( j % 16 == 0 ) printf ( "0x%08x",addr+j );
                    printf(" %02x", PageBuffer[j]);
                    if ( (j+1) % 16 == 0 ) puts ("");
                }
                addr += QSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 5:
            printf("Enable memory mapped mode- ");
            ret = QSpi_EnableMemoryMappedMode(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 6:
            printf("Abort operation - ");
            ret = QSpi_Abort(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 7:
            printf("Read Status");
            QSpi_DumpStatus(&QSpi1Handle);
            break;
        case 8:
            printf("Enter power down - ");
            ret = QSpi_EnterDeepPowerDown(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 9:
            printf("Exit power down - ");
            ret = QSpi_LeaveDeepPowerDown(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 10:
        case 11:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Write <addr> <size> - write <size> bytes, beginning @ <addr>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            cnt = CMD_to_number ( word, wordlen );
            if ( cnt > PGSIZE ) {
                printf("maximum size is %d\n",PGSIZE);
                return false;
            }
            for ( i=0; i <=cnt; i++ ) {
               PageBuffer[i]=i;
            }
            printf("Write %d bytes from addr 0x%08x\n", cnt, addr );
             
            if ( (uint32_t)arg == 11 )
                ret = QSpi_WriteDMA(&QSpi1Handle, PageBuffer, addr, cnt );
            else
                ret = QSpi_WriteIT(&QSpi1Handle, PageBuffer, addr, cnt );
            printf ( "%s\n", ret ? "ok": "fail");

            DEBUG_PRINTTS("Waiting for Async op to be done\n");
            break;
        default:
          DEBUG_PUTS("PM_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtQSPI (void)
    {
      return "QuadSpi";
    }


    static const CommandSetT cmdQSPI[] = {
        { "Geometry",                 ctype_fn, .exec.fn = QSPI_Menu, VOID(0), "Get Qspi Flash geometry" },
        { "Erase <sector#> [<cnt>]",  ctype_fn, .exec.fn = QSPI_Menu, VOID(1), "Erase Sector# and <cnt> following sectors" },
        { "Write <page#> [<cnt>]",    ctype_fn, .exec.fn = QSPI_Menu, VOID(2), "Write page #num and <cnt> following pages" },
        { "Compare <page#> [<cnt>]",  ctype_fn, .exec.fn = QSPI_Menu, VOID(3), "Read page #num and <cnt> following pages" },
        { "Dump <page#> [<cnt>]",     ctype_fn, .exec.fn = QSPI_Menu, VOID(4), "Read page #num and <cnt> following pages" },
        { "Memory Mapped",            ctype_fn, .exec.fn = QSPI_Menu, VOID(5), "Enable Memory mapped mode" },
        { "Abort",                    ctype_fn, .exec.fn = QSPI_Menu, VOID(6), "Abort current operation" },
        { "Flash status",             ctype_fn, .exec.fn = QSPI_Menu, VOID(7), "Dump flash chip status" },
        { "Power Down",               ctype_fn, .exec.fn = QSPI_Menu, VOID(8), "Enable Power down mode" },
        { "End Power Down",           ctype_fn, .exec.fn = QSPI_Menu, VOID(9), "Exit Power down mode" },
        { "Write much IT",            ctype_fn, .exec.fn = QSPI_Menu, VOID(10), "Write many bytes IRQ mode" },
        { "Write much DMA",           ctype_fn, .exec.fn = QSPI_Menu, VOID(11),"Write many bytes DMA mode" },
    };
    ADD_SUBMODULE(QSPI);
#endif


#include "eeprom.h"

/*********************************************************************************
  * @brief  Show/Alter the persistent EEPROM settings
  *         
  * @retval true on success, false otherwise
  ********************************************************************************/
bool Settings(char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline); UNUSED(len); UNUSED(arg);
  char *word;
  size_t wordlen;
  uint32_t argnum = CMD_argc();
  uint8_t idx;
  uint8_t val;
  bool result;

  if ( argnum >= 2 ) {
      while ( argnum > 1 ) {
        CMD_get_one_word( &word, &wordlen );
        idx = CMD_to_number ( word, wordlen );
        CMD_get_one_word( &word, &wordlen );
        val = CMD_to_number ( word, wordlen );
        result = Config_SetVal(idx, val); 
        DEBUG_PRINTF("%s No. %d to 0x%02x\n", result? "Set" : "Failed to set", idx, val );
        argnum -= 2;
      }
  }
  
  /* Show all settings */
  Config_Dump();

  return true;
}
/*********************************************************************************
  * @brief  MainMenu
  *         
  * @retval true on success, false otherwise
  ********************************************************************************/
static bool MainMenu(char *cmdline, size_t len, const void * arg )
{
    char *word;
    size_t wordlen;

    UNUSED(cmdline); UNUSED(len);
    switch((uint32_t)arg) {
#if DEBUG_MODE > 0
        case 0:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Level [<n>] - Show/Set Debuglevel\n");
            } else {
                CMD_get_one_word( &word, &wordlen );
                debuglevel = CMD_to_number ( word, wordlen );
            }
            printf("Debuglevel=%d\n", debuglevel);
            break;
#endif
        default:
            DEBUG_PUTS("MainMenu: command not implemented");
            return false;
    }
    return true;
} 
static const char *pmtBasic (void)
{
  return "";
}


static const CommandSetT cmdBasic[] = {
  { "Settings",        ctype_fn,  .exec.fn = Settings,        VOID(0),  "Persistent settings"  },
#if DEBUG_MODE > 0
  { "Level",           ctype_fn,  .exec.fn = MainMenu,        VOID(0),  "Set Debuglevel"  },
#endif
  { "UID",             ctype_fn,  .exec.fn = HW_DumpID,       VOID(0),  "Print Hardware UID"  },
#if DEBUG_FEATURES > 0 
  { "Clock&Pwr",       ctype_sub, .exec.sub = &mdlClkCfg,      0,       "Clock & Power Config submenu" },
  { "Devices",         ctype_sub, .exec.sub = &mdlDevices,     0,       "Peripheral devices submenu" },
#endif
#if USE_I2C > 0
  { "I2C"    ,         ctype_sub, .exec.sub = &mdlI2C,         0,       "I2C submenu" },
#endif
#if DEBUG_PROFILING > 0
  { "Profile",         ctype_fn,  .exec.fn = ProfilerDump,     VOID(0), "Dump profiling info" },
#endif
  { "Test&System",     ctype_sub, .exec.sub = &mdlTest,        0,       "Test & System submenu" },
#if USE_QSPI > 0
  { "QSpi test",       ctype_sub, .exec.sub = &mdlQSPI,        0,       "Test QuadSpi functions" },
#endif
};
ADD_SUBMODULE(Basic);


/**
  * @}
  */
