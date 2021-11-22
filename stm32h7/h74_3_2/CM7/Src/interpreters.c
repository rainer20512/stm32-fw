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

#include "system/clockconfig.h"
#include "config/devices_config.h"
#include "version.h"
#include "interpreters.h"
#include "debug_helper.h"
#include "system/profiling.h"
//#include "system/util.h"
//#include "wireless.h"
#include "timer.h"
//#include "rfm/rfm_packets.h"
#include "task/minitask.h"
//#include "com.h"
#include "rtc.h"
#include "log.h"
#include "dev/i2c_dev.h"
//#include "system/status.h"
#include "system/hw_util.h"
#include "cmdline.h"
//#include "sensors/thp_sensor.h"
//#include "eeprom.h"
#include "dev/devices.h"

#if USE_DS18X20  > 0
    #include "onewire.h"
    #include "ds18xxx20.h"
#endif

#if USE_QENCODER
    #include "dev/qencode.h"
#endif
#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
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

void SystemClock_Set(CLK_CONFIG_T clk_config_byte, bool bSwitchOffMSI );

 
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
      DBG_dump_clocksetting();
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
    case 7:
      if ( CMD_argc() < 1 ) {
          printf("Usage: 'SetSysClk nn \n");
          return false;
      }
      CMD_get_one_word( &word, &wordlen );
      ret = CMD_to_number ( word, wordlen );
      SystemClock_Set(ret, true);
      break;
    case 8:
      DEBUG_PUTS("Calibrating HSI clock");
      HSIClockCalibrate();
      break;
    case 9:
      if ( CMD_argc() < 1 ) {
          printf("Usage:MCO output <n>, 0=Off, 1=HSI, 2=LSE, 3=HSE, 4=PLL1Q, 5=HSI48");
          return false;
      }
      CMD_get_one_word( &word, &wordlen );
      ret = CMD_to_number ( word, wordlen );
      DEBUG_PRINTF("MCO Source %d\n", ret );
      EnableMCO ( ret );
      break;
    default:
      DEBUG_PUTS("RFM_Menu: command not implemented");
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
  { "DMA settng",ctype_fn, {Config_Menu},  VOID(6), "Show DAM settings"  },
  { "SetSysClk", ctype_fn, {Config_Menu},  VOID(7), "Set system clock to predefined settings"  },
  { "Calib-HSI", ctype_fn, {Config_Menu},  VOID(8), "Calibrate HSI Clock"  },
  { "MCO output",ctype_fn, {Config_Menu},  VOID(9), "Activate MCO Output PA8" },
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


#if USE_I2C > 0
    #if USE_THPSENSOR > 0
        /*********************************************************************************
          * @brief  Submenu for Sensor access functions
          *         
          * @retval true on success, false otherwise
          *
          * @note   will try to read as many parameters as needed
          ********************************************************************************/
        static bool Sensor_Menu ( char *cmdline, size_t len, const void * arg )
        {
          const THPSENSOR_DecisTypeDef Init = {-1,-1,-1};
          int32_t work;
          UNUSED(cmdline);UNUSED(len);
          switch((uint32_t)arg) {
            case 0:
              DEBUG_PUTS("Sensor init...");  
              THPSENSOR_Init(&Init);
              break;
            case 1:
              DEBUG_PUTS("Measure pressure...");  
              THPSENSOR_Measure(THPSENSOR_HAS_P);
              break;
            case 2:
              DEBUG_PUTS("Measure all...");  
              THPSENSOR_Measure(ALL_SENSOR_CHANNELS);
              break;
            case 3:
              break;
            case 4:
              break;
            case 99:
                DEBUG_PUTS("All sensor data:");  
                if ( THPSENSOR_HAS_T & THPSENSOR_GetCapability() ) {
                    work = THPSENSOR_GetT();
                    DEBUG_PRINTF("Temp=%d.%1d\n",work/10, work%10);
                }
                if ( THPSENSOR_HAS_H & THPSENSOR_GetCapability() ) {
                    work = THPSENSOR_GetH();
                    DEBUG_PRINTF("Hum= %d.%1d\n",work/10, work%10);
                }
                if ( THPSENSOR_HAS_P & THPSENSOR_GetCapability() ) {
                    work = THPSENSOR_GetP();
                    DEBUG_PRINTF("Pres=%d.%1d\n",work/10, work%10);
                }
                break;
            default:
              DEBUG_PUTS("Sensor_Menu: command not implemented");
          } /* end switch */

          return true;

    } /* Sensor_Menu */
    #endif

#include "ext_eeprom.h"
  #define MAX_BUFSIZE   8192
  static uint8_t buffer[MAX_BUFSIZE];
  static EEPROM_InitTypeDef t = {0};
  extern EEPROM_DrvTypeDef FM24V10_EEPROM;
  static EEPROM_DrvTypeDef *e = &FM24V10_EEPROM;
/*********************************************************************************
  * @brief  Submenu for FM24 Access functions
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
static bool FM24_Menu ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);
  uint16_t i;
  uint8_t actlen;
  uint32_t addr;
  uint16_t size;
  uint32_t tstart, tend;
  char *word;
  size_t wordlen;
  uint32_t ret;

  switch((uint32_t)arg) {
    case 0:
      if (e->Init(FM24V10_I2C_ADDR, &t) != EEPROM_OK ) {
        puts("EEPROM Init failed");
      }
      break;
    case 1:
      if ( e->GetID(buffer, &actlen, 255 ) != EEPROM_OK ) {
        puts("EEPROM Read ID failed");
      } else {
          for ( i=0; i< actlen; i++) 
             printf("%02x", buffer[i]);
          printf("\n");
      }
      break;
    case 2:
      if ( CMD_argc() < 2 ) {
          printf("Usage: 'Write <addr> <numBtyes> \n");
          return false;
      }
     CMD_get_one_word( &word, &wordlen );
     addr = CMD_to_number ( word, wordlen );
     CMD_get_one_word( &word, &wordlen );
     size = CMD_to_number ( word, wordlen );
     if ( size > MAX_BUFSIZE ) size = MAX_BUFSIZE;

      for(i=0; i <size; i++ ) buffer[i] = (uint8_t)i; 
      tstart = ProfilerGetMicrosecond();
      ret = e->WriteBuffer(addr, buffer, size);
      tend = ProfilerGetMicrosecond();
      if ( ret != EEPROM_OK ) {
        puts("EEPROM Write failed");
      } else {
        printf("Exected in %d microsecs\n", tend-tstart);
      }
      break;
    case 3:
      if ( CMD_argc() < 2 ) {
          printf("Usage: 'Read <addr> <numBtyes> \n");
          return false;
      }
     CMD_get_one_word( &word, &wordlen );
     addr = CMD_to_number ( word, wordlen );
     CMD_get_one_word( &word, &wordlen );
     size = CMD_to_number ( word, wordlen );
     if ( size > MAX_BUFSIZE ) size = MAX_BUFSIZE;
      if ( e->ReadBuffer(addr, buffer, size ) != EEPROM_OK ) {
        puts("EEPROM Read failed");
      }
      for ( i=0; i< size; i++) 
         printf("%02x ", buffer[i]);
      printf("\n");
      break;
    case 4:
      if ( CMD_argc() < 3 ) {
          printf("Usage: 'Fill <addr> <numBtyes> <fillbyte>\n");
          return false;
      }
     CMD_get_one_word( &word, &wordlen );
     addr = CMD_to_number ( word, wordlen );
     CMD_get_one_word( &word, &wordlen );
     size = CMD_to_number ( word, wordlen );
     if ( size > MAX_BUFSIZE ) size = MAX_BUFSIZE;
     CMD_get_one_word( &word, &wordlen );
     actlen = CMD_to_number ( word, wordlen );

      for(i=0; i <size; i++ ) buffer[i] = (uint8_t)actlen;  
      if ( e->WriteBuffer(addr, buffer, size ) != EEPROM_OK ) {
        puts("EEPROM  Fill failed");
      }
      break;
    case 5:
      if ( CMD_argc() < 2 ) {
          printf("Usage: 'WriteIT <addr> <numBtyes> \n");
          return false;
      }
     CMD_get_one_word( &word, &wordlen );
     addr = CMD_to_number ( word, wordlen );
     CMD_get_one_word( &word, &wordlen );
     size = CMD_to_number ( word, wordlen );
     if ( size > MAX_BUFSIZE ) size = MAX_BUFSIZE;

      for(i=0; i <size; i++ ) buffer[i] = (uint8_t)i; 
      tstart = ProfilerGetMicrosecond();
      ret = e->WriteBufferIT(addr, buffer, size);
      tend = ProfilerGetMicrosecond();
      if ( ret != EEPROM_OK ) {
        puts("EEPROM Write failed");
      } else {
        printf("Exected in %d microsecs\n", tend-tstart);
      }
      break;
    case 6:
      if ( CMD_argc() < 2 ) {
          printf("Usage: 'WriteDMA <addr> <numBtyes> \n");
          return false;
      }
     CMD_get_one_word( &word, &wordlen );
     addr = CMD_to_number ( word, wordlen );
     CMD_get_one_word( &word, &wordlen );
     size = CMD_to_number ( word, wordlen );
     if ( size > MAX_BUFSIZE ) size = MAX_BUFSIZE;

      for(i=0; i <size; i++ ) buffer[i] = (uint8_t)i; 
      tstart = ProfilerGetMicrosecond();
      ret = e->WriteBufferDMA(addr, buffer, size);
      tend = ProfilerGetMicrosecond();
      if ( ret != EEPROM_OK ) {
        puts("EEPROM Write failed");
      } else {
        printf("Exected in %d microsecs\n", tend-tstart);
      }
      break;
    default:
      DEBUG_PUTS("Test_Menu: command not implemented");
  } /* end switch */

  return true;

} /* FM24_Menu */
#endif

#if USE_I2C > 0
    static const char *pmtI2C (void)
    {
      return "I2C";
    }

    static const CommandSetT cmdI2C[] = {
      { "Scan"          ,ctype_fn, .exec.fn = Scan_I2c,     VOID(0), "Scan I2C2 bus" },
      #if USE_THPSENSOR > 0
          { "Sensor Init"    ,ctype_fn, .exec.fn = Sensor_Menu,  VOID(0),  "Sensor Init" },
          { "Meas Pressure"  ,ctype_fn, .exec.fn = Sensor_Menu,  VOID(1),  "Trigger Pressure Measure" },
          { "Meas Pressure"  ,ctype_fn, .exec.fn = Sensor_Menu,  VOID(2),  "Measure all channels" },
          { "Display Data"   ,ctype_fn, .exec.fn = Sensor_Menu,  VOID(99), "Display all supported data" },
      #endif
      { "FM24V10 Init"  ,ctype_fn, .exec.fn =FM24_Menu,     VOID(0), "FM24V10 Init" },
      { "FM24V10 Read ID",ctype_fn, .exec.fn =FM24_Menu,  VOID(1), "FM24V10 Read ID" },
      { "FM24V10 Write" ,ctype_fn, .exec.fn =FM24_Menu,   VOID(2), "FM24V10 Write <addr> <len>" },
      { "FM24V10 Read"  ,ctype_fn, .exec.fn =FM24_Menu,   VOID(3), "FM24V10 Read  <addr> <len>" },
      { "FM24V10 Fill"  ,ctype_fn, .exec.fn =FM24_Menu,   VOID(4), "FM24V10 Fill <addr> <len> <byte>" },
      { "FM24V10 WrIT"  ,ctype_fn, .exec.fn =FM24_Menu,   VOID(5), "FM24V10 WriteIT <addr> <len>" },
      { "FM24V10 WrDMA" ,ctype_fn, .exec.fn =FM24_Menu,   VOID(6), "FM24V10 WriteDMA <addr> <len>" },

    };
    ADD_SUBMODULE(I2C);
#endif


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
  #if USE_QENCODER > 0
      static void TestOnRotate(int32_t delta )
      {
          DEBUG_PRINTF("TestOnRotate, Delta=%d\n", delta);
      }
      static void TestOnClick(void)
      {
          COM_print_time('t',false);DEBUG_PUTS(" TestOnClick");
      }
      static void TestOnDblClick(void)
      {
          COM_print_time('T',false);DEBUG_PUTS(" TestOnDblClick");
      }
  #endif
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

static void TmrExpired(uint32_t tickstart)
{
    uint32_t tickdiff = HAL_GetTick() - tickstart;
    DEBUG_PRINTF("%d ticks during wait\n", tickdiff);
}

/*********************************************************************************
  * @brief  Submenu for system functions
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
static bool System_Menu ( char *cmdline, size_t len, const void * arg )
{
  #define MAX_SPI 40  
  char *word;
  size_t wordlen;
  uint32_t num;
  UNUSED(cmdline);UNUSED(len);
  uint8_t spi_in[MAX_SPI],spi_out[MAX_SPI];

  switch((uint32_t)arg) {
    case 0:
        TaskDumpList();
        break;
    case 1:
        PeriodicDumpList();
        break;
    case 2:
        if ( CMD_argc() < 1 ) {
            printf("Usage: 'SysTickTest <n> : Wait <n> seconds \n");
            return false;
        }
        CMD_get_one_word( &word, &wordlen );
        num = CMD_to_number ( word, wordlen );
        if ( SecTimerSetRel(num,false, TmrExpired, HAL_GetTick() ) == NO_TIMER_ID ) {
            DEBUG_PUTS("Cannot allocate timer");
        } else {
            DEBUG_PRINTF("Waiting %d seconds...\n", num);
        }
        break;
#if USE_ETH > 0 && USE_ETH_PHY_ENC28J60 > 0
    case 3:
        ethernetif_statistic();
        break;
    case 4:
        DEBUG_PUTS("Reset Ethernet IF hardware");
        ethernetif_restart();
        break;
    case 5:
        if ( CMD_argc() > 1 ) {
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( num > MAX_SPI ) num = MAX_SPI;
        } else {
            num = 2;
        }
        num = ENC_Test();
        DEBUG_PRINTF("PHSTAT2=%02x\n", num);
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

  { "TmrAbs",          ctype_fn, .exec.fn = Test_TmrAbs,    VOID(0), "Set Abs Timer" },
  { "TmrRel",          ctype_fn, .exec.fn = Test_TmrRel,    VOID(0), "Set Rel Timer" },
  { "TmrDel",          ctype_fn, .exec.fn = Test_TmrDel,    VOID(0), "Delete priodic timer" },
  { "Multi Tmr",       ctype_fn, .exec.fn = Test_TmrMulti,  VOID(0), "Lots of timers" }, 
  { "Kill Tmr",        ctype_fn, .exec.fn = Test_TmrKill,   VOID(0), "Generates Error" }, 
  { "Toggle",          ctype_fn, .exec.fn = Test_Toggle,    VOID(0), "Toggle UserLed2" },
  { "SysTickTest",     ctype_fn, .exec.fn = System_Menu,    VOID(2), "Check Systick" },

#if defined(USE_USART2)
  { "USART2 Out",      ctype_fn, .exec.fn = Test_Menu,      VOID(5), "Output via USART2" },
#endif
#if USE_QENCODER > 0
  { "QEncoder test",   ctype_fn, .exec.fn = Test_Menu,      VOID(2), "Test quadrature encoder"}, 
#endif
#if defined(USE_LPUART1) 
  { "LPUART1 test",     ctype_fn, .exec.fn = Test_Menu,     VOID(4), "Test LPUART1"}, 
#endif
#if USE_EEPROM_EMUL > 0
  { "Write config ",   ctype_fn, .exec.fn = Test_Menu,      VOID(3), "Write config[31] x times"}, 
#endif
#if defined(USE_ETH)
  { "ETH IF statistic",ctype_fn, .exec.fn = System_Menu,    VOID(3), "Show ETH Interface stats"},   
  { "ETH IF reset",    ctype_fn, .exec.fn = System_Menu,    VOID(4), "Reset ETH Interface"},   
  { "ETH SPI Test",    ctype_fn, .exec.fn = System_Menu,    VOID(5), "Test SPI Interface to ETH"},   
#endif

};
ADD_SUBMODULE(Test);

#if defined(USE_ADC3)
    /*********************************************************************************
      * @brief  Submenu for FM24 Access functions
      *         
      * @retval true on success, false otherwise
      *
      * @note   will try to read as many parameters as needed
      ********************************************************************************/
    static bool ADC_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint8_t ch;

      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            ADC_Calibrate(&HW_ADC3);
            /* fall through */
        case 1:
            ADC_MeasureVdda ((void *)&HW_ADC3);
            printf("Vdda = %dmV\n", ADC3Handle.vdda); 
            break;
        case 2:
            ADC_MeasureChipTemp(&HW_ADC3);
            printf("Chip Temp = %d°C\n", ADC3Handle.chiptemp); 
            break;
        case 3:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'ADC channel <num> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            ADC_MeasureChannel(&HW_ADC3, ch);
            break;
        case 4:
            puts("Measure group"); 
            ADC_SetupGroup(&HW_ADC3);
            ADC_MeasureGroup(&HW_ADC3);
            break;
        case 5:
            ADC_DisableRefintCh(&HW_ADC3);
            puts("Refint channel disabled"); 
            break;
        case 6:
            ADC_DisableAllInternalCh(&HW_ADC3);
            puts("Refint channel disabled"); 
            break;
        default:
          DEBUG_PUTS("Test_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtADC (void)
    {
      return "ADC";
    }


    static const CommandSetT cmdADC[] = {
        { "Measure Vdda+Calibrate", ctype_fn, .exec.fn = ADC_Menu,VOID(0), "Calibrate ADC & Measure Vdda" },
        { "Measure Vdda",           ctype_fn, .exec.fn = ADC_Menu,VOID(1), "Measure Vdda w/o Calibration" },
        { "Measure Temp",           ctype_fn, .exec.fn = ADC_Menu,VOID(2), "Measure Chip temperature" },
        { "Measure channel",        ctype_fn, .exec.fn = ADC_Menu,VOID(3), "Measure channel <n>" },
        { "Measure group",          ctype_fn, .exec.fn = ADC_Menu,VOID(4), "Measure whole sequence" },
        { "Refint disable",         ctype_fn, .exec.fn = ADC_Menu,VOID(5), "Disable Refint ADC channel" },
        { "All int. Ch. Disable",   ctype_fn, .exec.fn = ADC_Menu,VOID(6), "Disable all internal ADC channels" },
    };
    ADD_SUBMODULE(ADC);
#endif



#if USE_DS18X20 > 0
    /*********************************************************************************
      * @brief  Submenu for DS18X20/OneWire Access 
      *         
      * @retval true on success, false otherwise
      *
      * @note  will try to read as many parameters as needed
      ********************************************************************************/
    static bool DS_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint8_t ch;
      int16_t t;
      
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            puts("Perform a OneWire ROM Search");
            if ( ow_rom_search() == 0 ) {
                    DEBUG_PUTS("No OneWire devices found!");
            } else {
                ow_dump_rom();
                DS18X20_Init();
            }
            break;
        case 1:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Measure  0|1 (one sensor/all sensors)\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            DS18X20_start_meas(ch != 0);
            break;
        case 2:
            puts("DS18X20 read BitStatus");
            DS18X20_read_bitstatus();
            break;
        case 3:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Read SP  0|1 (one sensor/all sensors)\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            DS18X20_read_scratchpad(ch != 0);
            break;
        case 4:
            t = DS18X20_GetTemp();
            printf("Temp=%d.%ddegC\n",t/10,t%10);
            break;
        case 5:
            printf("Successful=%d, BadVal=%d, Errors=%d\n", ds_ok_cnt, ds_bad_cnt, ds_err_cnt );
            break;
#if OW_TEST > 0
        case 6:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Test <vector len>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            test_write_buffer_with_cb(ch);
            break;
#endif
        default:
          DEBUG_PUTS("DS_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmt18X20 (void)
    {
      return "DS18X20";
    }


    static const CommandSetT cmd18X20[] = {
        { "ROM Search",             ctype_fn, .exec.fn = DS_Menu,VOID(0), "Perform a OW ROM search" },
        { "Measure Temp",           ctype_fn, .exec.fn = DS_Menu,VOID(1), "Initiate temp measurement" },
        { "Read Bitstatus",         ctype_fn, .exec.fn = DS_Menu,VOID(2), "Read Bitstatus" },
        { "Read Scratchpad",        ctype_fn, .exec.fn = DS_Menu,VOID(3), "Read scratchpad" },
        { "Get Temperature",        ctype_fn, .exec.fn = DS_Menu,VOID(4), "Get last temperature" },
        { "Get Stat. Count",        ctype_fn, .exec.fn = DS_Menu,VOID(5), "Show Status Counter" },
#if OW_TEST > 0
        { "Test U(S)ART",           ctype_fn, .exec.fn = DS_Menu,VOID(6), "Loopback test" },
#endif
    };
    ADD_SUBMODULE(18X20);
#endif

#if USE_PWMTIMER > 0
    /*********************************************************************************
      * @brief  Submenu for PWM 
      *         
      * @retval true on success, false otherwise
      *
      * @note  will try to read as many parameters as needed
      ********************************************************************************/
    static bool PWM_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t ch;
      bool ret;
      
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Frequency  <frq>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            ret = TMR_SetPWMFrq(&HW_PWMTIMER, num );
            DEBUG_PRINTF("Setting PWM frq to %d - %s\n", num, ret ? "ok" : "fail");
            break;
        case 1:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Init <ch> <mode>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            ret = TMR_InitPWMCh(&HW_PWMTIMER, ch, num!=0); 
            DEBUG_PRINTF("Init PWM ch %d - %s\n", ch, ret ? "ok" : "fail");
            break;
        case 2:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Start Promille <ch> <promille>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            TMR_StartPWMChPromille (&HW_PWMTIMER, ch, num); 
            DEBUG_PRINTF("Start PWM ch %d w. %d/1000\n", ch, num);
            break;
        case 3:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Start S256 <ch> <s256>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            TMR_StartPWMChS256(&HW_PWMTIMER, ch, num); 
            DEBUG_PRINTF("Start PWM ch %d w. %d/256\n", ch, num);
            break;
        case 4:
            if ( CMD_argc() < 1) {
              printf("Usage: 'Stop PWM <ch>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            TMR_StopPWMCh(&HW_PWMTIMER, ch); 
            DEBUG_PRINTF("Stop PWM ch %d\n", ch);
            break;
        default:
          DEBUG_PUTS("PM_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtPWM (void)
    {
      return "PWM";
    }


    static const CommandSetT cmdPWM[] = {
        { "Frequency",              ctype_fn, .exec.fn = PWM_Menu,VOID(0), "Set PWM frequency" },
        { "Init PWM Ch",            ctype_fn, .exec.fn = PWM_Menu,VOID(1), "Init PWM Ch <x> to normal/invert (0|1)" },
        { "Start Promille",         ctype_fn, .exec.fn = PWM_Menu,VOID(2), "Start PWM Ch <x> w. duty cycle of <y>/1000" },
        { "Start S256",             ctype_fn, .exec.fn = PWM_Menu,VOID(3), "Start PWM Ch <x> w. duty cycle of <y>/256" },
        { "Stop PWM ch",            ctype_fn, .exec.fn = PWM_Menu,VOID(4), "Stop PWM ch <x>" },
    };
    ADD_SUBMODULE(PWM);
#endif

#if USE_QSPI > 0

    #include "dev/xspi_dev.h"
    #include "dev/xspi/xspi_specific.h"
     

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

    void rddoneCB(XSpiHandleT *hnd )
    {
        UNUSED(hnd);
        DEBUG_PRINTTS("read terminated ok\n");
    }
    void wrdoneCB(XSpiHandleT *hnd )
    {
        UNUSED(hnd);
        DEBUG_PRINTTS("write/erase terminated ok\n");
    }
    void errorCB(XSpiHandleT *hnd )
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
      XSpi_SetAsyncCallbacks(&QSpi1Handle, rddoneCB, wrdoneCB, errorCB);
#endif

      switch((uint32_t)arg) {
        case 0:
            printf("Flash Size= ...... %d kByte\n", QSpi1Handle.geometry.FlashSize/1024);
            printf("Erase sector size= %d Byte\n", QSpi1Handle.geometry.EraseSectorSize);
            printf("   sector count= . %d\n", QSpi1Handle.geometry.EraseSectorsNumber);
            printf("Write page size= . %d Byte\n", QSpi1Handle.geometry.ProgPageSize);
            printf("   page count= ... %d\n", QSpi1Handle.geometry.ProgPagesNumber);
            printf("pages per sector=  %d\n", QSpi1Handle.geometry.EraseSectorSize/QSpi1Handle.geometry.ProgPageSize);   
            printf("Qspi Clock= ...... %d kHz\n", QSpi1Handle.clkspeed / 1000 );
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
            addr =  QSpi1Handle.geometry.ProgPageSize * num;
            printf("Erase sector (%d Bytes) with page %d (startaddr=0x%08x)", 
                    QSpi1Handle.geometry.EraseSectorSize, num, addr );
            ret = XSpi_EraseSectorWait(&QSpi1Handle, addr, cnt+1);
            printf ( "%s\n", ret ? "ok": "fail");
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
                ret = XSpi_WriteWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize );
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
                    ret = XSpi_ReadIT(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
                #else
                    ret = XSpi_ReadWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
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
                    ret = XSpi_ReadWait(&QSpi1Handle, PageBuffer, addr, QSpi1Handle.geometry.ProgPageSize);
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
            ret = XSpecific_EnableMemoryMappedMode(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 6:
            printf("Abort operation - ");
            ret = XSpi_Abort(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 7:
            printf("Read Status");
            XSpi_DumpStatus(&QSpi1Handle);
            break;
        case 8:
            printf("Enter power down - ");
            ret = XSpecific_EnterDeepPowerDown(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 9:
            printf("Exit power down - ");
            ret = XSpecific_LeaveDeepPowerDown(&QSpi1Handle);
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
                ret = XSpi_WriteDMA(&QSpi1Handle, PageBuffer, addr, cnt );
            else
                ret = XSpi_WriteIT(&QSpi1Handle, PageBuffer, addr, cnt );
            printf ( "%s\n", ret ? "ok": "fail");

            DEBUG_PRINTTS("Waiting for Async op to be done\n");
            break;
        case 12:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Clk speed <n> - Set Qspi clk speed to <n> kHz\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            printf("Set Qspi Clock speed to %d MHz",  addr );
             
                ret = XSpi_SetSpeed(&QSpi1Handle, addr * 1000 );
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 13:
            printf("QSPI chip reset - ");
            ret = XSpecific_ResetMemory(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 14:
            printf("QSPI chip erase - ");
            ret = XSpi_EraseChipWait(&QSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
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
        { "Clk speed <n>",            ctype_fn, .exec.fn = QSPI_Menu, VOID(12),"Set QSPI clock speed to <n> kHz" },
        { "Reset memory",             ctype_fn, .exec.fn = QSPI_Menu, VOID(13),"Reset QSPI Memory" },
        { "Chip erase",               ctype_fn, .exec.fn = QSPI_Menu, VOID(14),"Erase whole chip" },
    };
    ADD_SUBMODULE(QSPI);
#endif

#if USE_CAN > 0

    #include "dev/can_dev.h"
    /*********************************************************************************
     * @brief  Submenu for Can test functions
     *         
     * @retval true on success, false otherwise
     *
     * @note  will try to read as many parameters as needed
     ********************************************************************************/

    static void OnCanRx(uint32_t fifonum, CanTxRxDataT *rx)
    {
        DEBUG_PRINTF("Received by Fifo#%d with %s Id, Type=%s :\n", fifonum, rx->IDE ? "Extended" : "Standard", rx->RTR ? "Remote Req" : "Data");
        DEBUG_PRINTF("ID=0x%08x Len=%d, Data=[",rx->Id, rx->DLC);
        uint8_t *ptr = rx->data;
        for ( uint32_t i = 0; i < rx->DLC; i++) {
            DEBUG_PUTC(' ');print_hexXX(*(ptr++));
        }
        DEBUG_PUTS("]");
    }

    uint32_t ID;
    CanTxRxDataT tx;
    uint8_t txdata[8];
    static void Can_Transmit(uint32_t num, uint32_t size ) {
        uint32_t ticks;
        CanTxMboxNum mbxnum;
        if ( size > 8 ) size = 8;
        tx.DLC = size;
        tx.IDE = ( ID > 0x7FF );
        tx.Id  = ID;
        tx.RTR = 0;
        tx.data = txdata;
        for ( uint32_t i=0; i < num; i++ ) {
            for ( uint32_t j = 0; j<size; j++ ) 
                txdata[j]='a'+i+j;
            ticks = HAL_GetTick();
            while(!CAN_IsTxMboxFree(&CAN1Handle) );
            DEBUG_PRINTF("Packet %2d - Waited %d ticks - ",i, HAL_GetTick() - ticks);
            if ( mbxnum=CAN_Transmit(&CAN1Handle, &tx), !num ) 
                DEBUG_PRINTF("Xmit failed\n");
            else
                DEBUG_PRINTF("Mbox#%d\n",mbxnum);
        }
    }


    static bool CAN_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t addr;
      CanFilterT flt;
      bool ret;
      
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            CAN_DumpCan(CAN1);
            break;
        case 1:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Baudrate <idx> - Set CAN baudrate\n");
              printf("       0-1000k, 1-500, 2-250, 3-200, 4=125, 5=100, 6=50, 7=20, 8=10\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            ret = CAN_ChangeBaudrate(&CAN1Handle, num);
            printf("Can baudrate change %s\n", ret ? "ok" : "failed");
            break;
        case 2:
            /* Match every Std Frame */
            CAN_Setup16BitFilter0( &flt, 0, 0, 0, 1, false, false, 0);
            /* Match every remote transm. Frame */
            CAN_Setup16BitFilter1( &flt, 0, 1, 0, 1, false, false, 0);
            // CAN_Setup16BitFilter1( &flt, addr+1, 0, 0x7FF, 1, false, false, 0);
            ret = CAN_SetFilter(CAN1, &flt, 13, true);
            puts("Setting filter 14 to match every data frame");
            break;
        case 3:
            if ( CMD_argc() < 2 ) {
              printf("Usage: Set Filter <no> to <id> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            CAN_Setup16BitFilter0( &flt, addr,   0, 0x7FF, 1, false, false, 0);
            CAN_Setup16BitFilter1( &flt, addr+1, 0, 0x7FF, 1, false, false, 0);
            ret = CAN_SetFilter(CAN1, &flt, num, true);
            printf("Setting filter %d to id 0x%03x %s\n", num, addr, ret ? "ok" : "failed");
            break;
        case 4:
            if ( CMD_argc() < 2 ) {
              printf("Usage: Set Filter <no> to <eid> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            CAN_Setup32BitFilter(&flt, addr, 0, 0XFFFFFFFF, 1, false, true, 1 );
            ret = CAN_SetFilter(CAN1, &flt, num, true);
            printf("Setting filter %d to id 0x%08x %s\n", num, addr, ret ? "ok" : "failed");
            break;
        case 5:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Tx ID <n> <m> : Set Tx msg ID to <n>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ID = CMD_to_number ( word, wordlen );
            break;
        case 6:
            if ( CMD_argc() < 2 ) {
              printf("Usage: Tx  <n> <m> : Transmit <n> msgs of len <m> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            printf("Transmitting %d msgs of size %d with ID 0x%x\n", num, addr, ID);
            Can_Transmit(num, addr);
            break;
        case 9:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Start/Stop [0|1] CAN \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            if ( num ) {
                CAN_RegisterCallbacks(&CAN1Handle, OnCanRx, NULL, NULL);
                ret = CAN_Start(&CAN1Handle);
            } else {
                CAN_RegisterCallbacks(&CAN1Handle, NULL, NULL, NULL);
                ret = CAN_Stop(&CAN1Handle);
            }
            DEBUG_PRINTF("CAN1 %s %s\n", num ? "started" : "stopped", ret ? "ok" : "with error");
            break;
        default:
          DEBUG_PUTS("CAN_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtCAN (void)
    {
      return "Can";
    }


    static const CommandSetT cmdCAN[] = {
        { "Dump",                     ctype_fn, .exec.fn = CAN_Menu, VOID(0), "Dump Can Settings" },
        { "Baudrate <idx>",           ctype_fn, .exec.fn = CAN_Menu, VOID(1), "Set Can Baudrate " },
        { "Set Filter @all",          ctype_fn, .exec.fn = CAN_Menu, VOID(2), "Set filter to match all" },
        { "Set Std Filter",           ctype_fn, .exec.fn = CAN_Menu, VOID(3), "Set 11 bit filter id" },
        { "Set Ext Filter",           ctype_fn, .exec.fn = CAN_Menu, VOID(4), "Set 28 bit filter id" },
        { "Set Tx ID",                ctype_fn, .exec.fn = CAN_Menu, VOID(5), "Set Msg ID for Xmit" },
        { "Tx <n> Len <m>",           ctype_fn, .exec.fn = CAN_Menu, VOID(6), "Tx <n> Msg of Len <m>" },
        { "Start/Stop CAN",           ctype_fn, .exec.fn = CAN_Menu, VOID(9), "Start(1) /Stop(0) CAN" },
    };
    ADD_SUBMODULE(CAN);
#endif

#if USE_FMC > 0

    #define BUFFER_SIZE_WORD    (0x8000UL)
    #define BUFFER_SIZE_BYTE    (BUFFER_SIZE_WORD*2)
    static uint32_t extmemBase = 0x68000000;
    static uint16_t AXISMEM aTxBuffer[BUFFER_SIZE_WORD];
    static uint16_t AXISMEM aRxBuffer[BUFFER_SIZE_WORD];
    uint32_t FMC_SdramCheck();

    /* Status variables */
    __IO bool uwWriteReadStatus = 0;

    /* Counter index */
    static uint32_t uwIndex = 0;

    static void FillBufferByte(uint8_t *pBuffer, uint32_t uwBufferLength, uint8_t ubValue)
    {
      /* Put in global buffer different values */
      for (uint32_t tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++)
        pBuffer[tmpIndex] = ubValue;
    }

    static void FillBufferWord(uint16_t *pBuffer, uint32_t uwBufferLength, uint16_t uwValue)
    {
      /* Put in global buffer different values */
      for (uint32_t tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++)
        pBuffer[tmpIndex] = uwValue;
      }

    static bool BufferCmpWord(uint16_t *pBuffer1, uint16_t *pBuffer2, uint32_t BufferLength)
    {
      uint32_t i = 0;
      while (i < BufferLength)
      {
        if (*pBuffer1 != *pBuffer2)
        {
          DEBUG_PRINTF("@0x%04x: Expected 0x%04x, got 0x%04x\n", i*2,*pBuffer1, *pBuffer2 );
          return false;
        }

        pBuffer1++;
        pBuffer2++;
        i++;
      }

      return true;
    }


    static void FMC_Sram_TestWord(uint32_t ofs, uint16_t pattern)
    {
        uint16_t *extmemptr = (uint16_t *)(extmemBase + ofs );

        
        /* Fill the buffer to write */
        FillBufferWord(aTxBuffer, BUFFER_SIZE_WORD, pattern);

        /* Write data to the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE_WORD; uwIndex++)
        {
        *(extmemptr+uwIndex) = aTxBuffer[uwIndex];
        }

        /* Read back data from the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE_WORD; uwIndex++)
        {
        aRxBuffer[uwIndex] = *(extmemptr+uwIndex);
        }


        /*##-3- Checking data integrity ############################################*/
        uwWriteReadStatus = BufferCmpWord(aTxBuffer, aRxBuffer, BUFFER_SIZE_WORD);

        if(!uwWriteReadStatus) {
            DEBUG_PRINTTS("Sram test @0x%08x failed\n",extmemBase + ofs );
        } else {
            DEBUG_PRINTTS("Sram test @0x%08x ok\n",extmemBase + ofs);

        }
    }

    static void FMC_Sram_TestByte(uint32_t ofs, uint8_t pattern)
    {
        uint8_t *extmemptr = (uint8_t *)(extmemBase + ofs );
        uint8_t *srcptr;
        
        /* Fill the buffer to write */
        FillBufferByte(aTxBuffer, BUFFER_SIZE_BYTE, pattern);

        srcptr    = (uint8_t *)aTxBuffer;
        /* Write data to the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE_BYTE; uwIndex++)
        {
            *(extmemptr+uwIndex) = *(srcptr++);
        }

        /* Read back data from the SRAM memory */
        uint8_t *destptr = (uint8_t *)aRxBuffer;
        for(uwIndex = 0; uwIndex < BUFFER_SIZE_BYTE; uwIndex++)
        {
            *(destptr++) = *(extmemptr+uwIndex);
        }


        /*##-3- Checking data integrity ############################################*/
        uwWriteReadStatus = BufferCmpWord(aTxBuffer, aRxBuffer, BUFFER_SIZE_WORD);

        if(!uwWriteReadStatus) {
            DEBUG_PRINTTS("Sram test @0x%08x failed\n",extmemBase + ofs );
        } else {
            DEBUG_PRINTTS("Sram test @0x%08x ok\n",extmemBase + ofs);

        }
    }

    #define PATTERN_1_32    (~0xDEADBEEF)
    #define PATTERN_2_32    0xDEADBEEF
    #define PATTERN_1_16    ((uint16_t)0xAC7E)
    #define PATTERN_2_16    ((uint16_t)(~PATTERN_1_16))
    #define PATTERN_1_8     ((uint8_t)0xA5)
    #define PATTERN_2_8     ((uint8_t)~PATTERN_1_8)

    static void loop32( uint32_t *mem, size_t len, uint32_t u0, uint32_t u1 )
    {
        register uint32_t *p = mem+2; 
        register uint32_t *p1 = mem+1;
        register uint32_t *p2 = mem;
        *p2=u0;
        *p1=u1;

        for ( register uint32_t i = 2; i < len; i++ ) {
            *(p++) = *(p1++) + *(p2++);
        }
    }
    static void loop16( uint16_t *mem, size_t len, uint16_t u0, uint16_t u1 )
    {
        register uint16_t *p = mem+2; 
        register uint16_t *p1 = mem+1;
        register uint16_t *p2 = mem;
        *p2=u0;
        *p1=u1;

        for ( register uint32_t i = 2; i < len; i++ ) {
            *(p++) = *(p1++) + *(p2++);
        }
    }

    static void loop8( uint8_t *mem, size_t len, uint8_t u0, uint8_t u1 )
    {
        register uint8_t *p = mem+2; 
        register uint8_t *p1 = mem+1;
        register uint8_t *p2 = mem;
        *p2=u0;
        *p1=u1;

        for ( register uint32_t i = 2; i < len; i++ ) {
            *(p++) = *(p1++) + *(p2++);
        }
    }

    static void wr32( uint32_t *mem, size_t len, uint32_t u0, uint32_t rep)
    {
        while ( rep-- ) {
            register uint32_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                *(p++) = u0;
            }
        }
    }
    static uint32_t rd32( uint32_t *mem, size_t len, uint32_t rep)
    {
        uint32_t ret;
        while ( rep-- ) {
            register uint32_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                ret = *(p++);
            }
        }
        return ret;
    }

    static void wr16( uint16_t *mem, size_t len, uint16_t u0, uint32_t rep)
    {
        while ( rep-- ) {
            register uint16_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                *(p++) = u0;
            }
        }
    }
    static uint16_t rd16( uint16_t *mem, size_t len, uint32_t rep)
    {
        uint16_t ret;
        while ( rep-- ) {
            register uint16_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                ret = *(p++);
            }
        }
        return ret;
    }

    static void wr8( uint8_t *mem, size_t len, uint8_t u0, uint32_t rep)
    {
        while ( rep-- ) {
            register uint8_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                *(p++) = u0;
            }
        }
    }
    static uint8_t rd8( uint8_t *mem, size_t len, uint32_t rep)
    {
        uint8_t ret;
        while ( rep-- ) {
            register uint8_t *p = mem; 
            for ( register uint32_t i = 0; i < len; i++ ) {
                ret = *(p++);
            }
        }
        return ret;
    }
    static void do_ram_test( uint32_t *mem, size_t len )
    {
        uint32_t tstart, tend;

        tstart = ProfilerGetMicrosecond();
        wr32(mem, len, 0xdeadbeef, 10);
        tend = ProfilerGetMicrosecond();
        printf("32bit write: %d microsecs\n", (tend-tstart)/10);
        tstart = ProfilerGetMicrosecond();
        rd32(mem, len, 10);
        tend = ProfilerGetMicrosecond();
        printf("32bit read : %d microsecs\n", (tend-tstart)/10);
    
        tstart = ProfilerGetMicrosecond();
        wr16((uint16_t*)mem, len, 0xdead, 10);
        tend = ProfilerGetMicrosecond();
        printf("16bit write: %d microsecs\n", (tend-tstart)/10);
        tstart = ProfilerGetMicrosecond();
        rd16((uint16_t*)mem, len, 10);
        tend = ProfilerGetMicrosecond();
        printf("16bit read : %d microsecs\n", (tend-tstart)/10);

        tstart = ProfilerGetMicrosecond();
        wr8((uint8_t*)mem, len, 0xde, 10);
        tend = ProfilerGetMicrosecond();
        printf("8bit write: %d microsecs\n", (tend-tstart)/10);
        tstart = ProfilerGetMicrosecond();
        rd8((uint8_t*)mem, len, 10);
        tend = ProfilerGetMicrosecond();
        printf("8bit read : %d microsecs\n", (tend-tstart)/10);
    }

    extern uint32_t psram[];
    extern uint32_t sdram[];
    extern uint32_t dtcm[];
    extern uint32_t axi[];
    extern uint32_t sram2[];
    extern uint32_t sram4[];
    void CPU_CACHE_Enable(void);
    void D_CACHE_Disable(void);

    static bool FMC_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t addr;
      
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            FMC_DumpGeometry();
            break;
        case 1:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Sram test <n> - test <n> blocks \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            for ( uint32_t i = 0; i < num; i++ ) {
                FMC_Sram_TestWord(i*BUFFER_SIZE_WORD, 0x0000);
                FMC_Sram_TestWord(i*BUFFER_SIZE_WORD, 0xFFFF);
            }
            break;
        case 2:
            if ( CMD_argc() < 2 ) {
              printf("Usage: Sram test <n> <pattern> - test <n> blocks w pattern <pattern> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            for ( uint32_t i = 0; i < num; i++ ) {
                FMC_Sram_TestWord(i*BUFFER_SIZE_WORD*2, (uint16_t)addr);
            }
            break;
        case 3:
            if ( CMD_argc() < 2 ) {
              printf("Usage: SramB test <n> <pattern> - test <n> blocks w BYTE pattern <pattern> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            for ( uint32_t i = 0; i < num; i++ ) {
                FMC_Sram_TestByte(i*BUFFER_SIZE_BYTE, (uint8_t)addr);
            }
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            #if 0
            if ( CMD_argc() < 1 ) {
              printf("Usage: Sram test <size> - test sram of size uint32_t[<size>] \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            #endif
            num = 16384;
            switch((uint32_t)arg) {
                case 4: do_ram_test( psram, num ); break;
                case 5: do_ram_test( dtcm, num ); break;
                case 6: do_ram_test( axi, num ); break;
                case 7: do_ram_test( sram2, num ); break;
                case 8: do_ram_test( sram4, num ); break;
                case 9: do_ram_test( sdram, num ); break;
            }
            break;
        case 90:
            num = FMC_SdramCheck();
            if ( num == 0 ) DEBUG_PUTS("SDRAM check ok");
            break;
        case 97:
            if ( CMD_argc() < 1 ) {
              printf("Usage: Base addr <n> - set Extmem base to <n>0000 hex \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            extmemBase = ((uint32_t)num) << 16;
            DEBUG_PRINTF("ExtMem base=%p\n",extmemBase);
            break;
        case 98:
            DEBUG_PUTS("D- and I-Cache enabled");
            CPU_CACHE_Enable();
            break;
        case 99:
            DEBUG_PUTS("D-Cache disabled");
            D_CACHE_Disable();
            break;
        default:
          DEBUG_PUTS("FMC Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtFMC (void)
    {
      return "Fmc";
    }


    static const CommandSetT cmdFMC[] = {
        { "Dump",                     ctype_fn, .exec.fn = FMC_Menu, VOID(0), "Dump FMC Block Settings" },
        { "SRAM Test",                ctype_fn, .exec.fn = FMC_Menu, VOID(1), "Do FMC SRAM test 0/1" },
        { "SRAM Test pattern",        ctype_fn, .exec.fn = FMC_Menu, VOID(2), "Do FMC SRAM test w pattern" },
        { "SRAM Byte Test pattern",   ctype_fn, .exec.fn = FMC_Menu, VOID(3), "Do FMC SRAM bytewise test w pattern" },
        { "PSRAM access speed test",  ctype_fn, .exec.fn = FMC_Menu, VOID(4), "Do a PSRAM access speed test" },
        { "DTCM access speed test",   ctype_fn, .exec.fn = FMC_Menu, VOID(5), "Do a DTCM access speed test" },
        { "AXISRAM access speed test",ctype_fn, .exec.fn = FMC_Menu, VOID(6), "Do a AXISRAM access speed test" },
        { "SRAM2 access speed test",  ctype_fn, .exec.fn = FMC_Menu, VOID(7), "Do a SRAM2 access speed test" },
        { "SRAM4 access speed test",  ctype_fn, .exec.fn = FMC_Menu, VOID(8), "Do a SRAM4 access speed test" },
        { "SDRAM access speed test",  ctype_fn, .exec.fn = FMC_Menu, VOID(9), "Do a SDRAM access speed test" },
        { "SDRAM check",              ctype_fn, .exec.fn = FMC_Menu, VOID(90),"Perform SDRAM Checksum check" },
        { "Disable D-Cache",          ctype_fn, .exec.fn = FMC_Menu, VOID(99), "" },
        { "Enable D-Cache",           ctype_fn, .exec.fn = FMC_Menu, VOID(98), "" },
        { "Set ExtMem base addr",     ctype_fn, .exec.fn = FMC_Menu, VOID(97), "Base addr to 0x<XXXX>0000" },
    };
    ADD_SUBMODULE(FMC);
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
#if LOGTO_FATFS > 0
    #include "logfile.h"
    char logbuf[80];
    uint32_t num;
#endif
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
                Log_SetDebugLevels(CMD_to_number ( word, wordlen ));
            }
            printf("Console Debuglevel=%d\n", console_debuglevel);
            printf("FatFS   Debuglevel=%d\n", fatfs_debuglevel);
            break;
#endif
#if LOGTO_FATFS > 0
        case 1:
            printf("Flush Logfile\n");
            LogFile_Flush();
            break;
        case 2:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Write Log <n> - Write <n>+10 lines to log\n");
            } else {
                CMD_get_one_word( &word, &wordlen );
                num = CMD_to_number ( word, wordlen );
            }
            printf("Write %d lines to logfile\n", num);
            for ( uint32_t i = 0; i < num ; i++ ) {
                for ( uint32_t j=0; j < 10; j++ ) {
                    snprintf(logbuf,80, "This is Block %d, line %d", i+1, j+1);
                    FLOGU_ALWAYS(logbuf, strlen(logbuf) );
                }
            }
            break;
#endif
        case 3:
            Dump_VersionInfo();
            break;
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
  { "Version",          ctype_fn,  .exec.fn = MainMenu,       VOID(3),  "Version info"  },
#endif
#if USE_I2C > 0
  { "I2C"    ,         ctype_sub, .exec.sub = &mdlI2C,         0,       "I2C submenu" },
#endif
#if defined(USE_ADC3)
  { "ADC"    ,         ctype_sub, .exec.sub = &mdlADC,         0,       "ADC submenu" },
#endif
#if USE_PWMTIMER > 0
  { "PWM"    ,         ctype_sub, .exec.sub = &mdlPWM,         0,       "PWM Timer submenu" },
#endif
#if USE_DS18X20 > 0
  { "DS18X20"    ,     ctype_sub, .exec.sub = &mdl18X20,       0,       "Dallas 18X20 submenu" },
#endif
#if DEBUG_PROFILING > 0
  { "Profile",         ctype_fn,  .exec.fn = ProfilerDump,     VOID(0), "Dump profiling info" },
#endif
  { "Test&System",     ctype_sub, .exec.sub = &mdlTest,        0,       "Test & System submenu" },
#if USE_PWMTTIMER > 0
  { "PWM test",        ctype_sub, .exec.sub = &mdlPWM,         0,       "Test PWM functions" },
#endif
#if USE_QSPI > 0
  { "QSpi test",       ctype_sub, .exec.sub = &mdlQSPI,        0,       "Test QuadSpi functions" },
#endif
#if USE_CAN > 0
  { "CAN test",        ctype_sub, .exec.sub = &mdlCAN,         0,       "Test CAN functions" },
#endif
#if USE_FMC > 0
  { "FMC test",        ctype_sub, .exec.sub = &mdlFMC,         0,       "Test FMC functions" },
#endif
#if LOGTO_FATFS > 0
  { "Write Logfile",    ctype_fn, .exec.fn = MainMenu,        VOID(2), "Write Logfile" },
  { "Flush Logfile",    ctype_fn, .exec.fn = MainMenu,        VOID(1), "Flush Logfile" },
#endif
};
ADD_SUBMODULE(Basic);


/**
  * @}
  */
