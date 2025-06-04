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

#include "config/config.h"
#include "system/clockconfig.h"
#include "config/devices_config.h"
#include "interpreters.h"
#include "debug_helper.h"
#include "system/profiling.h"
#include "system/util.h"
#include "wireless.h"
#include "timer.h"
#include "rfm/rfm_packets.h"
#include "task/minitask.h"
#include "com.h"
#include "dev/i2c_dev.h"
#include "system/status.h"
#include "system/hw_util.h"
#include "cmdline.h"
#include "sensors/thp_sensor.h"
#include "eeprom.h"
#include "dev/devices.h"
#include "system/tm1637.h"
#include "version.h"

#if USE_DS18X20  > 0
    #include "onewire.h"
    #include "ds18xxx20.h"
#endif
#if USE_EPAPER > 0
    #include "disp/epaper.h"
#endif
#if USE_DOGM132 > 0
    #include "disp/dogm-graphic.h"
    #include "disp/fonts/lcd_fonts.h"
#endif
#if USE_QENCODER > 0
    #include "dev/qencode.h"
#endif
#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif

#if USE_HW_PWMTIMER > 0 || USE_USER_PWMTIMER > 0
    #include "dev/pwm_timer.h"
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

void SystemClock_Set(CLK_CONFIG_T clk_config_byte, bool bSwitchOffMSI );

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
      DBG_dump_peripheralclocksetting();
      break;
    case 4:
      DBG_dump_peripheralclockconfig();
      break;
    case 5:
      DBG_dump_peripheralclocksetting_insleepmode();
      break;
    case 6:
      for ( uint32_t i = 0; i < 5; i++ )
         Config_Menu( cmdline, len, VOID(i) );
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
          printf("Usage:MCO output <n>, 0=Off, 1=SYSCLK, 2=MSI, 3=HSI, 4=HSE, 5=PLL, 6=LSE, 7=LSI");
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
  { "All",       ctype_fn, {Config_Menu},  VOID(6), "Show Items 0) to 4) "  },
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
  ARD_PinT dio, clk;
    
  UNUSED(cmdline);UNUSED(len);

  switch( (uint32_t)arg )  {
    case 0:
      DBG_sram();
      break;
    case 1:
      DBG_dump_exti_config(0);
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
    case 6:
      clk.gpio = GPIOA; clk.pin = 0;  
      dio.gpio = GPIOA; dio.pin = 1;  
      TM1637_Init(clk,dio, 100);
      break;
    case 7:
    case 10:
      if ( CMD_argc() < 1 ) {
        printf("Usage: 'Display <number>'\n");
        return false;
      }
      CMD_get_one_word( &word, &wordlen );
      initarg = CMD_to_number ( word, wordlen );
      if ( (uint32_t)arg == 7 )
          TM1637_displayInteger(initarg,0, 99);
      else
        TM1637_displayHex(initarg,0, 99);
      break;
    case 8:
      if ( CMD_argc() < 1 ) {
        printf("Usage: 'Display <number>'\n");
        return false;
      }
      CMD_get_one_word( &word, &wordlen );
      initarg = CMD_to_number ( word, wordlen );
      TM1637_displayInteger(initarg,1, 2);
      break;
    case 9:
      TM1637_clearDisplay();
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
  { "Disp Init", ctype_fn, {Devices_Menu},    VOID(6), "Init 6 DD"  },
  { "Disp Num4", ctype_fn, {Devices_Menu},    VOID(7), "Write <num> to 6 DD"  },
  { "Disp Num6", ctype_fn, {Devices_Menu},    VOID(8), "Write <num> to 6 DD"  },
  { "Disp hex",  ctype_fn, {Devices_Menu},    VOID(10),"Write <hex> to 6 DD"  },
  { "Disp Clr",  ctype_fn, {Devices_Menu},    VOID(9), "Clear display"  },
};
ADD_SUBMODULE(Devices);


#endif  // DEBUG_FEATURES >0

#if USE_RFM12 > 0 || USE_RFM69 > 0
    #include "global_flags.h"
    #include "rfm/rfm_spi_interface.h"
    int16_t testAVG;
    void wirelessReceivePacket(PKT_StatusType rxStatus);
    /*********************************************************************************
      * @brief  Submenu for RFM functions
      *         
      * @retval true on success, false otherwise
      *
      * @note   will try to read as many parameters as needed
      ********************************************************************************/
    static bool RFM_Menu ( char *cmdline, size_t len, const void * arg )
    {
      UNUSED(cmdline);UNUSED(len);
      char *word;
      size_t wordlen;
      uint32_t n,m;

      switch((uint32_t)arg) {
        case 0:
          printf("Missed syncs=%d\n", (uint32_t)missed_syncs);
          printf("Timeskew @0 =%d\n", (uint32_t)avg_skew_00);
          printf("Timeskew @3 =%d\n", (uint32_t)avg_skew_30);
          #if USE_RFM_OOK > 0
              printf("OOK Waittime=%d\n", (uint32_t)avg_waitms);
              printf("OOK-Retries =%d\n", (uint32_t)OOK_RetryCount);
          #endif
          break;
        case 1:
          DEBUG_PUTS("RFM Init");
          rfm->rfmXX_Init();
          break;
    #if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0
        case 2:
          rfm->rfmXX_Dump_status();
          break;
    #endif
        case 3:
          COM_print_debug(true);
          break;
        case 4:
          DEBUG_PUTS("Begin receive");
          SetFSKStatus(FSK_STATUS_WAITFORSYNC,false);
          PKT_ReceiveDefault(wirelessReceivePacket);
          break;
        case 5:
            DEBUG_PUTS("Force Resync");
            time_sync_tmo =0;
            wirelesTimeSyncCheck();
            break;
        case 6:
          COM_print_version(false);
          break;
    #if USE_RFM_OOK > 0
        case 7:
         if ( CMD_argc() < 1 ) {
            printf("Usage: 'O-Recv 0/1 \n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          if ( n ) {
            DEBUG_PUTS("OOK on");
            SetOOKStatus(OOK_STATUS_WAITFORSYNC, false);
            Start_OOK_Receiver(0);
            SetFlagBit(gflags, GFLAG_FORCE_OOK_BIT ); 
          } else {
            DEBUG_PUTS("OOK off");
            Reset_OOK_Receiver();
            ClearFlagBit(gflags, GFLAG_FORCE_OOK_BIT ); 
            SetOOKStatus(OOK_STATUS_NO_PEER, false);
          }
          break;
    #endif
    #if USE_RFM69 > 0
        case 8:
         if ( CMD_argc() < 1 ) {
            printf("Usage: 69-Get <n> : Get RFM69-Reg <n>\n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          m = rfm_spi_read8((uint8_t)n);
          DEBUG_PRINTF("RFM69 Reg# 0x%02x = 0x%02x\n", (uint8_t)n, (uint8_t)m);
          rfm_spi_write8((uint8_t)n, (uint8_t)m);
          break;
        case 9:
         if ( CMD_argc() < 2 ) {
            printf("Usage: 69-Set <n> <m> : Set RFM69-Reg <n> to <m>\n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          CMD_get_one_word( &word, &wordlen );
          m = CMD_to_number ( word, wordlen );
          DEBUG_PRINTF("Set RFM69 Reg# 0x%02x = 0x%02x\n", (uint8_t)n, (uint8_t)m);
          rfm_spi_write8((uint8_t)n, (uint8_t)m);
          break;
    #endif
        case 10:
         if ( CMD_argc() < 1 ) {
            printf("Usage: Average <n> : Put <n> to average\n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          testAVG=average3(testAVG, n);
          printf("Average=%d\n", testAVG);
          break;
        case 11:
          testAVG=0;
          break;
    #if USE_RFM_OOK > 0
        case 12:
         if ( CMD_argc() < 1 ) {
            printf("Usage: 'Retime OOK <sec>\n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          n = n % 60;
          DEBUG_PRINTF("Retime OOK-Receiver to second %02d ",n);
          DEBUG_PUTS(Retime_OOK_Receiver(n) ? "ok" : "fail");
          break;
        case 13:
         if ( CMD_argc() < 1 ) {
            printf("Usage: 'OOK-Retries <n>\n");
            return false;
          }
          CMD_get_one_word( &word, &wordlen );
          n = CMD_to_number ( word, wordlen );
          OOK_retries = n;
          DEBUG_PRINTF("OOK-Retries set to %d ",n);
          break;
    #endif
        default:
          DEBUG_PUTS("RFM_Menu: command not implemented");
      } /* end switch */

      return true;

    } /* RFM_Menu */

    static const char *pmtRFM (void)
    {
      return "RFM";
    }

    static const CommandSetT cmdRFM[] = {
      { "Resync",          ctype_fn, .exec.fn = RFM_Menu,  VOID(5), "Enforce resync"}, 
      { "Missed",          ctype_fn, .exec.fn = RFM_Menu,  VOID(0), "Show number of missed sync frames" },
      { "Init",            ctype_fn, .exec.fn = RFM_Menu,  VOID(1), "Initialize RFM to FSK mode" },
    #if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0
      { "Status",          ctype_fn, .exec.fn = RFM_Menu,  VOID(2), "Show RFM status word"  },
    #endif
      { "D-cmd",           ctype_fn, .exec.fn = RFM_Menu,  VOID(3), "Show system status"  },
      { "Receive",         ctype_fn, .exec.fn = RFM_Menu,  VOID(4), "Switch on RFM receiver"  },
    #if USE_RFM_OOK > 0
      { "O-Recv",          ctype_fn, .exec.fn = RFM_Menu,  VOID(7), "Initialize RFM to OOK mode" },
    #endif
    #if USE_RFM69 > 0
      { "69-Get <n>",      ctype_fn, .exec.fn = RFM_Menu,  VOID(8), "Get RFM69-Reg <n>" },
      { "69-Set <n> <m>",  ctype_fn, .exec.fn = RFM_Menu,  VOID(9), "Set RFM69-Reg <n> to <m>" },
    #endif
      { "Average <n>",     ctype_fn, .exec.fn = RFM_Menu,  VOID(10), "Add <n> to Average" },
      { "Average Init",    ctype_fn, .exec.fn = RFM_Menu,  VOID(11), "Init Average" },
    #if USE_RFM_OOK > 0
      { "Retime OOK <n>",  ctype_fn, .exec.fn = RFM_Menu,  VOID(12), "Retime OOK receiver to second <n>" },
      { "OOK-retries <n>", ctype_fn, .exec.fn = RFM_Menu,  VOID(13), "Set OOK-retries to <n>" },
    #endif
    };
    ADD_SUBMODULE(RFM);
#endif

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
          const THPSENSOR_DecisTypeDef Init = {-1,-1,-1,0,0};
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
                    work = THPSENSOR_GetP_MSL();
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
  uint8_t chnlen;
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
      if ( e->GetID(buffer, &chnlen, 255 ) != EEPROM_OK ) {
        puts("EEPROM Read ID failed");
      } else {
          for ( i=0; i< chnlen; i++) 
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
     chnlen = CMD_to_number ( word, wordlen );

      for(i=0; i <size; i++ ) buffer[i] = (uint8_t)chnlen;  
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

static bool Test_Bitband ( char *cmdline, size_t len, const void * arg )
{
  uint32_t *start, *stop;
  UNUSED(cmdline);UNUSED(len);UNUSED(arg);
  DEBUG_PRINTF("LPUART1->CR1=%04x\n",(uint16_t)LPUART1->CR1);
  
  start = HW_GetPeriphBitBandAddr(&LPUART1->CR1,0);
  stop  = HW_GetPeriphBitBandAddr(&LPUART1->CR1,15);
  for ( ;start <= stop; start++ )
    DEBUG_PRINTF("%04x ",(uint16_t)*start);
  DEBUG_PUTS("");
 
  return true;
}

#if USE_EPAPER > 0 || USE_QENCODER > 0 || 1
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

#if defined(USE_LPUART1) 
    #define OUTBUF_SIZE 256
    #define INBUF_SIZE  64

    static DMAMEM uint8_t outbuf[OUTBUF_SIZE];
    static DMAMEM uint8_t inbuf[INBUF_SIZE];    
    static CircBuffT o;
    static LinBuffT i; 
#endif

static bool Test_Menu ( char *cmdline, size_t len, const void * arg )
{
  UNUSED(cmdline);UNUSED(len);
  char *word;
  uint8_t c;
  size_t wordlen;
  uint8_t pattern;

  switch((uint32_t)arg) {
  #if USE_EPAPER > 0
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
  #endif
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
       case 5:
         CircBuff_Init       (&o, OUTBUF_SIZE, outbuf);
         LinBuff_Init        (&i, INBUF_SIZE,  inbuf );
         Usart_AssignBuffers (&HandleCOM9, &i, &o ); 
         CircBuff_PutStr     (&o,(uint8_t *)"Am Ende ist man immer schlauer",30);
         UsartStartTx        (&HandleCOM9, o.buf+o.rdptr,CBUF_GET_USED(o));
         break; 
#endif
       default:
         DEBUG_PUTS("Test_Menu: command not implemented");
  } /* end switch */

  return true;

} /* Test_Menu */
#endif
#include "system/periodic.h"
/*********************************************************************************
  * @brief  Submenu for system functions
  *         
  * @retval true on success, false otherwise
  *
  * @note   will try to read as many parameters as needed
  ********************************************************************************/
static bool System_Menu ( char *cmdline, size_t len, const void * arg )
{
  typedef void (*func) (void);
  func fn=NULL;
  uint32_t *test = NULL;  
  UNUSED(cmdline);UNUSED(len);

  switch((uint32_t)arg) {
    case 0:
        TaskDumpList();
        break;
    case 1:
        PeriodicDumpList();
        break;
    case 2:
        fn();
        break;
    case 3:
        *test = 17;
        break;

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
  { "Call 0",          ctype_fn, .exec.fn = System_Menu,    VOID(2), "Provoke NULL ptr error" },
  { "Access 0",        ctype_fn, .exec.fn = System_Menu,    VOID(3), "write to NULL ptr" },

  { "TmrAbs",          ctype_fn, .exec.fn = Test_TmrAbs,    VOID(0), "Set Abs Timer" },
  { "TmrRel",          ctype_fn, .exec.fn = Test_TmrRel,    VOID(0), "Set Rel Timer" },
  { "TmrDel",          ctype_fn, .exec.fn = Test_TmrDel,    VOID(0), "Delete priodic timer" },
  { "Multi Tmr",       ctype_fn, .exec.fn = Test_TmrMulti,  VOID(0), "Lots of timers" }, 
  { "Kill Tmr",        ctype_fn, .exec.fn = Test_TmrKill,   VOID(0), "Generates Error" }, 
  { "Toggle",          ctype_fn, .exec.fn = Test_Toggle,    VOID(0), "Toggle UserLed2" },
  { "Bit-band test",   ctype_fn, .exec.fn = Test_Bitband,   VOID(0), "Test Bit-banding"}, 
#if USE_EPAPER > 0
  { "ePaper Test",     ctype_fn, .exec.fn = Test_Menu,      VOID(1), "Test ePaper"}, 
#endif
#if USE_QENCODER > 0
  { "QEncoder test",   ctype_fn, .exec.fn = Test_Menu,      VOID(2), "Test quadrature encoder"}, 
#endif
#if defined(USE_LPUART1) 
  { "LPUART1 R/W byte",ctype_fn, .exec.fn = Test_Menu,      VOID(4), "Test LPUART1 r/w"}, 
  { "LPUART1 W string",ctype_fn, .exec.fn = Test_Menu,      VOID(5), "Test LPUART1 write str "}, 
#endif
#if USE_EEPROM_EMUL > 0
  { "Write config ",   ctype_fn, .exec.fn = Test_Menu,      VOID(3), "Write config[31] x times"}, 
#endif
};
ADD_SUBMODULE(Test);

#if defined(USE_ADC1)
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
            ADC_Calibrate(&HW_ADC1);
            /* fall through */
        case 1:
            ADC_MeasureVdda ((void *)&HW_ADC1);
            printf("Vdda = %dmV\n", ADC1Handle.vdda); 
            break;
        case 2:
            ADC_MeasureChipTemp(&HW_ADC1);
            printf("Chip Temp = %dC\n", ADC1Handle.chiptemp); 
            break;
        case 3:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'ADC channel <num> \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            ADC_MeasureChannel(&HW_ADC1, ch);
            break;
        case 4:
            puts("Measure group manually"); 
            ADC_SetupGroup(&HW_ADC1, true);
            ADC_MeasureGroup(&HW_ADC1);
            ADC_ShowStatus(&HW_ADC1);
            break;
        case 8:
            puts("Measure group auto"); 
            ADC_SetupGroup(&HW_ADC1, false);
            ADC_MeasureGroup(&HW_ADC1);
            ADC_ShowStatus(&HW_ADC1);
            break;
        case 5:
            ADC_DisableRefintCh(&HW_ADC1);
            puts("Refint channel disabled"); 
            break;
        case 6:
            ADC_DisableAllInternalCh(&HW_ADC1);
            puts("Refint channel disabled"); 
            break;
        case 7:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Peripheral 1/0\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            PeriphTimer_StartStop(ch!=0);
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
        { "Measure group manually", ctype_fn, .exec.fn = ADC_Menu,VOID(4), "Measure whole sequence once manually" },
        { "Measure group auto",     ctype_fn, .exec.fn = ADC_Menu,VOID(8), "Measure whole sequence automatically repeated" },
        { "Refint disable",         ctype_fn, .exec.fn = ADC_Menu,VOID(5), "Disable Refint ADC channel" },
        { "All int. Ch. Disable",   ctype_fn, .exec.fn = ADC_Menu,VOID(6), "Disable all internal ADC channels" },
        { "Periph Timer Start/Stop",ctype_fn, .exec.fn = ADC_Menu,VOID(7), "Start/Stop the perpheral timer" },
    };
    ADD_SUBMODULE(ADC);
#endif

#if USE_DOGM132 > 0
    /*********************************************************************************
      * @brief  Submenu for Dogm132 testing
      *         
      * @retval true on success, false otherwise
      *
      * @note  will try to read as many parameters as needed
      ********************************************************************************/
    static bool Dogm_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char     *word;
      size_t   wordlen;
      uint8_t  ch;
      uint32_t argnum;
      
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            if ( CMD_argc() < 1 ) {
              printf("Usage: 'Wipe 0|1 (normal/invert)\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            if ( ch )
                dogm_clear_display(INVERT); 
            else
                dogm_clear_display(NORMAL); 
            break;
        case 1:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Prop8 <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char prop 8"); 
	    lcd_set_font(FONT_PROP_8, NORMAL);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
	   break;
        case 2:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Fixed8 <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char fixed 8"); 
	    lcd_set_font(FONT_FIXED_8, NORMAL);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
            break;
        case 3:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Prop8 dbl <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char prop 8 dbl size"); 
	    lcd_set_font(FONT_PROP_8, DOUBLE_SIZE);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
            break;
        case 4:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Prop16 Invert <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char prop 16 invert"); 
	    lcd_set_font(FONT_PROP_16, INVERT);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
            break;
        case 5:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Dig24 <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char dig 24"); 
	    lcd_set_font(FONT_DIGITS_24, NORMAL);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
	   break;
        case 6:
            argnum = CMD_argc();
            if ( argnum < 1 ) {
              printf("Usage: 'Dig36 <c1> [<c2> [...]]\n");
              return false;
            }
  	    puts("Char dig 24"); 
	    lcd_set_font(FONT_DIGITS_32, NORMAL);
            while ( argnum > 0 ) {
                CMD_get_one_word( &word, &wordlen );
                ch = CMD_to_number ( word, wordlen );
                lcd_putc(ch);
                argnum--;
           }
	   break;
#if USE_HW_PWMTIMER > 0
        case 7:
            argnum = CMD_argc();
            if ( argnum < 2 ) {
              printf("Usage: 'PWM <ch> <val>: PWM on <ch> to <val>/256, 0=off\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            argnum = CMD_to_number ( word, wordlen );
  	    printf("PWM on Ch %d to %d\n", ch, argnum); 
            if ( argnum > 0 )
                PWM_CH_StartPWMChS256(&HW_PWMTIMER, ch, argnum);
            else
                PWM_CH_StopPWMCh(&HW_PWMTIMER, ch);
            break;
#endif
        default:
          DEBUG_PUTS("Dogm_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtDogm (void)
    {
      return "Dogm132 testing";
    }


    static const CommandSetT cmdDogm[] = {
        { "Wipe Display",           ctype_fn, .exec.fn = Dogm_Menu,VOID(0), "Wipe display" },
        { "Prop8",                  ctype_fn, .exec.fn = Dogm_Menu,VOID(1), "chars in Prop8 font" },
        { "Fixed8",                 ctype_fn, .exec.fn = Dogm_Menu,VOID(2), "chars in Fixed8 font" },
        { "Prop8 double",           ctype_fn, .exec.fn = Dogm_Menu,VOID(3), "chars in Prop8 font, double size" },
        { "Prop16 invert",          ctype_fn, .exec.fn = Dogm_Menu,VOID(4), "chars in Prop16 font, inverted" },
        { "Dig24",                  ctype_fn, .exec.fn = Dogm_Menu,VOID(5), "Digits in 24pt" },
        { "Dig32",                  ctype_fn, .exec.fn = Dogm_Menu,VOID(6), "Digits in 32pt" },
#if USE_HW_PWMTIMER > 0
        { "PWM <ch> <val>",         ctype_fn, .exec.fn = Dogm_Menu,VOID(7), "Digits in 32pt" },
#endif
    };
    ADD_SUBMODULE(Dogm);
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
            t = DS18X20_GetOneTemp();
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
        { "ROM Search",             ctype_fn, .exec.fn = DS_Menu,VOID(0), "Calibrate ADC & Measure Vdda" },
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

#if USE_HW_PWMTIMER > 0 || USE_USER_PWMTIMER > 0
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
      const PwmChannelT* chn;     
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            len=0;
            DEBUG_PRINTF("List of all PWM channels\n");
            DEBUG_PRINTF("  No. Timer Chn Inv User Basefrq PWMfrq Duty Active?\n"); 
            for ( chn=PWM_CH_IterateBegin(); chn; chn=PWM_CH_IterateNext(), len++ ) {
                TMR_GetActualBaseAndPwmFrq(chn->tmr, &num, &ch);
                ret = PWM_CH_GetPWMPromille(chn->tmr, chn->channel);
                DEBUG_PRINTF("  %2d: %s   %d   %d  %3d %8d %6d %3d%% %s\n",
                              len, chn->tmr->devName, chn->channel, chn->bInvert,  chn->bUserByte,
                              num, ch, (ret+5)/10,
                              TMR_IsChnActive(chn->tmr, chn->channel) ? "Y" : "N" );
            }
            break;
        case 1:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Frequency <ch> <frq>\n");
              return false;
            }
            ch = CMD_to_number ( word, wordlen );
            chn =  PWM_CH_GetCh(ch);
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            ret = PWM_TMR_SetPWMFrq(chn->tmr, num );
            DEBUG_PRINTF("Setting PWM frq to %d - %s\n", num, ret ? "ok" : "fail");
            break;
        case 2:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Init <ch> <mode>\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            chn =  PWM_CH_GetCh(ch);
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            PwmChannelT work= {chn->tmr, chn->channel, num!=0, 0 };
            ret = PWM_CH_Init(&work); 
            DEBUG_PRINTF("Init PWM ch %d - %s\n", ch, ret ? "ok" : "fail");
            break;
        case 3:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Start Promille <ch> <promille>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            chn =  PWM_CH_GetCh(ch);
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            PWM_CH_StartPWMChPromille (chn->tmr, chn->channel, num); 
            DEBUG_PRINTF("Start PWM ch %d w. %d/1000\n", ch, num);
            break;
        case 4:
            if ( CMD_argc() < 2 ) {
              printf("Usage: 'Start S256 <ch> <s256>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            chn =  PWM_CH_GetCh(ch);
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            PWM_CH_StartPWMChS256(chn->tmr, chn->channel, num); 
            DEBUG_PRINTF("Start PWM ch %d w. %d/256\n", ch, num);
            break;
        case 5:
            if ( CMD_argc() < 1) {
              printf("Usage: 'Stop PWM <ch>");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            ch = CMD_to_number ( word, wordlen );
            chn =  PWM_CH_GetCh(ch);
            PWM_CH_StopPWMCh(chn->tmr, chn->channel); 
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
        { "PWM Channel List",       ctype_fn, .exec.fn = PWM_Menu,VOID(0), "Show list of all PWM channels defined" },
        { "Frequency <ch> <frq>",   ctype_fn, .exec.fn = PWM_Menu,VOID(1), "Set PWM frequency of channel <ch> to <frq>" },
        { "Init <Ch> <mode>",       ctype_fn, .exec.fn = PWM_Menu,VOID(2), "Init PWM Ch <x> to normal/invert (0|1)" },
        { "Start <ch> Promille",    ctype_fn, .exec.fn = PWM_Menu,VOID(3), "Start PWM Ch <x> w. duty cycle of <y>/1000" },
        { "Start <ch> S256",        ctype_fn, .exec.fn = PWM_Menu,VOID(4), "Start PWM Ch <x> w. duty cycle of <y>/256" },
        { "Stop <ch>",              ctype_fn, .exec.fn = PWM_Menu,VOID(5), "Stop PWM ch <x>" },
    };
    ADD_SUBMODULE(PWM);
#endif

#if USE_QSPI > 0 || USE_OSPI > 0

    #if   USE_OSPI > 0
        #define XSpi1Handle         OSpi1Handle
        #define XSPI1_USE_IRQ       OSPI1_USE_IRQ
        #define XSPI_BASE           OCTOSPI1_BASE
        #define XSPI_HW             HW_OSPI1
    #elif USE_OSPI2 > 0
        #define XSpi1Handle         OSpi2Handle
        #define XSPI1_USE_IRQ       OSPI2_USE_IRQ
        #define XSPI_BASE           OCTOSPI2_BASE
        #define XSPI_HW             HW_OSPI2
    #else
        #define XSpi1Handle         QSpi1Handle
        #define XSPI1_USE_IRQ       QSPI1_USE_IRQ
        #define XSPI_BASE           QSPI_BASE
        #define XSPI_HW             HW_QSPI1
    #endif
    #include "dev/xspi_dev.h"   
    
    bool XSpecific_WriteEnable(XXSPI_HandleTypeDef *hxspi);
    
    /*********************************************************************************
     * @brief  Submenu for QuadSpi
     *         
     * @retval true on success, false otherwise
     *
     * @note  will try to read as many parameters as needed
     ********************************************************************************/

    #define PGSIZE  10240
    static uint8_t PageBuffer[PGSIZE];
#if defined(XSPI1_USE_IRQ)

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
#if defined(XSPI1_USE_IRQ)
      XSpi_SetAsyncCallbacks(&XSpi1Handle, rddoneCB, wrdoneCB, errorCB);
#endif

      switch((uint32_t)arg) {
        case 0:
            printf("Flash Size= ...... %d kByte\n", 1 << (XSpi1Handle.geometry.FlashSize-10));
            printf("Erase sector size= %d Byte\n", XSpi1Handle.geometry.EraseSize[0]);
            printf("   sector count= . %d\n", XSpi1Handle.geometry.EraseNum[0]);
            printf("Write page size= . %d Byte\n", XSpi1Handle.geometry.ProgPageSize);
            printf("   page count= ... %d\n", XSpi1Handle.geometry.ProgPagesNumber);
            printf("pages per sector=  %d\n", XSpi1Handle.geometry.EraseSize[0]/XSpi1Handle.geometry.ProgPageSize);            
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
            addr =  XSpi1Handle.geometry.EraseSize[0] * num;
            printf("Erase sector %d to %d (startaddr=0x%08x)\n", num, num+cnt, addr);
            ret = XSpi_EraseIT(&XSpi1Handle, XSPI_ERASE_SECTOR, addr, cnt+1);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 2:
            if ( XSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", XSpi1Handle.geometry.ProgPageSize);
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
            addr =  XSpi1Handle.geometry.ProgPageSize * num;
            for ( i=0; i <=cnt; i++ ) {
                for ( j  = 0; j < XSpi1Handle.geometry.ProgPageSize; j++ )
                    PageBuffer[j]=i+j;
                printf("Write page %d (startaddr=0x%08x) - ", num+i, addr );
                ret = XSpi_WriteWait(&XSpi1Handle, PageBuffer, addr, XSpi1Handle.geometry.ProgPageSize );
                printf ( "%s\n", ret ? "ok": "fail");
                addr += XSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 3:
            if ( XSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", XSpi1Handle.geometry.ProgPageSize);
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
            addr =  XSpi1Handle.geometry.ProgPageSize * num;
            for ( i=0; i <=cnt; i++ ) {
                printf("compare page %d (startaddr=0x%08x) - Read ", num+i, addr );
                #if defined(XSPI1_USE_IRQ)
                    ret = XSpi_ReadIT(&XSpi1Handle, PageBuffer, addr, XSpi1Handle.geometry.ProgPageSize);
                #else
                    ret = XSpi_ReadWait(&XSpi1Handle, PageBuffer, addr, XSpi1Handle.geometry.ProgPageSize);
                #endif
                printf ( "%s\n", ret ? "ok": "fail");
                #if defined(XSPI1_USE_IRQ)
                    DEBUG_PUTS("Waiting for Async op to be done");
                    uint32_t start = HAL_GetTick();
                    while ( XSpi1Handle.bAsyncBusy );
                    DEBUG_PRINTF("Wait terminated after %d ticks\n", HAL_GetTick() - start);
                #endif
                for ( j  = 0; j < XSpi1Handle.geometry.ProgPageSize; j++ ) {
                    if ( PageBuffer[j] != (uint8_t)(i+j) ) {
                        printf("Compare failed Page offset 0x%02x: read 0x%02x, expected 0x%02x\n", j, PageBuffer[j], i+j);
                        break;
                    }
                }
                addr += XSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 4:
            if ( XSpi1Handle.geometry.ProgPageSize > PGSIZE ) {
                printf("Internal page size has to be at least %d- aborting test\n", XSpi1Handle.geometry.ProgPageSize);
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
            addr =  XSpi1Handle.geometry.ProgPageSize * num;
            /* In memory mapped mode, use the mapped address */
            if ( XSpi1Handle.bIsMemoryMapped ) addr += XSPI_BASE;
            for ( i=0; i <=cnt; i++ ) {
                printf("Read page %d (startaddr=0x%08x) in %s mode - ", num+i, addr,  XSpi1Handle.bIsMemoryMapped ? "MemoryMapped" : "QSPI" );
                if ( XSpi1Handle.bIsMemoryMapped ) {
                    memmove(PageBuffer,(void *)(addr), XSpi1Handle.geometry.ProgPageSize);
                    ret = true;
                } else {
                    ret = XSpi_ReadWait(&XSpi1Handle, PageBuffer, addr, XSpi1Handle.geometry.ProgPageSize);
                }
                printf ( "%s\n", ret ? "ok": "fail");
                for ( j  = 0; j < XSpi1Handle.geometry.ProgPageSize; j++ ) {
                    if ( j % 16 == 0 ) printf ( "0x%08x",addr+j );
                    printf(" %02x", PageBuffer[j]);
                    if ( (j+1) % 16 == 0 ) puts ("");
                }
                addr += XSpi1Handle.geometry.ProgPageSize;
            }
            break;
        case 5:
            printf("Enable memory mapped mode- ");
            ret = XSpi_EnableMemoryMappedMode(&XSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 6:
            printf("Abort operation - ");
            ret = XSpi_Abort(&XSpi1Handle);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 7:
            printf("Read Status");
            XSpi_DumpStatus(&XSpi1Handle);
            break;
        case 8:
            printf("Enter power down - ");
            ret = XSpi_SetDeepPowerDown(&XSpi1Handle, true);
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 9:
            printf("Exit power down - ");
            ret = XSpi_SetDeepPowerDown(&XSpi1Handle, false);
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
                ret = XSpi_WriteDMA(&XSpi1Handle, PageBuffer, addr, cnt );
            else
                ret = XSpi_WriteIT(&XSpi1Handle, PageBuffer, addr, cnt );
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
            printf("Set Qspi Clock speed to %d kHz",  addr );
             
                ret = XSpi_SetSpeed(&XSPI_HW, addr * 1000 );
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 13:
            if ( CMD_argc() < 1 ) {
              printf("Usage: WEN {0|1}  - Reset ot Set Write Enable Flag\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            //ret = XSpecific_WriteEnable(&XSpi1Handle.hxspi);
            ret = XSpiLL_WriteEnable(&XSpi1Handle, addr != 0 );
            printf ( "%s\n", ret ? "ok": "fail");
            break;
        case 14:
            const NOR_FlashCmdListT *cmd = &XSpi1Handle.interface->cmd;
            if ( cmd->r_jid ) {   
                ret = XHelper_CmdArgRead ( &XSpi1Handle, cmd->r_jid, 0, (uint8_t *)&addr, 3 );
                if ( ret ) {
                    printf ( "%02x %02x %02x \n", addr & 0xff, (addr>>8)&0xff,(addr>>16)&0xff);
                } else {
                    printf("Read JedecID failed\n");
                }
            } else {
                printf("Read JedecID not implemented\n");
            }
            break;
        case 15:
            if ( CMD_argc() < 1 ) {
              printf("Usage: QSPI mode {0|1|2}  - Set QSPI mode 1-1-1, 1-2-2 or 1-4-4\n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            if ( addr > 2 ) return false;
            XSpi_SetRWMode(&XSpi1Handle, addr);
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
        { "Write much IT",            ctype_fn, .exec.fn = QSPI_Menu, VOID(10),"Write many bytes IRQ mode" },
        { "Write much DMA",           ctype_fn, .exec.fn = QSPI_Menu, VOID(11),"Write many bytes DMA mode" },
        { "Clk speed <n>",            ctype_fn, .exec.fn = QSPI_Menu, VOID(12),"Set QSPI clock speed to <n> kHz" },
        { "WEN {0|1}",                ctype_fn, .exec.fn = QSPI_Menu, VOID(13),"Reset ot Set Write Enable Flag" },
        { "Read JedecID",             ctype_fn, .exec.fn = QSPI_Menu, VOID(14),"Read 3-byte Jedec ID" },
        { "QSPI mode {0|1|2}",        ctype_fn, .exec.fn = QSPI_Menu, VOID(15),"Set QSPI mode 1-1-1, 1-2-2, 1-4-4" },

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
#if USE_BMP085 > 0 || USE_BME280 > 0 || USE_CCS811 > 0

    #include "sensors/thp_sensor.h"
    #if USE_CCS811 > 0
        #include "sensors/ccs811.h"
    #endif

    /*********************************************************************************
     * @brief  Submenu for THP sensor test functions
     *         
     * @retval true on success, false otherwise
     *
     * @note  will try to read as many parameters as needed
     ********************************************************************************/


    /*********************************************************************************
     * @brief  Submenu for THP sensor test functions
     *         
     * @retval true on success, false otherwise
     *
     * @note  will try to read as many parameters as needed
     ********************************************************************************/
    static bool THP_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t addr;
      int32_t ret;
 
      /* Init all sensor channels to deliver their value with one decimal digit */
      const THPSENSOR_DecisTypeDef Init = {-1,-1,-1,0,0};

     
      UNUSED(cmdline);UNUSED(len);

      switch((uint32_t)arg) {
        case 0:
            printf("THP Sensor initialized %sok\n", THPSENSOR_Init(&Init) == THPSENSOR_OK ? "" : "not ");
            break;
        case 1:
            num = THPSENSOR_IsBusy();
            printf("THP Sensor %s\n", num == 0 ? " in idle" : num > 0 ? "is busy" : "has comm err"  );
            break;
        case 3:
            num = THPSENSOR_Measure(ALL_SENSOR_CHANNELS);
            printf("THP Sensor Measure %s\n", num == THPSENSOR_OK ? "ok" :  "err"  );
            break;
        case 4:
            num = THPSENSOR_GetCapability();
            if ( num & THPSENSOR_HAS_T ) {
                ret= THPSENSOR_GetT();
                printf("THP Sensor Temp=%d\n", ret );
            } else  
                printf("No Temp capability\n");
            if ( num & THPSENSOR_HAS_H ) {
                ret= THPSENSOR_GetH();
                printf("RelHum=%d\n", ret );
            } else  
                printf("No rel. Hum. capability\n");
            if ( num & THPSENSOR_HAS_P ) {
                ret= THPSENSOR_GetLocalP();
                printf("local Pressure=%d\n", ret );
                ret= THPSENSOR_GetP_MSL();
                printf("MSL Pressure=%d\n", ret );
            } else  
                printf("No pressure capability\n");
            if ( num & THPSENSOR_HAS_CO2 ) {
                ret= THPSENSOR_GetCO2();
                printf(", CO2=%dppm\n", ret );
            } else  
                printf("No CO2 capability\n");
            if ( num & THPSENSOR_HAS_TVOC ) {
                ret= THPSENSOR_GetTVOC();
                printf(", TVOC=%dppb\n", ret );
            } else  
                printf("No TVOC capability\n");
            break;
        case 5:
            THPSENSOR_Diagnostics();
            break;
    #if USE_CCS811 > 0
        case 6:
            if ( CMD_argc() < 2 ) {
              printf("Usage: compensate  <rh> <temp> : rh=0.100 temp=0..100 \n");
              return false;
            }
            CMD_get_one_word( &word, &wordlen );
            num = CMD_to_number ( word, wordlen );
            CMD_get_one_word( &word, &wordlen );
            addr = CMD_to_number ( word, wordlen );
            printf("Compensate RH=%d%%, Temp=%d\n",num, addr);
            ccs811_setEnvironmentalData(&ccs811Dev, (uint8_t)num, (uint16_t)addr*100);
            break;
    #endif
        default:
          DEBUG_PUTS("THP_Menu: command not implemented");
      } /* end switch */

      return true;
    }

    static const char *pmtTHP (void)
    {
      return "THP-Sensor";
    }


    static const CommandSetT cmdTHP[] = {
        { "Init",                     ctype_fn, .exec.fn = THP_Menu, VOID(0), "Init THP Sensor" },
        { "Is Busy",                  ctype_fn, .exec.fn = THP_Menu, VOID(1), "Check THP Sensor busy" },
        { "Get Capability",           ctype_fn, .exec.fn = THP_Menu, VOID(2), "Get THP Sensor capability" },
        { "Trigger Measurement",      ctype_fn, .exec.fn = THP_Menu, VOID(3), "Initiate Measurement" },
        { "Read Data",                ctype_fn, .exec.fn = THP_Menu, VOID(4), "Readout Sensor data" },
        { "Diagnostics",              ctype_fn, .exec.fn = THP_Menu, VOID(5), "Print Sensor diagnostics" },
    #if USE_CCS811 > 0
        { "Compensate CCS811",        ctype_fn, .exec.fn = THP_Menu, VOID(6), "compensate RH and Temp" },
    #endif
    };
    ADD_SUBMODULE(THP);
#endif

#if USE_FMC > 0

    #define BUFFER_SIZE         ((uint32_t)0x8000)
    #define WRITE_READ_ADDR     ((uint32_t)0x0800)
    #define SRAM_BANK_ADDR      ((uint32_t)0x60000000)

    static uint16_t aTxBuffer[BUFFER_SIZE];
    static uint16_t aRxBuffer[BUFFER_SIZE];

    /* Status variables */
    __IO bool uwWriteReadStatus = 0;

    /* Counter index */
    static uint32_t uwIndex = 0;

    static void Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLength, uint16_t uwOffset)
    {
      uint16_t tmpIndex = 0;

      /* Put in global buffer different values */
      for (tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++)
            pBuffer[tmpIndex] = uwOffset;
      }

    static bool Buffercmp(uint16_t *pBuffer1, uint16_t *pBuffer2, uint16_t BufferLength)
    {
      uint16_t i = 0;
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
        uint16_t *extmemptr = (uint16_t *)(SRAM_BANK_ADDR + ofs );

        
        /* Fill the buffer to write */
        Fill_Buffer(aTxBuffer, BUFFER_SIZE, pattern);

        /* Write data to the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
        {
        *(extmemptr+uwIndex) = aTxBuffer[uwIndex];
        }

        /* Read back data from the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
        {
        aRxBuffer[uwIndex] = *(extmemptr+uwIndex);
        }


        /*##-3- Checking data integrity ############################################*/
        uwWriteReadStatus = Buffercmp(aTxBuffer, aRxBuffer, BUFFER_SIZE);

        if(!uwWriteReadStatus) {
            DEBUG_PRINTTS("Sram test @0x%08x failed\n",SRAM_BANK_ADDR + ofs );
        } else {
            DEBUG_PRINTTS("Sram test @0x%08x ok\n",SRAM_BANK_ADDR + ofs);

        }
    }

    static void FMC_Sram_TestByte(uint32_t ofs, uint16_t pattern)
    {
        uint8_t *extmemptr = (uint8_t *)(SRAM_BANK_ADDR + ofs );
        uint8_t *srcptr;
        
        /* Fill the buffer to write */
        Fill_Buffer(aTxBuffer, BUFFER_SIZE, pattern);

        srcptr    = (uint8_t *)aTxBuffer;
        /* Write data to the SRAM memory */
        for(uwIndex = 0; uwIndex < BUFFER_SIZE*2; uwIndex++)
        {
            *(extmemptr+uwIndex) = *(srcptr++);
        }

        /* Read back data from the SRAM memory */
        uint8_t *destptr = (uint8_t *)aRxBuffer;
        for(uwIndex = 0; uwIndex < BUFFER_SIZE*2; uwIndex++)
        {
            *(destptr++) = *(extmemptr+uwIndex);
        }


        /*##-3- Checking data integrity ############################################*/
        uwWriteReadStatus = Buffercmp(aTxBuffer, aRxBuffer, BUFFER_SIZE);

        if(!uwWriteReadStatus) {
            DEBUG_PRINTTS("Sram test @0x%08x failed\n",SRAM_BANK_ADDR + ofs );
        } else {
            DEBUG_PRINTTS("Sram test @0x%08x ok\n",SRAM_BANK_ADDR + ofs);

        }
    }


    static bool FMC_Menu ( char *cmdline, size_t len, const void * arg )
    {
      char *word;
      size_t wordlen;
      uint32_t num;
      uint32_t addr;
      bool ret;
      
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
                FMC_Sram_TestWord(i*BUFFER_SIZE, 0x0000);
                FMC_Sram_TestWord(i*BUFFER_SIZE, 0xFFFF);
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
                FMC_Sram_TestWord(i*BUFFER_SIZE, (uint16_t)addr);
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
                FMC_Sram_TestByte(i*BUFFER_SIZE, (uint16_t)addr);
            }
            break;
        default:
          DEBUG_PUTS("CAN_Menu: command not implemented");
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
    };
    ADD_SUBMODULE(FMC);
#endif


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
                console_debuglevel = CMD_to_number ( word, wordlen );
            }
            printf("Console Debuglevel=%d\n", console_debuglevel);
            printf("FatFS   Debuglevel=%d\n", fatfs_debuglevel);
            break;
#endif
        case 1:
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
#if USE_RFM12 > 0 || USE_RFM69 > 0
  { "D-cmd",           ctype_fn,  .exec.fn = RFM_Menu,        VOID(3),  "Show system status"  },
#endif
#if DEBUG_FEATURES > 0
  { "Clock&Pwr",       ctype_sub, .exec.sub = &mdlClkCfg,      0,       "Clock & Power Config submenu" },
  { "Devices",         ctype_sub, .exec.sub = &mdlDevices,     0,       "Peripheral devices submenu" },
#endif
  { "Version",         ctype_fn,  .exec.fn = MainMenu,        VOID(1),  "Show version info"  },
#if USE_RFM12 > 0 || USE_RFM69 > 0
  { "RFM"    ,         ctype_sub, .exec.sub = &mdlRFM,         0,       "RFM submenu" },
#endif
#if USE_I2C > 0
  { "I2C"    ,         ctype_sub, .exec.sub = &mdlI2C,         0,       "I2C submenu" },
#endif
#if defined(USE_ADC1)
  { "ADC"    ,         ctype_sub, .exec.sub = &mdlADC,         0,       "ADC submenu" },
#endif
#if USE_DOGM132 > 0
  { "DOGM132"    ,     ctype_sub, .exec.sub = &mdlDogm,        0,       "DOGM132 submenu" },
#endif
#if USE_HW_PWMTIMER > 0
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
#if USE_QSPI > 0 || USE_OSPI > 0
  { "XSpi test",       ctype_sub, .exec.sub = &mdlQSPI,        0,       "Test Quad/OctoSpi functions" },
#endif
#if USE_BME280 > 0 ||  USE_BMP085 > 0  || USE_CCS811 > 0
  { "THP Sensor test", ctype_sub, .exec.sub = &mdlTHP,        0,       "Test THP Sensor functions" },
#endif
#if USE_CAN > 0
  { "CAN test",        ctype_sub, .exec.sub = &mdlCAN,        0,       "Test CAN functions" },
#endif
#if USE_FMC > 0
  { "FMC test",        ctype_sub, .exec.sub = &mdlFMC,        0,       "Test FMC functions" },
#endif
};
ADD_SUBMODULE(Basic);


/**
  * @}
  */
