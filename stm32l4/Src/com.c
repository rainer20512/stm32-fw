/**
  ******************************************************************************
  * @file    com.c
  * @author  Rainer 
  * @brief   Debug Input handling and command interpreter
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup STM32L0xx_HAL_Examples
  * @{
  */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <inttypes.h>

#include "config/config.h"
#include "dev/adc_dev.h"
#include "ds18xxx20.h"
#include "debug_helper.h"
#include "wireless.h"
#include "system/status.h"
#include "version.h"
#include "eeprom.h"
#include "watch.h"
#include "timer.h"

#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif

/** @addtogroup COM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/*
 * Store character to input buffer and call handler if buffer full or \n received 
 */

#define EXT_TYPE_NOTYPE		0x00
#define EXT_TYPE_OUTDOOR 	0x51
#define EXT_TYPE_COUNTER        0x52
#define EXT_TYPE_ALARM		0x53


void COM_print_debug(bool rfm_transmit) 
{
    DEBUG_PRINTF("D:%s", RTC_GetStrDateTime() );

    DEBUG_PRINTF(" %s", Get_FSK_status_text());
    #ifdef TX18LISTENER
          DEBUG_PUTC('/');
          StatusPrintOOKStatus();
    #endif

    DEBUG_PRINTF(" B: %04d ", ADC_GetVdda(&HW_ADC1) );

    #if defined(GASSENSOR)
        DEBUG_PRINTF(" V:%04d", pa7_average*2 );
    #endif
    #if defined(ZAEHLERV1) || defined(ZAEHLERV2)
        DEBUG_PRINTF(" Z:%05d%03d.%c" countervalueHi,countervalue/10, '0'+countervalue%10);
    #endif

    #if USE_THPSENSOR > 0
        DEBUG_PRINTF(" P:%04d", THPSENSOR_GetP()/10);
    #endif

    // RHB Added: Global Flag	
    DEBUG_PRINTF(" G:");
    StatusPrintGlobalStatus();

    // RHB Added: Wireless Timeout	
    DEBUG_PRINTF(" C:%02hhx" , time_sync_tmo);

    // RHB Added: WirelessBufPtr
    DEBUG_PRINTF(" W:%02" PRIx8, WL_BufPtr);

    #if DEBUG_MEM_CHECK > 0 
        DEBUG_PRINTF(" M:%06" PRI24x, get_mem_unused());
        DEBUG_PRINTF(" SP:%08" PRI32x, DumpStackPtr());
    #endif

    if (CTL_error!=0) {
        DEBUG_PRINTF(" E:%02hhx", CTL_error);
    }                   
    DEBUG_PUTC('\n');

    if ( !rfm_transmit ) return;

    wireless_putchar('D');
    /* 01 */wireless_putchar( RTC_GetMinute() );
    
    // Flag extended packet frame, if used
    /* 02 */wireless_putchar( RTC_GetSecond()| 0x80 ); 
    /* 03 */wireless_putchar(CTL_error);
    
    
    #if defined(TX18LISTENER) || USE_DS18X20 > 0  
        #if USE_DS18X20 > 0
            int16_t t = 10*DS18X20_GetTemp();
            /* 04 */wireless_putchar(t >> 8); // current temp
            /* 05 */wireless_putchar(t & 0xff);
        #else
            /* 04 */wireless_putchar(abstemp >> 8); // current temp
            /* 05 */wireless_putchar(abstemp & 0xff);
        #endif
    #else 
        /* 04 */wireless_putchar(0); 
        /* 05 */wireless_putchar(0);
    #endif
    
    #if defined(GASSENSOR)
        /* 06 */wireless_putchar(pa7_average >> 7); // Vin * 2 
        /* 07 */wireless_putchar(pa7_average << 1 & 0xff);
    #else
        uint16_t v = ADC_GetVdda(&HW_ADC1);
        /* 06 */wireless_putchar(v>>8); 
        /* 07 */wireless_putchar(v &0xff);
    #endif

    /* 08 */wireless_putchar(time_sync_tmo);

    #if defined(TX18LISTENER) 
        /* 09 */wireless_putchar(EXT_TYPE_OUTDOOR); // Outdoor Frame type
        #if USE_THPSENSOR > 0
            { 
                uint16_t p = THPSENSOR_GetP();
                /* 0A */wireless_putchar(p >> 8); // Pressure
                /* 0B */wireless_putchar((uint8_t)p & 0xff);
            }
        #else
            /* 0A */wireless_putchar(0); // Pressure
            /* 0B */wireless_putchar(0);
        #endif
        /* 0C */wireless_putchar(relhum); 				// rel. Humidity
        #if USE_BMP085 > 0 || USE_LSM303D > 0 || USE_FM24V10 > 0 || USE_25X512_EEPROM > 0 || USE_OPTICAL> 0 || USE_OPTICAL_EMETER > 0
                /* 0D */wireless_putchar(general_error_code);	// general error code
        #else
                /* 0D */wireless_putchar(0);					// placeholder
        #endif
    #elif defined (UNIVERSAL) 
        /* 09 */wireless_putchar(EXT_TYPE_OUTDOOR);                     // Outdoor Frame type
        /* 0A */wireless_putchar(0);					// Pressure
        /* 0B */wireless_putchar(0);
        /* 0C */wireless_putchar(0); 					// rel. Humidity
        #if USE_BMP085 > 0 || USE_LSM303D > 0 || USE_FM24V10 > 0
            /* 0D */wireless_putchar(general_error_code);	// general error code
        #else
            /* 0D */wireless_putchar(0);				// no general error code
        #endif
    #elif defined(GASSENSOR) || defined(STROMSENSOR)
        /* 09 */wireless_putchar(EXT_TYPE_COUNTER);		// Counter Frame type
        /* 0A */wireless_putchar(countervalue&0xff);	// Lower 4 Digits
        /* 0B */wireless_putchar(countervalue>>8);
        /* 0C */wireless_putchar(countervalueHi&0xff);	// Lower 4 Digits
        /* 0D */wireless_putchar(countervalueHi>>8);
        /* 0E */wireless_putchar(general_error_code);	// general error code
    #elif defined (ALARM) 
        /* 09 */wireless_putchar(EXT_TYPE_ALARM);		// Alarm Frame type
        /****C002****/
        #if DEBUG_MEM_CHECK > 0 
            uint16_t temp = get_mem_unused();
            /* 0A */wireless_putchar(temp >> 8); // Unused Memory Bytes
            /* 0B */wireless_putchar((uint8_t) temp & 0xff);
        #else
            /* 0A */wireless_putchar(0); 
            /* 0B */wireless_putchar(0);
        #endif
    #elif defined(NOEXTENSION)
        /* 09 */wireless_putchar(EXT_TYPE_NOTYPE); // No extension type
    #else
        #error "Extended type is not defined!"
    #endif
}

/*!
 *******************************************************************************
 *  \brief write an uint16_t to send buffer
 *******************************************************************************
 */ 
static void COM_write_u16(uint16_t w) {
    wireless_putchar(w>>8);
    wireless_putchar(w&0xff); 
}

/*!
 *******************************************************************************
 *  \brief helper function print version string
 *
 *  \note
 ******************************************************************************/
void COM_print_version(bool bToWireless) 
{
    char *s = VERSION_STRING "\n";
    
    DEBUG_PUTC('V');
    char c;
    for (c = *s; c; ++s, c = *s ) {
      DEBUG_PUTC(c);
      if (bToWireless) wireless_putchar(c);
    }
}

/*!
 *******************************************************************************
 *  \brief parse command from wireless
 *******************************************************************************
 */ 
void COM_wireless_command_parse (uint8_t * buf, uint8_t bufen) {
    register uint8_t pos=0;
    uint8_t c;
    register uint8_t current;
    register uint8_t next;

    while (bufen>pos) {
        c=buf[pos++];
        wireless_putchar(c|0x80);
        current = buf[pos];
        next    = buf[pos+1];
        switch(c) {
            case 'V':
                COM_print_version(true);
                break;
            case 'D':
                COM_print_debug(true);
                break;
            case 'C':		
                time_sync_tmo = 0;
                COM_print_debug(true);
                break;
            case 'G':
            case 'S':
                if (c=='S') Config_SetVal(current, next);
                wireless_putchar(current);
                if (current==0xff) {
                     wireless_putchar(EE_LAYOUT);
                } else if ( current == EEPROM_FORCE_RESET_IDX ) {
                        // Always return 0 as ForceReset-Value, 
                        // as thsi value will be set to zero at reset
                        wireless_putchar(0);
                } else {
                     wireless_putchar(Config_GetVal(current));
                }
                if (c=='S') pos++;
                pos++;
                break;
            case 'T':
                wireless_putchar(current);
                COM_write_u16(Get_watch(current));
                pos++;
            break;
            case 'B':
                if ((current==0x13) && (next==0x24)) {
                    COM_write_u16(0x1324);
                    TimerWatchdogReset(10);
                    /* previous call will not return! */
                }
                pos+=2;
            break;
            default:
            break;
       }
    } // while
}



/**
  * @}
  */
