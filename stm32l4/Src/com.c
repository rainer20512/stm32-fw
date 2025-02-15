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
#include "system/util.h"
#include "version.h"
#include "eeprom.h"
#include "watch.h"
#include "timer.h"

#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif

#if defined(MULTITEMP)
    #include "onewire.h"
#endif

#if USE_RFM12 > 0 || USE_RFM69 > 0
    #include "rfm/rfm.h"
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
#define EXT_TYPE_ENVIRONMENT    0x55
#define EXT_TYPE_MULTITEMP      0x56            // Lots of DS18X20 temperature sensors tied to one device ***** 009 *****/


void COM_print_debug(bool rfm_transmit) 
{
    uint16_t work;
    int16_t  t;

    DEBUG_PRINTF("D:%s", RTC_GetStrDateTime(NULL) );

    DEBUG_PRINTF(" %s", Get_FSK_status_text());
    #ifdef TX18LISTENER
          DEBUG_PUTC('/');
          StatusPrintOOKStatus();
    #endif

    #if defined(USER_ADC)
        DEBUG_PRINTF(" B: %04d ", ADC_GetVdda(&USER_ADC) );
    #endif

    #if defined(GASSENSOR)
        DEBUG_PRINTF(" V:%04d", pa7_average*2 );
    #endif
    #if defined(ZAEHLERV1) || defined(ZAEHLERV2)
        DEBUG_PRINTF(" Z:%05d%03d.%c" countervalueHi,countervalue/10, '0'+countervalue%10);
    #endif

    #if defined(TX18LISTENER) || defined(UNIVERSAL) || defined(PWM_DEVICE) 
        #if USE_THPSENSOR > 0
            DEBUG_PRINTF(" P:%04d", THPSENSOR_GetP_MSL()/10);
        #endif
    #endif

    #if defined(ENVIRONMENTAL)
        #if USE_THPSENSOR > 0
            DEBUG_PRINTF(" Q:%05d", THPSENSOR_GetCO2());
            DEBUG_PRINTF(" O:%05d", THPSENSOR_GetTVOC());
        #endif
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
    
    
    #if defined(TX18LISTENER) || USE_DS18X20 > 0 || USE_BME280 > 0 
        #if USE_DS18X20 > 0
            #if defined(MULTITEMP)
                /***** 009 ***** when multiple temp sensors are configured, the first one is coded here with 16 bit */
                t = 10*DS18X20_GetTemp(0);
            #else
                /***** 009 ***** Otherwise read the first one in the list of sensors */
                /***** 013 ***** if no DS18X20 sensor is found, then use BME280      */
                if (  DS18X20_Found() ) 
                    t = 10*DS18X20_GetOneTemp();
                else 
                    t = abstemp;
            #endif
        #else
            /* TX18LISTENER or BME280 w/o DS18X20 */
            t = abstemp;
        #endif
    #else 
        /* No temp sensor at all */
        t = 0;
    #endif

    /* 04 */wireless_putchar(t >> 8); // current temp
    /* 05 */wireless_putchar(t & 0xff);
    
    #if defined(GASSENSOR)
        /* 06 */wireless_putchar(pa7_average >> 7); // Vin * 2 
        /* 07 */wireless_putchar(pa7_average << 1 & 0xff);
    #else
        #if defined(USER_ADC)
            work = ADC_GetVdda(&USER_ADC);
            /* 06 */wireless_putchar(work>>8); 
            /* 07 */wireless_putchar(work&0xff);
        #else
            /* 06 */wireless_putchar(0); 
            /* 07 */wireless_putchar(0);
        #endif
    #endif

    /* 08 */wireless_putchar(time_sync_tmo);

    #if defined(TX18LISTENER) || defined(UNIVERSAL) || defined(PWM_DEVICE)
        /* 09 */wireless_putchar(EXT_TYPE_OUTDOOR); // Outdoor Frame type
        #if USE_THPSENSOR > 0
            { 
                /**** 06 **** do the pressure compensation to MSL */
                work = THPSENSOR_GetP_MSL();
                /* 0A */wireless_putchar(work >> 8); // Pressure
                /* 0B */wireless_putchar((uint8_t)work & 0xff);
            }
        #else
            /* 0A */wireless_putchar(0); // Pressure
            /* 0B */wireless_putchar(0);
        #endif
        #if defined(TX18LISTENER)
            /* 0C */wireless_putchar(relhum); 				// rel. Humidity
        #elif USE_THPSENSOR > 0
            // UNIVERSAL with THP_SENSOR, value is promille
            work = THPSENSOR_GetH() / 10;
            /* 0C */wireless_putchar(work);
        #else
            /* 0C */wireless_putchar(0); 	
        #endif
        #if USE_BMP085 > 0 || USE_LSM303D > 0 || USE_FM24V10 > 0 || USE_25X512_EEPROM > 0 || USE_OPTICAL> 0 || USE_OPTICAL_EMETER > 0
                /* 0D */wireless_putchar(general_error_code);	// general error code
        #else
                /* 0D */wireless_putchar(0);					// placeholder
        #endif
    #elif defined(ENVIRONMENTAL)
        /* 09 */wireless_putchar(EXT_TYPE_ENVIRONMENT); // Environmental Frame type
        #if USE_THPSENSOR > 0
            { 
                work = THPSENSOR_GetTVOC();
                /* 0A */wireless_putchar(work >> 8); // TVOC
                /* 0B */wireless_putchar((uint8_t)work & 0xff);
                work = THPSENSOR_GetCO2();
                /* 0C */wireless_putchar(work >> 8); // TVOC
                /* 0D */wireless_putchar((uint8_t)work & 0xff);
            }
        #else
            /* 0A */wireless_putchar(0); // TVOC
            /* 0B */wireless_putchar(0);
            /* 0C */wireless_putchar(0); // CO2
            /* 0D */wireless_putchar(0);
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
    #elif defined(MULTITEMP)
        /***** 009 ***** Multiple DS18X20 Sensors on 1Wire-lane. Up to 5 may be coded in RFM_frame */
        /* 09 */wireless_putchar(EXT_TYPE_MULTITEMP); // Multitemp Frame type
        #define MAXTEMPVALS         4               // max 4 temperature values may be transmitted, first one already coded
        /* 0A, 0B, 0C, 0D, 0E 0F */
        for ( work = 1; work < MAXTEMPVALS; work++ ) {
            t = ( work < ow_nSensors ? 10*DS18X20_GetTemp(work) : 0);
            wireless_putchar(t >> 8); 
            wireless_putchar(t & 0xff);
        }
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
 *  \brief helper function print one char of the version string
 *
 *  \note
 ******************************************************************************/
static void COM_print_vchar ( const char c, bool bToWireless)
{
    DEBUG_PUTC(c);
    if (bToWireless) wireless_putchar(c);
}

/*!
 *******************************************************************************
 *  \brief helper function print one portion of the whole version string
 *         written to serial in any case,
 *         written to wirelessif "bToWireless" is true
 *         "cAppendChar" is appended to string, if != '\0'
 *  \note
 ******************************************************************************/
static void COM_print_vstring ( char *s, bool bToWireless, char cAppendChar)
{
    for (char c = *s; c; ++s, c = *s ) {
      COM_print_vchar(c, bToWireless);
    }
    if ( cAppendChar ) COM_print_vchar(cAppendChar, bToWireless);

}

/*!
 *******************************************************************************
 *  \brief helper function print version string
 *
 *  \note  MAX string length to transmit via wireless is roundabout 55 bytes
 *         make sure not to exceed!
 ******************************************************************************/
void COM_print_version(bool bToWireless) 
{
    COM_print_vstring(APP_STRING, bToWireless, ' ');
    COM_print_vstring(MCU, bToWireless, ' ');
#if USE_RFM12 > 0 || USE_RFM69 > 0
    COM_print_vstring(rfm->name, bToWireless, ' ');
#endif
    /* '\n' is mandatory as last character when transmitting wireless, so append in any case */
    COM_print_vstring(BUILD_STRING, bToWireless,'\n');
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
