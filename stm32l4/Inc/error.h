/**
  ******************************************************************************
  * @file    error.h
  * @author  Rainer
  * @brief   Functions for fault and error handling, assertions, ...
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERROR_H
#define __ERROR_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define E_NOERROR_VAL     0
#define E_SPI2ERROR_BIT   1

extern uint8_t CTL_error;

// ERRORS
#define ERR_BATT_LOW                    (1<<7)
#define ERR_BATT_WARNING                (1<<6)
#define ERR_SECFAIL                     (1<<5)    		// RHB Added: Timekeeping (ms->s) went wrong
#define ERR_RFM_SYNC                    (1<<4)
#if defined(TX18LISTENER)
    #define ERR_OOK_INOP                (1<<3)	  		// RHB Changed: Instead of ERR_MOTOR of original HR20	
#elif defined(ALARM)
    #define ERR_OVF_ALARM               (1<<3)	  		// Too many alarms per second
#elif  USE_DS18X20 > 0
    #define ERR_DS18X20_INOP            (1<<3)	  		// No DS18X20-Sensors found
#else
    #define ERR_UUU_UNUSED3             (1<<3)	  		// unused
#endif

#define ERR_GENERAL                     (1<<2)	  		// Some general error. Reason is set in error_code
#define ERR_RFM_INOP                    (1<<1)    		// RHB Added: RFM module seem to be inop ( torn off, e.g. )
#if defined(TX18LISTENER)
    #define ERR_SENSOR_BATT             (1<<0)	  		// Sensor Battery low
#elif USE_THPSENSOR > 0
    #define ERR_THPSENSOR               (1<<0)	  		// THP sensor measurement returned with errir    
#else
    #define ERR_UUU_UNUSED0             (1<<0)	  		// unused
#endif


#if USE_BMP085 > 0
    #define GEN_ERR_BMP085_SM                   17		 // BMP085 restarted while active
    #define GEN_ERR_BMP085_BAD_CALIBRATION      18		 // at least one of the bmp085 calibration items was 0x0000 or 0xffff
    #define GEN_ERR_BMP085_TIMEOUT		19               // Timeout when waiting for slave
#endif

#if USE_LSM303D > 0
	#define GEN_ERR_LSM303_SM		11		 // LSM303 SM restarted while active
	#define GEN_ERR_LSM303_TIMEOUT		12       // Timeout when waiting for LSM303D Slave
#endif

#if USE_FM24V10	> 0
	#define GEN_ERR_FM24V10_SM		13		 // FS24V10 SN restarted while active
	#define GEN_ERR_FM24V10_TIMEOUT		14		 // Timeout in i2c communication wirh F24v10 fram
	#define	GEN_ERR_FM24V10_TOO_LONG	15               // Block to be read or written too long to fit in TWI buffer
	#define	GEN_ERR_FM24V10_BUSY		16               // Block to be read or written too long to fit in TWI buffer
#endif

/* 
 * general error codes 128-255 are reserved for module specific error codes 
 * and are defined/maintained there
 */

#if USE_BMP085 > 0 || USE_LSM303D > 0 || USE_FM24V10 > 0 || USE_25X512_EEPROM > 0 || USE_OPTICAL> 0 || USE_OPTICAL_EMETER > 0
    extern uint8_t general_error_code;
#endif

void Error_Handler(char *file, int line);

#define log_error(...)   do { DEBUG_PRINTF("Error: "); DEBUG_PRINTF(__VA_ARGS__ ); CRLF(); } while (0)

#ifdef  USE_FULL_ASSERT
    void assert_failed(char* file, uint32_t line);
    #define assert(condition) if(!(condition)) assert_failed(__FILE__, __LINE__)
#else
    #define assert(a) (void)(a)
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ERROR_H */
