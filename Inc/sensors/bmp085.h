/*
 ***************************************************************
 * TWI Higher level functions
 ***************************************************************
 */

#pragma once // multi-iclude prevention. gcc knows this pragma

#include "sensors/thp_sensor.h"


#define BMP058STATUS_FINISHED				 0x80 // State Machine has singaled finish or continue elsewhere
#define BMP058STATUS_TWIERROR				 0x81 // TWI-Module returned an error
#define BMP058STATUS_TIMER                               0x82 // TWI-Module timer event
#define BMP058STATUS_CONTINUE				 0x83 // Continue with state  "_bmp085State" 
#define BMP058STATUS_TIMEOUT				 0x84 // The Slave did not respond within a certain interval, assume no slave present


THPSENSOR_StatusEnum BMP085_SoftReset(void);

extern uint16_t bmp085_pressure;      			// pressure in hPA * 10 valid after succesful call of BMP085_GetPressure
// extern int16_t  hpa_average;                            // average of "bmp085_pressure" 
extern const THPSENSOR_DrvTypeDef BMP085_Driver;
