/**
  ******************************************************************************
  * @file    thp_sensor.h
  * @author  Rainer
  * @brief   Hardware independent interface for 
  *          temperature, humidity and pressure sensors
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THP_SENSOR_H
#define __THP_SENSOR_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************
 * Return status of function calls
 *****************************************************************************/
typedef enum {
  THPSENSOR_OK = 0,
  THPSENSOR_ERROR
}THPSENSOR_StatusEnum;

/******************************************************************************
 * Capability flags for different types of values the sensor may return
 *****************************************************************************/
#define THPSENSOR_HAS_T    (1<<0)   /* Sensor is able to measure temperature     */
#define THPSENSOR_HAS_H    (1<<1)   /* Sensor is able to measure rel. humidity   */
#define THPSENSOR_HAS_P    (1<<2)   /* Sensor is able to measure air pressure    */       
#define ALL_SENSOR_CHANNELS (THPSENSOR_HAS_T | THPSENSOR_HAS_H | THPSENSOR_HAS_P)

/******************************************************************************
 * Init structure to be returned by sensor initialization. It will hold 
 * information about the values the sensor may return, information how to
 * scale the returned raw values 
 * This structure is also used to set the desired scaling factor when
 * reading "cooked" sensor values
 * A value i > 0 means "returned value has to be divided by 10^i,
 * A value i < 0 means "returned value has to be multiplied by 10^i,
 *****************************************************************************/
typedef struct {
  int8_t t_decis;               /* number of decimals of raw temp value          */
  int8_t h_decis;               /* number of decimals of raw rel. humidity value */
  int8_t p_decis;               /* number of decimals of raw pressure value      */
}THPSENSOR_DecisTypeDef;


THPSENSOR_StatusEnum THPSENSOR_Init         (const THPSENSOR_DecisTypeDef *);
uint32_t             THPSENSOR_IsBusy       (void);
uint32_t             THPSENSOR_GetCapability(void);
THPSENSOR_StatusEnum THPSENSOR_Calibrate    (void);
THPSENSOR_StatusEnum THPSENSOR_Measure      (const uint32_t what);
int32_t              THPSENSOR_GetT         (void);
int32_t              THPSENSOR_GetH         (void);
int32_t              THPSENSOR_GetP         (void);

/******************************************************************************
 * The driver structure a specific hardware driver has to implement
 *****************************************************************************/
typedef struct {
    THPSENSOR_StatusEnum (*Init)          (THPSENSOR_DecisTypeDef *);
    uint32_t             (*IsBusy)        (void);
    uint32_t             (*GetCapability) (void);
    THPSENSOR_StatusEnum (*Calibrate)     (void);
    THPSENSOR_StatusEnum (*TriggerMeasure)(uint32_t what);
    int32_t              (*GetTRaw)       (void);
    int32_t              (*GetHRaw)       (void);    
    int32_t              (*GetPRaw)       (void);
} THPSENSOR_DrvTypeDef;

extern uint32_t thpsensorFlags;

#define THPSENSOR_FOUND_FLAG       (1<<0)				// No BMP085 Sensor found
#define THPSENSOR_CALIBRATED_FLAG  (1<<1)				// Calibration values have been successfully read. Do not use BMP085 unless this flag is set
#define THPSENSOR_ERROR_FLAG       (1<<2)				// Last TWI transfer was erroneeous
#define THPSENSOR_ALL_FLAGS        (THPSENSOR_FOUND_FLAG|THPSENSOR_CALIBRATED_FLAG|THPSENSOR_ERROR_FLAG) 
    

#define THPSENSOR_IsCalibrated() 	((thpsensorFlags & THPSENSOR_CALIBRATED_FLAG)!=0)
#define THPSENSOR_IsAvailable() 	((thpsensorFlags & THPSENSOR_FOUND_FLAG     )!=0)
#define THPSENSOR_IsUseable()           ((thpsensorFlags & THPSENSOR_ALL_FLAGS)==(THPSENSOR_FOUND_FLAG|THPSENSOR_CALIBRATED_FLAG))

void task_init_thp   ( void );
void task_handle_thp ( uint32_t arg );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __THP_SENSOR_H */
