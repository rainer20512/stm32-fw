/**
  ******************************************************************************
  * @file    th_sensor.h
  * @author  Rainer
  * @brief   Hardware independent interface for temperature and humidity sensors
  *
  ******************************************************************************
  */

#include "config/config.h"
#include "config/devices_config.h"

#if USE_THPSENSOR > 0 

#include "sensors/thp_sensor.h"
#include "dev/i2c_abstract.h"
#include "dev/devices.h"
#include "debug_helper.h"
#include "system/periodic.h"

#if USE_BME280 > 0
    #include "sensors/bme280.h"
    #define THPSENSOR_DRIVER        BME280_Driver
#elif USE_BMP085 > 0
    #include "sensors/bmp085.h"
    #define THPSENSOR_DRIVER        BMP085_Driver
#elif USE_CCS811 > 0
    #include "sensors/ccs811.h"
    #define THPSENSOR_DRIVER        CCS811_Driver
#endif

#if USE_DISPLAY > 0
    #include "ui/lcd.h"
    #include "ui/lcd_status.h"
    #include "system/util.h"
#endif
#define DEBUG_THP   0

#define SET_SENSORFLAG(a)          thpsensorFlags |= a
#define CLR_SENSORFLAG(a)          thpsensorFlags &= ~a

uint32_t thpsensorFlags;

static const THPSENSOR_DrvTypeDef *thpsensor_drv;  
static THPSENSOR_DecisTypeDef conversiondecis;

/******************************************************************************
 * compute the conversion factor to convert the sensor decimals to the desired
 * user decimals. 
 * A value i < 0 means "returned value has to be divided by 10**i,
 * A value i > 0 means "returned value has to be multiplied by 10**i,
 *****************************************************************************/
static void SetDecis( const THPSENSOR_DecisTypeDef *user, const THPSENSOR_DecisTypeDef *sensor)
{
    conversiondecis.t_decis = user->t_decis - sensor->t_decis;
    conversiondecis.h_decis = user->h_decis - sensor->h_decis;
    conversiondecis.p_decis = user->p_decis - sensor->p_decis;
}

static bool IsSensorBusy(const char *actionstr)
{
    if ( thpsensor_drv->IsBusy() ) {
        #if DEBUG_MODE > 0 && DEBUG_THP > 0
            DEBUG_PRINTF("Sensor busy when trying to %s\n", actionstr);
        #else   
            UNUSED(actionstr);
        #endif
        return true;
    } else 
       return false;
}

static bool HasCapability( uint32_t capability )
{
    bool ret = ( thpsensor_drv->GetCapability() & capability) != 0;
    #if DEBUG_MODE > 0 && DEBUG_THP > 0
        if ( !ret ) DEBUG_PRINTF("Sensor lacks capability #%d\n", capability);
    #endif
    return ret;
}

static uint32_t pwr10 ( uint32_t exp )
{
    if ( exp >  9 ) return 1;

    uint32_t ret = 1;
    for ( uint32_t i = 0; i < exp; i++ )
        ret = ret * 10;

    return ret;
}

static int32_t DoScale ( int32_t raw, int8_t delta_decis )
{
    if ( delta_decis == 0 ) 
        return raw;
    else 
        if ( delta_decis > 0 )
            return raw * pwr10(delta_decis);
        else
            return raw / pwr10(-delta_decis);
}

/**
  * @brief  Initializes peripherals used by the I2C Temperature Sensor driver.
  * @retval TSENSOR status
  */
THPSENSOR_StatusEnum THPSENSOR_Init (const THPSENSOR_DecisTypeDef *userdecis)
{  
    THPSENSOR_DecisTypeDef sensordecis;
    #if defined(THPSENSOR_DRIVER)
        thpsensor_drv = &THPSENSOR_DRIVER;
    #else 
        #error "No THP-sensor low level driver assigned"
    #endif
    
    /* Check for I2c device being initialized correctly */
    if ( USER_I2C_HANDLE.hI2c.Instance == NULL ) return THPSENSOR_ERROR;

    /* Low level init */
    SENSOR_IO_Init( &USER_I2C_HANDLE, NULL );
    thpsensorFlags = 0;

    /* Init underlying sensor hardware and setup value scaling */   
    if ( THPSENSOR_OK == thpsensor_drv->Init(&sensordecis) ) {
        SET_SENSORFLAG(THPSENSOR_FOUND_FLAG);
        if ( !thpsensor_drv->Calibrate || thpsensor_drv->Calibrate() == THPSENSOR_OK ) {
            SET_SENSORFLAG(THPSENSOR_CALIBRATED_FLAG);
            SetDecis(userdecis, &sensordecis);
        }
    }

    if ( THPSENSOR_IsUseable() )
        return THPSENSOR_OK;
    else
        return THPSENSOR_ERROR;
}

uint32_t THPSENSOR_IsBusy(void)
{
    return thpsensor_drv->IsBusy();
}

uint32_t THPSENSOR_GetCapability(void)
{
    return thpsensor_drv->GetCapability();
}

THPSENSOR_StatusEnum THPSENSOR_Calibrate (void)
{
    if ( IsSensorBusy("calibrate") ) 
        return THPSENSOR_ERROR;
    else
        return thpsensor_drv->Calibrate();
}

THPSENSOR_StatusEnum THPSENSOR_Measure(const uint32_t what)
{
    if ( IsSensorBusy("measure") ) 
        return THPSENSOR_ERROR;
    else
        return thpsensor_drv->TriggerMeasure(what);
}


int32_t THPSENSOR_GetT (void)
{
    if ( !HasCapability(THPSENSOR_HAS_T) ) 
        return THPSENSOR_ERROR;
    else
        return DoScale(thpsensor_drv->GetTRaw(), conversiondecis.t_decis);
}

int32_t THPSENSOR_GetH (void)
{
    if ( !HasCapability(THPSENSOR_HAS_H) ) 
        return THPSENSOR_ERROR;
    else
        return DoScale(thpsensor_drv->GetHRaw(), conversiondecis.h_decis);
}

int32_t THPSENSOR_GetP (void)
{
    if ( !HasCapability(THPSENSOR_HAS_P) ) 
        return THPSENSOR_ERROR;
    else
        return DoScale(thpsensor_drv->GetPRaw(), conversiondecis.p_decis);
}

/**** 001 ****/
int32_t THPSENSOR_GetCO2 (void)
{
    if ( !HasCapability(THPSENSOR_HAS_CO2) ) 
        return THPSENSOR_ERROR;
    else
        return DoScale(thpsensor_drv->GetCO2Raw(), conversiondecis.co2_decis);
}

/**** 001 ****/
int32_t THPSENSOR_GetTVOC (void)
{
    if ( !HasCapability(THPSENSOR_HAS_TVOC) ) 
        return THPSENSOR_ERROR;
    else
        return DoScale(thpsensor_drv->GetTVOCRaw(), conversiondecis.tvoc_decis);
}

void THPSENSOR_Diagnostics(void)
{
    if ( thpsensor_drv->Diagnostics ) 
        thpsensor_drv->Diagnostics();
    else
        DEBUG_PRINTF("Diagnostics not implemented");
}

static void task_do_measure ( void *arg )
{
    THPSENSOR_Measure((uint32_t)arg);
}

static void task_do_display ( void *arg)
{
    UNUSED(arg);
     #if USE_DISPLAY > 0
        if ( HasCapability(THPSENSOR_HAS_P) ) {
            LCD_DisplayStatus(LCD_STATUS_PRESSURE);
        }
        if ( HasCapability(THPSENSOR_HAS_T) ) {
            abstemp = THPSENSOR_GetT();
            LCD_DisplayStatus(LCD_STATUS_TEMP);
        }
     #endif
}

void task_init_thp ( void )
{
    /* Init : Temp to deliver with 2 digits, other channels with one decimal digit, co2 ppm and tvoc ppb w/o any decimal digits */
    const THPSENSOR_DecisTypeDef Init = {2,1,1,0,0};

    if ( THPSENSOR_Init(&Init) == THPSENSOR_OK ) {
//        AtSecond(28, task_do_measure, (void *)ALL_SENSOR_CHANNELS, "THP sensor measure");
//        AtSecond(29, task_do_display, (void *)0, "THP sensor display");
        AtSecond(58, task_do_measure, (void *)ALL_SENSOR_CHANNELS, "THP sensor measure");
        AtSecond(59, task_do_display, (void *)0, "THP sensor display");
    } else {
        #if DEBUG_MODE > 0 && DEBUG_THP > 0
            DEBUG_PRINTF("THP Sensor init failed\n");
        #endif   
    }
}

/******************************************************************************
 * THP sensor doesnot have a task handler, everything to do is handeled by
 * periodic tasks.
 * the empty handler is neccessary, because a task handler must not be NULL
 *****************************************************************************/
void task_handle_thp( uint32_t arg )
{
    UNUSED(arg);
}

#endif // USE_THPSENSOR > 0