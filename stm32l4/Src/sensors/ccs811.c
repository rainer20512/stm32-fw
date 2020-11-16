/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bme280.c
* @date       2020-03-28
* @version    v3.5.0
*
*/

/*! @file bme280.c
 * @brief Sensor driver for BME280 sensor
 */

#include "config/config.h"

#if USE_CCS811 > 0

#define DEBUG_CCS811            1


#include "sensors/ccs811.h"

#if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
    #include "debug_helper.h"

    /* plain texts for CSS811 ERROR_ID bits in ascending bit order */
    const char * const ccs811_errtxt[]={
        "Invalid write addr", 
        "Invalid read addr", 
        "Invalid measure mode",
        "Resistor measurement out of range",
        "Heater fault", 
        "Heater voltage failure", 
        "Resvd.", 
        "Resvd.", 
    };    

    static const char* ccs811_GetErrTxt(uint32_t sel)
    {
      if ( sel < sizeof(ccs811_errtxt)/sizeof(char *) ) 
        return ccs811_errtxt[sel];
      else
        return "??? ";
    }

#endif




/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static int8_t null_ptr_check(const struct ccs811_dev *dev)
{
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
        /* Device structure pointer is not valid */
        return  CCS811_E_NULL_PTR;
    } else {
        /* Device structure is fine */
        return CCS811_OK;
    }
}


/****************** Global Function Definitions *******************************/

/*!
 * @brief readdata from the given register address of the sensor.
 */
static int8_t ccs811_read_reg(struct ccs811_dev *dev, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /* Check for null pointer in the device structure*/
    if ( len == 0 || reg_data == NULL || null_ptr_check(dev) != CCS811_OK )  return CCS811_E_NULL_PTR;

    /* Read the data  */
    dev->intf_rslt = dev->read(dev, reg_addr, reg_data, len);

    /* Check for communication error */
    if (dev->intf_rslt != CCS811_INTF_RET_SUCCESS) return CCS811_E_COMM_FAIL;

    return CCS811_OK;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
static int8_t ccs811_write_reg(struct ccs811_dev *dev, uint8_t reg_addr, const uint8_t *reg_data, uint16_t len )
{
    /* Check for arguments validity */
    if ( len == 0 || reg_data == NULL || null_ptr_check(dev) != CCS811_OK )  return CCS811_E_NULL_PTR;

     dev->intf_rslt = dev->write(dev, reg_addr, reg_data, len );

    /* Check for communication error */
    if (dev->intf_rslt != CCS811_INTF_RET_SUCCESS) return CCS811_E_COMM_FAIL;

    return CCS811_OK;
}

/*!
 * @brief This API writes just a register value w/o any data
 * ( Boot loader cmd APP_START and APP_VERIFY are these commands )
 */
static int8_t ccs811_write_zero(struct ccs811_dev *dev, uint8_t reg_addr )
{
    /* Check for arguments validity */
    if ( null_ptr_check(dev) != CCS811_OK )  return CCS811_E_NULL_PTR;

     dev->intf_rslt = dev->write(dev, reg_addr, NULL, 0 );

    /* Check for communication error */
    if (dev->intf_rslt != CCS811_INTF_RET_SUCCESS) return CCS811_E_COMM_FAIL;

    return CCS811_OK;
}

/**************************************************************************/
/*!
    @brief  sample rate of the sensor.
    @param  mode one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC,
   CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS.
*/
void ccs811_setDriveMode(struct ccs811_dev *dev, uint8_t mode) 
{
  SET_MEAS_MODE(dev->devData, mode);
  ccs811_write_reg(dev, CCS811_MEAS_MODE, &dev->devData._meas_mode,1);
}

/**************************************************************************/
/*!
    @brief  enable the data ready interrupt pin on the device.
*/
/**************************************************************************/
void ccs811_enableInterrupt(struct ccs811_dev *dev ) 
{
  dev->devData._meas_mode |= CCS811_MEAS_MODE_INT_DATARDY;
  ccs811_write_reg(dev, CCS811_MEAS_MODE, &dev->devData._meas_mode,1);
}

/**************************************************************************/
/*!
    @brief  disable the data ready interrupt pin on the device
*/
/**************************************************************************/
void ccs811_disableInterrupt(struct ccs811_dev *dev ) {
  dev->devData._meas_mode &= ~CCS811_MEAS_MODE_INT_DATARDY;
  ccs811_write_reg(dev, CCS811_MEAS_MODE, &dev->devData._meas_mode,1);
}


/**************************************************************************/
/*!
    @brief   read the status register and store in _status
    @returns the error bits from the status register of the device.
*/
/**************************************************************************/
void ccs811_readStatus(struct ccs811_dev *dev) 
{
  ccs811_read_reg(dev, CCS811_STATUS, &dev->devData._status, 1 );
}


/**************************************************************************/
/*!
    @brief   read the status register and store any errors.
    @returns the error bits from the status register of the device.
*/
/**************************************************************************/
int8_t ccs811_checkError(struct ccs811_dev *dev) 
{
  ccs811_readStatus(dev);

  /* If we have an error, read and store the error source ID */
  if ( dev->devData._status & CCS811_STATUS_ERROR ) {
    ccs811_read_reg(dev, CCS811_ERROR_ID, &dev->devData._error_id, 1 );
    return  CCS811_E_DEV_ERROR;
  }
  return CCS811_OK;
}

#if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
    void ccs811_printError (struct ccs811_dev *dev) 
    {
        uint8_t mask = 1;
        printf("CCS811 ERROR_ID=%02x, decoded:\n", dev->devData._error_id);
        for ( uint32_t i = 0; i < 8; i ++ ) {
            if ( dev->devData._error_id & mask ) {
                printf(" - %s\n", ccs811_GetErrTxt(i) );
            }
            mask <<= 1;
        }
    }
#endif

/**************************************************************************/
/*!
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
bool ccs811_dataAvailable(struct ccs811_dev *dev) 
{
  ccs811_readStatus(dev);
  return (dev->devData._status & CCS811_STATUS_DATA_READY) != 0;
}

/**************************************************************************/
/*!
    @brief  read and store the sensor data. This data can be accessed with
   getTVOC() and geteCO2()
    @returns CCS811_OK,         if no error, data si stored in _eCO2 and _TVOC
    @returns CCS811_E_NO_DATA   if no new data vailable
    @returns CCS811_E_DEV_ERROR if any error bit is set, _status and _error_id
                                are updated
    @returns CCS811_E_COMM_FAIL if i2c comm failed
*/
/**************************************************************************/
int8_t ccs811_readData(struct ccs811_dev *dev) 
{
    if (!ccs811_dataAvailable(dev)) {
        #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
            ccs811_checkError(dev);
            DEBUG_PUTS("CCS811: No new data available");
            DEBUG_PRINTF("CCS811 status: Status=0x%02x, Error=0x%02x", 
                          dev->devData._status,dev->devData._error_id );
            uint8_t mode;
            ccs811_read_reg(dev, CCS811_MEAS_MODE, &mode, 1 );
            DEBUG_PRINTF(", Mode=0x%02x\n", mode );
        #endif
        return CCS811_E_NO_DATA;
    } else {
        uint8_t buf[8];
        if ( ccs811_read_reg(dev, CCS811_ALG_RESULT_DATA, buf, 8) == CCS811_OK ) {
            dev->devData._eCO2     = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
            dev->devData._TVOC     = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);
            dev->devData._status   = buf[4];
            dev->devData._error_id = buf[5];
            #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
                DEBUG_PRINTF("CCS811 read: CO2=%d, TVOC=%d, Status=0x%02x, Error=0x%02x\n", 
                              dev->devData._eCO2,dev->devData._TVOC, dev->devData._status,dev->devData._error_id );
            #endif
            return ( dev->devData._status & CCS811_STATUS_ERROR ) != 0 ? CCS811_E_DEV_ERROR : CCS811_OK;
        } else {
           /* error during i2c read */
            #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
                DEBUG_PUTS("CCS811 read: I2C comm error");
            #endif
           return CCS811_E_COMM_FAIL;
        }
    } 
}



/**************************************************************************/
/*!
    @brief  trigger a software reset of the device
*/
/**************************************************************************/
int8_t ccs811_soft_reset_and_wait(struct ccs811_dev *dev)
{
    int8_t rslt;

    /* Reset requires a predefined sequence of data bytes to be submitted */
    uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};

    /* null ptr check will be performed in write_reg */
    rslt = ccs811_write_reg(dev,  CCS811_SW_RESET, seq, 4);

    /* Proceed if successful*/
    if (rslt == CCS811_OK) {
        /* Wait the neccessary time after (SW) reset */
        dev->delay_ms(100);
    }

    return rslt;
}

/**************************************************************************/
/*!
    @brief  Read all kinds if IDs and Version from chip and store in 
            dev-structure 
*/
/**************************************************************************/
int8_t ccs811_read_id(struct ccs811_dev *dev)
{
    int8_t rslt;

  /* first make sure, the HW is that of CCS811 Sensor  */
  rslt = ccs811_read_reg(dev, CCS811_HW_ID, &dev->hw_id, 1 );
  if ( rslt != CCS811_OK ) return rslt;
  if ( dev->hw_id != CCS811_HW_ID_CODE ) return CCS811_E_WRONG_TYPE;
  
  /* read HW version, bootloader and app version */
  rslt = ccs811_read_reg(dev, CCS811_HW_VERSION, &dev->hw_version, 1 );
  rslt = ccs811_read_reg(dev,CCS811_FW_BOOT_VERSION, (uint8_t*)&dev->boot_version, 2 );
  rslt = ccs811_read_reg(dev,CCS811_FW_APP_VERSION,  (uint8_t*)&dev->app_version,  2 );

  return rslt;
}

/**************************************************************************/
/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
    @retval  CSS811_OK -> Success.
    @retval  CCS811_E_DEV_ERROR -> Init ok, but device indicates error
                                   value is stored in devData._error_id
    @retval  CCS811_E_NULL_PTR     Some mandatory fn pointers are not set
    @retval  CCS811_E_NO_VALID_APP No valid app found in device 
    @retval  CCS811_E_DEV_NOT_FOUND No CCS811 sensor found
    @retval  CCS811_E_WRONG_MODE   Sensor not in App mode
*/
/**************************************************************************/
int8_t ccs811_init(struct ccs811_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    if ( null_ptr_check(dev) != CCS811_OK  )return CCS811_E_NULL_PTR;
     
    rslt = ccs811_soft_reset_and_wait(dev);
    if ( rslt != CCS811_OK ) return CCS811_E_DEV_NOT_FOUND;

    /* Read ID and Firmware version */
    rslt = ccs811_read_id (dev);
    if ( rslt != CCS811_OK ) return rslt;

    /* Read status and ensure, the APP_VALID status bit is set */
    rslt = ccs811_checkError(dev);
    if ( ( dev->devData._status & CCS811_STATUS_APP_VALID) == 0 ) return CCS811_E_NO_VALID_APP;

    // try to start the app
    ccs811_write_zero(dev, CCS811_BOOTLOADER_APP_START);
    dev->delay_ms(100);

    /* make sure there are no errors */
    rslt = ccs811_checkError(dev);
    if ( rslt != CCS811_OK ) return rslt;

    /* Ensure we are in app mode */
    if ( ( dev->devData._status & CCS811_STATUS_APP_MODE) == 0 ) return CCS811_E_WRONG_MODE;

    ccs811_disableInterrupt(dev);

    return CCS811_OK;
}

/*------------------------------------------------------------------------------
 * End of the CCS811  Driver
 *
 * Start of the THPSensor Wrapper
 *----------------------------------------------------------------------------*/

#include "sensors/thp_sensor.h"
#include "dev/i2c_abstract.h"

static struct ccs811_dev  ccs811Dev;

CCS811_INTF_RET_TYPE ccs811Read(struct ccs811_dev *dev, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    return SENSOR_IO_ReadMultiple(dev->i2c_addr, reg_addr, reg_data, (uint16_t)len ) == HAL_OK ? CCS811_INTF_RET_SUCCESS : CCS811_E_I2C_ERR;
}

CCS811_INTF_RET_TYPE ccs811Write(struct ccs811_dev *dev, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len )
{
    HAL_StatusTypeDef ret;

    if ( reg_data == NULL || len == 0 )
       ret = SENSOR_IO_WriteZero(dev->i2c_addr, reg_addr );
    else
       ret = SENSOR_IO_WriteMultiple(dev->i2c_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len );

    return ret  == HAL_OK ? CCS811_INTF_RET_SUCCESS : CCS811_E_I2C_ERR;
}

void ccs811Delayms(uint32_t period)
{
    HAL_Delay(period);
}


static uint32_t CCS811_GetCapability(void)
{
    /* Sensor may suppy CO2 and TVOC */
    return THPSENSOR_HAS_CO2 | THPSENSOR_HAS_TVOC;
}


/*******************************************************************************
 * Init CCS811 before first use
 * - read and check correct ID
 * - read and store the calibration data
 * - setup sampling parameters to "weather monitoring"
 * - set sensor mode to "forced", ie manual trigger a measuerment 
 * After correct initialization, use the macro BME280_IsUseable()
 * to check whether BMP085 is ready for use
 ******************************************************************************/
THPSENSOR_StatusEnum CCS811_Init(THPSENSOR_DecisTypeDef *Init)
{
    int8_t ccsInitResult;

    /* All sensor values will be returned with no decimal digits */
    Init->co2_decis  = 0;  // CO2 concentration in ppm
    Init->tvoc_decis = 0;  // TVOC will be returned in ppb

    ccs811Dev.i2c_addr  = CCS811_ADDRESS;
    ccs811Dev.read      = ccs811Read;
    ccs811Dev.write     = ccs811Write;
    ccs811Dev.delay_ms  = ccs811Delayms;
    ccs811Dev.flags     = 0;


    ccsInitResult = ccs811_init(&ccs811Dev);
    if ( ccsInitResult != CCS811_OK ) {
        #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
            switch ( ccsInitResult ) {
             case CCS811_E_DEV_NOT_FOUND:
                DEBUG_PUTS("No CCS811 Sensor found");
                break;
             case CCS811_E_NULL_PTR:
                DEBUG_PUTS("Mandatory data/functions unset");
                break;
             case CCS811_E_NO_VALID_APP:
                DEBUG_PUTS("No valid app in sensor");
                break;
             case CCS811_E_WRONG_MODE:
                DEBUG_PUTS("Error setting App mode");
                break;
             case CCS811_E_DEV_ERROR:
                ccs811_printError (&ccs811Dev); 
                break;
             default:
                DEBUG_PRINTF("Unknown return code %d\n", ccsInitResult);
            }
        #endif
        return THPSENSOR_ERROR;
    } 

    #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
        DEBUG_PRINTF("Found CCS811 sensor ID 0x%02x, Ver 0x%02x\n", ccs811Dev.hw_id,ccs811Dev.hw_version);
        DEBUG_PRINTF("Found CCS811 sensor BL Ver 0x%02x, App Ver 0x%02x\n", ccs811Dev.boot_version,ccs811Dev.app_version);
    #endif

    // default to read every min
    ccs811_setDriveMode(&ccs811Dev, CCS811_DRIVE_MODE_10SEC);

    return THPSENSOR_OK;
}
    
uint32_t CCS811_IsBusy(void)
{
    // RHB tbd
    return 0;
}

/******************************************************************************
 * Trigger a measurement. As the ccs811 sensor triggers its measurements 
 * automatically ( the measuerment period depends from the mode selected)
 * the only thing we do here is to readout actual sensor values ( if there are
 * any )
 *****************************************************************************/
THPSENSOR_StatusEnum CCS811_Measure(const uint32_t what)
{
    UNUSED(what);
    /* 
     * read data will either return CCS811OK, CCS811_E_NO_DATA, 
     * CCS811_E_DEV_ERROR or CCS811_E_COMM_FAIL
     * The only error conditions are the return of CCS811_E_DEV_ERROR or CCS811_E_COMM_FAIL.
     * even with CCS811_E_NO_DATA we will return THPSENSOR_OK. 
     * This will result in previous sensor data being read again
     */

    int8_t rslt = ccs811_readData(&ccs811Dev);

    if ( rslt == CCS811_OK || rslt == CCS811_E_NO_DATA )
        return THPSENSOR_OK;
    else 
        return THPSENSOR_ERROR;
}


/******************************************************************************
 * CCS811 does not have to be calibration. compensation values are read once
 * at initialization
 *****************************************************************************/

static THPSENSOR_StatusEnum CCS811_Calibrate(void)
{
    return THPSENSOR_OK;
}

int32_t CCS811_GetCO2(void)
{
    return (int32_t) ccs811Dev.devData._eCO2;
}
int32_t CCS811_GetTVOC(void)
{
    return (int32_t) ccs811Dev.devData._TVOC;
}

#if BME280_USE_PRESS > 0
int32_t BME280_GetPressure(void)
{ 
    return bme280_check_data_availability() ? comp_data.pressure : 0;
}
#endif

#if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
    const char * const status_txt[]={"Error", "Bit 1 - resvd", "Bit 2 - resvd", "Data ready ",
                                     "App valid", "Verify OK",  "Erase done", "App Mode"};
    static const char* get_status_txt(uint32_t sel)
    {
      if ( sel < sizeof(status_txt)/sizeof(char *) ) 
        return status_txt[sel];
      else
        return "???";
    }

    const char * const mode_txt[]={"Idle","Constant power 1s","Pulse heating 10s","LP pulse heating","Constant power 250ms, raw data"};
    static const char* get_mode_txt(uint32_t sel)
    {
      if ( sel < sizeof(mode_txt)/sizeof(char *) ) 
        return mode_txt[sel];
      else
        return "Illegal mode";
    }

    void CCS811_Diagnostics ( void )
    {
        int8_t rslt;
        uint8_t work, mask;
        uint8_t mode;

        /* Check for null pointer in the device structure*/
        rslt = null_ptr_check(&ccs811Dev);
        if ( rslt != CCS811_OK ) return; 

        /* Read status and error register, then dump status and error register, if error bit is set */
        ccs811_checkError( &ccs811Dev); 
        mask = 1;
        work = ccs811Dev.ccs811Data._status;
        DEBUG_PRINTF("CCS811 Status=0x%02x\n", work);
        for ( uint32_t i=0; i < 8; i++ ) {
            if ( work & mask ) {
                DEBUG_PRINTF("   %s\n", get_status_txt(i));
            }
            mask <<= 1;
        }
        if ( work & CCS811_STATUS_ERROR ) {
            ccs811_printError(&ccs811Dev);
        }
        ccs811_read_reg(&ccs811Dev, CCS811_MEAS_MODE, &ccs811Dev.devData._meas_mode, 1 );
        mode = ( ccs811Dev.devData._meas_mode >> CCS811_MEAS_MODE_Pos ) & CCS811_MEAS_MODE_Msk;
        DEBUG_PRINTF("Mode %d: %s\n", mode, get_mode_txt(mode) );
        DEBUG_PRINTF("DataRdy-Int: %s\n", mode & CCS811_MEAS_MODE_INT_DATARDY ? "On" : "Off" );
        DEBUG_PRINTF("Int Mode: %s\n", mode & CCS811_MEAS_MODE_INT_THRESH ? "Threshold" : "Normal" );

         
    }
#endif

const THPSENSOR_DrvTypeDef CCS811_Driver = {
        .Init           = CCS811_Init,
        .IsBusy         = CCS811_IsBusy,
        .GetCapability  = CCS811_GetCapability,
        .Calibrate      = CCS811_Calibrate,
        .TriggerMeasure = CCS811_Measure,
        .GetTRaw        = NULL,
        .GetHRaw        = NULL,
        .GetPRaw        = NULL,
        .GetCO2Raw      = CCS811_GetCO2, 
        .GetTVOCRaw     = CCS811_GetTVOC,
        .Diagnostics    =
            #if DEBUG_MODE > 0 && DEBUG_CCS811 > 0
                    CCS811_Diagnostics,
            #else   
                    NULL,
            #endif
};

#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////

#include "Adafruit_CCS811.h"

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is
   0x5A
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool Adafruit_CCS811::begin(uint8_t addr) {
  _i2caddr = addr;

  _i2c_init();

  SWReset();
  delay(100);

  // check that the HW id is correct
  if (this->read8(CCS811_HW_ID) != CCS811_HW_ID_CODE)
    return false;

  // try to start the app
  this->write(CCS811_BOOTLOADER_APP_START, NULL, 0);
  delay(100);

  // make sure there are no errors and we have entered application mode
  if (checkError())
    return false;
  if (!_status.FW_MODE)
    return false;

  disableInterrupt();

  // default to read every second
  setDriveMode(CCS811_DRIVE_MODE_1SEC);

  return true;
}


/**************************************************************************/
/*!
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
bool Adafruit_CCS811::available() {
  _status.set(read8(CCS811_STATUS));
  if (!_status.DATA_READY)
    return false;
  else
    return true;
}

/**************************************************************************/
/*!
    @brief  read and store the sensor data. This data can be accessed with
   getTVOC() and geteCO2()
    @returns 0 if no error, error code otherwise.
*/
/**************************************************************************/
uint8_t Adafruit_CCS811::readData() {
  if (!available())
    return false;
  else {
    uint8_t buf[8];
    this->read(CCS811_ALG_RESULT_DATA, buf, 8);

    _eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
    _TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);

    if (_status.ERROR)
      return buf[5];

    else
      return 0;
  }
}

/**************************************************************************/
/*!
    @brief  set the humidity and temperature compensation for the sensor.
    @param humidity the humidity data as a percentage. For 55% humidity, pass in
   integer 55.
    @param temperature the temperature in degrees C as a decimal number.
   For 25.5 degrees C, pass in 25.5
*/
/**************************************************************************/
void Adafruit_CCS811::setEnvironmentalData(uint8_t humidity,
                                           double temperature) {
  /* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
  default value is 50% = 0x64, 0x00. As an example 48.5%
  humidity would be 0x61, 0x00.*/

  /* Temperature is stored as an unsigned 16 bits integer in 1/512
  degrees; there is an offset: 0 maps to -25°C. The default value is
  25°C = 0x64, 0x00. As an example 23.5% temperature would be
  0x61, 0x00.
  The internal algorithm uses these values (or default values if
  not set by the application) to compensate for changes in
  relative humidity and ambient temperature.*/

  uint8_t hum_perc = humidity << 1;

  float fractional = modf(temperature, &temperature);
  uint16_t temp_high = (((uint16_t)temperature + 25) << 9);
  uint16_t temp_low = ((uint16_t)(fractional / 0.001953125) & 0x1FF);

  uint16_t temp_conv = (temp_high | temp_low);

  uint8_t buf[] = {hum_perc, 0x00, (uint8_t)((temp_conv >> 8) & 0xFF),
                   (uint8_t)(temp_conv & 0xFF)};

  this->write(CCS811_ENV_DATA, buf, 4);
}

/**************************************************************************/
/*!
    @deprecated hardware support removed by vendor
    @brief  calculate the temperature using the onboard NTC resistor.
    @returns temperature as a double.
*/
/**************************************************************************/
double Adafruit_CCS811::calculateTemperature() {
  uint8_t buf[4];
  this->read(CCS811_NTC, buf, 4);

  uint32_t vref = ((uint32_t)buf[0] << 8) | buf[1];
  uint32_t vntc = ((uint32_t)buf[2] << 8) | buf[3];

  // from ams ccs811 app note
  uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;

  double ntc_temp;
  ntc_temp = log((double)rntc / CCS811_REF_RESISTOR); // 1
  ntc_temp /= 3380;                                   // 2
  ntc_temp += 1.0 / (25 + 273.15);                    // 3
  ntc_temp = 1.0 / ntc_temp;                          // 4
  ntc_temp -= 273.15;                                 // 5
  return ntc_temp - _tempOffset;
}

/**************************************************************************/
/*!
    @brief  set interrupt thresholds
    @param low_med the level below which an interrupt will be triggered.
    @param med_high the level above which the interrupt will ge triggered.
    @param hysteresis optional histeresis level. Defaults to 50
*/
/**************************************************************************/
void Adafruit_CCS811::setThresholds(uint16_t low_med, uint16_t med_high,
                                    uint8_t hysteresis) {
  uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF),
                   (uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF),
                   hysteresis};

  this->write(CCS811_THRESHOLDS, buf, 5);
}

/**************************************************************************/
/*!
    @brief  trigger a software reset of the device
*/
/**************************************************************************/
void Adafruit_CCS811::SWReset() {
  // reset sequence from the datasheet
  uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
  this->write(CCS811_SW_RESET, seq, 4);
}

/**************************************************************************/
/*!
    @brief   read the status register and store any errors.
    @returns the error bits from the status register of the device.
*/
/**************************************************************************/
bool Adafruit_CCS811::checkError() {
  _status.set(read8(CCS811_STATUS));
  return _status.ERROR;
}

/**************************************************************************/
/*!
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void Adafruit_CCS811::write8(byte reg, byte value) {
  this->write(reg, &value, 1);
}

/**************************************************************************/
/*!
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
uint8_t Adafruit_CCS811::read8(byte reg) {
  uint8_t ret;
  this->read(reg, &ret, 1);

  return ret;
}

void Adafruit_CCS811::_i2c_init() {
  Wire.begin();
#ifdef ESP8266
  Wire.setClockStretchLimit(500);
#endif
}

void Adafruit_CCS811::read(uint8_t reg, uint8_t *buf, uint8_t num) {
  uint8_t value;
  uint8_t pos = 0;

  // on arduino we need to read in 32 byte chunks
  while (pos < num) {

    uint8_t read_now = min((uint8_t)32, (uint8_t)(num - pos));
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg + pos);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, read_now);

    for (int i = 0; i < read_now; i++) {
      buf[pos] = Wire.read();
      pos++;
    }
  }
}

void Adafruit_CCS811::write(uint8_t reg, uint8_t *buf, uint8_t num) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t *)buf, num);
  Wire.endTransmission();
}
#endif

#endif // #if USE_CCS811 > 0