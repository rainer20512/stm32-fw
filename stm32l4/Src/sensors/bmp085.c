/*
 ***************************************************************
 * TWI Higher level functions
 ***************************************************************
 */

#include "config/config.h"

#if USE_BMP085 > 0

#include "dev/i2c_abstract.h"
#include "sensors/thp_sensor.h"
#include "sensors/bmp085.h"
#include "rtc.h"
#include "timer.h"
#include "eeprom.h"

#include "debug_helper.h"

#define BMP085_TWI_ADDR 	0x77			    // 7-bit I2C-Address -> EE for write, EF for read

#define BMP085_CAL_REGISTER     0xAA                        // First register address for calibration data
#define BMP085_CAL_SIZE 	(0xC0-BMP085_CAL_REGISTER)  // Block ends at 0xC0, size is specified in BYTES, not WORDS!

#define BMP085_TEM_REG_W 	0xF4                        // Register Idx for TempReadout-Command
#define BMP085_TEM_REG_R 	0xF6			    // Register Idx to readout raw temperature
#define BMP085_TEM_VALUE 	0x2E			    // Register value for "GetTemp"
#define BMP085_TEM_STATE 	10			    // SM-Start for reading calibration data
#define BMP085_TEMP_WAITMS      8			    // Wait 8 ms before reading temp value

#define OSS 0                                               // Oversampling Setting (note: code is not set up to use other OSS values)

#define BMP085_PR_REG_W 	0xF4                        // Register Idx for PressureReadout-Command
#define BMP085_PR_REG_R 	0xF6			    // Register Idx to readout raw temperature
#define BMP085_PR_VALUE 	0x34 + ( OSS << 6 )         // Register value for "GetPressure"
#define BMP085_PR_STATE 	20                          // SM-Start for reading calibration data
#if OSS == 0 || OSS == 1 
	#define BMP085_PR_WAITMS    8                       // Wait 8 ms before reading temp value
#elif OSS == 2
	#define BMP085_PR_WAITMS    16                      // Wait 16 ms before reading temp value
#elif
	#define BMP085_PR_WAITMS    28                      // Wait 28 ms before reading temp value
#else
	#error "BMP085: Invalid Value for OSS"
#endif

#define BMP085_SOFT_RESET_REG		0xE0		    // 	
#define BMP085_SOFT_RESET_ARG		0xB6
#define BMP085_SOFT_RESET_STATE 	200                 // SM-Start for reading calibration data

#define BMP085_CHIP_ID_REG		0xD0
#define BMP085_VERSION_REG		0xD1
#define BMP085_ID_SIZE			2		    // Two Bytes: ID and Version
#define BMP085_CHIP_ID			0x55

uint8_t bmpbuf[BMP085_CAL_SIZE];                            // Buffer to communicate with BMP085. Max length is required to read calibration data block
uint8_t bmp085_chip_id;					    // ID of BMP085 ( should be 0x55 )
uint8_t bmp085_chip_vers;                                   // Chip version
uint8_t bmp085_busy;

int16_t hpa_average;


/**************************************************************
 * all the calibration parameters
 *************************************************************/
static int16_t ac1;
static int16_t ac2; 
static int16_t ac3; 
static uint16_t ac4;
static uint16_t ac5;
static uint16_t ac6;
static int16_t b1;
static int16_t b2;
static int16_t mb;
static int16_t mc;
static int16_t md;


static uint16_t ut;
static uint16_t up;
static int16_t bmp085_temp;			// T in °C * 10
uint16_t bmp085_pressure;      // pressure in hPA * 10
static int32_t x1, x2, b5, b6, x3, b3, p;
static uint32_t b4, b7;



uint8_t bmp085Flags;

/* Forward declarations ******************************************************/
static THPSENSOR_StatusEnum BMP085_Pressure(void);

#define SET_BUSY()  bmp085_busy = 1
#define CLR_BUSY()  bmp085_busy = 0

static void BMP085_CalcTemp(void)
{
    x1 = ((int32_t)ut - ac6) * ac5 >> 15;
    x2 = ((int32_t)mc << 11) / (x1 + md);
    b5 = x1 + x2;

    int32_t work = (b5 + 8) >> 4;
    if ( work > 0x010000L ) DEBUG_PUTS("Too big");
    bmp085_temp = (int16_t)work;

    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) DEBUG_PRINTF("Temp = %d\n",bmp085_temp);
    #endif

}

static void BMP085_CalcPressure(void)
{
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((int32_t) ac1 * 4 + x3) + 2)/4;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) up - b3) * (50000 >> OSS);
    p = b7 < 0x80000000L ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    bmp085_pressure = (uint16_t)((p + ((x1 + x2 + 3791) >> 4)+5)/10);
    bmp085_pressure += ((uint16_t)config.PressureComp)*10;
    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) {
            DEBUG_PRINTF("Press = %d\n",bmp085_pressure);
        }
    #endif
}

static void BMP085_ReadTempFinished(uint32_t bContinue)
{

    bmpbuf[0] = BMP085_TEM_REG_R;
    SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 1 );
    SENSOR_IO_ReadDirect (BMP085_TWI_ADDR, bmpbuf, 2 );
    ut = ((int16_t)bmpbuf[0])<<8 | bmpbuf[1];

    if ( bContinue ) {
        BMP085_Pressure();
    } else {
        BMP085_CalcTemp();
        CLR_BUSY();
    }
}


/*******************************************************************************
 * Trigger an BMP085 temperature or Temp/Pressure measurement
 ******************************************************************************/
THPSENSOR_StatusEnum BMP085_Temp(uint32_t bContinue)
{
    SET_BUSY();
    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) DEBUG_PUTS("Temperature:");
    #endif	

    bmpbuf[0] = BMP085_TEM_REG_W;
    bmpbuf[1] = BMP085_TEM_VALUE;
    if ( HAL_OK != SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 2 ) ) {
        #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
            DEBUG_PUTS("Error when starting BMP085 Temperature readout");
        #endif
        CLR_BUSY();
        return THPSENSOR_ERROR;
    }

    MsTimerSetRel(MILLISEC_TO_TIMERUNIT(BMP085_TEMP_WAITMS), 0, BMP085_ReadTempFinished, bContinue);
    return THPSENSOR_OK;
}

/*******************************************************************************
 * Read the pressure data after conversion and convert to hPa
 ******************************************************************************/
static void BMP085_ReadPressureFinished(uint32_t arg)
{
    UNUSED(arg);

    bmpbuf[0] = BMP085_PR_REG_R;
    SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 1 );
    SENSOR_IO_ReadDirect (BMP085_TWI_ADDR, bmpbuf, 2 );
    up = ((int16_t)bmpbuf[0])<<8 | bmpbuf[1];

    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) DEBUG_PRINTF("PressureFinished, up = %d\n", up);
    #endif

    // First do the Temp calculation
    BMP085_CalcTemp();

    // First do the Temp calculation
    BMP085_CalcPressure();
    CLR_BUSY();
}


static THPSENSOR_StatusEnum BMP085_Pressure(void)
{
    SET_BUSY();
    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) DEBUG_PUTS("Pressure:");
    #endif

    bmpbuf[0] = BMP085_PR_REG_W;
    bmpbuf[1] = BMP085_PR_VALUE;
    if ( HAL_OK != SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 2 ) ) {
        #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
            DEBUG_PUTS("Error when starting BMP085 Pressure readout");
        #endif
        CLR_BUSY();
        return THPSENSOR_ERROR;
    }

    MsTimerSetRel(MILLISEC_TO_TIMERUNIT(BMP085_PR_WAITMS), 0, BMP085_ReadPressureFinished, 0);
    return THPSENSOR_OK;
}

/*******************************************************************************
 * raw calibration data is stored in the calibration-Buffer. 
 * So read/convert it and store back into calibration buffer
 ******************************************************************************/
static THPSENSOR_StatusEnum BMP085_CalibrationFinished(void)
{
    uint8_t ptr=0;
    uint8_t err=0;

    /******************************************************
    * local helper function for reading calibration data. 
    * Any value reading 0x0000 or 0xffff indicates an error
    *****************************************************/
    uint16_t read_calibration_word(void)
    {
        // Byte orderis MSB/LSB
        uint16_t temp = bmpbuf[ptr++]<<8;
        temp |= bmpbuf[ptr++];
        if ( temp == 0x0000 || temp == 0xffff ) err = 1;
        return temp;
    }

    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) {
            DEBUG_PUTS("CalibrationFinished ");
            for ( uint8_t i=0; i < BMP085_CAL_SIZE>>1; i++ ) {
                print_hexXX(bmpbuf[i*2]);print_hexXX(bmpbuf[i*2+1]); DEBUG_PUTC(' ');
            }
            CRLF();
        }
    #endif 

    ptr = 0;
    ac1 = (int16_t)read_calibration_word();
    ac2 = (int16_t)read_calibration_word();
    ac3 = (int16_t)read_calibration_word();
    ac4 = (uint16_t)read_calibration_word();
    ac5 = (uint16_t)read_calibration_word();
    ac6 = (uint16_t)read_calibration_word();
    b1 = (int16_t)read_calibration_word();
    b2 = (int16_t)read_calibration_word();
    mb = (int16_t)read_calibration_word();
    mc = (int16_t)read_calibration_word();
    md = (int16_t)read_calibration_word();

    #if DEBUG_MODE && DEBUG_BMP085
    if ( debuglevel > 2 ) {
        DEBUG_PRINTF("\tAC1 = %d\n",ac1);
        DEBUG_PRINTF("\tAC2 = %d\n",ac2);
        DEBUG_PRINTF("\tAC3 = %d\n",ac3);
        DEBUG_PRINTF("\tAC4 = %u\n",ac4);
        DEBUG_PRINTF("\tAC5 = %u\n",ac5);
        DEBUG_PRINTF("\tAC6 = %u\n",ac6);
        DEBUG_PRINTF("\tB1  = %d\n",b1);
        DEBUG_PRINTF("\tB2  = %d\n",b2);
        DEBUG_PRINTF("\tMB  = %d\n",mb);
        DEBUG_PRINTF("\tMC  = %d\n",mc);
        DEBUG_PRINTF("\tMD  = %d\n",md);
        DEBUG_PRINTF("Calibration data %s\n",(err ? "not ok":"ok"));
    }
    #endif

    if ( err ) {
        // Flag error 
        general_error_code = GEN_ERR_BMP085_BAD_CALIBRATION;
        CTL_error |= ERR_GENERAL;
    } 
    return err==0 ? THPSENSOR_OK : THPSENSOR_ERROR;
}

/*******************************************************************************
 * Read the BMP085 calibration data block and
 * store into internal variables
 ******************************************************************************/
THPSENSOR_StatusEnum BMP085_Calibrate(void)
{
    THPSENSOR_StatusEnum  ret = THPSENSOR_ERROR;
    SET_BUSY();

    #if DEBUG_MODE && DEBUG_BMP085
        if ( debuglevel > 2 ) DEBUG_PUTS("Calibration:");
    #endif

    bmpbuf[0] = BMP085_CAL_REGISTER;
    if ( HAL_OK != SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 1 ) ) {
        #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
            DEBUG_PUTS("Error when setting BMP085 CAL register");
        #endif
    } else {
       if ( HAL_OK != SENSOR_IO_ReadDirect(BMP085_TWI_ADDR, bmpbuf, BMP085_CAL_SIZE ) ) {
            #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
                DEBUG_PUTS("Error when reading BMP085 calibration data");
            #endif
        } else {
            ret = BMP085_CalibrationFinished();
            #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
                if ( ret != THPSENSOR_OK ) DEBUG_PUTS("Invalid calibration data");
            #endif
        }
    }

    CLR_BUSY();
    return ret;
}


/*******************************************************************************
 * Read the BMP085 ID and HW version
 * and store into passed pointers
 ******************************************************************************/
 THPSENSOR_StatusEnum BMP085_GetIdVersion(uint8_t *id, uint8_t *version)
{
    THPSENSOR_StatusEnum ret = THPSENSOR_ERROR;
    SET_BUSY();

    //DEBUG_PRINTTS("Start\n");
    bmpbuf[0] = BMP085_CHIP_ID_REG;
    if ( HAL_OK != SENSOR_IO_WriteDirect(BMP085_TWI_ADDR, bmpbuf, 1 ) ) {
        #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
            DEBUG_PUTS("Error when setting BMP085 ID register");
        #endif
       goto id_err;
    }

    if ( HAL_OK != SENSOR_IO_ReadDirect(BMP085_TWI_ADDR, bmpbuf, BMP085_ID_SIZE ) ) {
        #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
            DEBUG_PUTS("Error when reading BMP085 ID and Version");
        #endif
        goto id_err;
    }

    *id      = bmpbuf[0];
    *version = bmpbuf[1];
    #if DEBUG_MODE > 0 && DEBUG_BMP085 > 0
        if ( *id != BMP085_CHIP_ID ) DEBUG_PRINTF("Wrong ID 0x%02x for BMP085, should be %0x02x\n", *id, BMP085_CHIP_ID);
    #endif
    //DEBUG_PRINTTS("Stop\n");
    ret = THPSENSOR_OK;

id_err:
    CLR_BUSY();
    return ret;
}

uint32_t BMP085_GetCapability(void)
{
    /* Sensor may suppy Temp ( chip temp ) and pressure */
    return THPSENSOR_HAS_T | THPSENSOR_HAS_P;
}

/*******************************************************************************
 * Init BMP085 before first use
 * - read and check correct ID
 * - read the calibration data
 * After correct initialization, use the macro BMP085_IsUseable()
 * to check whether BMP085 is ready for use
 ******************************************************************************/
THPSENSOR_StatusEnum BMP085_Init(THPSENSOR_DecisTypeDef *Init)
{
    bmp085Flags = 0;
    bmp085_busy = 0;

    /* Both values will return values with one decimal digit */
    Init->p_decis = 1;
    Init->t_decis = 1;

    /* Check correct ID */
    if ( THPSENSOR_OK != BMP085_GetIdVersion(&bmp085_chip_id, &bmp085_chip_vers) || bmp085_chip_id != BMP085_CHIP_ID ) {
        DEBUG_PUTS("No BMP085 Sensor found");
        return THPSENSOR_ERROR;
    } else {
        DEBUG_PRINTF("Found BMP085 V%02d\n",bmp085_chip_vers);
    }
    
    /* read calibration data */
    // DEBUG_PRINTTS("Start\n");
    BMP085_Calibrate();
    //DEBUG_PRINTTS("Stop\n");
    return THPSENSOR_OK;
}

uint32_t BMP085_IsBusy(void)
{
    return (uint32_t)bmp085_busy;
}

/******************************************************************************
 * Trigger a measurement. BMP085 has capability of temp and pressure measurement
 * it explicit temp measurement only is requested, do that, otherwise trigger
 * a pressure measurement, which implicitely requires/triggers a temp measurement
 *****************************************************************************/
THPSENSOR_StatusEnum BMP085_Measure(const uint32_t what)
{
    if ( what == THPSENSOR_HAS_T ) 
        return BMP085_Temp(0);
    else
        return BMP085_Temp(1);
}

int32_t BMP085_GetTemp(void)
{ 
    return bmp085_temp;
}

int32_t BMP085_GetPressure(void)
{ 
    return bmp085_pressure;
}

const THPSENSOR_DrvTypeDef BMP085_Driver = {
        BMP085_Init,
        BMP085_IsBusy,
        BMP085_GetCapability,
        BMP085_Calibrate,
        BMP085_Measure,
        BMP085_GetTemp,
        NULL,
        BMP085_GetPressure,
};
/*
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
*/
#endif //USE_BMP085

