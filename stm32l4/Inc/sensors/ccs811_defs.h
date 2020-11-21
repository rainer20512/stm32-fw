
#ifndef CCS811_DEFS_H_
#define CCS811_DEFS_H_

#include <stdint.h>
#include <stddef.h>

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
    #define INT8_C(x)    S8_C(x)
    #define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
    #define INT16_C(x)   S16_C(x)
    #define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
    #define INT32_C(x)   S32_C(x)
    #define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
    #define INT64_C(x)   S64_C(x)
    #define UINT64_C(x)  U64_C(x)
#endif

/**\name C standard macros */
#ifndef NULL
    #ifdef __cplusplus
        #define NULL                             0
    #else
        #define NULL                             ((void *) 0)
    #endif
#endif

/********************************************************/

#ifndef TRUE
    #define TRUE                                  UINT8_C(1)
#endif
#ifndef FALSE
    #define FALSE                                 UINT8_C(0)
#endif

/**
 * CCS811_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef CCS811_INTF_RET_TYPE
    #define CCS811_INTF_RET_TYPE                  int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef CCS811_INTF_RET_SUCCESS
    #define CCS811_INTF_RET_SUCCESS               INT8_C(0)
#endif

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define CCS811_ADDRESS                            UINT8_C(0x5A)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {
  CCS811_STATUS =           UINT8_C(0x00),
  CCS811_MEAS_MODE =        UINT8_C(0x01),
  CCS811_ALG_RESULT_DATA =  UINT8_C(0x02),
  CCS811_RAW_DATA =         UINT8_C(0x03),
  CCS811_ENV_DATA =         UINT8_C(0x05),
  CCS811_NTC =              UINT8_C(0x06),
  CCS811_THRESHOLDS =       UINT8_C(0x10),
  CCS811_BASELINE =         UINT8_C(0x11),
  CCS811_HW_ID =            UINT8_C(0x20),
  CCS811_HW_VERSION =       UINT8_C(0x21),
  CCS811_FW_BOOT_VERSION =  UINT8_C(0x23),
  CCS811_FW_APP_VERSION =   UINT8_C(0x24),
  CCS811_ERROR_ID =         UINT8_C(0xE0),
  CCS811_SW_RESET =         UINT8_C(0xFF),
};

// bootloader registers
enum {
  CCS811_BOOTLOADER_APP_ERASE =  UINT8_C(0xF1),
  CCS811_BOOTLOADER_APP_DATA =   UINT8_C(0xF2),
  CCS811_BOOTLOADER_APP_VERIFY = UINT8_C(0xF3),
  CCS811_BOOTLOADER_APP_START =  UINT8_C(0xF4),
};

/**\name Sensor operating modes */
enum {
  CCS811_DRIVE_MODE_IDLE =       UINT8_C(0x00),
  CCS811_DRIVE_MODE_1SEC =       UINT8_C(0x01),
  CCS811_DRIVE_MODE_10SEC =      UINT8_C(0x02),
  CCS811_DRIVE_MODE_60SEC =      UINT8_C(0x03),
  CCS811_DRIVE_MODE_250MS =      UINT8_C(0x04),
};

/*=========================================================================*/

#define CCS811_HW_ID_CODE       0x81

#define CCS811_REF_RESISTOR     100000

/**\name API success code */
#define CCS811_OK                                 INT8_C(0)

/**\name API error codes */
#define CCS811_E_NULL_PTR                         INT8_C(-1)
#define CCS811_E_DEV_NOT_FOUND                    INT8_C(-2)
#define CCS811_E_DEV_ERROR                        INT8_C(-3)
#define CCS811_E_COMM_FAIL                        INT8_C(-4)
#define CCS811_E_NO_DATA                          INT8_C(-5)
#define CCS811_E_WRONG_TYPE                       INT8_C(-6)
#define CCS811_E_I2C_ERR                          INT8_C(-7)
#define CCS811_E_WRONG_MODE                       INT8_C(-8)
#define CCS811_E_NO_VALID_APP                     INT8_C(-9)

/**\name Operation status flags for CCS811 driver */
#define CCS811_FLAG_INITIALIZED                   ((uint8_t)( 1 << 0 ))
#define CCS811_FLAG_MEASURE                       ((uint8_t)( 1 << 1 ))  
#define CCS811_FLAG_DATA_READY                    ((uint8_t)( 1 << 2 ))



/*=========================================================================
      STATUS BITFIELDS
  -----------------------------------------------------------------------*/

#define CCS811_STATUS_ERROR             ( 1 << 0 )
#define CCS811_STATUS_DATA_READY        ( 1 << 3 )
#define CCS811_STATUS_APP_VALID         ( 1 << 4 )
// #define CCS811_STATUS_APP_VERIFY        ( 1 << 5 )
// #define CCS811_STATUS_APP_ERASE         ( 1 << 6 )
#define CCS811_STATUS_APP_MODE           ( 1 << 7 )

#define  SET_STATUS(ccsData, status) ( ccsData.status |= status )

/*=========================================================================
      Measurement and conditions register
  -----------------------------------------------------------------------*/

#define CCS811_MEAS_MODE_INT_THRESH      ( 1 << 2 )
#define CCS811_MEAS_MODE_INT_DATARDY     ( 1 << 3 )
#define CCS811_MEAS_MODE_Pos             4
#define CCS811_MEAS_MODE_Msk             0b111

#define SET_MEAS_MODE(ccsData,newMode) \
    do {\
        ccsData._meas_mode &= ~(CCS811_MEAS_MODE_Msk <<  CCS811_MEAS_MODE_Pos); \
        ccsData._meas_mode |= newMode << CCS811_MEAS_MODE_Pos; \
    } while(0)

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
struct ccs811_data
{
  uint16_t _TVOC;
  uint16_t _eCO2;
  uint8_t _status;   
  uint8_t _meas_mode;
  uint8_t _error_id;
};

/* forward declaration */
struct ccs811_dev;

/*!
 * @brief Type definitions
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef CCS811_INTF_RET_TYPE (*ccs811_read_fptr_t)(struct ccs811_dev *dev, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef CCS811_INTF_RET_TYPE (*ccs811_write_fptr_t)(struct ccs811_dev *dev, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);
/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in miilliseconds [ RHB changed to milliseconds ]
 *
 */
typedef void (*ccs811_delay_ms_fptr_t)(uint32_t period);


/*!
 * @brief css811 device structure
 */
struct ccs811_dev
{
    /* I2C address */
    uint8_t i2c_addr;

    /*< HW Id and version*/
    uint8_t hw_id, hw_version;

    /* FW boot and app version */
    uint16_t boot_version, app_version;

    /*< Read function pointer */
    ccs811_read_fptr_t read;

    /*< Write function pointer */
    ccs811_write_fptr_t write;

    /*< Delay function pointer */
    ccs811_delay_ms_fptr_t delay_ms;

    /*< Variable to store result of read/write function */
    CCS811_INTF_RET_TYPE intf_rslt;

    /*< copy of some internal registers and sensor data */
    struct ccs811_data devData;

    /*< CCS811 operational status flags */
    uint8_t flags;

};

#endif /* CCS811_DEFS_H_ */
