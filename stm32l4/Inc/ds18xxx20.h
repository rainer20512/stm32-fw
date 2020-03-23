/******************************************************************************
 * Implementation of DS18X20 functions via onewire driver layer
 *
 *****************************************************************************/

#ifndef __DS18X20_H_
#define __DS18X20_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DS18X20_OK,
    DS18X20_SM_ERROR,
    DS18X20_START_FAIL,
    DS18X20_ERROR_CRC,
    DS18X20_BADVALUE,
    DS18X20_NO_DEVICE,
    DS18X20_TIMEOUT,
    DS18X20_WORKING,
    DS18X20_NROF_ELEMENTS                        /* Keep this element as last entry in enum ! */

} DS18X20StateEnum;

#define DS18X20_ERRTXT \
{ \
  "No Error",               \
  "State machine error",    \
  "StartSM failed",         \
  "CRC error",              \
  "Bad Value",              \
  "NO Device found",        \
  "Comm. Timeout",          \
  "SM in progress",         \
}


#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE    0x00
#define DS18X20_POWER_EXTERN      0x01

#define DS18X20_CONVERSION_DONE   0x00
#define DS18X20_CONVERTING        0x01

/* DS18X20 specific values (see datasheet) */
#define DS18S20_FAMILY_CODE       0x10
#define DS18B20_FAMILY_CODE       0x28
#define DS1822_FAMILY_CODE        0x22

#define DS18X20_CONVERT_T         0x44
#define DS18X20_READ              0xBE
#define DS18X20_WRITE             0x4E
#define DS18X20_EE_WRITE          0x48
#define DS18X20_EE_RECALL         0xB8
#define DS18X20_READ_POWER_SUPPLY 0xB4

#define DS18B20_CONF_REG          4
#define DS18B20_9_BIT             0
#define DS18B20_10_BIT            (1<<5)
#define DS18B20_11_BIT            (1<<6)
#define DS18B20_12_BIT            ((1<<6)|(1<<5))
#define DS18B20_RES_MASK          ((1<<6)|(1<<5))

// undefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF        ((1<<0)|(1<<1)|(1<<2))
#define DS18B20_10_BIT_UNDF       ((1<<0)|(1<<1))
#define DS18B20_11_BIT_UNDF       ((1<<0))
#define DS18B20_12_BIT_UNDF       0

// conversion times in milliseconds
#define DS18B20_TCONV_12BIT       750
#define DS18B20_TCONV_11BIT       DS18B20_TCONV_12_BIT/2
#define DS18B20_TCONV_10BIT       DS18B20_TCONV_12_BIT/4
#define DS18B20_TCONV_9BIT        DS18B20_TCONV_12_BIT/8
#define DS18S20_TCONV             DS18B20_TCONV_12_BIT

// constant to convert the fraction bits to cel*(10^-4)
#define DS18X20_FRACCONV          625

// scratchpad size in bytes
#define DS18X20_SP_SIZE           9

// DS18X20 EEPROM-Support
#define DS18X20_WRITE_SCRATCHPAD  0x4E
#define DS18X20_COPY_SCRATCHPAD   0x48
#define DS18X20_RECALL_E2         0xB8
#define DS18X20_COPYSP_DELAY      10 /* ms */
#define DS18X20_TH_REG            2
#define DS18X20_TL_REG            3

#define DS18X20_DECIMAL_CHAR      '.'

// scratchpad size in bytes
#define DS18X20_SP_SIZE           9


typedef void (*DS18X20_TerminationCB) ( void );
typedef void (*DS18X20_ErrorCB)       ( DS18X20StateEnum, uint32_t );

extern uint32_t ds_ok_cnt, ds_bad_cnt, ds_err_cnt;

bool        DS18X20_Init            (void);
bool        DS18X20_Found           (void);
void        DS18X20_set_callbacks   (DS18X20_TerminationCB, DS18X20_ErrorCB);
void        DS18X20_start_meas      (bool bTriggerAll);
void        DS18X20_read_bitstatus  (void);
void        DS18X20_read_scratchpad (bool bReadAll);
int16_t     DS18X20_GetTemp         (void);
bool        DS18X20_GetTempStr      ( int16_t decicelsius, char str[], size_t n);

void        task_handle_ds          (uint32_t arg);
void        task_init_ds            (void);

#ifdef __cplusplus
}
#endif

#endif // __DS18X20_H
