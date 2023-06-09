/******************************************************************************
 * Implementation of OneWire Protocol via USART
 *
 *****************************************************************************/

#ifndef __ONEWIRE_H_
#define __ONEWIRE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "config/config.h"


/*******************************************/
#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0

#define OW_SEARCH_FIRST 0xFF        // start new search
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE  0x00        // last device found

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8
#define MAXSENSORS 8


// Nrof Sensors and their IDs
extern uint8_t ow_nSensors;
extern uint8_t ow_SensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

typedef  void (*OneWireCB ) ( void );

void     OW_PostInit(const HW_DeviceType *dev, void *arg);
void     OW_PreDeInit(const HW_DeviceType *dev);
bool     ow_Initialized(void);

uint8_t  ow_rom_search(void);
void     ow_dump_rom(void);
void     ow_dump_one_rom( uint8_t *rom );

void     ow_set_callbacks(OneWireCB onCommDone, OneWireCB onTimeout);
bool     ow_reset_successful ( void );
void     ow_reset_with_cb(bool bWithTimeout);
void     ow_write_buffer_with_cb(bool bWithTimeout);
bool     ow_get_bitval(void);
void     ow_write_bit_with_cb(bool bitval, bool bWithTimeout);
void     ow_read_bit_with_cb(bool bWithTimeout);

void     ow_encoder_reset(void );
void     ow_encoder_preset( uint32_t len );
bool     ow_encode_byte( const uint8_t b );
bool     ow_encode_vector( uint8_t *in, uint32_t len );
void     ow_decode_byte ( uint8_t *byte, uint32_t ofs );
void     ow_decode_vector ( uint8_t *vector );
uint32_t ow_encoder_getbuflen(void);


void OWTEST_PostInit(const HW_DeviceType *dev, void *arg);
void test_write_buffer_with_cb(uint8_t len);
#ifdef __cplusplus
}
#endif

#endif // __ONEWIRE_H
