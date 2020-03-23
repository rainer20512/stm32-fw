/**
  ******************************************************************************
  * @file    fm24v10.c
  * @author  Rainer
  * @brief   Access functions for FM24V10 FRAM. We use the eeprom access base
  *          functions for this
  *          Implements an interface as defined in "ext_eeprom.h"
  ******************************************************************************
  */

#include "config/config.h"
#include "config/devices_config.h"

#if USE_I2C > 0

#include "sensors/thp_sensor.h"
#include "ext_eeprom.h"
#include "dev/i2c_abstract.h"
#include "dev/devices.h"

#include <string.h>

/*
 * The Base Address is 0x50, there are two address pins that allow to change the base
 * address to 0x52, 0x54 or 0x56. So a maximum of four FRAM devices can be connected to one
 * I2C bus. 
 * As the FM24V10 has 1<<17 Bytes, but only 16 bit memory addressing, the LS address bit is 
 * used as 'bannk select, i.e. when using I2C address 0x50, this will select the lower bank,
 * while 0x51 will select the upper bank.
 */

#define FM24V10_SPECIAL_ADDR    0xF8        /*>! special I2C addr to perform meta functions */
#define FM24V10_BYTE_SIZE       (1U << 17)
#define FM24V10_ID_SIZE         3
#define FM24V10_MAX_TRIALS      20


/* --------------- Private Variables --------------------------------------------------------- */
static EEPROM_InitTypeDef*  myInitType;
static uint8_t              myI2cAddress; /*>! my (lower bank) I2C address */

/* --------------- Forward declarations ------------------------------------------------------ */
EEPROM_ResultT      FM24V10_Init           (uint16_t I2CAddr, EEPROM_InitTypeDef *);
uint8_t             FM24V10_IsReady        (void);
EEPROM_ResultT      FM24V10_ReadBuffer     (uint32_t EepAddr, uint8_t *buffer, uint32_t readsize );
EEPROM_ResultT      FM24V10_WriteBuffer    (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
EEPROM_ResultT      FM24V10_WriteBufferIT  (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
EEPROM_ResultT      FM24V10_WriteBufferDMA (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
EEPROM_ResultT      FM24V10_GetID          (uint8_t *retbuf, uint8_t *retsize, uint8_t bufsize );
uint32_t            FM24V10_GetSize        (void) { return FM24V10_BYTE_SIZE; }
uint32_t            FM24V10_GetBlockSize   (void) { return 0; }
EEPROM_ResultT      FM24V10_GotoSleep      (void);

/* Driver structure for the FM24V10 FRAM */
EEPROM_DrvTypeDef FM24V10_EEPROM =
{
  .Init           = FM24V10_Init,
  .IsReady        = FM24V10_IsReady,
  .ReadBuffer     = FM24V10_ReadBuffer,
  .WriteBuffer    = FM24V10_WriteBuffer,
  .WriteBufferIT  = FM24V10_WriteBufferIT,
  .WriteBufferDMA = FM24V10_WriteBufferDMA,
  .GetID          = FM24V10_GetID,
  .GetSize        = FM24V10_GetSize,
  .GetBlockSize   = FM24V10_GetBlockSize,
  .GotoSleep      = FM24V10_GotoSleep,
};

EEPROM_ResultT      FM24V10_Init        (uint16_t I2CAddr, EEPROM_InitTypeDef *myInit )
{
   myI2cAddress = I2CAddr;
   myInitType   = myInit;
   if (!EEPROM_IO_Init(&USER_I2C_HANDLE, NULL)) return EEPROM_ERR;

   return FM24V10_IsReady();
}

uint8_t             FM24V10_IsReady     (void)
{
    return ( HAL_OK == EEPROM_IO_IsDeviceReady(myI2cAddress, FM24V10_MAX_TRIALS) ? EEPROM_OK : EEPROM_ERR );
}

EEPROM_ResultT      FM24V10_ReadBuffer  (uint32_t EepAddr, uint8_t *buffer, uint32_t readsize )
{
    if ( EepAddr >=FM24V10_GetSize() ) return EEPROM_PERR;
    uint8_t myAddrPS = myI2cAddress + ( EepAddr >= (1<<16) ? 2 : 0 );
    uint16_t intAddr = EepAddr & 0xFFFF;
    if ( HAL_OK != EEPROM_IO_ReadData(myAddrPS, intAddr,buffer, readsize) ) return EEPROM_ERR;

    return EEPROM_OK;

}

EEPROM_ResultT      FM24V10_WriteBuffer (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize )
{
    if ( EepAddr >=FM24V10_GetSize() ) return EEPROM_PERR;
    uint8_t myAddrPS = myI2cAddress + ( EepAddr >= (1<<16) ? 2 : 0 );
    uint16_t intAddr = EepAddr & 0xFFFF;
    if ( HAL_OK != EEPROM_IO_WriteData(myAddrPS, intAddr, buffer, writesize) ) return EEPROM_ERR;

    return EEPROM_OK;
}

EEPROM_ResultT      FM24V10_WriteBufferIT (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize )
{
    if ( EepAddr >=FM24V10_GetSize() ) return EEPROM_PERR;
    uint8_t myAddrPS = myI2cAddress + ( EepAddr >= (1<<16) ? 2 : 0 );
    uint16_t intAddr = EepAddr & 0xFFFF;
    if ( HAL_OK != EEPROM_IO_WriteDataIT(myAddrPS, intAddr, buffer, writesize) ) return EEPROM_ERR;

    return EEPROM_OK;
}

EEPROM_ResultT      FM24V10_WriteBufferDMA (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize )
{
    if ( EepAddr >=FM24V10_GetSize() ) return EEPROM_PERR;
    uint8_t myAddrPS = myI2cAddress + ( EepAddr >= (1<<16) ? 2 : 0 );
    uint16_t intAddr = EepAddr & 0xFFFF;
    if ( HAL_OK != EEPROM_IO_WriteDataDMA(myAddrPS, intAddr, buffer, writesize) ) return EEPROM_ERR;

    return EEPROM_OK;
}

EEPROM_ResultT      FM24V10_GetID       (uint8_t *retbuf, uint8_t *retsize, uint8_t bufsize )
{
    if ( bufsize < FM24V10_ID_SIZE ) return EEPROM_PERR;

    if ( HAL_OK != EEPROM_IO_ReadRegister(FM24V10_SPECIAL_ADDR, myI2cAddress, retbuf, FM24V10_ID_SIZE) ) return EEPROM_ERR;

    *retsize = FM24V10_ID_SIZE;
    return EEPROM_OK;
}

EEPROM_ResultT      FM24V10_GotoSleep   (void)
{
    return EEPROM_OK;
}




#endif /* USE_I2C */