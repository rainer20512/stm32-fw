/**
  ******************************************************************************
  * @file    ext_eeprom.h
  * @author  Rainer
  * @brief   Defines the interface of an external EEPROM (that is connected via
  *          SPI or I2C eg. 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXT_EEPROM_H
#define __EXT_EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif

#define FM24V10_I2C_ADDR        (0x50<<1)       // HAL wants left aligned I2C Adresses

/* Includes ------------------------------------------------------------------*/

#include "config/config.h"


/* Return types for various eeprom functions */
typedef enum {
   EEPROM_OK   = 0,   /*>!  No Errpr        */
   EEPROM_ERR  = 1,   /*>!  I2C Error       */
   EEPROM_BUSY = 2,   /*>!  Device busy     */
   EEPROM_PERR = 3,   /*>!  Parameter error */
} EEPROM_ResultT; 

typedef struct EITD {
  uint8_t UseHiSpeedMode;       /* != 0, if High Speed I2c Mode shall be enabled*/
} EEPROM_InitTypeDef;
/**
  * @}
  */

/** @defgroup TSENSOR_Driver_structure  Temperature Sensor Driver structure
  * @{
  */
typedef struct
{ 
    EEPROM_ResultT      (*Init)           (uint16_t I2CAddr, EEPROM_InitTypeDef *);
    uint8_t             (*IsReady)        (void);
    EEPROM_ResultT      (*ReadBuffer)     (uint32_t EepAddr, uint8_t *buffer, uint32_t readsize );
    EEPROM_ResultT      (*WriteBuffer)    (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
    EEPROM_ResultT      (*WriteBufferIT)  (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
    EEPROM_ResultT      (*WriteBufferDMA) (uint32_t EepAddr, uint8_t *buffer, uint32_t writesize );
    EEPROM_ResultT      (*GetID)          (uint8_t *retbuf, uint8_t *retsize, uint8_t bufsize );
    uint32_t            (*GetSize)        (void);
    uint32_t            (*GetBlockSize)   (void);
    EEPROM_ResultT      (*GotoSleep)      (void);

}EEPROM_DrvTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __EXT_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
