/**
  ******************************************************************************
  * @file    storage_provider.h 
  * @author  Rainer
  * @brief   All methods a usb MSC class storage has to provide
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STPROVIDER_H
#define __STPROVIDER_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define ST_OK               0
#define ST_FAIL             (-1)

int8_t ST_Init              (uint8_t lun);
int8_t ST_GetCapacity       (uint8_t lun, uint32_t *block_num, uint16_t *block_size);
int8_t ST_IsReady           (uint8_t lun);
int8_t ST_IsWriteProtected  (uint8_t lun);
int8_t ST_Read              (uint8_t lun, uint32_t *buf, uint32_t blk_idx, uint32_t blk_count);
int8_t ST_Write             (uint8_t lun, uint32_t *buf, uint32_t blk_idx, uint32_t blk_count);
int8_t ST_GetMaxLun         (void);
extern int8_t ST_Inquirydata[36];

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STPROVIDER_H */

