/**
  ******************************************************************************
  * @file   encoding.h
  * @author Rainer
  * @brief  exports for XTEA encode/decode and CMAC Algorithm
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODING_H
#define __ENCODING_H

#include "debug.h"


#ifdef __cplusplus
 extern "C" {
#endif

#define XTEA_ENC(dest, src, key)  encipher((uint32_t *)(dest), (uint32_t *)(src), (uint32_t *)(key))
#define XTEA_DEC(dest, src, key)  decipher((uint32_t *)(dest), (uint32_t *)(src), (uint32_t *)(key))
#define CMAC_BLOCK_SIZE 4


/* Exported functions ------------------------------------------------------------*/
void encoder_init(void);
void encipher ( uint32_t dest[2], uint32_t v[2], uint32_t const k[4]);
void decipher (uint32_t dest[2], uint32_t v[2], uint32_t const k[4]);
uint32_t cmac_calc (uint8_t* m, uint8_t bytes, uint8_t* data_prefix, uint32_t check);
void encrypt_decrypt (uint8_t* p, uint8_t len);

#if  !defined(NOENCRYPT) && DEBUG_DUMP_KEYS>0
  void crypto_dump(void);
#else
  #define crypto_dump()
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ENCODING_H */

/**
  * @}
  */
