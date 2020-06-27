/**
  ******************************************************************************
  * @file    EEPROM_Emul/Porting/STM32L4/flash_interface.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the EEPROM
  *          emulation flash interface.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_INTERFACE_H
#define __FLASH_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

#define EEPROM_PAGE_SIZE        4096                    /*!< Size of one eeprom sector ( i.e. minimum eraseable unit )    */    
#define EEPROM_BANK_SIZE        4096                    
#define EE_ACCESS_32BITS    /*!< Enable EEPROM 32bits R/W functions, only valid for flash allowing 64bits access*/

/* Page state header */
#define EE_PAGESTAT_ERASED      (uint64_t)0xFFFFFFFFFFFFFFFFU  /*!< State saved in 1st,2nd,3rd,4th data type of page header */
#define EE_PAGESTAT_RECEIVE     (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 1st data type of page header */
#define EE_PAGESTAT_ACTIVE      (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 2nd data type of page header */
#define EE_PAGESTAT_VALID       (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 3rd data type of page header */
#define EE_PAGESTAT_ERASING     (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 4th data type of page header */

/* Description of the 8 Bytes (64 bits) element in flash   */
/* Bit:  63                  32  31      16  15         0  */
/*       <--- Data Value ----->  <-unused->  <-VirtAddr->  */
#define EE_ELEMENT_SIZE         8U                            /*!< Size of element in Bytes */
#define EE_ELEMENT_TYPE         uint64_t                      /*!< Type of element */
#define EE_VIRTUALADDRESS_TYPE  uint16_t                      /*!< Type of Virtual Address */
#define EE_VIRTUALADDRESS_SHIFT 0U                            /*!< Bits Shifting to get Virtual Address in element */
#define EE_CRC_TYPE             uint16_t                      /*!< Type of Crc */
#define EE_CRC_SHIFT            16U                           /*!< Bits Shifting to get Crc in element */
#define EE_DATA_TYPE            uint32_t                      /*!< Type of Data */
#define EE_DATA_SHIFT           32U                           /*!< Bits Shifting to get Data value in element */
#define EE_MASK_VIRTUALADDRESS  (uint64_t)0x000000000000FFFFU
#define EE_MASK_CRC             (uint64_t)0x00000000FFFF0000U
#define EE_MASK_DATA            (uint64_t)0xFFFFFFFF00000000U
#define EE_MASK_FULL            (uint64_t)0xFFFFFFFFFFFFFFFFU

/* Private functions ---------------------------------------------------------*/

EE_ELEMENT_TYPE ElementRead( uint32_t Address );
HAL_StatusTypeDef EE_FLASH_PROGRAM(uint32_t Address, uint64_t Data);
EE_Status PageErase(uint32_t Page, uint16_t NbPages);
EE_Status PageErase_IT(uint32_t Page, uint16_t NbPages);
EE_Status CheckBankConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_INTERFACE_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
