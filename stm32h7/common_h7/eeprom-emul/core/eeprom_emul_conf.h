/**
  ******************************************************************************
  * @file    eeprom_emul_conf.h
  * @author  MCD Application Team
  * @brief   EEPROM emulation configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
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

/** @addtogroup EEPROM_Emulation
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_EMUL_CONF_H
#define __EEPROM_EMUL_CONF_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Private constants ---------------------------------------------------------*/

/* Configuration of eeprom emulation in QSPI flash, can be custom */
#if defined(STM32H742REF) 
    /* STM32H742REF has an 8MByte qspi flash on PCB */
    #define START_PAGE_ADDRESS      0x7F8000   /*!< Last 8 sectors of 4kiByte size each = 0x7f8000 .. 0x7fffff */
#elif defined(STM32H743EVAL2) 
    /* STM32H743EVAL2 has two 64MByte qspi flash on board, we use only one of these */
    #define START_PAGE_ADDRESS      0x3ff8000   /*!< Last 8 sectors of 4kiByte size each = 0x3ff8000 .. 0x3ffffff */
#else
    #error "No Start Page for EEPROM emulation set!"
#endif

#define CYCLES_NUMBER           1U   /*!< Number of 10Kcycles requested, minimum 1 for 10Kcycles (default),
                                        for instance 10 to reach 100Kcycles. This factor will increase
                                        pages number */
#define GUARD_PAGES_NUMBER      2U   /*!< Number of guard pages avoiding frequent transfers (must be multiple of 2): 0,2,4.. */

/* Configuration of crc calculation for eeprom emulation in flash */
#define CRC_POLYNOMIAL_LENGTH   LL_CRC_POLYLENGTH_16B /* CRC polynomial lenght 16 bits */
#define CRC_POLYNOMIAL_VALUE    0x8005U /* Polynomial to use for CRC calculation */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define NB_OF_VARIABLES         500U  /*!< Number of variables to handle in eeprom */


/* Select one to define a minimum voltage which will lead to suspend operation 
 * when actual voltage is lower than selected level. 
 * If no value is defined, low voltage monitoring is disabled */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_0    /*!< PVD threshold around 2.0 V */
#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_1    /*!< PVD threshold around 2.2 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_2    /*!< PVD threshold around 2.4 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_3    /*!< PVD threshold around 2.5 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_4    /*!< PVD threshold around 2.6 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_5    /*!< PVD threshold around 2.8 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_6    /*!< PVD threshold around 2.9 V */
//#define EEPROM_PVDLEVEL               PWR_PVDLEVEL_7    /*!< External input analog voltage (compared internally to VREFINT) */

#ifdef __cplusplus
}
#endif


#endif /* __EEPROM_EMUL_CONF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
