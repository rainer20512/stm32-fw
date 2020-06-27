/**
  ******************************************************************************
  * @file    EEPROM_Emul/Porting/STM32L4/flash_interface.c
  * @author  MCD Application Team
  * @brief   This file provides all the EEPROM emulation flash interface functions.
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

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"

#if USE_EEPROM_EMUL> 0

#include "../../core/eeprom_emul.h"
#include "flash_interface.h"
#include "dev/qspi_dev.h"

/* Private typedef -----------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

EE_ELEMENT_TYPE ElementRead( uint32_t Address )
{
    EE_ELEMENT_TYPE ret;
    QSpi_ReadWait(&QSPI_HND, (uint8_t*)&ret, Address, EE_ELEMENT_SIZE ); 
       
    return ret;
}

HAL_StatusTypeDef EE_FLASH_PROGRAM(uint32_t Address, uint64_t Data) 
{
    if ( QSpi_WriteWait(&QSPI_HND, (uint8_t*)&Data, Address,  EE_ELEMENT_SIZE ) )
        return HAL_OK;
    else
      return HAL_ERROR;
}


/**
  * @brief  Erase a page in polling mode
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status PageErase(uint32_t Page, uint16_t NbPages)
{
   uint32_t addr;
   addr = PAGE_ADDRESS(Page);
   if ( !QSpi_EraseSectorWait(&QSPI_HND, addr, NbPages) ) return EE_ERASE_ERROR;

  return EE_OK;
}

/**
  * @brief  Erase a page with interrupt enabled
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status PageErase_IT(uint32_t Page, uint16_t NbPages)
{
  return PageErase(Page, NbPages);
}



/**
  * @brief  Check if the configuration is 128-bits bank or 2*64-bits bank
  * @param  None
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status CheckBankConfig(void)
{
  return EE_OK;
}

/**
  * @brief  Programmable Voltage Detector (PVD) Configuration
  *         PVD set to level 6 for a threshold around 2.9V.
  * @param  None
  * @retval None
  */
void EE_PVD_Config(uint32_t pvd_level)
{
      PWR_PVDTypeDef sConfigPVD;
      
      sConfigPVD.PVDLevel = pvd_level;
      sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
      HAL_PWR_ConfigPVD(&sConfigPVD);

      /* Enable PVD */
      HAL_PWR_EnablePVD();

      /* Enable and set PVD Interrupt priority */
      HAL_NVIC_SetPriority(PVD_AVD_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(PVD_AVD_IRQn);
}

#endif // #if USE_EEPROM_EMUL> 0

