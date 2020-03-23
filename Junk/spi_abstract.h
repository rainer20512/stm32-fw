/**
  ******************************************************************************
  * @file    stm32l4r9i_eval.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L4R9I_EVAL's LEDs,
  *          push-buttons and COM ports hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_ABSTRACT_H
#define __SPI_ABSTRACT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

#ifdef HAL_SPI_MODULE_ENABLED

    typedef void ( *SPIx_ErrorCB) (struct __SPI_HandleTypeDef *hsp); 

    #include "stm32l4xx.h"
    #include "dev/spi.h"

    /* Link function for SPI memory functions */
    SPI_HandleTypeDef*        MEM_IO_Init           (SpiHandleT *spi, SPIx_ErrorCB errCb);
    void                      MEM_IO_DeInit         (void);
    void                      MEM_IO_Write          (uint8_t Addr, uint8_t Reg, uint8_t Value);
    uint8_t                   MEM_IO_Read           (uint8_t Addr, uint8_t Reg);
    uint16_t                  MEM_IO_ReadMultiple   (uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
    uint16_t                  MEM_IO_WriteMultiple  (uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

    /* Link function for SPI Peripheral functions */
    // tbd 

#endif /* HAL_SPI_MODULE_ENABLED */


#ifdef __cplusplus
}
#endif

#endif /* SPI_ABSTRACT_H */
