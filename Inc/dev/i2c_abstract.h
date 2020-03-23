/*
 ******************************************************************************
 * @file    i2c_abstract.h
 * @author  rainer
 * @brief   abstract interface to I2C-Devices of different kind
 *          Implementer has to put functionality behind this functions
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_ABSTRACT_H
#define __I2C_ABSTRACT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

#ifdef HAL_I2C_MODULE_ENABLED

    typedef void ( *I2Cx_ErrorCB) (struct __I2C_HandleTypeDef *hi2c); 

    #include "stm32l4xx.h"
    #include "dev/i2c_dev.h"
    /* Link function for TOUCHSCREEN IO functions */
    I2C_HandleTypeDef*        TS_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb);
    void                      TS_IO_DeInit(void);
    void                      TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
    uint8_t                   TS_IO_Read(uint8_t Addr, uint8_t Reg);
    uint16_t                  TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
    void                      TS_IO_Delay(uint32_t Delay);

    /* Link function for EEPROM peripheral over I2C */
    I2C_HandleTypeDef*        EEPROM_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb);
    void                      EEPROM_IO_DeInit(void);
    HAL_StatusTypeDef         EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
    HAL_StatusTypeDef         EEPROM_IO_WriteDataIT(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
    HAL_StatusTypeDef         EEPROM_IO_WriteDataDMA(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
    HAL_StatusTypeDef         EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
    HAL_StatusTypeDef         EEPROM_IO_ReadRegister(uint16_t DevAddress, uint8_t specialAddress, uint8_t* pBuffer, uint32_t BufferSize);
    HAL_StatusTypeDef         EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

    /* Link function for Audio Codec peripheral */
    I2C_HandleTypeDef*        AUDIO_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb);
    void                      AUDIO_IO_DeInit(void);
    void                      AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
    uint16_t                  AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
    void                      AUDIO_IO_Delay(uint32_t delay);


    I2C_HandleTypeDef*        SENSOR_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb);
    void                      SENSOR_IO_DeInit(void);
    HAL_StatusTypeDef         SENSOR_IO_WriteDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length );
    HAL_StatusTypeDef         SENSOR_IO_ReadDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length);
    uint8_t                   SENSOR_IO_Read(uint16_t Addr, uint8_t Reg);
    HAL_StatusTypeDef         SENSOR_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
    HAL_StatusTypeDef         SENSOR_IO_ReadMultipleIT(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length, pI2C_CallbackTypeDef pCallback);
    void                      SENSOR_IO_Write( uint16_t Addr, uint8_t Reg, uint8_t Value);
    HAL_StatusTypeDef         SENSOR_IO_WriteMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
    HAL_StatusTypeDef         SENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
    void                      SENSOR_IO_Delay(uint32_t Delay);

#endif /* HAL_I2C_MODULE_ENABLED */


#ifdef __cplusplus
}
#endif

#endif /* I2C_ABSTRACT_H */
