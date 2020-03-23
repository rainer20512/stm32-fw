/**
  ******************************************************************************
  * @file    i2c_abstract.c
  * @author  Rainer, Reengineered from STM32L4 Framework example code
  * @brief   Abstraction layer for hardware specific I2C access functions
  *          
  */

/* Includes ------------------------------------------------------------------*/

#include "config/config.h"
#include "dev/devices.h"

#include "spi_abstract.h"
#include "debug_helper.h"

#ifdef __cplusplus
 extern "C" {
#endif




#if 0 // defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3)


#define SPIx_TIMEOUT_MAX                   3000

/**
  * @brief BSP I2C users
  */
#define ABS_SPI_NO_USER         0
#define ABS_SPI_MEM_USER        (1U << 0 )
#define ABS_SPI_PERIP_USER      (1U << 1 )
#define ABS_SPI_ALL_USERS       (SPI_PERIP_USER | SPI_MEM_USER)


/**
 * @brief BUS variables
 */
static SpiHandleT *mySpi =NULL;                            /*<! Device to use, is supplied by Init-function   */
static uint32_t mySpiUsers = ABS_SPI_NO_USER;              /*<! Set of actual Users of this I2C-Module        */

/**
  * @brief  Error Handler 
  * @note   Defined as a weak function to be overwritten by the application.
  * @retval None
  */
void __attribute__((weak)) Spi_Abstract_ErrorHandler(const char * const where) 
{
  DEBUG_PRINTF("I2Cx_%s failed!\n", where);
}


/*******************************************************************************
 *                              BUS OPERATIONS
 ******************************************************************************/

/******************************* I2C Routines**********************************/

/**
  * @brief Eval I2Cx Bus initialization
  * @retval None
  */
static SPI_HandleTypeDef* SPIx_Init(SpiHandleT *spi, SPIx_ErrorCB errCb, uint32_t user)
{
  if( !spi )
  {
    #if defined(BSP_USE_CMSIS_OS)
        /* Create semaphore to prevent multiple I2C access */
        osSemaphoreDef(ABS_I2C_SEM);
        MyI2cSemaphore = osSemaphoreCreate(osSemaphore(ABS_I2C_SEM), 1);
    #endif

    HAL_SPI_DeInit(&mySpi->hSpi);
    /* Init the I2C */
    mySpi = NULL;
    return NULL;
  } else {
    mySpi = spi;
    if ( errCb ) {
         #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            HAL_SPI_RegisterCallback(&mySpi->hSpi, HAL_SPI_ERROR_CB_ID, errCb)
         #endif
    }
  }
  /* Update BSP I2C users list */
  mySpiUsers |= user;
  return &mySpi->hSpi;
}


/**
  * @brief Eval I2Cx Bus deinitialization
  * @retval None
  */
static void SPIx_DeInit(uint32_t user)
{
  /* Update SPI users list */
  mySpiUsers &= ~(user);

  if((HAL_SPI_GetState(&mySpi->hSpi) != HAL_SPI_STATE_RESET) && (mySpiUsers == ABS_SPI_NO_USER))
  {
    /* Deinit the I2C */
    HAL_SPI_DeInit(&mySpi->hSpi);
 
    #if defined(BSP_USE_CMSIS_OS)
        /* Delete semaphore to prevent multiple I2C access */
        osSemaphoreDelete(MyI2cSemaphore);
    #endif
    mySpi->hSpi = NULL;
  }
}


    SPI_HandleTypeDef*        MEM_IO_Init           (SpiHandleT *spi, SPIx_ErrorCB errCb);
    void                      MEM_IO_DeInit         (void);
    void                      MEM_IO_Write          (uint8_t Addr, uint8_t Reg, uint8_t Value);
    uint8_t                   MEM_IO_Read           (uint8_t Addr, uint8_t Reg);
    uint16_t                  MEM_IO_ReadMultiple   (uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
    uint16_t                  MEM_IO_WriteMultiple  (uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

  #if defined(BSP_USE_CMSIS_OS)
    static osSemaphoreId MyI2cSemaphore;
  #endif

/** Implementation for SPI Memory Access *************************************/

/**
  * @brief  Initialize peripherals used by the I2C EEPROM driver.
  * @retval None
  */
SPI_HandleTypeDef* MEM_IO_Init(SpiHandleT *spi, SPIx_ErrorCB errCb);
{
  return SPIx_Init(spi, errCb, ABS_SPI_MEM_USER);
}

void MEM_IO_DeInit(void)
{
  SPIx_DeInit(ABS_I2C_EEPROM_USER);
}

/**
  * @brief  Write data to I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef MEM_IO_Write(uint8_t* pBuffer, uint16_t writeSize)
{
  return HAL_SPI_Transmit(pbuffer, writeSize, SPIx_TIMEOUT_MAX)
}

HAL_StatusTypeDef EEPROM_IO_WriteDataIT(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return I2Cx_WriteMultipleIT(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize);
}

HAL_StatusTypeDef EEPROM_IO_WriteDataDMA(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return I2Cx_WriteMultipleDMA(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize);
}

/**
  * @brief  Read data from I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_ReadMultiple(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Read data from I2C EEPROM Register
  * @param  DevAddress: Target device address
  * @param  MemAddress: Special register address
  * @param  pBuffer: Pointer to return data buffer
  * @param  BufferSize: Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_ReadRegister(uint16_t DevAddress, uint8_t specialAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_ReadMultiple(DevAddress, specialAddress, I2C_MEMADD_SIZE_8BIT, pBuffer, BufferSize));
}

/**
* @brief  Checks if target device is ready for communication.
* @note   This function is used with Memory devices
* @param  DevAddress: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
HAL_StatusTypeDef EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  return (I2Cx_IsDeviceReady(DevAddress, Trials));
}


#endif /* defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) */


