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

#include "dev/i2c_abstract.h"
#include "debug_helper.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(BSP_USE_CMSIS_OS)
  #include "cmsis_os.h"
#endif

#define I2C_WRITE_ADDR(a)   (a<<1)
#define I2C_READ_ADDR(a)    ((a<<1)|0b1)

#ifdef HAL_I2C_MODULE_ENABLED
/**
  * @brief BSP I2C users
  */
#define ABS_I2C_NO_USER     0
#define ABS_I2C_AUDIO_USER  (1U << 0 )
#define ABS_I2C_TS_USER     (1U << 1 )
#define ABS_I2C_SENSOR_USER (1U << 2 )
#define ABS_I2C_EEPROM_USER (1U << 3 )
#define ABS_I2C_ALL_USERS   ( ABS_I2C_AUDIO_USER | ABS_I2C_TS_USER | ABS_I2C_SENSOR_USER | ABS_I2C_EEPROM_USER )


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define EVAL_I2Cx_TIMEOUT_MAX                   3000

/**
 * @brief BUS variables
 */
  static uint32_t hi2c_evalTimeout = EVAL_I2Cx_TIMEOUT_MAX;   /*<! Value of Timeout when I2C communication fails */
  static I2cHandleT *myI2c =NULL;                             /*<! Device to use, is supplied by Init-function   */
  static I2C_HandleTypeDef *myI2cHandle;
  static uint32_t v_bspI2cUsers = ABS_I2C_NO_USER;            /*<! Set of actual Users of this I2C-Module        */

  #if defined(BSP_USE_CMSIS_OS)
    static osSemaphoreId MyI2cSemaphore;
  #endif


/* 
 * Internal functions to Initialize and do the various HAL Read and Write calls 
 * It is assumed, that the HAL_I2C layer is compiled with USE_HAL_I2C_REGISTER_CALLBACKS=1
 */
static I2C_HandleTypeDef*   I2Cx_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb, uint32_t user);
static void                 I2Cx_DeInit(uint32_t user);
static HAL_StatusTypeDef    I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void                 I2Cx_Error (const char * const);
static void                 I2Cx_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value);
static HAL_StatusTypeDef    I2Cx_WriteMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static uint8_t              I2Cx_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize);
static HAL_StatusTypeDef    I2Cx_ReadMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);



/**
  * @brief  Error Handler 
  * @note   Defined as a weak function to be overwritten by the application.
  * @retval None
  */
void __attribute__((weak)) I2c_Abstract_ErrorHandler(const char * const where) 
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
static I2C_HandleTypeDef* I2Cx_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb, uint32_t user)
{
  if( !i2c )
  {
    #if defined(BSP_USE_CMSIS_OS)
        /* Create semaphore to prevent multiple I2C access */
        osSemaphoreDef(ABS_I2C_SEM);
        MyI2cSemaphore = osSemaphoreCreate(osSemaphore(ABS_I2C_SEM), 1);
    #endif

    HAL_I2C_DeInit(&myI2c->hI2c);
    /* Init the I2C */
    myI2c = NULL;
    myI2cHandle = NULL;
    return NULL;
  } else {
    myI2c = i2c;
    myI2cHandle = &myI2c->hI2c;
    HAL_I2C_Init(&myI2c->hI2c);
    if ( errCb ) 
         HAL_I2C_RegisterCallback(myI2cHandle, HAL_I2C_ERROR_CB_ID, errCb);
  }
  /* Update BSP I2C users list */
  v_bspI2cUsers |= user;
  return myI2cHandle;
}


/**
  * @brief Eval I2Cx Bus deinitialization
  * @retval None
  */
static void I2Cx_DeInit(uint32_t user)
{
  /* Update BSP I2C users list */
  v_bspI2cUsers &= ~(user);

  if((HAL_I2C_GetState(myI2cHandle) != HAL_I2C_STATE_RESET) && (v_bspI2cUsers == ABS_I2C_NO_USER))
  {
    /* Deinit the I2C */
    HAL_I2C_DeInit(myI2cHandle);
 
    #if defined(BSP_USE_CMSIS_OS)
        /* Delete semaphore to prevent multiple I2C access */
        osSemaphoreDelete(MyI2cSemaphore);
    #endif
    myI2cHandle = NULL;
  }
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @retval read register value
  */
static uint8_t I2Cx_ReadData(uint16_t Addr, uint16_t Reg, uint16_t RegSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t value = 0x0;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif

    status = HAL_I2C_Mem_Read(myI2cHandle, Addr, Reg, RegSize, &value, 1, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
        /* Execute user timeout callback */
        I2Cx_Error("ReadData");
    }
    return value;
}

/**
  * @brief  Read multiple data from a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  MemAddress: Memory address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif

    status = HAL_I2C_Mem_Read(myI2cHandle, I2C_READ_ADDR((uint8_t)Addr), Reg, MemAddress, Buffer, Length, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      I2Cx_Error("ReadMultiple");
    }
    return status;
}
/**
  * @brief  Register a User I2C Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_RegisterCallback(HAL_I2C_CallbackIDTypeDef CallbackID, pI2C_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_I2C_RegisterCallback(myI2cHandle, CallbackID, pCallback);
    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      I2Cx_Error("Register Callback");
    }
    return status;
}

/**
  * @brief  Write a sequence of bytes directly onto bus
  * @param  Addr: Device address on BUS
  * @param  Buffer: Pointer to the data buffer
  * @param  Length: Length of the data
  */
static HAL_StatusTypeDef I2Cx_WriteDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(myI2cHandle, I2C_WRITE_ADDR(Addr), Buffer, Length, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      I2Cx_Error("SetRegister8");
    }
    return status;
}

/**
  * @brief  Read multiple data directly (without addressing) from I2C device
  * @param  Addr: Device address on BUS
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  */
static HAL_StatusTypeDef I2Cx_ReadDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(myI2cHandle, I2C_READ_ADDR(Addr), Buffer, Length, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      I2Cx_Error("ReadDirect");
    }
    return status;
}

/**
  * @brief  Read multiple data from a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  MemAddress: Memory address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultipleIT(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
    HAL_StatusTypeDef status = HAL_OK;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif

    status = HAL_I2C_Mem_Read(myI2cHandle, Addr, Reg, MemAddress, Buffer, Length, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      I2Cx_Error("ReadMultiple");
    }
    return status;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  Value: The target register value to be written
  * @retval None
  */
static void I2Cx_WriteData(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(MyI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_Mem_Write(myI2cHandle, Addr, (uint16_t)Reg, RegSize, &Value, 1, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(MyI2cSemaphore);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error("WriteData");
  }
}


/**
  * @brief  Write multiple data value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address
  * @param  Buffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
    {
    HAL_StatusTypeDef status = HAL_OK;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif

    status = HAL_I2C_Mem_Write(myI2cHandle, I2C_WRITE_ADDR((uint8_t)Addr), (uint16_t)Reg, MemAddress, Buffer, Length, hi2c_evalTimeout);

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Re-Initiaize the BUS */
      I2Cx_Error("WriteMultiple");
    }
    
    return status;
}

/**
  * @brief  Write multiple data value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address
  * @param  Buffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2Cx_WriteMultipleIT(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
    {
    HAL_StatusTypeDef status = HAL_OK;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif
    status = HAL_I2C_Mem_Write_IT(myI2cHandle, Addr, (uint16_t)Reg, MemAddress, Buffer, Length );

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Re-Initiaize the BUS */
      I2Cx_Error("WriteMultiple");
    }
    
    return status;
}

/**
  * @brief  Write multiple data value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address
  * @param  Buffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2Cx_WriteMultipleDMA(uint16_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
    {
    HAL_StatusTypeDef status = HAL_OK;

    #if defined(BSP_USE_CMSIS_OS)
        /* Get semaphore to prevent multiple I2C access */
        osSemaphoreWait(MyI2cSemaphore, osWaitForever);
    #endif
    status = HAL_I2C_Mem_Write_DMA(myI2cHandle, Addr, (uint16_t)Reg, MemAddress, Buffer, Length );

    #if defined(BSP_USE_CMSIS_OS)
        /* Release semaphore to prevent multiple I2C access */
        osSemaphoreRelease(MyI2cSemaphore);
    #endif

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Re-Initiaize the BUS */
      I2Cx_Error("WriteMultiple");
    }
    
    return status;
}



/**
* @brief  Checks if target device is ready for communication.
* @note   This function is used with Memory devices
* @param  DevAddress: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(MyI2cSemaphore, osWaitForever);
#endif

  status = HAL_I2C_IsDeviceReady(myI2cHandle, DevAddress, Trials, hi2c_evalTimeout);

#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(MyI2cSemaphore);
#endif

  return status;
}


/**
  * @brief Eval I2Cx error treatment function
  * @retval None
  */
static void I2Cx_Error (const char * const where)
{
  uint32_t tmpI2cUsers;
  I2Cx_ErrorCB errCB;

  I2c_Abstract_ErrorHandler(where);

  /* De-initialize the I2C communication BUS */
  tmpI2cUsers = v_bspI2cUsers;
  errCB = myI2cHandle->ErrorCallback;
  I2Cx_DeInit(ABS_I2C_ALL_USERS);

  /* Re- Initiaize the I2C communication BUS */
  I2Cx_Init(myI2c, errCB, tmpI2cUsers);
}


/********************************* LINK TOUCHSCREEN *********************************/

/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
I2C_HandleTypeDef* TS_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb)
{
  return I2Cx_Init(i2c, errCb, ABS_I2C_TS_USER);
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  TS delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void TS_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initialize Audio low level.
  * @retval None
  */
I2C_HandleTypeDef* AUDIO_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb)
{
  return I2Cx_Init(i2c, errCb, ABS_I2C_AUDIO_USER);
}

/**
  * @brief  Deinitialize Audio low level.
  * @retval None
  */
void AUDIO_IO_DeInit(void)
{
  I2Cx_DeInit(ABS_I2C_AUDIO_USER);
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint16_t tmp = Value;

  Value = ((uint16_t)(tmp >> 8) & 0x00FF);
  Value |= ((uint16_t)(tmp << 8)& 0xFF00);

  I2Cx_WriteMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
  uint16_t Read_Value = 0, tmp = 0;

  I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Read_Value, 2);

  tmp = ((uint16_t)(Read_Value >> 8) & 0x00FF);
  tmp |= ((uint16_t)(Read_Value << 8)& 0xFF00);

  Read_Value = tmp;

  return Read_Value;
}

/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/********************************* LINK I2C EEPROM *****************************/
/**
  * @brief  Initialize peripherals used by the I2C EEPROM driver.
  * @retval None
  */
I2C_HandleTypeDef* EEPROM_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb)
{
  return I2Cx_Init(i2c, errCb, ABS_I2C_EEPROM_USER);
}

void EEPROM_IO_DeInit(void)
{
  I2Cx_DeInit(ABS_I2C_EEPROM_USER);
}

/**
  * @brief  Write data to I2C EEPROM driver
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return I2Cx_WriteMultiple(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize);
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


/********************************* LINK I2C Sensor *****************************/
I2C_HandleTypeDef* SENSOR_IO_Init(I2cHandleT *i2c, I2Cx_ErrorCB errCb)
{
  return I2Cx_Init(i2c, errCb, ABS_I2C_SENSOR_USER);
}

/**
  * @}
  */
void SENSOR_IO_DeInit(void)
{
  I2Cx_DeInit(ABS_I2C_SENSOR_USER);
}

/**
  * @brief  Set a Register address as a prerequisite for a successing read
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read later 
  */
HAL_StatusTypeDef SENSOR_IO_WriteDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
    return I2Cx_WriteDirect(Addr, Buffer, Length );
}

/**
  * @brief  Read multiple data directly (without addressing) from I2C device
  * @param  Addr: Device address on BUS
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
HAL_StatusTypeDef SENSOR_IO_ReadDirect(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
    return I2Cx_ReadDirect(Addr, Buffer, Length);
}


/**
  * @brief  Reads a single data.
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @retval Data to be read
  */
uint8_t SENSOR_IO_Read(uint16_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData (Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT );
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from Sensor in polling mdoe.
  * @param  Addr  I2C address
  * @param  Reg  Register address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval HAL status
  */
HAL_StatusTypeDef SENSOR_IO_ReadMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
    return I2Cx_ReadMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from Sensor in Interrupt mdoe.
  * @param  Addr  I2C address
  * @param  Reg  Register address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @param  pCallback Fn to call when transfer is complete
  * @retval HAL status
  */
HAL_StatusTypeDef SENSOR_IO_ReadMultipleIT(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length, pI2C_CallbackTypeDef pCallback)
{
    I2Cx_RegisterCallback(HAL_I2C_MEM_RX_COMPLETE_CB_ID ,pCallback);
    return I2Cx_ReadMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Writes a single data.
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @param  Value  Data to be written
  * @retval None
  */
void SENSOR_IO_Write( uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Value);
}

/**
  * @brief  Writes multiple data with I2C communication
  *         channel from MCU to TouchScreen.
  * @param  Addr  I2C address
  * @param  Reg  Register address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval None
  */
HAL_StatusTypeDef SENSOR_IO_WriteMultiple(uint16_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
   return I2Cx_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress  Target device address
  * @param  Trials  Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef SENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return I2Cx_IsDeviceReady(DevAddress, Trials);
  ;
}

/**
  * @brief  Delay function used in Sensor low level driver.
  * @param  Delay  Delay in ms
  * @retval None
  */
void SENSOR_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

#endif /*HAL_I2C_MODULE_ENABLED*/

