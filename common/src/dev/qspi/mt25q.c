/*******************************************************************************
 * @file    mt25q.c
 * @author  Rainer
 * @brief   Low level driver for MT25Q series Quad SPI flash memory
 *          works for 
 *          suitable code will be selected at runtime 
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "config/config.h"
#include "system/clockconfig.h"
#include "mt25q.h"
#include "dev/qspi_dev.h"
#include "dev/qspi/qspecific.h"

#include <stdio.h>

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#define MT25Q_QUAD_DISABLE       0x0
#define MT25Q_QUAD_ENABLE        0x1

#define MX25_HIGH_PERF_DISABLE  0x0
#define MX25_HIGH_PERF_ENABLE   0x1

#define MX25_DLY_TO_SLEEP       30              /* min time [us] the flash needs to enter deep sleep */
#define MX25_DLY_FM_SLEEP       45              /* min time [us] the flash needs to exit deep sleep  */
                                                /* this is the highr value when in high perf mode    */

/* Maximum frequency in Ultra low power mode */
#define QSPI_MAX_ULP_FREQUENCY  33000000

/* safe operating speed to readout ID values */
#define QSPI_INITIAL_SPEED      1000000   


/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
static uint8_t  MT25Q_GetStatus               (QSPI_HandleTypeDef *hqspi);
       bool     QSpecific_WriteEnable        (QSPI_HandleTypeDef *hqspi);
       bool     QSpecific_WaitForWriteDone   (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static bool     QSpecific_WaitForResetDone   (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static bool     MT25Q_QuadMode                (QSpiHandleT *myHandle, uint8_t Operation);
static bool     MT25Q_GetGeometry             (QSpiHandleT *myHandle);

/* Exported functions -------------------------------------------------------*/

bool QSpecific_HPerfMode ( QSPI_HandleTypeDef *hqspi, bool bHPerf )
{
   /* MT25Q has no low power/high perf. mode */
   UNUSED(hqspi); UNUSED(bHPerf);
    return 1;
}
#if DEBUG_MODE > 0

    const char * const dens_txt[]={"064 (8", "128 (16", "256 (32", "512 (64", "01G (128", "02G (256", };
    static const char* get_dens_txt(uint32_t sel)
    {
      if ( sel < sizeof(dens_txt)/sizeof(char *) ) 
        return dens_txt[sel];
      else
        return "Unknown";
    }

    const char * const drvstr_txt[]={"Resvd.", "90 Ohms", "Resvd.", "45 Ohms", "Resvd.", "20 Ohms", "Resvd.", "30 Ohms", }; 
    static const char* get_drvstr_txt(uint32_t sel)
    {
      if ( sel < sizeof(drvstr_txt)/sizeof(char *) ) 
        return drvstr_txt[sel];
      else
        return "Unknown";
    }

    const char * const wrapmode_txt[]={"mod 16", "mod 32", "mod 64", "no wrap" }; 
    static const char* get_wrapmode_txt(uint32_t sel)
    {
      if ( sel < sizeof(wrapmode_txt)/sizeof(char *) ) 
        return wrapmode_txt[sel];
      else
        return "Unknown";
    }

    /**************************************************************************************
     * Return the chip name as string-
     * idbuf has to contain the chip ID, which was read by CMD 0x90/0x9F
     * The Manufacurer ID is not checked here
     *************************************************************************************/
    char *QSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize)
    {
        char vChar;
        /* Voltage range, L=2.7-3.6V, U=1.7-2.0V */
        switch(idbuf[1]) {
            case 0xBA: vChar = 'L';break;
            case 0xBB: vChar = 'U';break;
            default: vChar = '?';
        }
        uint8_t nDens = idbuf[2];
        nDens = nDens - ( nDens >= 0x20 ? 0x1d : 0x17 );
        
        if ( bufsize < 25 ) return NULL;

        snprintf(retbuf,bufsize, "MT25Q%c%sMbyte)", vChar, get_dens_txt(nDens));
         
        return retbuf;
    
    }
#endif

/******************************************************************************
 * Setup the QSPI prescaler, so that the achieved operating freqency is at or
 * below ( due to the granularity of the prescaler ) "desired_frq"
 * This routine dowes not check for max. Frq. in Ultra low power mode ( 33MHz )
 * It is the callers responsibility to do so
 *****************************************************************************/
bool QSpecific_BasicInit(QSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size, bool bFirstInit )
{  
    UNUSED(bFirstInit);
    QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;

    /* calculate prescaler,so that the initial frequency is at or below max. ULP frequency */ 
    uint32_t prescaler = ( clk_frq + desired_frq - 1 ) / desired_frq;
    if ( prescaler > 0 ) prescaler--;
    desired_frq = clk_frq / (prescaler+1);

    if ( prescaler > 255 ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PRINTF("QSpecific_BasicInit - QSPI clk too low, minimum is %d\n", clk_frq/256 );
        #endif
        return false;
    }

    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        DEBUG_PRINTF("Qspi: Clk=%d\n", desired_frq );
    #endif

    /* QSPI initialization */
    hqspi->Init.ClockPrescaler     = prescaler;
    hqspi->Init.FifoThreshold      = 4;
    hqspi->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hqspi->Init.FlashSize          = POSITION_VAL(flash_size) - 1;
    hqspi->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    hqspi->Init.ClockMode          = QSPI_CLOCK_MODE_0;
    hqspi->Init.FlashID            = QSPI_FLASH_ID_1;
    hqspi->Init.DualFlash          = QSPI_DUALFLASH_DISABLE;


    if (HAL_QSPI_Init(hqspi) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpecific_BasicInit - Error: HAL Init failed");
        #endif
        return false;
    }

    /* Update Quadspi clkspeed to the actual speed*/
    prescaler = ( QUADSPI->CR & QUADSPI_CR_PRESCALER_Msk ) >> QUADSPI_CR_PRESCALER_Pos;
    myHandle->clkspeed = desired_frq;

    return true;
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi : QSPI handle
  * @retval None
  */
bool QSpecific_ResetMemory(QSpiHandleT *myHandle)
{
  QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
  QSPI_CommandTypeDef sCommand;

  /* Initialize the reset enable command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
 

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;


  /* Configure automatic polling mode to wait the memory is ready */  
  if ( !QSpecific_WaitForWriteDone(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) ) return false;

  return true;
}


/******************************************************************************
 * Return the flash memory chip ID
 * @param buf - writebuffer to write to ID to, requires at least 3 bytes 
 * @return writebuffer on success,
 *         NULL in case of failure
 * @note the buffer will contain 4 bytes with this info
 *       offset 0 : Manufacturer ID
 *       offset 1 : Memory type
 *       offset 2 : Memory density
 *       offset 3 : Device ID
 *****************************************************************************/
uint32_t QSpecific_GetID ( QSpiHandleT *me )
{
  QSPI_CommandTypeDef sCommand;

  /* Read 3 byte ID,  */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_ID_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 3;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure and read  */
  if (HAL_QSPI_Command(&me->hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return QSPI_ERROR;
  if (HAL_QSPI_Receive(&me->hqspi, me->id, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)    return QSPI_ERROR; 
  
  return QSPI_OK;
}

/******************************************************************************
 * Dump the status of Status and Configuration register
 *****************************************************************************/
void QSpecific_DumpStatusInternal(QSpiHandleT *myHandle)
{
    QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;
    QSPI_CommandTypeDef sCommand;
    uint8_t reg, dummy, sr[2];
    char txtbuf[25];
    bool ret;

    /* Read Status Register */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_STATUS_REG_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 1;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Dump Status register */
    ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    if (ret) ret = HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    QSpecific_GetChipTypeText(myHandle->id, txtbuf, 25 );

    printf("%s Status register\n", txtbuf );
    if ( ret ) {
        printf("   SR Write protect = %d\n",    reg & MT25Q_SR_SRWD ? 1 : 0 );
        printf("   Block protect    = 0x%1x\n", MT25Q_GET_BP_NORMALIZED(reg));
        printf("   Block protect dir: from %s\n",   reg & MT25Q_SR_BPDIR   ? "top" : "bottom" );
        printf("   Write Enable     = %d\n", reg & MT25Q_SR_WEL  ? 1 : 0 );
        printf("   Write in progess = %d\n", reg & MT25Q_SR_WIP  ? 1 : 0 );
    } else {
        puts("   Cannot read SR");
    }

    /* Dump flag status register */
     sCommand.Instruction       = READ_FLAG_STATUS_REG_CMD;

    ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    if (ret) ret = HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    printf("%s Flag status register\n", txtbuf );
    if ( ret ) {
        printf("   Mem modify done  = %d\n",    reg & MT25Q_FSR_PGM_DONE ? 1 : 0 );
        printf("   Erase suspended  = %d\n",    reg & MT25Q_FSR_ERA_SUSP ? 1 : 0 );
        printf("   Erase ERROR      = %d\n",    reg & MT25Q_FSR_ERA_ERR ? 1 : 0 );
        printf("   Program ERROR    = %d\n",    reg & MT25Q_FSR_PGM_ERR ? 1 : 0 );
        printf("   Program suspended= %d\n",    reg & MT25Q_FSR_PGM_SUSP ? 1 : 0 );
        printf("   Access ERROR     = %d\n",    reg & MT25Q_FSR_ACC_ERR ? 1 : 0 );
        printf("   Address bytes    = %d\n",    reg & MT25Q_FSR_4BYTE_MODE ? 4 : 3 );
    } else {
        puts("   Cannot read FSR");
    }
    /* Volatile Configuration Register */
    sCommand.Instruction = READ_VOL_CFG_REG_CMD;
    ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    if ( ret )  ret = HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    printf("%s Volatile configueration register \n", txtbuf );
    if ( ret ) {
        dummy = MT25Q_GET_DUMMYCYCLES_NORMALIZED(reg);
        if ( dummy == 0 ) dummy = 15;
        printf("   Dummy CLK cycles = %d\n",    dummy);
        printf("   XIP Read         = %s\n",    reg & MT25Q_EVCR_QUAD_DISABLED ? "Off" : "On" );
        printf("   Wrap mode        = %s\n",    get_wrapmode_txt(MT25Q_GET_WRAPMODE_NORMALIZED(reg)));
    } else {
        puts("   Cannot read VCR");
    }

    /* Enhanced Volatile Configuration Register */
    sCommand.Instruction = READ_ENHANCED_VOL_CFG_REG_CMD;
    ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    if ( ret )  ret = HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;

    printf("%s Enhanced volatile configueration register \n", txtbuf );
    if ( ret ) {
        printf("   QUAD I/O         = %s\n",    reg & MT25Q_EVCR_QUAD_DISABLED ? "Off" : "On" );
        printf("   DUAL I/O         = %s\n",    reg & MT25Q_EVCR_DUAL_DISABLED ? "Off" : "On" );
        printf("   DTR  I/O         = %s\n",    reg & MT25Q_EVCR_DTR_DISABLED  ? "Off" : "On" );
        printf("   Outp drv strngth = %s\n",    get_drvstr_txt(MT25Q_GET_DRV_STRENGTH_NORMALIZED(reg)));
    } else {
        puts("   Cannot read EVCR");
    }
}


/******************************************************************************
 * Hardware specific part of Quadspi initialization:
 * - set the flash size and timing parameters 
 * - reset flash controller
 *****************************************************************************/
bool QSpi_SpecificInit(const HW_DeviceType *self,QSpiHandleT *myHandle, uint32_t clk_frq)
{ 
    UNUSED(self);
    uint32_t clkspeed = myHandle->clkspeed;
    
    /* 
     * The initialization clock speed is the minimum of selected operating speed and QSPI_INITIAL_SPEED
     * which is a safe speed for initialization 
     */
    uint32_t init_speed = ( clkspeed < QSPI_INITIAL_SPEED ? clkspeed : QSPI_INITIAL_SPEED );

    /* Setup the deep sleep timing parameters, if deep sleep is supported */
    if ( myHandle->dsInfo ) {
        myHandle->dsInfo->dlyFmSleep = MX25_DLY_FM_SLEEP;
        myHandle->dsInfo->dlyToSleep = MX25_DLY_TO_SLEEP;
    }


    /* Basic Initialization with a safe speed and minimum flash size to readout ID data */
    if ( !QSpecific_BasicInit(myHandle, clk_frq, init_speed, MT25QY064_FLASH_SIZE, true) ) return false;


    /* QSPI memory reset */
    if (!QSpecific_ResetMemory(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: ResetMemory failed");
        #endif
    }

    /* read ID bytes */
    if ( QSpecific_GetID(myHandle) != QSPI_OK ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: Cannot read ID bytes");
        #endif
        return false;
    }

    /*
     * Do not enable quad mode, but keep device in SPI mode generally.
     * Read and write operations will have to switch to 4bit mode
     * manually.
     * To switch the device to permanent QUAD mode ( ie 4-4-4 operation for nearly
     * all commands and to switch back, the driver has to keep track of the selected
     * mode and setup the command, data and address slicing accordingly.
     */
#if 0
    /* QSPI quad enable */
    if ( !MT25Q_QuadMode(myHandle, MT25Q_QUAD_ENABLE) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: Quad mode not set");
        #endif
        return false;
    }
#endif

    /* Get geometry data */
    if ( !MT25Q_GetGeometry(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: Cannot set geometry");
        #endif
        return false;
    }

    QSpecific_DumpStatusInternal(myHandle);

 
    /* The operating speed is now set to user configured speed and flash size is set to correct size */
    if ( !QSpecific_BasicInit(myHandle, clk_frq, clkspeed, myHandle->geometry.FlashSize, false) ) return false;

  return true;
}

/**
  * @brief  This function enter the QSPI memory in deep power down mode.
  * @retval QSPI memory status
  */
bool QSpecific_EnterDeepPowerDown(QSpiHandleT *myHandle)
{

  /* check for deep sleep capbility */
  if (!myHandle->dsInfo ) return false;

  QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
  QSPI_CommandTypeDef sCommand;
  
  /* Initialize the deep power down command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = DEEP_POWER_DOWN_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    
  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  
  /* ---          Memory takes 10us max to enter deep power down          --- */
  /* --- At least 30us should be respected before leaving deep power down --- */

  myHandle->dsInfo->bIsDeepSleep = true;
  return true;
}

/**
  * @brief  This function leave the QSPI memory from deep power down mode.
  * @retval QSPI memory status
  */
bool QSpecific_LeaveDeepPowerDown(QSpiHandleT *myHandle)
{
  /* check for deep sleep capbility */
  if (!myHandle->dsInfo ) return false;

  QSPI_HandleTypeDef *hqspi       = &myHandle->hqspi;
  QSPI_CommandTypeDef sCommand;
  
  /* Send NoOp command to wake up */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = EXIT_POWER_DOWN_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    
  /* Send the command */
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  
  /* An exit power down command is sent to the memory, as the nCS should be low for at least 20 ns */
  /* On MX25R devices, the nCS signal going low is sufficient to wake up device                    */
  /* Memory takes 35us min to leave deep power down                                                */
  
  myHandle->dsInfo->bIsDeepSleep = false;
  return true;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
bool QSpecific_EnableMemoryMappedMode(QSpiHandleT *myHandle)
{
  QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;
  QSPI_CommandTypeDef      sCommand;
  QSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* Wake up device, if deep sleep capability and in in deep sleep */
  if ( myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep && ! QSpecific_LeaveDeepPowerDown(myHandle) ) return false;

  /* Configure the command for the read instruction */
  sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction        = QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD;
  sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize        = QSPI_ADDRESS_32_BITS;
  sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.DataMode           = QSPI_DATA_4_LINES;
  sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the memory mapped mode */
  sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  
  if (HAL_QSPI_MemoryMapped(hqspi, &sCommand, &sMemMappedCfg) != HAL_OK)
  {
    return false;
  }

  myHandle->bIsMemoryMapped = true;
  return true;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @retval QSPI memory status
  */
bool QSpecific_SuspendErase(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  
  /* Check whether the device is busy (erase operation is 
  in progress).
  */
  if (MT25Q_GetStatus(hqspi) == QSPI_BUSY)
  {
    /* Initialize the erase command */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = PROG_ERASE_SUSPEND_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    
    /* Send the command */
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
    
    if (MT25Q_GetStatus(hqspi) == QSPI_SUSPENDED) return true;
    
    return false;
  }
  
  return true;
}

/**
  * @brief  This function resumes a paused erase command.
  * @retval QSPI memory status
  */
bool QSpecific_ResumeErase(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  
  /* Check whether the device is in suspended state */
  if (MT25Q_GetStatus(hqspi) == QSPI_SUSPENDED)
  {
    /* Initialize the erase command */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = PROG_ERASE_RESUME_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    
    /* Send the command */
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
    
    /*
    When this command is executed, the status register write in progress bit is set to 1, and
    the flag status register program erase controller bit is set to 0. This command is ignored
    if the device is not in a suspended state.
    */
    
    if (MT25Q_GetStatus(hqspi) == QSPI_BUSY) return true;
    
    return false;
  }

  return true;
}

/******************************************************************************
 * Read Operation command
 *****************************************************************************/
bool QSpecific_ReadCMD(QSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size)
{
    QSPI_CommandTypeDef sCommand;

    /* Initialize the read command */
    sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction        = QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD;
    sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize        = QSPI_ADDRESS_32_BITS;
    sCommand.Address            = Addr;
    sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
    sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
    sCommand.DataMode           = QSPI_DATA_4_LINES;
    sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
    sCommand.NbData             = Size;
    sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;

    /* Configure the command */
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_Read - Error: Command not send");
        #endif
        return false;
    }

    return true;
}

/******************************************************************************
 * Enable Write and send write Operation command
 *****************************************************************************/
bool QSpecific_WriteCMD(QSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size)
{
    QSPI_CommandTypeDef sCommand;

    /* Initialize the program command */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_4_LINES;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    sCommand.Address           = Addr;
    sCommand.NbData            = Size;

    /* Enable write operations */
    if ( !QSpecific_WriteEnable(hqspi) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpecific_WriteCMD - Error: Write enable failed");
        #endif
      return false;
    }

    /* Configure the command */
        if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpecific_WriteCMD - Error: Command not send");
        #endif
        return false;
    }

    return true;
}

/******************************************************************************
 * return the maximum erase time ( according to data sheet ) and tpye of
 * erase
 *****************************************************************************/
bool QSpecific_GetEraseParams(uint32_t erasemode, uint32_t *timeout_ms, uint8_t *opcode)
{
    bool ret = true;
 
    switch(erasemode)
    {
        case QSPI_ERASE_SECTOR:
            *timeout_ms = MT25QY512_SECTOR_ERASE_MAX_TIME;
            *opcode     = SUBSECTOR_4K_ERASE_4_BYTE_ADDR_CMD;
            break;
        case QSPI_ERASE_SUBBLOCK:
            *timeout_ms = MT25QY512_SUBBLOCK_ERASE_MAX_TIME;
            *opcode     = SUBSECTOR_32K_ERASE_4_BYTE_ADDR_CMD;
            break;
        case QSPI_ERASE_BLOCK:
            *timeout_ms = MT25QY512_BLOCK_ERASE_MAX_TIME;
            *opcode     = SECTOR_ERASE_4_BYTE_ADDR_CMD;
            break;
        case QSPI_ERASE_ALL:
            *timeout_ms = MT25QY512_CHIP_ERASE_MAX_TIME;
            *opcode     = BULK_ERASE_CMD;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("GetEraseTimeout - Error: unkown erasemode %d\n", erasemode);
            #endif
            *timeout_ms = MT25QY512_SECTOR_ERASE_MAX_TIME;
            *opcode     = SECTOR_ERASE_CMD;
            ret = false;
    }
    return ret;
}


/**
  * @brief  Erases the specified block of the QSPI memory. 
  * @param  BlockAddress : Block address to erase  
  * @retval QSPI memory status
  */
bool QSpecific_EraseCMD(QSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t eraseMode )
{
    uint32_t tmo_unused;
    uint8_t  opcode;

    QSPI_CommandTypeDef sCommand;

    if ( !QSpecific_GetEraseParams(eraseMode, &tmo_unused, &opcode ) ) return false;

    /* Initialize the erase command */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = opcode;
    if ( eraseMode == QSPI_ERASE_ALL ) {
        sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    } else {
        sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
        sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
        sCommand.Address           = Address;
    }
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Enable write operations */
    if ( !QSpecific_WriteEnable(hqspi) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificEraseBlockWait - Error: Write enable failed");
        #endif
        return false;
    }

    /* Send the command */
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificEraseBlockWait - Error: send command failed");
        #endif
      return false;
    }

    return true;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


/******************************************************************************
 * Set the geometry data according to flash deviceID set in myHandle->id
 *****************************************************************************/
static bool MT25Q_GetGeometry(QSpiHandleT *myHandle)
{
    /* get density byte from ID string */
    uint8_t density = myHandle->id[2];
    uint8_t multiplier;
    uint32_t flash_size;

    if ( density >= 0x17 && density <= 0x19 ) {
            multiplier = density - 0x17;
    } else if ( density >= 0x20 && density <= 0x22 ) {
            multiplier = density - 0x1d;
    } else {
        #if DEBUG_MDOE > 0 && DEBBUG_QSPI > 0
            DEBUG_PRINTF("geometry data not defined for density byte 0x%02x\n", density);
        #endif
        return false;
    }

    flash_size = MT25QY064_FLASH_SIZE << multiplier;
    QSpi_SetGeometry ( &myHandle->geometry, flash_size, MT25QYXXX_PAGE_SIZE, MT25QYXXX_SECTOR_SIZE );
    return true;

}



/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
static uint8_t MT25Q_GetStatus(QSPI_HandleTypeDef *hqspi)
{
    QSPI_CommandTypeDef sCommand;
    uint8_t reg;

    /* Initialize the read of flag status register command */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_FLAG_STATUS_REG_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 1;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Read */
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return QSPI_ERROR;
    if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return QSPI_ERROR;

    /* Check the value of the register for PGM eor erase error */
    if ((reg & ( MT25Q_FSR_ERA_ERR |  MT25Q_FSR_PGM_ERR)) != 0) {
        return QSPI_ERROR;
    } else if ((reg & (MT25Q_FSR_ERA_SUSP | MT25Q_FSR_PGM_SUSP)) != 0) {
        return QSPI_SUSPENDED;
    }

    /* Check the value of the register */
    if ((reg & MT25Q_FSR_PGM_DONE) == 0) {
        return QSPI_BUSY;
    } else {
        return QSPI_OK;
    }
}



/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi : QSPI handle
  * @retval None
  */
bool QSpecific_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  
  /* Configure automatic polling mode to wait for write enabling */  
  sConfig.Match           = MT25Q_SR_WEL;
  sConfig.Mask            = MT25Q_SR_WEL;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    /* If during AutoPoll a timeout occurs, the State "HAL_QSPI_STATE_ERROR" will never be reset
       So do it manually here  */
    hqspi->State = HAL_QSPI_STATE_READY;
    return false;
  }
  return true;
}

/******************************************************************************
 * @brief  Perform an AutoPoll for Program/Erase bit of Flag Status register
 * @param  hqspi : QUADSPI HAL-Handle
 * @param  Timeout : Timeout for auto-polling
 * @retval true if bit was set before timeout
 *         false if timeout occured
 *****************************************************************************/
static bool QSpecific_WaitForResetDone(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
    QSPI_CommandTypeDef     sCommand;
    QSPI_AutoPollingTypeDef sConfig;

    /* Configure automatic polling mode to wait for memory ready */  
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_FLAG_STATUS_REG_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    sConfig.Match           = MT25Q_FSR_PGM_DONE;
    sConfig.Mask            = MT25Q_FSR_PGM_DONE;
    sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
    sConfig.StatusBytesSize = 1;
    sConfig.Interval        = 0x10;
    sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, Timeout) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_AutoPolling - Error: Timeout when waiting for write done");
        #endif
        return false;
    }
    return true;
}
/******************************************************************************
 * @brief  Perform an AutoPoll for reset of the WIP flag. The timeout may vary
 *         in dependance of the kind of write op ( page write, sector erase,
 *         block erase, chip erase
 * @param  hqspi : QUADSPI HAL-Handle
 * @param  Timeout : Timeout for auto-polling
 * @retval true if WIP was reset before timeout
 *         false if timeout occured
 *****************************************************************************/
static bool MT25Q_WaitForWriteDoneInternal(QSPI_HandleTypeDef *hqspi, uint32_t Timeout, uint32_t opmode)
{
    QSPI_CommandTypeDef     sCommand;
    QSPI_AutoPollingTypeDef sConfig;

    /* Configure automatic polling mode to wait for memory ready */  
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_STATUS_REG_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    sConfig.Match           = 0;
    sConfig.Mask            = MT25Q_SR_WIP;
    sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
    sConfig.StatusBytesSize = 1;
    sConfig.Interval        = 0x10;
    sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

   switch(opmode) {
        case QSPI_MODE_IRQ:
        case QSPI_MODE_DMA:
            if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK) {
                #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                    DEBUG_PUTS("QSpi_AutoPolling_IT - Error: Setup failed");
                #endif
                return false;
            }
            break;
        default:
            if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, Timeout) != HAL_OK) {
                #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                    DEBUG_PUTS("QSpi_AutoPolling - Error: Timeout when waiting for write done");
                #endif
                return false;
            }
    } // switch
    return true;
}

bool QSpecific_WaitForWriteDone(QSPI_HandleTypeDef *hqspi, uint32_t Timeout) {
    return MT25Q_WaitForWriteDoneInternal(hqspi, Timeout, QSPI_MODE_POLL);
}
bool QSpecific_WaitForWriteDone_IT(QSPI_HandleTypeDef *hqspi) {
    return MT25Q_WaitForWriteDoneInternal(hqspi, 0, QSPI_MODE_IRQ);
}
#if 0
/**
  * @brief  This function enables/disables the Quad mode of the MX25L memory.
  *         Immediately after execution, most commands will only work in 4-4-4 mode!
  * @param  hqspi     : QSPI handle
  * @param  Operation : MT25Q_QUAD_ENABLE or MT25Q_QUAD_DISABLE mode  
  * @retval None
  */
static bool MT25Q_QuadMode(QSpiHandleT *myHandle, uint8_t Operation)
{
    QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;
    QSPI_CommandTypeDef sCommand;
    uint8_t reg;
    bool ret;

    /* Enable or disable quad mode, depeding from operation parameter */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = Operation == MT25Q_QUAD_ENABLE ? ENTER_QUAD_CMD : EXIT_QUAD_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;


   /* Read EVCR Register */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
    sCommand.Instruction       = READ_ENHANCED_VOL_CFG_REG_CMD;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_4_LINES;
    sCommand.NbData            = 1;

    /* Dump Status register */
    ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
    if (ret) ret = HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
     
    if ( !ret ) return false;

    if (  (((reg & MT25Q_EVCR_QUAD_DISABLED) != 0) && (Operation == MT25Q_QUAD_ENABLE)) ||
          (((reg & MT25Q_EVCR_QUAD_DISABLED) == 0) && (Operation == MT25Q_QUAD_DISABLE))
       ) {
        return false;
    }

    return true;
}
#endif

