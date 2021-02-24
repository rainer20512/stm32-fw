/*******************************************************************************
 * @file    mx25_xxx35f.c
 * @author  Rainer, derived from MCD Application Team
 * @brief   Low level driver for Macronix MX25_XXX35F Quad SPI flash memory
 *          works for 
 *          - MX25R6435F 
 *          - MX25L12835F
 *          suitable code will be selected at runtime 
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"

#if USE_QSPI_MX25 > 0
 
#include "system/clockconfig.h"
#include "mx25r6435f.h"
#include "dev/qspi_dev.h"
#include "dev/qspi/qspecific.h"

#if DEBUG_MODE > 0 && DEBUG_QSPI > 0
    #include "debug_helper.h"
#endif

#include <stdio.h>


#define MX25_QUAD_DISABLE       0x0
#define MX25_QUAD_ENABLE        0x1

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
static uint8_t  MX25_GetStatus               (QSPI_HandleTypeDef *hqspi);
       bool     QSpecific_WriteEnable        (QSPI_HandleTypeDef *hqspi);
       bool     QSpecific_WaitForWriteDone   (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static bool     MX25_QuadMode                (QSpiHandleT *myHandle, uint8_t Operation);
static bool     MX25_GetGeometry             (QSpiHandleT *myHandle);
static bool     MX25RXX35_HighPerfMode       (QSPI_HandleTypeDef *hqspi, uint8_t Operation);
static uint16_t MX25_GetType                 (QSpiHandleT *myHandle); 

/* Exported functions -------------------------------------------------------*/

bool QSpecific_HPerfMode ( QSPI_HandleTypeDef *hqspi, bool bHPerf )
{
    bool ret = MX25RXX35_HighPerfMode(hqspi, bHPerf ? MX25_HIGH_PERF_ENABLE : MX25_HIGH_PERF_DISABLE);
    #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
        if ( !ret ) 
            DEBUG_PRINTF("QSpi - Error: Unable to set %s mode\n", bHPerf ? "high perf.":"ultra low power");
        else
            DEBUG_PRINTF("QSpi: Set %s mode\n", bHPerf ? "high perf.":"ultra low power");
    #endif
    return ret;
}
#if DEBUG_MODE > 0

    /**************************************************************************************
     * Return the chip name as string-
     * idbuf has to contain the chip ID, which was read by CMD 0x90/0x9F
     * The Manufacurer ID is not checked here
     *************************************************************************************/
    char *QSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize)
    {
        /* 16 bit combination of chip type and density */
        uint16_t typNdens = idbuf[1];
        typNdens = typNdens << 8 | idbuf[2];
        const char *txt;

        if ( bufsize < 14 ) return NULL;
        
        switch(typNdens) {
            case 0x2817: txt = "MX25R6435F"; break;
            case 0x2018: txt = "MX25L12835F"; break;
            default:     txt = "Unknown Chip";
        }
        
        strncpy(retbuf, txt,bufsize);
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
    QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;
    bool ret;
    bool hperf_enable;

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

    /* if not first init, then if selected operating speed is higher than max. ULP speed, select high performance mode */
    /* on first init, the caller has to assure, that "desired_frq" is at or below QSPI_MAX_ULP_FREQUENCY */
    if ( !bFirstInit && MX25_GetType(myHandle) == MX25R6435F_DEVID ) {
        hperf_enable = desired_frq > QSPI_MAX_ULP_FREQUENCY;
        ret = QSpecific_HPerfMode( hqspi, hperf_enable);
        if ( !ret ) return false;
    }

    /* QSPI initialization */
    hqspi->Init.ClockPrescaler     = prescaler;
    hqspi->Init.FifoThreshold      = 4;
    hqspi->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
    hqspi->Init.FlashSize          = POSITION_VAL(flash_size) - 1;
    hqspi->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    hqspi->Init.ClockMode          = QSPI_CLOCK_MODE_0;

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

  /* Read the 2.byte device ID first, first byte MfgID will be overwritten later */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_ELEC_MANUFACTURER_DEVICE_ID_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 2;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure and read */
  if (HAL_QSPI_Command(&me->hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return QSPI_ERROR;
  if (HAL_QSPI_Receive(&me->hqspi, me->id+2, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)  return QSPI_ERROR;

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
  uint8_t cr, sr[2];
  char txtbuf[20];
  bool ret;

  /* Read the 2.byte device ID first, first byte MfgID will be overwritten later */
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

  QSpecific_GetChipTypeText(myHandle->id,txtbuf,20);
  /* Dump Status register */
  ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_QSPI_Receive(hqspi, &cr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  printf("%s Status register\n",  txtbuf);
  if ( ret ) {
    printf("   SR Write protect = %d\n", cr & MX25_XXX35F_SR_SRWD ? 1 : 0 );
    printf("   Quad Enable      = %d\n", cr & MX25_XXX35F_SR_QE   ? 1 : 0 );
    printf("   Block protect    = 0x%1x\n", (cr & MX25_XXX35F_SR_BP ) >> 2);
    printf("   Write Enable     = %d\n", cr & MX25_XXX35F_SR_WEL  ? 1 : 0 );
    printf("   Write in progess = %d\n", cr & MX25_XXX35F_SR_WIP  ? 1 : 0 );
  } else {
    puts("   Cannot read SR");
  }

  /* Dump configuration register, its content depends from chip tpye  */
  uint16_t devID             = MX25_GetType(myHandle);
  sCommand.Instruction       = READ_CFG_REG_CMD;
  sCommand.NbData            = ( devID == MX25R6435F_DEVID ? 2 : 1 );

  ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_QSPI_Receive(hqspi, sr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  printf("%s Configuration register\n", txtbuf );
  if ( ret ) {
    printf("   Bottom area prot = %d\n", sr[0] & ( 1 << 3 ) ? 1 : 0 );

    switch ( devID ) {
        case MX25R6435F_DEVID:
            printf("   2/4Rd dummy cyc. = %d\n", sr[0] & ( 1 << 6 ) ? 1 : 0 );
            printf("   High perf. mode  = %d\n", sr[1] & ( 1 << 1 ) ? 1 : 0 );
            break;
        case MX25L12835F_DEVID:
            printf("   Dummy cycles.    = %d\n", sr[0] >> 6);
            printf("   Outp.Drv.Str.    = %d\n", sr[0] & 0b111 );
            break;
        default:
            #if DEBUG_MDOE > 0 && DEBBUG_QSPI > 0
                DEBUG_PRINTF("Config Reg. unknown for DevID 0x%04x\n", devID);
            #endif
            ;
    }
  } else {
    puts("   Cannot read CR");
  }

  /* Configure and read SCUR */
  sCommand.Instruction       = READ_SEC_REG_CMD;
  ret = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_QSPI_Receive(hqspi, &cr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  printf("%s Security register\n", txtbuf );
  if ( ret ) {
    printf("   last Erase fail. = %d\n", cr & MX25_XXX35F_SECR_E_FAIL ? 1 : 0 );
    printf("   last Prog failed = %d\n", cr & MX25_XXX35F_SECR_P_FAIL ? 1 : 0 );
    printf("   Erase suspended  = %d\n", cr & MX25_XXX35F_SECR_ESB    ? 1 : 0 );
    printf("   Prog suspended   = %d\n", cr & MX25_XXX35F_SECR_PSB    ? 1 : 0 );
    printf("   LDSO bit         = %d\n", cr & MX25_XXX35F_SECR_LDSO   ? 1 : 0 );
    printf("   factury OTP bit  = %d\n", cr & MX25_XXX35F_SECR_SOI    ? 1 : 0 );
  } else {
    puts("   Cannot read SCUR");
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
    if ( !QSpecific_BasicInit(myHandle, clk_frq, init_speed, MX25_XXX35F_MINIMUM_FLASH_SIZE, true) ) return false;


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

    /* QSPI quad enable */
    if ( !MX25_QuadMode(myHandle, MX25_QUAD_ENABLE) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: Quad mode not set");
        #endif
        return false;
    }

    /* Get geometry data */
    if ( !MX25_GetGeometry(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
            DEBUG_PUTS("QSpi_SpecificInit - Error: Cannot set geometry");
        #endif
        return false;
    }

 
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
  sCommand.Instruction        = QUAD_INOUT_READ_CMD;
  sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
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
  if (MX25_GetStatus(hqspi) == QSPI_BUSY)
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
    
    if (MX25_GetStatus(hqspi) == QSPI_SUSPENDED) return true;
    
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
  if (MX25_GetStatus(hqspi) == QSPI_SUSPENDED)
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
    
    if (MX25_GetStatus(hqspi) == QSPI_BUSY) return true;
    
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
    sCommand.Instruction        = QUAD_INOUT_READ_CMD;
    sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
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
    sCommand.Instruction       = QUAD_PAGE_PROG_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
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
            *timeout_ms = MX25R6435F_SECTOR_ERASE_MAX_TIME;
            *opcode     = SECTOR_ERASE_CMD;
            break;
        case QSPI_ERASE_SUBBLOCK:
            *timeout_ms = MX25R6435F_SUBBLOCK_ERASE_MAX_TIME;
            *opcode     = SUBBLOCK_ERASE_CMD;
            break;
        case QSPI_ERASE_BLOCK:
            *timeout_ms = MX25R6435F_BLOCK_ERASE_MAX_TIME;
            *opcode     = BLOCK_ERASE_CMD;
            break;
        case QSPI_ERASE_ALL:
            *timeout_ms = MX25L12835F_CHIP_ERASE_MAX_TIME;
            *opcode     = CHIP_ERASE_CMD;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_QSPI > 0
                DEBUG_PRINTF("GetEraseTimeout - Error: unkown erasemode %d\n", erasemode);
            #endif
            *timeout_ms = MX25R6435F_SECTOR_ERASE_MAX_TIME;
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
 * Return the device identification ( 16 bit ) from the ID bytes
 *****************************************************************************/
static uint16_t MX25_GetType(QSpiHandleT *myHandle)
{
    uint16_t ret = myHandle->id[1];
    return ret << 8 | myHandle->id[2];
}


/******************************************************************************
 * Set the geometry data according to flash deviceID set in myHandle->id
 *****************************************************************************/
static bool MX25_GetGeometry(QSpiHandleT *myHandle)
{
    /* get type and set all geometry parameters according to that */
    uint16_t devID = MX25_GetType(myHandle);

    switch ( devID ) {
        case MX25R6435F_DEVID:
            QSpi_SetGeometry ( &myHandle->geometry, MX25R6435F_FLASH_SIZE, MX25R6435F_PAGE_SIZE, MX25R6435F_SECTOR_SIZE );
            return true;
        case MX25L12835F_DEVID:
            QSpi_SetGeometry ( &myHandle->geometry, MX25L12835F_FLASH_SIZE, MX25L12835F_PAGE_SIZE, MX25L12835F_SECTOR_SIZE );
            return true;
        default:
            #if DEBUG_MDOE > 0 && DEBBUG_QSPI > 0
                DEBUG_PRINTF("geometry data not defined for DevID 0x%04x\n", devID);
            #endif
            return false;
    }

}



/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
static uint8_t MX25_GetStatus(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Initialize the read security register command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_SEC_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }
  
  /* Check the value of the register */
  if ((reg & (MX25_XXX35F_SECR_P_FAIL | MX25_XXX35F_SECR_E_FAIL)) != 0)
  {
    return QSPI_ERROR;
  }
  else if ((reg & (MX25_XXX35F_SECR_PSB | MX25_XXX35F_SECR_ESB)) != 0)
  {
    return QSPI_SUSPENDED;
  }

  /* Initialize the read status register command */
  sCommand.Instruction       = READ_STATUS_REG_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the value of the register */
  if ((reg & MX25_XXX35F_SR_WIP) != 0)
  {
    return QSPI_BUSY;
  }
  else
  {
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
  sConfig.Match           = MX25_XXX35F_SR_WEL;
  sConfig.Mask            = MX25_XXX35F_SR_WEL;
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
 * @brief  Perform an AutoPoll for reset of the WIP flag. The timeout may vary
 *         in dependance of the kind of write op ( page write, sector erase,
 *         block erase, chip erase
 * @param  hqspi : QUADSPI HAL-Handle
 * @param  Timeout : Timeout for auto-polling
 * @retval true if WIP was reset before timeout
 *         false if timeout occured
 *****************************************************************************/
static bool MX25_WaitForWriteDoneInternal(QSPI_HandleTypeDef *hqspi, uint32_t Timeout, uint32_t opmode)
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
    sConfig.Mask            = MX25_XXX35F_SR_WIP;
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
    return MX25_WaitForWriteDoneInternal(hqspi, Timeout, QSPI_MODE_POLL);
}
bool QSpecific_WaitForWriteDone_IT(QSPI_HandleTypeDef *hqspi) {
    return MX25_WaitForWriteDoneInternal(hqspi, 0, QSPI_MODE_IRQ);
}

/**
  * @brief  This function enables/disables the Quad mode of the MX25L memory.
  * @param  hqspi     : QSPI handle
  * @param  Operation : MX25_QUAD_ENABLE or MX25_QUAD_DISABLE mode  
  * @retval None
  */
static bool MX25_QuadMode(QSpiHandleT *myHandle, uint8_t Operation)
{
    QSPI_HandleTypeDef *hqspi = &myHandle->hqspi;
    QSPI_CommandTypeDef sCommand;
    uint8_t reg;

    /* check proper type */
    uint16_t devID = MX25_GetType(myHandle);
    if ( devID != MX25R6435F_DEVID && devID != MX25L12835F_DEVID ) {
        #if DEBUG_MDOE > 0 && DEBBUG_QSPI > 0
            DEBUG_PRINTF("Enter Quadmode not defined for DevID 0x%04x\n", devID);
        #endif
        return false;
    }

    /* Read status register */
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

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Enable write operations */
    if ( !QSpecific_WriteEnable(hqspi) ) return false;

    /* Activate/deactivate the Quad mode, clear all other bits */
    reg = 0;
    if (Operation == MX25_QUAD_ENABLE) {
        SET_BIT(reg, MX25_XXX35F_SR_QE);
    } 

    sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Wait that memory is ready */  
    if (!QSpecific_WaitForWriteDone(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) ) return false;

    /* Check the configuration has been correctly done */
    sCommand.Instruction = READ_STATUS_REG_CMD;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)  return false;

    if (  (((reg & MX25_XXX35F_SR_QE) == 0) && (Operation == MX25_QUAD_ENABLE)) ||
          (((reg & MX25_XXX35F_SR_QE) != 0) && (Operation == MX25_QUAD_DISABLE))
       ) {
        return false;
    }

    return true;
}


/**
  * @brief  This function enables/disables the high performance mode of the memory.
  * @param  hqspi     : QSPI handle
  * @param  Operation : MX25_HIGH_PERF_ENABLE or MX25_HIGH_PERF_DISABLE high performance mode    
  * @retval None
  */
static bool MX25RXX35_HighPerfMode(QSPI_HandleTypeDef *hqspi, uint8_t Operation)
{
    QSPI_CommandTypeDef sCommand;
    uint8_t reg[3];

    /* Read status register */
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

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Receive(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Read configuration registers */
    sCommand.Instruction = READ_CFG_REG_CMD;
    sCommand.NbData      = 2;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Receive(hqspi, &(reg[1]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Enable write operations */
    if ( !QSpecific_WriteEnable(hqspi) ) return false;

    /* Activate/deactivate the hig performance mode */
    if (Operation == MX25_HIGH_PERF_ENABLE) {
        SET_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
    } else {
        CLEAR_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
    }

    sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;
    sCommand.NbData      = 3;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_QSPI_Transmit(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Wait that memory is ready */  
    if ( !QSpecific_WaitForWriteDone(hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) ) return false;

    /* Check the configuration has been correctly done */
    sCommand.Instruction = READ_CFG_REG_CMD;
    sCommand.NbData      = 2;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;


    if (HAL_QSPI_Receive(hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if ( (((reg[1] & MX25R6435F_CR2_LH_SWITCH) == 0) && (Operation == MX25_HIGH_PERF_ENABLE)) ||
         (((reg[1] & MX25R6435F_CR2_LH_SWITCH) != 0) && (Operation == MX25_HIGH_PERF_DISABLE))
       ) {
        return false;
    }

    return true;
}

#endif /* #if USE_QSPI_MX25 > 0 */
