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

#if USE_XSPI_MX25 > 0 && ( USE_QSPI > 0 ||  USE_OSPI > 0 )

/* Set >0 to generate DEBUG output */
#define DEBUG_XSPI              2

#include "system/clockconfig.h"
#include "mx25r6435f.h"
#include "dev/xspi_dev.h"
#include "dev/xspi/xspi_specific.h"
#include "log.h"

#if DEBUG_MODE > 0 && DEBUG_XSPI > 0
    #include "debug_helper.h"
#endif

#if USE_OSPI > 0
    #if USE_OSPI1 > 0
        #define     XSpiStr                         "OSPI1"
    #else
        #define     XSpiStr                         "OSPI2"
    #endif
#else
        #define     XSpiStr                         "QSPI"
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
#define XSPI_MAX_ULP_FREQUENCY  33000000

/* safe operating speed to readout ID values */
#define XSPI_INITIAL_SPEED      1000000   


/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
static uint8_t  MX25_GetStatus               (XXSPI_HandleTypeDef *hxspi);
       bool     XSpecific_WriteEnable        (XXSPI_HandleTypeDef *hxspi);
       bool     XSpecific_WaitForWriteDone   (XXSPI_HandleTypeDef *hxspi, uint32_t Timeout);
static bool     MX25_QuadMode                (XSpiHandleT *myHandle, uint8_t Operation);
static bool     MX25_GetGeometry             (XSpiHandleT *myHandle);
static bool     MX25RXX35_HighPerfMode       (XXSPI_HandleTypeDef *hxspi, uint8_t Operation);
static uint16_t MX25_GetType                 (XSpiHandleT *myHandle); 

/* Exported functions -------------------------------------------------------*/

bool XSpecific_HPerfMode ( XXSPI_HandleTypeDef *hxspi, bool bHPerf )
{
    bool ret = MX25RXX35_HighPerfMode(hxspi, bHPerf ? MX25_HIGH_PERF_ENABLE : MX25_HIGH_PERF_DISABLE);
    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        if ( !ret ) 
            LOG_ERROR("%s: Unable to set %s mode\n", XSpiStr, bHPerf ? "high perf.":"ultra low power");
        else
            LOG_INFO(": Set %s mode\n",XSpiStr, bHPerf ? "high perf.":"ultra low power");
    #endif
    return ret;
}
#if DEBUG_MODE > 0

    /**************************************************************************************
     * Return the chip name as string-
     * idbuf has to contain the chip ID, which was read by CMD 0x90/0x9F
     * The Manufacurer ID is not checked here
     *************************************************************************************/
    char *XSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize)
    {
        /* 16 bit combination of chip type and density */
        uint16_t typNdens = idbuf[1];
        typNdens = typNdens << 8 | idbuf[2];
        const char *txt;

        if ( bufsize < 14 ) return NULL;
        
        switch(typNdens) {
            case 0x2817: txt = "MX25R6435F"; break;
            case 0x2018: txt = "MX25L1283xF"; break;
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
bool XSpecific_BasicInit(XSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size, bool bFirstInit )
{  
    XXSPI_HandleTypeDef *hxspi = &myHandle->hxspi;
    bool ret;
    bool hperf_enable;

    /* calculate prescaler,so that the initial frequency is at or below max. ULP frequency */ 
    uint32_t prescaler = ( clk_frq + desired_frq - 1 ) / desired_frq;
    if ( prescaler > 0 ) prescaler--;
    desired_frq = clk_frq / (prescaler+1);

    if ( prescaler > 255 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOG_WARN("XSpecific_BasicInit - QSPI clk too low, minimum is %d\n", clk_frq/256 );
        #endif
        return false;
    }

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        LOG_INFO("%s: Clk=%d\n", XSpiStr, desired_frq );
    #endif

    /* if not first init, then if selected operating speed is higher than max. ULP speed, select high performance mode */
    /* on first init, the caller has to assure, that "desired_frq" is at or below XSPI_MAX_ULP_FREQUENCY */
    if ( !bFirstInit && MX25_GetType(myHandle) == MX25R6435F_DEVID ) {
        hperf_enable = desired_frq > XSPI_MAX_ULP_FREQUENCY;
        ret = XSpecific_HPerfMode( hxspi, hperf_enable);
        if ( !ret ) return false;
    }

    /* QSPI initialization */
    hxspi->Init.ClockPrescaler              = prescaler;
    hxspi->Init.FifoThreshold               = 4;
    #if USE_OSPI > 0
        hxspi->Init.DualQuad                = HAL_OSPI_DUALQUAD_DISABLE;
        hxspi->Init.MemoryType              = HAL_OSPI_MEMTYPE_MACRONIX;
        hxspi->Init.SampleShifting          = HAL_OSPI_SAMPLE_SHIFTING_NONE;
        hxspi->Init.DeviceSize              = POSITION_VAL(flash_size) - 1;
        hxspi->Init.ChipSelectHighTime      = 1;
        hxspi->Init.ClockMode               = HAL_OSPI_CLOCK_MODE_0;
        hxspi->Init.FreeRunningClock        = HAL_OSPI_FREERUNCLK_DISABLE;
        hxspi->Init.DelayHoldQuarterCycle   = HAL_OSPI_DHQC_DISABLE;
        hxspi->Init.ChipSelectBoundary      = 0;
        hxspi->Init.DelayBlockBypass        = HAL_OSPI_DELAY_BLOCK_USED;
    #else
        hxspi->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
        hxspi->Init.FlashSize          = POSITION_VAL(flash_size) - 1;
        hxspi->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
        hxspi->Init.ClockMode          = QSPI_CLOCK_MODE_0;
#endif
    if (HAL_XSPI_Init(hxspi) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpecific_BasicInit - Error: HAL Init failed");
        #endif
        return false;
    }

    /* Update Quadspi clkspeed to the actual speed*/
    #if USE_OSPI > 0
        prescaler = ( hxspi->Instance->CR & OCTOSPI_DCR2_PRESCALER_Msk ) >> OCTOSPI_DCR2_PRESCALER_Pos;
    #else
        prescaler = ( hxspi->Instance->CR & QUADSPI_CR_PRESCALER_Msk ) >> QUADSPI_CR_PRESCALER_Pos;
    #endif
    myHandle->clkspeed = desired_frq;

    return true;
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hxspi : QSPI handle
  * @retval None
  */
bool XSpecific_ResetMemory(XSpiHandleT *myHandle)
{
  XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
  XSPI_CommandTypeDef sCommand = {0};

  /* Initialize the reset enable command */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = RESET_ENABLE_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = RESET_ENABLE_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  #endif

  /* Send the command */
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
 

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;


  /* Configure automatic polling mode to wait the memory is ready */  
  if ( !XSpecific_WaitForWriteDone(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE) ) return false;

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
uint32_t XSpecific_GetID ( XSpiHandleT *me )
{
  XSPI_CommandTypeDef sCommand={0};

  /* Read the 2.byte device ID first, first byte MfgID will be overwritten later */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_ELEC_MANUFACTURER_DEVICE_ID_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 2;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
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
  #endif
  
  /* Configure and read */
  if (HAL_XSPI_Command(&me->hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return XSPI_ERROR;
  if (HAL_XSPI_Receive(&me->hxspi, me->id+2, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)  return XSPI_ERROR;

  /* Read 3 byte ID,  */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_ID_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 3;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
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
  #endif

  /* Configure and read  */
  if (HAL_XSPI_Command(&me->hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return XSPI_ERROR;
  if (HAL_XSPI_Receive(&me->hxspi, me->id, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)    return XSPI_ERROR; 
  
  return XSPI_OK;
}

/******************************************************************************
 * Dump the status of Status and Configuration register
 *****************************************************************************/
void XSpecific_DumpStatusInternal(XSpiHandleT *myHandle)
{
  XXSPI_HandleTypeDef *hxspi = &myHandle->hxspi;
  XSPI_CommandTypeDef sCommand={0};
  uint8_t cr, sr[2];
  char txtbuf[20];
  bool ret;

  /* Read the 2.byte device ID first, first byte MfgID will be overwritten later */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_STATUS_REG_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 1;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
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
  #endif
  XSpecific_GetChipTypeText(myHandle->id,txtbuf,20);

  /* Dump Status register */
  ret = HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_XSPI_Receive(hxspi, &cr, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
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

  ret = HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_XSPI_Receive(hxspi, sr, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
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
  ret = HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
  if (ret) ret = HAL_XSPI_Receive(hxspi, &cr, XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
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
bool XSpecific_SpecificInit(const HW_DeviceType *self,XSpiHandleT *myHandle, uint32_t clk_frq)
{ 
    UNUSED(self);
    uint32_t clkspeed = myHandle->clkspeed;
    
    /* 
     * The initialization clock speed is the minimum of selected operating speed and XSPI_INITIAL_SPEED
     * which is a safe speed for initialization 
     */
    uint32_t init_speed = ( clkspeed < XSPI_INITIAL_SPEED ? clkspeed : XSPI_INITIAL_SPEED );

    /* Setup the deep sleep timing parameters, if deep sleep is supported */
    if ( myHandle->dsInfo ) {
        myHandle->dsInfo->dlyFmSleep = MX25_DLY_FM_SLEEP;
        myHandle->dsInfo->dlyToSleep = MX25_DLY_TO_SLEEP;
    }

    /* Basic Initialization with a safe speed and minimum flash size to readout ID data */
    if ( !XSpecific_BasicInit(myHandle, clk_frq, init_speed, MX25_XXX35F_MINIMUM_FLASH_SIZE, true) ) return false;


    /* QSPI memory reset */
    if (!XSpecific_ResetMemory(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI SpecificInit - Error: ResetMemory failed");
        #endif
    }

    /* read ID bytes */
    if ( XSpecific_GetID(myHandle) != XSPI_OK ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI SpecificInit - Error: Cannot read ID bytes");
        #endif
        return false;
    }

    /* QSPI quad enable */
    if ( !MX25_QuadMode(myHandle, MX25_QUAD_ENABLE) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI SpecificInit - Error: Quad mode not set");
        #endif
        return false;
    }

    /* Get geometry data */
    if ( !MX25_GetGeometry(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI SpecificInit - Error: Cannot set geometry");
        #endif
        return false;
    }

 
    /* The operating speed is now set to user configured speed and flash size is set to correct size */
    if ( !XSpecific_BasicInit(myHandle, clk_frq, clkspeed, myHandle->geometry.FlashSize, false) ) return false;

  return true;
}

/**
  * @brief  This function enter the QSPI memory in deep power down mode.
  * @retval QSPI memory status
  */
bool XSpecific_EnterDeepPowerDown(XSpiHandleT *myHandle)
{

  /* check for deep sleep capbility */
  if (!myHandle->dsInfo ) return false;

  XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
  XSPI_CommandTypeDef sCommand={0};
  
  /* Initialize the deep power down command */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = DEEP_POWER_DOWN_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = DEEP_POWER_DOWN_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  #endif
    
  /* Send the command */
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  
  /* ---          Memory takes 10us max to enter deep power down          --- */
  /* --- At least 30us should be respected before leaving deep power down --- */

  myHandle->dsInfo->bIsDeepSleep = true;
  return true;
}

/**
  * @brief  This function leave the QSPI memory from deep power down mode.
  * @retval QSPI memory status
  */
bool XSpecific_LeaveDeepPowerDown(XSpiHandleT *myHandle)
{
  /* check for deep sleep capbility */
  if (!myHandle->dsInfo ) return false;

  XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
  XSPI_CommandTypeDef sCommand={0};
  
  /* Send NoOp command to wake up */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = EXIT_POWER_DOWN_CMD;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = EXIT_POWER_DOWN_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  #endif    
  /* Send the command */
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  
  /* An exit power down command is sent to the memory, as the nCS should be low for at least 20 ns */
  /* On MX25R devices, the nCS signal going low is sufficient to wake up device                    */
  /* Memory takes 35us min to leave deep power down                                                */
  
  BASTMR_DelayUs(DS_RECOVERY_TIME_US);

  myHandle->dsInfo->bIsDeepSleep = false;
  return true;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @retval QSPI memory status
  */
bool XSpecific_EnableMemoryMappedMode(XSpiHandleT *myHandle)
{
  XXSPI_HandleTypeDef *hxspi = &myHandle->hxspi;
  XSPI_CommandTypeDef      sCommand={0};
  XSPI_MemoryMappedTypeDef sMemMappedCfg={0};

  /* Wake up device, if deep sleep capability and in in deep sleep */
  if ( myHandle->dsInfo && myHandle->dsInfo->bIsDeepSleep && ! XSpecific_LeaveDeepPowerDown(myHandle) ) return false;

  /* Configure the command for the read instruction */
  #if USE_OSPI > 0
    
    /* Enable Write Operations */
    if ( !XSpecific_WriteEnable(hxspi) ) return false;

    /* Preset common parameters to read and write operation */
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
    sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    /* Configure the write operation */
    sCommand.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
    sCommand.Instruction   = QUAD_PAGE_PROG_CMD;
    sCommand.DataMode      = HAL_OSPI_DATA_4_LINES;
    sCommand.DummyCycles   = 0;
    sCommand.NbData        = 1; 
        
    if (HAL_OSPI_Command(hxspi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI MemoryMapped - Error: Cannot configure write operation");
        #endif
        return false;
    }

    /* Configure the read operation */
    sCommand.OperationType      = HAL_OSPI_OPTYPE_READ_CFG;
    sCommand.Instruction        = QUAD_INOUT_READ_CMD;
    sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
    sCommand.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
    
    if (HAL_OSPI_Command(hxspi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSPI MemoryMapped - Error: Cannot configure read operation");
        #endif
        return false;
    }

    /* Configure and enable memory mapped mode */
    sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
    if (HAL_XSPI_MemoryMapped(hxspi, &sMemMappedCfg) != HAL_OK) {
        return false;
    }
  #else 
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

    if (HAL_XSPI_MemoryMapped(hxspi, &sCommand, &sMemMappedCfg) != HAL_OK)
    {
    return false;
    }
  #endif

  myHandle->bIsMemoryMapped = true;
  return true;
}

/**
  * @brief  This function suspends an ongoing erase command.
  * @retval QSPI memory status
  */
bool XSpecific_SuspendErase(XXSPI_HandleTypeDef *hxspi)
{
  XSPI_CommandTypeDef sCommand={0};
  
  /* Check whether the device is busy (erase operation is 
  in progress).
  */
  if (MX25_GetStatus(hxspi) == XSPI_BUSY)
  {
    /* Initialize the erase command */
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = PROG_ERASE_SUSPEND_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_NONE;
        sCommand.DummyCycles       = 0;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
        sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = PROG_ERASE_SUSPEND_CMD;
        sCommand.AddressMode       = QSPI_ADDRESS_NONE;
        sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = QSPI_DATA_NONE;
        sCommand.DummyCycles       = 0;
        sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
        sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    #endif
        
    /* Send the command */
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
    
    if (MX25_GetStatus(hxspi) == XSPI_SUSPENDED) return true;
    
    return false;
  }
  
  return true;
}

/**
  * @brief  This function resumes a paused erase command.
  * @retval QSPI memory status
  */
bool XSpecific_ResumeErase(XXSPI_HandleTypeDef *hxspi)
{
  XSPI_CommandTypeDef sCommand={0};
  
  /* Check whether the device is in suspended state */
  if (MX25_GetStatus(hxspi) == XSPI_SUSPENDED)
  {
    /* Initialize the erase command */
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = PROG_ERASE_RESUME_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_NONE;
        sCommand.DummyCycles       = 0;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
        sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = PROG_ERASE_RESUME_CMD;
        sCommand.AddressMode       = QSPI_ADDRESS_NONE;
        sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = QSPI_DATA_NONE;
        sCommand.DummyCycles       = 0;
        sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
        sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    #endif
        
    /* Send the command */
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
    
    /*
    When this command is executed, the status register write in progress bit is set to 1, and
    the flag status register program erase controller bit is set to 0. This command is ignored
    if the device is not in a suspended state.
    */
    
    if (MX25_GetStatus(hxspi) == XSPI_BUSY) return true;
    
    return false;
  }

  return true;
}

/******************************************************************************
 * Read Operation command
 *****************************************************************************/
bool XSpecific_ReadCMD(XXSPI_HandleTypeDef *hxspi, uint32_t Addr, uint32_t Size)
{
    XSPI_CommandTypeDef sCommand={0};

    /* Initialize the read command */
    #if USE_OSPI > 0
        sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
        sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.Instruction        = QUAD_INOUT_READ_CMD;
        sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
        sCommand.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
        sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
        sCommand.Address            = Addr;
        sCommand.NbData             = Size;
        sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
        sCommand.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
        sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
        sCommand.DataMode           = HAL_OSPI_DATA_4_LINES;
        sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
        sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
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
    #endif
    /* Configure the command */
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpi_Read - Error: Command not send");
        #endif
        return false;
    }

    return true;
}

/******************************************************************************
 * Enable Write and send write Operation command
 *****************************************************************************/
bool XSpecific_WriteCMD(XXSPI_HandleTypeDef *hxspi, uint32_t Addr, uint32_t Size)
{
    XSPI_CommandTypeDef sCommand={0};

    /* Initialize the program command */
    #if USE_OSPI > 0
        sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
        sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.Instruction        = QUAD_PAGE_PROG_CMD;
        sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
        sCommand.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
        sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
        sCommand.Address            = Addr;
        sCommand.NbData             = Size;
        sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode           = HAL_OSPI_DATA_4_LINES;
        sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles        = 0;
        sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
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
    #endif

    /* Enable write operations */
    if ( !XSpecific_WriteEnable(hxspi) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpecific_WriteCMD - Error: Write enable failed");
        #endif
      return false;
    }

    /* Configure the command */
        if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpecific_WriteCMD - Error: Command not send");
        #endif
        return false;
    }

    return true;
}

/******************************************************************************
 * return the maximum erase time ( according to data sheet ) and tpye of
 * erase
 *****************************************************************************/
bool XSpecific_GetEraseParams(uint32_t erasemode, uint32_t *timeout_ms, uint8_t *opcode)
{
    bool ret = true;
 
    switch(erasemode)
    {
        case XSPI_ERASE_SECTOR:
            *timeout_ms = MX25R6435F_SECTOR_ERASE_MAX_TIME;
            *opcode     = SECTOR_ERASE_CMD;
            break;
        case XSPI_ERASE_SUBBLOCK:
            *timeout_ms = MX25R6435F_SUBBLOCK_ERASE_MAX_TIME;
            *opcode     = SUBBLOCK_ERASE_CMD;
            break;
        case XSPI_ERASE_BLOCK:
            *timeout_ms = MX25R6435F_BLOCK_ERASE_MAX_TIME;
            *opcode     = BLOCK_ERASE_CMD;
            break;
        case XSPI_ERASE_ALL:
            *timeout_ms = MX25L12835F_CHIP_ERASE_MAX_TIME;
            *opcode     = CHIP_ERASE_CMD;
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
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
bool XSpecific_EraseCMD(XXSPI_HandleTypeDef *hxspi, uint32_t Address, uint32_t eraseMode )
{
    uint32_t tmo_unused;
    uint8_t  opcode;

    XSPI_CommandTypeDef sCommand={0};

    if ( !XSpecific_GetEraseParams(eraseMode, &tmo_unused, &opcode ) ) return false;

    /* Initialize the erase command */
     #if USE_OSPI > 0
        sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
        sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.Instruction        = opcode;
        if ( eraseMode == XSPI_ERASE_ALL ) {
            sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        } else {
            sCommand.AddressMode       = HAL_OSPI_ADDRESS_1_LINE;
            sCommand.AddressSize       = HAL_OSPI_ADDRESS_24_BITS;
            sCommand.AddressDtrMode    = HAL_OSPI_ADDRESS_DTR_DISABLE;
            sCommand.Address           = Address;
        }
        sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode           = HAL_OSPI_DATA_NONE;
        sCommand.DummyCycles        = 0;
        sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
        sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = opcode;
        if ( eraseMode == XSPI_ERASE_ALL ) {
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
    #endif 
       
    /* Enable write operations */
    if ( !XSpecific_WriteEnable(hxspi) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("QSpi_SpecificEraseBlockWait - Error: Write enable failed");
        #endif
        return false;
    }

    /* Send the command */
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
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
static uint16_t MX25_GetType(XSpiHandleT *myHandle)
{
    uint16_t ret = myHandle->id[1];
    return ret << 8 | myHandle->id[2];
}


/******************************************************************************
 * Set the geometry data according to flash deviceID set in myHandle->id
 *****************************************************************************/
static bool MX25_GetGeometry(XSpiHandleT *myHandle)
{
    /* get type and set all geometry parameters according to that */
    uint16_t devID = MX25_GetType(myHandle);

    switch ( devID ) {
        case MX25R6435F_DEVID:
            XSpi_SetGeometry ( &myHandle->geometry, MX25R6435F_FLASH_SIZE, MX25R6435F_PAGE_SIZE, MX25R6435F_SECTOR_SIZE );
            return true;
        case MX25L12835F_DEVID:
            XSpi_SetGeometry ( &myHandle->geometry, MX25L12835F_FLASH_SIZE, MX25L12835F_PAGE_SIZE, MX25L12835F_SECTOR_SIZE );
            return true;
        default:
            #if DEBUG_MDOE > 0 && DEBUG_XSPI > 0
                DEBUG_PRINTF("geometry data not defined for DevID 0x%04x\n", devID);
            #endif
            return false;
    }

}



/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
static uint8_t MX25_GetStatus(XXSPI_HandleTypeDef *hxspi)
{
  XSPI_CommandTypeDef sCommand={0};
  uint8_t reg;

  /* Initialize the read security register command */
  #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = READ_SEC_REG_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles       = 0;
        sCommand.NbData            = 1;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
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
  #endif  

  /* Configure the command */
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return XSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_XSPI_Receive(hxspi, &reg, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return XSPI_ERROR;
  }
  
  /* Check the value of the register */
  if ((reg & (MX25_XXX35F_SECR_P_FAIL | MX25_XXX35F_SECR_E_FAIL)) != 0)
  {
    return XSPI_ERROR;
  }
  else if ((reg & (MX25_XXX35F_SECR_PSB | MX25_XXX35F_SECR_ESB)) != 0)
  {
    return XSPI_SUSPENDED;
  }

  /* Initialize the read status register command */
  sCommand.Instruction       = READ_STATUS_REG_CMD;

  /* Configure the command */
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return XSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_XSPI_Receive(hxspi, &reg, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return XSPI_ERROR;
  }

  /* Check the value of the register */
  if ((reg & MX25_XXX35F_SR_WIP) != 0)
  {
    return XSPI_BUSY;
  }
  else
  {
    return XSPI_OK;
  }
}



/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hxspi : QSPI handle
  * @retval None
  */
bool XSpecific_WriteEnable(XXSPI_HandleTypeDef *hxspi)
{
  XSPI_CommandTypeDef     sCommand={0};
  XSPI_AutoPollingTypeDef sConfig={0};

  /* Enable write operations */
  #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = WRITE_ENABLE_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_NONE;
        sCommand.DummyCycles       = 0;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
      sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      sCommand.Instruction       = WRITE_ENABLE_CMD;
      sCommand.AddressMode       = QSPI_ADDRESS_NONE;
      sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      sCommand.DataMode          = QSPI_DATA_NONE;
      sCommand.DummyCycles       = 0;
      sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
      sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  #endif
    
  if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  
  /* Configure automatic polling mode to wait for write enabling */  
  #if USE_OSPI > 0
      /* Most sCommand parameter settings are taken from above */
      sCommand.Instruction = READ_STATUS_REG_CMD;
      sCommand.DataMode    = HAL_OSPI_DATA_1_LINE;
      sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
      sCommand.NbData      = 1; 

      if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MDOE > 0 && DEBUG_XSPI > 0
            DEBUG_PRINTF("XSPI WriteEnable: cannot configure AutoPoll");
        #endif
        return false;
      }

      sConfig.Match           = MX25_XXX35F_SR_WEL;
      sConfig.Mask            = MX25_XXX35F_SR_WEL;
      sConfig.MatchMode       = HAL_OSPI_MATCH_MODE_AND;
      sConfig.Interval        = 10;
      sConfig.AutomaticStop   = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

      if (HAL_XSPI_AutoPolling(hxspi, &sConfig, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MDOE > 0 && DEBUG_XSPI > 0
            DEBUG_PRINTF("XSPI WriteEnable: cannot start AutoPoll");
        #endif
        return false;
      }
  #else
      sConfig.Match           = MX25_XXX35F_SR_WEL;
      sConfig.Mask            = MX25_XXX35F_SR_WEL;
      sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
      sConfig.StatusBytesSize = 1;
      sConfig.Interval        = 0x10;
      sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

      sCommand.Instruction    = READ_STATUS_REG_CMD;
      sCommand.DataMode       = QSPI_DATA_1_LINE;

      if (HAL_XSPI_AutoPolling(hxspi, &sCommand, &sConfig, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        /* If during AutoPoll a timeout occurs, the State "HAL_QSPI_STATE_ERROR" will never be reset
           So do it manually here  */
        hxspi->State = HAL_QSPI_STATE_READY;
        return false;
      }
  #endif
  return true;
}

/******************************************************************************
 * @brief  Perform an AutoPoll for reset of the WIP flag. The timeout may vary
 *         in dependance of the kind of write op ( page write, sector erase,
 *         block erase, chip erase
 * @param  hxspi : QUADSPI HAL-Handle
 * @param  Timeout : Timeout for auto-polling
 * @retval true if WIP was reset before timeout
 *         false if timeout occured
 *****************************************************************************/
static bool MX25_WaitForWriteDoneInternal(XXSPI_HandleTypeDef *hxspi, uint32_t Timeout, uint32_t opmode)
{
    XSPI_CommandTypeDef     sCommand={0};
    XSPI_AutoPollingTypeDef sConfig={0};

    /* Configure automatic polling mode to wait for memory ready */  
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       =  READ_STATUS_REG_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles       = 0;
        sCommand.NbData            = 1;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
        if (HAL_OSPI_Command(hxspi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            #if DEBUG_MDOE > 0 && DEBUG_XSPI > 0
                DEBUG_PRINTF("WaitForWrite: Cannot setup autopoll");
            #endif
            return false;
        }

        sConfig.Match           = 0;
        sConfig.Mask            = MX25_XXX35F_SR_WIP;
        sConfig.MatchMode       = HAL_OSPI_MATCH_MODE_AND;
        sConfig.Interval        = 0x10;
        sConfig.AutomaticStop   = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    #else
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
   #endif 
   
   switch(opmode) {
        case XSPI_MODE_IRQ:
        case XSPI_MODE_DMA:
        #if USE_OSPI > 0
            if (HAL_XSPI_AutoPolling_IT(hxspi, &sConfig) != HAL_OK) {
        #else
            if (HAL_XSPI_AutoPolling_IT(hxspi, &sCommand, &sConfig) != HAL_OK) {
        #endif
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    DEBUG_PUTS("QSpi_AutoPolling_IT - Error: Setup failed");
                #endif
                return false;
            }
            break;
        default:
        #if USE_OSPI > 0
            if (HAL_XSPI_AutoPolling(hxspi, &sConfig, Timeout) != HAL_OK) {
         #else
            if (HAL_XSPI_AutoPolling(hxspi, &sCommand, &sConfig, Timeout) != HAL_OK) {
         #endif
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    DEBUG_PUTS("QSpi_AutoPolling - Error: Timeout when waiting for write done");
                #endif
                return false;
            }
    } // switch
    return true;
}

bool XSpecific_WaitForWriteDone(XXSPI_HandleTypeDef *hxspi, uint32_t Timeout) {
    return MX25_WaitForWriteDoneInternal(hxspi, Timeout, XSPI_MODE_POLL);
}
bool XSpecific_WaitForWriteDone_IT(XXSPI_HandleTypeDef *hxspi) {
    return MX25_WaitForWriteDoneInternal(hxspi, 0, XSPI_MODE_IRQ);
}

/**
  * @brief  This function enables/disables the Quad mode of the MX25L memory.
  * @param  hxspi     : QSPI handle
  * @param  Operation : MX25_QUAD_ENABLE or MX25_QUAD_DISABLE mode  
  * @retval None
  */
static bool MX25_QuadMode(XSpiHandleT *myHandle, uint8_t Operation)
{
    XXSPI_HandleTypeDef *hxspi = &myHandle->hxspi;
    XSPI_CommandTypeDef sCommand = {0};
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
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = READ_STATUS_REG_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles       = 0;
        sCommand.NbData            = 1;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
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
    #endif
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Receive(hxspi, &reg, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Enable write operations */
    if ( !XSpecific_WriteEnable(hxspi) ) return false;

    /* Activate/deactivate the Quad mode, clear all other bits */
    reg = 0;
    if (Operation == MX25_QUAD_ENABLE) {
        SET_BIT(reg, MX25_XXX35F_SR_QE);
    } 

    sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;

    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Transmit(hxspi, &reg, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Wait that memory is ready */  
    if (!XSpecific_WaitForWriteDone(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE) ) return false;

    /* Check the configuration has been correctly done */
    sCommand.Instruction = READ_STATUS_REG_CMD;

    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Receive(hxspi, &reg, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)  return false;

    if (  (((reg & MX25_XXX35F_SR_QE) == 0) && (Operation == MX25_QUAD_ENABLE)) ||
          (((reg & MX25_XXX35F_SR_QE) != 0) && (Operation == MX25_QUAD_DISABLE))
       ) {
        return false;
    }

    return true;
}


/**
  * @brief  This function enables/disables the high performance mode of the memory.
  * @param  hxspi     : QSPI handle
  * @param  Operation : MX25_HIGH_PERF_ENABLE or MX25_HIGH_PERF_DISABLE high performance mode    
  * @retval None
  */
static bool MX25RXX35_HighPerfMode(XXSPI_HandleTypeDef *hxspi, uint8_t Operation)
{
    XSPI_CommandTypeDef sCommand={0};
    uint8_t reg[3];

    /* Read status register */
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = READ_STATUS_REG_CMD;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.DummyCycles       = 0;
        sCommand.NbData            = 1;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
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
    #endif
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Receive(hxspi, &(reg[0]), XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Read configuration registers */
    sCommand.Instruction = READ_CFG_REG_CMD;
    sCommand.NbData      = 2;

    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Receive(hxspi, &(reg[1]), XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Enable write operations */
    if ( !XSpecific_WriteEnable(hxspi) ) return false;

    /* Activate/deactivate the hig performance mode */
    if (Operation == MX25_HIGH_PERF_ENABLE) {
        SET_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
    } else {
        CLEAR_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
    }

    sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;
    sCommand.NbData      = 3;

    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if (HAL_XSPI_Transmit(hxspi, &(reg[0]), XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Wait that memory is ready */  
    if ( !XSpecific_WaitForWriteDone(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE) ) return false;

    /* Check the configuration has been correctly done */
    sCommand.Instruction = READ_CFG_REG_CMD;
    sCommand.NbData      = 2;

    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;


    if (HAL_XSPI_Receive(hxspi, &(reg[0]), XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    if ( (((reg[1] & MX25R6435F_CR2_LH_SWITCH) == 0) && (Operation == MX25_HIGH_PERF_ENABLE)) ||
         (((reg[1] & MX25R6435F_CR2_LH_SWITCH) != 0) && (Operation == MX25_HIGH_PERF_DISABLE))
       ) {
        return false;
    }

    return true;
}

#endif /* #if USE_XSPI_MX25 > 0 */
