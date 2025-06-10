/*******************************************************************************
 * @file    xspi_lowlevel.c
 * @author  Rainer
 * @brief   QSPI/OSPI lowlevel functions which are independent from specific
 *          Flash device, all access is done via interface definition
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"

#if  USE_QSPI > 0 || USE_OSPI > 0 

#include "dev/xspi_dev.h"
#include "dev/xspi/xspi_helper.h"

/* Set >0 to generate DEBUG output */
#define DEBUG_XSPI              2

#if DEBUG_MODE > 0 && DEBUG_XSPI > 0
    #include "debug_helper.h"
    #include "log.h"
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


#if DEBUG_MODE > 0 
    
    char *XSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize);
    
    /**************************************************************************************
     * Return the Manufacturer Name as string                                             *
     *************************************************************************************/
    const char *XSpiLL_GetChipManufacturer(uint8_t mf_id )
    {
        switch(mf_id) {
            case 0x1f: return "Renesas"; 
            case 0x20: return "Micron"; 
            case 0xc2: return "Macronix"; 
            default:
                return "Unknown Manufacturer";
        }
    }



    /**************************************************************************************
     * Read the chip ID and print chip info. Can be used to set chip specific parameters  *
     * automatically later. Now just chip info is dumped                                  *
     *************************************************************************************/
    void XSpiLL_DumpChipInfo(uint8_t *idbuf)
    {
        char type[25];
        const char *mf   =  XSpiLL_GetChipManufacturer(idbuf[0] );
        XSpecific_GetChipTypeText(idbuf, type, 25);
        LOG_INFO("%s: Found %s %s", XSpiStr, mf, type);
    }
    /**************************************************************************************
     * Dump the geometry data  *
     * automatically later. Now just chip info is dumped                                  *
     *************************************************************************************/
    void XSpiLL_DumpGeometry(XSpiGeometryT *geo)
    {
        uint32_t size = geo->FlashSize;
        if ( size >= 20 ) {
            LOG_INFO("%s: Flash size is %dMiB", XSpiStr, 1<<(size-20));
        } else if ( size >= 10 ) {
            LOG_INFO("%s: Flash size is %dkiB", XSpiStr, 1<<(size-10));
        } else {
            LOG_INFO("%s: Flash size is %dB", XSpiStr, 1<<size);
        }
        LOG_INFO("%s: %d write pages with %d bytes", XSpiStr, geo->ProgPagesNumber, geo->ProgPageSize);
        for ( uint32_t i =0; i < MAX_ERASE; i++ ) {
            if ( geo->EraseSize[i] > 0 ) {
                LOG_INFO("%s: Erase element %d: Size = %d, Num= %d,", XSpiStr, i, geo->EraseSize[i] , geo->EraseNum[i] );
            }
        }
    }

    #if DEBUG_XSPI > 1
        /**************************************************************************************
         * Dump CommandTypeDef structure
         * automatically later. Now just chip info is dumped                                  *
         *************************************************************************************/
        void XSpiLL_DumpScmd(XSPI_CommandTypeDef *sCmd)
        {
            DEBUG_PRINTF("sCommand: Cmd=0x%02x IMode=0x%08x D.cycles=%d\n", sCmd->Instruction, sCmd->InstructionMode, sCmd->DummyCycles);
            if ( sCmd->AddressMode > 0 ) {
                DEBUG_PRINTF( "Addr==0x%08x, AMode=0x%08x, ASize=0x%08x\n", sCmd->Address, sCmd->AddressMode, sCmd->AddressSize);
            }
            if ( sCmd->DataMode > 0 ) {
                DEBUG_PRINTF( "DMode=0x%08x, DSize=%d\n", sCmd->DataMode, sCmd->NbData);
            }
            if ( sCmd->AlternateByteMode > 0 ) {
                DEBUG_PRINTF( "ABMode=0x%08x, ABSize=%d, ABytes=0x%08x\n", sCmd->AlternateByteMode, sCmd->AlternateBytesSize, sCmd->AlternateBytes);
            }
            
            DEBUG_PRINTF( "ABMode=0x%08x, ABSize=%d, ABytes=0x%08x\n", sCmd->AlternateByteMode, sCmd->AlternateBytesSize, sCmd->AlternateBytes);
            DEBUG_PRINTF( "DDRmode=%d, SIOOmode=%d, HoldHalf=%d\n", sCmd->DdrMode, sCmd->SIOOMode, sCmd->DdrHoldHalfCycle);
        }
    #endif

#endif

/******************************************************************************
 * Setup the QSPI prescaler, so that the achieved operating freqency is at or
 * below ( due to the granularity of the prescaler ) "desired_frq"
 * @param myHandle    - my XSpi handle
 * @param clk_frq     - QSPI/OSPI clock frq[Hz]
 * @param desired_frq - desired operating frq[Hz]
 * @param flash_size  - flash size [Pwr of 2], ie 16 for "64kiB"
 *****************************************************************************/
bool XSpiLL_ClockInit(XSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size )
{  
    XXSPI_HandleTypeDef *hxspi = &myHandle->hxspi;
 
    /* calculate prescaler,so that the actual frequency is at or below desired frequency */ 
    uint32_t prescaler = ( clk_frq + desired_frq - 1 ) / desired_frq;
    if ( prescaler > 0 ) prescaler--;
    desired_frq = clk_frq / (prescaler+1);

    if ( prescaler > 255 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            LOG_WARN("XSpiLL_ClockInit - desired frq too low, minimum is %d\n", clk_frq/256 );
        #endif
        return false;
    }

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        LOG_INFO("%s: Flash Clk=%d\n", XSpiStr, desired_frq );
    #endif

    /* QSPI initialization */
    hxspi->Init.ClockPrescaler              = prescaler;
    hxspi->Init.FifoThreshold               = 4;
    #if USE_OSPI > 0
        hxspi->Init.DualQuad                = HAL_OSPI_DUALQUAD_DISABLE;
        hxspi->Init.MemoryType              = HAL_OSPI_MEMTYPE_MACRONIX;
        hxspi->Init.SampleShifting          = HAL_OSPI_SAMPLE_SHIFTING_NONE;
//        hxspi->Init.DeviceSize              = POSITION_VAL(flash_size) - 1;
        hxspi->Init.DeviceSize              = flash_size;
        hxspi->Init.ChipSelectHighTime      = 1;
        hxspi->Init.ClockMode               = HAL_OSPI_CLOCK_MODE_0;
        hxspi->Init.FreeRunningClock        = HAL_OSPI_FREERUNCLK_DISABLE;
        hxspi->Init.DelayHoldQuarterCycle   = HAL_OSPI_DHQC_DISABLE;
        hxspi->Init.ChipSelectBoundary      = 0;
        hxspi->Init.DelayBlockBypass        = HAL_OSPI_DELAY_BLOCK_USED;
    #else
        hxspi->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
//        hxspi->Init.FlashSize          = POSITION_VAL(flash_size) - 1;
        hxspi->Init.FlashSize          = flash_size;
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

/*******************************************************************************************
 * @brief execute the predefined command "cmd". 
 * @param retbuf    - if cmd has answer bytes, these are copied to retbuf
 * @param retlen    - maximum byte size of retbuf, will be overwritten with actual answer length
 * @param exec_time - if not NULL, execution time of cmd is returned herein
 * @retval true on success, otherwise false
 ******************************************************************************************/
uint32_t XSpiLL_ExecuteCmd( XSpiHandleT *myHandle, const NOR_FlashCmdT *cmd, uint32_t arg, uint8_t *argbuf, uint32_t *arglen, uint32_t *exec_time )
{    
  uint32_t argsize;
  if ( cmd->arg_size != 0 ) {
    argsize = ( cmd->arg_size < 0 ? cmd->arg_size * -1 : cmd->arg_size );
    /* Check for sufficient return data buffer size */
    if ( arglen ) {
        if ( *arglen < argsize ) {
            LOG_ERROR("XSpiLL_ExecuteCmd: r/w buffer too short");
            return false;
        }
        if (cmd->arg_size > 0) *arglen = cmd->arg_size;
    } else {
        LOG_ERROR("XSpiLL_ExecuteCmd: not argument length given");
        return false;
    }
  }
  if ( exec_time ) *exec_time = cmd->exec_time;
  return XHelper_CmdArgRead ( myHandle, cmd, arg, argbuf, cmd->arg_size  ); 
}

/*******************************************************************************************
 * @brief Active Wait for WIP bit to be reset 
 * @param myHandle - XSpi handle
 * @param Timeout  - Wait timeout in ms
 * @retval true on success, false on parameter error or timeout
 ******************************************************************************************/
bool XSpiLL_WaitForWriteDone(XSpiHandleT *myHandle, uint32_t Timeout) 
{
    /* get the "write in progress" bit query */
    const NOR_FlashQueryT *qry = myHandle->interface->query.wip;

    /* active wait for that bit to reset */
    return XHelper_WaitForBitsSetOrReset(myHandle, qry, Timeout, XSPI_MODE_POLL, 0);  
}

/*******************************************************************************************
 * @brief Interrupt Wait for WIP bit to be reset, Staus Match interrupt will be fired then
 * @param myHandle - XSpi handle
 * @retval true on success, false on parameter error
 ******************************************************************************************/
bool XSpiLL_WaitForWriteDone_IT(XSpiHandleT *myHandle) {
    /* get the "write in progress" bit query */
    const NOR_FlashQueryT *qry = myHandle->interface->query.wip;

    /* wait by interrupt for that bit to reset */
    return XHelper_WaitForBitsSetOrReset(myHandle, qry, 0, XSPI_MODE_IRQ, 0);  
}

/*******************************************************************************************
 * @brief enter deep power down state, device must not be in dpd state before
 * @param myHandle - XSpi handle
 * @retval execution or wait time until Deep power down state is active, 
 *         -1 in case of error or dpd state was active before
 ******************************************************************************************/
int32_t XSpi_SetDeepPowerDown(XSpiHandleT *myHandle, bool bEna)
{
    uint32_t exec_time;
    
    /* Assert deep power down operation is defined */
    const NOR_FlashCmdT *cmd;
    cmd = bEna ? myHandle->interface->cmd.dpen : myHandle->interface->cmd.dpdis;
    assert( cmd );
    
    /* Only execute, if mode actually changes */
    if ( (bEna && myHandle->bInDeepPwrDown) || (!bEna && !myHandle->bInDeepPwrDown) ) return -1;

    if ( !XSpiLL_ExecuteCmd(myHandle, cmd, 0, NULL, NULL, &exec_time ) ) return -1;
    
    myHandle->bInDeepPwrDown = bEna;
    return exec_time;
}


/*******************************************************************************************
 * @brief Set device to High performance mode, must not be in HP mode before
 * @param myHandle - XSpi handle
 * @param bEna - true=Enable, false=Disable
 * @retval execution or wait time until HP mode is active, 
 *         -1 in case of error or dpd state was active before
 ******************************************************************************************/
int32_t XSpiLL_SetHPMode(XSpiHandleT *myHandle, bool bEna)
{
    uint32_t exec_time;
    /* Assert deep power down enable cmd is defined */
    const NOR_FlashCmdT *cmd;
    cmd = bEna ? myHandle->interface->cmd.hpen : myHandle->interface->cmd.hpdis;
    assert( cmd );
    
    /* Only execute, if mode actually changes */
    if ( (bEna && myHandle->bInHPMode) || (!bEna && !myHandle->bInHPMode) ) return -1;

    if ( !XSpiLL_ExecuteCmd(myHandle, cmd, 0, NULL, NULL, &exec_time ) ) return -1;
    
    myHandle->bInHPMode = bEna;
    return exec_time;
}

/*******************************************************************************************
 * @brief leave High performance mode
 * @param myHandle - XSpi handle
 * @retval execution or wait time until HP mode is left
 *         -1 in case of error
 ******************************************************************************************/
int32_t XSpiLL_LeaveHPMode(XSpiHandleT *myHandle)
{
    uint32_t exec_time;
    /* Assert deep power down disable cmd is defined */
    const NOR_FlashCmdT *cmd = myHandle->interface->cmd.hpdis;
    assert( cmd );

    if ( !myHandle->bInHPMode || !XSpiLL_ExecuteCmd(myHandle, cmd, 0, NULL, NULL, &exec_time ) ) return -1;

    myHandle->bInHPMode = false;
    return exec_time;
}

/*******************************************************************************************
 * @brief  This function reset the QSPI memory.
 * @param  hxspi : QSPI handle
 * @retval true on success, false otherwise
 ******************************************************************************************/
bool XSpiLL_ResetMemory(XSpiHandleT *myHandle)
{
    uint8_t reset_bytes[2];
    uint8_t cnt = 0;

    /* Reset enable command */
    const NOR_FlashCmdT *cmd = myHandle->interface->cmd.resen;
    if ( cmd ) reset_bytes[cnt++] = cmd->cmd;

    /* Reset command */
    cmd = myHandle->interface->cmd.reset;
    if ( ! cmd ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpiLL_ResetMemory - Error: no Reset command");
        #endif
        return false;
    }

    reset_bytes[cnt++] = cmd->cmd;

    if ( !XHelper_SendCommandBytesSerial(myHandle, reset_bytes, cnt ) ) return false;

    /* Configure automatic polling mode to wait the memory is ready */  
    if ( !XSpiLL_WaitForWriteDone(myHandle, XSPI_TIMEOUT_DEFAULT_VALUE) ) return false;

    return true;
}


/*******************************************************************************************
 * @brief  Set or Reset Write Enable Flag
 * @param  hxspi : QSPI handle
 * @param  bEna : true = set , false = Reset
 * @retval true on success, false otherwise
 ******************************************************************************************/
bool XSpiLL_WriteEnable(XSpiHandleT *myHandle, bool bEna)
{
    /* Send write Enable cmd*/
    const NOR_FlashCmdT *cmd;
    cmd = bEna ? myHandle->interface->cmd.wen : myHandle->interface->cmd.wdis;
    assert( cmd );

    if ( myHandle->bInHPMode || !XSpiLL_ExecuteCmd(myHandle, cmd, 0, NULL, NULL, NULL ) ) return false;


    /* active wait for Write Enable bit to set or reset */
    const NOR_FlashQueryT *qry = myHandle->interface->query.wel;
    assert( qry );
   return XHelper_WaitForBitsSetOrReset(myHandle, qry, XSPI_TIMEOUT_DEFAULT_VALUE, XSPI_MODE_POLL, bEna ? 1 : 0);  
}

/******************************************************************************
 * Return the flash memory chip ID
 * @param buf - writebuffer to write to ID to, requires at least 3 bytes 
 * @return XSPI_OK if successfully read, SSPI_ERROR in case of failure
 * @note the buffer will contain 4 bytes with this info
 *       offset 0-1 : Device ID Byte1 and 2 
 *       offset 2   : Manufacturer ID
 *       offset 3   : Device Type or Density ?  
 *****************************************************************************/
bool XSpiLL_GetID(XSpiHandleT *myHandle)
{
  const NOR_FlashCmdListT *cmd = &myHandle->interface->cmd;
  /* Read the 2.byte device ID first, first byte MfgID will be overwritten later */
  if ( cmd->r_ems ) {
    if (!XHelper_CmdArgRead ( myHandle, cmd->r_ems, 0, myHandle->id+2, 2 ) ) return false;
  }

  /* Read 3 byte JEDEC ID,  */
  if ( cmd->r_jid ) {   
    if (!XHelper_CmdArgRead ( myHandle, cmd->r_jid, 0, myHandle->id, 3 ) ) return false;
  } else {
    #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
        DEBUG_PUTS("XSpiLL_GetID - Error: Read JEDEC ID cmd not set");
    #endif
    return false;
  }
  return true;
}

/**
  * @brief  Erases the specified block of the QSPI memory. 
  * @param  BlockAddress : Block address to erase  
  * @retval QSPI memory status
  */
bool XSpiLL_Erase(XSpiHandleT *myHandle, uint32_t Address, const NOR_FlashCmdT  *ecmd )
{
    /* Enable write operations */
    if ( !XSpiLL_WriteEnable(myHandle, true) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpiLL_Erase - Error: Write enable failed");
        #endif
        return false;
    }

    /* Send the command */
    if(!XSpiLL_ExecuteCmd(myHandle, ecmd, Address, NULL, NULL, NULL ) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpiLL_Erase - Error: execution failed");
        #endif
      return false;
    }

    return true;
}

/**************************************************************************************************
 * @brief  Configure the QSPI in memory-mapped mode
 * @retval QSPI memory status
 *************************************************************************************************/
bool XSpi_EnableMemoryMappedMode(XSpiHandleT *myHandle)
{
    XSPI_CommandTypeDef      sCommand={0};
    XSPI_MemoryMappedTypeDef sMemMappedCfg={0};
    const NOR_RWModeTypeT*   rd;

    /* Wake up device, if deep sleep capability and in in deep sleep */
    if ( myHandle->bInDeepPwrDown && XSpi_SetDeepPowerDown(myHandle,false)==-1 ) return false;

    /* Get a continuous quad read mode: first try 444 then 144 */
    if ( rd = myHandle->interface->read.rf_444c, !rd ) rd = myHandle->interface->read.rf_144c;
        
    if ( !rd ) {
        /* if no continuous read mode was found, return false */ 
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpi_EnableMemoryMappedMode - Error: No cont. read mode found");
        #endif
        return false;
    }
    
    /* Initialize the read command */
    if ( !XHelper_SetupCommand( &sCommand, &rd->cmd, 3, 0, 1 )) return false;

    /*
     *  SetupCommand does not handle Alternate Bytes, so if alternate bytes are
     *  required, set them here for continuous read
     */
    if ( rd->mode_bytes > 0 ) XHelper_SetupAltBytes(&sCommand, rd, myHandle->interface->read.continuous_rd_on );

    /* Configure and enable memory mapped mode */
    sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_XSPI_MemoryMapped( &myHandle->hxspi, &sCommand, &sMemMappedCfg) != HAL_OK) return false;

    myHandle->bIsMemoryMapped = true;
    return true;
}


/******************************************************************************
 * Read Operation command
 *****************************************************************************/
bool XSpiLL_ReadCMD(XSpiHandleT *myHandle, uint32_t Addr, uint32_t Size)
{
    XSPI_CommandTypeDef sCommand={0};

    /* get the read command. that suits the select RWmode  (1-1-1, 1-2-2 or 1-4-4) */
    const NOR_RWModeTypeT* rd = XHelper_FindReadCmd( myHandle );
    if ( !rd ) return false;

#if XSPI_TUNE_MANUALLY > 0
        NOR_RWModeTypeT r2 = *rd;
        if ( myHandle->bReadTuning ) r2.cmd.dummy_cycles = myHandle->waitCycles;
        if ( !XHelper_SetupCommand( &sCommand, &r2.cmd, 3, Addr, Size )) return false;
#else
    /* Initialize the read command */
    if ( !XHelper_SetupCommand( &sCommand, &rd->cmd, 3, Addr, Size )) return false;
#endif

    /*
     *  SetupCommand does not handle Alternate Bytes, so if alternate bytes are
     *  required, set them here 
     */
    if ( rd->mode_bytes > 0 ) XHelper_SetupAltBytes(&sCommand, rd, myHandle->interface->read.continuous_rd_off );
 
     #if DEBUG_MODE > 0 && DEBUG_XSPI > 1
        XSpiLL_DumpScmd(&sCommand);
    #endif

    /* Excute read */
    if (HAL_XSPI_Command(&myHandle->hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpi_Read - Error: Command not send");
        #endif
        return false;
    }

    return true;
}

/******************************************************************************
 * Read Operation command
 *****************************************************************************/
bool XSpiLL_WriteCMD(XSpiHandleT *myHandle, uint32_t Addr, uint32_t Size)
{
    XSPI_CommandTypeDef sCommand={0};

    /* get the read command. that suits the select RWmode  (1-1-1, 1-2-2 or 1-4-4) */
    const NOR_RWModeTypeT* wr = XHelper_FindWriteCmd( myHandle );
    if ( !wr ) return false;

    /* Enable write operations */
    if ( !XSpiLL_WriteEnable(myHandle, true) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpiLL_WriteCMD - Error: Write enable failed");
        #endif
        return false;
    }

    /* Initialize the read command */
    if ( !XHelper_SetupCommand( &sCommand, &wr->cmd, 3, Addr, Size )) return false;

    /* Send write command */
    if (HAL_XSPI_Command(&myHandle->hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpi_Read - Error: Command not send");
        #endif
        return false;
    }

    return true;
}


/******************************************************************************
 * @brief Execute one query plus bit operation and write the result to writebuf
 * @param  myHandle - XSpi handle
 * @param  op - query + bitoperation
 * @param  writebuf - result write buffer
 * @returns length of query result in bytes or -1 in case of error
 *****************************************************************************/
static int8_t ExecuteOp(XSpiHandleT *myHandle, const NOR_FlashOpT *op, uint8_t *writebuf, uint32_t *numarg)
{
    /* Length of query result */
    uint32_t qlen = op->qry->exec->arg_size;
    uint32_t work;
    if ( qlen > 4 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PRINTF("XSpiLL_ExecuteOp: Query length of %d exceededs limit of 4\n", qlen);
        #endif
        return -1;
    }
    uint32_t arglen = 4;
    /* Execute query */
    if ( !XSpiLL_ExecuteCmd(myHandle, op->qry->exec, 0, (uint8_t*)&work, &arglen, NULL ) ) return -1;

    /* Execute bit operation */
    switch(op->op ) {
        case XSPI_BITOP_NONE:   // No Operation
            break;
        case XSPI_BITOP_SET:    // Set queried bits 
            work |= ( op->qry->access_mask << op->qry->access_shift );
            break;
        case XSPI_BITOP_RESET:  // Reset queried bits 
            work &= ~( op->qry->access_mask << op->qry->access_shift );
            break;
        case XSPI_BITOP_SETNUM: // Set queried to a specified value
            if ( !numarg ) {
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    DEBUG_PRINTF("XSpiLL_ExecuteOp, Op SETNUM: numarg not specified\n");
                #endif
                return -1;
            }
            /* clear masked bits */
            work &= ~( op->qry->access_mask << op->qry->access_shift );  
            /* fill in numarg */
            work |= ((*numarg & op->qry->access_mask ) << op->qry->access_shift ); 
            break;
        default:
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                DEBUG_PRINTF("XSpiLL_ExecuteOp: undefined opcode of %d\n",op->op);
            #endif
            return -1;
    }

    /* copy reault to write buffer */
    for ( uint8_t i = 0; i < arglen; i++ ) {
        *(writebuf+i) = work & 0xff;
        work >>= 8;
    }
    return (int8_t)arglen;
}

/******************************************************************************
 * @brief Execute a set or reset command
 * @param  myHandle - XSpi handle
 * @param  rs - Set or Reset command sequence
 * @param  numarg - numerical value in case of XSPI_BITOP_SETNUM command
 *                  must be at least as long as the list of operations
 *****************************************************************************/
bool XSpiLL_Execute ( XSpiHandleT *myHandle, const NOR_FlashSetterT* rs, uint32_t numarg[] )
{
    static const uint8_t MAX_LEN = 8;
    uint8_t writebuf[MAX_LEN];
    uint32_t wrptr = 0;
    int8_t len;
    const NOR_FlashOpT *op;
    bool ret=true, bWasDPD=myHandle->bInDeepPwrDown;

    /* wake up is in Deep Sleep */ 
    if ( bWasDPD ) XSpi_SetDeepPowerDown(myHandle, false);

    /* Iterate thru all read operations/manipulations */
    for ( uint8_t i=0; i < FLASH_MAX_OPS; i++ ) {
        if ( op = rs->ops[i], op ) {
            /* check the argsize being >= 0 and total length not exceeding write buffer */
            len = op->qry->exec->arg_size;
            if ( len < 0 || wrptr+len > MAX_LEN ) {
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    DEBUG_PRINTF("XSpiLL_Execute: actual len=%d, total len=%d: limits exceeded\n", len, wrptr);
                #endif
                ret=false;
                break;
            }
            if ( len = ExecuteOp(myHandle, op, writebuf+wrptr, numarg ? numarg+i : NULL ), len < 0 ) { ret=false; break; }
            wrptr+= len; 
        }
    }

    if ( ret ) {
        /* check that length of all read operations matches write op length */
        /* argsize is < 0 for write commands */
        if ( wrptr + rs->set_cmd->arg_size != 0 ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                DEBUG_PRINTF("XSpiLL_Execute: Rd (%d) and Wr size (%d) do not match\n", wrptr, rs->set_cmd->arg_size*-1);
            #endif
            return false;
        }
    
        ret = XHelper_CmdArgRead ( myHandle, rs->set_cmd, 0, writebuf, rs->set_cmd->arg_size );
    }

    /* if in Deep Sleep before go to deep sleep again*/ 
    if ( bWasDPD ) XSpi_SetDeepPowerDown(myHandle, true);
    return true;
}
/* Exported functions -------------------------------------------------------*/


#endif /* #if  USE_QSPI > 0 || USE_OSPI > 0  */
