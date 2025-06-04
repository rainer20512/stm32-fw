/*******************************************************************************
 * @file    xspi_helper.c
 * @author  Rainer
 * @brief   QSPI/OSPI helper functions for all kinds of specific Flash drivers
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
    static const uint32_t cmd_lines[5]  = { HAL_OSPI_INSTRUCTION_NONE,      HAL_OSPI_INSTRUCTION_1_LINE,      HAL_OSPI_INSTRUCTION_2_LINES,     0,                                HAL_OSPI_INSTRUCTION_4_LINES, };
    static const uint32_t addr_size[5] =  { 0,                              HAL_OSPI_ADDRESS_8_BITS,          HAL_OSPI_ADDRESS_16_BITS,         HAL_OSPI_ADDRESS_24_BITS,         HAL_OSPI_ADDRESS_32_BITS, };
    static const uint32_t addr_lines[5] = { HAL_OSPI_ADDRESS_NONE,          HAL_OSPI_ADDRESS_1_LINE,          HAL_OSPI_ADDRESS_2_LINES,         0,                                HAL_OSPI_ADDRESS_4_LINES, };
    static const uint32_t data_lines[5] = { HAL_OSPI_DATA_NONE,             HAL_OSPI_DATA_1_LINE,             HAL_OSPI_DATA_2_LINES,            0,                                HAL_OSPI_DATA_4_LINES, };
    static const uint32_t alt_lines[5]  = { HAL_OSPI_ALTERNATE_BYTES_NONE,  HAL_OSPI_ALTERNATE_BYTES_1_LINE,  HAL_OSPI_ALTERNATE_BYTES_2_LINES, 0,                                HAL_OSPI_ALTERNATE_BYTES_4_LINES, };
    static const uint32_t alt_size[5]   = { 0,                              HAL_OSPI_ALTERNATE_BYTES_8_BITS,  HAL_OSPI_ALTERNATE_BYTES_16_BITS, HAL_OSPI_ALTERNATE_BYTES_24_BITS, HAL_OSPI_ALTERNATE_BYTES_32_BITS, };
#else
    static const uint32_t cmd_lines[5]  = { QSPI_INSTRUCTION_NONE,     QSPI_INSTRUCTION_1_LINE,      QSPI_INSTRUCTION_2_LINES,     0,                            QSPI_INSTRUCTION_4_LINES, };
    static const uint32_t addr_size[5]  = { 0,                         QSPI_ADDRESS_8_BITS,          QSPI_ADDRESS_16_BITS,         QSPI_ADDRESS_24_BITS,         QSPI_ADDRESS_32_BITS, };
    static const uint32_t addr_lines[5] = { QSPI_ADDRESS_NONE,         QSPI_ADDRESS_1_LINE,          QSPI_ADDRESS_2_LINES,         0,                            QSPI_ADDRESS_4_LINES, };
    static const uint32_t data_lines[5] = { QSPI_DATA_NONE,            QSPI_DATA_1_LINE,             QSPI_DATA_2_LINES,            0,                            QSPI_DATA_4_LINES, };
    static const uint32_t alt_lines[5]  = { QSPI_ALTERNATE_BYTES_NONE, QSPI_ALTERNATE_BYTES_1_LINE,  QSPI_ALTERNATE_BYTES_2_LINES, 0,                            QSPI_ALTERNATE_BYTES_4_LINES, };
    static const uint32_t alt_size[5]   = { 0,                         QSPI_ALTERNATE_BYTES_8_BITS,  QSPI_ALTERNATE_BYTES_16_BITS, QSPI_ALTERNATE_BYTES_24_BITS, QSPI_ALTERNATE_BYTES_32_BITS, };
#endif

const uint8_t CmdLines[]    = CMD_LINES;
const uint8_t AddrLines[]   = ADDR_LINES;
const uint8_t DataLines[]   = DATA_LINES;


/* Exported functions -------------------------------------------------------*/

/******************************************************************************
 * Fill a CommandTypeDef structure completely, ie ready to execute 
 * only restriction: AlternateByteMode is always NONE
 * @param sCmd     - CommandTypeDef struct to be filled
 * @param cmd      - command to be executed, fields "cmd", "rw_mode", "dummy_cycles"
 *                   must be filled. "cmd_argsize" and "answersize" are ignored
 * @param arglen   - length of cmd argument[Bytes] ( instead of cmd.cmdarg_size )
 * @param arg      - argument value, only used when arglen > 0
 * @param retlen   - length if answer[Bytes] ( instead of cmd.answer_size )
 * @returns          true, if everything is ok, false otherwise
 ******************************************************************************/
bool XHelper_SetupCommand( XSPI_CommandTypeDef *sCmd, const NOR_FlashCmdT *cmd, uint32_t arglen, uint32_t arg, uint32_t retlen )
{

    /* check argument length, max is 32 bit */
    if ( arglen > 4 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("CmdArgRead - Error: Argument vector too long");
        #endif
        return false;
    }

    #if USE_OSPI > 0
        sCmd->OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCmd->FlashId           = HAL_OSPI_FLASH_ID_1;
        sCmd->InstructionMode   = cmd_lines[CmdLines[cmd->rw_mode]];
        sCmd->AddressMode       = arglen > 0 ? addr_lines[AddrLines[cmd->rw_mode]] : HAL_OSPI_ADDRESS_NONE;
        sCmd->AddressSize       = adr_size[arglen];
        sCmd->AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCmd->DataMode          = retlen ? data_lines[DataLines[cmd->rw_mode]] : HAL_OSPI_DATA_NONE; 
        sCmd->DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCmd->SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
    #else
        sCmd->InstructionMode   = cmd_lines[CmdLines[cmd->rw_mode]];
        sCmd->AddressMode       = arglen > 0 ? addr_lines[AddrLines[cmd->rw_mode]] : QSPI_ADDRESS_NONE;
        sCmd->AddressSize       = addr_size[arglen];
        sCmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        sCmd->DataMode          = retlen ? data_lines[DataLines[cmd->rw_mode]] : QSPI_DATA_NONE;
        sCmd->DdrMode           = QSPI_DDR_MODE_DISABLE;
        sCmd->DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        sCmd->SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    #endif

    sCmd->Instruction           = cmd->cmd;
    sCmd->DummyCycles           = cmd->dummy_cycles;
    sCmd->Address               = arglen > 0 ? arg : 0;
    sCmd->NbData                = retlen;

    #if DEBUG_MODE > 0 && DEBUG_XSPI > 1
        DEBUG_PRINTF("SetupCommand: Cmd=0x%02x, RWMode=%d, CmdLines=%d AddrLines=%d AddrSize=%d, DataLines=%d, DummyCyc=%d, DataLen=%d\n",
                      cmd->cmd, cmd->rw_mode, CmdLines[cmd->rw_mode], arglen > 0 ? AddrLines[cmd->rw_mode]:0, arglen, DataLines[cmd->rw_mode],cmd->dummy_cycles, retlen );
    #endif

    return true;
}

/******************************************************************************
 * Setup Alt Bytes for a read command
 * @param sCmd      - CommandTypeDef struct to be filled
 * @param alt_bytes - max 4 alt by 
 ******************************************************************************/
void XHelper_SetupAltBytes( XSPI_CommandTypeDef *sCmd, const NOR_RWModeTypeT *rd, uint32_t alt_bytes)
{
    if ( rd->mode_bytes > 0 ) {
        if ( rd->mode_bytes > 4 ) {
            #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                DEBUG_PUTS("SetupAltBytes - Error: #AltBytes too big");
            #endif
        }
        sCmd->AlternateByteMode  = alt_lines[AddrLines[rd->cmd.rw_mode]];
        sCmd->AlternateBytesSize = alt_size[rd->mode_bytes];
        sCmd->AlternateBytes     = alt_bytes;
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 1
            DEBUG_PRINTF("SetupAltBytes: Cmd=0x%02x, RWMode=%d, AltLines=%d AltSize=%d, AltBytes=0x%08x\n",
                          rd->cmd.cmd, rd->cmd.rw_mode, AddrLines[rd->cmd.rw_mode], rd->mode_bytes, alt_bytes );
        #endif
    }
}
    
/**
  * @brief  Send a set of single bytes commands serially, ie. one by one 
  *         with NCS going high in between the single bytes
  * @param  myHandle : XSPI handle
  * @param  bytes    : byte vector to send
  * @param  num      : number of bytes in sector 
  * @retval true, if all sent successfully, false otherwise
  */
bool XHelper_SendCommandBytesSerial(XSpiHandleT *myHandle, uint8_t *bytes, uint32_t num)
{
  XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
  XSPI_CommandTypeDef sCommand     = {0};
  
  /* Initialize transfer structure */
  #if USE_OSPI > 0
    sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.AddressMode       = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
  #else
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  #endif

  for ( uint32_t i=0; i < num; i++ ) {
      sCommand.Instruction = *(bytes++ );
      if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  } // for

  return true;
}

/**************************************************************************************
 * @brief  Perform an AutoPoll for a certain bit of a one byte read command to be RESET
           eg: one certain bit of status register
 * @param  myHandle : XSPI handle
 * @param  Timeout  : Timeout for auto-polling
 * @param  cmd      : one byte read CMD
 * @param  mask     : mask byte
 * @param  opmode   : one of XSPI_MODE_POLL, XSPI_MODE_IRQ, XSPI_MODE_DMA
 + @param  bSet     : true = wait for bit(s) set, false = wait for bit(s) reset
 * @retval true if bit was reset before timeout
 *         false if timeout occured
 *************************************************************************************/
bool XHelper_WaitForBitsSetOrReset(XSpiHandleT *myHandle, const NOR_FlashQueryT *qry, uint32_t Timeout, uint32_t opmode, bool bSet)
{
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;
    XSPI_CommandTypeDef     sCommand ={0};
    XSPI_AutoPollingTypeDef sConfig  ={0};

    uint8_t addrlen = qry->exec->cmdaddr_size;
    
    /* Check for argument size > 0 , ie read command */
    if( qry->exec->arg_size <= 0 ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("WaitForBitSet/Reset: Not a read cmd");
        #endif
        return false;
    }


    /* Configure automatic polling mode to wait for memory ready */  
    #if USE_OSPI > 0
        sCommand.OperationType     = HAL_OSPI_OPTYPE_COMMON_CFG;
        sCommand.FlashId           = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode   = HAL_OSPI_INSTRUCTION_1_LINE;
        sCommand.AddressMode       = HAL_OSPI_ADDRESS_1_LINE;
        sCommand.AddressSize       = adr_size[addrlen];
        sCommand.AlternateBytesMode= HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = HAL_OSPI_DATA_1_LINE;
        sCommand.DataDtrMode       = HAL_OSPI_DATA_DTR_DISABLE;
        sCommand.SIOOMode          = HAL_OSPI_SIOO_INST_EVERY_CMD;
        }
    #else
        sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
        sCommand.AddressSize       = addr_size[addrlen];
        sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = QSPI_DATA_1_LINE;
        sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
        sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
        sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    #endif 
   
    if ( bSet ) {
        /* config for bits set */
        sConfig.Match           = qry->access_mask << qry->access_shift;
    } else {
        /* config for bits reset */
        sConfig.Match           = 0;
    }
    /* 
     * dummy cycle are fixed set 0, because we don't have dummy cycles coded 
     * in ordinary commands. could be improved in future
     */
    sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
    sConfig.Mask            = qry->access_mask << qry->access_shift;
    sCommand.DummyCycles    = 0 ;
    sCommand.Instruction    = qry->exec->cmd;
    sConfig.Interval        = 0x10;
    sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
    sConfig.StatusBytesSize = (uint32_t)qry->exec->arg_size;

    switch(opmode) {
        case XSPI_MODE_IRQ:
        case XSPI_MODE_DMA:
        #if USE_OSPI > 0
            if (HAL_XSPI_AutoPolling_IT(hxspi, &sConfig) != HAL_OK) {
        #else
            if (HAL_XSPI_AutoPolling_IT(hxspi, &sCommand, &sConfig) != HAL_OK) {
        #endif
                #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
                    DEBUG_PUTS("XSpi_AutoPolling_IT - Error: Setup failed");
                #endif
                /* Automatic poling mode must be aborted in case of timeout */
                hxspi->State = HAL_QSPI_STATE_BUSY;
                HAL_QSPI_Abort_IT(hxspi);
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
                    DEBUG_PUTS("OSpi_AutoPolling - Error: Timeout when waiting for write done");
                #endif
                /* Automatic poling mode must be aborted in case of timeout */
                hxspi->State = HAL_QSPI_STATE_BUSY;
                HAL_QSPI_Abort(hxspi);
                return false;
            }
    } // switch
    return true;
}



/******************************************************************************
 * Write one-byte-cmd, optional cmd-arg vector, clock out dummy bytes,
 * optional read answer vector 
 * @param  myHandle  : XSPI handle
 * @param  cmd       : command byte
 * @param  addr      : command address vector [0 ... 4 bytes]
 * @param  addrlen   : length of address vector  [ 0 ... 4 ]
 * @param  dummy_cyc : number of dummy cycles before readout [ 0 ... n ]
 * @param  retbuf    : return buffer for the answer bytes
 * @param  retlen    : number of expected answer bytes to read or bytes to write when < 0 
 * @return true on success,
 *         false in case of failure
 * @note   All command phases are assumed to be 1 line !!!
 *****************************************************************************/
bool XHelper_CmdArgRead ( XSpiHandleT *myHandle, const NOR_FlashCmdT *cmd, uint32_t addr, uint8_t *retbuf, int32_t retlen )
{
    XSPI_CommandTypeDef sCommand={0};
    XXSPI_HandleTypeDef *hxspi       = &myHandle->hxspi;

    uint32_t addrlen = cmd->cmdaddr_size;

    if ( !XHelper_SetupCommand(&sCommand, cmd, addrlen, addr, retlen < 0 ? retlen * -1 : retlen ) ) return false;
  
    /* Send Command */
    if (HAL_XSPI_Command(hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

    /* Send command arguments */
    if        ( retlen < 0 ) {
        if (HAL_XSPI_Transmit(hxspi, retbuf , XSPI_TIMEOUT_DEFAULT_VALUE)  != HAL_OK) return false;
    } else if ( retlen > 0 )  {
        /* if anything to read, then read */
        if (HAL_XSPI_Receive(hxspi, retbuf, XSPI_TIMEOUT_DEFAULT_VALUE)  != HAL_OK) return false;
    }

    return true;
}


/******************************************************************************
 * returns true, iff Flash device has deep power down mode                    *
 * this is the case, if both commands for entering and leaving DPD are set    *
 *****************************************************************************/
bool XHelper_HasDeepPowerDown (XSpiHandleT *myHandle )
{
    return ( myHandle->interface->cmd.dpdis && myHandle->interface->cmd.dpen );
}

/******************************************************************************
 * returns true, iff Flash device has high performance mode                   *
 * this is the case, if both commands for entering and leaving HP mode are set*
 *****************************************************************************/
bool XHelper_HasHighPerformanceMode (XSpiHandleT *myHandle )
{
    return ( myHandle->interface->cmd.hpdis && myHandle->interface->cmd.hpen );
}

/******************************************************************************
 * Set the geometry data according to flash interface definitions
 *****************************************************************************/
void XHelper_SetGeometry(XSpiHandleT *myHandle, XSpiGeometryT *geometry)
{
    #define PWR2DIV(a,b)    (1<<((a)-(32-__CLZ(b))))
    
    uint32_t flash_size, size;
    
    /* FlashSize is a power of 2 value ! */
    flash_size = myHandle->interface->global.size;
    geometry->FlashSize          = flash_size;      
    geometry->ProgPageSize       = myHandle->interface->write.page_size;
    geometry->ProgPagesNumber    = PWR2DIV(flash_size,geometry->ProgPageSize);
   
    for ( uint32_t i=0; i < MAX_ERASE; i++ ) {
        if ( myHandle->interface->erase.p_erase[i] )
            size = myHandle->interface->erase.p_erase[i]->block_size;
        else
            size = 0;
        geometry->EraseSize[i]=size;
        geometry->EraseNum[i] = size > 0 ? PWR2DIV(flash_size,size) : 0;
    }

    #if DEBUG_MODE > 0
        /* Check validity */
        #define PWROF2(a)   ( (a & (a-1)) == 0 ) 
        if ( !PWROF2(geometry->ProgPageSize) ) {
            LOG_WARN("Page size 0x%x is not a power of 2, sure?", geometry->ProgPageSize);
        }
        for ( uint32_t i=0; i < MAX_ERASE; i++ ) {
            size = geometry->EraseSize[i];
            if ( size > 0 && !PWROF2(size) ) {
                LOG_WARN("Erase element %d: size 0x%x is not a power of 2, sure?", i, size);
            }
        }
    #endif

}

/******************************************************************************
 * @brief Get the read command from flash interface, that meets the
 * currently selected Read or write mode ( 1-1-1, 1-2-2 or 1-4-4 )
 * @param   myHandle - XSpi handle
 * @returns ptr to suitable read command, NULL if not defined
 *****************************************************************************/
const NOR_RWModeTypeT* XHelper_FindReadCmd( XSpiHandleT *myHandle )
{
    NOR_FlashReadT const *rd = &myHandle->interface->read;
    switch ( myHandle->myRWmode ) {
        case XSPI_RW_FAST1:
            /* When fast read is defined, then fast read, otherwise normal read */
            if ( rd->rf_111 ) return rd->rf_111;
            if ( rd->r_111 ) return rd->r_111;
            break;
        case XSPI_RW_DUAL2:
            /* if 1-2-2 is defined then return this, otherwise 1-1-2 */
            if ( rd->rf_122 ) return rd->rf_122;
            if ( rd->rf_112 ) return rd->rf_112;
            break;
        case XSPI_RW_QUAD4:
            /* 4-4-4, then 1-4-4, then 1-1-4 */
            if ( rd->rf_444 ) return rd->rf_444;
            if ( rd->rf_144 ) return rd->rf_144;
            if ( rd->rf_114 ) return rd->rf_114;
            break;
    }

    /* No suitable read option was found */
    return NULL;    
}

/******************************************************************************
 * @brief Get the write command from flash interface, that meets the
 * currently selected Read or write mode ( 1-1-1, 1-2-2 or 1-4-4 )
 * @param   myHandle - XSpi handle
 * @returns ptr to suitable read command, NULL if not defined
 *****************************************************************************/
const NOR_RWModeTypeT* XHelper_FindWriteCmd( XSpiHandleT *myHandle )
{
    NOR_FlashWriteT const *wr = &myHandle->interface->write;
    switch ( myHandle->myRWmode ) {
        case XSPI_RW_FAST1:
            /* When fast read is defined, then fast read, otherwise normal read */
            if ( wr->p_111 ) return wr->p_111;
            break;
        case XSPI_RW_DUAL2:
            /* DUAL QSPI for write currently not supported */
            break;
        case XSPI_RW_QUAD4:
            /* 4-4-4, then 1-4-4, then 1-1-4 */
            if ( wr->p_444 ) return wr->p_444;
            if ( wr->p_144 ) return wr->p_144;
            if ( wr->p_114 ) return wr->p_114;
            break;
    }

    /* No suitable read option was found */
    return NULL;    
}

#endif /* #if  USE_QSPI > 0 || USE_OSPI > 0  */
