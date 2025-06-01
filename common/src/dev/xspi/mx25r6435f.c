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
#include "dev/xspi/xspi_helper.h"
#include "dev/xspi/flash_interface.h"
#include "log.h"

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


#include <stdio.h>


#define MX25_QUAD_DISABLE       0x0
#define MX25_QUAD_ENABLE        0x1

#define MX25_HIGH_PERF_DISABLE  0x0
#define MX25_HIGH_PERF_ENABLE   0x1

#define MX25_DLY_TO_SLEEP       30              /* min time [us] the flash needs to enter deep sleep */
#define MX25_DLY_FM_SLEEP       45              /* min time [us] the flash needs to exit deep sleep  */
                                                /* this is the highr value when in high perf mode    */
/* WIP flag is bit 0 in SR */
#define WIP_MASK 0b1
#define WIP_SHFT 0

/* Write Enable flag is bit 1 in SR */
#define WEL_MASK 0b1
#define WEL_SHFT 1

/* Quad Enable flag is bit 6 in SR */
#define QE_MASK 0b1
#define QE_SHFT 6

/* SR write protect flag is bit 7 in SR */
#define SRWP_MASK 0b1
#define SRWP_SHFT 7


/* Block protect bits are bit2 ... bit5 in SR */
#define BP_MASK 0x0f
#define BP_SHFT 2

/* Top/Bottom protect bit is bit3 in CR word */
#define TB_MASK 0b01
#define TB_SHFT 3

/* Dummy cyc. config bit is bit 6 in CR word */
#define DC_MASK 0b01
#define DC_SHFT 6

/* Low/High Pwr cnfig bit is bit9 in CR word */
#define HP_MASK 0b01
#define HP_SHFT 9

/* Program Suspend bit is bit2 in SCUR */
#define PSB_MASK 0b01
#define PSB_SHFT 2

/* Erase Suspend bit is bit3 in SCUR */
#define ESB_MASK 0b01
#define ESB_SHFT 3

/* Program fail bit is bit5 in SCUR */
#define PFAIL_MASK 0b01
#define PFAIL_SHFT 5

/* Erase fail bit is bit6 in SCUR */
#define EFAIL_MASK 0b01
#define EFAIL_SHFT 6



/* Flash Interface definitions ---------------------------------------------------------*/

/* All frquencies in MhZ */
/* Maximum frequency in Ultra low power mode */
#define XSPI_MAX_ULP_FREQUENCY  8

/* safe operating speed to readout ID values */
#define XSPI_INITIAL_SPEED      1   

static const uint8_t OP_FREQ = 8;      

static const uint8_t OP_FR   = 33;      /* Frequency FR  from datasheet */
static const uint8_t OP_LPR  = 8;       /* Frequency for all read Ops except 111 read in Low power mode */

/* Dummy cycle for ultra low poer mode, data sheet p.7 */
static const NOR_RWModeTypeT  r_111     = { .cmd = {.cmd = 0x03, .rw_mode = XSPI_MODE_111, .dummy_cycles = 0, }, .mode_bytes = 0, .max_frq = OP_FR, };
static const NOR_RWModeTypeT  rf_111    = { .cmd = {.cmd = 0x0b, .rw_mode = XSPI_MODE_111, .dummy_cycles = 8, }, .mode_bytes = 0, .max_frq = OP_LPR, };
static const NOR_RWModeTypeT  rf_112    = { .cmd = {.cmd = 0x3b, .rw_mode = XSPI_MODE_112, .dummy_cycles = 8, }, .mode_bytes = 0, .max_frq = OP_LPR, };
static const NOR_RWModeTypeT  rf_114    = { .cmd = {.cmd = 0x6b, .rw_mode = XSPI_MODE_114, .dummy_cycles = 8, }, .mode_bytes = 0, .max_frq = OP_LPR, };
static const NOR_RWModeTypeT  rf_122    = { .cmd = {.cmd = 0xbb, .rw_mode = XSPI_MODE_122, .dummy_cycles = 4, }, .mode_bytes = 1, .max_frq = OP_LPR, };
static const NOR_RWModeTypeT  rf_144    = { .cmd = {.cmd = 0xeb, .rw_mode = XSPI_MODE_144, .dummy_cycles = 6, }, .mode_bytes = 1, .max_frq = OP_LPR, };

static const NOR_RWModeTypeT  p_111     = { .cmd = {.cmd = 0x02, .rw_mode = XSPI_MODE_111, .dummy_cycles = 0, }, .pgm_time = 10000 };
static const NOR_RWModeTypeT  p_144     = { .cmd = {.cmd = 0x38, .rw_mode = XSPI_MODE_144, .dummy_cycles = 0, }, .pgm_time = 10000 };

static const NOR_EraseModeTypeT e_01    = { .cmd = { .cmd = 0x20, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=240000, },    .block_size = 0x1000, };
static const NOR_EraseModeTypeT e_02    = { .cmd = { .cmd = 0x52, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=3000000, },   .block_size = 0x8000, };
static const NOR_EraseModeTypeT e_03    = { .cmd = { .cmd = 0xd8, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=3500000, },   .block_size = 0x10000, };
static const NOR_EraseModeTypeT e_chip  = { .cmd = { .cmd = 0x60, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time=240000000, }, .block_size = 0, };

static const NOR_FlashCmdT      wen     = { .cmd = 0x06, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Write enable
static const NOR_FlashCmdT      wdis    = { .cmd = 0x04, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Write disable
static const NOR_FlashCmdT      resen   = { .cmd = 0x66, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Reset enable
static const NOR_FlashCmdT      reset   = { .cmd = 0x99, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 35 };      // Software reset
static const NOR_FlashCmdT      dpdstart= { .cmd = 0xb9, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 40 };      // Deep power down
static const NOR_FlashCmdT      dpdend  = { .cmd = 0xab, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 35 };      // Leave Deap power down
static const NOR_FlashCmdT      pe_susp = { .cmd = 0x75, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 20 };      // Program or erase suspend
static const NOR_FlashCmdT      pe_resum= { .cmd = 0x7a, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 400 };     // Program or erase resume

static const NOR_FlashCmdT      r_st1   = { .cmd = 0x05, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 1, .exec_time = 0 };       // Read Status byte 1
static const NOR_FlashCmdT      r_cr1   = { .cmd = 0x15, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 2, .exec_time = 0 };       // Read Config Register ( 2 Bytes )
static const NOR_FlashCmdT      r_sc    = { .cmd = 0x2b, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 1, .exec_time = 0 };       // Read Security Register byte
static const NOR_FlashCmdT      w_st1   = { .cmd = 0x01, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size =-1, .exec_time = 0 };       // Write Status byte 
static const NOR_FlashCmdT      w_stcr  = { .cmd = 0x01, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size =-3, .exec_time = 0 };       // Write Status byte and CR word
static const NOR_FlashCmdT      r_ems   = { .cmd = 0x90, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 2, .exec_time = 0 };       // Read Device ID: Manufaturers DeviceID, addr includes 2 dummy b.
static const NOR_FlashCmdT      r_jid   = { .cmd = 0x9F, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 3, .exec_time = 0 };       // Read JEDEC-ID: ManufacturerID, MemTypeID, MemDensityID


static const NOR_FlashQueryT    r_wip   = { .exec = &r_st1, .access_mask = WIP_MASK, .access_shift = WIP_SHFT, };    // How to access WIP flag
static const NOR_FlashQueryT    r_wel   = { .exec = &r_st1, .access_mask = WEL_MASK, .access_shift = WEL_SHFT, };    // How to access WEL flag
static const NOR_FlashQueryT    r_qe    = { .exec = &r_st1, .access_mask = QE_MASK,  .access_shift = QE_SHFT,  };    // How to access QE flag
static const NOR_FlashQueryT    r_psb   = { .exec = &r_sc,  .access_mask = PSB_MASK, .access_shift = PSB_SHFT, };    // How to access program suspend bit
static const NOR_FlashQueryT    r_esb   = { .exec = &r_sc,  .access_mask = ESB_MASK, .access_shift = ESB_SHFT, };    // How to access erase suspend bit
static const NOR_FlashQueryT    r_hp    = { .exec = &r_cr1, .access_mask = HP_MASK,  .access_shift = HP_SHFT,  };    // How to access High Performance bit

static const NOR_FlashOpT       op_s_qe =  { .qry = &r_qe, .op = XSPI_BITOP_SET, };                                  // Operation to set quad enable bit          
static const NOR_FlashOpT       op_r_qe =  { .qry = &r_qe, .op = XSPI_BITOP_RESET, };                                // Operation to reset quad enable bit          
static const NOR_FlashOpT       op_x_qe =  { .qry = &r_qe, .op = XSPI_BITOP_NONE, };                                 // Operation to read SR. no modification           
static const NOR_FlashOpT       op_s_hp =  { .qry = &r_hp, .op = XSPI_BITOP_SET, };                                  // Operation to set high performance
static const NOR_FlashOpT       op_r_hp =  { .qry = &r_hp, .op = XSPI_BITOP_RESET, };                                // Operation to reset high performance

static const NOR_FlashSetterT   ss_qe    =  { .ops = { &op_s_qe, NULL, },     .set_cmd = &w_st1 };                        // Set Quad Enable
static const NOR_FlashSetterT   sr_qe    =  { .ops = { &op_r_qe, NULL, },     .set_cmd = &w_st1 };                        // Reset Quad Enable
static const NOR_FlashSetterT   ss_hp    =  { .ops = { &op_x_qe, &op_s_hp, }, .set_cmd = &w_stcr };                       // Set High performance bit
static const NOR_FlashSetterT   sr_hp    =  { .ops = { &op_x_qe, &op_r_hp, }, .set_cmd = &w_stcr };                       // Reset High performance bit



const NOR_FlashT flash_interface = {
    .global = {
        .flags         = FCOMMON_HAS_INIT_SPEED | FCOMMON_HAS_24B_ADDR,
        .operating_frq = 1E6 * OP_FREQ,   
        .size          = 23,    /* 128 MBit, 8MByte = 2^23 */ 
        },
    .read = {
        .r_111    = &r_111,
        .rf_111   = &rf_111,
        .rf_112   = &rf_112,
        .rf_114   = &rf_114,
        .rf_122   = &rf_122,
        .rf_144   = &rf_144,
        .rf_144c  = &rf_144,
        .rf_444   = NULL,
        .rf_444c  = NULL,
        .continuous_rd_on =  0xaa,
        .continuous_rd_off = 0xa5,
        },
    .write = {
        .p_111    = &p_111,
        .p_114    = NULL,
        .p_144    = &p_144,
        .p_444    = NULL,
        .page_size= 256,
        },
    .erase = {
        .p_erase = { &e_01, &e_02, &e_03, NULL },
        .f_erase = &e_chip,
        },
    .cmd = {
        .wen      = &wen,  
        .wdis     = &wdis,
        .dpen     = &dpdstart,
        .dpdis    = &dpdend,
        .hpen     = NULL,
        .hpdis    = NULL,
        .resen    = &resen,
        .reset    = &reset,
        .pe_susp  = &pe_susp,  
        .pe_resume= &pe_resum,
        .r_ems    = &r_ems,
        .r_jid    = &r_jid,
        .r_uid    = NULL,
        .r_jedec  = NULL,
        .status_rd = { &r_st1, &r_cr1,  &r_sc, NULL, },
        .status_wr = { &w_st1, &w_stcr, NULL,  NULL, },
        },
    .query = {
        .wip     = &r_wip,   
        .wel     = &r_wel,
        .qe      = &r_qe,
        .drv     = NULL,
        .psb     = &r_psb,
        .esb     = &r_esb,
        .mode    = NULL,
        },   
     .setter = {
        .qe      = {&ss_qe, &sr_qe},
        .drv     = {NULL, NULL },
        .mode    = {&ss_hp, &sr_hp},
        },     
};


/* forward declarations -------------------------------------------------------*/
static uint16_t MX25_GetType(XSpiHandleT *myHandle);

/* Exported functions -------------------------------------------------------*/

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
 * Dump the status of Status and Configuration register
 *****************************************************************************/
void XSpecific_DumpStatusInternal(XSpiHandleT *myHandle)
{
    uint8_t sr, scur;
    uint8_t cr[2];
    char txtbuf[20];
    bool ret;

    /* get the list of status read commands */
    const NOR_FlashCmdT * const *status_rd = myHandle->interface->cmd.status_rd; 

    /* get plain text of QSPI chip */
    XSpecific_GetChipTypeText(myHandle->id,txtbuf,20);

    /* Get Deice ID from id bytes */
    uint16_t devID = MX25_GetType(myHandle);

    /* print chip name */
    XSpecific_GetChipTypeText(myHandle->id,txtbuf,20);

    /* Read and dump Status register */
    ret = XHelper_CmdArgRead ( myHandle, status_rd[0], 0, &sr, 1 );
    printf("%s Status register\n",  txtbuf);
    if ( ret ) {
        printf("   SR Write protect = %d\n", sr & (SRWP_MASK << SRWP_SHFT) ? 1 : 0 );
        printf("   Quad Enable      = %d\n", sr & (QE_MASK << QE_SHFT) ? 1 : 0 );
        printf("   Block protect    = 0x%1x\n", (sr >> BP_SHFT ) & BP_MASK);
        printf("   Write Enable     = %d\n", sr & (WEL_MASK << WEL_SHFT)  ? 1 : 0 );
        printf("   Write in progess = %d\n", sr & (WIP_MASK << WIP_SHFT)  ? 1 : 0 );
    } else {
        puts("   Cannot read SR");
    }

    /* read and dump configuration register */
    printf("%s Configuration register\n", txtbuf );
    ret = XHelper_CmdArgRead ( myHandle, status_rd[1], 0, cr, ( devID == MX25R6435F_DEVID ? 2 : 1 ) );
    if ( ret ) {
        printf("   Bottom area prot = %d\n", cr[0] & ( TB_MASK << TB_SHFT ) ? 1 : 0 );
        switch ( devID ) {
        case MX25R6435F_DEVID:
            printf("   2/4Rd dummy cyc. = %d\n", cr[0] & ( 1 << 6 ) ? 1 : 0 );
            printf("   High perf. mode  = %d\n", cr[1] & ( HP_MASK << (HP_SHFT-8) ) ? 1 : 0 );
            break;
        case MX25L12835F_DEVID:
            printf("   Dummy cycles.    = %d\n", cr[0] >> 6);
            printf("   Outp.Drv.Str.    = %d\n", cr[0] & 0b111 );
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

    /*read and dump SCUR */
    ret = XHelper_CmdArgRead ( myHandle, status_rd[2], 0, &scur, 1 );
    printf("%s Security register\n", txtbuf );
    if ( ret ) {
        printf("   last Erase fail. = %d\n", scur & (EFAIL_MASK << EFAIL_SHFT) ? 1 : 0 );
        printf("   last Prog failed = %d\n", scur & (PFAIL_MASK << PFAIL_SHFT) ? 1 : 0 );
        printf("   Erase suspended  = %d\n", scur & (ESB_MASK << ESB_SHFT) ? 1 : 0 );
        printf("   Prog suspended   = %d\n", scur & (ESB_MASK << ESB_SHFT) ? 1 : 0 );
        printf("   LDSO bit         = %d\n", scur & 0b10  ? 1 : 0 );
        printf("   factury OTP bit  = %d\n", scur & 0b01  ? 1 : 0 );
    } else {
        puts("   Cannot read SCUR");
    }

}
#if 0
/******************************************************************************
 * Read Operation command
 *****************************************************************************/
bool XSpecific_ReadSFDP(XSpiHandleT *myHandle)
{
    // SFDP header constants (for the MX25R6435F)
    #define SFDP_HEADER_SIZE 8

    // Buffer to store SFDP data
    #define SFDP_BUFFER_SIZE 256
    
    uint8_t sfdp_buffer[SFDP_BUFFER_SIZE];
    uint8_t sfdp_header[SFDP_HEADER_SIZE];


    /* read SFDP date */
    if ( XHelper_CmdArgRead ( myHandle, SFDP_READ_CMD, 0, 3, MX25R6435F_DUMMY_CYCLES_READ_SFDP, sfdp_header, SFDP_HEADER_SIZE) != HAL_OK ) {

        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpi_ReadSFDP - Error: Command not send");
        #endif
        return false;
    }

    return true;
}
#endif


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
        /* Automatic poling mode must be aborted in case of timeout */
        hxspi->State = HAL_QSPI_STATE_BUSY;
        HAL_QSPI_Abort(hxspi);
        return false;
      }
  #endif
  return true;
}


#endif /* #if USE_XSPI_MX25 > 0 */

