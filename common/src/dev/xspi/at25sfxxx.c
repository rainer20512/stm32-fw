/*******************************************************************************
 * @file    at25sfxxx.c
 * @author  Rainer 
 * @brief   flash SPI/QSPI definitions for 
 *          Renesas AT25SFxxx Quad SPI flash memory
 *          works for 
 *          - AT25SF128A
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"

#if USE_XSPI_AT25SF > 0 && ( USE_QSPI > 0 ||  USE_OSPI > 0 )

#include <stddef.h>
#include "dev/xspi/flash_interface.h"

/* WIP flag is bit 0 in status 1 */
#define WIP_MASK 0b1
#define WIP_SHFT 0

/* Write Enable flag is bit 1 in status 1 */
#define WEL_MASK 0b1
#define WEL_SHFT 1

/* Driver Strngth is bit 5/6 in status 3 */
#define DRV_MASK     0b11
#define DRV_SHFT     5
#define DRV_STRENGTH 0           // 0=100%, 1=75%, 2=50%, 3=25% 

/* Quad Enable flag is bit 1 in status 2*/
#define QE_MASK 0b1
#define QE_SHFT 1

/* Program suspend flag is bit 2 in status 2 (SUS2) */
#define PSB_MASK 0b1
#define PSB_SHFT 2

/* Erase suspend flag is bit 7 in status 2 (SUS1) */
#define ESB_MASK 0b1
#define ESB_SHFT 7


/* All frquencies in MhZ */

static const uint8_t OP_FREQ = 10;      

static const uint8_t OP_FC1  = 133;     /* Frequency FC1 from datasheet */
static const uint8_t OP_FC2  = 70;      /* Frequency FC2 from datasheet */
static const uint8_t OP_FC3  = 133;     /* Frequency FC3 from datasheet */
static const uint8_t OP_FC4  = 108;     /* Frequency FC4 from datasheet */

/* when Vcc is guaranteed between 3.0 and 3.6V, max frq is FC3 instead of FC4 */
static const NOR_RWModeTypeT  r_111     = { .cmd = {.cmd = 0x03, .rw_mode = XSPI_MODE_111, .dummy_cycles = 0, }, .mode_bytes = 0, .max_frq = OP_FC2, };
static const NOR_RWModeTypeT  rf_111    = { .cmd = {.cmd = 0x0b, .rw_mode = XSPI_MODE_111, .dummy_cycles = 1, }, .mode_bytes = 0, .max_frq = OP_FC3, };
static const NOR_RWModeTypeT  rf_112    = { .cmd = {.cmd = 0x3b, .rw_mode = XSPI_MODE_112, .dummy_cycles = 1, }, .mode_bytes = 0, .max_frq = OP_FC3, };
static const NOR_RWModeTypeT  rf_114    = { .cmd = {.cmd = 0x6b, .rw_mode = XSPI_MODE_114, .dummy_cycles = 1, }, .mode_bytes = 0, .max_frq = OP_FC1, };
static const NOR_RWModeTypeT  rf_122    = { .cmd = {.cmd = 0xbb, .rw_mode = XSPI_MODE_122, .dummy_cycles = 0, }, .mode_bytes = 1, .max_frq = OP_FC3, };
static const NOR_RWModeTypeT  rf_144    = { .cmd = {.cmd = 0xeb, .rw_mode = XSPI_MODE_144, .dummy_cycles = 2, }, .mode_bytes = 1, .max_frq = OP_FC3, };

static const NOR_RWModeTypeT  p_111     = { .cmd = {.cmd = 0x02, .rw_mode = XSPI_MODE_111, .dummy_cycles = 0, }, .pgm_time = 2400 };
static const NOR_RWModeTypeT  p_114     = { .cmd = {.cmd = 0x32, .rw_mode = XSPI_MODE_114, .dummy_cycles = 0, }, .pgm_time = 2400 };

static const NOR_EraseModeTypeT e_01    = { .cmd = { .cmd = 0x20, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=300000, },    .block_size = 0x1000, };
static const NOR_EraseModeTypeT e_02    = { .cmd = { .cmd = 0x52, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=1600000, },   .block_size = 0x8000, };
static const NOR_EraseModeTypeT e_03    = { .cmd = { .cmd = 0xd8, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 0, .exec_time=2000000, },   .block_size = 0x10000, };
static const NOR_EraseModeTypeT e_chip  = { .cmd = { .cmd = 0x60, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time=120000000, }, .block_size = 0, };

static const NOR_FlashCmdT      wen     = { .cmd = 0x06, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Write enable
static const NOR_FlashCmdT      wdis    = { .cmd = 0x04, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Write disable
static const NOR_FlashCmdT      resen   = { .cmd = 0x66, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 0 };       // Reset enable
static const NOR_FlashCmdT      reset   = { .cmd = 0x99, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 30000 };   // Software reset
static const NOR_FlashCmdT      dpdstart= { .cmd = 0xb9, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 20 };      // Deep power down
static const NOR_FlashCmdT      dpdend  = { .cmd = 0xab, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 20 };      // Leave Deap power down
static const NOR_FlashCmdT      pe_susp = { .cmd = 0x75, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 20 };      // Program or erase suspend
static const NOR_FlashCmdT      pe_resum= { .cmd = 0x7a, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 0, .exec_time = 400 };     // Program or erase resume

static const NOR_FlashCmdT      r_st1   = { .cmd = 0x05, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 1, .exec_time = 0 };       // Read Status byte 1
static const NOR_FlashCmdT      r_st2   = { .cmd = 0x35, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 1, .exec_time = 0 };       // Read Status byte 2
static const NOR_FlashCmdT      r_st3   = { .cmd = 0x15, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 1, .exec_time = 0 };       // Read Status byte 3
static const NOR_FlashCmdT      w_st1   = { .cmd = 0x01, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size =-1, .exec_time = 0 };       // Write Status byte 1
static const NOR_FlashCmdT      w_st2   = { .cmd = 0x31, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size =-1, .exec_time = 0 };       // Write Status byte 2
static const NOR_FlashCmdT      w_st3   = { .cmd = 0x11, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size =-1, .exec_time = 0 };       // Write Status byte 3
static const NOR_FlashCmdT      r_ems   = { .cmd = 0x90, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 3, .dummy_cycles = 0, .arg_size = 2, .exec_time = 0 };       // Read Device ID: Manufaturers DeviceID
static const NOR_FlashCmdT      r_jid   = { .cmd = 0x9F, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 0, .arg_size = 3, .exec_time = 0 };       // Read JEDEC-ID: ManufacturerID, MemTypeID, MemDensityID
static const NOR_FlashCmdT      r_uid   = { .cmd = 0x4b, .rw_mode = XSPI_MODE_111, .cmdaddr_size = 0, .dummy_cycles = 4, .arg_size = 8, .exec_time = 0 };       // Read UID


static const NOR_FlashQueryT    r_wip   = { .exec = &r_st1, .access_mask = WIP_MASK, .access_shift = WIP_SHFT, };    // How to query WIP flag
static const NOR_FlashQueryT    r_wel   = { .exec = &r_st1, .access_mask = WEL_MASK, .access_shift = WEL_SHFT, };    // How to query WEL flag
static const NOR_FlashQueryT    r_qe    = { .exec = &r_st2, .access_mask = QE_MASK,  .access_shift = QE_SHFT,  };    // How to query QE flag
static const NOR_FlashQueryT    r_drv   = { .exec = &r_st3, .access_mask = DRV_MASK, .access_shift = DRV_SHFT, };    // How to query Driver strength
static const NOR_FlashQueryT    r_psb   = { .exec = &r_st2, .access_mask = PSB_MASK, .access_shift = PSB_SHFT, };    // How to query program suspend bit
static const NOR_FlashQueryT    r_esb   = { .exec = &r_st2, .access_mask = ESB_MASK, .access_shift = ESB_SHFT, };    // How to query erase suspend bit

static const NOR_FlashSetterT   s_qe     = { .qry = &r_qe,  &w_st2, };                                               // Set quad enable bit        
static const NOR_FlashSetterT   s_drv    = { .qry = &r_drv, &w_st3, };                                               // Set driver strength

const NOR_FlashT flash_interface = {
    .global = {
        .flags         = FCOMMON_HAS_24B_ADDR,
        .operating_frq = 1E6 * OP_FREQ,   
        .size          = 24,    /* 128 MBit, 16MByte = 2^24 */ 
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
        .p_114     = &p_114,
        .p_144    = NULL,
        .p_444      = NULL,
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
        .r_uid    = &r_uid,
        .r_jedec  = NULL,
        .status_rd = { &r_st1, &r_st2, &r_st3, NULL, },
        .status_wr = { &w_st1, &w_st2, &w_st3, NULL, },
        },
    .query = {
        .wip     = &r_wip,   
        .wel     = &r_wel,
        .qe      = &r_qe,
        .drv     = &r_drv,
        .psb     = &r_psb,
        .esb     = &r_esb,
        .mode    = NULL,
        },   
     .setter = {
        .qe      = &s_qe,
        .drv     = &s_drv,
        .mode    = NULL,
        },     
};

/* Set >0 to generate DEBUG output */
#define DEBUG_XSPI              2

#include "dev/xspi_dev.h"
#include "dev/xspi/xspi_helper.h"
#include "dev/xspi/xspi_lowlevel.h"


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



/* Private variables ---------------------------------------------------------*/



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
            case 0x8901: txt = "AT25SF128A"; break;
            default:     txt = "Unknown Chip";
        }
        
        strncpy(retbuf, txt,bufsize);
        return retbuf;
    }

    /******************************************************************************
     * Dump the status of Status and Configuration register
     *****************************************************************************/
    void XSpecific_DumpStatusInternal(XSpiHandleT *myHandle)
    {
      /* Status bit definitions DWORD based here */
      #define DRV__MASK  0b11
      #define DRV__SHFT  21
      #define ESUS__MASK 0b1
      #define ESUS__SHFT 15
      #define CMP__SHFT  14
      #define CMP__MASK  0b1
      #define LB__SHFT   11
      #define LB__MASK   0b111
      #define WSUS__SHFT 10
      #define WSUS__MASK 0b1
      #define QE__SHFT   9
      #define QE__MASK   0b1
      #define SRP__SHFT  7
      #define SRP__MASK  0b11
      #define BP__SHFT   2
      #define BP__MASK   0x1f
      #define WEL__SHFT  1
      #define WEL__MASK  0b1
      #define WIP__SHFT  0
      #define WIP__MASK  0b1
  
      uint32_t sr;
      uint32_t temp;
      uint8_t work;
      char txtbuf[20];
      bool ret;

      /* get the list of status read commands */
      const NOR_FlashCmdT * const *status_rd = myHandle->interface->cmd.status_rd; 

      /* get plain text of QSPI chip */
      XSpecific_GetChipTypeText(myHandle->id,txtbuf,20);

#if 0
      ret = XSpiLL_ResetMemory(myHandle);
      /* Status register is read byte wise, but interpreted as DWORD */
      work = 0;
      ret = XHelper_CmdArgRead ( myHandle, myHandle->interface->cmd.status_wr[1], 0, &work, -1 );

      work =0x20;
      ret = XHelper_CmdArgRead ( myHandle, myHandle->interface->cmd.status_wr[2], 0, &work, -1 );#endif
#endif

      /* Status register is read byte wise, but interpreted as DWORD */
      ret = XHelper_CmdArgRead ( myHandle, status_rd[2], 0, &work, 1 );
      sr = work;
      ret = XHelper_CmdArgRead ( myHandle, status_rd[1], 0, &work, 1 );
      sr = sr <<8 | work;
      ret = XHelper_CmdArgRead ( myHandle, status_rd[0], 0, &work, 1 );
      sr = sr << 8 | work;

      printf("%s Status register \n",  txtbuf);
      if ( ret ) {
        #define MASKOUT(a) (sr >> a##__SHFT ) & a##__MASK;
    
        temp = MASKOUT(DRV);
        printf("   DRV Strength     = %d - %d%%\n",temp, (4-temp)*25 );
        temp = MASKOUT(ESUS);
        printf("   Erase Suspend    = %d\n", temp ? 1 : 0 );
        temp = MASKOUT(CMP);
        printf("   CMP              = %d\n", temp ? 1 : 0 );
        temp = MASKOUT(LB);
        printf("   LB               = 0x%x\n", temp);
        temp = MASKOUT(WSUS);
        printf("   Write Suspend    = %d\n", temp ? 1 : 0 );
        temp = MASKOUT(QE);
        printf("   Quad Enable      = %d\n", temp ? 1 : 0 );
        temp = MASKOUT(SRP);
        printf("   SRP              = 0x%x\n", temp);
        temp = MASKOUT(BP);
        printf("   Block protect    = 0x%x\n", temp);
        temp = MASKOUT(WEL);
        printf("   Write Enable     = %d\n", temp ? 1 : 0 );
        temp = MASKOUT(WIP);
        printf("   Write in progess = %d\n", temp ? 1 : 0 );
      } else {
        puts("   Cannot read SR");
      }

    }
#endif

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




#if 0
/******************************************************************************
 * Enable Write and setup write Operation command
 *****************************************************************************/
bool XSpecific_SetupWrite(XSpiHandleT *myHandle, uint32_t Addr, uint32_t Size)
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
    if ( XSpiLL_WriteEnable(myHandle) ) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpecific_SetupWrite - Error: Write enable failed");
        #endif
      return false;
    }

    /* Configure the command */
        if (HAL_XSPI_Command(&myHandle->hxspi, &sCommand, XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        #if DEBUG_MODE > 0 && DEBUG_XSPI > 0
            DEBUG_PUTS("XSpecific_SetupWrite - Error: Command not send");
        #endif
        return false;
    }

    return true;
}


#endif




#endif /* #if USE_XSPI_AT25SF > 0 > 0 */
