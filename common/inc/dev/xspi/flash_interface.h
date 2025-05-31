/*******************************************************************************
 * @file    flash_interface.h
 * @author  Rainer
 * @brief   structure to contain all vendor and type specific data 
 *          and access functions of a nor-flash
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_INTERFACE_H
#define __FLASH_INTERFACE_H

#include "config/config.h"

/* Different modes of QUADSPI operation */
#define XSPI_MODE_POLL      0       // Execute function in Polling/active wait mode
#define XSPI_MODE_IRQ       1       // Execute function in Interrupt/NoWalt mode
#define XSPI_MODE_DMA       2       // Execute function in DMA/NoWalt mode, makes only sense for read/write operations

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum {
    FCOMMON_HAS_INIT_SPEED  = 0,          // Has to be initialized with a lower speed
    FCOMMON_HAS_24B_ADDR    = 1,          // Has 24 bit addressing
    FCOMMON_HAS_32B_ADDR    = 2,          // Has 32 bit addressing  
    FCOMMON_HAS_SERIAL      = 3,          // Device has a serial number
} Flash_Common_Flags;

typedef enum {
    XSPI_MODE_111           = 0,        // Mode 1-1-1
    XSPI_MODE_112           = 1,        // Mode 1-1-2
    XSPI_MODE_114           = 2,        // Mode 1-1-4
    XSPI_MODE_122           = 3,        // Mode 1-2-2
    XSPI_MODE_144           = 4,        // Mode 1-4-4
    XSPI_MODE_444           = 5,        // Mode 4-4-4
} XSPI_ModeT;

/* number of cmd-, address- and data lines for the above XSPI modes */
#define CMD_LINES           { 1, 1, 1, 1, 1, 4 }
#define ADDR_LINES          { 1, 1, 1, 2, 4, 4 }
#define DATA_LINES          { 1, 2, 4, 2, 4, 4 }



typedef struct NOR_FlashGlobalType {
    uint32_t flags;                     // capabilities in form of common flags
    uint32_t size;                      // flash size in bytes, specified as 2^size
    uint32_t init_frq;                  // lower operating frq at initialization
    uint32_t operating_frq;             // operating frq in MHz
} NOR_FlashGlobalT; 

/* 
 * A flash command consists of command byte, optional argument bytes, optional answer bytes 
 * and an optional maximum execution time 
 */
typedef struct NOR_FlashCmdType {       // description of other commands, ie no read, pgm or erase
    uint8_t cmd;                        // command byte
    XSPI_ModeT rw_mode;                 // QSPI mode for read or write commands
    uint8_t cmdaddr_size;                // size of cmd addrss in bytes
    uint8_t dummy_cycles;               // number of dummy cycles at specified frq
    int8_t arg_size;                    // size of argument in bytes ( negative = write bytes, 0 = no answer, > 0 read bytes )
    uint32_t exec_time;                 // Maximum execution time in us, 0 = no restricitions
} NOR_FlashCmdT; 

/* 
 * A (status) query consists of a command, which should return a value, a mandatory
 * mask to filter relevant bits and a mandatory shift to return the bits right shifted
 */
typedef struct NOR_FlashQueryType {     
    const NOR_FlashCmdT* exec;
    uint32_t access_mask;
    uint8_t access_shift;
} NOR_FlashQueryT;

typedef enum NOR_FlashBitOpType { 
    XSPI_BITOP_NONE   = 0,                      // No Operation
    XSPI_BITOP_SET    = 1,                      // Set queried bits 
    XSPI_BITOP_RESET  = 2,                      // Reset queried bits 
    XSPI_BITOP_SETNUM = 3,                      // Set queried to a specified value
} NOR_FlashBitOpT;
/*
 * An Operation consist of a qery, an optional reset mask to reset bits
 * and an optional set mask to set bits. 
 * although set and reset masks are 32bit, 
 * the result length of the query defines the length of the result
 */
typedef struct NOR_FlashOpType {
    const NOR_FlashQueryT *qry;               // associated query
    const NOR_FlashBitOpT op;                 // operation to perform on queried bits
} NOR_FlashOpT;

#define FLASH_MAX_OPS      2
/*
 * A Setter consists of a set op operations, which in sum define the length of
 * write argument and a setter, that processes the operations result
 * Sum of all query/operation bytes and setter expected bytes
 * have to be identical
 */
typedef struct NOR_FlashSetterType {
    const NOR_FlashOpT *ops[FLASH_MAX_OPS];     // associated queries and operations
    const NOR_FlashCmdT* set_cmd;             // write/set command      
} NOR_FlashSetterT;

/* all characteristics of a read operation */
typedef struct NOR_RWModeType{   
    NOR_FlashCmdT cmd;                  // definition of read or write command
    uint8_t qe;                         // flag for "quad mode must be enabled before"
    uint8_t mode_bytes;                 // number of mode bytes to enable continuous read
    uint8_t max_frq;                    // max allowed Frq in MHz
    uint16_t pgm_time;                  // max page program time in us from datasheet 
} NOR_RWModeTypeT; 

/* all characteristics of an erase operation */
typedef struct NOR_EraseModeType {
    NOR_FlashCmdT cmd;                  // definition of erase command
    uint32_t block_size;                // Size of erase block for this cmd, 0 = whole chip
} NOR_EraseModeTypeT;

/* all possible read operations, NULL if not possible */
typedef struct NOR_FlashReadType {
     const NOR_RWModeTypeT *r_111;          // 1-1-1 Read
     const NOR_RWModeTypeT *rf_111;         // 1-1-1 Fast Read
     const NOR_RWModeTypeT *rf_112;         // 1-1-2 Read
     const NOR_RWModeTypeT *rf_122;         // 1-2-2 Read
     const NOR_RWModeTypeT *rf_114;         // 1-1-4 Read
     const NOR_RWModeTypeT *rf_144;         // 1-4-4 Read
     const NOR_RWModeTypeT *rf_144c;        // 1-4-4 continuous Read
     const NOR_RWModeTypeT *rf_444;         // 4-4-4 Read
     const NOR_RWModeTypeT *rf_444c;        // 4-4-4 continuous Read
     const uint8_t continuous_rd_on;          // mode byte for "continuous read on"
     const uint8_t continuous_rd_off;         // mode byte for "continuous read off"
} NOR_FlashReadT;

/* all possible write operations, NULL if not possible */
typedef struct NOR_FlashWriteType {
    const NOR_RWModeTypeT  *p_111;           // 1-1-1 page write
    const NOR_RWModeTypeT  *p_114;           // 1-1-4 page write
    const NOR_RWModeTypeT  *p_144;           // 1-4-4 page write
    const NOR_RWModeTypeT  *p_444;           // 4-4-4 page write
    uint16_t page_size;                       // Size of write buffer
} NOR_FlashWriteT;

#define FLASH_MAX_STATUS  4                          // We support up to 4 Status read and write commands

/* all possible configuration commands, NULL if not defined */
typedef struct NOR_FlashCmdListType {         
    const NOR_FlashCmdT *wen;                       // Write enable
    const NOR_FlashCmdT *wdis;                      // Write disable
    const NOR_FlashCmdT *dpen;                      // deep power down enable
    const NOR_FlashCmdT *dpdis;                     // deep power down disable
    const NOR_FlashCmdT *hpen;                      // high performance mode enable
    const NOR_FlashCmdT *hpdis;                     // high performance mode disable
    const NOR_FlashCmdT *resen;                     // reset enable
    const NOR_FlashCmdT *reset;                     // reset
    const NOR_FlashCmdT *pe_susp;                   // program or erase suspend
    const NOR_FlashCmdT *pe_resume;                 // program or erase resume
    const NOR_FlashCmdT *status_rd[FLASH_MAX_STATUS];// all status read commands
    const NOR_FlashCmdT *status_wr[FLASH_MAX_STATUS];// all status write commands
    const NOR_FlashCmdT *r_ems;                     // Read Manufacturer and Device ID
    const NOR_FlashCmdT *r_jid;                     // Read JEDEC ID
    const NOR_FlashCmdT *r_uid;                     // Read chips UID
    const NOR_FlashCmdT *r_jedec;                   // Read JEDEC data
} NOR_FlashCmdListT;

/* all possible queries of status bits etc, NULL if not defined */
typedef struct NOR_FlashQryListType {         
    const NOR_FlashQueryT *wip;                     // How to query WIP flag
    const NOR_FlashQueryT *wel;                     // How to query WEL flag
    const NOR_FlashQueryT *qe;                      // How to query QE flag
    const NOR_FlashQueryT *drv;                     // How to query Driver strength
    const NOR_FlashQueryT *psb;                     // How to query program suspend bit
    const NOR_FlashQueryT *esb;                     // How to query erase suspend bit
    const NOR_FlashQueryT *mode;                    // How to query operation modes
} NOR_FlashQryListT;

/* all possible setters of mode bits etc, NULL if not defined */
typedef struct NOR_FlashSetListType {         
    const NOR_FlashSetterT *qe[2];                     // How to set/reset quad enable flag
    const NOR_FlashSetterT *drv[2];                    // How to set/reset driver strength
    const NOR_FlashSetterT *mode[2];                   // How to set any operation mode
} NOR_FlashSetListT;

#define MAX_ERASE  4                                // We support up to 4 partial erase commands

typedef struct NOR_FlashEraseType {
    const  NOR_EraseModeTypeT *p_erase[MAX_ERASE];  // list or partial erase commands
    const  NOR_EraseModeTypeT *f_erase;             // full erase
} NOR_FlashEraseT;

typedef bool (*NOR_FlashInitFn)(void);

typedef struct NOR_FlashType{   
    const NOR_FlashGlobalT  global;            // global data
    const NOR_FlashReadT    read;              // read data
    const NOR_FlashWriteT   write;             // write data
    const NOR_FlashEraseT   erase;             // erase data
    const NOR_FlashCmdListT cmd;               // list of all other commands
    const NOR_FlashQryListT query;             // list of status bit queries
    const NOR_FlashSetListT setter;            // list of status bit Setters
    const NOR_FlashInitFn   init;              // chip specific init function
} NOR_FlashT;


/*----------------------------------------------------------------------------------------------
 * chip specific functions, which also have to be defined individually for every device (family)
 *--------------------------------------------------------------------------------------------*/



#ifdef __cplusplus
 }
#endif

#endif // __FLASH_INTERFACE_H