/*
 ******************************************************************************
 * @file    sdmmc_dev.c
 * @author  Rainer
 * @brief  SDMMC hardware wrapped into HW_Device
 *
 *****************************************************************************/

/** @addtogroup SDMMC
  * @{
   
*/
#include "config/config.h"

#if USE_SDMMC > 0 

/* Debug ------------------------------------------------------------------------*/
#define DSBUG_SDMMC          1

#include "config/devices_config.h"
#include "config/sdmmc_config.h"
#include "dev/sdmmc_dev.h"
#include "system/hw_util.h"

#include "error.h"
#include "task/minitask.h"

#include "debug_helper.h"


/* Private typedef --------------------------------------------------------------*/

typedef struct {
    SdmmcHandleT              *mySdmmcHandle;            /* my associated handle */
} Sdmmc_AdditionalDataType;

/* Private or driver functions ------------------------------------------------------*/
static  SdmmcHandleT * Sdmmc_GetMyHandle(const HW_DeviceType *self)
{
    return ((Sdmmc_AdditionalDataType *)(self->devData))->mySdmmcHandle;
}



/******************************************************************************
 * Clear the entire CanHandleT structure
 *****************************************************************************/
static void SdmmcResetMyHandle ( SdmmcHandleT *handle ) 
{
    memset(handle, 0, sizeof(SdmmcHandleT) );
}


/**************************************************************************************
 * Some Devices support different clock sources for FSMC. On configurable FMC Clock   *
 * sources, we assume that FMC Clock source is HCLK. In any case make sure, that      *   
 * Fmc_SetClockSource and Fmc_GetClockSpeed() will match                              *
 *************************************************************************************/
#if defined(STM32L4_FAMILY)
    /* STM32L4xx has no clock mux for FMC device */
    #define Fmc_SetClockSource(a)           (true)
    #define Fmc_GetClockSpeed()             HAL_RCC_GetHCLKFreq()
#elif defined(STM32H7_FAMILY)
    static bool Sdmmc_SetClockSource(const void *hw)
    {
      UNUSED(hw);
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* FSMC has to be operated with HCLK. Routines, which will set      */
      /* FMC timing constants, will call Fmc_GetClockSpeed() to determine */
      /* the current clock speed                                          */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC; 
      PeriphClkInit.FmcClockSelection    = RCC_SDMMCCLKSOURCE_PLL;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for SDMMC");
        return false;
      }

      return true;
    }
    #define Fmc_GetClockSpeed()             HAL_RCC_GetHCLKFreq()
#else 
    #error "No FMC clock assignment defined"
#endif


#if DEBUG_MODE >  0

    const char * const memtype_txt[]={"SRAM","NOR", "NAND"};
    static const char* fmc_get_memtype_txt(uint32_t sel )
    {
      if ( sel < sizeof(memtype_txt)/sizeof(char *) ) 
        return memtype_txt[sel];
      else
        return "Illegal";
    }

    const char * const fmcctl_txt[] = FMC_CTL_STR;
    static const char* fmc_get_ctl_txt(uint32_t sel )
    {
      if ( sel < sizeof(fmcctl_txt)/sizeof(char *) ) 
        return fmcctl_txt[sel];
      else
        return "Illegal";
    }

    static void FmcPrintCtlText( uint32_t bits )
    {
        uint32_t mask = 1;
        uint32_t idx = 0;
        while ( idx < FMC_CTL_MAX ) {
            if ( mask & bits )  DEBUG_PRINTF("%s ", fmc_get_ctl_txt(idx));
            mask <<= 1;
            idx++;
        }
    }

    static void FmcPrintBitVector ( const char *prefix, uint32_t bits, uint32_t maxidx )
    {
        uint32_t mask;
        uint32_t idx;
        uint32_t highest = 0;   /* highest set bit */

        mask = 1;
        idx = 0;

        while ( bits ) {
            // find lowest set bit //
            while ( idx < maxidx && ( mask & bits ) == 0 ) {
                mask <<= 1;
                idx++;
            }

            if ( idx >= maxidx ) return;

            DEBUG_PRINTF("%s%02d...", prefix, idx );

            // find highest set bit //
            while ( idx < maxidx && ( mask & bits )  ) {
                highest = idx;
                bits &= ~mask;
                mask <<= 1;
                idx++;
            }
            DEBUG_PRINTF("%s%02d ", prefix, highest );
    
        }
    }


    /**************************************************************************************
     * Dump the FMC parameters  *
     *************************************************************************************/
    static void FMC_DumpOneSramGeometry(uint32_t idx, FmcDataT *curr)
    {


        /* Get Timing data */
        Fmc_SramTimingDataT *sram_timing = (Fmc_SramTimingDataT *)(Fmc_GetMyExtMemData(&HW_FMC, idx)->fmcTiming);
        
        DEBUG_PRINTF("FMC Bank %d:\n", idx);
        if ( curr->fmcIsUsed ) {
            DEBUG_PRINTF("   Type:  %s\n", fmc_get_memtype_txt(curr->fmcType));
            DEBUG_PRINTF("   Muxed: %s\n", curr->fmcIsMuxed ? "yes" : "no" );
            if ( curr->fmcIsMuxed ) {
                DEBUG_PRINTF("   Addr/Data: ");FmcPrintBitVector("DA",curr->fmcDataBits, FMC_D_MAX);DEBUG_PRINTF("\n");
            } else {
                DEBUG_PRINTF("   Data: ");FmcPrintBitVector("D",curr->fmcDataBits, FMC_D_MAX);DEBUG_PRINTF("\n");
            }
            DEBUG_PRINTF("   Addr: ");FmcPrintBitVector("A",curr->fmcAddrBits, FMC_A_MAX);DEBUG_PRINTF("\n");
            DEBUG_PRINTF("   Ctl:  ");FmcPrintCtlText(curr->fmcCtlBits);DEBUG_PRINTF("\n");
            
            /* capacity in kBytes and start address */
            uint32_t capa = (1 << sram_timing->AddrBits ) * sram_timing->DataBits/8/1024;
            uint32_t base = 0x60000000 + 0x4000000 * idx;
             
            DEBUG_PRINTF("Organisation:       2^%d x %d bit\n", sram_timing->AddrBits, sram_timing->DataBits );
            DEBUG_PRINTF("Resulting capacity: %dKB or %dMB\n", capa, capa/1024);
            DEBUG_PRINTF("Address Range     : 0x%08x ... 0x%08x\n", base, base+capa*1024-1);
            DEBUG_PRINTF("Timing parameters:\n");
            DEBUG_PRINTF("Address setup time: %dns\n",sram_timing->T_AddrSet);
            DEBUG_PRINTF("Data setup time:    %dns\n",sram_timing->T_DataSet);
            if ( curr->fmcIsMuxed ) 
                DEBUG_PRINTF("Address hold time  %dns\n",sram_timing->T_AddrHold);
        } else {
            DEBUG_PUTS("   unused");
        }
    }

    /**************************************************************************************
     * Dump the FMC parameters  for a SDRAM bank*
     *************************************************************************************/
    static void FMC_DumpOneSdramGeometry(uint32_t idx, FmcDataT *curr)
    {


        /* Get Timing data */
        Fmc_SdramTimingDataT *sdram_data = (Fmc_SdramTimingDataT *)(Fmc_GetMyExtMemData(&HW_FMC, idx)->fmcTiming);
        
        DEBUG_PRINTF("internal bank #%d", idx);
        if ( curr->fmcIsUsed || curr->fmcType != FMC_TYPE_SDRAM ) {
            DEBUG_PRINTF(" / SDRAM-Bank #%d\n", sdram_data->sdramBankNum);
            DEBUG_PRINTF("   %2d row bits\n", sdram_data->RowBits);
            DEBUG_PRINTF("   %2d col bits\n", sdram_data->ColBits);
            DEBUG_PRINTF("    2 bank select bits\n");
            DEBUG_PRINTF("   %2d data bits\n", sdram_data->DataBits);
            DEBUG_PRINTF("   Addr: ");FmcPrintBitVector("A",curr->fmcAddrBits, FMC_A_MAX);DEBUG_PRINTF("\n");
            DEBUG_PRINTF("   Data: ");FmcPrintBitVector("D",curr->fmcDataBits, FMC_D_MAX);DEBUG_PRINTF("\n");
            DEBUG_PRINTF("   Ctl:  ");FmcPrintCtlText(curr->fmcCtlBits);DEBUG_PRINTF("\n");
            
            /* capacity in kBytes and start address */
            uint32_t capa = (1 << (sdram_data->RowBits+sdram_data->ColBits+2)) * sdram_data->DataBits/8/1024;
            uint32_t base = 0xC0000000 + 0x10000000 * (sdram_data->sdramBankNum-1);
            DEBUG_PRINTF("Resulting capacity: %dKB or %dMB\n", capa, capa/1024);
            DEBUG_PRINTF("Address Range     : 0x%08X ... 0x%08X\n", base, base+capa*1024-1);
            /* SDRAM controller is initialized with a fixed clock prescaler of 2 */
            DEBUG_PRINTF("SDRAM Clk Freq.   : %dMHz\n", Fmc_GetClockSpeed()/2/1000000);
            DEBUG_PRINTF("Timing parameters:\n");
        } else {
            DEBUG_PUTS("   unused or type is not SDRAM");
        }
    }
#endif

void FMC_DumpGeometry(void)
{

    for ( uint32_t i=0; i < FMC_MAX_BLOCKS; i++) {
        #if DEBUG_MODE > 0 
            FMC_DumpOneSramGeometry(i, FmcHandle.fmcData+i);
        #endif
    }
    #if DEBUG_MODE > 0
        DEBUG_PRINTF("Resulting Data lines:"); FmcPrintBitVector("D",FmcHandle.allDataBits, FMC_D_MAX);DEBUG_PRINTF("\n");
        DEBUG_PRINTF("Resulting Addr lines:"); FmcPrintBitVector("A",FmcHandle.allAddrBits, FMC_A_MAX);DEBUG_PRINTF("\n");
        DEBUG_PRINTF("Resulting Ctl  lines:"); ;FmcPrintCtlText(FmcHandle.allCtlBits);DEBUG_PRINTF("\n");
    #endif
}


/*
 * Init or DeInit Clock / clocksource 
 */
static bool Fmc_ClockInit(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !Fmc_SetClockSource( self->devBase ) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock( (void*)self->devBase, bDoInit );
    return true;
}

/******************************************************************************
 * Set then HAL Timing parameters ( in multiples of fmc_ker_ck ) on base of the
 * passed "myTiming"-times (in nanoseconds ).
 * 0 will be returned, if timing data is not settable (due to limited bitwidth
 * of HAL (resp FMC) timing data fields.
 * The underlying FMC clock speed [Hz] will be returned on success
 *****************************************************************************/
static uint32_t Fmc_SetSramTiming( FMC_NORSRAM_TimingTypeDef *halTiming, const Fmc_SramTimingDataT *myTiming )
{
    #define MIN_ADDSET       0         /* min clk cycles for address setup time */
    #define MAX_ADDSET      15         /* max clk cycles for address setup time */
    #define MIN_ADDHLD       1         /* min clk cycles for address hold  time */
    #define MAX_ADDHLD      15         /* max clk cycles for address hold  time */
    #define MIN_DATASET      1         /* min clk cycles for data setup time    */
    #define MAX_DATASET    255         /* max clk cycles for data setup time    */

    uint32_t work;
    uint32_t newClkHz = Fmc_GetClockSpeed();

    /* number of picosecs per FMC clock cycle */
    uint32_t ps_per_clk = 1000000 / (newClkHz/1000000);

    /* Address setup time */
    work = myTiming->T_AddrSet;
    work = ( work * 1000 + ps_per_clk - 1 )  / ps_per_clk;
    if ( work > MAX_ADDSET ) {
        DEBUG_PRINTF("Desired address setup time of %d clk cycles too big\n",work);
        work = MAX_ADDSET; // return false;
    }
    halTiming->AddressSetupTime = work;

    /* Address hold time */
    work = myTiming->T_AddrHold;
    work = ( work * 1000 + ps_per_clk - 1 )  / ps_per_clk;
    if ( work > MAX_ADDHLD ) {
        DEBUG_PRINTF("Desired address hold time of %d clk cycles too big\n",work);
        work = MAX_ADDHLD; // return false;
    }
    halTiming->AddressHoldTime = MAX(MIN_ADDHLD, work);

    /* data setup time */
    work = myTiming->T_DataSet;
    work = ( work * 1000 + ps_per_clk - 1 )  / ps_per_clk;
    if ( work > MAX_DATASET ) {
        DEBUG_PRINTF("Desired data setup time of %d clk cycles too big\n",work);
        work = MAX_DATASET; // return false;
    }
    halTiming->DataSetupTime = MAX(MIN_DATASET, work);

    /* Don#t care, but should be set */
    halTiming->BusTurnAroundDuration  = 0;
    halTiming->CLKDivision            = 2;
    halTiming->DataLatency            = 2;
   
    halTiming->AccessMode             = myTiming->accessMode;

    return newClkHz;
}

/******************************************************************************
 * Do the Initialization of an SRAM bank.
 * Parameters:
 *    fmc_idx     - FMC bank index [0 .. 3]
 *    sram_timing - neccessary timing and size parameters
 * initializes all associated IO pins and initializes the FMC bank. After return
 * the FMC bank is enabled/visible at its prefined memory address range
 * Returns
 *    true on success
 *    false in case of parameter or HAL layer initialization error
 *****************************************************************************/
bool Fmc_SRAM_Init(uint32_t fmc_idx, Fmc_SramTimingDataT *sram_timing )
{
    FMC_NORSRAM_InitTypeDef     *init;
    FMC_NORSRAM_TimingTypeDef   SRAM_Timing;
    
    FmcDataT *curr   = FmcHandle.fmcData+fmc_idx; 
    
    /* Check for selected FMC bank being unused */
    if ( curr->fmcIsUsed ) {
        DEBUG_PRINTF("Fmc Bank #%d already used\n",  fmc_idx );
        return false;
    }

    /* SRAM device configuration */  

//    uint32_t idx         = BANK_TO_IDX(Init->NSBank);
    curr->fmcRefClockSpeed = Fmc_SetSramTiming( &SRAM_Timing, sram_timing );
    if ( curr->fmcRefClockSpeed == 0  ) {
        DEBUG_PRINTF("Timing setup of Fmc Bank #%d failed\n",  fmc_idx );
        return false;
    }

    /* Check DataWidth to be 8 or 16 */
    if ( sram_timing->DataBits != 8 && sram_timing->DataBits != 16 ) {
        DEBUG_PRINTF("Data Width of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }

    curr->fmcIsUsed      = 1;
    curr->fmcType        = FMC_TYPE_SRAM;
    curr->fmcIsMuxed     = sram_timing->AD_Muxed;
    curr->fmcDataBits    = sram_timing->DataBits; 
    curr->fmcDataBits    = BITS_TO_MASK(curr->fmcDataBits);
    curr->fmcAddrBits    = BITS_TO_MASK(sram_timing->AddrBits);

    /* In case of multiplexed address/data, exclude the multiplexed address lines */
    if ( curr->fmcIsMuxed) curr->fmcAddrBits &= ~(curr->fmcDataBits);

    /* Enable NBL0, NBL1, NOE, NWE and selected enable-line */
    curr->fmcCtlBits     = 1 << FMC_NBL0_OFS | 1 << FMC_NBL1_OFS | 1 << FMC_NOE_OFS | 1 << FMC_NWE_OFS | 1 << ( FMC_NE1_OFS + fmc_idx ) ;

    /* enable Address valid line in muxed mode */
    if ( curr->fmcIsMuxed ) curr->fmcCtlBits |= ( 1 << FMC_NL_OFS );

    curr->hHal.hsram.Instance  = FMC_NORSRAM_DEVICE;
    curr->hHal.hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;
    init                       = &curr->hHal.hsram.Init;

    init->MemoryType         = FMC_MEMORY_TYPE_SRAM;
    init->NSBank             = IDX_TO_BANK(fmc_idx);
    init->DataAddressMux     = curr->fmcIsMuxed ? FMC_DATA_ADDRESS_MUX_ENABLE : FMC_DATA_ADDRESS_MUX_DISABLE;
    init->MemoryDataWidth    = curr->fmcDataBits == 8 ? FMC_NORSRAM_MEM_BUS_WIDTH_8 : FMC_NORSRAM_MEM_BUS_WIDTH_16;
    init->BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
    init->WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    init->WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    init->WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
    init->WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
    init->ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
    init->AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    init->WriteBurst         = FMC_WRITE_BURST_DISABLE;
    init->ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    init->PageSize           = FMC_PAGE_SIZE_NONE;
    init->WriteFifo          = FMC_WRITE_FIFO_ENABLE;


#if defined(STM32L476EVAL)
    /* When using STM32L476-EVAL with glass LCD deployed, also activate FMCs NE3 select line to deactivate LCD data output */
    curr->fmcCtlBits |= 1 << ( FMC_NE3_OFS );
#endif


    /* enable io lines and keep track of all activated IO lines  */
    FmcAddIOLines(curr);
 
    if ( HAL_SRAM_Init(&curr->hHal.hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK ) {
        DEBUG_PRINTF("SRAM init for Fmc Bank #%d failed\n",  fmc_idx );
        curr->fmcIsUsed = 0;
        return false;
    }

    FMC_DumpOneSramGeometry(fmc_idx, curr);

    return true;
}

/******************************************************************************
 * Compute value of the refresh counter, see 74x refman, section "SDRAM refresh timer register" 
 * Refresh time from row to row is total refresh time (usually 64ms ) / number of rows converted to sdram clk cycles
 * Params
 *    myData     - SDRAM timing data stored in flash
 *    sdramClkHz - actual SDRAM clk speed. Is usually half the FMC input clock
 * Returns
 *    the new refresh counter value for the SDRAM at the specified SDRAM clk
 *    0 on failure
 *****************************************************************************/
static uint32_t Fmc_CalcSdram_RefreshCounter(const Fmc_SdramTimingDataT *myData, uint32_t sdramClkHz )
{
    /* Hardware defined margins for the SDRAM counter [ 41 .. 8191 ] */
    #define MIN_CNT_VALUE   41
    #define MAX_CNT_VALUE   ((1U<<13)-1)

    uint32_t clkPerRow = (myData->T_Refresh * (sdramClkHz/1000) ) / ( 1 << myData->RowBits );   
    
    /* subtract 20 as safety margin */
    clkPerRow -= 20;

    /* check range */
    if ( clkPerRow < MIN_CNT_VALUE || clkPerRow > MAX_CNT_VALUE ) {
            DEBUG_PRINTF("Refresh counter value out of range, Value=%d\n",clkPerRow);
            return 0;
    }
    
    return clkPerRow;
}

/******************************************************************************
 * Set then HAL Timing parameters ( in multiples of fmc_ker_ck ) on base of the
 * passed "myTiming"-times (in nanoseconds ).
 * Parameters:
 * halTiming - FMC_SDRAM_TimingTypeDef structure, which will contain the sdram
 *             timing parameters [in sdram clk cycles] upon return
 * newCounterVal - New refresh counter value [in sdram clk cycles]. Has to be
 *                 programmed to the refresh counter register after return
 * myData    - all the internal sdram specific data and times
 *
 * 0 will be returned, if timing data is not settable (due to limited bitwidth
 * of HAL (resp FMC) timing data fields.
 * the underlying SDRAM clk speed [Hz]  will be returned on success. 
 *
 * NOTE: SDRAM clk is half the FMC input clock
 *
 *****************************************************************************/
static uint32_t Fmc_SetSdramTiming(FMC_SDRAM_TimingTypeDef *halTiming, const Fmc_SdramTimingDataT *myData )
{
   
    /* all timing parameters must be in the range [ 1 .. 16] ... */
    #define MIN_TXX_VALUE    1 
    #define MAX_TXX_VALUE   16 

    /*
     * convert a timinng value in ns to the corresponding value in sdram clk cycles 
     * value is always rounded up to the next greater integral value                
     * 0 is returned in case of value out of range [ 1 .. 16 ]
     */
    uint32_t convert ( uint32_t inval_ns, uint32_t picos_per_clk, const char *errtxt )
    {
        /* Convert to ps, round to next integral value and convert back to ns */
        register uint32_t retval = ( inval_ns * 1000 + picos_per_clk - 1 )  / picos_per_clk;
        /* check range */
        if ( retval > MAX_TXX_VALUE ) {
            DEBUG_PRINTF("%s, Value=%d\n",errtxt, retval);
            retval = 0; 
        }
        return retval;
    }

#if 0
    halTiming->LoadToActiveDelay    = 2;    // tMRD - Mode Register program time -> 12ns
    halTiming->ExitSelfRefreshDelay = 7;    // tXSR - Self refresh exit time     -> 70ns 
    halTiming->SelfRefreshTime      = 4;    // min. self refresh period = tRAS   -> 40ns
    halTiming->RowCycleDelay        = 7;    // tRC  - Command Period/Act-to-Act  -> 67.5ns
    halTiming->WriteRecoveryTime    = 2;    // tRAS-rRCD -> 40-20                -> 20ns
    halTiming->RPDelay              = 2;    // tRP  - Precharge to Active Delay  -> 18ns 
    halTiming->RCDDelay             = 2;    // tRCD - Active to Read/Write Delay -> 18ns 
#endif

    /* SDRAM clk is FMC input clock / 2 */
    uint32_t newClkHz = Fmc_GetClockSpeed() / 2;
 
    /* number of picosecs per SDRAM clock cycle */
    uint32_t ps_per_clk = 1000000 / (newClkHz/1000000);

    const FMC_SDRAM_TimingTypeDef *nstimes = &myData->times_ns;

    /* All SDRAM timing parameters are in units of SDRAM clk cycles */

    if ( (halTiming->LoadToActiveDelay    = convert( nstimes->LoadToActiveDelay,    ps_per_clk, "tMRD out of range" )) == 0) return 0;
    if ( (halTiming->ExitSelfRefreshDelay = convert( nstimes->ExitSelfRefreshDelay, ps_per_clk, "tXSR out of range" )) == 0) return 0;
    if ( (halTiming->SelfRefreshTime      = convert( nstimes->SelfRefreshTime,      ps_per_clk, "tRAS out of range" )) == 0) return 0;
    if ( (halTiming->RowCycleDelay        = convert( nstimes->RowCycleDelay,        ps_per_clk, "tRC out of range" ))  == 0) return 0;
    if ( (halTiming->WriteRecoveryTime    = convert( nstimes->WriteRecoveryTime,    ps_per_clk, "tWR out of range" ))  == 0) return 0;
    if ( (halTiming->RPDelay              = convert( nstimes->RPDelay,              ps_per_clk, "tRP out of range" ))  == 0) return 0;
    if ( (halTiming->RCDDelay             = convert( nstimes->RCDDelay,             ps_per_clk, "tRCD out of range" )) == 0) return 0;

    return newClkHz;
}

#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)
#define REFRESH_COUNT                    ((uint32_t)0x0603)   /* SDRAM refresh counter */

/******************************************************************************
 * @brief  Perform the SDRAM exernal memory inialization sequence 
 *         This step has to be done once on startup
 * @param  hsdram: SDRAM handle
 * @param  Command: Pointer to SDRAM command structure
 * @retval None
 *****************************************************************************/
static void SDRAM_Initialization(SDRAM_HandleTypeDef *hsdram, uint32_t sdramTargetNum )
{
  FMC_SDRAM_CommandTypeDef Command;

  /* Note the difference between FMC_SDRAM_BANKx and FMC_SDRAM_CMD_TARGET_BANKx ! */
  uint32_t cmdTarget = sdramTargetNum == FMC_SDRAM_BANK1 ? FMC_SDRAM_CMD_TARGET_BANK1 : FMC_SDRAM_CMD_TARGET_BANK2;

  /* Step 1:  Configure a clock configuration enable command */
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = cmdTarget;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = cmdTarget;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, &Command, SDRAM_TIMEOUT);
}

static void SDRAM_AutoRefresh(SDRAM_HandleTypeDef *hsdram, uint32_t sdramTargetNum, uint32_t modeReg, uint32_t refresh_counter)
{
  FMC_SDRAM_CommandTypeDef Command;
  /* Note the difference between FMC_SDRAM_BANKx and FMC_SDRAM_CMD_TARGET_BANKx ! */
  uint32_t cmdTarget = sdramTargetNum == FMC_SDRAM_BANK1 ? FMC_SDRAM_CMD_TARGET_BANK1 : FMC_SDRAM_CMD_TARGET_BANK2;

  /* Step 4 : Configure a Auto-Refresh command */
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = cmdTarget;
  Command.AutoRefreshNumber = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, &Command, SDRAM_TIMEOUT);

  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = cmdTarget;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = modeReg;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(hsdram, refresh_counter); 

}

static void SDRAM_SelfRefresh(SDRAM_HandleTypeDef *hsdram, uint32_t sdramTargetNum)
{
  FMC_SDRAM_CommandTypeDef Command;
  /* Note the difference between FMC_SDRAM_BANKx and FMC_SDRAM_CMD_TARGET_BANKx ! */
  uint32_t cmdTarget = sdramTargetNum == FMC_SDRAM_BANK1 ? FMC_SDRAM_CMD_TARGET_BANK1 : FMC_SDRAM_CMD_TARGET_BANK2;

  /* Configure a Self refresh command */
  Command.CommandMode = FMC_SDRAM_CMD_SELFREFRESH_MODE;
  Command.CommandTarget = cmdTarget;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, &Command, SDRAM_TIMEOUT);
}


/* Convert cas latency to register values */
#define ENC_CAS_LATENCY(lat)    (lat<<7)

/* Convert column bit number to register value */
#define ENC_COL_BITS(col)       (col-8)

/* Convert row bit number to register value */
#define ENC_ROW_BITS(row)       ((row-11) << 2)

/* Convert row bit number to register value */
#define ENC_DATA_WID(wid)       (wid == 8 ? FMC_SDRAM_MEM_BUS_WIDTH_8 : wid == 16 ? FMC_SDRAM_MEM_BUS_WIDTH_16 : FMC_SDRAM_MEM_BUS_WIDTH_32)

uint32_t FMC_SdramCheck();


/******************************************************************************
 * Do the Initialization of an SDRAM bank.
 * Parameters:
 *    fmc_idx      - FMC bank index [0 .. 3]
 *    sdram_data - neccessary timing and size parameters
 * initializes all associated IO pins and initializes the FMC bank. After return
 * the FMC bank is enabled/visible at its prefined memory address range
 * Returns
 *    true on success
 *    false in case of parameter or HAL layer initialization error
 *****************************************************************************/
bool Fmc_SDRAM_Init(uint32_t fmc_idx, const Fmc_SdramTimingDataT *sdram_data )
{
    FMC_SDRAM_InitTypeDef     *init;
    FMC_SDRAM_TimingTypeDef   SDRAM_Timing;
    uint32_t                  refreshCounter; // Value for the SDRAM controller refresh counter
    
    FmcDataT *curr   = FmcHandle.fmcData+fmc_idx; 
    
    /* Check for selected FMC bank being unused */
    if ( curr->fmcIsUsed ) {
        DEBUG_PRINTF("Fmc Bank #%d already used\n",  fmc_idx );
        return false;
    }

    /* Parameter Checking */
    
    /* Check DataWidth to be 8 or 16 */
    if ( sdram_data->DataBits != 8 && sdram_data->DataBits != 16 && sdram_data->DataBits != 32 ) {
        DEBUG_PRINTF("Data Width of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }

    /* Check Column address bits to be in the range [8 .. 11] */
    if ( sdram_data->ColBits < 8 || sdram_data->ColBits > 11 ) {
        DEBUG_PRINTF("Column Width of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }

    /* Check Row address bits to be in the range [11 .. 13] */
    if ( sdram_data->RowBits < 11 || sdram_data->RowBits > 13 ) {
        DEBUG_PRINTF("Column Width of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }
    /* Check SDRAM bank number to be in the range [1 ..2] */
    if ( sdram_data->sdramBankNum < 1 || sdram_data->sdramBankNum > 2 ) {
        DEBUG_PRINTF("Invalid SDRAM bank number for Fmc Bank #%d\n",  fmc_idx );
        return false;
    }

    /* Check CAS Latency to be in the range [1 .. 3] */
    if ( sdram_data->N_CASLatency < 1 || sdram_data->N_CASLatency > 3) {
        DEBUG_PRINTF("CAS latency of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }

    /* SDRAM device configuration */  
    curr->fmcRefClockSpeed = Fmc_SetSdramTiming( &SDRAM_Timing, sdram_data);
    if ( curr->fmcRefClockSpeed == 0 ) {
        DEBUG_PRINTF("Timing setup of Fmc Bank #%d failed\n",  fmc_idx );
        return false;
    }

    /* calculate refresh counter */
    if ((refreshCounter = Fmc_CalcSdram_RefreshCounter( sdram_data, curr->fmcRefClockSpeed ))==0) return false;

    curr->fmcIsUsed      = 1;
    curr->fmcType        = FMC_TYPE_SDRAM;
    curr->fmcSDBankNum   = sdram_data->sdramBankNum;
    /* Bit mask of all used Address and Data lines */
    curr->fmcAddrBits    = BITS_TO_MASK(sdram_data->ColBits) | BITS_TO_MASK(sdram_data->RowBits);
    /* independent from any SDRAM size/adress and data widths, A14 and A15 are use as bank select lines */
    curr->fmcAddrBits    |= ( 1 << 14 | 1 << 15 ); 

    curr->fmcDataBits    = BITS_TO_MASK(sdram_data->DataBits);

    /* Enable SDNWE, SDNCAS, SDNRAS, SDCLK and NBL0 in every case */
    curr->fmcCtlBits     = 1 << FMC_SDNWE_OFS | 1 << FMC_NCAS_OFS | 1 << FMC_NRAS_OFS | 1 << FMC_SDCLK_OFS | 1 << FMC_NBL0_OFS;

    /* Enable the proper Chip select and chip clock lines */
    if ( curr->fmcSDBankNum   == 1 ) {
        curr->fmcCtlBits |= ( 1 << FMC_SDNE0_OFS | 1 << FMC_SDCKE0_OFS );
    } else {
        curr->fmcCtlBits |= ( 1 << FMC_SDNE1_OFS | 1 << FMC_SDCKE1_OFS );
    }

    /* depending from the data with, enable more Byte-select lines */
    if ( sdram_data->DataBits > 8  ) curr->fmcCtlBits |= 1 << FMC_NBL1_OFS ;
    if (sdram_data->DataBits > 16 )  curr->fmcCtlBits |= ( 1 << FMC_NBL2_OFS | 1 << FMC_NBL3_OFS);

    curr->hHal.hsdram.Instance = FMC_SDRAM_DEVICE;
    init                       = &curr->hHal.hsdram.Init;
    init->SDBank               =  curr->fmcSDBankNum == 1 ? FMC_SDRAM_BANK1 : FMC_SDRAM_BANK2;
    init->ColumnBitsNumber     = ENC_COL_BITS(sdram_data->ColBits);       // HAL constants for this: FMC_SDRAM_COLUMN_BITS_NUM_9 eg
    init->RowBitsNumber        = ENC_ROW_BITS(sdram_data->RowBits);       // HAL constants for this: FMC_SDRAM_ROW_BITS_NUM_12 eg
    init->MemoryDataWidth      = ENC_DATA_WID(sdram_data->DataBits);      // HAL constants for this: FMC_SDRAM_MEM_BUS_WIDTH_8 eg
    init->CASLatency           = ENC_CAS_LATENCY(sdram_data->N_CASLatency);
    init->InternalBankNumber   = FMC_SDRAM_INTERN_BANKS_NUM_4;              // Always 4 Bank numbers
    init->WriteProtection      = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    init->SDClockPeriod        = FMC_SDRAM_CLOCK_PERIOD_2;                  // Always half the kernel clock
    init->ReadBurst            = FMC_SDRAM_RBURST_ENABLE;
    init->ReadPipeDelay        = FMC_SDRAM_RPIPE_DELAY_0;

    /* enable io lines and keep track of all activated IO lines  */
    FmcAddIOLines(curr);
 

    /* Initialize the SDRAM controller */
    if(HAL_SDRAM_Init(&curr->hHal.hsdram , &SDRAM_Timing) != HAL_OK) {
        DEBUG_PRINTF("SDRAM init for SDRAM Bank #%d failed\n",  curr->fmcSDBankNum );
        curr->fmcIsUsed = 0;
        return false;
    }

    SDRAM_Initialization(&curr->hHal.hsdram, init->SDBank);
    SDRAM_AutoRefresh(&curr->hHal.hsdram, init->SDBank, sdram_data->modeReg, refreshCounter);
    
    FMC_DumpOneSdramGeometry(fmc_idx, curr);

    
    #define BASE    0xD0000000
    #define VALUE   0xA5A5A5A5
    #define SIZE    0x2000000
    uint32_t sum = 0;
    for ( uint32_t i = 0; i < SIZE/4-1;i++ ) {
        *((uint32_t*)(BASE+i*4)) = VALUE;
        sum += VALUE;
    }
    sum = (~sum)+1;
    *((uint32_t*)(BASE+SIZE-4)) = sum;

    FMC_SdramCheck();

    return true;

}

uint32_t FMC_SdramCheck() 
{
    uint32_t sum = 0;
    uint32_t val;
    for ( uint32_t i = 0; i < SIZE/4-1;i++ ) {
        val = *((uint32_t*)(BASE+i*4));
        sum += val;
        if ( val != VALUE ) DEBUG_PRINTF("Wrong value 0x%08X at offset 0x%08X\n", val, i );
    }
    val = *((uint32_t*)(BASE+SIZE-4));
    sum += val;
    if ( sum != 0 ) DEBUG_PRINTF("Checksum 0x%08X is not correct\n",sum);
    return sum;
}

bool Fmc_ChngSdramTiming( uint32_t fmc_idx, Fmc_SdramTimingDataT *sdram_data )
{
    FmcDataT                  *curr    = FmcHandle.fmcData+fmc_idx; 
    SDRAM_HandleTypeDef       *hsdram  = &curr->hHal.hsdram;
    FMC_SDRAM_TimingTypeDef   SDRAM_Timing;
    uint32_t                  refreshCounter; // Value for the SDRAM controller refresh counter
    uint32_t                  newSdramClkSpeed;
    // uint32_t                  sdramBank = curr->fmcSDBankNum == 1 ? FMC_SDRAM_BANK1 : FMC_SDRAM_BANK2;

    /* Calculate new SDRAM timing parameters */  
    newSdramClkSpeed = Fmc_SetSdramTiming( &SDRAM_Timing, sdram_data);
    if ( newSdramClkSpeed == 0 ) {
        DEBUG_PRINTF("Change timing of Fmc Bank #%d failed\n",  fmc_idx );
        return false;
    }

    /* calculate refresh counter */
    if ((refreshCounter = Fmc_CalcSdram_RefreshCounter( sdram_data, curr->fmcRefClockSpeed ))==0) return false;
  
    /* Switch to Self refresh */
    SDRAM_SelfRefresh(hsdram, hsdram->Init.SDBank);

    /* change timing */
    FMC_SDRAM_Timing_Init(hsdram->Instance, &SDRAM_Timing, hsdram->Init.SDBank);

    /* Switch back to Auto-Refresh */
    SDRAM_AutoRefresh(hsdram, hsdram->Init.SDBank, sdram_data->modeReg, refreshCounter);

    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void Fmc_DeInit(const HW_DeviceType *self)
{
    SdmmcHandleT *myFmc = Fmc_GetMyHandle(self);

    /* DeInit GPIO */
    GpioAFDeInitAll(GetDevIdx(&HW_FMC),self->devGpioAF);
  
    /* disable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    for ( uint32_t i = 0; i < FMC_MAX_BLOCKS; i++ )
        if ( myFmc->fmcData[i].fmcIsUsed ) {
            switch ( myFmc->fmcData[i].fmcType ) {
#if USE_FMC_SRAM > 0 
                case FMC_TYPE_SRAM:
                    HAL_SRAM_DeInit(&(myFmc->fmcData[i].hHal.hsram));
                    break;
#endif
#if USE_FMC_NOR > 0 
                case FMC_TYPE_NOR:
                    HAL_NOR_DeInit(&(myFmc->fmcData[i].hHal.hnor));
                    break;
#endif
#if USE_FMC_NOR > 0 
                case FMC_TYPE_NAND:
                    HAL_NAND_DeInit&(myFmc->fmcData[i].hHal.hnand));
                    break;
#endif
#if USE_FMC_SDRAM > 0
                case FMC_TYPE_SDRAM:
                    HAL_SDRAM_DeInit(&(myFmc->fmcData[i].hHal.hsdram));
                    break;
#endif
                default:
                    DEBUG_PUTS("Fmc_DeInit: No type specific handler");
            }
    } // if, for

    /* To leave a clean state, reset QUADSPI hardware */
    HW_Reset((void *)self->devBase);

    /* disable FMC clock */
    Fmc_ClockInit(self, false );
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool Fmc_Init(const HW_DeviceType *self)
{
    SdmmcHandleT              *myFmcHandle = Fmc_GetMyHandle(self);
    const Fmc_ExtMemDataT   *extmemData;
    bool                    ret;
    uint32_t                i;
    
    Fmc_ClockInit(self, true );
    HW_Reset((void *)self->devBase);
    FmcResetMyHandle(myFmcHandle);

    for ( i = 0, ret=true; ret && i < FMC_MAX_BLOCKS; i++ ) {
        extmemData = Fmc_GetMyExtMemData(self, i);
        if ( extmemData ) switch ( extmemData->fmcType ) {
            case FMC_TYPE_SRAM:
                ret = Fmc_SRAM_Init(i,(Fmc_SramTimingDataT *)(extmemData->fmcTiming) );
                break;
            case FMC_TYPE_SDRAM:
                ret = Fmc_SDRAM_Init(i,(Fmc_SdramTimingDataT *)(extmemData->fmcTiming) );
                break;
            case FMC_TYPE_NOR:
            case FMC_TYPE_NAND:
            default:
                DEBUG_PRINTF("No init method for FMC type %d\n", extmemData->fmcType );
                ret = false;
        } // if, switch
    } // for


    return ret;
}

/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool Fmc_OnFrqChange(const HW_DeviceType *self)
{
    FMC_NORSRAM_TimingTypeDef SRAM_Timing;
    const Fmc_ExtMemDataT *extmemData;

    SdmmcHandleT *myFmcHandle = Fmc_GetMyHandle(self);
    for ( uint32_t i = 0; i < FMC_MAX_BLOCKS; i++ ) if ( myFmcHandle->fmcData[i].fmcIsUsed ) {    
        extmemData = Fmc_GetMyExtMemData(self, i);
        switch(extmemData->fmcType ) {
            case FMC_TYPE_SRAM:
            case FMC_TYPE_NOR:
                if ( !Fmc_SetSramTiming( &SRAM_Timing, (Fmc_SramTimingDataT *)(extmemData->fmcTiming) ) ) {
                    DEBUG_PRINTF("Timing setup of SRAM Fmc Bank #%d failed\n", i );
                    return false;
                }
                /* change SRAM timing */
                (void)FMC_NORSRAM_Timing_Init(myFmcHandle->fmcData[i].hHal.hsram.Instance, &SRAM_Timing, myFmcHandle->fmcData[i].hHal.hsram.Init.NSBank);
            break;
            case FMC_TYPE_SDRAM:
                if ( !Fmc_ChngSdramTiming(i, (Fmc_SdramTimingDataT *)(extmemData->fmcTiming)) ) {
                    DEBUG_PRINTF("Timing setup of SDRAM Fmc Bank #%d failed\n", i );
                    return false;
                }
            break;
            case FMC_TYPE_NAND:
            default:
                DEBUG_PRINTF("No Timing setup for FMC type %d\n", extmemData->fmcType );
                return false;
        } // switch
    } // for

    return true;
}

/******************************************************************************
 * Callback _before_ going to sleep:
 * Put all dynamic memories into self refresh mode
 *****************************************************************************/
bool Fmc_OnSleep( const HW_DeviceType *self)
{
    FmcDataT                    *curr; 
    const Fmc_ExtMemDataT       *extmemData;
    SDRAM_HandleTypeDef         *hsdram;
    uint32_t                    i;
    
    for ( i = 0; i < FMC_MAX_BLOCKS; i++ ) {
        extmemData = Fmc_GetMyExtMemData(self, i);
        if ( extmemData ) switch ( extmemData->fmcType ) {
            case FMC_TYPE_SDRAM:
                curr            = FmcHandle.fmcData+i; 
                hsdram          = &curr->hHal.hsdram;
                /* Switch to Self refresh */
                SDRAM_SelfRefresh(hsdram, hsdram->Init.SDBank);
            break;
        } // if, switch
    } // for
    return true;
}

/******************************************************************************
 * Callback _after_ wakeup:
 * Put all dynamic memories back into normal(auto) refresh mode
 *****************************************************************************/
bool Fmc_OnWakeup( const HW_DeviceType *self)
{
    FmcDataT                    *curr; 
    const Fmc_ExtMemDataT       *extmemData;
    const Fmc_SdramTimingDataT  *sdram_data;
    SDRAM_HandleTypeDef         *hsdram;
    uint32_t                    i;
    uint32_t                    refreshCounter;
    
    for ( i = 0; i < FMC_MAX_BLOCKS; i++ ) {
        extmemData = Fmc_GetMyExtMemData(self, i);
        if ( extmemData ) switch ( extmemData->fmcType ) {
            case FMC_TYPE_SDRAM:
                curr            = FmcHandle.fmcData+i; 
                hsdram          = &curr->hHal.hsdram;
                sdram_data      = (const Fmc_SdramTimingDataT *)(extmemData->fmcTiming);
                refreshCounter  = Fmc_CalcSdram_RefreshCounter( sdram_data, curr->fmcRefClockSpeed );
                /* Switch back to Auto-Refresh */
                SDRAM_AutoRefresh(hsdram, hsdram->Init.SDBank, sdram_data->modeReg, refreshCounter);
            break;
        } // if, switch
    } // for
    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(FMC_Bank1_R) && USE_FMC > 0 
    SdmmcHandleT  FmcHandle;

    #if defined( FMC_USE_IRQ )
    const HW_IrqList irq_fmc = {
        .num = 1,
        .irq = {FMC_IRQ },
    };
    #endif

    /* 16 or 32 Data pins, up to 26 Addr pins, FMC_CTL_MAX control pins */
    static const HW_GpioList_AF gpio_fmc = {
        .num  = FMC_D_MAX + FMC_A_MAX + FMC_CTL_MAX,
        .gpio = { 
            FMC_D00, FMC_D01, FMC_D02, FMC_D03, FMC_D04, FMC_D05, FMC_D06, FMC_D07,
            FMC_D08, FMC_D09, FMC_D10, FMC_D11, FMC_D12, FMC_D13, FMC_D14, FMC_D15,
            #if FMC_D_MAX > 16
                FMC_D16, FMC_D17, FMC_D18, FMC_D19, FMC_D20, FMC_D21, FMC_D22, FMC_D23, 
                FMC_D24, FMC_D25, FMC_D26, FMC_D27, FMC_D28, FMC_D29, FMC_D30, FMC_D31,
            #endif

            FMC_A00, FMC_A01, FMC_A02, FMC_A03, FMC_A04, FMC_A05, FMC_A06, FMC_A07,
            FMC_A08, FMC_A09, FMC_A10, FMC_A11, FMC_A12, FMC_A13, FMC_A14, FMC_A15,
            FMC_A16, FMC_A17, FMC_A18, FMC_A19, FMC_A20, FMC_A21, FMC_A22, FMC_A23, 
            FMC_A24, FMC_A25, 

            FMC_CTL_CLK,   FMC_CTL_NWAIT,  FMC_CTL_NOE,     FMC_CTL_NWE,
            FMC_CTL_NE1,   FMC_CTL_NE2,    FMC_CTL_NE3,     FMC_CTL_NE4,
            FMC_CTL_NBL0,  FMC_CTL_NBL1,   FMC_CTL_NBL2,    FMC_CTL_NBL3,   
            FMC_CTL_INT,     FMC_CTL_NL,
            FMC_CTL_SDNWE, FMC_CTL_SDNCAS, FMC_CTL_SDNRAS,  FMC_CTL_SDNE0,
            FMC_CTL_SDNE1, FMC_CTL_SDCKE0, FMC_CTL_SDCKE1,  FMC_CTL_SDCLK,
        }
    };

        #define FMC_T1      NULL
        #define FMC_T2      NULL

    #if defined(FMC_TYPE3)
        #if FMC_TYPE3 == FMC_TYPE_SDRAM
            static const Fmc_SdramTimingDataT memdata3 = SDRAM_TIMING3;
        #elif FMC_TYPE3 == FMC_TYPE_SRAM
            static const Fmc_SramTimingDataT memdata3  = SRAM_TIMING3;
        #else
            #error "No Memory data for external memory block 3"
        #endif
        static const Fmc_ExtMemDataT extmem3 = { &memdata3, FMC_TYPE3 };
        #define FMC_T3      &extmem3
    #else   
        #define FMC_T3      NULL
    #endif
 
    #if defined(FMC_TYPE4)
        #if FMC_TYPE4 >= FMC_TYPE_SDRAM
            static const Fmc_SdramTimingDataT memdata4 = SDRAM_TIMING4;
        #elif FMC_TYPE4 >= FMC_TYPE_SRAM
            static const Fmc_SramTimingDataT memdata4  = SRAM_TIMING4;
        #else
            #error "No Memory data for external memory block 4"
        #endif
        static const Fmc_ExtMemDataT extmem4 = { &memdata4, FMC_TYPE4 };
        #define FMC_T4      &extmem4
    #else   
        #define FMC_T4      NULL
    #endif

    static const Fmc_AdditionalDataType additional_fmc = {
        .myFmcHandle  = &FmcHandle,
        .myExtMemData = { FMC_T1, FMC_T2, FMC_T3, FMC_T4 },
    };


const HW_DeviceType HW_FMC = {
    .devName        = "FMC",
    .devBase        = FMC_Bank1_R,
    .devGpioAF      = &gpio_fmc,
    .devType        = HW_DEVICE_FMC,
    .devData        = &additional_fmc,
    .devIrqList     = 
        #if defined(FMC_USE_IRQ) 
            &irq_fmc,
        #else
            NULL,
        #endif
    .devDmaRx       = NULL, 
    .devDmaTx       = NULL, 
    .Init           = Fmc_Init,
    .DeInit         = Fmc_DeInit,
    .OnFrqChange    = Fmc_OnFrqChange,
    .AllowStop      = NULL,
    .OnSleep        = Fmc_OnSleep,
    .OnWakeUp       = Fmc_OnWakeup,
};
#endif

#if 0
void FMC_PostInit( const HW_DeviceType *self, void *args)
{
    UNUSED(self); UNUSED(args);

    FMC_NORSRAM_InitTypeDef Init;

    if (!Fmc_SRAM_Init(self, &Init)) {
        DEBUG_PUTS("FMC SRAM init failed!");
    }

    #if 0
    uint16_t *extmemptr = (uint16_t *)(0x60000000 );
    for ( uint32_t i = 0; i < 1 << 23; i++ )
        *(extmemptr + i) = 0;
    #endif
}
#endif


#endif /* if USE_FMC > 0 */


/**
  * @}
  */


