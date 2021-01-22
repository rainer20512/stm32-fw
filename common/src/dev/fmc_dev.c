/*
 ******************************************************************************
 * @file    fmc_dev.c
 * @author  Rainer
 * @brief  FMC hardware wrapped into HW_Device
 *
 *****************************************************************************/

/** @addtogroup QUADSPI
  * @{
   
*/
#include "config/config.h"

#if USE_FMC > 0 

/* Debug ------------------------------------------------------------------------*/
#define DEBUG_FMC          1

#include "config/devices_config.h"
#include "config/fmc_config.h"
#include "dev/fmc_dev.h"
#include "system/hw_util.h"

#include "error.h"
#include "task/minitask.h"

#include "debug_helper.h"

/* My macros --------------------------------------------------------------------*/
#define BANK_TO_IDX(bank_id)   ((bank_id) >> 1)
#define IDX_TO_BANK(idx)       (idx << 1)
#define BITS_TO_MASK(nrofbits)  ((1ULL << (nrofbits))-1)

/* Private typedef --------------------------------------------------------------*/
typedef struct {
     uint32_t accessMode;             /* access mode ( one of FMC_ACCESS_MODE_xxx, where xxx=A,B,C,D )         */
     uint8_t  AddrBits;               /* number of address bits of ext mem chip                                */
     uint8_t  DataBits;               /* number of data bits of ext mem chip ( only 8 or 16 are valid for sram */
     uint8_t  AD_Muxed;               /* != 0, when Data and address bits are multiplexed                      */
     uint16_t T_AddrSet;              /* memories Address setup time [ns] */
     uint16_t T_AddrHold;             /* Memories Address hold time  [ns] */
     uint16_t T_DataSet;              /* Memories Data setup time    [ns] */
} Fmc_SramTimingDataT;

typedef struct {
     uint32_t accessMode;             /* access mode ( one of FMC_ACCESS_MODE_xxx, where xxx=A,B,C,D )         */
     uint8_t  RowBits;                /* number of row address bits of ext mem chip                            */
     uint8_t  ColBits;                /* number of column address bits of ext mem chip                         */
     uint8_t  DataBits;               /* number of data bits of ext mem ( only 8,16 and 32 are valid f. sdram) */
     uint16_t T_AddrSet;              /* memories Address setup time [ns] */
     uint16_t T_AddrHold;             /* Memories Address hold time  [ns] */
     uint16_t T_DataSet;              /* Memories Data setup time    [ns] */
} Fmc_SdramTimingDataT;

typedef struct {
     FmcTypeE fmcType;                /* type of FMC external memory */
     const void *fmcTiming;           /* Ptr to corresponding timing data */
} Fmc_ExtMemDataT;

typedef struct {
    FmcHandleT              *myFmcHandle;    /* my associated handle */
    const Fmc_ExtMemDataT   *myExtMemData[FMC_MAX_BLOCKS];
} Fmc_AdditionalDataType;

/* Private or driver functions ------------------------------------------------------*/
static  FmcHandleT * Fmc_GetMyHandle(const HW_DeviceType *self)
{
    return ((Fmc_AdditionalDataType *)(self->devData))->myFmcHandle;
}

static const Fmc_ExtMemDataT * Fmc_GetMyExtMemData(const HW_DeviceType *self, uint32_t idx)
{
    return ((Fmc_AdditionalDataType *)(self->devData))->myExtMemData[idx];
}


/******************************************************************************
 * Add/enable the IO lines required by the current FMC bank with respect to the
 * io lines enabled so far
 *****************************************************************************/
static void FmcAddIOLines(FmcDataT *curr)
{
    uint32_t         i;
    uint32_t         delta;
    GPIO_InitTypeDef Init;
    uint32_t         devIdx;

    Init.Mode  = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    devIdx     = (uint32_t)GetDevIdx(&HW_FMC);

    /* Enable all data lines, that are not enabled so far */
    delta = curr->fmcDataBits & ~FmcHandle.allDataBits;
    for ( i = 0 ; i < FMC_D_MAX; i++ ) if ( delta & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(HW_FMC.devGpioAF->gpio[i]), &Init );
    FmcHandle.allDataBits |= delta;

    /* Enable all address lines, that are not enabled so far */
    delta = curr->fmcAddrBits & ~FmcHandle.allAddrBits;
    for ( i = 0 ; i < FMC_A_MAX; i++ ) if ( delta & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(HW_FMC.devGpioAF->gpio[FMC_D_MAX+i]), &Init );
    FmcHandle.allAddrBits |= delta;

    /* Enable all control lines, that are not enabled so far */
    delta = curr->fmcCtlBits & ~FmcHandle.allCtlBits;
    for ( i = 0 ; i < FMC_CTL_MAX; i++ ) if ( delta & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(HW_FMC.devGpioAF->gpio[FMC_D_MAX+FMC_A_MAX+i]), &Init );
    FmcHandle.allCtlBits |= delta;
}

/******************************************************************************
 * Clear the entire CanHandleT structure
 *****************************************************************************/
static void FmcResetMyHandle ( FmcHandleT *handle ) 
{
    memset(handle, 0, sizeof(FmcHandleT) );
}




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
            if ( mask & bits )  DEBUG_PRINTF(" %s", fmc_get_ctl_txt(idx));
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


/**************************************************************************************
 * Some Devices support different clock sources for FSMC. Make sure, that             *   
  * Fmc_SetClockSource and Fmc_GetClockSpeed() will match                            *
 *************************************************************************************/
#if defined(STM32L476xx) || defined(STM32L496xx)
    /* STM32L4xx has no clock mux for QUADSPI device */
    #define Fmc_SetClockSource(a)           (true)
    #define Fmc_GetClockSpeed()             HAL_RCC_GetHCLKFreq()
#elif defined(STM32H745xx) || defined(STM32H742xx)  || defined(STM32H743xx)
    static bool Fmc_SetClockSource(const void *hw)
    {
      UNUSED(hw);
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      /* FSMC has to be operated with HCLK. Routines, which will set      */
      /* FMC timing constants, will call Fmc_GetClockSpeed() to determine */
      /* the current clock speed                                          */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FMC; 
      PeriphClkInit.FmcClockSelection    = RCC_FMCCLKSOURCE_D1HCLK;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for FSMC");
        return false;
      }

      return true;
    }
    #define Fmc_GetClockSpeed()             HAL_RCC_GetHCLKFreq()
#else 
    #error "No qspi clock assignment defined"
#endif

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
 * FALSE will be returned, if timing data is not settable (due to limited bitwidth
 * of HAL (resp FMC) timing data fields.
 * TRUE will be returned on success
 *****************************************************************************/
static bool Fmc_SetSramTiming( FMC_NORSRAM_TimingTypeDef *halTiming, const Fmc_SramTimingDataT *myTiming )
{
    #define MIN_ADDSET       0         /* min clk cycles for address setup time */
    #define MAX_ADDSET      15         /* max clk cycles for address setup time */
    #define MIN_ADDHLD       1         /* min clk cycles for address hold  time */
    #define MAX_ADDHLD      15         /* max clk cycles for address hold  time */
    #define MIN_DATASET      1         /* min clk cycles for data setup time    */
    #define MAX_DATASET    255         /* max clk cycles for data setup time    */

    uint32_t work;

    /* number of picosecs per FMC clock cycle */
    uint32_t ps_per_clk = 1000000 / (Fmc_GetClockSpeed()/1000000);

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

    return true;
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
    
    if ( !Fmc_SetSramTiming( &SRAM_Timing, sram_timing) ) {
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
 * Do the Initialization of an SDRAM bank.
 * Parameters:
 *    fmc_idx      - FMC bank index [0 .. 3]
 *    sdram_timing - neccessary timing and size parameters
 * initializes all associated IO pins and initializes the FMC bank. After return
 * the FMC bank is enabled/visible at its prefined memory address range
 * Returns
 *    true on success
 *    false in case of parameter or HAL layer initialization error
 *****************************************************************************/
bool Fmc_SDRAM_Init(uint32_t fmc_idx, Fmc_SdramTimingDataT *sdram_timing )
{
    FMC_SDRAM_InitTypeDef     *init;
    FMC_SDRAM_TimingTypeDef   SDRAM_Timing;
    
    FmcDataT *curr   = FmcHandle.fmcData+fmc_idx; 
    
    /* Check for selected FMC bank being unused */
    if ( curr->fmcIsUsed ) {
        DEBUG_PRINTF("Fmc Bank #%d already used\n",  fmc_idx );
        return false;
    }

    /* SDRAM device configuration */  

    
    if ( !Fmc_SetSdramTiming( &SDRAM_Timing, sdram_timing) ) {
        DEBUG_PRINTF("Timing setup of Fmc Bank #%d failed\n",  fmc_idx );
        return false;
    }

    /* Check DataWidth to be 8 or 16 */
    if ( sdram_timing->DataBits != 8 && sdram_timing->DataBits != 16 && sdram_timing->DataBits != 32 ) {
        DEBUG_PRINTF("Data Width of Fmc Bank #%d invalid\n",  fmc_idx );
        return false;
    }

    curr->fmcIsUsed      = 1;
    curr->fmcType        = FMC_TYPE_SDRAM;
    curr->fmcIsMuxed     = 0;
    curr->fmcDataBits    = sram_timing->DataBits; 
    curr->fmcDataBits    = BITS_TO_MASK(curr->fmcDataBits);
    curr->fmcAddrBits    = BITS_TO_MASK(sram_timing->AddrBits);

    /* Enable NBL0, NBL1, NOE, NWE and selected enable-line */
    curr->fmcCtlBits     = 1 << FMC_NBL0_OFS | 1 << FMC_NBL1_OFS | 1 << FMC_NOE_OFS | 1 << FMC_NWE_OFS | 1 << ( FMC_NE1_OFS + fmc_idx ) ;

  hsdram.Instance = FMC_SDRAM_DEVICE;

    curr->hHal.hsdram.Instance = FMC_SDRAM_DEVICE;
    init                       = &curr->hHal.hsdram.Init;
----> here
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

///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void Fmc_DeInit(const HW_DeviceType *self)
{
    FmcHandleT *myFmc = Fmc_GetMyHandle(self);

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
    FmcHandleT              *myFmcHandle = Fmc_GetMyHandle(self);
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

    FmcHandleT *myFmcHandle = Fmc_GetMyHandle(self);
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
            case FMC_TYPE_NAND:
            case FMC_TYPE_SDRAM:
            default:
                DEBUG_PRINTF("No Timing setup for FMC type %d\n", extmemData->fmcType );
                return false;
        } // switch
    } // for

    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if defined(FMC_Bank1_R) && USE_FMC > 0 
    FmcHandleT  FmcHandle;

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

    #if defined(FMC_TYPE1)
        #if FMC_TYPE1 == FMC_TYPE_SRAM
            static const Fmc_SramTimingDataT memdata1  = SRAM_TIMING1;
        #elif FMC_TYPE1 == FMC_TYPE_SDRAM
            static const Fmc_SdramTimingDataT memdata1 = SDRAM_TIMING1;
        #else
            #error "No Memory data for external memory block 1"
        #endif
        static const Fmc_ExtMemDataT extmem1 = { FMC_TYPE1, &memdata1 };
        #define FMC_T1      &extmem1
    #else   
        #define FMC_T1      NULL
    #endif

    #if defined(TIMING2)
        static const Fmc_ExtMemDataT timing2 = TIMING2;
        #define FMC_T2      &timing2
    #else   
        #define FMC_T2      NULL
    #endif
    #if defined(FMC_TYPE3)
        #if FMC_TYPE3 == FMC_TYPE_SRAM
            static const Fmc_SramTimingDataT memdata3  = SRAM_TIMING3;
        #elif FMC_TYPE3 == FMC_TYPE_SDRAM
            static const Fmc_SdramTimingDataT memdata3 = SDRAM_TIMING3;
        #else
            #error "No Memory data for external memory block 3"
        #endif
        static const Fmc_ExtMemDataT extmem3 = { FMC_TYPE3, &memdata3 };
        #define FMC_T3      &extmem3
    #else   
        #define FMC_T3      NULL
    #endif
    #if defined(TIMING4)
        static const Fmc_ExtMemDataT timing4 = TIMING4;
        #define FMC_T4      &timing4
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
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
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


