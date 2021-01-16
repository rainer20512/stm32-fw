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
#define BITS_TO_MASK(nrofbits)  ((1ULL << (nrofbits))-1)

/* Private typedef --------------------------------------------------------------*/
typedef struct {
    uint32_t accessMode;             /* access mode ( one of FMC_ACCESS_MODE_xxx, where xxx=A,B,C,D ) */
    uint8_t  AddrBits;               /* number of address bits of ext mem chip */
    uint16_t T_AddrSet;              /* memories Address setup time [ns] */
    uint16_t T_AddrHold;             /* Memories Address hold time  [ns] */
    uint16_t T_DataSet;              /* Memories Data setup time    [ns] */
} Fmc_TimingDataT;

typedef struct {
    FmcHandleT      *myFmcHandle;    /* my associated handle */
    const Fmc_TimingDataT *myTiming[FMC_MAX_BLOCKS];
} Fmc_AdditionalDataType;

/* Private or driver functions ------------------------------------------------------*/
static  FmcHandleT * Fmc_GetMyHandle(const HW_DeviceType *self)
{
    return ((Fmc_AdditionalDataType *)(self->devData))->myFmcHandle;
}

static const Fmc_TimingDataT * Fmc_GetMyTiming(const HW_DeviceType *self, uint32_t idx)
{
    return ((Fmc_AdditionalDataType *)(self->devData))->myTiming[idx];
}


static void FmcSetEffectiveBits ( void )
{
    uint32_t i;
    uint32_t abits = 0, dbits = 0, cbits = 0;
    for ( i = 0 ; i < FMC_MAX_BLOCKS; i++ ) if ( FmcHandle.fmcData[i].fmcIsUsed ) {
        abits |= FmcHandle.fmcData[i].fmcAddrBits;
        dbits |= FmcHandle.fmcData[i].fmcDataBits;
        cbits |= FmcHandle.fmcData[i].fmcCtlBits;
    } // for, if    
    FmcHandle.allAddrBits = abits;
    FmcHandle.allDataBits = dbits;
    FmcHandle.allCtlBits  = cbits;
}

static void FmcGpioInitAF(uint32_t devIdx, const HW_GpioList_AF *gpioaf)
{
    uint32_t i;
    GPIO_InitTypeDef Init;

    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    for ( i = 0 ; i < FMC_D_MAX; i++ ) if ( FmcHandle.allDataBits & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(gpioaf->gpio[i]), &Init );
    for ( i = 0 ; i < FMC_A_MAX; i++ ) if ( FmcHandle.allAddrBits & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(gpioaf->gpio[FMC_D_MAX+i]), &Init );
    for ( i = 0 ; i < FMC_CTL_MAX; i++ ) if ( FmcHandle.allCtlBits & ( 1 << i ) )
        GpioAFInitOne (devIdx, &(gpioaf->gpio[FMC_D_MAX+FMC_A_MAX+i]), &Init );
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
    static void FMC_DumpOneGeometry(uint32_t idx, FmcDataT *curr)
    {
        
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
        } else {
            DEBUG_PUTS("   unused");
        }
    }

#endif

void FMC_DumpGeometry(void)
{
    for ( uint32_t i=0; i < FMC_MAX_BLOCKS; i++) {
        #if DEBUG_MODE > 0 
            FMC_DumpOneGeometry(i, FmcHandle.fmcData+i);
        #endif
    }
    #if DEBUG_MODE > 0
        DEBUG_PRINTF("Resulting Data lines:"); FmcPrintBitVector("D",FmcHandle.allDataBits, FMC_D_MAX);DEBUG_PRINTF("\n");
        DEBUG_PRINTF("Resulting Addr lines:"); FmcPrintBitVector("A",FmcHandle.allAddrBits, FMC_A_MAX);DEBUG_PRINTF("\n");
        DEBUG_PRINTF("Resulting Ctl  lines:"); ;FmcPrintCtlText(FmcHandle.allCtlBits);DEBUG_PRINTF("\n");
    #endif
}

void Fmc_MspInit(void)
{
  GPIO_InitTypeDef GPIO_Init_Structure;

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  /*## Data Bus #######*/
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | \
                              GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
                              GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | \
                              GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);
  
  /*## Address Bus #######*/
  /* GPIOF configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | \
                              GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);
  
  /* GPIOG configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | \
                              GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);
  
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);

  /*## NOE and NWE configuration #######*/ 
  GPIO_Init_Structure.Pin = GPIO_PIN_4 |GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /*## Enable Bank pin configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

#if 0 && defined(USE_STM32L476G_EVAL_REVB)
  /*## LCD NE3 configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);
#endif /* USE_STM32L476G_EVAL_REVB */

  /*## NBL0, NBL1 configuration #######*/
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure); 
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
static bool Fmc_SetTiming( FMC_NORSRAM_TimingTypeDef *halTiming, const Fmc_TimingDataT *myTiming )
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

bool Fmc_SRAM_Init(const HW_DeviceType *self, FMC_NORSRAM_InitTypeDef *Init)
{
    FMC_NORSRAM_TimingTypeDef SRAM_Timing;
    /* SRAM device configuration */  

    uint32_t idx         = BANK_TO_IDX(Init->NSBank);
    FmcDataT *curr       = FmcHandle.fmcData+idx; 
    if ( curr->fmcIsUsed ) {
        DEBUG_PRINTF("Fmc Bank #%d already used\n",  idx );
        return false;
    }
    
    const Fmc_TimingDataT *timing = Fmc_GetMyTiming(self, idx);
    if ( !timing ) {
        DEBUG_PRINTF("No timing for Fmc Bank #%d!\n",  idx );
        return false;
    }
    if ( !Fmc_SetTiming( &SRAM_Timing,  timing) ) {
        DEBUG_PRINTF("Timing setup of Fmc Bank #%d failed\n",  idx );
        return false;
    }

    curr->fmcIsUsed      = 1;
    curr->fmcType        = FMC_TYPE_SRAM;
    curr->fmcIsMuxed     = Init->DataAddressMux == FMC_DATA_ADDRESS_MUX_ENABLE;
    curr->fmcDataBits    = ( Init->MemoryDataWidth == FMC_NORSRAM_MEM_BUS_WIDTH_8 ? 8 : 16 );
    curr->fmcDataBits    = BITS_TO_MASK(curr->fmcDataBits);
    curr->fmcAddrBits    = BITS_TO_MASK(timing->AddrBits);
    /* In case of multiplexed address/data, exclude the multiplexed address lines */
    if ( curr->fmcIsMuxed) curr->fmcAddrBits &= ~(curr->fmcDataBits);
    /* Enable NBL0, NBL1, NOE, NWE and selected enable-line */
    curr->fmcCtlBits     = 1 << FMC_NBL0_OFS | 1 << FMC_NBL1_OFS | 1 << FMC_NOE_OFS | 1 << FMC_NWE_OFS | 1 << ( FMC_NE1_OFS + idx ) ;

#if defined(STM32L476EVAL)
    /* When using STM32L476-EVAL with glass LCD deployed, also activate FMCs NE3 select line to deactivate LCD data output */
    curr->fmcCtlBits |= 1 << ( FMC_NE3_OFS );
#endif

    /* enable Address valid line in muxed mode */
    if ( curr->fmcIsMuxed ) curr->fmcCtlBits |= ( 1 << FMC_NL_OFS );

    /* set effective io lines and activate all IO */
    FmcSetEffectiveBits();
    FmcGpioInitAF(GetDevIdx(&HW_FMC), HW_FMC.devGpioAF);
 
    FMC_DumpOneGeometry(idx, curr);

    curr->hHal.hsram.Instance  = FMC_NORSRAM_DEVICE;
    curr->hHal.hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;
    curr->hHal.hsram.Init      = *Init;

    if ( HAL_SRAM_Init(&curr->hHal.hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK ) {
        DEBUG_PRINTF("SRAM init for Fmc Bank #%d failed\n",  idx );
        return false;
    }

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
                default:
                    DEBUG_PUTS("Fmc_DeInit: No type specific handler");
            }
    } // if, for

    /* To leave a clean state, reset QUADSPI hardware */
    HW_Reset((void *)self->devBase);

    /* disable QUADSPI clock */
    Fmc_ClockInit(self, false );
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool Fmc_Init(const HW_DeviceType *self)
{
    FmcHandleT *myFmcHandle = Fmc_GetMyHandle(self);

    Fmc_ClockInit(self, true );
    HW_Reset((void *)self->devBase);
    FmcResetMyHandle(myFmcHandle);

    return true;
}

/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool Fmc_OnFrqChange(const HW_DeviceType *self)
{
    FMC_NORSRAM_TimingTypeDef SRAM_Timing;
    const Fmc_TimingDataT *timing;

    FmcHandleT *myFmcHandle = Fmc_GetMyHandle(self);
    for ( uint32_t i = 0; i < FMC_MAX_BLOCKS; i++ ) if ( myFmcHandle->fmcData[i].fmcIsUsed ) {    
        timing = Fmc_GetMyTiming(self, i);
        if ( !Fmc_SetTiming( &SRAM_Timing,  timing) ) {
            DEBUG_PRINTF("Timing setup of Fmc Bank #%d failed\n", i );
            return false;
        }
        
          /* change SRAM timing */
          (void)FMC_NORSRAM_Timing_Init(myFmcHandle->fmcData[i].hHal.hsram.Instance, &SRAM_Timing, myFmcHandle->fmcData[i].hHal.hsram.Init.NSBank);

    }

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

    /* 16 Data pins, 26 Addr pinx FMC_CTL_MAX control pins */
    static const HW_GpioList_AF gpio_fmc = {
        .num  = FMC_D_MAX + FMC_A_MAX + FMC_CTL_MAX,
        .gpio = { 
            FMC_D00, FMC_D01, FMC_D02, FMC_D03, FMC_D04, FMC_D05, FMC_D06, FMC_D07,
            FMC_D08, FMC_D09, FMC_D10, FMC_D11, FMC_D12, FMC_D13, FMC_D14, FMC_D15,
            FMC_A00, FMC_A01, FMC_A02, FMC_A03, FMC_A04, FMC_A05, FMC_A06, FMC_A07,
            FMC_A08, FMC_A09, FMC_A10, FMC_A11, FMC_A12, FMC_A13, FMC_A14, FMC_A15,
            FMC_A16, FMC_A17, FMC_A18, FMC_A19, FMC_A20, FMC_A21, FMC_A22, FMC_A23, 
            FMC_A24, FMC_A25, 
            FMC_CTL_CLK,  FMC_CTL_NWAIT, FMC_CTL_NOE, FMC_CTL_NWE,
            FMC_CTL_NE1,  FMC_CTL_NE2,   FMC_CTL_NE3, FMC_CTL_NE4,
            FMC_CTL_NBL0, FMC_CTL_NBL1,  FMC_CTL_INT, FMC_CTL_NL,
        }
    };

    #if defined(TIMING1)
        static const Fmc_TimingDataT timing1 = TIMING1;
        #define FMC_T1      &timing1
    #else   
        #define FMC_T1      NULL
    #endif
    #if defined(TIMING2)
        static const Fmc_TimingDataT timing2 = TIMING2;
        #define FMC_T2      &timing2
    #else   
        #define FMC_T2      NULL
    #endif
    #if defined(TIMING3)
        static const Fmc_TimingDataT timing3 = TIMING3;
        #define FMC_T3      &timing3
    #else   
        #define FMC_T3      NULL
    #endif
    #if defined(TIMING4)
        static const Fmc_TimingDataT timing4 = TIMING4;
        #define FMC_T4      &timing4
    #else   
        #define FMC_T4      NULL
    #endif
    static const Fmc_AdditionalDataType additional_fmc = {
        .myFmcHandle = &FmcHandle,
        .myTiming    = { FMC_T1, FMC_T2, FMC_T3, FMC_T4 },
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


void FMC_PostInit( const HW_DeviceType *self, void *args)
{
    UNUSED(self); UNUSED(args);

    FMC_NORSRAM_InitTypeDef Init;
    Init.NSBank             = FMC_NORSRAM_BANK3;
    Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
    Init.MemoryType         = FMC_MEMORY_TYPE_PSRAM;
    Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
    Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
    Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
    Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
    Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
    Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    Init.PageSize           = FMC_PAGE_SIZE_NONE;
    Init.WriteFifo          = FMC_WRITE_FIFO_ENABLE;

    if (!Fmc_SRAM_Init(self, &Init)) {
        DEBUG_PUTS("FMC SRAM init failed!");
    }

    #if 0
    uint16_t *extmemptr = (uint16_t *)(0x60000000 );
    for ( uint32_t i = 0; i < 1 << 23; i++ )
        *(extmemptr + i) = 0;
    #endif
}



#endif /* if USE_FMC > 0 */


/**
  * @}
  */


