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

static void FmcGpioInitAF(const HW_GpioList_AF *gpioaf)
{
    uint32_t i;
    GPIO_InitTypeDef Init;

    Init.Mode = GPIO_MODE_AF_PP;
    Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    for ( i = 0 ; i < FMC_D_MAX; i++ ) if ( FmcHandle.allDataBits & ( 1 << i ) )
        GpioAFInitOne (&(gpioaf->gpio[i]), &Init );
    for ( i = 0 ; i < FMC_A_MAX; i++ ) if ( FmcHandle.allAddrBits & ( 1 << i ) )
        GpioAFInitOne (&(gpioaf->gpio[FMC_D_MAX+i]), &Init );
    for ( i = 0 ; i < FMC_CTL_MAX; i++ ) if ( FmcHandle.allCtlBits & ( 1 << i ) )
        GpioAFInitOne (&(gpioaf->gpio[FMC_D_MAX+FMC_D_MAX+i]), &Init );
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
    static void FMC_DumpGeometry(uint32_t idx, FmcDataT *curr)
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


bool Fmc_SRAM_Init(FMC_NORSRAM_InitTypeDef *Init, uint32_t nAddrBits, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming)
{
    uint32_t idx         = BANK_TO_IDX(Init->NSBank);
    FmcDataT *curr       = FmcHandle.fmcData+idx; 
    curr->fmcIsUsed      = 1;
    curr->fmcType        = FMC_TYPE_SRAM;
    curr->fmcIsMuxed     = Init->DataAddressMux == FMC_DATA_ADDRESS_MUX_ENABLE;
    curr->fmcDataBits    = ( Init->MemoryDataWidth == FMC_NORSRAM_MEM_BUS_WIDTH_8 ? 8 : 16 );
    curr->fmcDataBits    = BITS_TO_MASK(curr->fmcDataBits);
    curr->fmcAddrBits    = BITS_TO_MASK(nAddrBits);
    /* In case of multiplexed address/data, exclude the multiplexed address lines */
    if ( curr->fmcIsMuxed) curr->fmcAddrBits &= ~(curr->fmcDataBits);
    /* Enable NBL0, NBL1, NOE, NWE and selected enable-line */
    curr->fmcCtlBits     = 1 << FMC_NBL0_OFS | 1 << FMC_NBL1_OFS | 1 << FMC_NOE_OFS | 1 << FMC_NWE_OFS | 1 << ( FMC_NE1_OFS + idx ) ;
    /* enable Address valid line in muxed mode */
    if ( curr->fmcIsMuxed ) curr->fmcCtlBits |= ( 1 << FMC_NL_OFS );

    /* set effective io lines and activate all IO */
    FmcSetEffectiveBits();
    FmcGpioInitAF(HW_FMC.devGpioAF);
    
    FMC_DumpGeometry(idx, curr);

    curr->hHal.hsram.Instance  = FMC_NORSRAM_DEVICE;
    curr->hHal.hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

    return HAL_SRAM_Init(&curr->hHal.hsram, Timing, ExtTiming) == HAL_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void Fmc_DeInit(const HW_DeviceType *self)
{
    FmcHandleT *myFmc = (FmcHandleT *)self->devData;

    /* DeInit GPIO */
    GpioAFDeInitAll(self->devGpioAF);
  
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
    HW_SetHWClock((void *)self->devBase, false);
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool Fmc_Init(const HW_DeviceType *self)
{
    FmcHandleT *myFmcHandle = (FmcHandleT *)self->devData;

    HW_SetHWClock( (void*)self->devBase, true );
    HW_Reset((void *)self->devBase);
    FmcResetMyHandle(myFmcHandle);

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
            FMC_CTL_NE1,  FMC_CTL_NE2,   FMC_CTL_NE3, FMC_CTL_NE1,
            FMC_CTL_NBL0, FMC_CTL_NBL1,  FMC_CTL_INT, FMC_CTL_NL,
        }
    };


const HW_DeviceType HW_FMC = {
    .devName        = "FMC",
    .devBase        = FMC_Bank1_R,
    .devGpioAF      = &gpio_fmc,
    .devType        = HW_DEVICE_FMC,
    .devData        = &FmcHandle,
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
    .OnFrqChange    = NULL,
    .AllowStop      = NULL,
    .OnSleep        = NULL,
    .OnWakeUp       = NULL,
};
#endif

void FMC_PostInit( const HW_DeviceType *self, void *args)
{
    UNUSED(self); UNUSED(args);

    FMC_NORSRAM_InitTypeDef Init;
    FMC_NORSRAM_TimingTypeDef SRAM_Timing;

    /* SRAM device configuration */  
    SRAM_Timing.AddressSetupTime       = 1;
    SRAM_Timing.AddressHoldTime        = 1;
    SRAM_Timing.DataSetupTime          = 1;
    SRAM_Timing.BusTurnAroundDuration  = 0;
    SRAM_Timing.CLKDivision            = 2;
    SRAM_Timing.DataLatency            = 2;
    SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;

    Init.NSBank             = FMC_NORSRAM_BANK1;
    Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
    Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
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

    Fmc_SRAM_Init(&Init, 24, &SRAM_Timing, &SRAM_Timing);
}



#endif /* if USE_FMC > 0 */


/**
  * @}
  */


