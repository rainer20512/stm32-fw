/*
 ******************************************************************************
 * @file    sdmmc_dev.c
 * @author  Rainer
 * @brief   SDMMC hardware wrapped into HW_Device
 *
 * @note    When using SDMMC in DMA mode, pls note that
 *          - SDMMC1 MDMA has access to AXSI RAM, Flash 
 *            and external memory via QUADSPI or FMC
 *          - SDMMC2 MDMA has access to AXSI RAM, SRAM1, 
 *            SRAM2, SRAM3, Flash and external memory 
 *            via QUADSPI or FMC
 *****************************************************************************/

/** @addtogroup SDMMC
  * @{
   
*/
#include "config/config.h"

#if USE_SDMMC > 0 

/* Debug ------------------------------------------------------------------------*/
#define DEBUG_SDMMC          1

#include "config/devices_config.h"
#include "config/sdmmc_config.h"
#include "dev/sdmmc_dev.h"
#include "system/hw_util.h"

#include "error.h"
#include "task/minitask.h"

#include "debug_helper.h"
#include "log.h"

/* Private defines --------------------------------------------------------------*/
#define SD_DEFAULT_BLOCK_SIZE 512                   /* logical block size to use */

#ifndef SD_WRITE_TIMEOUT
  #define SD_WRITE_TIMEOUT       100U
#endif
#ifndef SD_READ_TIMEOUT
  #define SD_READ_TIMEOUT        100U
#endif

/* Private typedef --------------------------------------------------------------*/



typedef struct {
    SdmmcHandleT              *mySdmmcHandle;            /* my associated handle */
    uint8_t                   myDataLinesNum;       /* number of used Data lines */             
} Sdmmc_AdditionalDataType;


/* Private or driver functions ------------------------------------------------------*/

static SdmmcHandleT *GetMyHandleFromHalHandle ( SD_HandleTypeDef *hsd )
{
#if defined(USE_SDMMC1)
    if      ( &Sdmmc1Handle.halHandle == hsd ) 
        return &Sdmmc1Handle;
#endif
#if defined(USE_SDMMC2)
    if ( &Sdmmc2Handle.halHandle == hsd )  
        return &Sdmmc2Handle;
#endif    
    return NULL;
}

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
    /* STM32H7xx: Sdmmc may be clocked by PLL1Q (default) or PLL2R */
    
    /*********************************************************************************
     * Get the SDMMC clock sourcte, which is either either PLL1Q or PLL2R
     * @returns  0 - SDMMC clk source is PLL1Q; 1 - SDMMC clk source is PLL2R
     *********************************************************************************/
    static uint32_t Sdmmc_GetClockSource(void)
    {
        return ( RCC->D1CCIPR & RCC_D1CCIPR_SDMMCSEL ) ? 1: 0 ;
    }

    /*********************************************************************************
     * Set the SDMMC clock sourcte to either PLL1Q or PLL2R
     * @param clksrc - 0 = PLL1Q; 1=PLL2R
     *********************************************************************************/
    static bool Sdmmc_SetClockSource(const void *hw, uint32_t clksrc)
    {
      UNUSED(hw);
      if ( clksrc ) 
        SET_BIT(RCC->D1CCIPR, RCC_D1CCIPR_SDMMCSEL);
      else
        CLEAR_BIT(RCC->D1CCIPR, RCC_D1CCIPR_SDMMCSEL);
      RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

      /* FSMC has to be operated with HCLK. Routines, which will set      */
      /* FMC timing constants, will call Fmc_GetClockSpeed() to determine */
      /* the current clock speed                                          */

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC; 
      PeriphClkInit.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        DEBUG_PUTS("failed to set CLK source for SDMMC");
        return false;
      }

      return true;
    }


    /*********************************************************************************
     * Set the SDMMC clock speed by reprogramming the actual clock source
     * @param xxxkhz - desired SDMMC input clock speed in kHz
     * @note If you want to change the Clock source, change it by "Sdmmc_SetClockSource"
     *       before you set the speed
     *********************************************************************************/
    static bool Sdmmc_SetClockSpeed(const void *hw, uint32_t xxxkhz)
    {
      UNUSED(hw);

      return true;
    }

    /*********************************************************************************
     * Return the SDMMC clock speed 
     * The effective clock speed is affected by the SDMMC IP input speed from either 
     *********************************************************************************/
    static bool Sdmmc_GetClockSpeed(const void *hw, uint32_t xxxkhz)
    {
      UNUSED(hw);

      return true;
    }

#else 
    #error "No FMC clock assignment defined"
#endif


#if DEBUG_MODE >  0


#endif



/*
 * Init or DeInit Clock / clocksource 
 */
static bool Sdmmc_ClockInit(const HW_DeviceType *self, bool bDoInit)
{
    /* Select clock source on init*/
    if ( bDoInit ) {
        if ( !Sdmmc_SetClockSource(self->devBase, 0) ) return false;
    }

    /* Enable/Disable clock */
    HW_SetHWClock( (void*)self->devBase, bDoInit );
    return true;
}

static void Sdmmc_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    uint32_t devIdx = GetDevIdx(self);
    GpioAFInitAll(devIdx, self->devGpioAF, &GPIO_InitStruct);
}



///////////////////////////////////////////////////////////////////////////////
// Device functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void Sdmmc_DeInit(const HW_DeviceType *self)
{
    SdmmcHandleT *myHandle = Sdmmc_GetMyHandle(self);

    /* DeInit GPIO */
    GpioAFDeInitAll(GetDevIdx(self),self->devGpioAF);
  
    /* disable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* To leave a clean state, reset hardware */
    HW_Reset((void *)self->devBase);

    /* disable SDMMC clock */
    Sdmmc_ClockInit(self, false );
}

/**************************************************************************************
 * Initialize quadrature encoder, i.e. set all GPIO pins and activate all interrupts  *
 * After successful initialization, the device is deactivated to reduce power         *
 * consumption. So, to use the device, it has to be activated first                   *
 *************************************************************************************/
bool Sdmmc_Init(const HW_DeviceType *self)
{
    SdmmcHandleT            *myHandle = Sdmmc_GetMyHandle(self);
    SD_HandleTypeDef        *hsd;
    Sdmmc_GPIO_Init(self);
    if ( !Sdmmc_ClockInit(self, true ) ) return false;

    HW_Reset((void *)self->devBase);
    SdmmcResetMyHandle(myHandle);

    /* Configure the NVIC, enable interrupts */
    if (self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);

    hsd                           = &myHandle->halHandle;
    hsd->Instance                 = (SD_TypeDef *)self->devBase;
    hsd->Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd->Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd->Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
#if  USE_SD_TRANSCEIVER > 0
    hsd->Init.TranceiverPresent   = SDMMC_TRANSCEIVER_PRESENT;
#endif
    #if ( USE_SD_HIGH_PERFORMANCE > 0 )
        hsd->Init.ClockDiv            = SDMMC_HSpeed_CLK_DIV;
    #else
        hsd->Init.ClockDiv            = SDMMC_INIT_CLK_DIV;
    #endif

    /* HAL SD initialization   */

    HAL_StatusTypeDef ret = HAL_SD_Init(hsd);
    if ( ret != HAL_OK ) {
        #if DEBUG_SDMMC > 0 
            if ( ret == HAL_TIMEOUT )
                DEBUG_PRINTF("%s: Feels like no card in slot...\n", self->devName);
            else 
                DEBUG_PRINTF("Error initializing %s\n", self->devName);
        #endif
        return false;
    }

    /* Card is now in state "transfer" */

    HAL_SD_CardInfoTypeDef ci;
    HAL_SD_GetCardInfo ( hsd, &ci );
    /* Number of R/W sectors */
    myHandle->uNumBlocks = ci.LogBlockNbr;
    /* size of one R/W sector */
    myHandle->uSectorSize = ci.LogBlockSize;
    /* erase block size in unit of sectors */
    myHandle->uEraseBlockSize = ci.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;

    if(HAL_SD_ConfigWideBusOperation(hsd, SDMMC_BUS_WIDE_4B) != HAL_OK) {
        #if DEBUG_SDMMC > 0
            DEBUG_PRINTF("%s: Cannot set wide bus mode\n",self->devName);
        #endif
        return false;
    }

    if ( HAL_SD_ConfigSpeedBusOperation(hsd, SDMMC_SPEED_MODE_HIGH) ) {
        #if DEBUG_SDMMC > 0
            DEBUG_PRINTF("%s: Cannot set speed mode\n",self->devName);
        #endif
        return false;
    }

    myHandle->bIsInitialized = true;

    #if DEBUG_SDMMC > 0
        SDMMC_DebugInfo(self);
    #else
        SDMMC_DumpGeometry(self);
    #endif

    return true;
}

/******************************************************************************
 * Callback _after_ frequency changes: Recalculate the QUADSPI hardware 
 * prescaler to a give an operating freqency at or below desired frequency
 *****************************************************************************/
bool Sdmmc_OnFrqChange(const HW_DeviceType *self)
{
    SdmmcHandleT *myHandle = Sdmmc_GetMyHandle(self);

    return true;
}

/******************************************************************************
 * Callback _before_ going to sleep:
 * Put all dynamic memories into self refresh mode
 *****************************************************************************/
bool Sdmmc_OnSleep( const HW_DeviceType *self)
{
    return true;
}

/******************************************************************************
 * Callback _after_ wakeup:
 * Put all dynamic memories back into normal(auto) refresh mode
 *****************************************************************************/
bool Sdmmc_OnWakeup( const HW_DeviceType *self)
{
    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Global Variables  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if USE_SDMMC1 > 0 
    SdmmcHandleT  Sdmmc1Handle;

    #if defined( SDMMC1_USE_IRQ )
    const HW_IrqList irq_sdmmc1 = {
        .num = 1,
        .irq = {SDMMC1_IRQ },
    };
    #endif

    /* 16 or 32 Data pins, up to 26 Addr pins, FMC_CTL_MAX control pins */
    static const HW_GpioList_AF gpio_sdmmc1 = {
        .num  = 
            #if defined(SDMMC1_USE_8LINES)
                2+8,
            #else
                2+4,
            #endif
        .gpio = { 
            SDMMC1_CK, SDMMC1_CMD, 
            SDMMC1_D0, SDMMC1_D1, SDMMC1_D2, SDMMC1_D3,
            SDMMC1_D4, SDMMC1_D5, SDMMC1_D6, SDMMC1_D7,
        }
    };


    static const Sdmmc_AdditionalDataType additional_sdmmc1 = {
        .mySdmmcHandle  = &Sdmmc1Handle,
        .myDataLinesNum = 
            #if defined(SDMMC1_USE_8LINES)
                8,
            #else
                4,
            #endif
    };


    const HW_DeviceType HW_SDMMC1 = {
        .devName        = "SDMMC1",
        .devBase        = SDMMC1,
        .devGpioAF      = &gpio_sdmmc1,
        .devType        = HW_DEVICE_SDMMC,
        .devData        = &additional_sdmmc1,
        .devIrqList     = 
            #if defined(SDMMC1_USE_IRQ) 
                &irq_sdmmc1,
            #else
                NULL,
            #endif
        // SDMMC IP has its own internal DMA controller, so these handles should be NULL
        .devDmaRx       = NULL, 
        .devDmaTx       = NULL, 
        .Init           = Sdmmc_Init,
        .DeInit         = Sdmmc_DeInit,
        .OnFrqChange    = Sdmmc_OnFrqChange,
        .AllowStop      = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
    #endif

#if 0
void SDMMC_PostInit( const HW_DeviceType *self, void *args)
{
    UNUSED(self); UNUSED(args);

}
#endif

/******************************************************************************
 *
 * SDMMC high level transfer functions
 *
 *****************************************************************************/


/******************************************************************************
 * Get current card transfer status
 * @returns true  - if card status is HAL_SD_CARD_READY 
 *          false - in all other cases, that are
 *****************************************************************************/
#if DEBUG_SDMMC > 0
    const char * const state_txt[] = { "Ready",     "Ident",       "Standby",      "Transfer", 
                                        "Sending",  "Receiving",   "Programming",  "Disconnected", 
                                        "????",     "Error", 
                                     };
    static const char *get_state_str( uint32_t state ) 
    {
        if ( state > HAL_SD_CARD_DISCONNECTED && state != HAL_SD_CARD_ERROR ) state = HAL_SD_CARD_DISCONNECTED + 1;
        if ( state == HAL_SD_CARD_ERROR ) state = HAL_SD_CARD_DISCONNECTED + 2;
        state -= 1;
        if ( state < sizeof(state_txt)/sizeof(char *) ) 
            return state_txt[state];
        else
            return "????";
    }
#endif

/******************************************************************************
 * Check for card being in "transfer" state, dump error message if not
 *****************************************************************************/
bool SDMMC_CardInTransferState(SdmmcHandleT *myHandle)
{
  uint32_t ret = HAL_SD_GetCardState(&myHandle->halHandle);
  #if DEBUG_SDMMC > 0
    if ( ret != HAL_SD_CARD_TRANSFER ) {
        DEBUG_PRINTF("HAL_SD_GetCardState returned state %s(#%d) when exepecting ""trans""\n", get_state_str(ret), ret );
    }
  #endif
  return ret == HAL_SD_CARD_TRANSFER;
}

/******************************************************************************
 * Wait for card going into "transfer" state
 * @param - timeout: max time [ms] to wait before an error is flagged
 * @returns true, if "transfer" state is reached before timeout
 * @returns false, if "transfer" state is not reached until timeout
 * @note to be used after long lasting write or erase operations
 *****************************************************************************/
bool SDMMC_WaitForTransferState(SdmmcHandleT *myHandle, uint32_t tmo)
{
    uint32_t ret;
    uint32_t loops = 0;
    #if DEBUG_SDMMC > 0
        LOGU_VERBOSE("Wait for transfer starts\n" );
    #endif

    uint32_t timeout = HAL_GetTick();
    while((HAL_GetTick() - timeout) < tmo) {
        ret = HAL_SD_GetCardState(&myHandle->halHandle); 
        loops++;
        if (ret == HAL_SD_CARD_TRANSFER ) break;
    }

    #if DEBUG_SDMMC > 0
        LOGU_VERBOSE("Wait for transfer finishes after %d loops: %s\n", loops, ret == HAL_SD_CARD_TRANSFER ? "OK" : "FAIL" );
    #endif

    return ret == HAL_SD_CARD_TRANSFER;
}

/******************************************************************************
 * Dump the SDMMC geometry
 *****************************************************************************/
void SDMMC_DumpGeometry(const HW_DeviceType *self)
{
    SdmmcHandleT *myHandle = Sdmmc_GetMyHandle(self);
    DEBUG_PRINTF("%s: ", self->devName);
    if ( myHandle->bIsInitialized ) {
        uint32_t blocks_per_MB = ( 1024 << 10 ) / myHandle->uSectorSize;
        DEBUG_PRINTF(" %d Sectors of size %d = %d MiB, EraseBlockSize=%d\n", myHandle->uNumBlocks, myHandle->uSectorSize, myHandle->uNumBlocks/blocks_per_MB, myHandle->uEraseBlockSize* myHandle->uSectorSize);
    } else {
        DEBUG_PRINTF("card not initialized\n");
    }
}

/******************************************************************************
 * Read data blockwise from SD card, polling mode
 * @param pdata     - pointer to the byte vector the read data will be written to
 *                    caller has to assure, that there is enough room to write
 * @param BlockIdx  - number of first block to read
 * @param BlocksNbr - number of consecutive blocks to read
 * @returns true    - read successful
 * @returns false   - read failed
 * @note Care should be taken when using this fn in an multitreading environment
 *       like FreeRTOS. A RxFifo Overrun or WrFifo underrun is very likely to occur
 *       Use DMA or IT read instead
 *****************************************************************************/
bool SDMMC_ReadBlocks(SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{
   
   uint32_t timeout = SD_READ_TIMEOUT*BlocksNbr;
   myHandle->lastError = HAL_SD_ReadBlocks(&myHandle->halHandle, (uint8_t *)pData, BlockIdx, BlocksNbr, timeout);
   #if DEBUG_SDMMC > 0
       if ( myHandle->lastError != HAL_OK ) {
            DEBUG_PRINTF("SD_ReadBlocks returned %s errorcode 0x%08x\n", DBG_get_hal_errtxt(myHandle->lastError), myHandle->halHandle.ErrorCode);
       }
   #endif
   return myHandle->lastError == HAL_OK;
}

/******************************************************************************
 * Read data blockwise from SD card, DMA mode
 * @param pdata     - pointer to the byte vector the read data will be written to
 *                    caller has to assure, that there is enough room to write
 * @param BlockIdx  - number of first block to read
 * @param BlocksNbr - number of consecutive blocks to read
 * @returns true    - DMA read initiated successful
 * @returns false   - DMA read could not be initiated 
 *****************************************************************************/
bool SDMMC_ReadBlocksDMA(SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{
   /* Set "performing read" flag */
   myHandle->bPerformingRead   = 1;
   myHandle->AsyncErrorCode    = 0;
 
   myHandle->lastError = HAL_SD_ReadBlocks_DMA(&myHandle->halHandle, (uint8_t *)pData, BlockIdx, BlocksNbr);
   #if DEBUG_SDMMC > 0
       if ( myHandle->lastError != HAL_OK ) {
            DEBUG_PRINTF("SD_ReadBlocksDMA returned %s errorcode 0x%08x\n", DBG_get_hal_errtxt(myHandle->lastError), myHandle->halHandle.ErrorCode);
       }
   #endif

   return myHandle->lastError == HAL_OK;
}


/******************************************************************************
 * Write data blockwise to SD card, polling mode
 * @param pdata     - pointer to the byte vector that will be written
 *                    caller has to assure, that vector size is sufficient
 * @param BlockIdx  - number of first block to write to
 * @param BlocksNbr - number of consecutive blocks to be written
 * @returns true    - write successful
 * @returns false   - write failed
 *****************************************************************************/
bool SDMMC_WriteBlocks(SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{

   uint32_t timeout = SD_WRITE_TIMEOUT*BlocksNbr;
   myHandle->lastError = HAL_SD_WriteBlocks(&myHandle->halHandle, (uint8_t *)pData, BlockIdx, BlocksNbr, timeout);

   #if DEBUG_SDMMC > 0
       if ( myHandle->lastError != HAL_OK ) {
            DEBUG_PRINTF("SD_WriteBlocks returned %s errorcode 0x%08x\n", DBG_get_hal_errtxt(myHandle->lastError), myHandle->halHandle.ErrorCode);
       }
   #endif
   return myHandle->lastError == HAL_OK;
}

/******************************************************************************
 * Write data blockwise to SD card, DMA mode
 * @param pdata     - pointer to the byte vector that will be written
 *                    caller has to assure, that vector size is sufficient
 * @param BlockIdx  - number of first block to write to
 * @param BlocksNbr - number of consecutive blocks to be written
 * @returns true    - write successful
 * @returns false   - write failed
 *****************************************************************************/
bool SDMMC_WriteBlocksDMA(SdmmcHandleT *myHandle, uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{
   /* Set "performing write" flag */
   myHandle->bPerformingWrite  = 1;
   myHandle->AsyncErrorCode    = 0;

   myHandle->lastError = HAL_SD_WriteBlocks_DMA(&myHandle->halHandle, (uint8_t *)pData, BlockIdx, BlocksNbr);

   #if DEBUG_SDMMC > 0
       if ( myHandle->lastError != HAL_OK ) {
            DEBUG_PRINTF("SD_WriteBlocksDMA returned %s errorcode 0x%08x\n", DBG_get_hal_errtxt(myHandle->lastError), myHandle->halHandle.ErrorCode);
       }
   #endif
   return myHandle->lastError == HAL_OK;
}

/******************************************************************************
 * HAL SDMMC interrupt handlers
 *****************************************************************************/

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  SdmmcHandleT* hnd= GetMyHandleFromHalHandle(hsd);
  assert(hnd);
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd  SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  SdmmcHandleT* hnd= GetMyHandleFromHalHandle(hsd);
  assert(hnd);
  hnd->bPerformingWrite = 0;
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd  SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  SdmmcHandleT* hnd= GetMyHandleFromHalHandle(hsd);
  assert(hnd);
  hnd->bPerformingRead = 0;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  SdmmcHandleT* hnd= GetMyHandleFromHalHandle(hsd);
  assert(hnd);
  hnd->bPerformingRead  = 0;
  hnd->bPerformingWrite = 0;
  hnd->AsyncErrorCode   = hnd->halHandle.ErrorCode;

}


/******************************************************************************
 * Dump SD Card data
 *****************************************************************************/
#if DEBUG_SDMMC > 0
    struct mmc_cid {
            uint8_t 		manfid;
            char			prod_name[8];
            uint32_t		serial;
            uint16_t		oemid;
            uint16_t		year;
            unsigned char		hwrev;
            unsigned char		fwrev;
            unsigned char		month;
    };

    struct mmc_csd {
            unsigned char		mmca_vsn;
            unsigned short		cmdclass;
            unsigned short		tacc_clks;
            unsigned int		tacc_ns;
            unsigned int		r2w_factor;
            unsigned int		max_dtr;
            unsigned int		read_blkbits;
            unsigned int		write_blkbits;
            unsigned int		capacity;
            unsigned int		read_partial:1,
                                    read_misalign:1,
                                    write_partial:1,
                                    write_misalign:1;
    };

    struct sd_scr {
            unsigned char		sda_vsn;
            unsigned char		bus_widths;
    #define SD_SCR_BUS_WIDTH_1	(1<<0)
    #define SD_SCR_BUS_WIDTH_4	(1<<2)
    };

    static const unsigned int tran_exp[] = {
            10000,		100000,		1000000,	10000000,
            0,		0,		0,		0
    };

    static const unsigned char tran_mant[] = {
            0,	10,	12,	13,	15,	20,	25,	30,
            35,	40,	45,	50,	55,	60,	70,	80,
    };

    static const unsigned int tacc_exp[] = {
            1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
    };

    static const unsigned int tacc_mant[] = {
            0,	10,	12,	13,	15,	20,	25,	30,
            35,	40,	45,	50,	55,	60,	70,	80,
    };

    #define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})



    /*
     * Given the decoded CSD structure, decode the raw CID to our CID structure.
     */
    #define u32    uint32_t
    static void mmc_decode_cid(u32 *resp, struct mmc_cid *cid)
    {

            memset(cid, 0, sizeof(struct mmc_cid));

            /*
             * SD doesn't currently have a version field so we will
             * have to assume we can parse this.
             */
            cid->manfid                     = UNSTUFF_BITS(resp, 120, 8);
            cid->oemid			= UNSTUFF_BITS(resp, 104, 16);
            cid->prod_name[0]		= UNSTUFF_BITS(resp, 96, 8);
            cid->prod_name[1]		= UNSTUFF_BITS(resp, 88, 8);
            cid->prod_name[2]		= UNSTUFF_BITS(resp, 80, 8);
            cid->prod_name[3]		= UNSTUFF_BITS(resp, 72, 8);
            cid->prod_name[4]		= UNSTUFF_BITS(resp, 64, 8);
            cid->prod_name[5]               = '\0';
            cid->hwrev			= UNSTUFF_BITS(resp, 60, 4);
            cid->fwrev			= UNSTUFF_BITS(resp, 56, 4);
            cid->serial		        = UNSTUFF_BITS(resp, 24, 32);
            cid->year			= UNSTUFF_BITS(resp, 12, 8);
            cid->month			= UNSTUFF_BITS(resp, 8, 4);

            cid->year += 2000; /* SD cards year offset */
    }

    /*
     * Given a 128-bit response, decode to our card CSD structure.
     */
    static int mmc_decode_csd(u32 *resp, struct mmc_csd *csd)
    {
	unsigned int e, m, csd_struct;

	csd_struct = UNSTUFF_BITS(resp, 126, 2);

	switch (csd_struct) {
	case 0:
		m = UNSTUFF_BITS(resp, 115, 4);
		e = UNSTUFF_BITS(resp, 112, 3);
		csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
		csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		e = UNSTUFF_BITS(resp, 47, 3);
		m = UNSTUFF_BITS(resp, 62, 12);
		csd->capacity	  = (1 + m) << (e + 2);

		csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
		csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
		csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
		csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
		csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
		csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
		csd->write_partial = UNSTUFF_BITS(resp, 21, 1);
		break;
	case 1:
		/*
		 * This is a block-addressed SDHC card. Most
		 * interesting fields are unused and have fixed
		 * values. To avoid getting tripped by buggy cards,
		 * we assume those fixed values ourselves.
		 */
		// RHB todo mmc_card_set_blockaddr(card);

		csd->tacc_ns	 = 0; /* Unused */
		csd->tacc_clks	 = 0; /* Unused */

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		m = UNSTUFF_BITS(resp, 48, 22);
		csd->capacity     = (1 + m) << 10;

		csd->read_blkbits = 9;
		csd->read_partial = 0;
		csd->write_misalign = 0;
		csd->read_misalign = 0;
		csd->r2w_factor = 4; /* Unused */
		csd->write_blkbits = 9;
		csd->write_partial = 0;
		break;
	default:
		DEBUG_PRINTF("unrecognised CSD structure version %d\n",
			csd_struct);
		return 0;
	}

	return 1;
    }

    /*
     * Given a 64-bit response, decode to our card SCR structure.
     */
    static int mmc_decode_scr(u32 *raw_scr, struct sd_scr *scr)
    {
            unsigned int scr_struct;
            u32 resp[4];

            resp[3] = raw_scr[1];
            resp[2] = raw_scr[0];

            scr_struct = UNSTUFF_BITS(resp, 60, 4);
            if (scr_struct != 0) {
                    LOG_ERROR("unrecognised SCR structure version %d\n", scr_struct);
                    return 0;
            }

            scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
            scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);

            return 1;
    }

    uint32_t power(uint32_t base, uint32_t exp) {
        uint32_t i, result = 1;
        for (i = 0; i < exp; i++)
            result *= base;
        return result;
     }

    static const uint8_t m_values[16] = 
        { 10, 12, 13, 15, 20, 25, 30, 35,
          35, 40, 45, 50, 55, 60, 70, 80 };
    static const char * const e_unit[3] = 
        { "ns", "us", "ms" };

    const char *decode_taac( char *retbuf, size_t buflen,uint8_t taac ) {
        uint8_t exp = taac & 0b111;
        uint8_t mant = taac >> 3;
        uint32_t val = (uint32_t)m_values[mant] * power(10, exp) / 10;
        uint8_t unit=0;
        if ( val > 10000 )  {
            val /= 1000; unit++;
            if ( val > 10000 )  {
                val /= 1000; unit++;
            }
        }
        snprintf(retbuf, buflen, "%d%s", val, e_unit[unit]);
        retbuf[buflen-1] ='\0';

        return retbuf;
    }   
    /**************************************************************************
     * be sure to call this function only, if card is currently in "trans" state
     *************************************************************************/
    void    SDMMC_DebugInfo         (const HW_DeviceType *self)

    {
        SdmmcHandleT *h = Sdmmc_GetMyHandle(self);
    
        DEBUG_PRINTF("Dumping Data of %s\n", self->devName);
        SDMMC_DumpGeometry(self);

        SDMMC_DumpStatusRegister(self);
        DEBUG_PRINTF("------------------------------\n");

        SDMMC_DumpCardStatus(self);       
        DEBUG_PRINTF("------------------------------\n");

        SDMMC_DumpCid ( self );
        DEBUG_PRINTF("------------------------------\n");

        SDMMC_DumpCsd ( self );
        DEBUG_PRINTF("------------------------------\n");

    }

    /**************************************************************************
     * SDMMC_DumpStatusRegister will issue CMD13, which has no impact on the 
     * card state ( mostly "transfer" - before and afterwards )
     *************************************************************************/
    void SDMMC_DumpStatusRegister(const HW_DeviceType *self)
    {
        SdmmcHandleT *h = Sdmmc_GetMyHandle(self);
        uint32_t status;

        DEBUG_PRINTF("Dumping status register of %s\n", self->devName);

        HAL_StatusTypeDef ret = SD_SendStatus(&h->halHandle, &status);
        if ( ret != HAL_OK ) {
            DEBUG_PRINTF("SD_SendStatus returned with error code %s\n", DBG_get_hal_errtxt(ret));
            return;
        } 

        DEBUG_PRINTF("Error bits are only dumped when set\n");
        DBG_setIndentAbs(2);
        DBG_setPadLen(20);
        DBG_dump_uint32_hex("Status raw", status);
        DBG_dump_onlyifset("out of range", status, 1<<31 );
        DBG_dump_onlyifset("Address err", status, 1<<30 );
        DBG_dump_onlyifset("Block len err", status, 1<<29 );
        DBG_dump_onlyifset("Erase seq err", status, 1<<28 );
        DBG_dump_onlyifset("Erase param err", status, 1<<27 );
        DBG_dump_onlyifset("WP violation", status, 1<<26 );
        DBG_dump_onlyifset("CMD crc err", status, 1<<23 );
        DBG_dump_onlyifset("Illegal CMD", status, 1<<22 );
        DBG_dump_onlyifset("general err", status, 1<<19 );
        DBG_dump_onlyifset("CSD overwr err", status, 1<<16 );
        DBG_dump_setresetvalue("WP erase skip", status, 1<<16 );
        DBG_dump_setresetvalue("Erase reset", status, 1<<16 );
        uint32_t state = ( status >> 9 ) & 0x0f;
        DBG_dump_textvalue("Status", get_state_str(state));
        DBG_dump_bitvalue("Rdy for data", status, 1<<8 );
    }

    const char * const  class_num[] = { "0",  "2",  "4",  "6", "10", };
    static const char *get_class( uint32_t raw_class)
    {
        if ( raw_class < sizeof(class_num)/sizeof(int8_t) ) 
            return class_num[raw_class];
        else
            return  "????";
    }

    const char * const au_size[] = { "Undef",  "16k",  "32k",  "64k",  "128k", "256k", "512k", "1MB",
                                     "2MB",    "4MB",  "8MB",  "12MB", "16MB", "24MB", "32MB", "64MB"};
    static const char *get_au_size( uint32_t raw_size)
    {
        if ( raw_size < sizeof(au_size)/sizeof(char *) ) 
            return au_size[raw_size];
        else
            return  "????";
    }

    void SDMMC_DumpCardStatus(const HW_DeviceType *self)
    {
        SdmmcHandleT *h = Sdmmc_GetMyHandle(self);
        HAL_SD_CardStatusTypeDef status;

        DEBUG_PRINTF("Dumping Card Status of %s\n", self->devName);

        HAL_StatusTypeDef ret =  HAL_SD_GetCardStatus(&h->halHandle, &status);
        if ( ret != HAL_OK ) {
            DEBUG_PRINTF("HAL_SD_GetCardStatus returned with error code %s\n", DBG_get_hal_errtxt(ret));
            return;
        } 

        DBG_setIndentAbs(2);
        DBG_setPadLen(20);
        DBG_dump_textvalue("Databus Width", status.DataBusWidth==0 ? "1" : status.DataBusWidth==2 ? "4" : "????" );
        DBG_dump_bitvalue ("Secured mode", status.SecuredMode, status.SecuredMode);
        DBG_dump_textvalue("Card type", status.CardType==0 ? "SDMMC" : status.CardType==1 ? "SDROM" : status.CardType==2 ? "OTP" : "????");
        DBG_dump_uint32_hex("Prot. area size", status.ProtectedAreaSize);
        DBG_dump_textvalue("Speed class", get_class(status.SpeedClass));
        DBG_dump_number   ("Performance move", status.PerformanceMove);
        DBG_dump_textvalue("Alloc. unit size(AU)", get_au_size(status.AllocationUnitSize));
        DBG_dump_number   ("Erase size[AU]", status.EraseSize);
        DBG_dump_number   ("Erase timeout[s]", status.EraseTimeout);
        DBG_dump_number   ("Erase offset[s]", status.EraseOffset);
        DBG_dump_number   ("UHS speed grade", status.UhsSpeedGrade);
        DBG_dump_number   ("UHS AU size", status.UhsAllocationUnitSize);
        DBG_dump_number   ("Video speed class", status.VideoSpeedClass);
    }

    void SDMMC_DumpCid(const HW_DeviceType *self)
    {
        SdmmcHandleT    *h = Sdmmc_GetMyHandle(self);
        struct mmc_cid  cid;
        char workbuf[20];

        /* decode raw csd and cid data to human readable format */
        mmc_decode_cid(h->halHandle.CID, &cid);

        DEBUG_PRINTF("Dumping CID of %s\n", self->devName);
        DBG_setIndentAbs(2);
        DBG_setPadLen(20);
        DBG_dump_uint8_hex("ManufacturerID", cid.manfid);
        DBG_dump_uint16_hex("OEM appl. ID", cid.oemid);
        DBG_dump_textvalue("Prod.name", cid.prod_name);
        sprintf(workbuf, "%02d/%04d", cid.month, cid.year);
        DBG_dump_textvalue("Mfg.date", workbuf);
        sprintf(workbuf, "%02d/%02d", cid.hwrev, cid.fwrev);
        DBG_dump_textvalue("HW/FW", workbuf);
        DBG_dump_uint32_hex("Prod.SN",cid.serial);
    }

    void SDMMC_DumpCsd(const HW_DeviceType *self)
    {
        SdmmcHandleT   *h = Sdmmc_GetMyHandle(self);
        struct mmc_csd csd;

        /* decode raw csd and cid data to human readable format */
        mmc_decode_csd(h->halHandle.CSD,&csd);

        DEBUG_PRINTF("Dumping CSD of %s\n", self->devName);
        DBG_setIndentAbs(2);
        DBG_setPadLen(20);
        DBG_dump_number ("MMCA Version", csd.mmca_vsn );
        DBG_dump_number("cmdclass", csd.cmdclass );
        DBG_dump_number("TAAC clks", csd.tacc_clks );
        DBG_dump_number("TAAC ns", csd.tacc_ns );
        DBG_dump_number("R2W factor", csd.r2w_factor );
        DBG_dump_number("max DTR", csd.max_dtr );
        DBG_dump_number("Rd blkbits", csd.read_blkbits );
        DBG_dump_number("Wr blkbits", csd.write_blkbits );
        DBG_dump_number("Capacity", csd.capacity );
        DBG_dump_bitvalue("Rd partial", csd.read_partial,csd.read_partial );
        DBG_dump_bitvalue("Rd misalign", csd.read_misalign, csd.read_misalign );
        DBG_dump_bitvalue("Wr partial", csd.write_partial, csd.write_partial );
        DBG_dump_bitvalue("Wr misalign", csd.write_misalign, csd.write_misalign );
    }
#else
    void    SDMMC_DebugInfo         (const HW_DeviceType *self) {UNUSED(self);}
    void    SDMMC_DumpCardStatus    (const HW_DeviceType *self) {UNUSED(self);}
    void    SDMMC_DumpCid           (const HW_DeviceType *self) {UNUSED(self);}
    void    SDMMC_DumpCsd           (const HW_DeviceType *self) {UNUSED(self);}
#endif

#endif /* if USE_SDMMC > 0 */


/**
  * @}
  */


