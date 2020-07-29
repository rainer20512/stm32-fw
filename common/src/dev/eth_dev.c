/**
 ******************************************************************************
 * @file    ETH_dev.c
 * @author  Rainer
 * @brief   ETH device functions. 
 */

/** @addtogroup ETH Device Functions
  * @{
  */

#include "config/devices_config.h"

#if USE_ETH > 0 

#if DEBUG_MODE > 0
    #define DEBUG_ETH               1
    #define DEBUG_MIN_ENABLE_DUMP   2
#endif

#include "error.h"
#include "dev/hw_device.h"
#include "dev/eth_dev.h"

#include "system/hw_util.h"
#include "config/eth_config.h"
#include "debug_helper.h"




/* forward declarations ------------------------------------------------------------*/
bool ETH_InitDev(const HW_DeviceType *self);
void ETH_DeInitDev(const HW_DeviceType *self);



/* Debugging functions ------------------------------------------------------*/
#define PAD_LEN     18

#if DEBUG_ETH >= DEBUG_MIN_ENABLE_DUMP

#endif // DEBUG_ETH > xx

void Eth_DumpEth ( ETH_TypeDef *hEth )
{
  DEBUG_PUTS("ETH configuration -------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  #if DEBUG_ETH >= DEBUG_MIN_ENABLE_DUMP
      DBG_setPadLen(PAD_LEN);
  #else
    UNUSED(hEth);
    DBG_printf_indent("Disabled.\n");
  #endif
  DBG_setIndentAbs(oldIndent);
}

 
/* Private or driver functions ------------------------------------------------------*/

/******************************************************************************
 * Clear the entire EthHandleT structure
 *****************************************************************************/
static void Eth_ResetMyHandle ( EthHandleT *handle ) 
{
    memset(handle, 0, sizeof(EthHandleT) );
}


static void Eth_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Configure CAN_RX and CAN_TX pins */  

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    GpioAFInitAll(GetDevIdx(self), self->devGpioAF, &GPIO_InitStruct);

    /* Enable Ethernet clocks - there is more than one clock, so we use HAL here */
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
}

static void Eth_GPIO_DeInit(const HW_DeviceType *self)
{
    /* Disable Ethernet clocks - there is more than one clock, so we use HAL here */
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();
    __HAL_RCC_ETH1MAC_CLK_DISABLE();

    /* Disable GPIO Pins */
    GpioAFDeInitAll(GetDevIdx(self), self->devGpioAF);
}


/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, whenever CAN is in SLEEP or INIT Mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool ETH_FrqChange(const HW_DeviceType *self)
{
    EthHandleT* me = (EthHandleT*)self->devData;
    /* Minimum HCLK frequeny is 20 MHZ */
    if ( HAL_RCC_GetHCLKFreq() < 20000000 ) return false;
    
    HAL_ETH_SetMDIOClockRange(me); 
    return true;
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Can Device initialization: Reset handle, init GPIO pins and interrupts,
 * set default baudrate and normal bus mode, CAN will remain in sleep mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool ETH_InitDev(const HW_DeviceType *self)
{
    EthHandleT* me    = (EthHandleT*)self->devData;
    ETH_TypeDef *inst = (ETH_TypeDef *)self->devBase;

    /* Initialize my handle to 'fresh' */
    Eth_ResetMyHandle(me);

    /* In the embedded Handle only set the Instance member */
    me->Instance = inst;

    /* Init GPIO and Clocks */
    Eth_GPIO_Init(self);

    /* Configure the NVIC, enable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);

  return true;
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Reset CAN peripheral, deassign interrupts and DeInit GPIO Pins
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
void ETH_DeInitDev(const HW_DeviceType *self)
{
    /*Reset peripherals */
    HW_Reset((ETH_TypeDef *)self->devBase );

    Eth_GPIO_DeInit(self);

    /* Disable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
}

/* Static configurations ---------------------------------------------------------*/

#if defined(ETH) && defined(USE_ETH)
    EthHandleT EthHandle;

    static const HW_GpioList_AF gpio_eth = {
        #if defined(ETH_USE_RMII)
             .gpio = { ETH_REF_CLK, ETH_RX0, ETH_RX1, ETH_CRS_DV, ETH_MDC, ETH_MDIO, ETH_TX0, ETH_TX1, ETH_TX_EN },
             .num = 9,
        #elif defined ( ETH_USE_MII )
             .gpio = { ETH_COL, ETH_CRS, ETH_RX_CLK, ETH_RX0, ETH_RX1, ETH_RX2, ETH_RX3, ETH_RX_DV, ETH_RX_ER, 
                       ETH_TX_CLK, ETH_TX0, ETH_TX1, ETH_TX2, ETH_TX3, ETH_TX_EN, ETH_TX_ER, ETH_MDC, ETH_MDIO  },
             .num = 18,
        #else
            #error "No Ethernet RMII config for selected board"
        #endif
    };

    #ifdef ETH_USE_IRQ
        static const HW_IrqList irq_eth = {
            .num = 2,
            .irq = { ETH_IRQ, ETH_WKUP_IRQ },
        };
    #endif

    const HW_DeviceType HW_ETH = {
        .devName        = "ETH",
        .devBase        = ETH,
        .devGpioAF      = &gpio_eth,
        .devGpioIO      = NULL,
        .devType        =  HW_DEVICE_ETH,
        .devData        = &EthHandle,
        .devIrqList     = 
        #if defined(ETH_USE_IRQ)
            &irq_eth,
        #else
            NULL,
        #endif
        /* No DMA for ETH, has its own DMA */
        .devDmaTx = NULL,
        .devDmaRx = NULL,
        .Init           = ETH_InitDev,
        .DeInit         = ETH_DeInitDev,
        .OnFrqChange    = ETH_FrqChange,
        .AllowStop      = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif



#endif // USE_ETH



