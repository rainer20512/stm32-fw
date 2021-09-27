/*
 ******************************************************************************
 * @file    hw_util.h
 * @author  Rainer
 * @brief   Utility Functions for Hardware access
 *
 ******************************************************************************
 */

#include "config/config.h"
#include "config/devices_config.h"
#include "system/hw_util.h"
#include "debug_helper.h"

/* Private define -----------------------------------------------------------------------*/

/* Private macro ------------------------------------------------------------------------*/
#define TMR_DEBUG_STOP()                 (DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_LPTIM1_STOP)
#define RTC_DEBUG_STOP()                 (DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_RTC_STOP )



/* Public functions ---------------------------------------------------------------------*/


/* Private typedef ----------------------------------------------------------------------*/
typedef struct {
    void *   HW_TypeDef;
    __IO     uint32_t *ClkReg;
    uint8_t  ClkBitpos;
    __IO     uint32_t *ResetReg;
    uint8_t  ResetBitpos;
    uint16_t key;   
} GPIO_RegisterBitStructType;

/* Private variables --------------------------------------------------------------------*/
static const GPIO_RegisterBitStructType GPIO_ClockBits[] = {
/* DMA -------------------------------------------------------------------------------------------------------------------- */
#ifdef DMA1
   { DMA1, &RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_Pos,       &RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST_Pos,    COMBINE('D', 1) },
#endif
#ifdef DMA2
   { DMA2, &RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Pos,       &RCC->AHB1RSTR, RCC_AHB1RSTR_DMA2RST_Pos,    COMBINE('D', 2) },
#endif
/**** 003 ***/
#ifdef DMAMUX1
   { DMAMUX1, &RCC->AHB1ENR, RCC_AHB1ENR_DMAMUX1EN_Pos, &RCC->AHB1RSTR, RCC_AHB1RSTR_DMAMUX1RST_Pos, COMBINE('D', 5) },
#endif

/* GPIO ------------------------------------------------------------------------------------------------------------------- */
#ifdef GPIOA
   { GPIOA, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOAEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST_Pos,  COMBINE('G', 'A') },
#endif
#ifdef GPIOB
   { GPIOB, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOBEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOBRST_Pos,  COMBINE('G', 'B') },
#endif
#ifdef GPIOC
   { GPIOC, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOCEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOCRST_Pos,  COMBINE('G', 'C') },
#endif
#ifdef GPIOD
   { GPIOD, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIODEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIODRST_Pos,  COMBINE('G', 'D') },
#endif
#ifdef GPIOE
   { GPIOE, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOEEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOERST_Pos,  COMBINE('G', 'E') },
#endif
#ifdef GPIOF
   { GPIOF, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOFEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOFRST_Pos,  COMBINE('G', 'F') },
#endif
#ifdef GPIOG
   { GPIOG, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOGEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOGRST_Pos,  COMBINE('G', 'G') },
#endif
#ifdef GPIOH
   { GPIOH, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOHEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOHRST_Pos,  COMBINE('G', 'H') },
#endif
#ifdef GPIOI
   { GPIOI, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOIEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOIRST_Pos,  COMBINE('G', 'I') },
#endif
#ifdef GPIOJ
   { GPIOJ, &RCC->AHB2ENR,  RCC_AHB2ENR_GPIOJEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOJRST_Pos,  COMBINE('G', 'J') },
#endif

/* U(s)art ----------------------------------------------------------------------------------------------------------------- */
#if defined(USART1) && defined(USE_USART1)
   { USART1, &RCC->APB2ENR, RCC_APB2ENR_USART1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Pos, COMBINE('C', 1) },
#endif
#if defined(USART2) && defined(USE_USART2)
   { USART2, &RCC->APB1ENR1, RCC_APB1ENR1_USART2EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_USART2RST_Pos, COMBINE('C', 2) },
#endif
#if defined(USART3) && defined(USE_USART3)
   { USART3, &RCC->APB1ENR1, RCC_APB1ENR1_USART3EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_USART3RST_Pos, COMBINE('C', 3) },
#endif
#if defined(UART4) && defined(USE_UART4)
   { UART4, &RCC->APB1ENR1, RCC_APB1ENR1_UART4EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_UART4RST_Pos, COMBINE('C', 4) },
#endif
#if defined(UART5) && defined(USE_UART5)
   { UART5, &RCC->APB1ENR1, RCC_APB1ENR1_UART5EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_UART5RST_Pos, COMBINE('C', 5) },
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
   { LPUART1, &RCC->APB1ENR2, RCC_APB1ENR2_LPUART1EN_Pos, &RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST_Pos, COMBINE('C', 9) },
#endif

/* SPI --------------------------------------------------------------------------------------------------------------------- */
#if defined(SPI1) && defined ( USE_SPI1 )
   { SPI1, &RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Pos, COMBINE('S', 1) },
#endif
#if defined(SPI2) && defined ( USE_SPI2 )
   { SPI2, &RCC->APB1ENR1, RCC_APB1ENR1_SPI2EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_SPI2RST_Pos, COMBINE('S', 2) },
#endif
#if defined(SPI3) && defined ( USE_SPI3 )
   { SPI3, &RCC->APB1ENR1, RCC_APB1ENR1_SPI3EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_SPI3RST_Pos, COMBINE('S', 3) },
#endif

/* I2C --------------------------------------------------------------------------------------------------------------------- */
#if defined(I2C1) && defined(USE_I2C1)
   { I2C1, &RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C1RST_Pos, COMBINE('I', 1) },
#endif
#if defined(I2C2) && defined(USE_I2C2)
   { I2C2, &RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C2RST_Pos, COMBINE('I', 2) },
#endif
#if defined(I2C3) && defined(USE_I2C3)
   { I2C3, &RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_I2C3RST_Pos, COMBINE('I', 3) },
#endif
#if defined(I2C4) && defined(USE_I2C4)
   { I2C4, &RCC->APB1ENR2, RCC_APB1ENR2_I2C4EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR2_I2C4RST_Pos, COMBINE('I', 3) },
#endif

/* ADC     - All ADC share ONE clock enable and ONE reset bit  -------------------------------------------------------------- */
#if defined(ADC1) && defined(USE_ADC1)
   { ADC1, &RCC->AHB2ENR, RCC_AHB2ENR_ADCEN_Pos,   &RCC->AHB2RSTR, RCC_AHB2RSTR_ADCRST_Pos,    COMBINE('A', 1) },
#endif
#if defined(ADC2) && defined(USE_ADC2)
   { ADC2, &RCC->AHB2ENR, RCC_AHB2ENR_ADCEN_Pos,   &RCC->AHB2RSTR, RCC_AHB2RSTR_ADCRST_Pos,    COMBINE('A', 2) },
#endif
#if defined(ADC3) && defined(USE_ADC3)
   { ADC3, &RCC->AHB2ENR, RCC_AHB2ENR_ADCEN_Pos,   &RCC->AHB2RSTR, RCC_AHB2RSTR_ADCRST_Pos,    COMBINE('A', 3) },
#endif
/* Alle Timer --------------------------------------------------------------------------------------------------------------- */
#if defined(TIM1) 
   { TIM1, &RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST_Pos, COMBINE('T', 1) },
#endif
#if defined(TIM2) 
   { TIM2, &RCC->APB1ENR1, RCC_APB1ENR1_TIM2EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM2RST_Pos, COMBINE('T', 2) },
#endif
#if defined(TIM3) 
   { TIM3, &RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM3RST_Pos, COMBINE('T', 3) },
#endif
#if defined(TIM4) 
   { TIM4, &RCC->APB1ENR1, RCC_APB1ENR1_TIM4EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM4RST_Pos, COMBINE('T', 4) },
#endif
#if defined(TIM5) 
   { TIM5, &RCC->APB1ENR1, RCC_APB1ENR1_TIM5EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM5RST_Pos, COMBINE('T', 5) },
#endif
#if defined(TIM6) 
   { TIM6, &RCC->APB1ENR1, RCC_APB1ENR1_TIM6EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM6RST_Pos, COMBINE('T', 6) },
#endif
#if defined(TIM7) 
   { TIM7, &RCC->APB1ENR1, RCC_APB1ENR1_TIM7EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_TIM7RST_Pos, COMBINE('T', 7) },
#endif
#if defined(TIM8) 
   { TIM8, &RCC->APB2ENR, RCC_APB2ENR_TIM8EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM8RST_Pos, COMBINE('T', 8) },
#endif
#if defined(TIM15) 
   { TIM15, &RCC->APB2ENR, RCC_APB2ENR_TIM15EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM15RST_Pos, COMBINE('T', 15) },
#endif
#if defined(TIM16) 
   { TIM16, &RCC->APB2ENR, RCC_APB2ENR_TIM16EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM16RST_Pos, COMBINE('T', 16) },
#endif
#if defined(TIM17) 
   { TIM17, &RCC->APB2ENR, RCC_APB2ENR_TIM17EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM17RST_Pos, COMBINE('T', 17) },
#endif
#if defined(LPTIM1) 
   { LPTIM1, &RCC->APB1ENR1, RCC_APB1ENR1_LPTIM1EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_LPTIM1RST_Pos, COMBINE('T', 20) },
#endif
/* LPTIM2 not coded */
/* QUADSPI/OCTOSPI  ------------------------------------------------------------------------------------------------------- */
#if defined(QUADSPI) && defined(USE_QSPI1) 
   { QUADSPI, &RCC->AHB3ENR, RCC_AHB3ENR_QSPIEN_Pos, &RCC->AHB3RSTR, RCC_AHB3RSTR_QSPIRST_Pos, COMBINE('Q', 1) },
#endif
#if defined(OCTOSPI1) && defined(USE_OSPI1) 
   { OCTOSPI1, &RCC->AHB3ENR, RCC_AHB3ENR_OSPI1EN_Pos, &RCC->AHB3RSTR, RCC_AHB3RSTR_OSPI1RST_Pos, COMBINE('O', 1) },
#endif
#if defined(OCTOSPI2) && defined(USE_OSPI2) 
   { OCTOSPI2, &RCC->AHB3ENR, RCC_AHB3ENR_OSPI2EN_Pos, &RCC->AHB3RSTR, RCC_AHB3RSTR_OSPI2RST_Pos, COMBINE('O', 2) },
#endif
/* CAN1  ------------------------------------------------------------------------------------------------------------------ */
#if defined(CAN1) && defined(USE_CAN1) 
   { CAN1, &RCC->APB1ENR1, RCC_APB1ENR1_CAN1EN_Pos, &RCC->APB1RSTR1, RCC_APB1RSTR1_CAN1RST_Pos, COMBINE('C', 1) },
#endif
/* USB OTG  --------------------------------------------------------------------------------------------------------------- */
#if defined(USB_OTG_FS) && defined(USE_USB) 
   { USB_OTG_FS, &RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN_Pos, &RCC->AHB2RSTR, RCC_AHB2RSTR_OTGFSRST_Pos, COMBINE('U', 1) },
#endif
#if defined(FMC_Bank1_R) && USE_FMC > 0 
   { FMC_Bank1_R, &RCC->AHB3ENR, RCC_AHB3ENR_FMCEN_Pos, &RCC->AHB3RSTR, RCC_AHB3RSTR_FMCRST_Pos, COMBINE('F', 1) },
#endif

};

/* Public functions ---------------------------------------------------------------------*/
void* HW_GetHW ( uint16_t key )
{
    /* Letter to UC */
    if ( key >> 8 == 'G' ) key &= ~0x0020;

    for ( uint32_t i = 0; i < sizeof(GPIO_ClockBits); i++ ) {
      if ( GPIO_ClockBits[i].key == key  ) 
        return GPIO_ClockBits[i].HW_TypeDef;
    }

    #if DEBUG_MODE > 0
        DEBUG_PRINTF("Illegal hardware key %c%c\n", (char)(key >>8), (char)key);
    #endif
    return NULL;
}

/*
 ***********************************************************************
 * Will return the GPIO_TypeDef of the passed portletter in gp
 * and the clock status of that port as return value
 * If the portletter is illegal, NULL will be returned in gp
 **********************************************************************/
bool HW_GetHWClockStatus ( void *hw )
{
    const GPIO_RegisterBitStructType *p = GPIO_ClockBits;

    for ( uint32_t i = 0; i < sizeof(GPIO_ClockBits); i++ ) {
      if ( p->HW_TypeDef == hw ) 
        return READ_BIT(*(p->ClkReg), 1UL << p->ClkBitpos ) != 0;
      p++;
    }

    Error_Handler(__FILE__, __LINE__); 
    return NULL;
}

/*
 ***********************************************************************
 * Will switch the hardware port clock on or off
 * @param  hw  - base address of peripheral
 * @param  bOn - if true, switch clock on, otherwise off
 **********************************************************************/
void HW_SetHWClock( void *hw, bool bOn )
{
    __IO uint32_t tmpreg;
    const GPIO_RegisterBitStructType *p = GPIO_ClockBits;

    for ( uint32_t i = 0; i < sizeof(GPIO_ClockBits); i++ ) {
      if ( p->HW_TypeDef == hw ) {
        if ( bOn )
            SET_BIT(*(p->ClkReg), 1UL << p->ClkBitpos);
        else
            CLEAR_BIT(*(p->ClkReg), 1UL << p->ClkBitpos);
        tmpreg = READ_BIT(*(p->ClkReg), 1UL << p->ClkBitpos);
        UNUSED(tmpreg);
        return;
      }
      p++;
    }

    Error_Handler(__FILE__, __LINE__); 
}

/*
 ***********************************************************************
 * Will Reset the Hardware identified by Hardware addr hw by
 * Setting the corresponding bit in RCC reset register
 * @param  hw     - Base Address of hardware component
 **********************************************************************/
void HW_Reset( void *hw )
{
    const GPIO_RegisterBitStructType *p = GPIO_ClockBits;

    for ( uint32_t i = 0; i < sizeof(GPIO_ClockBits); i++ ) {
      if ( p->HW_TypeDef == hw ) {
        /* Set bit and then clear bit again */
        SET_BIT(*(p->ResetReg), 1UL << p->ResetBitpos);
        CLEAR_BIT(*(p->ResetReg), 1UL << p->ResetBitpos);
        return;
      }
      p++;
    }
    #if DEBUG_MODE > 0
        DEBUG_PRINTF("HW_Reset: Do not know Base Addr %08x\n",(uint32_t)hw);
        Error_Handler(__FILE__, __LINE__); 
    #endif
}

void HW_SetDmaChClock ( const HW_DmaType *tx, const HW_DmaType *rx)
{
    DMA_Channel_TypeDef *use;
    /* At least one handler muist be specified */
    if ( !tx && !rx ) return;

    /* first try rx handler */
    if ( rx ) 
        use = rx->dmaChannel;
    else 
        use = tx->dmaChannel;
    #if !defined(DMA1)
        HW_SetHWClock(DMA1, 1 );
    #else
        /* Get the DMA instance from DMA channel */
        if ((uint32_t)(use) < (uint32_t)(DMA2_Channel1)) {
            /* DMA1 */
            HW_SetHWClock(DMA1, 1 );
        } else  {
            /* DMA2 */
            HW_SetHWClock(DMA2, 1 );
        }
    #endif

    /* Enable Clock for DMAMUX1 in nay case */
    #if defined(DMAMUX1)
        HW_SetHWClock(DMAMUX1, 1 );
    #endif
}

/*
 * Compute the BitBand word adress for a given bit 
 * Can be doneby a macro later, 
 * Initially as subroutine to check parameters 
 * at runtime
 * Bit band address is calculated by the following formula
 * 
 * bit_word_offset = (byte_offset x 32) + (bit_number x 4)
 * bit_word_addr = bit_band_base + bit_word_offset
 *
 * See Ref Man 2.3.
 */

uint32_t *    HW_GetPeriphBitBandAddr     ( __IO uint32_t *periphAddr, uint16_t bit_number )
{
    assert(bit_number < 32 );

    uint32_t byte_offset = (uint32_t)periphAddr - PERIPH_BASE;
    assert(byte_offset < 0x100000);

    uint32_t bit_word_offset = byte_offset*32 + bit_number*4;
    uint32_t bit_word_addr   = PERIPH_BB_BASE + bit_word_offset;
    return ( uint32_t *)bit_word_addr;
}


/* from a mathematical view, ln2(GPIO_pin) is returned */
uint16_t HW_GetLn2 ( uint16_t pwrof2 )
{
    uint16_t ret = 0;

    /* shift GPIO_pin until zero */
    while ( pwrof2 ) {
        pwrof2 >>= 1;
        ret++;
    }

    if ( ret > 0 ) ret--;

    return ret;
}

/*********************************************************
 * Returns the index for a given GPIO_Typedef *
 * i.e. 0 for GPIOA, 1 for GPIOB ...
 ********************************************************/
uint32_t HW_GetGPIOIdx(GPIO_TypeDef *gp) 
{
    uint32_t offset = (uint32_t)gp - AHB2PERIPH_BASE;
    return offset >> 10;
}

/*********************************************************
 * Returns the index for a given GPIO_Typedef *
 * i.e. 'A' for GPIOA, 'B' for GPIOB ...
 ********************************************************/
char HW_GetGPIOLetter(GPIO_TypeDef *gp)
{
    return 'A'+ HW_GetGPIOIdx(gp);
}

/******************************************************************************
 * Disable JTAG Pins after reset
 * We only need SWDIO and SWCLK pins PA13 and PA14
 *****************************************************************************/
void HW_InitJtagDebug(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_13);
    // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    __HAL_RCC_GPIOA_CLK_DISABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
    __HAL_RCC_GPIOB_CLK_DISABLE();

    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* Stop RTC and Timer clock when core is halted during debug */
    RTC_DEBUG_STOP(); TMR_DEBUG_STOP();

}

/******************************************************************************
 * Handling of Device UID, DevID and Revision ID
 * We only need SWDIO pins PA13 and PA14
 *****************************************************************************/
static const char *Get_DeviceName ( uint16_t devID )
{
    switch(devID) {
        case 0x461: return "STM32L496xx/4A6xx";
        case 0x415: return "STM32L475xx/476xx/486xx";
        case 0x470: return "STM32L4Rxxx/STM32L4Sxxx";
        case 0x471: return "STM32L4P5xx/STM32L4Q5xx";
        default:    return "Unknown Device";
    }
}

static const char *Get_PackageName( uint16_t package )
{
    #if defined(STM32L476xx) || defined(STM32L496xx)
        switch(package&0b11111) {
            case 0b00000: return "LQFP64";
            case 0b00010: return "LQFP100";
            case 0b00011: return "UFBGA132";
            case 0b00100: return "LQFP144, WLCSP81 or WLCSP72";
            case 0b10000: return "UFBGA169";
            case 0b10001: return "WLCSP100";
            default: return "Unknown Package";
        }
    #elif defined(STM32L4Sxxx) || defined(STM32L4Rxxx)
        switch(package&0b11111) {
            case 0b00010: return "LQFP100 without DSI";
            case 0b00011: return "UFBGA132 without DSI";
            case 0b00100: 
            case 0b00101: 
            case 0b00110: 
            case 0b00111: return "LQFP144 without DSI";
            case 0b10000: return "UFBGA169 without DSI";
            case 0b10010: return "LQFP100 with DSI";
            case 0b10011: return "UFBGA144 with DSI, WLCSP144 with DSI";
            case 0b10100: return "UFBGA169 with DSI";
            case 0b10101: return "LQFP144 with DSI";
            default: return "Unknown Package";
        }
    #endif
}

char Get_RevisionName( uint16_t devID, uint16_t revID)
{
    char work;
    switch(devID) {
        case 0x461:
        case 0x465:
            switch (revID) {
                case 0x1000 : work = '1'; break;
                case 0x1001 : work = '2'; break;
                case 0x1003 : work = '3'; break;
                case 0x1007 : work = '4'; break;
                default: return '?';
            }
            /* On 496/4A6 devices the revision numbers are revision letters */
            if ( devID == 0x461 )  work = work - '1'+'A';
            break;
        case 0x470:
        case 0x471:
            switch (revID) {
                case 0x1000 : work = 'A'; break;
                case 0x1001 : work = 'Z'; break;
                case 0x1003 : work = 'Y'; break;
                case 0x100f : work = 'W'; break;
                default: return '?';
            }
            break;
        default: return '?';
    } // outer switch 
    return work;
        
}

void HW_ReadID(DeviceIdT *id )
{
    memmove((uint8_t*)id, (uint8_t *)UID_BASE, 12 );
    id->devID = HAL_GetDEVID();
    id->revID = HAL_GetREVID();
    id->flashSize = *(uint16_t*)FLASHSIZE_BASE;
    id->package   = *(uint16_t*)PACKAGE_BASE;
}

bool HW_DumpID(char *cmdline, size_t len, const void * arg )
{
    UNUSED(cmdline);UNUSED(len);UNUSED(arg);
    DeviceIdT MyId;
    DeviceIdT *id = &MyId;
    HW_ReadID(id);

    printf("%s Rev.%c %s\n",Get_DeviceName(id->devID), Get_RevisionName(id->devID, id->revID), Get_PackageName(id->package) );
    printf("Flashsize: %d kB\n",id->flashSize);
    printf("UID=");
    for ( uint8_t i = 0; i < 7; i++ )
        putchar(id->lot[i]);
    printf("Wafer#=%d (X=%d,Y=%d)\n",id->waferNum, id->waferX, id->waferY);

    return true;
}

/******************************************************************************
 * Init an HAL DMA handle to the given parameters. This initialization is
 * hardware specific
 *****************************************************************************/
void HW_DMA_HandleInit(DMA_HandleTypeDef *hdma, const HW_DmaType *dma, void *parent )
{
  hdma->Instance                 = dma->dmaChannel;
  hdma->Parent                   = parent;
  hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma->Init.MemInc              = DMA_MINC_ENABLE;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma->Init.Mode                = DMA_NORMAL;
  hdma->Init.Priority            = dma->dmaPrio;
  hdma->Init.Request             = dma->dmaRequest;
}

