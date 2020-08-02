/*
 ******************************************************************************
 * @file    hw_util.h
 * @author  Rainer
 * @brief   Utility Functions for Hardware access for STM32H7 family
 *          This file is platform specific
 *
 ******************************************************************************
 */

#include "config/config.h"
#include "config/devices_config.h"
#include "system/hw_util.h"
#include "debug_helper.h"

/* Private define -----------------------------------------------------------------------*/

/* Private macro ------------------------------------------------------------------------*/

/* Stop rtc when either M7 or M4 is in debug mode */
#define TMR_DEBUG_STOP()                 do { DBGMCU->APB4FZ1 |=  DBGMCU_APB4FZ1_DBG_RTC;  } while (0)
/* Stop LPTIMERS when either M7 or M4 is in debug mode */
#define RTC_DEBUG_STOP()                 do {DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_LPTIM1;  \
                                             DBGMCU->APB4FZ1  |= DBGMCU_APB4FZ1_DBG_LPTIM2;   \
                                             DBGMCU->APB4FZ1  |= DBGMCU_APB4FZ1_DBG_LPTIM3;   \
                                             DBGMCU->APB4FZ1  |= DBGMCU_APB4FZ1_DBG_LPTIM4;   \
                                             DBGMCU->APB4FZ1  |= DBGMCU_APB4FZ1_DBG_LPTIM5;   \
                                         } while (0)



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
   { DMA1, &RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_Pos,   &RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST_Pos,    COMBINE('D', 1) },
#endif
#ifdef DMA2
   { DMA2, &RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Pos,   &RCC->AHB1RSTR, RCC_AHB1RSTR_DMA2RST_Pos,    COMBINE('D', 2) },
#endif
#ifdef BDMA
   { BDMA, &RCC->AHB4ENR, RCC_AHB4ENR_BDMAEN_Pos,   &RCC->AHB4RSTR, RCC_AHB4RSTR_BDMARST_Pos,    COMBINE('D', 3) },
#endif

/* GPIO ------------------------------------------------------------------------------------------------------------------- */
#ifdef GPIOA
   { GPIOA, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOAEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOARST_Pos,  COMBINE('G', 'A') },
#endif
#ifdef GPIOB
   { GPIOB, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOBEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOBRST_Pos,  COMBINE('G', 'B') },
#endif
#ifdef GPIOC
   { GPIOC, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOCEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOCRST_Pos,  COMBINE('G', 'C') },
#endif
#ifdef GPIOD
   { GPIOD, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIODEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIODRST_Pos,  COMBINE('G', 'D') },
#endif
#ifdef GPIOE
   { GPIOE, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOEEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOERST_Pos,  COMBINE('G', 'E') },
#endif
#ifdef GPIOF
   { GPIOF, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOFEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOFRST_Pos,  COMBINE('G', 'F') },
#endif
#ifdef GPIOG
   { GPIOG, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOGEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOGRST_Pos,  COMBINE('G', 'G') },
#endif
#ifdef GPIOH
   { GPIOH, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOHEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOHRST_Pos,  COMBINE('G', 'H') },
#endif
#ifdef GPIOI
   { GPIOI, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOIEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOIRST_Pos,  COMBINE('G', 'I') },
#endif
#ifdef GPIOJ
   { GPIOJ, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOJEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOJRST_Pos,  COMBINE('G', 'J') },
#endif
#ifdef GPIOK
   { GPIOK, &RCC->AHB4ENR,  RCC_AHB4ENR_GPIOKEN_Pos, &RCC->AHB4RSTR, RCC_AHB4RSTR_GPIOKRST_Pos,  COMBINE('G', 'K') },
#endif

/* U(s)art ----------------------------------------------------------------------------------------------------------------- */
#if defined(USART1) && defined(USE_USART1)
   { USART1, &RCC->APB2ENR, RCC_APB2ENR_USART1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Pos, COMBINE('U', 1) },
#endif
#if defined(USART2) && defined(USE_USART2)
   { USART2, &RCC->APB1LENR, RCC_APB1LENR_USART2EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_USART2RST_Pos, COMBINE('U', 2) },
#endif
#if defined(USART3) && defined(USE_USART3)
   { USART3, &RCC->APB1LENR, RCC_APB1LENR_USART3EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_USART3RST_Pos, COMBINE('U', 3) },
#endif
#if defined(UART4) && defined(USE_UART4)
   { UART4, &RCC->APB1LENR, RCC_APB1LENR_UART4EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_UART4RST_Pos, COMBINE('U', 4) },
#endif
#if defined(UART5) && defined(USE_UART5)
   { UART5, &RCC->APB1LENR, RCC_APB1LENR_UART5EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_UART5RST_Pos, COMBINE('U', 5) },
#endif
#if defined(USART6) && defined(USE_USART6)
   { USART6, &RCC->APB2ENR, RCC_APB2ENR_USART6EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_USART6RST_Pos, COMBINE('U', 6) },
#endif
#if defined(UART7) && defined(USE_UART7)
   { UART7, &RCC->APB1LENR, RCC_APB1LENR_UART7EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_UART7RST_Pos, COMBINE('U', 7) },
#endif
#if defined(UART8) && defined(USE_UART8)
   { UART8, &RCC->APB1LENR, RCC_APB1LENR_UART8EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_UART8RST_Pos, COMBINE('U', 8) },
#endif
#if defined(LPUART1) && defined(USE_LPUART1)
   { LPUART1, &RCC->APB4ENR, RCC_APB4ENR_LPUART1EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_LPUART1RST_Pos, COMBINE('U', 9) },
#endif

/* SPI --------------------------------------------------------------------------------------------------------------------- */
#if defined(SPI1) && defined ( USE_SPI1 )
   { SPI1, &RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Pos, COMBINE('S', 1) },
#endif
#if defined(SPI2) && defined ( USE_SPI2 )
   { SPI2, &RCC->APB1LENR, RCC_APB1LENR_SPI2EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_SPI2RST_Pos, COMBINE('S', 2) },
#endif
#if defined(SPI3) && defined ( USE_SPI3 )
   { SPI3, &RCC->APB1LENR, RCC_APB1LENR_SPI3EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_SPI3RST_Pos, COMBINE('S', 3) },
#endif
#if defined(SPI4) && defined ( USE_SPI4 )
   { SPI4, &RCC->APB2ENR, RCC_APB2ENR_SPI4EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_SPI4RST_Pos, COMBINE('S', 4) },
#endif
#if defined(SPI5) && defined ( USE_SPI5 )
   { SPI5, &RCC->APB2ENR, RCC_APB2ENR_SPI5EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_SPI5RST_Pos, COMBINE('S', 5) },
#endif
#if defined(SPI6) && defined ( USE_SPI6 )
   { SPI6, &RCC->APB4ENR, RCC_APB4ENR_SPI6EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_SPI6RST_Pos, COMBINE('S', 6) },
#endif

/* I2C --------------------------------------------------------------------------------------------------------------------- */
#if defined(I2C1) && defined(USE_I2C1)
   { I2C1, &RCC->APB1LENR, RCC_APB1LENR_I2C1EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_I2C1RST_Pos, COMBINE('I', 1) },
#endif
#if defined(I2C2) && defined(USE_I2C2)
   { I2C2, &RCC->APB1LENR, RCC_APB1LENR_I2C2EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_I2C2RST_Pos, COMBINE('I', 2) },
#endif
#if defined(I2C3) && defined(USE_I2C3)
   { I2C3, &RCC->APB1LENR, RCC_APB1LENR_I2C3EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_I2C3RST_Pos, COMBINE('I', 3) },
#endif
#if defined(I2C4) && defined(USE_I2C4)
   { I2C4, &RCC->APB4ENR, RCC_APB4ENR_I2C4EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_I2C4RST_Pos, COMBINE('I', 4) },
#endif

/* ADC     - All ADC share ONE clock enable and ONE reset bit  -------------------------------------------------------------- */
#if defined(ADC1) && defined(USE_ADC1)
   { ADC1, &RCC->AHB1ENR, RCC_AHB1ENR_ADC12EN_Pos,   &RCC->AHB1RSTR, RCC_AHB1RSTR_ADC12RST_Pos,    COMBINE('A', 1) },
#endif
#if defined(ADC2) && defined(USE_ADC2)
   { ADC2, &RCC->AHB1ENR, RCC_AHB1ENR_ADC12EN_Pos,   &RCC->AHB1RSTR, RCC_AHB1RSTR_ADC12RST_Pos,    COMBINE('A', 2) },
#endif
#if defined(ADC3) && defined(USE_ADC3)
   { ADC3, &RCC->AHB4ENR, RCC_AHB4ENR_ADC3EN_Pos,   &RCC->AHB4RSTR, RCC_AHB4RSTR_ADC3RST_Pos,    COMBINE('A', 3) },
#endif

/* Alle Timer --------------------------------------------------------------------------------------------------------------- */
#if defined(HRTIM1) 
   { HRTIM1, &RCC->APB2ENR, RCC_APB2ENR_HRTIMEN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_HRTIMRST_Pos, COMBINE('T', 0) },
#endif
#if defined(TIM1) 
   { TIM1, &RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST_Pos, COMBINE('T', 1) },
#endif
#if defined(TIM2) 
   { TIM2, &RCC->APB1LENR, RCC_APB1LENR_TIM2EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM2RST_Pos, COMBINE('T', 2) },
#endif
#if defined(TIM3) 
   { TIM3, &RCC->APB1LENR, RCC_APB1LENR_TIM3EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM3RST_Pos, COMBINE('T', 3) },
#endif
#if defined(TIM4) 
   { TIM4, &RCC->APB1LENR, RCC_APB1LENR_TIM4EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM4RST_Pos, COMBINE('T', 4) },
#endif
#if defined(TIM5) 
   { TIM5, &RCC->APB1LENR, RCC_APB1LENR_TIM5EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM5RST_Pos, COMBINE('T', 5) },
#endif
#if defined(TIM6) 
   { TIM6, &RCC->APB1LENR, RCC_APB1LENR_TIM6EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM6RST_Pos, COMBINE('T', 6) },
#endif
#if defined(TIM7) 
   { TIM7, &RCC->APB1LENR, RCC_APB1LENR_TIM7EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM7RST_Pos, COMBINE('T', 7) },
#endif
#if defined(TIM8) 
   { TIM8, &RCC->APB2ENR, RCC_APB2ENR_TIM8EN_Pos, &RCC->APB2RSTR, RCC_APB2RSTR_TIM8RST_Pos, COMBINE('T', 8) },
#endif

#if defined(TIM12) 
   { TIM12, &RCC->APB1LENR, RCC_APB1LENR_TIM12EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM12RST_Pos, COMBINE('T', 12) },
#endif
#if defined(TIM13) 
   { TIM13, &RCC->APB1LENR, RCC_APB1LENR_TIM13EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM13RST_Pos, COMBINE('T', 13) },
#endif
#if defined(TIM14) 
   { TIM14, &RCC->APB1LENR, RCC_APB1LENR_TIM14EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_TIM14RST_Pos, COMBINE('T', 14) },
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
   { LPTIM1, &RCC->APB1LENR, RCC_APB1LENR_LPTIM1EN_Pos, &RCC->APB1LRSTR, RCC_APB1LRSTR_LPTIM1RST_Pos, COMBINE('T', 21) },
#endif
#if defined(LPTIM2) 
   { LPTIM2, &RCC->APB4ENR, RCC_APB4ENR_LPTIM2EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_LPTIM2RST_Pos, COMBINE('T', 22) },
#endif
#if defined(LPTIM3) 
   { LPTIM3, &RCC->APB4ENR, RCC_APB4ENR_LPTIM3EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_LPTIM3RST_Pos, COMBINE('T', 23) },
#endif
#if defined(LPTIM4) 
   { LPTIM4, &RCC->APB4ENR, RCC_APB4ENR_LPTIM4EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_LPTIM4RST_Pos, COMBINE('T', 24) },
#endif
#if defined(LPTIM5) 
   { LPTIM5, &RCC->APB4ENR, RCC_APB4ENR_LPTIM5EN_Pos, &RCC->APB4RSTR, RCC_APB4RSTR_LPTIM5RST_Pos, COMBINE('T', 25) },
#endif

/* QUADSPI  --------------------------------------------------------------------------------------------------------------- */
#if defined(QUADSPI) && defined(USE_QSPI1) 
   { QUADSPI, &RCC->AHB3ENR, RCC_AHB3ENR_QSPIEN_Pos, &RCC->AHB3RSTR, RCC_AHB3RSTR_QSPIRST_Pos, COMBINE('Q', 1) },
#endif
#define USE_CAN1
/* CAN1  ------------------------------------------------------------------------------------------------------------------ */
#if defined(FDCAN1) && defined(USE_CAN1) 
   { FDCAN1, &RCC->APB1HENR, RCC_APB1HENR_FDCANEN_Pos, &RCC->APB1HRSTR, RCC_APB1HRSTR_FDCANRST_Pos, COMBINE('C', 1) },
#endif
/* ETH ---------------------------------------------------------------------------------------------------------------------- */
#if defined(USE_ETH) && defined(ETH)
   { ETH, &RCC->APB1HENR, RCC_AHB1ENR_ETH1MACEN_Pos, &RCC->AHB1RSTR, RCC_AHB1RSTR_ETH1MACRST_Pos, COMBINE('E', 1) },
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

    Error_Handler_XX(-7, __FILE__, __LINE__); 
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

    Error_Handler_XX(-7, __FILE__, __LINE__); 
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
        Error_Handler_XX(-7, __FILE__, __LINE__); 
    #endif
}

void HW_SetDmaChClock ( const HW_DmaType *tx, const HW_DmaType *rx)
{
    DMA_Stream_TypeDef *use;
    /* At least one handler muist be specified */
    if ( !tx && !rx ) return;

    /* first try rx handler */
    if ( rx ) 
        use = rx->dmaChannel;
    else 
        use = tx->dmaChannel;

    #if defined ( BDMA )
        if ((uint32_t)(use) >= (uint32_t)(BDMA_Channel0) && (uint32_t)(use) <= (uint32_t)(BDMA_Channel7) ) {
            HW_SetHWClock(BDMA, 1 );
            return;
        }
    #endif

    #if !defined(DMA2)
        HW_SetHWClock(DMA1, 1 );
    #else
        /* Get the DMA instance from DMA channel */
        if ((uint32_t)(use) < (uint32_t)(DMA2_Stream0)) {
            /* DMA1 */
            HW_SetHWClock(DMA1, 1 );
        } else  {
            /* DMA2 */
            HW_SetHWClock(DMA2, 1 );
        }
    #endif
}

/*
 * No bit banding in STM32H7
 */
uint32_t *    HW_GetPeriphBitBandAddr     ( __IO uint32_t *periphAddr, uint16_t bit_number )
{
    UNUSED(periphAddr);UNUSED(bit_number);
    Error_Handler_XX(-8, __FILE__, __LINE__); 
    return 0;
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
    uint32_t offset = (uint32_t)gp - D3_AHB1PERIPH_BASE;
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
  *(__IO uint32_t*)(0x5C001004) |= 0x00700000; // DBGMCU_CR D3DBGCKEN D1DBGCKEN TRACECLKEN
    RTC_DEBUG_STOP(); 
    TMR_DEBUG_STOP();

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
        case 0x450: return "STM32H745/755/747/757xx";
        default:    return "Unknown Device";
    }
}

static const char *Get_PackageName( uint16_t package )
{
    switch(package&0b1111) {
        case 0b00010: return "UFBGA169/LQFP176";
        case 0b00011: return "LQFP144";
        case 0b00110: return "LQFP176";
        case 0b00111: return "UFBGA176";
        case 0b01001: return "LQFP208";
        case 0b01010: return "LQFP208";
        default: return "Unknown Package";
    }
}

char Get_RevisionName( uint16_t devID, uint16_t revID)
{
    char work;
    switch (revID) {
        case 0x1000 : work = '1'; break;
        case 0x1001 : work = '2'; break;
        case 0x1003 : work = '3'; break;
        case 0x1007 : work = '4'; break;
        case 0x2001 : work = 'X'; break;
        case 0x2003 : work = 'V'; break;
        default: return '?';
    }
    if ( devID == 0x461 ) 
        work = work - '1'+'A';

    return work;
}

void HW_ReadID(DeviceIdT *id )
{
    uint32_t on_bit;

    memmove(&id->id, (uint8_t *)UID_BASE, 12 );
    
     /* Get clock status of PWR domain and switch on, if not already on*/
    id->devID = HAL_GetDEVID();
    id->revID = HAL_GetREVID();
    id->flashSize = *(uint16_t*)FLASHSIZE_BASE;

     /* Get clock status of PWR domain and switch on, if not already on*/
    on_bit = __HAL_RCC_SYSCFG_IS_CLK_ENABLED();
    if ( !on_bit ) __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* read package */
    id->package   = SYSCFG->PKGR;

    /* restore syscfg clock */
    if ( !on_bit ) __HAL_RCC_SYSCFG_CLK_DISABLE();
}

bool HW_DumpID(char *cmdline, size_t len, const void * arg )
{
    uint32_t i;

    UNUSED(cmdline);UNUSED(len);UNUSED(arg);
    DeviceIdT MyId;
    DeviceIdT *id = &MyId;
    HW_ReadID(id);

    printf("%s Rev.%c %s\n",Get_DeviceName(id->devID), Get_RevisionName(id->devID, id->revID), Get_PackageName(id->package) );
    printf("Flashsize: %d kB\n",id->flashSize);
    printf("UID=");
    for ( i = 0; i < 12;  ) {
        print_hexXX(id->id[i]); 
        i++;
        if ( i % 4 == 0 ) putchar(' ');
    }
    CRLF();

    return true;
}

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
  hdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma->Init.MemBurst            = DMA_MBURST_INC4;
  hdma->Init.PeriphBurst         = DMA_MBURST_INC4;

}


