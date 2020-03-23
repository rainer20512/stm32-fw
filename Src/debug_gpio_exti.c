/**
  ******************************************************************************
  * @file    debug_util.c
  * @author  Rainer
  * @brief   functions for debugging/dumping GPIO and EXTI Settings
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/** @addtogroup DEBUG_UTILS
  * @{
  */

#include "config/config.h"
#include "debug.h"
#include "system/hw_util.h"
#if DEBUG_FEATURES > 0

#include <string.h>

#include "stm32l4xx.h"
#include "debug_helper.h"
#include "system/exti_handler.h"

/*
 *************************************************************
 * defines
 *************************************************************
 */
#define EXTI_MAXNUM           40
#define EXTI1_IS_GPIO         0b00000000000000001111111111111111
#define EXTI2_IS_GPIO         0b00000000
#define EXTI1_IS_CONFIGURABLE 0b00000000011111011111111111111111
#define EXTI2_IS_CONFIGURABLE 0b01111000

/*
 *************************************************************
 * local helper functions 
 *************************************************************
 */


/*
 *************************************************************
 * Functions to dump GPIO status
 *************************************************************
 */


const char * const moder_txt[]={"Inp ", "Out ", "AF  ", "Ana ", "ADC " };
static const char* DBG_get_moder_txt(uint32_t sel, uint16_t ascr )
{
  // If Analog mode and connected to ADC, then annotate accordingly
  if ( sel == 0b11 && ascr ) sel = 4;
  if ( sel < sizeof(moder_txt)/sizeof(char *) ) 
    return moder_txt[sel];
  else
    return "??? ";
}

const char * const pupdr_txt[]={"--  ", "PU  ", "PD  ", "Res " };
static const char* DBG_get_pupdr_txt(uint32_t sel)
{
  if ( sel < sizeof(pupdr_txt)/sizeof(char *) ) 
    return pupdr_txt[sel];
  else
    return "??? ";
}

const char * const ospeedr_txt[]={"Low ", "Med ", "Hi  ", "Vhi " };
static const char* DBG_get_ospeedr_txt(uint32_t sel)
{
  if ( sel < sizeof(ospeedr_txt)/sizeof(char *) ) 
    return ospeedr_txt[sel];
  else
    return "??? ";
}



static void DBG_gpio_dump_moder ( uint32_t moder, uint32_t ascr )
{
  const uint32_t moder_mask = 0xC0000000;
  const uint16_t ascr_mask  = 0x8000;
  uint32_t cnt;
  for ( cnt=0; cnt < 16; cnt++ ) {
    DEBUG_PRINTF("%s",DBG_get_moder_txt((moder & moder_mask) >> 30, ( ascr &  ascr_mask ) >> 15 ));
    moder <<= 2;
    ascr <<= 1;
  }
}

static void DBG_gpio_dump_ospeedr ( uint32_t val )
{
  const uint32_t mask = 0xC0000000;
  uint32_t cnt;
  for ( cnt=0; cnt < 16; cnt++ ) {
    DEBUG_PRINTF("%s",DBG_get_ospeedr_txt((val & mask) >> 30 ));
    val <<= 2;
  }
}

static void DBG_gpio_dump_pupdr ( uint32_t val )
{
  const uint32_t mask = 0xC0000000;
  uint32_t cnt;
  for ( cnt=0; cnt < 16; cnt++ ) {
    DEBUG_PRINTF("%s",DBG_get_pupdr_txt((val & mask) >> 30 ));
    val <<= 2;
  }
}

static void DBG_gpio_dump_afr ( uint32_t val )
{
  const uint32_t mask = 0xF0000000;
  uint32_t cnt;
  uint32_t current;
  for ( cnt=0; cnt  < 8; cnt++ ) {
    current = (val & mask) >> 28;
    print_dec_number ( current, 2, true);DEBUG_PRINTF("  ");
    val <<= 4;
  }
}

static void DBG_gpio_dump_otyper ( uint32_t val )
{
  uint32_t position = 0x8000;
  uint32_t cnt;

  for ( cnt = 0; cnt <16; cnt++ ) {
    DEBUG_PRINTF( position & val ? "OD  " : "PP  " );
    position >>= 1;
  }
}

static void DBG_gpio_dump_bitwise ( uint32_t val )
{
  uint32_t position = 0x8000;
  uint32_t cnt;

  for ( cnt = 0; cnt <16; cnt++ ) {
    DEBUG_PRINTF( position & val ? "1   " : "0   " );
    position >>= 1;
  }
}

static void DBG_gpio_dump_bitnumbers ( char gpio_letter )
{
  int8_t cnt;

  for ( cnt = 15; cnt >= 0; cnt-- ) {
    DEBUG_PUTC ( gpio_letter );
    print_dec_number ( cnt, 2, true);
    DEBUG_PUTC(' ');
  }
}



void DBG_dump_gpio_status(char gpio_letter )
{
    GPIO_TypeDef *gp = HW_GetGPIO ( gpio_letter );
    bool gp_clck_ena; 
    

    DEBUG_PRINTF  ("Status of GPIO%c----------------------------------------------------------\n",gpio_letter);
    if ( !gp ) {
      DEBUG_PUTS  ("  ??? undefined ???");
      return;
    }

    gp_clck_ena = HW_GetHWClockStatus ( gp );
    if ( ! gp_clck_ena ) {
      DEBUG_PUTS  ("  Not clocked ");
      return;
    }

    int oldIndent = DBG_setIndentRel(+2);

    DEBUG_PRINTF("        ");DBG_gpio_dump_bitnumbers(gpio_letter);DEBUG_PUTC('\n');
    DEBUG_PRINTF("MODER  :");DBG_gpio_dump_moder(
                gp->MODER, 
            #if defined(STM32476xx)
                gp->ASCR);
            #else
                0);
            #endif 
    DEBUG_PUTC('\n');
    DEBUG_PRINTF("OTYPER :");DBG_gpio_dump_otyper(gp->OTYPER);DEBUG_PUTC('\n');
    DEBUG_PRINTF("OSPEEDR:");DBG_gpio_dump_ospeedr(gp->OSPEEDR);DEBUG_PUTC('\n');
    DEBUG_PRINTF("OPUPDR :");DBG_gpio_dump_pupdr(gp->PUPDR);DEBUG_PUTC('\n');
    DEBUG_PRINTF("AFR    :");DBG_gpio_dump_afr(gp->AFR[1]);DBG_gpio_dump_afr(gp->AFR[0]);DEBUG_PUTC('\n');
    DEBUG_PRINTF("IDR    :");DBG_gpio_dump_bitwise(gp->IDR);DEBUG_PUTC('\n');
    DEBUG_PRINTF("ODR    :");DBG_gpio_dump_bitwise(gp->ODR);DEBUG_PUTC('\n');
  
    DBG_setIndentAbs(oldIndent);
}

#define TOGGLES 8       /* 8 toggles */
#define DELAY   250     /* duration of every toggle */

void DBG_dump_toggle_pin(char portletter, uint8_t portnum, bool bToggleOnce)
{
    GPIO_TypeDef *gp = HW_GetGPIO ( portletter );
    GPIO_TypeDef save;
    GPIO_InitTypeDef init = {0};
    uint32_t pin_bitmap = 1 << (portnum & 0x0f);
    bool gp_clck_ena; 

    gp_clck_ena = HW_GetHWClockStatus ( gp );
    if ( !gp ) {
        DEBUG_PRINTF("Illegal portletter %c\n", portletter );
        return;
    }

    // Switch portclock on, if not already on
    if (!gp_clck_ena ) HW_SetHWClock(gp, true);

    if (bToggleOnce ) {
        DEBUG_PRINTF("Change %c%02d from %s\n", portletter, portnum, gp->ODR & pin_bitmap ? "H to L": "L to H");
        if (gp->ODR & pin_bitmap)
            gp->BRR = pin_bitmap;
        else
            gp->BSRR = pin_bitmap;
    } else {
        DEBUG_PRINTF("Toggle %c%02d %d times\n", portletter, portnum, TOGGLES);

        // Save the current GPIO status completely
        save = *gp;

        init.Mode =  GPIO_MODE_OUTPUT_PP;
        init.Pull =  GPIO_NOPULL;
        init.Speed = GPIO_SPEED_FREQ_LOW;
        init.Pin  = pin_bitmap; 
       GpioInitHW( gp, &init);

        for ( uint8_t i = 0; i < TOGGLES; i++ ) {
            gp->ODR  ^= pin_bitmap;
            HAL_Delay(DELAY);
        }

         // restore GPIO status
         *gp = save;
     }
     // Switch portclock off again, if it was off before
     if (!gp_clck_ena ) HW_SetHWClock(gp, false);
}

static void DBG_init_pin_internal(char portletter, uint8_t portnum, bool bDoInit, uint32_t speed, uint32_t pupd_status, uint32_t outval)
{
    GPIO_TypeDef *gp = HW_GetGPIO ( portletter );
    GPIO_InitTypeDef init = {0};
    uint32_t pin_bitmap = 1 << (portnum & 0x0f);
    bool gp_clck_ena; 

    gp_clck_ena = HW_GetHWClockStatus ( gp );
    if ( !gp ) {
        DEBUG_PRINTF("Illegal portletter %c\n", portletter );
        return;
    }

    // Switch portclock on, if not already on
    if (!gp_clck_ena ) HW_SetHWClock(gp, true);

    /* in case of DeInit */
    if ( !bDoInit ) {
        /* in case of DeInit */
        DEBUG_PRINTF("DeInit %c%02d\n", portletter, portnum);
        HAL_GPIO_DeInit(gp, pin_bitmap);
    } else {
        DEBUG_PRINTF("Init %c%02d as output\n", portletter, portnum);
        init.Mode =  GPIO_MODE_OUTPUT_PP;
        init.Pin  = pin_bitmap; 
        switch( pupd_status) {
            case 1:
                init.Pull =  GPIO_PULLUP;
                break;
            case 2:
                init.Pull =  GPIO_PULLDOWN;
                break;
            default:
                init.Pull =  GPIO_NOPULL;
        }
        switch(speed) {
            case 1:
                init.Speed = GPIO_SPEED_FREQ_MEDIUM;
                break;
            case 2:
                init.Speed = GPIO_SPEED_FREQ_HIGH;
                break;
            case 3:
                init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                break;
            default:
                init.Speed = GPIO_SPEED_FREQ_LOW;
        }
        switch(outval) {
            case 1:
                /* Set output pin high */
                gp->BSRR = pin_bitmap;
                break;
            default:
                /* Set output pin low */
                gp->BRR = pin_bitmap;
        }
       GpioInitHW( gp, &init);

    }
    // Switch portclock off again, if it was off before
    if (!gp_clck_ena ) HW_SetHWClock(gp, false);
}

void DBG_init_pin(char portletter, uint8_t portnum, uint32_t speed, uint32_t pupd_status, uint32_t init)
{
    DBG_init_pin_internal(portletter, portnum, true, speed, pupd_status, init);
}
void DBG_deinit_pin(char portletter, uint8_t portnum)
{
    DBG_init_pin_internal(portletter, portnum, false, 0, 0, 0);
}

/*
 *************************************************************
 * Functions to dump EXTI settings
 *************************************************************
 */
const char exti_gpio_name[] = "GPIO";
const char * const exti_line_name[]= { 
/* insert pattern ( max length is 12 ) 
  "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", "LPUART1 wkup", */
  "PVS",          "OTG FS wkup",  "RTC alarms",   "RTC tamper  ", "RTC wkup tmr", "COMP1 output", "COMP2 output", "I2C1 wkup",
  "I2C2 wkup",    "I231 wkup",    "USART1 wkup",  "USART2 wkup",  "USART3 wkup",  "UART4 wkup",   "UART5 wkup",   "LPUART1 wkup",
  "LPTIM1   ",    "LPTIM2   ",    "SWPMI1 wkup",  "PVM1 wkup",    "PVM2 wkup",    "PVM3 wkup",    "PVM4 wkup",    "LCD wkup",
  "I2C4 wkup",
};

/*
 *************************************************************
 * @brief Fill in the EXTI Line name
 * @param sel    - exti line number
 * @param retbuf - return buffer to be written to
 * @param maxlen - length of retbuf on retrun, right padded with blanks
 * @note retbuf must provide at least maxlen+1 bytes !!!
 *************************************************************
 */
static void DBG_get_exti_line_name(uint32_t sel, char *retbuf, const uint32_t maxlen)
{
  uint32_t cptr  = 0;
  const char *p;

  memset(retbuf, ' ', maxlen );
  if ( sel < 16 ) {
    /* assemble constant part plus number ( 2 digits ) */
    cptr = strlen(exti_gpio_name);
    strncpy(retbuf, exti_gpio_name,cptr );
    retbuf[cptr++] = '0' + sel/10;
    retbuf[cptr++] = '0' + sel % 10;
  } else {
    /* Copy namr from array */
    sel -= 16;
    if ( sel < sizeof(exti_line_name)/sizeof(char *) ) {
      p = exti_line_name[sel];
      strncpy(retbuf, p, strlen(p));
    }
  }

  /* append terminating \0 */
  retbuf[maxlen] = '\0';
}


static char DBG_get_exti_pinsource(uint32_t pinidx )
{
  /* one exti config register in SYSCFG->EXTICRx entry is 4 bit wide */
  const uint32_t exticr_mask = 0b1111;
  
  /* One register holds the configuration for four lines */
  uint32_t exticr_offset = pinidx >> 2;

  /* compute the amount of bits to shift mask and shift back result bits */
  uint32_t exticr_shift  = ( pinidx & 0b11 ) * 4;
  
  /* Get the relevant four bits right aligned */
  uint32_t exticr_bits = ( SYSCFG->EXTICR[exticr_offset] & ( exticr_mask << exticr_shift ) ) >> exticr_shift;
  if ( exticr_bits > 8 ) 
    return '?';
  else
    return 'A' + exticr_bits;
}


#define EXTI_MAXNUM           40
#define EXTI1_IS_GPIO         0b00000000000000001111111111111111
#define EXTI2_IS_GPIO         0b00000000
#define EXTI1_IS_CONFIGURABLE 0b00000000011111011111111111111111
#define EXTI2_IS_CONFIGURABLE 0b01111000

#define EXTI_NAMELEN          12

#define GET_IMR(i)  ( i > 31 ? EXTI->IMR2  : EXTI->IMR1 )
#define GET_EMR(i)  ( i > 31 ? EXTI->EMR2  : EXTI->EMR1 )
#define GET_PR(i)   ( i > 31 ? EXTI->PR2   : EXTI->PR1 )
#define GET_FTSR(i) ( i > 31 ? EXTI->FTSR2 : EXTI->FTSR1 )
#define GET_RTSR(i) ( i > 31 ? EXTI->RTSR2 : EXTI->RTSR1 )

#define EXTI_IS_GPIO(i)    ( i > 31 ? EXTI2_IS_GPIO : EXTI1_IS_GPIO )
#define EXTI_IS_CFGABLE(i) ( i > 31 ? EXTI2_IS_CONFIGURABLE : EXTI1_IS_CONFIGURABLE )

static void DBG_n_spaces(uint32_t num )
{
  while ( num ) {
    DEBUG_PUTC(' ');
    num--;
  }
}

static void DBG_dump_one_exti(uint32_t idx )
{
  char namebuf[EXTI_NAMELEN+1];
  uint32_t mask = 1 << ( idx & 0x1f );

  DBG_get_exti_line_name(idx, namebuf, EXTI_NAMELEN);
  DEBUG_PRINTF("%s",namebuf);
  if ( ( GET_IMR(idx) | GET_EMR(idx) )  & mask ) {
    DEBUG_PRINTF(" %s %s", GET_IMR(idx) & mask ? "I" : " ", GET_EMR(idx) & mask ? "E" : " ");
    DEBUG_PRINTF(" %s", GET_PR(idx) & mask ? "P" : " ");
    DEBUG_PRINTF(" %s", Exti_Has_Callback(idx) ? "Cb" : "  ");
    if ( mask & EXTI_IS_CFGABLE(idx) ) {
      DEBUG_PRINTF(" %s %s", GET_RTSR(idx) & mask ? "Rise" : "    ", GET_FTSR(idx) & mask ? "Fall" : "    " );
    } else {
        DBG_n_spaces(10);
    }

    if ( mask & EXTI_IS_GPIO(idx) ) {
        DEBUG_PRINTF(" P%c",DBG_get_exti_pinsource(idx));
        print_dec_number ( idx, 2, true);
    } else {
        DBG_n_spaces(5);
    }
  } else {
    DEBUG_PRINTF(" -----"); DBG_n_spaces(18);

  }
}

void DBG_dump_exti_config(void)
{
    uint32_t i;

    DEBUG_PUTS  ("EXTI Settings ------------------------------------------------------------");
    int oldIndent = DBG_setIndentRel(+2);

    for ( i = 0; i <= EXTI_MAXNUM/2; i++ )
    {
      DBG_dump_one_exti(i); DEBUG_PRINTF("  |  " );
      if ( i+EXTI_MAXNUM/2+1 <= EXTI_MAXNUM ) DBG_dump_one_exti(i+EXTI_MAXNUM/2+1); 
      DEBUG_PRINTF("\n" );
    }
 
    DBG_setIndentAbs(oldIndent);

}


/*
 *************************************************************
 * Functions to dump NVIC settings
 *************************************************************
 */

#define NVIC_NAME_LEN             15
const char * const nvic_name_undef= "???";
const char * const sys_nvic_name[]= { 
/*.             .             .             .             .             .             .             . */
  "",           "",           "NMI",        "HardFault",  "MemMgmt",    "BusFault",   "UsageFault", "", 
  "",           "",           "",           "SVCall",     "DebugMon",   "",           "PendSV",     "SysTick"
};

const char * const user_nvic_name[]= { 
/*.                 .                 .                 .                 .                 .                 .                 . */
  "WWDG",           "PVD/PVM",        "RTC tamper",     "RTC wkup",       "Flash",          "RCC",            "EXTI0",          "EXTI1", 
  "EXTI2",          "EXTI3",          "EXTI4",          "DMA1_CH1",       "DMA1_CH2",       "DMA1_CH3",       "DMA1_CH4",       "DMA1_CH5",
  "DMA1_CH6",       "DMA1_CH7",       "ADC1/ADC2",      "CAN1_TX",        "CAN1_RX0",       "CAN1_RX1",       "CAN1_SCE",       "EXTI9_5",  
  "TIM1_BRK/TIM15", "TIM1_UP/TIM16",  "TIM1_TRG/TIM17", "TIM1_CC",        "TIM2",           "TIM3",           "TIM4",           "I2C1_EV", 
  "I2C1_ER",        "I2C2_EV",        "I2C2_ER",        "SPI1",           "SPI2",           "USART1",         "USART2",         "USART3", 
  "EXTI15_10",      "RTC alarm",      "DFSDM1_FLT3",    "TIM8_BRK",       "TIM8_UP",        "TIM8_TRG_COM",   "TIM8_CC",        "ADC3", 
  "FMC",            "SDMMC1",         "TIM5",           "SPI3",           "UART4",          "UART5",          "TIM6_DACUNDER",  "TIM7", 
  "DMA2_CH1",       "DMA2_CH2",       "DMA2_CH3",       "DMA2_CH4",       "DMA2_CH5",       "DFSDM1_FLT0",    "DFSDM1_FLT1",    "DFSDM1_FLT2",
  "COMP",           "LPTIM1",         "LPTIM2",         "OTG_FS",         "DMA2_CH6",       "DMA2_CH7",       "LPUART1",        "QUADSPI", 
  "I2C3_EV",        "I2C3_ER",        "SAI1",           "SAI2",           "SWPMI1",         "TSC",            "LCD",            "AES", 
  "RNG",            "FPU",            "HASH/CRS",       "I2C4_EV",        "I2C4_ER",        "DCMI",           "CAN2_TX",        "CAN2_RX0", 
  "CAN1_RX1",       "CAN1_SCE",       "DMA2D",
};

/*
 *************************************************************
 * @brief Fill in the EXTI Line name
 * @param sel    - exti line number
 * @param retbuf - return buffer to be written to
 * @param maxlen - length of retbuf on retrun, right padded with blanks
 * @note retbuf must provide at least maxlen+1 bytes !!!
 *************************************************************
 */
static void DBG_get_nvic_irq_name(int32_t irqnum, char *retbuf, const uint32_t maxlen)
{
  uint32_t cptr  = 0;
  const char *p;

  memset(retbuf, ' ', maxlen );
  if ( irqnum < 0 ) 
    p = sys_nvic_name[16+irqnum];
  else
      if ( irqnum < (int32_t)(sizeof(user_nvic_name)/sizeof(char *)) ) 
        p = user_nvic_name[irqnum];
      else
        p = nvic_name_undef;
  /* Copy Name */
  cptr = strlen(p);
  strncpy(retbuf, p,cptr );

  /* append terminating \0 */
  retbuf[maxlen] = '\0';
}

static uint32_t NVIC_Is_IRQ_enabled(IRQn_Type IRQn)
{
  return((uint32_t)(((NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}



static void DBG_dump_one_nvic_entry(int32_t irqnum, uint32_t prigroup)
{
  char namebuf[NVIC_NAME_LEN+1];
  uint32_t uPreemptPriority, uSubPriority;
  char enabled;

  enabled = ( irqnum < 0 ? 'E' : ( NVIC_Is_IRQ_enabled((IRQn_Type)irqnum) ? 'E' : '-' ) );

  DBG_get_nvic_irq_name( irqnum, namebuf, NVIC_NAME_LEN);
  HAL_NVIC_GetPriority((IRQn_Type) irqnum, prigroup, &uPreemptPriority, &uSubPriority);
  DBG_printf_indent("%s %c %2d %2d", namebuf, enabled, uPreemptPriority, uSubPriority );
}

void DBG_dump_prio_gouping(uint32_t prigroup )
{
  uint32_t pri,subpri;
  pri=subpri=0;
  
  switch(prigroup ) {
    case 0:
    case 1:
    case 2:
    case 3:
      pri    = 16;
      break;
    case 4:
      pri    = 8;
      subpri = 2;
      break;
    case 5:
      pri    = 4;
      subpri = 4;
      break;
    case 6:
      pri    = 2;
      subpri = 8;
      break;
    case 7:
      subpri = 16;
      break;
  }
  DBG_printf_indent("Prigroup=%d, i.e. %d group priority levels and %d subpriority levels\n", prigroup, pri, subpri );
}  

void DBG_dump_nvic_config(void)
{ 
    int oldIndent, prigroup, max_ints;
    prigroup = NVIC_GetPriorityGrouping();
    max_ints = (( SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk ) << SCnSCB_ICTR_INTLINESNUM_Pos ) * 32 + 32;


    DEBUG_PUTS  ("NVIC Settings -------------------------------------------------------------" );
    oldIndent = DBG_setIndentRel(+2);
    DBG_dump_prio_gouping( prigroup );
    DBG_dump_number("Max Nr of Interrupts :", max_ints );
    
    DEBUG_PUTS  ("CortexM Interrupts ------------------------------------------------------" );
    DBG_setIndentRel(+2);
    DBG_dump_one_nvic_entry(NonMaskableInt_IRQn, prigroup );                /*! Cortex-M4 Non Maskable Interrupt         */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(HardFault_IRQn, prigroup);                      /*! Cortex-M4 Hard Fault Interrupt           */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(MemoryManagement_IRQn, prigroup);               /*! Cortex-M4 Memory Management Interrupt    */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(BusFault_IRQn, prigroup);                       /*! Cortex-M4 Bus Fault Interrupt            */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(UsageFault_IRQn, prigroup);                     /*! Cortex-M4 Usage Fault Interrupt          */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(SVCall_IRQn, prigroup);                         /*! Cortex-M4 SV Call Interrupt              */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(DebugMonitor_IRQn, prigroup);                   /*! Cortex-M4 Debug Monitor Interrupt        */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(PendSV_IRQn, prigroup);                         /*! Cortex-M4 Pend SV Interrupt              */
    DEBUG_PUTC('\n');
    DBG_dump_one_nvic_entry(SysTick_IRQn, prigroup);                        /*! Cortex-M4 System Tick Interrupt          */
    DEBUG_PUTC('\n');
    DBG_setIndentRel(-2);

    DEBUG_PUTS  ("Configurable Interrupts -------------------------------------------------" );
    DBG_setIndentRel(+2);
    for ( uint32_t i = 0; i <= (uint32_t)max_ints ; i++ ) {
      if ( NVIC_Is_IRQ_enabled((IRQn_Type) i ) ) {
        DBG_dump_one_nvic_entry(i, prigroup); 
        DEBUG_PUTC('\n');
      }
    }
    oldIndent = DBG_setIndentRel(+2);
 
    /* keks */

 
    DBG_setIndentAbs(oldIndent);
}


#endif // #if DEBUG_FEATURES > 0

/**
  * @}
  */
