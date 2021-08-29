/**
  ******************************************************************************
  * @file    debug_util.c
  * @author  Rainer
  * @brief   Miscellaneous tools for debug output
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/** @addtogroup DEBUG_UTILS
  * @{
  */

#include "debug.h"

#if DEBUG_FEATURES > 0

#include "stm32l4xx.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "debug_helper.h"
#include "rtc.h"


/*
 *************************************************************
 * local helper functions 
 *************************************************************
 */


/*
 *************************************************************
 * Functions to dump PWR settings
 *************************************************************
 */

const char * const vos_txt[]={"Illegal","Range 1: 1.2V","Range 2: 1.0V","Illegal"};
static const char* DBG_get_pwr_cr1_vos_txt(uint32_t sel)
{
  if ( sel < sizeof(vos_txt)/sizeof(char *) ) 
    return vos_txt[sel];
  else
    return "Illegal";
}

const char * const lpms_txt[]={"Stop 0", "Stop 1", "Stop 2", "Standby" };
static const char* DBG_get_pwr_cr1_lpms_txt(uint32_t sel)
{
  if ( sel > 3 )  return "Shutdown";
    
  return lpms_txt[sel];
}

const char * const pls_txt[]={"2.0V", "2.2V", "2.4V", "2.5V", "2.6V", "2.8V", "2.9V", "PB7 Ain" };
static const char* DBG_get_pwr_cr2_pls_txt(uint32_t sel)
{
  if ( sel < sizeof(pls_txt)/sizeof(char *) ) 
    return pls_txt[sel];
  else
    return "Illegal";
}


static void DBG_dump_pwr_cr1(void)
{
  DBG_setPadLen(24);
  DBG_dump_textvalue("LP Run Mode", READ_BIT(PWR->CR1, PWR_CR1_LPR) ? "LowPower Mode" : "Main Mode" );    
  DBG_dump_textvalue("Vcore value", DBG_get_pwr_cr1_vos_txt(( PWR->CR1 & PWR_CR1_VOS_Msk  ) >> PWR_CR1_VOS_Pos) );
  DBG_dump_bitvalue("Disable BkUp Wr Protect ", PWR->CR1, PWR_CR1_DBP);
  DBG_dump_textvalue("Low power Mode", DBG_get_pwr_cr1_lpms_txt(( PWR->CR1 & PWR_CR1_LPMS_Msk  ) >> PWR_CR1_LPMS_Pos) );
}

static void DBG_dump_pwr_cr2(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("VddUSB on", PWR->CR2, PWR_CR2_USV);
  DBG_dump_bitvalue("VddIO2 on", PWR->CR2, PWR_CR2_IOSV);
  DBG_dump_bitvalue("VddA  2.20V monitor", PWR->CR2, PWR_CR2_PVME4);
  DBG_dump_bitvalue("VddA  1.62V monitor", PWR->CR2, PWR_CR2_PVME3);
  DBG_dump_bitvalue("VddIO2 0.9V monitor", PWR->CR2, PWR_CR2_PVME2);
  DBG_dump_bitvalue("VddUSB 1.2V monitor", PWR->CR2, PWR_CR2_PVME1);
  DBG_dump_textvalue("Pwr voltage detect lvl", DBG_get_pwr_cr2_pls_txt(( PWR->CR2 & PWR_CR2_PLS_Msk  ) >> PWR_CR2_PLS_Pos) );
  DBG_dump_bitvalue("Voltage detect enable", PWR->CR2, PWR_CR2_PVDE);
}

static void DBG_dump_pwr_cr3(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("Internal WakeUp enabled", PWR->CR3, PWR_CR3_EIWUL);
  DBG_dump_bitvalue("Apply GPIO PU/PD", PWR->CR3, PWR_CR3_APC);
  DBG_dump_onoffvalue("SRAM2 pwr in Standby", PWR->CR3, PWR_CR3_RRS);
  DBG_dump_bitvalue("Enable WakeUp Pin5", PWR->CR3, PWR_CR3_EWUP5);
  DBG_dump_bitvalue("Enable WakeUp Pin4", PWR->CR3, PWR_CR3_EWUP4);
  DBG_dump_bitvalue("Enable WakeUp Pin3", PWR->CR3, PWR_CR3_EWUP3);
  DBG_dump_bitvalue("Enable WakeUp Pin2", PWR->CR3, PWR_CR3_EWUP2);
  DBG_dump_bitvalue("Enable WakeUp Pin1", PWR->CR3, PWR_CR3_EWUP1);
}

static void DBG_dump_pwr_cr4(void)
{
  DBG_setPadLen(24);
  DBG_dump_uint32_hex("PWR->CR4 raw", PWR->CR4);
}
 
static void DBG_dump_pwr_sr1(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("Wkup Flag Internal", PWR->SR1, PWR_SR1_WUFI);
  DBG_dump_bitvalue("Standby Flag", PWR->SR1, PWR_SR1_SBF);
  DBG_dump_bitvalue("Wkup Flag 5", PWR->SR1, PWR_SR1_WUF5);
  DBG_dump_bitvalue("Wkup Flag 4", PWR->SR1, PWR_SR1_WUF4);
  DBG_dump_bitvalue("Wkup Flag 3", PWR->SR1, PWR_SR1_WUF3);
  DBG_dump_bitvalue("Wkup Flag 2", PWR->SR1, PWR_SR1_WUF2);
  DBG_dump_bitvalue("Wkup Flag 1", PWR->SR1, PWR_SR1_WUF1);
}

static void DBG_dump_pwr_sr2(void)
{
  DBG_setPadLen(24);
  if ( PWR->CR2 & PWR_CR2_PVME4 )   DBG_dump_bitvalue("VddA < 2.20V", PWR->SR2, PWR_SR2_PVMO4);
  if ( PWR->CR2 & PWR_CR2_PVME3 )   DBG_dump_bitvalue("VddA < 1.62V", PWR->SR2, PWR_SR2_PVMO3);
  if ( PWR->CR2 & PWR_CR2_PVME2 )   DBG_dump_bitvalue("VddIO2 < 0.9V", PWR->SR2, PWR_SR2_PVMO2);
  if ( PWR->CR2 & PWR_CR2_PVME1 )   DBG_dump_bitvalue("VddUSB < 1.2V", PWR->SR2, PWR_SR2_PVMO1);
  if ( PWR->CR2 & PWR_CR2_PVDE )    DBG_dump_bitvalue("Vdd < Threshold", PWR->SR2, PWR_SR2_PVDO);
  DBG_dump_bitvalue("Vcore reg. in transit", PWR->SR2, PWR_SR2_VOSF);
  DBG_dump_bitvalue("Vcore reg. LP mode", PWR->SR2, PWR_SR2_REGLPF);
  DBG_dump_bitvalue("LP reg. ready", PWR->SR2, PWR_SR2_REGLPS);
}

void DBG_dump_powersetting(void)
{
  DEBUG_PUTS("PWR Settings ------------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);

  /********  PWR Control registers *****************/
  DBG_printf_indent("PWR: Power Control Register 1 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_cr1();
  DBG_setIndentRel(-2);

  DBG_printf_indent("PWR: Power Control Register 2 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_cr2();
  DBG_setIndentRel(-2);

  DBG_printf_indent("PWR: Power Control Register 3 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_cr3();
  DBG_setIndentRel(-2);

  DBG_printf_indent("PWR: Power Control Register 4 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_cr4();
  DBG_setIndentRel(-2);

  /********  PWR Status registers *****************/
  DBG_printf_indent("PWR: Power Status Register 1 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_sr1();
  DBG_setIndentRel(-2);

  DBG_printf_indent("PWR: Power Status Register 2 \n" );
  DBG_setIndentRel(+2);
  DBG_dump_pwr_sr2();
  DBG_setIndentRel(-2);

  DBG_setIndentAbs(oldIndent);
}



/*
 *************************************************************
 * Functions to dump RCC settings
 *************************************************************
 */


const char * const msirange_txt[]={"100kHz", "200kHz", "400kHz", "800kHz",
                            "1MHz", "2MHz", "4MHz", "8MHz", "16MHz", "24MHz", "32MHz", "48MHz" };
static const char* DBG_get_msirange_text ( uint32_t sel, uint32_t msirgsel )
{
  if ( msirgsel ) {
    // full range 
    if ( sel < sizeof(msirange_txt)/sizeof(char *) ) 
      return msirange_txt[sel];
    else
      return "Illegal";
  } else {
    // restricted range after Reset/Standby
    if ( sel >=4 && sel < 8  ) 
      return msirange_txt[sel];
    else
      return "Illegal";

  }
}
/* ---- RCC CR ---- */
static void DBG_dump_rcc_cr(void)
{
  DBG_setPadLen(22);
  DBG_dump_bitvalue("PLL_SAI2 on", RCC->CR, RCC_CR_PLLSAI2ON );
  if ( READ_BIT(RCC->CR, RCC_CR_PLLSAI2ON) ) DBG_dump_bitvalue("PLL_SAI2 ready", RCC->CR, RCC_CR_PLLSAI2RDY );
  DBG_dump_bitvalue("PLL_SAI1 on", RCC->CR, RCC_CR_PLLSAI1ON );
  if ( READ_BIT(RCC->CR, RCC_CR_PLLSAI1ON) ) DBG_dump_bitvalue("PLL_SAI1 ready", RCC->CR, RCC_CR_PLLSAI1RDY );

  DBG_dump_bitvalue("PLL on", RCC->CR, RCC_CR_PLLON );
  if ( READ_BIT(RCC->CR, RCC_CR_PLLON) ) DBG_dump_bitvalue("PLL ready", RCC->CR, RCC_CR_PLLRDY );

  DBG_dump_bitvalue("HSE on", RCC->CR, RCC_CR_HSEON );
  if ( READ_BIT( RCC->CR, RCC_CR_HSEON ) ) {
    DBG_dump_bitvalue("HSE ready", RCC->CR, RCC_CR_HSERDY);
    DBG_dump_bitvalue("HSE bypass", RCC->CR, RCC_CR_HSEBYP);
    DBG_dump_bitvalue("HSE CSS on", RCC->CR, RCC_CR_CSSON);
  }

  DBG_dump_bitvalue("HSI Autost. from Stop", RCC->CR, RCC_CR_HSIASFS );

  DBG_dump_bitvalue("HSI on", RCC->CR, RCC_CR_HSION );
  if ( READ_BIT( RCC->CR, RCC_CR_HSION ) ) {
    DBG_dump_bitvalue("HSI ready", RCC->CR, RCC_CR_HSIRDY);
    DBG_dump_bitvalue("HSIKern on", RCC->CR, RCC_CR_HSIKERON );
  }
  DBG_dump_bitvalue("MSI on", RCC->CR, RCC_CR_MSION );
  if ( READ_BIT( RCC->CR, RCC_CR_MSION ) ) {
    DBG_dump_bitvalue("MSI ready", RCC->CR, RCC_CR_MSIRDY);
    DBG_dump_bitvalue("MSI PLL enable", RCC->CR, RCC_CR_MSIPLLEN);
    DBG_dump_textvalue("MSI Range Select", READ_BIT( RCC->CR, RCC_CR_MSIRGSEL ) ? "Normal, full range" : "after StBy/Reset, restricted");

    if ( READ_BIT( RCC->CR, RCC_CR_MSIRGSEL ) ) 
      DBG_dump_textvalue("MSI Freq.", DBG_get_msirange_text((RCC->CR & RCC_CR_MSIRANGE_Msk)>>RCC_CR_MSIRANGE_Pos, 1) );
    else  
      DBG_dump_textvalue("MSI Freq.", DBG_get_msirange_text((RCC->CSR & RCC_CSR_MSISRANGE_Msk) >> RCC_CSR_MSISRANGE_Pos, 0) );
  }
}

const char * const rtcsel_txt[]={"No Clock","LSE","LSI","HSE/32"};
static const char* DBG_get_rcc_bdcr_rtcsel_txt(uint32_t sel)
{
  if ( sel < sizeof(rtcsel_txt)/sizeof(char *) ) 
    return rtcsel_txt[sel];
  else
    return "Illegal";
}

/* ---- RCC BDCR ---- */
static void DBG_dump_rcc_bdcr(void)
{
  DBG_setPadLen(16);
  DBG_dump_bitvalue("LSCO enable", RCC->BDCR, RCC_BDCR_LSCOEN);
  if ( READ_BIT( RCC->BDCR, RCC_BDCR_LSCOEN ) ) 
    DBG_dump_textvalue("LSCO source", READ_BIT(RCC->BDCR, RCC_BDCR_LSCOSEL) ? "LSE" : "LSI" );

  DBG_dump_bitvalue("RTC enable", RCC->BDCR, RCC_BDCR_RTCEN );
  if ( READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) )
    DBG_dump_textvalue("RTC/LCD Clk source", DBG_get_rcc_bdcr_rtcsel_txt((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos));
  
  DBG_dump_bitvalue("LSE CSS enable", RCC->BDCR, RCC_BDCR_LSECSSON);
  if ( READ_BIT(RCC->BDCR, RCC_BDCR_LSECSSON) ) 
    DBG_dump_bitvalue("LSE Clk failure", RCC->BDCR, RCC_BDCR_LSECSSD);
    
  DBG_dump_bitvalue("LSE on", RCC->BDCR, RCC_BDCR_LSEON );
  if ( READ_BIT(RCC->BDCR, RCC_BDCR_LSEON ) ) {
    DBG_dump_bitvalue("LSE rdy", RCC->BDCR, RCC_BDCR_LSERDY);
    DBG_dump_bitvalue("LSE bypass", RCC->BDCR, RCC_BDCR_LSEBYP);
    DBG_dump_number("LSE drv strength", (RCC->BDCR & RCC_BDCR_LSEDRV_Msk) >> RCC_BDCR_LSEDRV_Pos);
  }
}

/* ---- RCC ICSCR ---- */
static void DBG_dump_rcc_icscr(void)
{
  DBG_setPadLen(16);
  if ( READ_BIT(RCC->CR, RCC_CR_MSION) ) {
    DBG_dump_number("MSI Calibration", ( RCC->ICSCR & RCC_ICSCR_MSICAL_Msk ) >> RCC_ICSCR_MSICAL_Pos );
    DBG_dump_number("MSI Trimming", (RCC->ICSCR & RCC_ICSCR_MSITRIM_Msk ) >> RCC_ICSCR_MSITRIM_Pos );
  }
  if ( READ_BIT(RCC->CR, RCC_CR_HSION) ) {
    DBG_dump_number("HSI Calibration", (RCC->ICSCR & RCC_ICSCR_HSICAL_Msk ) >> RCC_ICSCR_HSICAL_Pos );
    DBG_dump_number("HSI Trimming", (RCC->ICSCR & RCC_ICSCR_HSITRIM_Msk ) >> RCC_ICSCR_HSITRIM_Pos );
  }
}

/* ---- RCC CFGR ---- */
const char * const mosel_txt[]={"MCO disabled","SYSCLK","MSI","HSI16","HSE","PLL","LSI","LSE"};
static const char* DBG_get_rcc_cfgr_mcosel_txt(uint32_t sel)
{
  if ( sel < sizeof(mosel_txt)/sizeof(char *) ) 
    return mosel_txt[sel];
  else
    return "Illegal";
}

/*
 * decode APB1 and APB2 prescaler values, see Ref.man. p 224 
 */
static int DBG_get_rcc_cfgr_apb_prescaler ( uint32_t inval )
{
  inval &= 0b111;
  if ( inval < 4 ) return 1;
  return 1 << ( inval-3 );
}

/*
 * decode AHB prescaler values, see Ref.man. p 224
 */
#if 0
static int DBG_get_rcc_cfgr_ahb_prescaler ( uint32_t inval )
{
  inval &= 0b1111;
  if ( inval < 0b1000 ) return 1;
  if ( inval < 0b1100 ) 
    return 1 << ( inval - 7 );
  else
   return 1 << ( inval - 6 );
}
#endif

const char * const sws_txt[]={"MSI","HSI16", "HSE", "PLL"};
static const char* DBG_get_rcc_cfgr_sws_txt(uint32_t sel)
{
  if ( sel < sizeof(sws_txt)/sizeof(char *) ) 
    return sws_txt[sel];
  else
    return "Illegal";
}

static void DBG_dump_rcc_cfgr(void)
{
  DBG_setPadLen(16);
  uint32_t mcosel = ( RCC->CFGR & RCC_CFGR_MCOSEL_Msk ) >> RCC_CFGR_MCOSEL_Pos;

  if ( mcosel == 0 ) {
    DBG_printf_indent("MCO disabled\n" );    
  } else {
    DBG_dump_po2("MCO prescaler",  (( RCC->CFGR & RCC_CFGR_MCOPRE_Msk ) >> RCC_CFGR_MCOPRE_Pos) );
    DBG_dump_textvalue("MCO Source", DBG_get_rcc_cfgr_mcosel_txt(( RCC->CFGR & RCC_CFGR_MCOSEL_Msk  ) >> RCC_CFGR_MCOSEL_Pos) );
  }
  DBG_dump_textvalue("WakeUp Clksrc", READ_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK) ? "HSI16" : "MSI" );    
  DBG_dump_number("APB2 Prescale", DBG_get_rcc_cfgr_apb_prescaler(( RCC->CFGR & RCC_CFGR_PPRE2_Msk ) >> RCC_CFGR_PPRE2_Pos) );
  DBG_dump_number("APB1 Prescale", DBG_get_rcc_cfgr_apb_prescaler(( RCC->CFGR & RCC_CFGR_PPRE1_Msk ) >> RCC_CFGR_PPRE1_Pos) );
  DBG_dump_number("AHB Prescale", DBG_get_rcc_cfgr_apb_prescaler(( RCC->CFGR & RCC_CFGR_HPRE_Msk ) >> RCC_CFGR_HPRE_Pos) );
  DBG_dump_textvalue("SysClk source", DBG_get_rcc_cfgr_sws_txt(( RCC->CFGR & RCC_CFGR_SWS_Msk  ) >> RCC_CFGR_SWS_Pos) );
}

/* ---- RCC PLLCFGR ---- */
const char * const pllsrc_txt[]={"No Clock","MSI", "HSI16", "HSE"};
static const char* DBG_get_rcc_pllcfgr_pllsrc_txt(uint32_t sel )
{
  if ( sel < sizeof(pllsrc_txt)/sizeof(char *) ) 
    return pllsrc_txt[sel];
  else
    return "Illegal";
}
static void DBG_dump_rcc_pllcfgr(uint32_t reg)
{
    #define _EN(reg,mask)       ( reg & mask ? "Enabled" : "" )
    DBG_setPadLen(20);
    DBG_dump_textvalue("Source", DBG_get_rcc_pllcfgr_pllsrc_txt(( RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Msk  ) >> RCC_PLLCFGR_PLLSRC_Pos) );  
    DBG_dump_number_and_text("M Div", ( (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk ) >> RCC_PLLCFGR_PLLM_Pos)+1, 3, "" );  
    DBG_dump_number_and_text("N Mul", ( reg & RCC_PLLCFGR_PLLN_Msk ) >> RCC_PLLCFGR_PLLN_Pos, 3, "" );   
    DBG_dump_number_and_text("R Div", ((( reg & RCC_PLLCFGR_PLLR_Msk ) >> RCC_PLLCFGR_PLLR_Pos)+1)*2,   3, _EN(reg, RCC_PLLCFGR_PLLREN_Msk) );  
    DBG_dump_number_and_text("Q DIV", ((( reg & RCC_PLLCFGR_PLLQ_Msk ) >> RCC_PLLCFGR_PLLQ_Pos)+1)*2,   3, _EN(reg, RCC_PLLCFGR_PLLQEN_Msk) );    
    DBG_dump_number_and_text("P DIV", (( reg & RCC_PLLCFGR_PLLP_Msk ) >> RCC_PLLCFGR_PLLP_Pos)==0?7:17, 3, _EN(reg, RCC_PLLCFGR_PLLPEN_Msk) );    
}

/*
static void DBG_dump_rcc_pllcfgr(void)
{
  DBG_setPadLen(20);
  if ( READ_BIT(RCC->CR, RCC_CR_PLLON ) ) {
    DBG_dump_bitvalue("PLLCLK out enabled", RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN);
    DBG_dump_textvalue("PLL source", DBG_get_rcc_pllcfgr_pllsrc_txt(( RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Msk  ) >> RCC_PLLCFGR_PLLSRC_Pos) );  
    DBG_dump_number("PLLM", ( RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk ) >> RCC_PLLCFGR_PLLM_Pos );  
    DBG_dump_number("PLLN", ( RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk ) >> RCC_PLLCFGR_PLLN_Pos );  
    DBG_dump_number("PLLR", ( RCC->PLLCFGR & RCC_PLLCFGR_PLLR_Msk ) >> RCC_PLLCFGR_PLLR_Pos );  
    DBG_dump_number("PLLQ", ( RCC->PLLCFGR & RCC_PLLCFGR_PLLQ_Msk ) >> RCC_PLLCFGR_PLLQ_Pos );  
    DBG_dump_number("PLLP", ( RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk ) >> RCC_PLLCFGR_PLLP_Pos );  
  } else  {
    DBG_printf_indent("PLL disabled\n" );    
  }
}
*/

static void DBG_dump_clocks(void)
{
  RCC_ClkInitTypeDef v;
  uint32_t f_latency;

  DBG_setPadLen(16);
  DBG_dump_number("SYSCLK", HAL_RCC_GetSysClockFreq() );
  DBG_dump_number("HCLK",  HAL_RCC_GetHCLKFreq());
  DBG_dump_number("PCLK1",  HAL_RCC_GetPCLK1Freq());
  DBG_dump_number("PCLK2",  HAL_RCC_GetPCLK2Freq());
  HAL_RCC_GetClockConfig(&v, &f_latency);
  DBG_dump_textvalue("Vcore value", DBG_get_pwr_cr1_vos_txt(( PWR->CR1 & PWR_CR1_VOS_Msk  ) >> PWR_CR1_VOS_Pos) );
  DBG_dump_number("Flash Latency", f_latency);  
}


void DBG_dump_clocksetting(void)
{
  DEBUG_PUTS("RCC Settings ------------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);

  /********  Clock Control register *****************/
  DBG_printf_indent("RCC: Clock Control Register\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_cr();
  DBG_setIndentRel(-2);

  /********  Backup Domain Control register *****************/
  DBG_printf_indent("RCC: Backup Domain Control Register\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_bdcr();
  DBG_setIndentRel(-2);

  /********  MSI/HSI calibration register *****************/
  if (  READ_BIT(RCC->CR, RCC_CR_MSION) || READ_BIT(RCC->CR, RCC_CR_HSION ) ) {
    DBG_printf_indent("RCC: MSI/HSI Calibration Register\n" );
    DBG_setIndentRel(+2);
    DBG_dump_rcc_icscr();
    DBG_setIndentRel(-2);
  }

  /******** Clock configuration register *****************/
  DBG_printf_indent("RCC: Clock configuration register\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_cfgr();
  DBG_setIndentRel(-2);

  /******** PLL configuration register *****************/
  if ( RCC->CR & RCC_CR_PLLON) {
      DBG_printf_indent("RCC: PLL configuration register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_pllcfgr( RCC->PLLCFGR );
      DBG_setIndentRel(-2);
  }

  if ( RCC->CR & RCC_CR_PLLSAI1ON) {
      DBG_printf_indent("RCC: PLLSAI1 configuration register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_pllcfgr( RCC->PLLSAI1CFGR );
      DBG_setIndentRel(-2);
  }

  if ( RCC->CR & RCC_CR_PLLSAI2ON) {
      DBG_printf_indent("RCC: PLLSAI2 configuration register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_pllcfgr( RCC->PLLSAI2CFGR );
      DBG_setIndentRel(-2);
  }

  /******** System Clocks *****************/
  DBG_printf_indent("Resulting System Clocks\n" );
  DBG_setIndentRel(+2);
  DBG_dump_clocks();
  DBG_setIndentRel(-2);


  DBG_setIndentAbs(oldIndent);
}

/*
 *************************************************************
 * Functions to dump peripheral clock settings
 *************************************************************
 */

/*
 *************************************************************
 * The follwowing routines are used to dump ordinary and
 * sleep mode register settings, so the register is passed as 
 * parameter. This only works, because bit positions in 
 * ordinary and sleep mode registers are identical
 *************************************************************
 */
void DBG_dump_rcc_ahbenr(uint32_t reg1, uint32_t reg2, uint32_t reg3, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "AHB1SMENR raw " : "AHB1ENR raw ", reg1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "AHB2SMENR raw " : "AHB2ENR raw ", reg2 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "AHB3SMENR raw " : "AHB3ENR raw ", reg3 );  

#if defined(STM32L4Sxxx)
  DBG_dump_onoffvalue  ("GFXMMU Clock",reg1, RCC_AHB1ENR_GFXMMUEN);  
  DBG_dump_onoffvalue  ("DMA2D Clock", reg1, RCC_AHB1ENR_DMA2DEN);  
#endif
  DBG_dump_onoffvalue  ("TSC Clock",   reg1, RCC_AHB1ENR_TSCEN);  
  DBG_dump_onoffvalue  ("CRC Clock",   reg1, RCC_AHB1ENR_CRCEN);  
  if ( bSleepRegisters)   DBG_dump_onoffvalue  ("SRAM1 Clock", reg1, RCC_AHB1SMENR_SRAM1SMEN);  
  DBG_dump_onoffvalue  ("Flash Clock", reg1, RCC_AHB1ENR_FLASHEN);  
#if defined(DMAMUX1)
  DBG_dump_onoffvalue  ("DMAMUX1 Clock",  reg1, RCC_AHB1ENR_DMAMUX1EN);  
#endif
  DBG_dump_onoffvalue  ("DMA2 Clock",  reg1, RCC_AHB1ENR_DMA2EN);  
  DBG_dump_onoffvalue  ("DMA1 Clock",  reg1, RCC_AHB1ENR_DMA1EN);  

#if defined(STM32L4Sxxx)
  DBG_dump_onoffvalue  ("SDMMC1 Clock",  reg2, RCC_AHB2ENR_SDMMC1EN);  
//  DBG_dump_onoffvalue  ("SDMMC2 Clock",  reg2, RCC_AHB2ENR_SDMMC2EN);  
  DBG_dump_onoffvalue  ("OSPIMgr Clock", reg2, RCC_AHB2ENR_OSPIMEN);  
#endif
  DBG_dump_onoffvalue  ("RNG Clock",  reg2, RCC_AHB2ENR_RNGEN);  
#if defined(HASH)
  DBG_dump_onoffvalue  ("HASH Clock",  reg2, RCC_AHB2ENR_HASHEN);  
#endif  
#if defined(AES)
  DBG_dump_onoffvalue  ("AES Clock",  reg2, RCC_AHB2ENR_AESEN);  
#endif  
#if defined(PKA)
  DBG_dump_onoffvalue  ("PKA Clock",  reg2, RCC_AHB2ENR_PKAEN);  
#endif  
#if defined(DCMI)
  DBG_dump_onoffvalue  ("DCMI Clock",  reg2, RCC_AHB2ENR_DCMIEN);  
#endif  

  DBG_dump_onoffvalue  ("ADC Clock",  reg2, RCC_AHB2ENR_ADCEN);  
  DBG_dump_onoffvalue  ("OTGFS Clock",  reg2, RCC_AHB2ENR_OTGFSEN);  
  if ( bSleepRegisters)  {
    DBG_dump_onoffvalue  ("SRAM2 Clock", reg2, RCC_AHB2SMENR_SRAM2SMEN);  
    #if defined( RCC_AHB2SMENR_SRAM3SMEN)
        DBG_dump_onoffvalue  ("SRAM3 Clock", reg2, RCC_AHB2SMENR_SRAM3SMEN);  
    #endif
  }
  DBG_dump_onoffvalue  ("GPIOH Clock",  reg2, RCC_AHB2ENR_GPIOHEN);  
  DBG_dump_onoffvalue  ("GPIOG Clock",  reg2, RCC_AHB2ENR_GPIOGEN);  
  DBG_dump_onoffvalue  ("GPIOF Clock",  reg2, RCC_AHB2ENR_GPIOFEN);  
  DBG_dump_onoffvalue  ("GPIOE Clock",  reg2, RCC_AHB2ENR_GPIOEEN);  
  DBG_dump_onoffvalue  ("GPIOD Clock",  reg2, RCC_AHB2ENR_GPIODEN);  
  DBG_dump_onoffvalue  ("GPIOC Clock",  reg2, RCC_AHB2ENR_GPIOCEN);  
  DBG_dump_onoffvalue  ("GPIOB Clock",  reg2, RCC_AHB2ENR_GPIOBEN);  
  DBG_dump_onoffvalue  ("GPIOA Clock",  reg2, RCC_AHB2ENR_GPIOAEN);  
    


  #if defined(QSPI)
    DBG_dump_onoffvalue  ("QSPI Clock",  reg3, RCC_AHB3ENR_QSPIEN);  
  #endif
  #if defined(OCTOSPI1)
    DBG_dump_onoffvalue  ("OSPI1 Clock",  reg3, RCC_AHB3ENR_OSPI1EN);  
  #endif
  #if defined(OCTOSPI2)
    DBG_dump_onoffvalue  ("OSPI2 Clock",  reg3, RCC_AHB3ENR_OSPI2EN);  
  #endif
  DBG_dump_onoffvalue  ("FMC Clock",  reg3, RCC_AHB3ENR_FMCEN);  


}

void DBG_dump_rcc_apb1enr(uint32_t reg1, uint32_t reg2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "APB1SMENR1 raw" : "APB1ENR1 raw", reg1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "APB1SMENR2 raw" : "APB1ENR2 raw", reg2 );  

  DBG_dump_onoffvalue  ("LPTIM1 Clock", reg1, RCC_APB1ENR1_LPTIM1EN);  
  DBG_dump_onoffvalue  ("OpAmp Clock", reg1, RCC_APB1ENR1_OPAMPEN);  
  DBG_dump_onoffvalue  ("DAC1 Clock", reg1, RCC_APB1ENR1_DAC1EN);  
  DBG_dump_onoffvalue  ("PWR Clock", reg1, RCC_APB1ENR1_PWREN);  
  DBG_dump_onoffvalue  ("CAN1 Clock", reg1, RCC_APB1ENR1_CAN1EN);  
  DBG_dump_onoffvalue  ("I2C3 Clock", reg1, RCC_APB1ENR1_I2C3EN);  
  DBG_dump_onoffvalue  ("I2C2 Clock", reg1, RCC_APB1ENR1_I2C2EN);  
  DBG_dump_onoffvalue  ("I2C1 Clock", reg1, RCC_APB1ENR1_I2C1EN);  
  DBG_dump_onoffvalue  ("UART5 Clock", reg1, RCC_APB1ENR1_UART5EN);  
  DBG_dump_onoffvalue  ("UART4 Clock", reg1, RCC_APB1ENR1_UART4EN);  
  DBG_dump_onoffvalue  ("USART3 Clock", reg1, RCC_APB1ENR1_USART3EN);  
  DBG_dump_onoffvalue  ("USART2 Clock", reg1, RCC_APB1ENR1_USART2EN);  
  DBG_dump_onoffvalue  ("SPI3 Clock", reg1, RCC_APB1ENR1_SPI3EN);  
  DBG_dump_onoffvalue  ("SPI2 Clock", reg1, RCC_APB1ENR1_SPI2EN);  
  DBG_dump_onoffvalue  ("WWDG Clock", reg1, RCC_APB1ENR1_WWDGEN);  
#if defined(STM32L476xx) || defined(STM32L496xx)
  DBG_dump_onoffvalue  ("LCD Clock", reg1, RCC_APB1ENR1_LCDEN);  
#endif
  DBG_dump_onoffvalue  ("TIM7 Clock", reg1, RCC_APB1ENR1_TIM7EN);  
  DBG_dump_onoffvalue  ("TIM6 Clock", reg1, RCC_APB1ENR1_TIM6EN);  
  DBG_dump_onoffvalue  ("TIM5 Clock", reg1, RCC_APB1ENR1_TIM5EN);  
  DBG_dump_onoffvalue  ("TIM4 Clock", reg1, RCC_APB1ENR1_TIM4EN);  
  DBG_dump_onoffvalue  ("TIM3 Clock", reg1, RCC_APB1ENR1_TIM3EN);  
  DBG_dump_onoffvalue  ("TIM2 Clock", reg1, RCC_APB1ENR1_TIM2EN);  

  DBG_dump_onoffvalue  ("LPTIM2 Clock", reg2, RCC_APB1ENR2_LPTIM2EN);  
#if defined(STM32L476xx) || defined(STM32L496xx)
  DBG_dump_onoffvalue  ("SWPMI1 Clock", reg2, RCC_APB1ENR2_SWPMI1EN);  
#endif
  DBG_dump_onoffvalue  ("I2C4 Clock",   reg2, RCC_APB1ENR2_I2C4EN);  
  DBG_dump_onoffvalue  ("LPUART1 Clock", reg2, RCC_APB1ENR2_LPUART1EN);  
} 

void DBG_dump_rcc_apb2enr(uint32_t reg, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "APB2SMENR raw" : "APB2ENR raw",reg );  
#if defined(STM32L4Sxxx)
  DBG_dump_onoffvalue  ("DSI Clock", reg, RCC_APB2ENR_DSIEN);  
  DBG_dump_onoffvalue  ("LTDC Clock", reg, RCC_APB2ENR_LTDCEN);  
#endif
  DBG_dump_onoffvalue  ("DFSDM1 Clock", reg, RCC_APB2ENR_DFSDM1EN);  
  DBG_dump_onoffvalue  ("SAI2 Clock", reg, RCC_APB2ENR_SAI2EN);  
  DBG_dump_onoffvalue  ("SAI1 Clock", reg, RCC_APB2ENR_SAI1EN);  
  DBG_dump_onoffvalue  ("TIM17 Clock", reg, RCC_APB2ENR_TIM17EN);  
  DBG_dump_onoffvalue  ("TIM16 Clock", reg, RCC_APB2ENR_TIM16EN);  
  DBG_dump_onoffvalue  ("TIM15 Clock", reg, RCC_APB2ENR_TIM15EN);  
  DBG_dump_onoffvalue  ("USART1 Clock", reg, RCC_APB2ENR_USART1EN);  
  DBG_dump_onoffvalue  ("TIM8 Clock", reg, RCC_APB2ENR_TIM8EN);  
  DBG_dump_onoffvalue  ("SPI1 Clock", reg, RCC_APB2ENR_SPI1EN);  
  DBG_dump_onoffvalue  ("TIM1 Clock", reg, RCC_APB2ENR_TIM1EN);  
#if defined(STM32L476xx) || defined(STM32L496xx)
  DBG_dump_onoffvalue  ("SDMMC1 Clock", reg, RCC_APB2ENR_SDMMC1EN);  
#endif
  DBG_dump_onoffvalue  ("FW Clock", reg, RCC_APB2ENR_FWEN);  
  DBG_dump_onoffvalue  ("SYSCFG Clock", reg, RCC_APB2ENR_SYSCFGEN);  
}


void DBG_dump_peripheralclocksetting(void)
{
  DEBUG_PUTS("Peripheral Clock Settings -----------------------" );
  int oldIndent = DBG_setIndentRel(+2);

  /********  AHB peripheral clock enable register *****************/
  DBG_printf_indent("AHB peripheral clocks\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_ahbenr(RCC->AHB1ENR, RCC->AHB2ENR, RCC->AHB3ENR, 0);
  DBG_setIndentRel(-2);

  /********  APB1 peripheral clock enable register *****************/
  DBG_printf_indent("APB1 peripheral clocks\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_apb1enr(RCC->APB1ENR1, RCC->APB1ENR2,0);
  DBG_setIndentRel(-2);

  /********  APB2 peripheral clock enable register *****************/
  DBG_printf_indent("APB2 peripheral clocks\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_apb2enr(RCC->APB2ENR,0);
  DBG_setIndentRel(-2);

  /********  Backup Domain control register *****************/
  DBG_printf_indent("Backup Domain / RTC clock\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_bdcr();
  DBG_setIndentRel(-2);

  DBG_setIndentAbs(oldIndent);
}

void DBG_dump_peripheralclocksetting_insleepmode(void)
{
  DEBUG_PUTS("Peripheral Clock Settings ** IN SLEEP MODE ** ---" );
  int oldIndent = DBG_setIndentRel(+2);

  /********  AHB peripheral clock enable register *****************/
  DBG_printf_indent("AHB peripheral clocks in SleepMode\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_ahbenr(RCC->AHB1SMENR, RCC->AHB2SMENR, RCC->AHB3SMENR, 1);
  DBG_setIndentRel(-2);

  /********  APB1 peripheral clock enable register *****************/
  DBG_printf_indent("APB1 peripheral clocks in SleepMode\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_apb1enr(RCC->APB1SMENR1, RCC->APB1SMENR2, 1);
  DBG_setIndentRel(-2);

  /********  APB2 peripheral clock enable register *****************/
  DBG_printf_indent("APB2 peripheral clocks in SleepMode\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_apb2enr(RCC->APB2SMENR, 1);
  DBG_setIndentRel(-2);

  DBG_setIndentAbs(oldIndent);
}

const char * const pclk_txt[]={"APB Clk", "SYSCLK", "HSI16 Clk", "LSE clk" };
static const char* DBG_get_perip_clksrc_txt(uint32_t sel, uint32_t bSel4, uint32_t bSpecial)
{
  /* 
   * Clock Source 01 on LPTIM1 is deviating from normal bitcode schema
   * so handle specially
   */
  if ( bSpecial && sel == 1 ) return "LSI Clk";
   
  if ( sel < sizeof(pclk_txt)/sizeof(char *) - ( bSel4 ? 0 : 1 ) ) 
    return pclk_txt[sel];
  else
    return "Illegal";
}


const char * const adcclk_txt[]={"No Clock","PLLSAI1-R","PLLSAI2-R","SYSCLK"};
static const char* DBG_get_rcc_ccipr_adcclk_txt(uint32_t sel)
{
  if ( sel < sizeof(adcclk_txt)/sizeof(char *) ) 
    return adcclk_txt[sel];
  else
    return "Illegal";
}

const char *clk48_txt[]={"No Clock","PLLSAI1-Q","PLL-Q","MSI"};
static const char* DBG_get_rcc_ccipr_clk48_txt(uint32_t sel)
{
  if ( sel < sizeof(clk48_txt)/sizeof(char *) ) 
    return clk48_txt[sel];
  else
    return "Illegal";
}

/* NB: HSI as SAI clock source only on L4Sx and Rx types */
const char *saiclk_txt[]={"PLLSAI1-P","PLLSAI2-P","PLL-P","ext.Clk","HSI"};
static const char* DBG_get_rcc_ccipr_saiclk_txt(uint32_t sel)
{
  if ( sel < sizeof(saiclk_txt)/sizeof(char *) ) 
    return saiclk_txt[sel];
  else
    return "Illegal";
}



#if defined(STM32L476xx) || defined(STM32L496xx)
static void DBG_dump_peripheralclock_specific(void)
{
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_DFSDM1EN ) )
    DBG_dump_textvalue("DFSDM1 Clk Source", READ_BIT(RCC->CCIPR , RCC_CCIPR_DFSDM1SEL) ? "SYSCLK" : "APB2CLK" );    
  if ( READ_BIT(RCC->APB1ENR2,RCC_APB1ENR2_SWPMI1EN ) )
    DBG_dump_textvalue("SWPMI1 Clk Source", READ_BIT(RCC->CCIPR , RCC_CCIPR_SWPMI1SEL) ? "HSI16" : "APB1CLK" );    

  /* CLK48 used by sdmmc, usb and rng */
  if ( READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SDMMC1EN) || READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN) || READ_BIT(RCC->APB2ENR,RCC_AHB2ENR_RNGEN) ) 
    DBG_dump_textvalue("CLK48 Source", DBG_get_rcc_ccipr_clk48_txt((RCC->CCIPR & RCC_CCIPR_CLK48SEL_Msk) >> RCC_CCIPR_CLK48SEL_Pos) );    
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI2EN ) )
    DBG_dump_textvalue("SAI2 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR & RCC_CCIPR_SAI2SEL_Msk) >> RCC_CCIPR_SAI2SEL_Pos) );    
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI1EN ) )
    DBG_dump_textvalue("SAI1 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR & RCC_CCIPR_SAI1SEL_Msk) >> RCC_CCIPR_SAI1SEL_Pos) );    
}
#elif defined(STM32L4Sxxx)
const char *ospiclk_txt[]={"HCLK","MSI","PLL-Q"};
static const char* DBG_get_rcc_ccipr_ospiclk_txt(uint32_t sel)
{
  if ( sel < sizeof(ospiclk_txt)/sizeof(char *) ) 
    return ospiclk_txt[sel];
  else
    return "Illegal";
}

static void DBG_dump_peripheralclock_specific(void)
{
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_DFSDM1EN ) )
    DBG_dump_textvalue("DFSDM1 Clk Source", READ_BIT(RCC->CCIPR2 , RCC_CCIPR2_DFSDM1SEL) ? "SYSCLK" : "APB2CLK" );    

/* CLK48 used by sdmmc, usb and rng */
#if defined( RCC_AHB2ENR_SDMMC2EN)
  if ( READ_BIT(RCC->AHB2ENR,(RCC_AHB2ENR_SDMMC2EN | RCC_AHB2ENR_SDMMC1EN)) || READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN) || READ_BIT(RCC->APB2ENR,RCC_AHB2ENR_RNGEN) ) 
#else
  if ( READ_BIT(RCC->AHB2ENR,RCC_AHB2ENR_SDMMC1EN) || READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN) || READ_BIT(RCC->APB2ENR,RCC_AHB2ENR_RNGEN) ) 
#endif
  {
      DBG_dump_textvalue("CLK48 Source", DBG_get_rcc_ccipr_clk48_txt((RCC->CCIPR & RCC_CCIPR_CLK48SEL_Msk) >> RCC_CCIPR_CLK48SEL_Pos) );    
#if defined( RCC_AHB2ENR_SDMMC2EN)
      if ( READ_BIT(RCC->AHB2ENR,(RCC_AHB2ENR_SDMMC2EN | RCC_AHB2ENR_SDMMC1EN)) ) 
#else
      if ( READ_BIT(RCC->AHB2ENR,RCC_AHB2ENR_SDMMC1EN) ) 
#endif
      {
          DBG_dump_textvalue("SDMMC Clk Source", READ_BIT(RCC->CCIPR2 , RCC_CCIPR2_SDMMCSEL) ? "PLL-P" : "CLK48" );    
      }
  }
  if ( READ_BIT(RCC->APB2ENR,RCC_APB2ENR_DSIEN) ) {
      DBG_dump_textvalue("DSI Clk Source", READ_BIT(RCC->CCIPR2 , RCC_CCIPR2_DSISEL) ? "PLL-DSI" : "DSI-PHY" );    
  }

  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI2EN ) )
    DBG_dump_textvalue("SAI2 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR2 & RCC_CCIPR2_SAI2SEL_Msk) >> RCC_CCIPR2_SAI2SEL_Pos) );    
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI1EN ) )
    DBG_dump_textvalue("SAI1 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR & RCC_CCIPR2_SAI1SEL_Msk) >> RCC_CCIPR2_SAI1SEL_Pos) );   
  if ( READ_BIT(RCC->AHB3ENR, ( RCC_AHB3ENR_OSPI1EN | RCC_AHB3ENR_OSPI2EN) ) )
    DBG_dump_textvalue("OCTOSPI Clk Source", DBG_get_rcc_ccipr_ospiclk_txt((RCC->CCIPR2 & RCC_CCIPR2_OSPISEL_Msk) >> RCC_CCIPR2_OSPISEL_Pos) );   
  if ( READ_BIT(RCC->AHB3ENR, ( RCC_AHB3ENR_OSPI1EN | RCC_AHB3ENR_OSPI2EN) ) )
    DBG_dump_textvalue("OCTOSPI Clk Source", DBG_get_rcc_ccipr_ospiclk_txt((RCC->CCIPR2 & RCC_CCIPR2_OSPISEL_Msk) >> RCC_CCIPR2_OSPISEL_Pos) );   
  /* RHB ToDo: PLLSAI2DIVR, ADFSDMSEL, DFSDMSEL, I2C4SEL */
}
#else
    #error "No hardware specific peripheral clock dump implemented"
#endif

void DBG_dump_peripheralclockconfig(void)
{
  DEBUG_PUTS("Peripheral clock configuration ------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  DBG_setPadLen(18);
    
  DBG_dump_peripheralclock_specific();

  if ( READ_BIT(RCC->AHB2ENR,RCC_AHB2ENR_ADCEN ) )
    DBG_dump_textvalue("ADC Clk Source", DBG_get_rcc_ccipr_adcclk_txt((RCC->CCIPR & RCC_CCIPR_ADCSEL_Msk) >> RCC_CCIPR_ADCSEL_Pos) );    
  if ( READ_BIT(RCC->APB1ENR2, RCC_APB1ENR2_LPTIM2EN ) )
       DBG_dump_textvalue("LPTIM2Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_LPTIM2SEL_Msk) >> RCC_CCIPR_LPTIM2SEL_Pos,1,1) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_LPTIM1EN ) )
       DBG_dump_textvalue("LPTIM1Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_LPTIM1SEL_Msk) >> RCC_CCIPR_LPTIM1SEL_Pos,1,1) ); 

  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN ) )
       DBG_dump_textvalue("I2C3 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_I2C3SEL_Msk) >> RCC_CCIPR_I2C3SEL_Pos,0,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN ) )
       DBG_dump_textvalue("I2C2 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_I2C2SEL_Msk) >> RCC_CCIPR_I2C2SEL_Pos,0,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN ) )
       DBG_dump_textvalue("I2C1 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_I2C1SEL_Msk) >> RCC_CCIPR_I2C1SEL_Pos,0,0) ); 
  if ( READ_BIT(RCC->APB1ENR2, RCC_APB1ENR2_LPUART1EN ) )
       DBG_dump_textvalue("LPUART1 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_LPUART1SEL_Msk) >> RCC_CCIPR_LPUART1SEL_Pos,1,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART5EN ) )
       DBG_dump_textvalue("UART5 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_UART5SEL_Msk) >> RCC_CCIPR_UART5SEL_Pos,1,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART4EN ) )
       DBG_dump_textvalue("UART4 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_UART4SEL_Msk) >> RCC_CCIPR_UART4SEL_Pos,1,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART3EN ) )
       DBG_dump_textvalue("USART3 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_USART3SEL_Msk) >> RCC_CCIPR_USART3SEL_Pos,1,0) ); 
  if ( READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN ) )
       DBG_dump_textvalue("USART2 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_USART2SEL_Msk) >> RCC_CCIPR_USART2SEL_Pos,1,0) ); 
  if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN ) )
       DBG_dump_textvalue("USART1 Clk source", DBG_get_perip_clksrc_txt((RCC->CCIPR & RCC_CCIPR_USART1SEL_Msk) >> RCC_CCIPR_USART1SEL_Pos,1,0) ); 

  DBG_setIndentAbs(oldIndent);
}

void DBG_dump_rtcclockconfig(void)
{
  DEBUG_PUTS("Peripheral clock configuration ------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  DBG_setPadLen(18);

  DBG_printf_indent("Not implemented yet");

  DBG_setIndentAbs(oldIndent);
}

#endif // #if DEBUG_FEATURES > 0

/**
  * @}
  */
