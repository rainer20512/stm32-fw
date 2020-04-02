/**
  ******************************************************************************
  * @file    debug_pwr_rcc.c
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

#include "hardware.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "debug_helper.h"
#include "rtc.h"

#define DEBUG_DUMP_PWR      1
#define DEBUG_DUMP_CLOCK    1
#define DEBUG_DUMP_PERCLK   0

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

const char * const vos_txt[]={"Reserved","Scale 3: 0.95V - 1.05V", "Scale 2: 1.05V - 1.15V","Scale 1: 1.15V - 1.26V"};
static const char* DBG_get_pwr_vos_txt(uint32_t sel)
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
const char * als_txt[]={"1.7V", "2.1V", "2.5V", "2.8V" };
static const char* get_pwr_cr1_als_txt(uint32_t sel)
{
  if ( sel < sizeof(als_txt)/sizeof(char *) ) 
    return als_txt[sel];
  else
    return "Illegal";
}

const char * svos_txt[]={"Reserved", "Scale5", "Scale4", "Scale3" };
static const char* get_pwr_cr1_svos_txt(uint32_t sel)
{
  if ( sel < sizeof(svos_txt)/sizeof(char *) ) 
    return svos_txt[sel];
  else
    return "Illegal";
}

const char * const pls_txt[]={"1.95V", "2.1V", "2.25V", "2.4V", "2.55V", "2.7V", "2.85V", "PB7 Ain" };
static const char* get_pwr_cr1_pls_txt(uint32_t sel)
{
  if ( sel < sizeof(pls_txt)/sizeof(char *) ) 
    return pls_txt[sel];
  else
    return "Illegal";
}


#if DEBUG_DUMP_PWR > 0
static void DBG_dump_pwr_cr1(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("Analog Voltage Det.lvl", PWR->CR1, PWR_CR1_AVDEN);
  DBG_dump_textvalue("Analog Voltage Level", get_pwr_cr1_als_txt(( PWR->CR1 & PWR_CR1_ALS_Msk  ) >> PWR_CR1_ALS_Pos) );
  DBG_dump_textvalue("Vscale in StopMode", get_pwr_cr1_svos_txt(( PWR->CR1 & PWR_CR1_SVOS_Msk  ) >> PWR_CR1_SVOS_Pos) );
  DBG_dump_textvalue("Flash in D1 StopMode ", READ_BIT(PWR->CR1, PWR_CR1_FLPS) ? "LowPower Mode" : "Normal Mode" );
  DBG_dump_bitvalue("Disable BkUp Wr Protect", PWR->CR1, PWR_CR1_DBP);
  DBG_dump_bitvalue("Digital Voltage Det.lvl", PWR->CR1, PWR_CR1_PLS);
  DBG_dump_textvalue("Digital Voltage Level", get_pwr_cr1_pls_txt(( PWR->CR1 & PWR_CR1_PLS_Msk  ) >> PWR_CR1_PLS_Pos) );
  DBG_dump_textvalue("V.lvl in Stop@SVOS3", READ_BIT(PWR->CR1, PWR_CR1_LPDS) ? "LowPower Mode" : "Main Mode" );
}

static void DBG_dump_pwr_csr1(void)
{
  DBG_setPadLen(24);
  DBG_dump_textvalue("Analog Voltage Status", READ_BIT(PWR->CSR1, PWR_CSR1_AVDO) ? "< AV lvl" : ">= AV lvl" );
  DBG_dump_textvalue("current VOS lvl", DBG_get_pwr_vos_txt((PWR->CSR1 & PWR_CSR1_ACTVOS_Msk) >> PWR_CSR1_ACTVOS_Pos));
  DBG_dump_bitvalue( "VOS lvl valid ", PWR->CSR1, PWR_CSR1_ACTVOSRDY);
  DBG_dump_textvalue("Digit. Voltage Status", READ_BIT(PWR->CSR1, PWR_CSR1_PVDO) ? "< V lvl" : ">= V lvl" );
}

static void DBG_dump_pwr_cr3(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("VddUSB rdy", PWR->CR3, PWR_CR3_USB33RDY);
  DBG_dump_bitvalue("VddUSB Regulator on", PWR->CR3, PWR_CR3_USBREGEN);
  DBG_dump_bitvalue("VddUSB V. detect on", PWR->CR3, PWR_CR3_USB33DEN);
  DBG_dump_bitvalue("Ext. SMPS rdy", PWR->CR3, PWR_CR3_SMPSEXTRDY);
  DBG_dump_textvalue("Vbat chrg resistor", READ_BIT(PWR->CR3, PWR_CR3_VBRS) ? "1k5" : "5k0" );
  DBG_dump_bitvalue("Vbat chrg enable", PWR->CR3, PWR_CR3_VBE);
  DBG_dump_bitvalue("SMPS stepdown enable", PWR->CR3, PWR_CR3_SMPSEN);
  DBG_dump_bitvalue("LDO enable", PWR->CR3, PWR_CR3_LDOEN);
  DBG_dump_bitvalue("PWR mgmt bypass", PWR->CR3, PWR_CR3_BYPASS);
}


static void DBG_dump_pwr_cpu1cr(void)
{
  DBG_setPadLen(24);
  DBG_dump_textvalue("D3 Stop", READ_BIT(PWR->CPUCR, PWR_CPUCR_RUN_D3) ? "keep running" : "Stop when D1 stops" );    
  DBG_dump_bitvalue("Hold CPU2 after Stop", PWR->CPUCR, PWR_CPUCR_HOLD2);
  DBG_dump_bitvalue("D2 standby flag", PWR->CPUCR, PWR_CPUCR_SBF_D2);
  DBG_dump_bitvalue("D1 standby flag", PWR->CPUCR, PWR_CPUCR_SBF_D1);
  DBG_dump_bitvalue("System standby flag", PWR->CPUCR, PWR_CPUCR_SBF);
  DBG_dump_bitvalue("System stop flag", PWR->CPUCR, PWR_CPUCR_STOPF);
  DBG_dump_bitvalue("D2 on hold flag", PWR->CPUCR, PWR_CPUCR_HOLD2F);
  DBG_dump_textvalue("D3 on PDD", READ_BIT(PWR->CPUCR, PWR_CPUCR_PDDS_D3) ? "allow Standby" : "continue StopMode" );    
  DBG_dump_textvalue("D2 on PDD", READ_BIT(PWR->CPUCR, PWR_CPUCR_PDDS_D2) ? "allow Standby" : "continue StopMode" );    
  DBG_dump_textvalue("D1 on PDD", READ_BIT(PWR->CPUCR, PWR_CPUCR_PDDS_D1) ? "allow Standby" : "continue StopMode" );    
}

static void DBG_dump_pwr_cpu2cr(void)
{
  DBG_setPadLen(24);
  DBG_dump_textvalue("D3 Stop", READ_BIT(PWR->CPU2CR, PWR_CPU2CR_RUN_D3) ? "keep running" : "Stop when D2 stops" );    
  DBG_dump_bitvalue("Hold CPU1 after Stop", PWR->CPU2CR, PWR_CPU2CR_HOLD1);
  DBG_dump_bitvalue("D2 standby flag", PWR->CPU2CR, PWR_CPU2CR_SBF_D2);
  DBG_dump_bitvalue("D1 standby flag", PWR->CPU2CR, PWR_CPU2CR_SBF_D1);
  DBG_dump_bitvalue("System standby flag", PWR->CPU2CR, PWR_CPU2CR_SBF);
  DBG_dump_bitvalue("System stop flag", PWR->CPU2CR, PWR_CPU2CR_STOPF);
  DBG_dump_bitvalue("D1 on hold flag", PWR->CPU2CR, PWR_CPU2CR_HOLD1F);
  DBG_dump_textvalue("D3 on PDD", READ_BIT(PWR->CPU2CR, PWR_CPU2CR_PDDS_D3) ? "allow Standby" : "continue StopMode" );    
  DBG_dump_textvalue("D2 on PDD", READ_BIT(PWR->CPU2CR, PWR_CPU2CR_PDDS_D2) ? "allow Standby" : "continue StopMode" );    
  DBG_dump_textvalue("D1 on PDD", READ_BIT(PWR->CPU2CR, PWR_CPU2CR_PDDS_D1) ? "allow Standby" : "continue StopMode" );    
}

static void DBG_dump_pwr_d3cr(void)
{
  DBG_setPadLen(24);
  DBG_dump_textvalue("current VOS lvl", DBG_get_pwr_vos_txt((PWR->D3CR & PWR_D3CR_VOS_Msk) >> PWR_D3CR_VOS_Pos));
  DBG_dump_bitvalue("VOS rdy", PWR->D3CR, PWR_D3CR_VOSRDY);
}


static void DBG_dump_pwr_wkupfr(void)
{
  DBG_setPadLen(24);
  DBG_dump_bitvalue("WkUp via WKUP0 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG1);
  DBG_dump_bitvalue("WkUp via WKUP1 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG2);
  DBG_dump_bitvalue("WkUp via WKUP2 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG3);
  DBG_dump_bitvalue("WkUp via WKUP3 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG4);
  DBG_dump_bitvalue("WkUp via WKUP4 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG5);
  DBG_dump_bitvalue("WkUp via WKUP5 Pin", PWR->WKUPFR, PWR_WAKEUP_FLAG6);
}

const char * const pupd_txt[]={"NoPull", "PU", "PD", "Resvd" };
static const char* get_pupd_txt(uint32_t sel)
{
  if ( sel < sizeof(pupd_txt)/sizeof(char *) ) 
    return pupd_txt[sel];
  else
    return "Illegal";
}

static void dump_one_wkup_pin( uint32_t ena, uint32_t pol, uint32_t pupd )
{
   DEBUG_PRINTF("%s %s %s\n",
        ( ena ? "Enabled " : "Disabled" ),
        ( pol ? "Falling" : "Rising "   ),
        get_pupd_txt( pupd )
   );
}
static void DBG_dump_pwr_wkupepr(void)
{
  DBG_setPadLen(24);
  uint32_t ena_pos = PWR_WKUPEPR_WKUPEN1;
  uint32_t pol_pos = PWR_WKUPEPR_WKUPP1;
  uint32_t pupd_mask = PWR_WKUPEPR_WKUPPUPD1_Msk;
  uint32_t pupd_pos = PWR_WKUPEPR_WKUPPUPD1_Pos;
  register uint32_t reg = PWR->WKUPEPR;
  for ( uint32_t i=0; i<6; i++ ) {
    DBG_printf_indent("WkUp pin %d config: ",i);
    dump_one_wkup_pin(reg & ena_pos, reg & pol_pos, (reg & pupd_mask ) >> pupd_pos);
    ena_pos  <<= 1;
    pol_pos  <<= 1;
    pupd_mask <<= 2;
    pupd_pos += 2;
  }
}

#endif

void DBG_dump_powersetting(void)
{
  DEBUG_PUTS("PWR Settings ------------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  #if DEBUG_DUMP_PWR > 0
      /********  PWR Control registers *****************/
      DBG_printf_indent("PWR: Power Control Register 1\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_cr1();
      DBG_setIndentRel(-2);

      DBG_printf_indent("PWR: Power Control&Status Register 1\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_csr1();
      DBG_setIndentRel(-2);

      DBG_printf_indent("PWR: Power Control Register 3 \n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_cr3();
      DBG_setIndentRel(-2);

      DBG_printf_indent("PWR: CPU1 Control Register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_cpu1cr();
      DBG_setIndentRel(-2);
 
      DBG_printf_indent("PWR: CPU2 Control Register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_cpu2cr();
      DBG_setIndentRel(-2);
 
      DBG_printf_indent("PWR: D3 Control Register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_d3cr();
      DBG_setIndentRel(-2);
 
      DBG_printf_indent("PWR: WakeUp Flag Register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_wkupfr();
      DBG_setIndentRel(-2);
 
      DBG_printf_indent("PWR: WakeUp Enable & Polarity Register\n" );
      DBG_setIndentRel(+2);
      DBG_dump_pwr_wkupepr();
      DBG_setIndentRel(-2);
 
  #else
      DBG_printf_indent("Dumping PWR settings configured out \n" );
  #endif
  DBG_setIndentAbs(oldIndent);
}



/*
 *************************************************************
 * Functions to dump RCC settings
 *************************************************************
 */


static void dbg_rcc_on_ready ( const char *instname, uint32_t reg, uint32_t on_bit, uint32_t ready_bit )
{
    DBG_dump_bitvalue2(instname, "on", reg, on_bit);
    if ( reg & on_bit ) DBG_dump_bitvalue2(instname, "ready", reg, ready_bit);    
}
/* ---- RCC CR ---- */
static void DBG_dump_rcc_cr(void)
{
  DBG_setPadLen(22);
  dbg_rcc_on_ready("PLL3", RCC->CR, RCC_CR_PLL3ON, RCC_CR_PLL2RDY );
  dbg_rcc_on_ready("PLL2", RCC->CR, RCC_CR_PLL2ON, RCC_CR_PLL2RDY );
  dbg_rcc_on_ready("PLL1", RCC->CR, RCC_CR_PLL1ON, RCC_CR_PLL1RDY );

  dbg_rcc_on_ready("HSE",  RCC->CR, RCC_CR_HSEON,  RCC_CR_HSERDY );
  if ( READ_BIT( RCC->CR, RCC_CR_HSEON ) ) {
    DBG_dump_bitvalue("HSE bypass", RCC->CR, RCC_CR_HSEBYP);
    DBG_dump_bitvalue("HSE CSS on", RCC->CR, RCC_CR_CSSHSEON);
  }

  DBG_dump_bitvalue("D2 Clock ready", RCC->CR, RCC_CR_D2CKRDY );
  DBG_dump_bitvalue("D1 Clock ready", RCC->CR, RCC_CR_D1CKRDY );

  dbg_rcc_on_ready("HSI48", RCC->CR, RCC_CR_HSI48ON, RCC_CR_HSI48RDY );

  dbg_rcc_on_ready("CSI", RCC->CR, RCC_CR_CSION, RCC_CR_CSIRDY );
  DBG_dump_bitvalue("CSI On in STOPmode", RCC->CR, RCC_CR_CSIKERON);

  dbg_rcc_on_ready("HSI", RCC->CR, RCC_CR_HSION, RCC_CR_HSIRDY );
  DBG_dump_bitvalue("HSI On in STOPmode", RCC->CR, RCC_CR_HSIKERON);
  if ( READ_BIT( RCC->CR, RCC_CR_HSION ) ) {
    DBG_dump_number("HSI Frequency (MHz)", 64 / ((( RCC->CR & RCC_CR_HSIDIV_Msk ) >>  RCC_CR_HSIDIV_Pos ) + 1) );
    DBG_dump_bitvalue("HSI frac divider", RCC->CR, RCC_CR_HSIDIVF);
  }
}

const char * const rtcsel_txt[]={"No Clock","LSE","LSI","HSE+RTC prediv"};
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

  DBG_dump_bitvalue("RTC enable", RCC->BDCR, RCC_BDCR_RTCEN );
  if ( READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) )
    DBG_dump_textvalue("RTC/LCD Clk source", DBG_get_rcc_bdcr_rtcsel_txt((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos));
    if ((RCC->BDCR & RCC_BDCR_RTCSEL_Msk)  == RCC_BDCR_RTCSEL_Msk ) {
        uint32_t rtc_presc = (RCC->CFGR & RCC_CFGR_RTCPRE_Msk) >> RCC_CFGR_RTCPRE_Pos;
        if ( rtc_presc < 2 )
            DBG_dump_textvalue("RTC/HSE prescaler","Disabled");
        else 
            DBG_dump_number("RTC/HSE prescaler", rtc_presc);
    }
      
  dbg_rcc_on_ready("LSE",  RCC->BDCR, RCC_BDCR_LSEON,  RCC_BDCR_LSERDY );
  if ( READ_BIT( RCC->BDCR, RCC_BDCR_LSEON ) ) {
    DBG_dump_bitvalue("LSE bypass", RCC->BDCR, RCC_BDCR_LSEBYP);
    DBG_dump_bitvalue("LSE CSS on", RCC->BDCR, RCC_BDCR_LSECSSON);
    DBG_dump_bitvalue("LSE CSS fault", RCC->BDCR, RCC_BDCR_LSECSSD);
    DBG_dump_number("LSE drv strength", (RCC->BDCR & RCC_BDCR_LSEDRV_Msk) >> RCC_BDCR_LSEDRV_Pos);
  }
}

/* ---- RCC HSICFGR ---- */
static void DBG_dump_rcc_hsicfgr(void)
{
  DBG_setPadLen(16);
  if ( READ_BIT(RCC->CR, RCC_CR_HSION) ) {
    DBG_dump_number("HSI Calibration", (RCC->HSICFGR & RCC_HSICFGR_HSICAL_Msk ) >> RCC_HSICFGR_HSICAL_Pos );
    DBG_dump_number("HSI Trimming", (RCC->HSICFGR & RCC_HSICFGR_HSITRIM_Msk ) >> RCC_HSICFGR_HSITRIM_Pos );
  }
}

/* ---- RCC CFGR ---- */
const char * const mosel1_txt[]={"HSI","LSE", "HSE","PLL1","HSI48"};
static const char* DBG_get_rcc_cfgr_mcosel1_txt(uint32_t sel)
{
  if ( sel < sizeof(mosel1_txt)/sizeof(char *) ) 
    return mosel1_txt[sel];
  else
    return "Illegal";
}

/*
 * decode prescaler values 
 */
static uint32_t count_bits_set (uint32_t arg )
{
    uint32_t ret = 0;

    for ( uint32_t i=0; i < 32; i++ ) {
        if ( arg & 0b1 ) ret++;
        arg >>= 1;
    }

    return ret;
}

static uint32_t decode_prescaler ( uint32_t prescaler, uint32_t relevant_bitnum )
{
    if ( relevant_bitnum == 0 ) return 0;
    uint32_t msb = 1 << ( relevant_bitnum - 1 );

    /* prescaler inactive ( or 1 ) when MSB is reset */
    if ( ( prescaler & msb ) == 0 ) return 1;
    
    /* reset msb */
    prescaler &= ~msb;

    return 1 << ( prescaler + 1 );
}

const char * const sws_txt[]={"HSI","CSI", "HSE", "PLL1"};
static const char* DBG_get_rcc_cfgr_sws_txt(uint32_t sel)
{
  if ( sel < sizeof(sws_txt)/sizeof(char *) ) 
    return sws_txt[sel];
  else
    return "Illegal";
}

static void DBG_dump_rcc_cfgr(void)
{
  DBG_setPadLen(20);
  uint32_t mcopre = ( RCC->CFGR & RCC_CFGR_MCO1PRE_Msk ) >> RCC_CFGR_MCO1PRE_Pos;

  if ( mcopre == 0 ) {
    DBG_printf_indent("MCO disabled\n" );    
  } else {
    DBG_dump_number("MCO prescaler", mcopre );
    DBG_dump_textvalue("MCO Source", DBG_get_rcc_cfgr_mcosel1_txt(( RCC->CFGR & RCC_CFGR_MCO1_Msk ) >> RCC_CFGR_MCO1_Pos) );
  }
  DBG_dump_textvalue("HRTIM Clksrc", READ_BIT(RCC->CFGR, RCC_CFGR_HRTIMSEL)    ? "CM7 Clk" : "APB1 Clk" );    
  DBG_dump_textvalue("System WkUp Clksrc", READ_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK)    ? "CSI" : "HSI" );    
  DBG_dump_textvalue("Kernel WkUp Clksrc", READ_BIT(RCC->CFGR, RCC_CFGR_STOPKERWUCK) ? "CSI" : "HSI" );    
  DBG_dump_textvalue("SysClk source", DBG_get_rcc_cfgr_sws_txt(( RCC->CFGR & RCC_CFGR_SWS_Msk  ) >> RCC_CFGR_SWS_Pos) );
}

/* ---- RCC PLLCFGR ---- */
const char * const pllsrc_txt[]={"HSI", "CSI", "HSE", "No Clock"};
static const char* DBG_get_rcc_pllcfgr_pllsrc_txt(uint32_t sel )
{
  if ( sel < sizeof(pllsrc_txt)/sizeof(char *) ) 
    return pllsrc_txt[sel];
  else
    return "Illegal";
}
static bool dump_pllconfig( RCC_PLLInitTypeDef *pll, uint32_t pllnum )
{
    switch ( pllnum ) {
        case 1:
            pll->PLLSource = (uint32_t)(RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
            pll->PLLM = (uint32_t) ((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1)>> RCC_PLLCKSELR_DIVM1_Pos);
            pll->PLLN = (uint32_t) ((RCC->PLL1DIVR & RCC_PLL1DIVR_N1)     >> RCC_PLL1DIVR_N1_Pos)+ 1U;
            pll->PLLR = (uint32_t) ((RCC->PLL1DIVR & RCC_PLL1DIVR_R1)     >> RCC_PLL1DIVR_R1_Pos)+ 1U;
            pll->PLLP = (uint32_t) ((RCC->PLL1DIVR & RCC_PLL1DIVR_P1)     >> RCC_PLL1DIVR_P1_Pos)+ 1U;
            pll->PLLQ = (uint32_t) ((RCC->PLL1DIVR & RCC_PLL1DIVR_Q1)     >> RCC_PLL1DIVR_Q1_Pos)+ 1U;
            pll->PLLRGE =(uint32_t)((RCC->PLLCFGR  & RCC_PLLCFGR_PLL1RGE) >> RCC_PLLCFGR_PLL1RGE_Pos);
            pll->PLLVCOSEL = (uint32_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLL1VCOSEL) >> RCC_PLLCFGR_PLL1VCOSEL_Pos);
            pll->PLLFRACN = (uint32_t)(((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1) >> RCC_PLL1FRACR_FRACN1_Pos));
           break;
        case 2:
            pll->PLLSource = (uint32_t)(RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
            pll->PLLM = (uint32_t)((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM2)>> RCC_PLLCKSELR_DIVM2_Pos);
            pll->PLLN = (uint32_t)((RCC->PLL2DIVR & RCC_PLL2DIVR_N2) >> RCC_PLL2DIVR_N2_Pos)+ 1U;
            pll->PLLR = (uint32_t)((RCC->PLL2DIVR & RCC_PLL2DIVR_R2) >> RCC_PLL2DIVR_R2_Pos)+ 1U;
            pll->PLLP = (uint32_t)((RCC->PLL2DIVR & RCC_PLL2DIVR_P2) >> RCC_PLL2DIVR_P2_Pos)+ 1U;
            pll->PLLQ = (uint32_t)((RCC->PLL2DIVR & RCC_PLL2DIVR_Q2) >> RCC_PLL2DIVR_Q2_Pos)+ 1U;
            pll->PLLRGE =(uint32_t)((RCC->PLLCFGR  & RCC_PLLCFGR_PLL2RGE) >> RCC_PLLCFGR_PLL2RGE_Pos);
            pll->PLLVCOSEL = (uint32_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLL2VCOSEL) >> RCC_PLLCFGR_PLL2VCOSEL_Pos);
            pll->PLLFRACN = (uint32_t)(((RCC->PLL2FRACR & RCC_PLL2FRACR_FRACN2) >> RCC_PLL2FRACR_FRACN2_Pos));
           break;
        case 3:
            pll->PLLSource = (uint32_t)(RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
            pll->PLLM = (uint32_t)((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM3)>> RCC_PLLCKSELR_DIVM3_Pos);
            pll->PLLN = (uint32_t)((RCC->PLL3DIVR & RCC_PLL3DIVR_N3) >> RCC_PLL3DIVR_N3_Pos)+ 1U;
            pll->PLLR = (uint32_t)((RCC->PLL3DIVR & RCC_PLL3DIVR_R3) >> RCC_PLL3DIVR_R3_Pos)+ 1U;
            pll->PLLP = (uint32_t)((RCC->PLL3DIVR & RCC_PLL3DIVR_P3) >> RCC_PLL3DIVR_P3_Pos)+ 1U;
            pll->PLLQ = (uint32_t)((RCC->PLL3DIVR & RCC_PLL3DIVR_Q3) >> RCC_PLL3DIVR_Q3_Pos)+ 1U;
            pll->PLLRGE =(uint32_t)((RCC->PLLCFGR  & RCC_PLLCFGR_PLL3RGE) >> RCC_PLLCFGR_PLL3RGE_Pos);
            pll->PLLVCOSEL = (uint32_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLL3VCOSEL) >> RCC_PLLCFGR_PLL3VCOSEL_Pos);
            pll->PLLFRACN = (uint32_t)(((RCC->PLL3FRACR & RCC_PLL3FRACR_FRACN3) >> RCC_PLL3FRACR_FRACN3_Pos));
           break;
        default:
            DBG_printf_indent("Missing decoder for PLL %d\n",pllnum);
            return false;
    }

    DBG_printf_indent("PLL%d configuration\n", pllnum );
    DBG_dump_textvalue("PLL source", DBG_get_rcc_pllcfgr_pllsrc_txt( pll->PLLSource >> RCC_PLLCKSELR_PLLSRC_Pos ) );  
    DBG_dump_number("PLLM", pll->PLLM );  
    DBG_dump_number("PLLN", pll->PLLN );  
    DBG_dump_number("PLLR", pll->PLLR );  
    DBG_dump_number("PLLQ", pll->PLLQ );  
    DBG_dump_number("PLLP", pll->PLLP );  
    DBG_dump_number("PLLRGE", pll->PLLRGE );
    DBG_dump_number("PLLVCOSEL", pll->PLLVCOSEL );
    DBG_dump_number("PLLFRACN", pll->PLLFRACN ); 
    return true;
} 

static void DBG_dump_rcc_pllcfgr(void)
{
  RCC_PLLInitTypeDef pll;
  DBG_setPadLen(20);

  if ( READ_BIT(RCC->CR, RCC_CR_PLL1ON ) ) {
    dump_pllconfig(&pll, 1);
  } else  {
    DBG_printf_indent("PLL1 disabled\n" );    
  }
  if ( READ_BIT(RCC->CR, RCC_CR_PLL2ON ) ) {
    dump_pllconfig(&pll, 2);
  } else  {
    DBG_printf_indent("PLL2 disabled\n" );    
  }
  if ( READ_BIT(RCC->CR, RCC_CR_PLL3ON ) ) {
    dump_pllconfig(&pll, 3);
  } else  {
    DBG_printf_indent("PLL3 disabled\n" );    
  }
}

static void DBG_dump_rcc_prescalers(void)
{
  DBG_setPadLen(20);
  DBG_dump_number("D1Clk prescaler",   decode_prescaler (  (RCC->D1CFGR & RCC_D1CFGR_D1CPRE_Msk) >> RCC_D1CFGR_D1CPRE_Pos, count_bits_set(RCC_D1CFGR_D1CPRE_Msk) ) );
  DBG_dump_number("AHB Clk prescaler", decode_prescaler (  (RCC->D1CFGR & RCC_D1CFGR_HPRE_Msk  ) >> RCC_D1CFGR_HPRE_Pos,   count_bits_set(RCC_D1CFGR_HPRE_Msk)   ) );
  DBG_dump_number("APB3 Clk prescaler", decode_prescaler( (RCC->D1CFGR & RCC_D1CFGR_D1PPRE_Msk ) >> RCC_D1CFGR_D1PPRE_Pos, count_bits_set(RCC_D1CFGR_D1PPRE_Msk) ) );
  DBG_dump_number("APB1 Clk prescaler", decode_prescaler( (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_Msk) >> RCC_D2CFGR_D2PPRE1_Pos,count_bits_set(RCC_D2CFGR_D2PPRE1_Msk)) );
  DBG_dump_number("APB2 Clk prescaler", decode_prescaler( (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2_Msk) >> RCC_D2CFGR_D2PPRE2_Pos,count_bits_set(RCC_D2CFGR_D2PPRE2_Msk)) );
  DBG_dump_number("APB4 Clk prescaler", decode_prescaler( (RCC->D3CFGR & RCC_D3CFGR_D3PPRE_Msk ) >> RCC_D3CFGR_D3PPRE_Pos, count_bits_set(RCC_D3CFGR_D3PPRE_Msk) ) );
        
  
}

static void DBG_dump_clocks(void)
{
  RCC_ClkInitTypeDef v;
  uint32_t f_latency;

  DBG_setPadLen(16);
  DBG_dump_number("SYSCLK", HAL_RCC_GetSysClockFreq() );
  DBG_dump_number("HCLK",   HAL_RCC_GetHCLKFreq());
  DBG_dump_number("PCLK1",  HAL_RCC_GetPCLK1Freq());
  DBG_dump_number("PCLK2",  HAL_RCC_GetPCLK2Freq());
  HAL_RCC_GetClockConfig(&v, &f_latency);
  DBG_dump_textvalue("Vcore value", DBG_get_pwr_vos_txt(( PWR->D3CR & PWR_D3CR_VOS_Msk  ) >> PWR_D3CR_VOS_Pos) );
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
  if (  READ_BIT(RCC->CR, RCC_CR_HSION ) ) {
    DBG_printf_indent("RCC: HSI Calibration Register\n" );
    DBG_setIndentRel(+2);
    DBG_dump_rcc_hsicfgr();
    DBG_setIndentRel(-2);
  }

  /******** Clock configuration register *****************/
  DBG_printf_indent("RCC: Clock configuration register\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_cfgr();
  DBG_setIndentRel(-2);

  /******** PLL configuration register *****************/
  DBG_printf_indent("RCC: PLL configuration\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_pllcfgr();
  DBG_setIndentRel(-2);

  /******** Clock prescaler registers *****************/
  DBG_printf_indent("RCC: Clock prescalers\n" );
  DBG_setIndentRel(+2);
  DBG_dump_rcc_prescalers();
  DBG_setIndentRel(-2);

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
#if DEBUG_DUMP_PERCLK > 0
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

  DBG_dump_onoffvalue  ("TSC Clock",   reg1, RCC_AHB1ENR_TSCEN);  
  DBG_dump_onoffvalue  ("CRC Clock",   reg1, RCC_AHB1ENR_CRCEN);  
  if ( bSleepRegisters)   DBG_dump_onoffvalue  ("SRAM1 Clock", reg1, RCC_AHB1SMENR_SRAM1SMEN);  
  DBG_dump_onoffvalue  ("Flash Clock", reg1, RCC_AHB1ENR_FLASHEN);  
  DBG_dump_onoffvalue  ("DMA2 Clock",  reg1, RCC_AHB1ENR_DMA2EN);  
  DBG_dump_onoffvalue  ("DMA1 Clock",  reg1, RCC_AHB1ENR_DMA1EN);  

  DBG_dump_onoffvalue  ("RNG Clock",  reg2, RCC_AHB2ENR_RNGEN);  
  DBG_dump_onoffvalue  ("ADC Clock",  reg2, RCC_AHB2ENR_ADCEN);  
  DBG_dump_onoffvalue  ("OTGFS Clock",  reg2, RCC_AHB2ENR_OTGFSEN);  
  if ( bSleepRegisters)   DBG_dump_onoffvalue  ("SRAM2 Clock", reg2, RCC_AHB2SMENR_SRAM2SMEN);  
  DBG_dump_onoffvalue  ("GPIOH Clock",  reg2, RCC_AHB2ENR_GPIOHEN);  
  DBG_dump_onoffvalue  ("GPIOG Clock",  reg2, RCC_AHB2ENR_GPIOGEN);  
  DBG_dump_onoffvalue  ("GPIOF Clock",  reg2, RCC_AHB2ENR_GPIOFEN);  
  DBG_dump_onoffvalue  ("GPIOE Clock",  reg2, RCC_AHB2ENR_GPIOEEN);  
  DBG_dump_onoffvalue  ("GPIOD Clock",  reg2, RCC_AHB2ENR_GPIODEN);  
  DBG_dump_onoffvalue  ("GPIOC Clock",  reg2, RCC_AHB2ENR_GPIOCEN);  
  DBG_dump_onoffvalue  ("GPIOB Clock",  reg2, RCC_AHB2ENR_GPIOBEN);  
  DBG_dump_onoffvalue  ("GPIOA Clock",  reg2, RCC_AHB2ENR_GPIOAEN);  

  DBG_dump_onoffvalue  ("QSPI Clock",  reg3, RCC_AHB3ENR_QSPIEN);  
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
  DBG_dump_onoffvalue  ("LCD Clock", reg1, RCC_APB1ENR1_LCDEN);  
  DBG_dump_onoffvalue  ("TIM7 Clock", reg1, RCC_APB1ENR1_TIM7EN);  
  DBG_dump_onoffvalue  ("TIM6 Clock", reg1, RCC_APB1ENR1_TIM6EN);  
  DBG_dump_onoffvalue  ("TIM5 Clock", reg1, RCC_APB1ENR1_TIM5EN);  
  DBG_dump_onoffvalue  ("TIM4 Clock", reg1, RCC_APB1ENR1_TIM4EN);  
  DBG_dump_onoffvalue  ("TIM3 Clock", reg1, RCC_APB1ENR1_TIM3EN);  
  DBG_dump_onoffvalue  ("TIM2 Clock", reg1, RCC_APB1ENR1_TIM2EN);  

  DBG_dump_onoffvalue  ("LPTIM2 Clock", reg2, RCC_APB1ENR2_LPTIM2EN);  
  DBG_dump_onoffvalue  ("SWPMI1 Clock", reg2, RCC_APB1ENR2_SWPMI1EN);  
  DBG_dump_onoffvalue  ("LPUART1 Clock", reg2, RCC_APB1ENR2_LPUART1EN);  
} 

void DBG_dump_rcc_apb2enr(uint32_t reg, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "APB2SMENR raw" : "APB2ENR raw",reg );  
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
  DBG_dump_onoffvalue  ("SDMMC1 Clock", reg, RCC_APB2ENR_SDMMC1EN);  
  DBG_dump_onoffvalue  ("FW Clock", reg, RCC_APB2ENR_FWEN);  
  DBG_dump_onoffvalue  ("SYSCFG Clock", reg, RCC_APB2ENR_SYSCFGEN);  
}
#endif /* #if DEBUG_DUMP_PERCLK > 0 */

void DBG_dump_peripheralclocksetting(void)
{
  DEBUG_PUTS("Peripheral Clock Settings -----------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  #if DEBUG_DUMP_PERCLK > 0
      /********  AHB peripheral clock enable register *****************/
      DBG_printf_indent("AHB peripheral clocks\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_ahbenr(RCC->AHB1ENR, RCC->AHB2ENR, RCC->AHB2ENR, 0);
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
  #else
      DBG_printf_indent("configured out\n");
  #endif
  DBG_setIndentAbs(oldIndent);
}

void DBG_dump_peripheralclocksetting_insleepmode(void)
{
  DEBUG_PUTS("Peripheral Clock Settings ** IN SLEEP MODE ** ---" );
  int oldIndent = DBG_setIndentRel(+2);

  #if DEBUG_DUMP_PERCLK > 0
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
  #else
      DBG_printf_indent("configured out\n");
  #endif
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

const char *saiclk_txt[]={"PLLSAI1-P","PLLSAI2-P","PLL-P","ext.Clk"};
static const char* DBG_get_rcc_ccipr_saiclk_txt(uint32_t sel)
{
  if ( sel < sizeof(saiclk_txt)/sizeof(char *) ) 
    return saiclk_txt[sel];
  else
    return "Illegal";
}


void DBG_dump_peripheralclockconfig(void)
{
  DEBUG_PUTS("Peripheral clock configuration ------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  DBG_setPadLen(18);

  #if DEBUG_DUMP_PERCLK > 0
      if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_DFSDM1EN ) )
        DBG_dump_textvalue("DFSDM1 Clk Source", READ_BIT(RCC->CCIPR , RCC_CCIPR_DFSDM1SEL) ? "SYSCLK" : "APB2CLK" );    
      if ( READ_BIT(RCC->APB1ENR2,RCC_APB1ENR2_SWPMI1EN ) )
        DBG_dump_textvalue("SWPMI1 Clk Source", READ_BIT(RCC->CCIPR , RCC_CCIPR_SWPMI1SEL) ? "HSI16" : "APB1CLK" );    
      if ( READ_BIT(RCC->AHB2ENR,RCC_AHB2ENR_ADCEN ) )
        DBG_dump_textvalue("ADC Clk Source", DBG_get_rcc_ccipr_adcclk_txt((RCC->CCIPR & RCC_CCIPR_ADCSEL_Msk) >> RCC_CCIPR_ADCSEL_Pos) );    

      /* CLK48 used by sdmmc, usb and rng */
      if ( READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SDMMC1EN) || READ_BIT(RCC->APB2ENR, RCC_AHB2ENR_OTGFSEN) || READ_BIT(RCC->APB2ENR,RCC_AHB2ENR_RNGEN) ) 
        DBG_dump_textvalue("CLK48 Source", DBG_get_rcc_ccipr_clk48_txt((RCC->CCIPR & RCC_CCIPR_CLK48SEL_Msk) >> RCC_CCIPR_CLK48SEL_Pos) );    
      if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI2EN ) )
        DBG_dump_textvalue("SAI2 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR & RCC_CCIPR_SAI2SEL_Msk) >> RCC_CCIPR_SAI2SEL_Pos) );    
      if ( READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI1EN ) )
        DBG_dump_textvalue("SAI1 Clk Source", DBG_get_rcc_ccipr_saiclk_txt((RCC->CCIPR & RCC_CCIPR_SAI1SEL_Msk) >> RCC_CCIPR_SAI1SEL_Pos) );    

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
  #else
      DBG_printf_indent("configured out\n");
  #endif

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
