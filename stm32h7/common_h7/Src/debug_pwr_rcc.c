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
#define DEBUG_DUMP_PERCLK   1

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

#if 0
const char * const lpms_txt[]={"Stop 0", "Stop 1", "Stop 2", "Standby" };
static const char* DBG_get_pwr_cr1_lpms_txt(uint32_t sel)
{
  if ( sel > 3 )  return "Shutdown";
    
  return lpms_txt[sel];
}
#endif
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
  if ( READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) ) {
    DBG_dump_textvalue("RTC/LCD Clk source", DBG_get_rcc_bdcr_rtcsel_txt((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos));
    if ((RCC->BDCR & RCC_BDCR_RTCSEL_Msk)  == RCC_BDCR_RTCSEL_Msk ) {
        uint32_t rtc_presc = (RCC->CFGR & RCC_CFGR_RTCPRE_Msk) >> RCC_CFGR_RTCPRE_Pos;
        if ( rtc_presc < 2 )
            DBG_dump_textvalue("RTC/HSE prescaler","Disabled");
        else 
            DBG_dump_number("RTC/HSE prescaler", rtc_presc);
    }
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

/* Flag for "dump all entries in 'DBG_dump_when_on', not only active ones" */ 
static bool bDumpAlways = false;

void DBG_dump_when_on( const char *txt, uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bitmask)
{
    char const *append;
    /* Only bump, if bit is set */
    if ( !bDumpAlways && (d1 & bitmask) == 0 ) return;
    
    if ( c2 & bitmask && c1 & bitmask ) 
        append = " CM7 CM4";
    else if ( c1 & bitmask )
        append = " CM7";
    else if ( c2 & bitmask )
        append = " CM4";
    else 
        append = "";
    DBG_dump_onoffvalue2(txt, d1, bitmask, append);
}

/*
 *************************************************************
 * The follwowing routines are used to dump ordinary and
 * sleep mode register settings, so the register is passed as 
 * parameter. This only works, because bit positions in 
 * ordinary and sleep mode registers are (nearly)identical
 *************************************************************
 */
void DBG_dump_rcc_ahb1enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   AHB1LPENR raw " : "   AHB1ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_AHB1LPENR raw " : "C1_AHB1ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_AHB1LPENR raw " : "C2_AHB1ENR raw ", c2 );  

  DBG_dump_when_on ("USB2OTG Clk",    d1, c1, c2, RCC_AHB1ENR_USB2OTGHSEN);  
  DBG_dump_when_on ("USB1 PHY Clk",   d1, c1, c2, RCC_AHB1ENR_USB1OTGHSULPIEN);  
  DBG_dump_when_on ("USB1OTG Clk",    d1, c1, c2, RCC_AHB1ENR_USB1OTGHSEN);  
  DBG_dump_when_on ("USB2 PHY Clk",   d1, c1, c2, RCC_AHB1ENR_USB2OTGHSULPIEN);  
  DBG_dump_when_on ("ETH1 Rx Clk",    d1, c1, c2, RCC_AHB1ENR_ETH1RXEN);  
  DBG_dump_when_on ("ETH1 Tx Clk",    d1, c1, c2, RCC_AHB1ENR_ETH1TXEN);  
  DBG_dump_when_on ("ETH1 MAC Clk",   d1, c1, c2, RCC_AHB1ENR_ETH1MACEN);  
  DBG_dump_when_on ("ART Clk",        d1, c1, c2, RCC_AHB1ENR_ARTEN);  
  DBG_dump_when_on ("ADC1,2 Clk",     d1, c1, c2, RCC_AHB1ENR_ADC12EN);  
  DBG_dump_when_on ("DMA2 Clk",       d1, c1, c2, RCC_AHB1ENR_DMA2EN);  
  DBG_dump_when_on ("DMA1 Clk",       d1, c1, c2, RCC_AHB1ENR_DMA1EN);
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_ahb2enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   AHB2LPENR raw " : "   AHB2ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_AHB2LPENR raw " : "C1_AHB2ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_AHB2LPENR raw " : "C2_AHB2ENR raw ", c2 );  

  DBG_dump_when_on ("SRAM3 Clk",   d1, c1, c2, RCC_AHB2ENR_SRAM3EN);  
  DBG_dump_when_on ("SRAM2 Clk",   d1, c1, c2, RCC_AHB2ENR_SRAM2EN);  
  DBG_dump_when_on ("SRAM1 Clk",   d1, c1, c2, RCC_AHB2ENR_SRAM1EN);  
  DBG_dump_when_on ("SDMMC2 Clk",  d1, c1, c2, RCC_AHB2ENR_SDMMC2EN);  
  DBG_dump_when_on ("RNG Clk",     d1, c1, c2, RCC_AHB2ENR_RNGEN);  
#if defined(HASH)
  DBG_dump_when_on ("HASH Clk",    d1, c1, c2, RCC_AHB2ENR_HASHEN);  
#endif
#if defined(CRYP)
  DBG_dump_when_on ("CRYPT Clk",   d1, c1, c2, RCC_AHB2ENR_CRYPEN);  
#endif
  DBG_dump_when_on ("DCMI Clk",    d1, c1, c2, RCC_AHB2ENR_DCMIEN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_ahb3enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   AHB3LPENR raw " : "   AHB3ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_AHB3LPENR raw " : "C1_AHB3ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_AHB3LPENR raw " : "C2_AHB3ENR raw ", c2 );  

  DBG_dump_when_on ("AXISRAM Clk",  d1, c1, c2, RCC_AHB3ENR_AXISRAMEN);  
  DBG_dump_when_on ("ITCM Clk",     d1, c1, c2, RCC_AHB3ENR_ITCMEN);  
  DBG_dump_when_on ("DTCM1 Clk",    d1, c1, c2, RCC_AHB3ENR_DTCM1EN);  
  DBG_dump_when_on ("DTCM2 Clk",    d1, c1, c2, RCC_AHB3ENR_DTCM2EN);  
  DBG_dump_when_on ("SDMMC1 Clk",   d1, c1, c2, RCC_AHB3ENR_SDMMC1EN);  
  DBG_dump_when_on ("QSPI Clk",   d1, c1, c2, RCC_AHB3ENR_QSPIEN);  
  DBG_dump_when_on ("FMC Clk",   d1, c1, c2, RCC_AHB3ENR_FMCEN);  
  #if defined(FLITF)
    DBG_dump_when_on ("FLITF Clk",   d1, c1, c2, RCC_AHB3ENR_FLITFEN);  
  #endif
  DBG_dump_when_on ("JPGDEC Clk",   d1, c1, c2, RCC_AHB3ENR_JPGDECEN);  
  DBG_dump_when_on ("DMA2D Clk",   d1, c1, c2, RCC_AHB3ENR_DMA2DEN);  
  DBG_dump_when_on ("MDMA Clk",   d1, c1, c2, RCC_AHB3ENR_MDMAEN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_ahb4enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   AHB4LPENR raw " : "   AHB4ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_AHB4LPENR raw " : "C1_AHB4ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_AHB4LPENR raw " : "C2_AHB4ENR raw ", c2 );  

  DBG_dump_when_on ("BKPRAM Clk", d1, c1, c2, RCC_AHB4ENR_BKPRAMEN);  
  DBG_dump_when_on ("HSEM Clk",   d1, c1, c2, RCC_AHB4ENR_HSEMEN);  
  DBG_dump_when_on ("ADC3 Clk",   d1, c1, c2, RCC_AHB4ENR_ADC3EN);  
  DBG_dump_when_on ("BDMA Clk",   d1, c1, c2, RCC_AHB4ENR_BDMAEN);  
  DBG_dump_when_on ("CRC Clk",    d1, c1, c2, RCC_AHB4ENR_CRCEN);  
  DBG_dump_when_on ("GPIOK Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOKEN);  
  DBG_dump_when_on ("GPIOJ Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOJEN);  
  DBG_dump_when_on ("GPIOI Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOIEN);  
  DBG_dump_when_on ("GPIOH Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOHEN);  
  DBG_dump_when_on ("GPIOG Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOGEN);  
  DBG_dump_when_on ("GPIOF Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOFEN);  
  DBG_dump_when_on ("GPIOE Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOEEN);  
  DBG_dump_when_on ("GPIOD Clk",  d1, c1, c2, RCC_AHB4ENR_GPIODEN);  
  DBG_dump_when_on ("GPIOC Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOCEN);  
  DBG_dump_when_on ("GPIOB Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOBEN);  
  DBG_dump_when_on ("GPIOA Clk",  d1, c1, c2, RCC_AHB4ENR_GPIOAEN);  
  DEBUG_PRINTF("\n");
}
 
void DBG_dump_rcc_apb1henr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   APB1HLPENR raw " : "   APB1HENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_APB1HLPENR raw " : "C1_APB1HENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_APB1HLPENR raw " : "C2_APB1HENR raw ", c2 );  

  DBG_dump_when_on ("FDCAN Clk", d1, c1, c2, RCC_APB1HENR_FDCANEN);  
  DBG_dump_when_on ("MDIOS Clk", d1, c1, c2, RCC_APB1HENR_MDIOSEN);  
  DBG_dump_when_on ("OPAMP Clk", d1, c1, c2, RCC_APB1HENR_OPAMPEN);  
  DBG_dump_when_on ("SWPMI Clk", d1, c1, c2, RCC_APB1HENR_SWPMIEN);  
  DBG_dump_when_on ("CRS Clk", d1, c1, c2, RCC_APB1HENR_CRSEN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_apb1lenr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   APB1LLPENR raw " : "   APB1LENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_APB1LLPENR raw " : "C1_APB1LENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_APB1LLPENR raw " : "C2_APB1LENR raw ", c2 );  

  DBG_dump_when_on ("UART8 Clk",    d1, c1, c2, RCC_APB1LENR_UART8EN);  
  DBG_dump_when_on ("UART7 Clk",    d1, c1, c2, RCC_APB1LENR_UART7EN);  
  DBG_dump_when_on ("DAC12 Clk",    d1, c1, c2, RCC_APB1LENR_DAC12EN);  
  DBG_dump_when_on ("CEC Clk",      d1, c1, c2, RCC_APB1LENR_CECEN);  
  DBG_dump_when_on ("I2C3 Clk",     d1, c1, c2, RCC_APB1LENR_I2C3EN);  
  DBG_dump_when_on ("I2C2 Clk",     d1, c1, c2, RCC_APB1LENR_I2C2EN);  
  DBG_dump_when_on ("I2C1 Clk",     d1, c1, c2, RCC_APB1LENR_I2C1EN);  
  DBG_dump_when_on ("UART5 Clk",    d1, c1, c2, RCC_APB1LENR_UART5EN);  
  DBG_dump_when_on ("UART4 Clk",    d1, c1, c2, RCC_APB1LENR_UART4EN);  
  DBG_dump_when_on ("USART3 Clk",   d1, c1, c2, RCC_APB1LENR_USART3EN);  
  DBG_dump_when_on ("USART2 Clk",   d1, c1, c2, RCC_APB1LENR_USART2EN);  
  DBG_dump_when_on ("SPDIFRX Clk",  d1, c1, c2, RCC_APB1LENR_SPDIFRXEN);  
  DBG_dump_when_on ("SPI3 Clk",     d1, c1, c2, RCC_APB1LENR_SPI3EN);  
  DBG_dump_when_on ("SPI2 Clk",     d1, c1, c2, RCC_APB1LENR_SPI2EN);  
  DBG_dump_when_on ("WWDG2 Clk",    d1, c1, c2, RCC_APB1LENR_WWDG2EN);  
  DBG_dump_when_on ("LPTIM1 Clk",   d1, c1, c2, RCC_APB1LENR_LPTIM1EN);  
  DBG_dump_when_on ("TIM14 Clk",    d1, c1, c2, RCC_APB1LENR_TIM14EN);  
  DBG_dump_when_on ("TIM13 Clk",    d1, c1, c2, RCC_APB1LENR_TIM13EN);  
  DBG_dump_when_on ("TIM12 Clk",    d1, c1, c2, RCC_APB1LENR_TIM12EN);  
  DBG_dump_when_on ("TIM7 Clk",     d1, c1, c2, RCC_APB1LENR_TIM7EN);  
  DBG_dump_when_on ("TIM6 Clk",     d1, c1, c2, RCC_APB1LENR_TIM6EN);  
  DBG_dump_when_on ("TIM5 Clk",     d1, c1, c2, RCC_APB1LENR_TIM5EN);  
  DBG_dump_when_on ("TIM4 Clk",     d1, c1, c2, RCC_APB1LENR_TIM4EN);  
  DBG_dump_when_on ("TIM3 Clk",     d1, c1, c2, RCC_APB1LENR_TIM3EN);  
  DBG_dump_when_on ("TIM2 Clk",     d1, c1, c2, RCC_APB1LENR_TIM2EN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_apb2enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   APB2LPENR raw " : "   APB2ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_APB2LPENR raw " : "C1_APB2ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_APB2LPENR raw " : "C2_APB2ENR raw ", c2 );  

  DBG_dump_when_on ("HRTIM Clk",    d1, c1, c2, RCC_APB2ENR_HRTIMEN);  
  DBG_dump_when_on ("DFSDM1 Clk",   d1, c1, c2, RCC_APB2ENR_DFSDM1EN);  
  DBG_dump_when_on ("SAI3 Clk",     d1, c1, c2, RCC_APB2ENR_SAI3EN);  
  DBG_dump_when_on ("SAI2 Clk",     d1, c1, c2, RCC_APB2ENR_SAI2EN);  
  DBG_dump_when_on ("SAI1 Clk",     d1, c1, c2, RCC_APB2ENR_SAI1EN);  
  DBG_dump_when_on ("SPI5 Clk",     d1, c1, c2, RCC_APB2ENR_SPI5EN);  
  DBG_dump_when_on ("TIM17 Clk",    d1, c1, c2, RCC_APB2ENR_TIM17EN);  
  DBG_dump_when_on ("TIM16 Clk",    d1, c1, c2, RCC_APB2ENR_TIM16EN);  
  DBG_dump_when_on ("TIM15 Clk",    d1, c1, c2, RCC_APB2ENR_TIM15EN);  
  DBG_dump_when_on ("SPI4 Clk",     d1, c1, c2, RCC_APB2ENR_SPI4EN);  
  DBG_dump_when_on ("SPI1 Clk",     d1, c1, c2, RCC_APB2ENR_SPI1EN);  
  DBG_dump_when_on ("USART6 Clk",   d1, c1, c2, RCC_APB2ENR_USART6EN);  
  DBG_dump_when_on ("USART1 Clk",   d1, c1, c2, RCC_APB2ENR_USART1EN);  
  DBG_dump_when_on ("TIM8 Clk",     d1, c1, c2, RCC_APB2ENR_TIM8EN);  
  DBG_dump_when_on ("TIM1 Clk",     d1, c1, c2, RCC_APB2ENR_TIM1EN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_apb3enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   APB3LPENR raw " : "   APB3ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_APB3LPENR raw " : "C1_APB3ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_APB3LPENR raw " : "C2_APB3ENR raw ", c2 );  

  DBG_dump_when_on ("WWDG1 Clk", d1, c1, c2, RCC_APB3ENR_WWDG1EN);  
  #if defined(DSI)
    DBG_dump_when_on ("DSI Clk",   d1, c1, c2, RCC_APB3ENR_DSIEN);
  #endif
  DBG_dump_when_on ("LTDC Clk",  d1, c1, c2, RCC_APB3ENR_LTDCEN);  
  DEBUG_PRINTF("\n");
}

void DBG_dump_rcc_apb4enr(uint32_t d1, uint32_t c1, uint32_t c2, uint32_t bSleepRegisters )
{
  DBG_setPadLen(16);
  DBG_dump_uint32_hex(bSleepRegisters ? "   APB4LPENR raw " : "   APB4ENR raw ", d1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C1_APB4LPENR raw " : "C1_APB4ENR raw ", c1 );  
  DBG_dump_uint32_hex(bSleepRegisters ? "C2_APB4LPENR raw " : "C2_APB4ENR raw ", c2 );  

  DBG_dump_when_on ("SAI4 Clk",   d1, c1, c2, RCC_APB4ENR_SAI4EN);  
  DBG_dump_when_on ("RTCAPB Clk", d1, c1, c2, RCC_APB4ENR_RTCAPBEN);  
  DBG_dump_when_on ("VREF Clk",   d1, c1, c2, RCC_APB4ENR_VREFEN);  
  DBG_dump_when_on ("COMP12 Clk", d1, c1, c2, RCC_APB4ENR_COMP12EN);  
  DBG_dump_when_on ("LPTIM5 Clk", d1, c1, c2, RCC_APB4ENR_LPTIM5EN);  
  DBG_dump_when_on ("LPTIM4 Clk", d1, c1, c2, RCC_APB4ENR_LPTIM4EN);  
  DBG_dump_when_on ("LPTIM3 Clk", d1, c1, c2, RCC_APB4ENR_LPTIM3EN);  
  DBG_dump_when_on ("LPTIM2 Clk", d1, c1, c2, RCC_APB4ENR_LPTIM2EN);  
  DBG_dump_when_on ("I2C4 Clk",   d1, c1, c2, RCC_APB4ENR_I2C4EN);  
  DBG_dump_when_on ("SPI6 Clk",   d1, c1, c2, RCC_APB4ENR_SPI6EN);  
  DBG_dump_when_on ("LPUART1 Clk",d1, c1, c2, RCC_APB4ENR_LPUART1EN);  
  DBG_dump_when_on ("SYSCFG Clk", d1, c1, c2, RCC_APB4ENR_SYSCFGEN);  
  DEBUG_PRINTF("\n");
}
#endif /* #if DEBUG_DUMP_PERCLK > 0 */



#define MK_3ARGS(a)     RCC_C1->a | RCC_C2->a, RCC_C1->a,  RCC_C2->a

void DBG_dump_peripheralclocksetting(bool bDumpAll)
{

  bDumpAlways = bDumpAll;
    
  DEBUG_PUTS("Peripheral Clock Settings -----------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  #if DEBUG_DUMP_PERCLK > 0
      /********  AHB peripheral clock enable register *****************/
      DBG_printf_indent("AHB devices state\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_ahb1enr(MK_3ARGS(AHB1ENR), 0);
      DBG_dump_rcc_ahb2enr(MK_3ARGS(AHB2ENR), 0);
      DBG_dump_rcc_ahb3enr(MK_3ARGS(AHB3ENR), 0);
      DBG_dump_rcc_ahb4enr(MK_3ARGS(AHB4ENR), 0);
      DBG_setIndentRel(-2);

      /********  APB peripheral clock enable register *****************/
      DBG_printf_indent("APB devices state\n" );
      DBG_setIndentRel(+2);
      DBG_dump_rcc_apb1henr(MK_3ARGS(APB1HENR), 0);
      DBG_dump_rcc_apb1lenr(MK_3ARGS(APB1LENR), 0);
      DBG_dump_rcc_apb2enr (MK_3ARGS(APB2ENR) , 0);
      DBG_dump_rcc_apb3enr (MK_3ARGS(APB3ENR) , 0);
      DBG_dump_rcc_apb4enr (MK_3ARGS(APB4ENR) , 0);
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

void DBG_dump_peripheralclocksetting_insleepmode(bool bDumpAll)
{
  bDumpAlways = bDumpAll;

  DEBUG_PUTS("Peripheral Clock Settings ** IN SLEEP MODE ** ---" );
  int oldIndent = DBG_setIndentRel(+2);

  #if DEBUG_DUMP_PERCLK > 0 && 0
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

const char * const perclk_txt[]={"HSI Clk", "CSI clk", "HSE Clk", "Resvd/Disabled" };
static const char* DBG_get_rcc_perclk_txt(uint32_t sel)
{
  /* 
   * Clock Source 01 on LPTIM1 is deviating from normal bitcode schema
   * so handle specially
   */   
  if ( sel < sizeof(perclk_txt)/sizeof(char *)) 
    return perclk_txt[sel];
  else
    return "Illegal";
}

const char * const hppp_txt[]={"AHB clk", "PLL1Q", "PLL2R", "PER clk" };
static const char* DBG_get_hclk_pll1q_pll2r_perclk_txt(uint32_t sel)
{
  /* 
   * Clock Source 01 on LPTIM1 is deviating from normal bitcode schema
   * so handle specially
   */   
  if ( sel < sizeof(perclk_txt)/sizeof(char *)) 
    return perclk_txt[sel];
  else
    return "Illegal";
}


#define OUTOF3(sel, a,b,c)          (sel==0?a:(sel==1?b:(sel==2?c:"resvd/disabled")))
#define OUTOF4(sel, a,b,c,d)        (sel==3?d:OUTOF3(sel,a,b,c))
#define OUTOF5(sel, a,b,c,d,e)      (sel==4?e:OUTOF4(sel,a,b,c,d))
#define OUTOF6(sel, a,b,c,d,e,f)    (sel==5?f:OUTOF5(sel,a,b,c,d,e))

void DBG_dump_peripheralclockconfig(void)
{
  DEBUG_PUTS("Peripheral clock configuration ------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  DBG_setPadLen(26);

  #if DEBUG_DUMP_PERCLK > 0
      DBG_dump_textvalue("Peripheral Clk Source", DBG_get_rcc_perclk_txt((RCC->D1CCIPR & RCC_D1CCIPR_CKPERSEL_Msk) >> RCC_D1CCIPR_CKPERSEL_Pos) );    
      if ( __HAL_RCC_SDMMC1_IS_CLK_ENABLED() || __HAL_RCC_SDMMC2_IS_CLK_ENABLED() ) 
          DBG_dump_textvalue("SDMMC Clk Source", READ_BIT(RCC->D1CCIPR ,RCC_D1CCIPR_SDMMCSEL) ? "PLL2R" : "PLL1Q" );    

      /* D1 Peripherals */
      #if defined(DSI)
      if ( __HAL_RCC_SDMMC1_IS_CLK_ENABLED() )
          DBG_printf_indent("*** Not implemented ***");
      #endif
      if ( __HAL_RCC_QSPI_IS_CLK_ENABLED() )
          DBG_dump_textvalue("QSPI Clk Source", DBG_get_hclk_pll1q_pll2r_perclk_txt((RCC->D1CCIPR & RCC_D1CCIPR_QSPISEL_Msk) >> RCC_D1CCIPR_QSPISEL_Pos) );    
      if ( __HAL_RCC_FMC_IS_CLK_ENABLED() )
          DBG_dump_textvalue("FMC Clk Source", DBG_get_hclk_pll1q_pll2r_perclk_txt((RCC->D1CCIPR & RCC_D1CCIPR_FMCSEL_Msk) >> RCC_D1CCIPR_FMCSEL_Pos) );    


      /* D2 Peripherals */
      if ( __HAL_RCC_SWPMI1_IS_CLK_ENABLED() )
          DBG_dump_textvalue("SWPMI Clk Source",  READ_BIT(RCC->D2CCIP1R ,RCC_D2CCIP1R_SWPSEL) ? "HSI" : "PCLK1" ); 
      if ( __HAL_RCC_FDCAN_IS_CLK_ENABLED() )
          DBG_dump_textvalue("FDCAN Clk Source", OUTOF4((RCC->D2CCIP1R & RCC_D2CCIP1R_FDCANSEL_Msk ) >> RCC_D2CCIP1R_FDCANSEL_Pos,"HSE","PLL1Q","PLL2R","resvd/disable" ));    
      if ( __HAL_RCC_DFSDM1_IS_CLK_ENABLED() )
          DBG_dump_textvalue("DFSDM1 Clk Source",  READ_BIT(RCC->D2CCIP1R ,RCC_D2CCIP1R_SWPSEL) ? "SYSCLK" : "PCLK2" ); 
      if ( __HAL_RCC_SPDIFRX_IS_CLK_ENABLED() )
          DBG_dump_textvalue("SPDIF Clk Source", OUTOF4((RCC->D2CCIP1R & RCC_D2CCIP1R_SPDIFSEL_Msk ) >> RCC_D2CCIP1R_SPDIFSEL_Pos,"PLL1Q","PLL2R","PLL3R","HSI" ));    
      if ( __HAL_RCC_SPI4_IS_CLK_ENABLED() || __HAL_RCC_SPI5_IS_CLK_ENABLED())
          DBG_dump_textvalue("SPI4,5 Clk Source", OUTOF6((RCC->D2CCIP1R & RCC_D2CCIP1R_SPI45SEL_Msk ) >> RCC_D2CCIP1R_SPI45SEL_Pos,"PCLK", "PLL2Q","PLL3Q","HSI","CSI","HSE" ));    
      if ( __HAL_RCC_SPI1_IS_CLK_ENABLED() || __HAL_RCC_SPI2_IS_CLK_ENABLED() || __HAL_RCC_SPI3_IS_CLK_ENABLED() )
          DBG_dump_textvalue("SPI1,2,3 Clk Source", OUTOF6((RCC->D2CCIP1R & RCC_D2CCIP1R_SPI123SEL_Msk ) >> RCC_D2CCIP1R_SPI123SEL_Pos,"PLL1Q", "PLL2P","PLL3P","I2S_IN","CSI","PER" ));    

      if ( __HAL_RCC_LPTIM1_IS_CLK_ENABLED() )
          DBG_dump_textvalue("LPTIM1 Clk Source", OUTOF6((RCC->D2CCIP2R & RCC_D2CCIP2R_LPTIM1SEL_Msk ) >> RCC_D2CCIP2R_LPTIM1SEL_Pos,"PCLK1", "PLL2P","PLL3R","LSE","LSI","PER" ));        
      if ( __HAL_RCC_CEC_IS_CLK_ENABLED() )
          DBG_dump_textvalue("CEC Clk Source", OUTOF3((RCC->D2CCIP2R & RCC_D2CCIP2R_CECSEL_Msk ) >> RCC_D2CCIP2R_CECSEL_Pos,"LSE", "LSI", "CSI"));        
      if ( __HAL_RCC_USB1_OTG_HS_IS_CLK_ENABLED() || __HAL_RCC_USB2_OTG_FS_IS_CLK_ENABLED() )
          DBG_dump_textvalue("USB Clk Source", OUTOF4((RCC->D2CCIP2R & RCC_D2CCIP2R_USBSEL_Msk ) >> RCC_D2CCIP2R_USBSEL_Pos,"Disabled", "PLL1Q", "PLL3Q","HSI48"));        
      if ( __HAL_RCC_I2C1_IS_CLK_ENABLED() || __HAL_RCC_I2C2_IS_CLK_ENABLED() || __HAL_RCC_I2C3_IS_CLK_ENABLED() )
          DBG_dump_textvalue("I2C1,2,3 Clk Source", OUTOF4((RCC->D2CCIP2R & RCC_D2CCIP2R_I2C123SEL_Msk ) >> RCC_D2CCIP2R_I2C123SEL_Pos,"PCLK1", "PLL3R", "HSI","CSI"));        
      if ( __HAL_RCC_RNG_IS_CLK_ENABLED() )
          DBG_dump_textvalue("RNG Clk Source", OUTOF4((RCC->D2CCIP2R & RCC_D2CCIP2R_RNGSEL_Msk ) >> RCC_D2CCIP2R_RNGSEL_Pos,"HSI48", "PLL1Q", "LSE","LSI"));        
      if ( __HAL_RCC_USART1_IS_CLK_ENABLED() || __HAL_RCC_USART6_IS_CLK_ENABLED() )
          DBG_dump_textvalue("USART1,6 Clk Source", OUTOF6((RCC->D2CCIP2R & RCC_D2CCIP2R_USART16SEL_Msk ) >> RCC_D2CCIP2R_USART16SEL_Pos,"PCLK2", "PLL2Q","PLL3Q","HSI","CSI","LSE" ));    
      if (  __HAL_RCC_USART2_IS_CLK_ENABLED() || __HAL_RCC_USART3_IS_CLK_ENABLED() 
          || __HAL_RCC_UART4_IS_CLK_ENABLED() || __HAL_RCC_UART5_IS_CLK_ENABLED() || __HAL_RCC_UART7_IS_CLK_ENABLED() || __HAL_RCC_UART8_IS_CLK_ENABLED() )
          DBG_dump_textvalue("U(S)ART2-5,7,8 Clk Source", OUTOF6((RCC->D2CCIP2R & RCC_D2CCIP2R_USART16SEL_Msk ) >> RCC_D2CCIP2R_USART16SEL_Pos,"PCLK1", "PLL2Q","PLL3Q","HSI","CSI","LSE" ));    

      /* D3 Peripherals */
      if ( __HAL_RCC_SPI6_IS_CLK_ENABLED() )
          DBG_dump_textvalue("SPI6 Clk Source", OUTOF6((RCC->D3CCIPR & RCC_D3CCIPR_SPI6SEL_Msk ) >> RCC_D3CCIPR_SPI6SEL_Pos, "PCLK4", "PLL2Q","PLL3Q","HSI","CSI","HSE" ));    
      if ( __HAL_RCC_SAI4_IS_CLK_ENABLED() ) {
          DBG_dump_textvalue("SAI4B Clk Source", OUTOF5((RCC->D3CCIPR & RCC_D3CCIPR_SAI4BSEL_Msk ) >> RCC_D3CCIPR_SAI4BSEL_Pos, "PLL1Q", "PLL2P","PLL3P","I2S_IN","PER" ));    
          DBG_dump_textvalue("SAI4A Clk Source", OUTOF5((RCC->D3CCIPR & RCC_D3CCIPR_SAI4ASEL_Msk ) >> RCC_D3CCIPR_SAI4ASEL_Pos, "PLL1Q", "PLL2P","PLL3P","I2S_IN","PER" ));    
      }
      if ( __HAL_RCC_ADC12_IS_CLK_ENABLED() || __HAL_RCC_ADC3_IS_CLK_ENABLED() )
          DBG_dump_textvalue("ADC1,2,3 Clk Source", OUTOF3((RCC->D3CCIPR & RCC_D3CCIPR_ADCSEL_Msk ) >> RCC_D3CCIPR_ADCSEL_Pos, "PLL2P", "PLL3R","PER" ));    
      if ( __HAL_RCC_LPTIM3_IS_CLK_ENABLED() || __HAL_RCC_LPTIM4_IS_CLK_ENABLED() || __HAL_RCC_LPTIM5_IS_CLK_ENABLED() )
          DBG_dump_textvalue("LPTIM3,4,5 Clk Source", OUTOF6((RCC->D3CCIPR & RCC_D3CCIPR_LPTIM345SEL_Msk ) >> RCC_D3CCIPR_LPTIM345SEL_Pos, "PCLK4", "PLL2P","PLL3R","LSE","LSI","PER" ));     
      if ( __HAL_RCC_LPTIM2_IS_CLK_ENABLED() )
          DBG_dump_textvalue("LPTIM2 Clk Source", OUTOF6((RCC->D3CCIPR & RCC_D3CCIPR_LPTIM2SEL_Msk ) >> RCC_D3CCIPR_LPTIM2SEL_Pos, "PCLK4", "PLL2P","PLL3R","LSE","LSI","PER" ));     
  #else
      DBG_printf_indent("configured out\n");
  #endif

  DBG_setIndentAbs(oldIndent);
}

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} xDMA_Stream_TypeDef;

#define GET_BIT(reg, mask )        ( reg & mask ? 1 : 0)
#define GET_2BIT(reg, mask, pos)   ( ( reg & mask ) >> pos )
const uint32_t burst_txt[] = { 1, 4, 8, 16 };
const char *dir_txt[] = {"Perip->Mem", "Mem->Perip", "Mem->Mem  ", "reserved  "};
const char *size_txt[] = {"8Bit ", "16Bit", "32Bit", "resvd"};

void DBG_dump_one_DMAStream (uint32_t idx, bool bOnlyActive, DMA_Stream_TypeDef *stream)
{
    uint32_t cr = stream->CR;
    if ( bOnlyActive && GET_BIT(cr, DMA_SxCR_EN) == 0 ) return;
    DBG_printf_indent("Stream# raw: 0x%08x\n", cr); 
    DBG_printf_indent("Stream#%d: En=%d, MBurst=%d, PBurst=%d, TrBuff=%d, DBM=%d, Pri=%d\n", 
         idx, GET_BIT(cr, DMA_SxCR_EN), burst_txt[GET_2BIT(cr, DMA_SxCR_MBURST_Msk, DMA_SxCR_MBURST_Pos)],
          burst_txt[GET_2BIT(cr, DMA_SxCR_PBURST_Msk, DMA_SxCR_PBURST_Pos)], GET_BIT(cr, (1 <<20 )), GET_BIT(cr, DMA_SxCR_DBM), GET_2BIT(cr, DMA_SxCR_PL_Msk, DMA_SxCR_PL_Pos) );
    DBG_printf_indent("   Circ=%d Dir=%s, PfCtrl=%d, TCIE=%d, HTIE=%d\n",
          GET_BIT(cr, DMA_SxCR_CIRC), dir_txt[GET_2BIT(cr, DMA_SxCR_DIR_Msk, DMA_SxCR_DIR_Pos)], GET_BIT(cr, DMA_SxCR_PFCTRL), GET_BIT(cr, DMA_SxCR_TCIE), GET_BIT(cr, DMA_SxCR_HTIE) );  
    DBG_printf_indent("   MemAddr0=0x%08x %s, MemAddr1=0x%08x %s, Size=%s, Inc=%d\n", 
          stream->M0AR, GET_BIT(cr, DMA_SxCR_CT) == 0 ? "(act)":"", stream->M1AR, GET_BIT(cr, DMA_SxCR_CT) == 1 ? "(act)":"",
          size_txt[GET_2BIT(cr, DMA_SxCR_MSIZE_Msk, DMA_SxCR_MSIZE_Pos)], GET_BIT(cr, DMA_SxCR_MINC) );
    DBG_printf_indent("   PeripAddr=0x%08x Size=%s, Inc=%d %s\n", 
          stream->PAR, size_txt[GET_2BIT(cr, DMA_SxCR_PSIZE_Msk, DMA_SxCR_PSIZE_Pos)], GET_BIT(cr, DMA_SxCR_PINC),
          GET_BIT(cr, DMA_SxCR_PINC) && GET_BIT(cr, DMA_SxCR_PINCOS) ? "Ofs32" : ""  );
           
    DBG_printf_indent("   Count=0x%08x FifoTH=%d,\n", 
          stream->NDTR, GET_2BIT(stream->FCR, DMA_SxFCR_FTH_Msk, DMA_SxFCR_FTH_Pos));
}

void DBG_dump_DMA ( uint32_t dmanum )
{
    uint32_t DMA_Base = ( dmanum == 1 ? DMA1_BASE : DMA2_BASE );
    DMA_Stream_TypeDef *stream;
    DBG_printf_indent("DMA%d Controller------------------\n",dmanum);
    int oldIndent = DBG_setIndentRel(+2);
    for ( uint32_t i = 0; i < 8; i++ ) {
        stream = ( DMA_Stream_TypeDef *)(DMA_Base + 0x10 + i *0x18);
        DBG_dump_one_DMAStream (i, true, stream);
    }
    DBG_setIndentAbs(oldIndent);
}

void DBG_dump_all_DMA (void )
{
  DEBUG_PUTS("DMA configuration ------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  DBG_setPadLen(18);

  DBG_dump_DMA(1);

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
