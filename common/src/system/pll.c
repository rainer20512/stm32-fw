/*
 ******************************************************************************
 * @file    pllconfig.c 
 * @author  Rainer
 * @brief   Routines for PLL configuration and housekeeping
 *
 *****************************************************************************/

/** @addtogroup PLL device
  * @{
  */

#include "config/config.h"
#include "error.h"
#include "hardware.h"

#include <string.h>          /* memset */
/* get the return codes */
#include "system/pll.h"

/* get the family specific PLL definitions */
#include "config/pll_config.h"

/*****************************************************************************
 * One Line of a PLL ( ie P,Q or R )
 ****************************************************************************/
typedef struct PllLineType {
    uint32_t x_min, x_max;          /* Min and max values for x-Stage output, 0=no restriction, x=P, Q or R */
    uint8_t  bIsImplemented;        /* != 0, if this line is implemented */
} PllLineT;

typedef struct PllType {
    char pllname[PLL_NAMELEN];
    uint32_t m_min, m_max;          /* Min and max values for M-Stage output, 0=no restriction, all values in kHz */
    uint32_t n_min, n_max;          /* Min and max values for internal N-Stage output, 0=no restriction, all values in kHz */
    uint16_t pqrdiv_max;            /* Maximum divisor value of fonal pll stage */
    PllLineT pllLine[MAX_LINENUM];  /* Limits and "implemented flag for the P,Q and R lines */
} PllT;

static const PllT pll_restriction[MAX_PLLNUM] = {
    {
    /* PLL1 */
    #if HAS_PLL1 > 0 
        .pllname = PLL1NAME,
        #if PLLM_MIN > 0
            .m_min = PLLM_MIN,
        #else
            .m_min = 0,
        #endif
        #if PLLM_MAX > 0
            .m_max = PLLM_MAX,
        #else
            .m_max = 0,
        #endif
        #if PLLN1_MIN > 0
            .n_min = PLLN1_MIN,
        #else
            .n_min = 0,
        #endif
        #if PLLN1_MAX > 0
            .n_max = PLLN1_MAX,
        #else
            .n_max = 0,
        #endif
        #if PLL_PQR_DIVMAX > 0
            .pqrdiv_max = PLL_PQR_DIVMAX,
        #else
            .pqrdiv_max =8,
        #endif
        .pllLine = {
            {
            #if PLL1P > 0
                #if PLLP1_MIN > 0
                    .x_min = PLLP1_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLP1_MAX > 0
                    .x_max = PLLP1_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL1Q > 0
                #if PLLQ1_MIN > 0
                    .x_min = PLLQ1_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLQ1_MAX > 0
                    .x_max = PLLQ1_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL1R > 0
                #if PLLR1_MIN > 0
                    .x_min = PLLR1_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLR1_MAX > 0
                    .x_max = PLLR1_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },

        },
    #else 
        /* no PLL1 */
        0,
    #endif   
    },
    {
    /* PLL2 */
    #if HAS_PLL2 > 0 
        .pllname = PLL2NAME,
        #if PLLM_MIN > 0
            .m_min = PLLM_MIN,
        #else
            .m_min = 0,
        #endif
        #if PLLM_MAX > 0
            .m_max = PLLM_MAX,
        #else
            .m_max = 0,
        #endif
        #if PLLN2_MIN > 0
            .n_min = PLLN2_MIN,
        #else
            .n_min = 0,
        #endif
        #if PLLN2_MAX > 0
            .n_max = PLLN2_MAX,
        #else
            .n_max = 0,
        #endif
        #if PLL_PQR_DIVMAX > 0
            .pqrdiv_max = PLL_PQR_DIVMAX,
        #else
            .pqrdiv_max =8,
        #endif
        .pllLine = {
            {
            #if PLL2P > 0
                #if PLLP2_MIN > 0
                    .x_min = PLLP2_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLP2_MAX > 0
                    .x_max = PLLP2_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL2Q > 0
                #if PLLQ2_MIN > 0
                    .x_min = PLLQ2_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLQ2_MAX > 0
                    .x_max = PLLQ2_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL2R > 0
                #if PLLR2_MIN > 0
                    .x_min = PLLR2_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLR2_MAX > 0
                    .x_max = PLLR2_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },

        },
    #else 
        /* no PLL2 */
        0,
    #endif    
    },
    {
    /* PLL3 */
    #if HAS_PLL3 > 0 
        .pllname = PLL3NAME,
        #if PLLM_MIN > 0
            .m_min = PLLM_MIN,
        #else
            .m_min = 0,
        #endif
        #if PLLM_MAX > 0
            .m_max = PLLM_MAX,
        #else
            .m_max = 0,
        #endif
        #if PLLN3_MIN > 0
            .n_min = PLLN3_MIN,
        #else
            .n_min = 0,
        #endif
        #if PLLN3_MAX > 0
            .n_max = PLLN3_MAX,
        #else
            .n_max = 0,
        #endif
        #if PLL_PQR_DIVMAX > 0
            .pqrdiv_max = PLL_PQR_DIVMAX,
        #else
            .pqrdiv_max =8,
        #endif
        .pllLine = {
            {
            #if PLL3P > 0
                #if PLLP3_MIN > 0
                    .x_min = PLLP3_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLP3_MAX > 0
                    .x_max = PLLP3_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL3Q > 0
                #if PLLQ3_MIN > 0
                    .x_min = PLLQ3_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLQ3_MAX > 0
                    .x_max = PLLQ3_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },
            {
            #if PLL3R > 0
                #if PLLR3_MIN > 0
                    .x_min = PLLR3_MIN,
                #else
                    .x_min = 0,
                #endif
                #if PLLR3_MAX > 0
                    .x_max = PLLR3_MAX,
                #else
                    .x_max = 0,
                #endif
                .bIsImplemented=1,
            #else
                0,
            #endif
            },

        },
    #else 
        /* no PLL3 */
        0,
    #endif    
    },
};

/******************************************************************************
 * check, whether PLL is in use
 * @param pllnum - index of PLL to check
 * @returns 1, if in use, 0 if switched off
 *****************************************************************************/
static const uint32_t pllOnFlags[MAX_PLLNUM] = PLLON_FLAGS;
uint32_t Pll_InUse( uint32_t pllnum )
{
    return ( RCC->CR & pllOnFlags[pllnum] ) != 0 ? 1 : 0; 
}


/******************************************************************************
 * helper function for Pll_GetActiveLines()
 *****************************************************************************/
static uint32_t getactive(uint32_t reg, uint32_t enP, uint32_t enQ, uint32_t enR )
{
    uint32_t ret = 0;
    if ( reg & enP ) ret |= PLL_LINE_P;
    if ( reg & enQ ) ret |= PLL_LINE_Q;
    if ( reg & enR ) ret |= PLL_LINE_R;

    return ret;
}

/******************************************************************************
 * Get the active lines for a PLL
 * @param pllnum - index of PLL to check
 * @returns a subset of { PLL1, PLL2, PLL3 } or 0, if no line is in use
 *****************************************************************************/
static uint32_t Pll_GetActiveLines( uint32_t pllnum )
{
    switch(pllnum) {
        case SYS_PLL1:
            return getactive(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN, RCC_PLLCFGR_DIVQ1EN, RCC_PLLCFGR_DIVR1EN );
        case SYS_PLL2:
            return getactive(RCC->PLLCFGR, RCC_PLLCFGR_DIVP2EN, RCC_PLLCFGR_DIVQ2EN, RCC_PLLCFGR_DIVR2EN );
        case SYS_PLL3:
            return getactive(RCC->PLLCFGR, RCC_PLLCFGR_DIVP3EN, RCC_PLLCFGR_DIVQ3EN, RCC_PLLCFGR_DIVR3EN );
        default:
            return 0xFFFFFFFF;
    }
}

/******************************************************************************
 * Mask out specific bits and right shift the resulting value
 * To use with the HAL constants XXXX_Msk and XXXX_Pos
 * @param value  - original value
 * @param mask   - Bitmask of bits to keep ( mask and value will be AND'ed )
 * @param rshift - number of bits the AND'ed result will be right shifted
 * @returns ( value & mask ) >> rshift
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
static uint32_t maskout(uint32_t value, uint32_t mask, uint32_t rshift )
{
    return ( value & mask ) >> rshift;
}

/******************************************************************************
 * return the M-Divisor value for the selected PLL
 * @param pllnum - index of PLL to check
 * @returns M-Divisor value if in use, 0 if PLL is unused
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
int32_t Pll_GetM(uint32_t pllnum )
{
    switch ( pllnum ) {
        case SYS_PLL1:
            return maskout(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1_Msk, RCC_PLLCKSELR_DIVM1_Pos);
        case SYS_PLL2:
            return maskout(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM2_Msk, RCC_PLLCKSELR_DIVM2_Pos);
        case SYS_PLL3:
            return maskout(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM3_Msk, RCC_PLLCKSELR_DIVM3_Pos);
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
}

/******************************************************************************
 * return the M-Divisor value for the selected PLL
 * @param pllnum - index of PLL to check
 * @returns M-Divisor value if in use, 0 if PLL is unused
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
int32_t Pll_GetN(uint32_t pllnum )
{
    switch ( pllnum ) {
        case SYS_PLL1:
            return maskout(RCC->PLL1DIVR, RCC_PLL1DIVR_N1_Msk, RCC_PLL1DIVR_N1_Pos);
        case SYS_PLL2:
            return maskout(RCC->PLL2DIVR, RCC_PLL2DIVR_N2_Msk, RCC_PLL2DIVR_N2_Pos);
        case SYS_PLL3:
            return maskout(RCC->PLL3DIVR, RCC_PLL3DIVR_N3_Msk, RCC_PLL3DIVR_N3_Pos);
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
}


/******************************************************************************
 * Helper function for Pll_CheckM, _CheckN, _CheckPQR
 * @param val     - value to check
 * @param lobound - low bound or 0, if no low bound is set
 * @param hibound - high bound or 0, if no hight bound is set
 * @returns 1 if value lies within bounds
 * @returns 0 if value lies out of bounds
 * @Note lobound and hibound are both included in range
 *****************************************************************************/
static uint32_t pll_checkbounds ( uint32_t val, uint32_t lobound, uint32_t hibound )
{
    /* Lower limit */
    if ( lobound > 0 && val < lobound ) return 0;
    
    /* upper limit */
    if ( hibound > 0 && val > hibound ) return 0;

    return 1;
}

/******************************************************************************
 * check, whether the passed PLL M value violates any restrictions for M
 * @param pllnum - index of PLL to check
 * @param m_val  - desired M output frequency in kHz
 * @returns 1, if ok, 0 if M restictions is violated
 *****************************************************************************/
static uint32_t Pll_CheckM ( uint32_t pllnum, uint32_t m_val )
{
    return pll_checkbounds( m_val, pll_restriction[pllnum].m_min, pll_restriction[pllnum].m_max);
}


/******************************************************************************
 * check, whether the passed PLL N value violates any restrictions for N
 * @param pllnum - index of PLL to check
 * @param n_val  - desired N output frequency in kHz
 * @returns 1, if ok, 0 if N restictions is violated
 *****************************************************************************/
static uint32_t Pll_CheckN ( uint32_t pllnum, uint32_t n_val )
{
    return pll_checkbounds( n_val, pll_restriction[pllnum].n_min, pll_restriction[pllnum].n_max);
}


/******************************************************************************
 * Setup M divider and N multiplier of a PLL with the following parameters
 * - the M stage output will always be PLLM_EVEN for even input frequencies
 *   or PLLM_ODD for input frequencies that are multiples of 5000 MHz
 * - all restrictions regarding M and N output frequencies are checked
 *   if any of these restrictions is violated, the function will return with 
 *   an error value
 * @param PLL         - structure to be filled with PLL parameters
 * @param pllnum      - desired PLL ( 0, 1, or 2, i.e. PLL1, PLL2, or PLL3 )
 * @param pll_out_khz - desired output frequency in kHz
 * @param pll_inp_khz - actual PLL input frequency in kHz
 * 
 *****************************************************************************/
static int32_t Pll_SetupMN ( RCC_PLLInitTypeDef *PLL, uint32_t pllnum, uint32_t pll_out_khz, uint32_t pll_inp_khz )
{
  uint32_t pll_khz_behind_m;    /* PLL frq behind the M stage */

  /* calculate M so that the frq after M is always 2MHz, or - in case of HSE not a multiple of 2 - 5Mhz */
  if ( pll_inp_khz / PLLM_ODD * PLLM_ODD == pll_inp_khz ) {
    pll_khz_behind_m = PLLM_ODD;
  } else {
    pll_khz_behind_m = PLLM_EVEN;
  }

  /* Check constraints for M output frequency */
  if ( !Pll_CheckM(pllnum, pll_khz_behind_m ) ) return PLL_CONFIG_VIOLATION_M;

  PLL->PLLM = pll_inp_khz / pll_khz_behind_m;

  /*
   * Check for minimum N stage restriction. If there is one, set the output divisor
   * so, that N min will not go below that minimum. 
   * If there is no limit, we will use a divisor of 2 ( which is the smallest possible )
   */
  uint32_t pqr_div;
  if ( pll_restriction[pllnum].n_min > 0 ) {
    uint32_t n_min = pll_restriction[pllnum].n_min;
    /* To be compatible with L4 family, final divisor must be one of 2,4,6,8 
     * compute the minimum possible of these values.
     */
     pqr_div = n_min / pll_out_khz;
     if ( pqr_div < 2 ) pqr_div = 2;
     /* find next higher multiple of 2 */
     pqr_div = (pqr_div+1)/2;
     pqr_div *= 2;
  } else {
    pqr_div = 2;
  }
  /* Check constraints for N output frequency */
  if ( !Pll_CheckN(pllnum, pll_out_khz * pqr_div ) ) return PLL_CONFIG_VIOLATION_N;

  PLL->PLLN = pll_out_khz / pll_khz_behind_m * pqr_div;
  return PLL_CONFIG_OK;   
}

/******************************************************************************
 * configure a PLL with the following parameters
 * - the M stage output will always be PLLM_EVEN for even input frequencies
 *   or PLLM_ODD for input frequencies that are multiples of 5000 MHz
 * - all restrictions regarding M, N, P, Q or R output frequencies are checked
 *   if any of these restrictions is violated, the function will return with a value
 * @param RCC_OscInitStruct - to be filled with PLL parameters
 * @param pllnum     - desired PLL ( 0, 1, or 2, i.e. PLL1, PLL2, or PLL3 )
 * @param pll_line    - desired line of PLL ( 0,1, or 2 ,i.e. P,Q or R-line )
 * @param pll_out_khz - desired SYSCLK 
 * @param pll_inp_khz - actual PLL input frequency
 * @returns           - any of the PLL_CONFIG_XXXX values defined in pll.h
 *****************************************************************************/
int32_t PLL_Configure (RCC_PLLInitTypeDef *PLL, uint32_t pllnum, uint32_t pll_line, uint32_t pll_out_khz, uint32_t pll_inp_khz )
{
  int32_t work;

  /* Clear PLL data structure */  
  memset(PLL, 0, sizeof(RCC_PLLInitTypeDef) );

  /* only inactive PLLs may be configured */
  if ( Pll_InUse(pllnum) ) return  PLL_CONFIG_INUSE;

  /* 
   * Check, whether PLL or PLL line is in use 
   * This is the case, if M is already set AND the currently configured line is not the only line used
   */

  /* Get M and N value and check for error */
  work = Pll_GetM(pllnum);
  if ( work < 0 ) return work;
  if ( work > 0  && Pll_GetActiveLines(pllnum) != pll_line ) {
    /* If PLL is in use, set M and N to the values actually configured in PLL without any checks */
    PLL->PLLM = work;
    work = Pll_GetN(pllnum);
    if ( work < 0 ) return work;
    PLL->PLLN =work;
  } else {
    /* otherwise compute M and N and check for errors */
    int32_t work = Pll_SetupMN( PLL, pllnum, pll_out_khz, pll_inp_khz);
    if ( work < 0 ) return work;
  }

  /* calculate divisor of final stage and check restrictions for max. Divisor of final stage */
  uint32_t pll_behind_n = pll_inp_khz / PLL->PLLM * PLL->PLLN;
  uint32_t pqr_div = pll_behind_n / pll_out_khz;
  if ( pqr_div > pll_restriction[pllnum].pqrdiv_max ) return  PLL_CONFIG_VIOLATION_PQR;


  PLL->PLLFRACN = 0;
  switch(pll_line) {
    case PLL_LINE_P:
      PLL->PLLP = pqr_div;
      break;
    case PLL_LINE_Q:
      PLL->PLLQ = pqr_div;
      break;
    case PLL_LINE_R:
      PLL->PLLR = pqr_div;
      break;
  }

  /* If SYSCLK <= 200 MHZ, select VCO medium */
  PLL->PLLVCOSEL = ( pll_behind_n <= 200000 ? RCC_PLL1VCOMEDIUM : RCC_PLL1VCOWIDE);
  /* 2 MHz Input is either Range 0 or Range 1 */
  if ( pll_inp_khz / PLL->PLLM  <= 4000 ) 
    PLL->PLLRGE = RCC_PLL1VCIRANGE_1;
  else if ( pll_inp_khz / PLL->PLLM  <= 8000 ) 
    PLL->PLLRGE = RCC_PLL1VCIRANGE_2;
  else
    PLL->PLLRGE = RCC_PLL1VCIRANGE_2;
  return PLL_CONFIG_OK;
}

/******************************************************************************
 * configure the PLL and PLL line that will feed the SYSCLK Multiplexer
 * @param RCC_OscInitStruct - to be filled with PLL parameters
 * @param pll_out_khz - desired SYSCLK 
 * @param pll_inp_khz - actual PLL input frequency
 * @returns           - any of the PLL_CONFIG_XXXX values defined in pll.h
 *****************************************************************************/
int32_t PLL_Configure_SYSCLK (RCC_OscInitTypeDef *RCC_OscInitStruct, uint32_t pll_out_khz, uint32_t pll_inp_khz )
{
    int32_t ret =  PLL_Configure(&RCC_OscInitStruct->PLL, SYSCLK_PLL, SYSCLK_PLL_LINE, pll_out_khz, pll_inp_khz);
    if ( ret < 0 ) return ret;

    RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;

    return PLL_CONFIG_OK;
}

