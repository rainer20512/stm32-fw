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



static uint32_t pll_input_frq=0;       /* The global PLL input clock frequency in kHz */
static uint32_t pll_use_flags=0;       /* mirror of the PLL-ON-Flags from RCC->CR, will be used to restore on wakeup from stop */
static const uint32_t pllOnFlags[MAX_PLLNUM]  = PLLON_FLAGS;
static const uint32_t pllRdyFlags[MAX_PLLNUM] = PLLRDY_FLAGS;

/******************************************************************************
 * check, whether PLL is in use
 * @param pllnum - index of PLL to check
 * @returns 1, if in use, 0 if switched off
 *****************************************************************************/
uint32_t PLL_InUse( uint32_t pllnum )
{
    return ( RCC->CR & pllOnFlags[pllnum] ) != 0 ? 1 : 0; 
}

/******************************************************************************
 * Start one PLL
 * @param pllnum - index of PLL to start
 * @returns PLL_CONFIG_OK, if ok, returns PLL_CONFIG_TIMEOUT if unsuccessful
 *****************************************************************************/
int32_t PLL_Start(uint32_t pllnum )
{

    if ( pllnum >= MAX_PLLNUM ) return PLL_CONFIG_PARAM_ERROR;

    /* Switch PLL on */
    SET_BIT(RCC->CR, pllOnFlags[pllnum] );

    /* Wait to PLL become ready */
    uint32_t tickstart = HAL_GetTick();
    while(READ_BIT(RCC->CR, pllRdyFlags[pllnum]) == 0U)
        if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE ) return PLL_CONFIG_TIMEOUT;

    /* Store ON-Flag to mirror */
    pll_use_flags |= ( 1 << pllnum );

    return PLL_CONFIG_OK;
}

/******************************************************************************
 * Stop one PLL
 * @param pllnum - index of PLL to start
 * @returns PLL_CONFIG_OK, if ok, PLL_CONFIG_TIMEOUT if unsuccessful
 *          PLL_CONFIG_PARAM_ERROR if pllnum invalid or if trying to stop
 *          a PLL that supplies the system clock
 *****************************************************************************/
int32_t PLL_Stop(uint32_t pllnum )
{

    if ( pllnum >= MAX_PLLNUM ) return PLL_CONFIG_PARAM_ERROR;

    /* Do not stop, if SYSCLK is supplied by PLL that is going to be stopped */
    if ( pllnum == SYSCLK_PLL && __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSPLL_MASK ) return PLL_CONFIG_PARAM_ERROR;

    /* Switch PLL off */
    CLEAR_BIT(RCC->CR, pllOnFlags[pllnum] );

    /* Wait to PLL become inactive */
    uint32_t tickstart = HAL_GetTick();
    while(READ_BIT(RCC->CR, pllRdyFlags[pllnum]) != 0U)
        if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE ) return PLL_CONFIG_TIMEOUT;

    /* Remove ON-Flag from mirror */
    pll_use_flags &= ~(1<<pllnum);

    return PLL_CONFIG_OK;
}

/******************************************************************************
 * Restart all PLLs, that are flagged as "started" in "pll_use_flag"
 * This functions is used to restart all active PLLs after wakeup from STOP
 *****************************************************************************/
void PLL_Restart(void)
{
    for ( uint32_t i = 0;i < MAX_PLLNUM; i++ ) 
        if ( pll_use_flags & ( 1 << i ) ) PLL_Start(i);

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
 * Write specific bits into a register
 * To use with the HAL constants XXXX_Msk and XXXX_Pos
 * @param reg    - register to read/modify/write
 * @param mask   - Bitmask of bits to clear before write
 * @param value  - value to be written
 * @param shift  - number of bits the value will be left shifted
 *****************************************************************************/
#define MASKIN(reg, mask, value, shift) MODIFY_REG( reg, mask, (value) << shift )

/******************************************************************************
 * Get the active lines for a PLL
 * @param pllnum - index of PLL to check
 * @returns a subset of { PLL1, PLL2, PLL3 } or 0, if no line is in use
 *****************************************************************************/
static uint32_t Pll_GetActiveLines( uint32_t pllnum )
{
#if defined(STM32H7_FAMILY)
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
#elif defined(STM32L4_FAMILY) || defined(STM32L4PLUS_FAMILY)
    switch(pllnum) {
        case SYS_PLL1:
            return getactive(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN, RCC_PLLCFGR_PLLQEN, RCC_PLLCFGR_PLLREN );
        case SYS_PLL2:
            return getactive(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN, RCC_PLLSAI1CFGR_PLLSAI1QEN, RCC_PLLSAI1CFGR_PLLSAI1REN );
        #if defined(PLL_HAS_PLL3)
            case SYS_PLL3:
                #if defined(STM32L4PLUS_FAMILY)
                    /* STM32L4 family has no Q line in PLLSAI2 */
                    return getactive(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2PEN, RCC_PLLSAI2CFGR_PLLSAI2QEN, RCC_PLLSAI2CFGR_PLLSAI2REN );
                #else
                    return getactive(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2PEN, 0,                          RCC_PLLSAI2CFGR_PLLSAI2REN );
                #endif
        #endif
        default:
            return 0xFFFFFFFF;
    }
#else
    #error "No Implemetation of ""Pll_GetActiveLines"""
#endif
}

/******************************************************************************
 * return the PLL's name
 *****************************************************************************/
const char * PLL_GetName(uint32_t pllnum )
{
    if ( pllnum >= MAX_PLLNUM )
        return "Illegal PLL idx";
    else    
        return pll_restriction[pllnum].pllname;
}


/******************************************************************************
 * return the M-Divisor value for the selected PLL
 * @param pllnum - index of PLL to check
 * @returns M-Divisor value if in use, 0 if PLL is unused
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
int32_t Pll_GetM(uint32_t pllnum )
{
#if defined(STM32H7_FAMILY)
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
#elif defined(STM32L4PLUS_FAMILY)
    switch ( pllnum ) {
        case SYS_PLL1:
            return maskout(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos) + 1;
        case SYS_PLL2:
            return maskout(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M_Msk, RCC_PLLSAI1CFGR_PLLSAI1M_Pos) + 1;
        case SYS_PLL3:
            return maskout(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2M_Msk, RCC_PLLSAI2CFGR_PLLSAI2M_Pos) + 1;
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
#elif defined(STM32L4_FAMILY)
    /* STM32L4 family has only one M stage for all PLLs */
    UNUSED(pllnum);
    return maskout(RCC->CFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos)+1;
#else
    #error "No Implemetation of ""Pll_GetM"""
#endif
}

/******************************************************************************
 * return the N-Multiplier value for the selected PLL
 * @param pllnum - index of PLL to check
 * @returns N-Multiplier if in use, 0 if PLL is unused
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
int32_t Pll_GetN(uint32_t pllnum )
{
#if defined(STM32H7_FAMILY)
    switch ( pllnum ) {
        case SYS_PLL1:
            return maskout(RCC->PLL1DIVR, RCC_PLL1DIVR_N1_Msk, RCC_PLL1DIVR_N1_Pos) + 1;
        case SYS_PLL2:
            return maskout(RCC->PLL2DIVR, RCC_PLL2DIVR_N2_Msk, RCC_PLL2DIVR_N2_Pos) + 1;
        case SYS_PLL3:
            return maskout(RCC->PLL3DIVR, RCC_PLL3DIVR_N3_Msk, RCC_PLL3DIVR_N3_Pos) + 1;
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
#elif defined(STM32L4_FAMILY) 
    switch ( pllnum ) {
        case SYS_PLL1:
            return maskout(RCC->CFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_Pos);
        case SYS_PLL2:
            return maskout(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N_Msk, RCC_PLLSAI1CFGR_PLLSAI1N_Pos);
        #if defined(PLL_HAS_PLL3)
            case SYS_PLL3:
                return maskout(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N_Msk, RCC_PLLSAI2CFGR_PLLSAI2N_Pos);
        #endif    
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
#else
    #error "No Implemetation of ""Pll_GetN"""
#endif
}

/******************************************************************************
 * return the P,Q or R divisor for the selected PLL
 * @param pllnum   - index of PLL to check
 * @param pll_line - PLL line to check
 * @returns P, Q or R divisor if in use, 0 if PLL is unused
 * @returns Error value ( < 0 ) if pllnum is invalid
 *****************************************************************************/
int32_t Pll_GetPQR(uint32_t pllnum, uint32_t pll_line )
{
    uint32_t cfgreg;
    #if defined( RCC_PLLCFGR_PLLPDIV )
        uint32_t temp;
    #endif

#if defined(STM32H7_FAMILY)
    switch ( pllnum ) {
        case SYS_PLL1:
            cfgreg = RCC->PLL1DIVR;
            break;
        case SYS_PLL2:
            cfgreg = RCC->PLL1DIVR;
            break;
        case SYS_PLL3:
            cfgreg = RCC->PLL1DIVR;
            break;
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
    /*
     * P, Q, and R posisitons in all CFG-registers are the same.
     * So we can use one HAL mask and position constant for all PLLs
     */
    switch ( pll_line )  {
        case PLL_LINE_P:
            return maskout(cfgreg, RCC_PLL1DIVR_P1_Msk, RCC_PLL1DIVR_P1_Pos) + 1;
        case PLL_LINE_Q:
            return maskout(cfgreg, RCC_PLL1DIVR_Q1_Msk, RCC_PLL1DIVR_Q1_Pos) + 1;
        case PLL_LINE_R:
            return maskout(cfgreg, RCC_PLL1DIVR_R1_Msk, RCC_PLL1DIVR_R1_Pos) + 1;
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }

#elif defined(STM32L4_FAMILY) 
    switch ( pllnum ) {
        case SYS_PLL1:
            cfgreg = RCC->PLLCFGR;
            break;
        case SYS_PLL2:
            cfgreg = RCC->PLLSAI1CFGR;
            break;
        #if defined(PLL_HAS_PLL3)
            case SYS_PLL3:
                cfgreg = RCC->PLLSAI2CFGR;
                break;
        #endif
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }
    /*
     * P, Q, and R posisitons in all CFG-registers are the same.
     * So we can use one HAL mask and position constant for all PLLs
     */
    switch ( pll_line )  {
        case PLL_LINE_P:
           /* If PLLP division bits are defined, these bit define the divisor, if = 0 */
           #if defined( RCC_PLLCFGR_PLLPDIV )
                temp = maskout(cfgreg, RCC_PLLCFGR_PLLPDIV_Msk , RCC_PLLCFGR_PLLPDIV_Pos);
                if ( temp ) return temp;
           #endif
           /* PLLP division bits undefined or 0: Must check PLLP bit */
           return cfgreg & RCC_PLLCFGR_PLLP ? 17 : 7;
        case PLL_LINE_Q:
            /* 2, 4, 6 or 8 are valid */
            return 2 * (maskout(cfgreg, RCC_PLLCFGR_PLLQEN_Msk, RCC_PLLCFGR_PLLQEN_Pos) + 1);
        case PLL_LINE_R:
            /* 2, 4, 6 or 8 are valid */
            return 2 * (maskout(cfgreg, RCC_PLLCFGR_PLLREN_Msk, RCC_PLLCFGR_PLLREN_Pos) + 1);
        default:
            return PLL_CONFIG_PARAM_ERROR;
    }

#else
    #error "No Implemetation of ""Pll_GetN"""
#endif
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
 * Set the PLL clock source globally and safe the PLL input frequency
 * @param Clocksource - one of the valid PLL clock sources defined in ...hal_rcc.h
 *                      ( differ from faimly to family )
 * @param srcclk_khz  - the input clock frequency in kHz. 
 * @returns PLL_CONFIG_OK if ok or PLL_CONFIG_INUSE if any PLL is in use 
 *          or PLL_CONFIG_PARAM_ERROR if no valid clock source is given
 * @Note Clocksource can be set only if NO PLL is active!
 *****************************************************************************/
uint32_t PLL_SetClockSource( uint32_t clocksource, uint32_t srcclk_khz)
{
    /* Check clock source */
    if (!IS_RCC_PLLSOURCE(clocksource) ) return PLL_CONFIG_PARAM_ERROR;

    /* Check for all PLLs being inactive */
    for ( uint32_t i = 0; i < MAX_PLLNUM; i++ )
        if ( PLL_InUse(i) ) return PLL_CONFIG_INUSE;
        
    /* Set the PLL clock source and save the pll input frequency */
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(clocksource);
    pll_input_frq = srcclk_khz;

    return PLL_CONFIG_OK;
}

/******************************************************************************
 * Return the PLL input frequency as set by "PLL_SetClockSource"
 * @returns PLL input frequency in kHz or PLL_CONFIG_NOTSET, if not set before
 *****************************************************************************/
int32_t Pll_GetInputFrqKhz ( void )
{
    return pll_input_frq==0 ? PLL_CONFIG_NOTSET : (int32_t)pll_input_frq;
}

/******************************************************************************
 * Program one or more lines of a specific PLL
 * @param pllnum - index of PLL to activate
 * @param PLL    - all neccessary parameters of the PLL 
 * @returns PLL_CONFIG_OK, if ok or PLL_CONFIG_INUSE, if PLL is active or
 *          PLL_CONFIG_PARAM_ERROR if illegal pllnum is given
 * @note    The PLL is not activated! This has to be done separately.
 *****************************************************************************/
int32_t PLL_Set ( RCC_PLLInitTypeDef *PLL, uint32_t pllnum )
{
    /* PLL or PLL line can only configured, if whole PLL is inactive */
    if ( PLL_InUse(pllnum) ) return PLL_CONFIG_INUSE;

#if defined(STM32H7_FAMILY)
    /* 
     * Write M, N, reset fractional bit, 
     * write Range and VCOSEL bits
     * write P,Q or R divisors and enable P,Q or R line if these are set 
     */
    switch(pllnum) {
        case SYS_PLL1:
            MASKIN(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1_Msk, PLL->PLLM,   RCC_PLLCKSELR_DIVM1_Pos);
            MASKIN(RCC->PLL1DIVR,  RCC_PLL1DIVR_N1_Msk,     PLL->PLLN-1, RCC_PLL1DIVR_N1_Pos);
            CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1VCOSEL, PLL->PLLVCOSEL);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1RGE_Msk, PLL->PLLRGE);
            if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL1DIVR,  RCC_PLL1DIVR_P1_Msk, PLL->PLLP-1, RCC_PLL1DIVR_P1_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVP1EN);
            }
            if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL1DIVR,  RCC_PLL1DIVR_Q1_Msk, PLL->PLLQ-1, RCC_PLL1DIVR_Q1_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVQ1EN);
            }
            if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL1DIVR,  RCC_PLL1DIVR_R1_Msk, PLL->PLLR-1, RCC_PLL1DIVR_R1_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVR1EN);
            }
            break;
        case SYS_PLL2:
            MASKIN(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM2_Msk, PLL->PLLM,   RCC_PLLCKSELR_DIVM2_Pos);
            MASKIN(RCC->PLL2DIVR,  RCC_PLL2DIVR_N2_Msk,     PLL->PLLN-1, RCC_PLL2DIVR_N2_Pos);
            CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2VCOSEL, PLL->PLLVCOSEL);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2RGE_Msk, PLL->PLLRGE);
            if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL2DIVR,  RCC_PLL2DIVR_P2_Msk,  PLL->PLLP-1, RCC_PLL2DIVR_P2_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVP2EN);
            }
            if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL2DIVR,  RCC_PLL2DIVR_Q2_Msk,  PLL->PLLQ-1, RCC_PLL2DIVR_Q2_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVQ2EN);
            }
            if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL2DIVR,  RCC_PLL2DIVR_R2_Msk,  PLL->PLLR-1, RCC_PLL2DIVR_R2_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVR2EN);
            }
            break;
        case SYS_PLL3:
            MASKIN(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM3_Msk, PLL->PLLM,   RCC_PLLCKSELR_DIVM3_Pos);
            MASKIN(RCC->PLL3DIVR,  RCC_PLL3DIVR_N3_Msk,     PLL->PLLN-1, RCC_PLL3DIVR_N3_Pos);
            CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3FRACEN);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3VCOSEL, PLL->PLLVCOSEL);
            MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3RGE_Msk, PLL->PLLRGE);
            if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL3DIVR,  RCC_PLL3DIVR_P3_Msk,  PLL->PLLP-1, RCC_PLL3DIVR_P3_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVP3EN);
            }
            if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL3DIVR,  RCC_PLL3DIVR_Q3_Msk,  PLL->PLLQ-1, RCC_PLL3DIVR_Q3_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVQ3EN);
            }
            if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLL3DIVR,  RCC_PLL3DIVR_R3_Msk,  PLL->PLLR-1, RCC_PLL3DIVR_R3_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_DIVR3EN);
            }
            break;
        default:
            return PLL_CONFIG_PARAM_ERROR;
    } // switch
#elif defined(STM32L4_FAMILY) || defined(STM32L4PLUS_FAMILY)
    /* 
     * write M divisor, N Multiplier, P,Q or R divisors and enable P,Q or R line if these are set 
     * NB: on STM32L4 all PLLs share one M-Divider stage PLLSAI2 has no Q-Line
     */
    switch(pllnum) {
        case SYS_PLL1:
            MASKIN(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk,      PLL->PLLM-1, RCC_PLLCFGR_PLLM_Pos);
            MASKIN(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk,      PLL->PLLN,   RCC_PLLCFGR_PLLN_Pos);
            if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk,  PLL->PLLP,   RCC_PLLCFGR_PLLP_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_PLLPEN);
            }
            if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk,  PLL->PLLQ/2-1, RCC_PLLCFGR_PLLQ_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_PLLQEN);
            }
            if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk,  PLL->PLLR/2-1, RCC_PLLCFGR_PLLR_Pos);
                SET_BIT(RCC->PLLCFGR,  RCC_PLLCFGR_PLLREN);
            }
            break;
        case SYS_PLL2:
            #if defined(STM32L4PLUS_FAMILY)
                MASKIN(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1M_Msk,      PLL->PLLM-1, RCC_PLLSAI1CFGR_PLLSAI1M_Pos);
            #endif
            MASKIN(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N_Msk,      PLL->PLLN,   RCC_PLLSAI1CFGR_PLLSAI1N_Pos);
            if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P_Msk,  PLL->PLLP,   RCC_PLLSAI1CFGR_PLLSAI1P_Pos);
                SET_BIT(RCC->PLLSAI1CFGR,  RCC_PLLSAI1CFGR_PLLSAI1PEN);
            }
            if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q_Msk,  PLL->PLLQ/2-1, RCC_PLLSAI1CFGR_PLLSAI1Q_Pos);
                SET_BIT(RCC->PLLSAI1CFGR,  RCC_PLLSAI1CFGR_PLLSAI1QEN);
            }
            if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                MASKIN(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R_Msk,  PLL->PLLR/2-1, RCC_PLLSAI1CFGR_PLLSAI1R_Pos);
                SET_BIT(RCC->PLLSAI1CFGR,  RCC_PLLSAI1CFGR_PLLSAI1REN);
            }
            break;
        #if defined(PLL_HAS_PLL3)
            case SYS_PLL3:
                #if defined(STM32L4PLUS_FAMILY)
                    MASKIN(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2M_Msk,      PLL->PLLM-1, RCC_PLLSAI2CFGR_PLLSAI2M_Pos);
                #endif
                MASKIN(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N_Msk,      PLL->PLLN,   RCC_PLLSAI2CFGR_PLLSAI2N_Pos);
                if ( PLL->PLLP != PLL_LINE_UNUSED ) {
                    MASKIN(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P_Msk,  PLL->PLLP,   RCC_PLLSAI2CFGR_PLLSAI2P_Pos);
                    SET_BIT(RCC->PLLSAI2CFGR,  RCC_PLLSAI2CFGR_PLLSAI2PEN);
                }
                #if defined(STM32L4PLUS_FAMILY)
                    if ( PLL->PLLQ != PLL_LINE_UNUSED ) {
                        MASKIN(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2Q_Msk,  PLL->PLLQ/2-1, RCC_PLLSAI2CFGR_PLLSAI2Q_Pos);
                        SET_BIT(RCC->PLLSAI2CFGR,  RCC_PLLSAI2CFGR_PLLSAI2QEN);
                    }
                #endif
                if ( PLL->PLLR != PLL_LINE_UNUSED ) {
                    MASKIN(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R_Msk,  PLL->PLLR/2-1, RCC_PLLSAI2CFGR_PLLSAI2R_Pos);
                    SET_BIT(RCC->PLLSAI2CFGR,  RCC_PLLSAI2CFGR_PLLSAI2REN);
                }
                break;
        #endif
        default:
            return PLL_CONFIG_PARAM_ERROR;
    } // switch
#else
    #error "No Implemetation of ""PLL_Set"""
#endif

    return PLL_CONFIG_OK;
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
static int32_t PLL_SetupMN ( RCC_PLLInitTypeDef *PLL, uint32_t pllnum, uint32_t pll_out_khz, uint32_t pll_inp_khz )
{
  uint32_t pll_khz_behind_m;    /* PLL frq behind the M stage (VCO) */
  uint32_t pll_delta_m;         /* increments for VCO to reach the minimum frq */

  /* calculate M so that the frq after M is always 2MHz, or - in case of HSE not a multiple of 2 - 5Mhz */
  if ( pll_inp_khz / PLLM_ODD * PLLM_ODD == pll_inp_khz ) {
    pll_khz_behind_m = pll_delta_m = PLLM_ODD;
  } else {
    pll_khz_behind_m = pll_delta_m = PLLM_EVEN;
  }

  /* if we have a minimum value restriction fot this stage, increment until above that minimum */
  if ( pll_restriction[pllnum].m_min > 0 ) {
    while ( pll_khz_behind_m < pll_restriction[pllnum].m_min ) 
        pll_khz_behind_m += pll_delta_m;
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
     pqr_div = (n_min + pll_out_khz - 1)/ pll_out_khz;
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
int32_t PLL_Configure (RCC_PLLInitTypeDef *PLL, uint32_t pllnum, uint32_t pll_line, uint32_t pll_out_khz )
{
  int32_t work;

  /* Clear PLL data structure */  
  memset(PLL, 0, sizeof(RCC_PLLInitTypeDef) );

  /* Assert PLL input frequency has been set before */
  if ( pll_input_frq == 0 ) return PLL_CONFIG_NOTSET;
  /* 
   * Check, whether PLL or PLL line is in use 
   * This is the case, if M is already set AND the currently configured line is not the only line used
   */

  /* Get M and N value and check for error */
  work = Pll_GetM(pllnum);
  if ( work < 0 ) return work;
  if ( work > 0  && Pll_GetActiveLines(pllnum) && Pll_GetActiveLines(pllnum) != pll_line ) {
    /* If PLL is in use, set M and N to the values actually configured in PLL without any checks */
    PLL->PLLM = work;
    work = Pll_GetN(pllnum);
    if ( work < 0 ) return work;
    PLL->PLLN =work;
  } else {
    /* otherwise compute M and N and check for errors */
    int32_t work = PLL_SetupMN( PLL, pllnum, pll_out_khz, pll_input_frq);
    if ( work < 0 ) return work;
  }

  /* calculate divisor of final stage and check restrictions for max. Divisor of final stage */
  uint32_t pll_behind_n = pll_input_frq / PLL->PLLM * PLL->PLLN;
  uint32_t pqr_div = pll_behind_n / pll_out_khz;
  if ( pqr_div > pll_restriction[pllnum].pqrdiv_max ) return  PLL_CONFIG_VIOLATION_PQR;

  /* check, whether the desired frequency is exactly hit */
  if ( pll_behind_n / pqr_div != pll_out_khz ) return PLL_CONFIG_UNABLE;

  PLL->PLLP = PLL->PLLQ = PLL->PLLR = PLL_LINE_UNUSED;
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

#if defined (STM32H7_FAMILY)
  /* Fractional divider, VCO and range select only in STM32H7 */
  PLL->PLLFRACN = 0;
  /* If SYSCLK <= 200 MHZ, select VCO medium */
  PLL->PLLVCOSEL = ( pll_behind_n <= 200000 ? RCC_PLL1VCOMEDIUM : RCC_PLL1VCOWIDE);
  /* 2 MHz Input is either Range 0 or Range 1 */
  if ( pll_input_frq / PLL->PLLM  <= 4000 ) 
    PLL->PLLRGE = RCC_PLL1VCIRANGE_1;
  else if ( pll_input_frq / PLL->PLLM  <= 8000 ) 
    PLL->PLLRGE = RCC_PLL1VCIRANGE_2;
  else
    PLL->PLLRGE = RCC_PLL1VCIRANGE_2;
#endif

  return PLL_CONFIG_OK;
}

/******************************************************************************
 * configure the PLL and PLL line that will feed the SYSCLK Multiplexer
 * @param RCC_OscInitStruct - to be filled with PLL parameters
 * @param pll_out_khz - desired SYSCLK 
 * @param pll_inp_khz - actual PLL input frequency
 * @returns           - any of the PLL_CONFIG_XXXX values defined in pll.h
 * @Note the field PLL.PLLSource MUST be set upon call of this function
 *****************************************************************************/
int32_t PLL_Configure_SYSCLK (RCC_OscInitTypeDef *RCC_OscInitStruct, uint32_t pll_out_khz, uint32_t pll_inp_khz )
{

    int32_t ret; 

    ret = PLL_SetClockSource(RCC_OscInitStruct->PLL.PLLSource, pll_inp_khz);
    if ( ret < 0 ) return ret;

    ret =  PLL_Configure(&RCC_OscInitStruct->PLL, SYSCLK_PLL, SYSCLK_PLL_LINE, pll_out_khz);
    if ( ret < 0 ) return ret;


    return PLL_CONFIG_OK;
}

/******************************************************************************
 * Configure one line of one PLL 
 * @param pllnum      - Number of the PLL to be configured
 * @param pllline     - Line Number of the PLL to be configured
 * @returns           - any of the PLL_CONFIG_XXXX values defined in pll.h
 * @Note    1. The whole PLL whose line is to be configured must be unused pon call
 * @Note    2. Due to already set M and N values, the desired output may be unable
 *             to be hit exactly. In this case PLL_CONFIG_UNABLE is returned
 * @Note    3. PLL is not started!
  *****************************************************************************/
int32_t PLL_Configure_Line ( uint32_t pllnum, uint32_t pllline, uint32_t pll_out_khz )
{
    RCC_PLLInitTypeDef PLL;
    int32_t ret;

    /* Try to setup the PLL parameters */
    ret = PLL_Configure(&PLL, pllnum, pllline, pll_out_khz);
    if ( ret < 0 ) return ret;

    /* If successful configured, setup PLL */
    ret = PLL_Set ( &PLL, pllnum );
    if ( ret < 0 ) return ret;

    return PLL_CONFIG_OK;
}

/******************************************************************************
 * Return the PLL output frequency of the specfied line of the specified PLL
 * @param pllnum      - Number of the PLL to be requested
 * @param pllline     - Line Number of the PLL to be requested
 * @returns           - The actual PLL output frequency in kHz, if PLL line is active
 *                    - 0xFFFFFFFF, if the pll line is not active
  *****************************************************************************/
uint32_t PLL_GetOutFrq ( uint32_t pllnum, uint32_t pll_line )
{

    /* First ensure, that PLL line is active */
    if ( Pll_GetActiveLines(pllnum) &&  Pll_GetActiveLines(pllnum) & pll_line ) {
        return pll_input_frq / Pll_GetM(pllnum) * Pll_GetN(pllnum) / Pll_GetPQR(pllnum, pll_line);
    } else {
        /* Line is not active */
        return PLL_LINE_UNUSED;
    }
}
