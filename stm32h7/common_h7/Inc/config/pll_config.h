/**
  ******************************************************************************
  * @file    pll_config.h 
  * @author  Rainer
  * @brief   all the stuff to do configuring and bookkeeping of the systems PLLs. 
  *          Here the configuration of STM32H7xx devices
  *
  ******************************************************************************
  */

/* Get the PLL names/aliases */
#include "system/pll.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLL_CONFIG_H
#define __PLL_CONFIG_H

#define PLL_NAMELEN     10    /* Max length of PLL name string ( including terminating \0 ) */
#define MAX_LINENUM     3     /* maximum number of lines per PLL ( 3 in most cases )        */
#define MAX_PLLNUM      3     /* maximum number of PLLs supported */

/* 
 * Array of "On"- and "Ready"-Flags in RCC->CR register 
 * RCC constant, that defines PLL as SYSCLK source
 */
#define PLLON_FLAGS     { RCC_CR_PLL1ON,  RCC_CR_PLL2ON,  RCC_CR_PLL3ON, }
#define PLLRDY_FLAGS    { RCC_CR_PLL1RDY, RCC_CR_PLL2RDY, RCC_CR_PLL1RDY, }
#define RCC_SYSPLL_MASK RCC_CFGR_SWS_PLL1

#define SYSCLK_PLL      SYS_PLL1   /* which PLL will generate the PLL SYSCLK ?                   */
#define SYSCLK_PLL_LINE PLL_LINE_P /* Which line in this PLL will generate the SYSCLK            */

/* Minimum and maximum output frq. after M-Stage in kHz */
#define PLLM_MIN        1000
#define PLLM_MAX        64000

#define PLLM_EVEN       2000  /* standard output frq of M-stage for even input frequencies */
#define PLLM_ODD        5000  /* standard output frq of M-stage for off input frequencies  */
#define PLL_PQR_DIVMAX  128   /* highest possible divisor in final stage                   */

/* We have three PLLs each of them has P,Q and R outputs */

/* PLL1 ------------------------------------------------ */
#define HAS_PLL1        1
#define PLL1NAME        "PLL1"
#define PLL1P           1
#define PLL1Q           1
#define PLL1R           1

/* Restrictions for PLL1 
     * - N Stage output frq between 192 and 836 MHz
     * - P Stage output max 480 MHz
    */
    #define PLLN1_MIN       192000
    #define PLLN1_MAX       960000
    #define PLLP1_MAX       480000


/* PLL2 ------------------------------------------------ */
#define HAS_PLL2        1
#define PLL2NAME        "PLL2"
#define PLL2P           1
#define PLL2Q           1
#define PLL2R           1
        
/* Restrictions for PLL2 
 * N Stage output frq between 192 and 960 MHz
*/
#define PLLN2_MIN       192000
#define PLLN2_MAX       960000

/* PLL3 ------------------------------------------------ */
#define HAS_PLL3        1
#define PLL3NAME        "PLL3"
#define PLL3P           1
#define PLL3Q           1
#define PLL3R           1

/* Restrictions for PLL3 
 * N Stage output frq between 192 and 960 MHz
*/
#define PLLN3_MIN       PLLN2_MIN
#define PLLN3_MAX       PLLN2_MAX

#endif /* __PLL_CONFIG_h */