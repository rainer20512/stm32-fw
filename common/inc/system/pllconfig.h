/**
  ******************************************************************************
  * @file    pllconfig.h 
  * @author  Rainer
  * @brief   all the stuff to do configuring and bookkeeping of the systems PLLs. 
  * 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLLCONFIG_H
#define __PLLCONFIG_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define PLL_NAMELEN         10    /* Max length of PLL name string ( including terminating \0 ) */
#define MAX_LINENUM         3     /* maximum number of lines per PLL ( 3 in most cases )        */
#define MAX_PLLNUM          3     /* maximum number of PLLs supported */

#if defined(STM32H7_FAMILY)
    /* Minimum and maximum output frq. after M-Stage in kHz */
    #define PLLM_MIN        1000
    #define PLLM_MAX        64000

    /* We have three PLLs each of them has P,Q and R outputs */

    /* PLL1 ------------------------------------------------ */
    #define HAS_PLL1        1
    #define PLL1NAME        "PLL1"
    #define PLL1P           1
    #define PLL1Q           1
    #define PLL1R           1

    /* Restrictions for PLL1 
     * - N Stage output frq between 150 and 960 MHz
     * - P Stage output max 480 MHz
    */
    #define PLLN1_MIN       150000
    #define PLLN1_MAX       960000
    #define PLLP1_MAX       480000


    /* PLL2 ------------------------------------------------ */
    #define HAS_PLL2        1
    #define PLL2NAME        "PLL2"
    #define PLL2P           1
    #define PLL2Q           1
    #define PLL2R           1
            
    /* Restrictions for PLL2 
     * N Stage output frq between 150 and 960 MHz
    */
    #define PLLN2_MIN       150000
    #define PLLN2_MAX       960000

    /* PLL3 ------------------------------------------------ */
    #define HAS_PLL3        1
    #define PLL3NAME        "PLL3"
    #define PLL3P           1
    #define PLL3Q           1
    #define PLL3R           1

    /* Restrictions for PLL3 
     * N Stage output frq between 150 and 960 MHz
    */
    #define PLLN3_MIN       150000
    #define PLLN3_MAX       960000

#elif defined(STM32L4PLUS_FAMILY)
    /* Minimum and maximum output frq. after M-Stage in kHz */
    #define PLLM_MIN        2660
    #define PLLM_MAX        8000

    /* We have three PLLs, named PLL, PLLSAI1 and PLLSAI2 */
    /* all these have the P,Q and R output                */

    /* PLL1 ------------------------------------------------ */
    #define HAS_PLL1        1
    #define PLL1NAME        "PLL1"
    #define PLL1P           1
    #define PLL1Q           1
    #define PLL1R           1

    /* Restrictions for PLL1 
     * - N Stage output frq between 64 and 344 MHz
     * - P,Q,R Stage output max 120 MHz
    */
    #define PLLN1_MIN       64000
    #define PLLN1_MAX       344000
    #define PLLP1_MAX       120000
    #define PLLQ1_MAX       120000
    #define PLLR1_MAX       120000


    /* PLL2 ------------------------------------------------ */
    #define HAS_PLL2        1
    #define PLL2NAME        "PLL2"
    #define PLL2P           1
    #define PLL2Q           1
    #define PLL2R           1
            
    /* Restrictions for PLL2 
     * - N Stage output frq between 64 and 344 MHz
     * - P,Q,R Stage output max 120 MHz
    */
    #define PLLN2_MIN       64000
    #define PLLN2_MAX       344000
    #define PLLQ2_MAX       120000

    /* PLL3 ------------------------------------------------ */
    #define HAS_PLL3        1
    #define PLL3NAME        "PLL3"
    #define PLL3P           1
    #define PLL3Q           1
    #define PLL3R           1

    /* Restrictions for PLL3 
     * N Stage output frq between 150 and 960 MHz
    */
    #define PLLN3_MIN       150000
    #define PLLN3_MAX       960000


#elif defined(STM32L4_FAMILY)
    /* Minimum and maximum output frq. after M-Stage in kHz */
    #define PLLM_MIN        4000
    #define PLLM_MAX        64000

    /* We have three PLLs, named PLL, PLLSAI1 and PLLSAI2 */

    /* PLL1 ------------------------------------------------ */
    #define HAS_PLL1        1
    #define PLL1MAME        "PLL"
    #define PLL1P           1
    #define PLL1Q           1
    #define PLL1R           1

    /* Restrictions for PLL1 
     * - N Stage output frq between 64 and 344 MHz
     * - P,Q,R Stage output between 8 and 80 MHz
    */
    #define PLLN1_MIN       64000
    #define PLLN1_MAX       344000
    #define PLLP1_MIN       8000
    #define PLLP1_MAX       80000
    #define PLLQ1_MIN       8000
    #define PLLQ1_MAX       80000
    #define PLLR1_MIN       8000
    #define PLLR1_MAX       80000


    /* PLLSAI1 ------------------------------------------------ */
    #define HAS_PLL2        1
    #define PLL2NAME        "PLLSAI1"
    #define PLL2P           1
    #define PLL2Q           1
    #define PLL2R           1
            
    /* Restrictions for PLL2 
     * - N Stage output frq between 64 and 344 MHz
     * - P,Q,R Stage output between 8 and 80 MHz
    */
    #define PLLN2_MIN       64000
    #define PLLN2_MAX       344000
    #define PLLP2_MIN       8000
    #define PLLP2_MAX       80000
    #define PLLQ2_MIN       8000
    #define PLLQ2_MAX       80000
    #define PLLR2_MIN       8000
    #define PLLR2_MAX       80000

    /* PLL3 ------------------------------------------------ */
    #define HAS_PLL3        1
    #define PLL3NAME        "PLLSAI2"
    #define PLL3P           1
    #define PLL3Q           0       // PLLSAI2 has no Q output
    #define PLL3R           1

    /* Restrictions for PLL3 
     * - Stage output frq between 150 and 960 MHz
    */
    #define PLLN3_MIN       64000
    #define PLLN3_MAX       344000


#else
    #error "No PLL configuration data for selected STM32 family"
#endif

#endif /* __PLLCONFIG_h */