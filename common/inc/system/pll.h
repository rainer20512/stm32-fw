/**
  ******************************************************************************
  * @file    pll.h 
  * @author  Rainer
  * @brief   all the stuff to do configuring and bookkeeping of the systems PLLs. 
  *          
  * @note    pll.h and pll.c are valid for all platforms, 
  *          pll_config.h is specific for STM32L4 and STM32H7 family
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLL_H
#define __PLL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Names/Aliases for the different PLLs */
#define SYS_PLL1        0
#define SYS_PLL         0
#define SYS_PLL2        1
#define SYS_PLLSAI1     1
#define SYS_PLL3        2
#define SYS_PLLSAI2     2


/* Names/Aliases for the lines within a PLL */
#define PLL_LINE_P       (1 << 0)
#define PLL_LINE_Q       (1 << 1)
#define PLL_LINE_R       (1 << 2)

/* magic value, that indicates "this line is unused" */
#define PLL_LINE_UNUSED  0xFFFFFFFF


/* PLL config return values */
#define PLL_CONFIG_OK               0             /* success                                             */
#define PLL_CONFIG_ERROR            (-1)          /* An unspecific error                                 */
#define PLL_CONFIG_VIOLATION_N      (-2)          /* Violation of N stage output frequency restriction   */
#define PLL_CONFIG_VIOLATION_M      (-3)          /* Violation of M stage output frequency restriction   */
#define PLL_CONFIG_VIOLATION_PQR    (-4)          /* Violation of P,Q or R output frequency restriction  */
#define PLL_CONFIG_INUSE            (-5)          /* PLL to configure is in use                          */
#define PLL_CONFIG_PARAM_ERROR      (-6)          /* invalid Parameter                                   */
#define PLL_CONFIG_NOTSET           (-7)          /* queried parameter is not set                        */
#define PLL_CONFIG_TIMEOUT          (-8)          /* timeout when polling for certain events             */
#define PLL_CONFIG_UNABLE           (-9)          /* output value not settable due to already set M or N */

uint32_t PLL_InUse( uint32_t pllnum );
int32_t  PLL_Set ( RCC_PLLInitTypeDef *PLL, uint32_t pllnum );
int32_t  PLL_Start(uint32_t pllnum );
int32_t  PLL_Stop(uint32_t pllnum );
void     PLL_Restart(void);
uint32_t PLL_SetClockSource( uint32_t clocksource, uint32_t srcclk_khz);
int32_t  PLL_Configure (RCC_PLLInitTypeDef *PLL, uint32_t pllnum, uint32_t pll_line, uint32_t pll_out_khz );
int32_t  PLL_Configure_SYSCLK (RCC_OscInitTypeDef*, uint32_t pll_out_khz, uint32_t pll_inp_khz );
int32_t  PLL_Configure_Line ( uint32_t pllnum, uint32_t pllline, uint32_t pll_out_khz );
const char * PLL_GetName(uint32_t pllnum );
uint32_t PLL_GetOutFrq ( uint32_t pllnum, uint32_t pll_line );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PLL_H */
