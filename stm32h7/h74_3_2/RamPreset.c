#include "config/config.h"

#if defined(STM32H7_FAMILY)
    #include "stm32h7xx.h"
#elif defined(STM32L4_FAMILY)
    #include "stm32l4xx.h"
#else
    #error "Unkonwn MCU family"
#endif

#define PRESET_RAMAREA(seg) \
  extern uint32_t seg##_segment_start__; \
  extern uint32_t seg##_segment_end__;   \
  RamPtr = &(seg##_segment_start__);     \
    RamEnd = &(seg##_segment_end__);     \
    while ( RamPtr < RamEnd ) {          \
        *RamPtr = preset;                \
        RamPtr++;                        \
    }                                    \


void RamPreset(void)
{
    register uint32_t preset = RAM_PRESET_VALUE;
    register uint32_t *RamPtr;
    register uint32_t *RamEnd;
    /* 
     * Enable SRAM 1,2, and SRAM3, if available 
     * **** 002 in clues.txt RHB added ****
     */
    #if defined(STM32H742xx)
        RCC->AHB2ENR = RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN;
    #else
        RCC->AHB2ENR = RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN;
    #endif

    PRESET_RAMAREA(__SRAM1);
    PRESET_RAMAREA(__SRAM2);

    #if !defined(STM32H742xx)
        PRESET_RAMAREA(__SRAM3);
    #endif
    PRESET_RAMAREA(__SRAM4);
    PRESET_RAMAREA(__AXISRAM);
    PRESET_RAMAREA(__DTCM);


   /**** 002 ****
    * Disable all previously enabled Domain-2-elements again.
    * This is neccessary to recognize in a multi-core environment, that the CM4 core in D2 
    * has stopped by evaluating RCC->CR D2CKRDY bit
    */

   RCC->AHB2ENR = 0;

}