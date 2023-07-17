/******************************************************************************
 * Preset all RAM with an uniform preset pattern to measure the dynamic ram
 * usage.
 * 
 * Note: DO NOT USE functions calls except to RamPreset, because the stack
 * will be overwritten, too
 *
 * Only one call is allowed, the return adress for this call is stored in lr. 
 * And this call is the call to RamPreset!
 *
 * This is the STM32H7-family implementation
 * 
 ******************************************************************************
 */
 #include "config/config.h"

#ifdef DO_RAM_PRESET

#if defined(STM32H7_FAMILY)
    #include "stm32h7xx.h"
#elif defined(STM32L4_FAMILY)
    #include "stm32l4xx.h"
#else
    #error "Unkonwn MCU family"
#endif

/* 
 * Execute the RAM fill by macro. Do not use function calls,
 * because the stack will be overwritten, too.
 */

#define PRESET_RAMAREA(seg) \
  extern uint32_t seg##_segment_start__; \
  extern uint32_t seg##_segment_end__;   \
  RamPtr = &(seg##_segment_start__);     \
    RamEnd = &(seg##_segment_end__);     \
    while ( RamPtr < RamEnd ) {          \
        *RamPtr = preset;                \
        RamPtr++;                        \
    }                                    \


/*
 ******************************************************************************************
 * Preset all RAM with an uniform pattern. This has to be done by the CM7 core only
 * No actions for the CM4 core
 ******************************************************************************************
 */
void RamPreset(void)
{
    #if defined(CORE_CM7)
        register uint32_t preset = RAM_PRESET_VALUE;
        register uint32_t *RamPtr;
        register uint32_t *RamEnd;
        /* 
         * Enable SRAM 1,2, and SRAM3, if available 
         * **** 002 in clues.txt RHB added ****
         */
        #if defined(RCC_AHB2ENR_SRAM3EN)
            RCC->AHB2ENR = RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN;
        #else
            RCC->AHB2ENR = RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN;
        #endif

        PRESET_RAMAREA(__SRAM1);
        PRESET_RAMAREA(__SRAM2);

        #if defined(RCC_AHB2ENR_SRAM3EN)
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
    #endif
}

#endif /* DO_RAM_PRESET */