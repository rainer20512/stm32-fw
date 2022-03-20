/******************************************************************************
 * Preset all RAM with an uniform preset pattern to measure the dynamic ram
 * usage.
 * 
 * Note: DO NOT USE functions calls except to RamPreset, because the stack
 * will be overwritten, too
 *
 * Only one call is allowed, the return adress for this call is stored in lr. 
 * And this call is the call to RamPreset! In normal cases, this call is done
 * from the startup code only, no need to do that from user code.
 * 
 * This ist the STM32L4-family implementation
 * 
 *001*
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


void RamPreset(void)
{
    register uint32_t preset = RAM_PRESET_VALUE;
    register uint32_t *RamPtr;
    register uint32_t *RamEnd;

    PRESET_RAMAREA(__RAM);
    PRESET_RAMAREA(__SRAM2);

    #if defined(STM32L4PLUS)
        PRESET_RAMAREA(__SRAM3);
    #endif

}

#endif /* DO_RAM_PRESET */