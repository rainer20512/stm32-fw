/**
  ******************************************************************************
  * @file    debug_util.c
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



#define A(p)  ((uint32_t)(&p))

#define DO_DUMP_SECTION(a,b) \
  extern uint32_t b##_start__; \
  extern uint32_t b##_end__; \
  DBG_dump_section(a, A(b##_start__ ), A(b##_end__ ),false )

#define DO_DUMP_RAMAREA(a,b) \
  extern uint32_t b##_start__; \
  extern uint32_t b##_end__; \
  DBG_dump_sram_area(a, A(b##_start__ ), A(b##_end__ ) )


/*
 *************************************************************
 * local helper functions 
 *************************************************************
 */

/* switch off warning regarding missing return statement */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
static uint32_t get_sp ( void )

{
   asm ("mov r3,sp;" );
}
/* re-enable warning */
#pragma GCC diagnostic pop

/*****************************************************************************
 * @brief get the number of used ram bytes in the specified RAM area
 * @param from RAM address of begin of area
 * @param to RAM adress of first byte _behind_ area
 * @ returns number of used bytes in this area
 * @note: although the parameters are dword addresses, the number of used 
 *        elements is returned in bytes
 *****************************************************************************/
static uint32_t get_ram_useage ( uint32_t *from, uint32_t *to )
{
  uint32_t ret = 0;
  while ( from < to ) {
    if ( *from != RAM_PRESET_VALUE ) ret++;
    from++;
  }
  return ret << 2;
}

/*****************************************************************************
 * @brief return the maximum stack useage
 * @param from - bottom of stack
 * @param to   - top of stack
 * @ returns number of used bytes in this area
 * @note: although the parameters are dword addresses, the number of used 
 *        elements is returned in bytes
 *****************************************************************************/
static uint32_t get_max_stack_useage ( uint32_t *from, uint32_t *to )
{
  uint32_t *p = from;
  uint32_t min_free; 
  while ( p < to && *p == RAM_PRESET_VALUE ) {
    p++;
  }
  min_free = p - from;
  return (( to - from ) - min_free) << 2 ;
}

static void DBG_dump_sram_area( const char *text, uint32_t start, uint32_t end )
{
  uint32_t total_size;
  uint32_t used_size;
  DBG_printf_indent("%s\n", text );
  int oldIndent = DBG_setIndentRel(+2);

  total_size = end - start;
  used_size = get_ram_useage( (uint32_t *)start, (uint32_t *)end);
  DBG_dump_section(text, start, end ,true );  
  DBG_dump_section_useage ("Used",  used_size, total_size );

  DBG_setIndentAbs(oldIndent);
}

#if defined(STM32L43xx) || defined(STM32L476xx) || defined(STM32L496xx)
static void DBG_dump_sram_areas(void)
{
  DBG_setPadLen(6);
  DO_DUMP_RAMAREA("RAM",__RAM_segment);
  DO_DUMP_RAMAREA("SRAM2",__SRAM2_segment);
  DEBUG_PUTC('\n');
}
#elif defined(STM32L4PLUS)
static void DBG_dump_sram_areas(void)
{
  DBG_setPadLen(6);
  DO_DUMP_RAMAREA("RAM",__RAM_segment);
  DO_DUMP_RAMAREA("SRAM2",__SRAM2_segment);
  DO_DUMP_RAMAREA("SRAM3",__SRAM3_segment);
  DEBUG_PUTC('\n');
}
#elif defined(STM32H747xx) || defined(STM32H745xx)
static void DBG_dump_sram_areas(void)
{
  DBG_setPadLen(8);
#if defined(CORE_CM7)
      DO_DUMP_RAMAREA("DTCM",__DTCM_segment);
      DO_DUMP_RAMAREA("AXISRAM",__AXISRAM_segment);
#endif
  DO_DUMP_RAMAREA("SRAM1",__SRAM1_segment);
  DO_DUMP_RAMAREA("SRAM2",__SRAM2_segment);
  DO_DUMP_RAMAREA("SRAM3",__SRAM3_segment);
  DO_DUMP_RAMAREA("SRAM4CM7",__SRAM4CM4_segment);
  DO_DUMP_RAMAREA("SRAM4CM4",__SRAM4CM7_segment);
  DEBUG_PUTC('\n');
}
#elif defined(STM32H742xx) || defined(STM32H743xx)
static void DBG_dump_sram_areas(void)
{
  DBG_setPadLen(8);
  DO_DUMP_RAMAREA("DTCM",__DTCM_segment);
  DO_DUMP_RAMAREA("AXISRAM",__AXISRAM_segment);
  DO_DUMP_RAMAREA("SRAM1",__SRAM1_segment);
  DO_DUMP_RAMAREA("SRAM2",__SRAM2_segment);
  #if !defined(STM32H742xx)
      DO_DUMP_RAMAREA("SRAM3",__SRAM3_segment);
  #endif
  DO_DUMP_RAMAREA("SRAM4",__SRAM4_segment);
  DEBUG_PUTC('\n');
}
#else
    #error "No RAM dump routine defined for selected hardware"
#endif

static void DBG_dump_sram_sections(void)
{
  DBG_setPadLen(16);
//  DO_DUMP_SECTION("IRQ-Vectors",    __vectors_ram );
  DO_DUMP_SECTION("IRQ-Vectors",    __vectors );
  DO_DUMP_SECTION("Init",           __init );
  DO_DUMP_SECTION("Text",           __text );
  DO_DUMP_SECTION("R/O Data",       __rodata );
  DO_DUMP_SECTION("Fast",           __fast);
//  DO_DUMP_SECTION("Fast run",       __fast_run);
  DO_DUMP_SECTION("Data",           __data);
  DO_DUMP_SECTION("BSS",            __bss);
  DO_DUMP_SECTION("TBSS",           __tbss);
  DO_DUMP_SECTION("Tdata",          __tdata);
//  DO_DUMP_SECTION("Tdata run",      __tdata_run);
  DO_DUMP_SECTION("Non init",       __non_init);
  DO_DUMP_SECTION("Heap",           __heap);
  DO_DUMP_SECTION("Stack",          __stack);
  DO_DUMP_SECTION("Process Stack",  __stack_process);
  DBG_dump_stack_useage( get_sp(), get_max_stack_useage(&__stack_start__,  &__stack_end__), A(__stack_start__),  A(__stack_end__) );
  DEBUG_PUTC('\n');
}

/*
extern uint32_t __init_rodata_start__;
extern uint32_t __init_rodata_end__;
extern uint32_t __text_start__;
extern uint32_t __text_end__;
extern uint32_t __rodata_start__;
extern uint32_t __rodata_end__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
*/


void DBG_sram(void)
{
  DEBUG_PUTS("SRAM properties ------------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);

  DBG_printf_indent("RAM Areas \n" );
  DBG_setIndentRel(+2);
  DBG_dump_sram_areas();
  DBG_setIndentRel(-2);

  DBG_printf_indent("RAM and Flash Sections \n" );
  DBG_setIndentRel(+2);
  DBG_dump_sram_sections();
  DBG_setIndentRel(-2);

  DBG_setIndentAbs(oldIndent);
}


#endif // #if DEBUG_FEATURES > 0

/**
  * @}
  */
