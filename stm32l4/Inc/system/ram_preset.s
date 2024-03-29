/******************************************************************************
 * code for a RAM preset. this code is included by STM32_Startup.s, so no
 * section or global definitions are neccessary
 * 
 * This file : RAM  preset for STM32L4xx
 *   - Preset RAM  and SRAM2 
 *
 * Store this file under on dir, that is contained in the include search path
 * with the subpath/name "system/ram_preset.s"
 * 
 ******************************************************************************
 */

  movw r2, RAM_PRESET_VALUE & 0xFFFF
  movt r2, RAM_PRESET_VALUE >> 16

  ldr r0, =__SRAM2_segment_start__
  ldr r1, =__SRAM2_segment_end__
r00:
  cmp r0, r1
  beq r01
  str r2, [r0]
  adds r0, r0, #4
  b r00

r01:
  ldr r0, =__non_init_start__
  ldr r1, =__RAM_segment_end__

r02:
  cmp r0, r1
  beq r03
  str r2, [r0]
  adds r0, r0, #4
  b r02

r03: