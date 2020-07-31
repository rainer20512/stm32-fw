/******************************************************************************
 * code for a RAM preset. this code is included by STM32_Startup.s, so no
 * section or global definitions are neccessary
 * 
 * This file : RAM  preset for CM7: Preset all RAM except SRAM1, which is used by 
 * CM4 in parallel and which will be preset by CM4, if desired
 *
 * Store this file in one directoy, that is contained in the include search path
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
  beq r03
  str r2, [r0]
  adds r0, r0, #4
  b r00

r03:
  ldr r0, =__AXISRAM_segment_start__
  ldr r1, =__AXISRAM_segment_end__
r04:
  cmp r0, r1
  beq r05
  str r2, [r0]
  adds r0, r0, #4
  b r04

r05:
  ldr r0, =__DTCM_segment_start__
  ldr r1, =__DTCM_segment_end__
r06:
  cmp r0, r1
  beq r07
  str r2, [r0]
  adds r0, r0, #4
  b r06

r07:
  ldr r0, =__SRAM3_segment_start__
  ldr r1, =__SRAM3_segment_end__
r08:
  cmp r0, r1
  beq r09
  str r2, [r0]
  adds r0, r0, #4
  b r08

r09:
  ldr r0, =__SRAM4_segment_start__
  ldr r1, =__SRAM4_segment_end__
r10:
  cmp r0, r1
  beq r11
  str r2, [r0]
  adds r0, r0, #4
  b r10
r11: