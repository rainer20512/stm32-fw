/******************************************************************************
 * code for a RAM preset. this code is included by STM32_Startup.s, so no
 * section or global definitions are neccessary
 * 
 * This file : RAM  preset for CM4: Preset SRAM1 _only_ , this is the
 * only memory that is assigned to CM4
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

  ldr r0, =__SRAM1_segment_start__
  ldr r1, =__SRAM1_segment_end__
r00:
  cmp r0, r1
  beq r03
  str r2, [r0]
  adds r0, r0, #4
  b r00

r03:
