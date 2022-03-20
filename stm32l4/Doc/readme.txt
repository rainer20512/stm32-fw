001:   RAM mit speziellem "unused"-pattern initialisieren
---------------------------------------------------------
Modify ...\Rowley Associates Limited\CrossWorks for ARM\v4\packages\targets\STM32\STM32_Startup.s at "reset_handler" to:

reset_handler:

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__RAM_segment_end__
  mov sp, r0
  bl SystemInit
#endif

// RHB chgd: Preset all RAM with an uniform pattern

#ifdef DO_RAM_PRESET
  ldr r0, =__RAM_segment_end__
  mov sp, r0
  bl RamPreset
#endif

// RHB End of insertion

#ifdef VECTORS_IN_RAM
  ..
  ..


Under system/ram_preset.c a code snippet must be provided, that will do the ram preset for the target processor
for STM32L4xx eg this could be

8x-----------------------------------------------------------------------------
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
--------------------------------------------------------------------------x8

002: Anderen Target-Prozessor auswählen
---------------------------------------

Das gestaltet sich kompliziert, weil aus irgendwelchen, mir nicht nachvollziehbaren Gründen, 
Crossworks ganz stur am alten Memory Map File festhält, also muss man es ihm mit Gewalt
wegnehmen, s. dazu die Punkte c - f:

- In den Project Properties 
   a  "Target Processor" anpassen, das ändert automatisch nach kurzer Zeit alle anderen
      Prozesssor-spezifischen Einstellungen, bis auf:
   b  "Preprocessor Definitions" anpassen
   c  "Memory Map File" anpassen, die liegen im Projektverzeichnis
   d  das alte Memory Map File umbenennen (!)
   e  altes ld-file löschen (!)
   f  Crossworks beenden und neu starten 

   zu b: Möglich sind: STM32L476xx, STM32L496xx, STM32L4Sxxx

003: Crosswork mkld loadfile generator files anpassen
-----------------------------------------------------
- Original liegen die in den Benutzerspezifischen Verzeichnissen 
- bzw im Crossworks-Programmverzeichnis
- Wenn der EEPROM emulator genutzt werden soll, ist es wichtig, vom
  FLASH-Umfang einige FLASH-Seiten für die Benutzung durch den EEPROM-
  Simulator "abzuziehen", damit die nicht für Programmcode verwendet
  werden. Das erfolgt im File "STM32Lxxxxx_MemoryMap.xml" 

004: Komische Dinge beim Ändern des MemoryMapFile-Parameters
------------------------------------------------------------
- auch wenn man das MemoryMapFile in den Projekteinstellungen ändert (z.B. beim
  Umstieg von STM32L476RG auf STM32L496RG), dann nimmt der LD-File-Generator MKLD
  immer noch das alte. Wenn man das .ld-file dann löscht, dann geht der Build überhaupt
  nicht mehr. 
  Abhilfe: Neues MemoryMapFile einstellen, altes löschen, CrossStudio neu starten
  Anmerkung: Offensichtlich wird dabei aus dem angegebenen Verzeichnis (ProjectDir)
  das erstbeste file genommen, was auf MemoryMap.xml endet. daher die nicht passenden
  memory map files abweichend umbenennen.

005: Werte aus enums können nicht in Preprocessor-Conditionals verwendet werden !!!
-----------------------------------------------------------------------------------
- see gnu CPP manual 4.2.2.: "The preprocessor does not know anything about types 
  in the language. Therefore, sizeof operators are not recognized in `#if', and 
  neither are enum constants. They will be taken as identifiers which are not macros, 
  and replaced by zero. In the case of sizeof, this is likely to cause the expression 
  to be invalid."

006: Prozessorgeschwindigkeit 8MHz unzureichend bei Thumb-Debug
-----------------------------------------------------------------------------------
- Im debug modus ist der Overhead zu groß, so das die Zeitsynchronisation mit dem
  Master zu langsam ist und immer ins timeout läuft.
  Abhilfe: Mit Thumb release kompilieren und/oder mit >= 24 MHz rennen lassen  
  

007: Recommended GPIO Clock speeds
-----------------------------------------------------------------------------------
GPIO_SPEED_FREQ_LOW        range up to 5 MHz
GPIO_SPEED_FREQ_MEDIUM     range  5 MHz to 25 MHz
GPIO_SPEED_FREQ_HIGH       range 25 MHz to 50 MHz
GPIO_SPEED_FREQ_VERY_HIGH  range 50 MHz to 80 MHz


