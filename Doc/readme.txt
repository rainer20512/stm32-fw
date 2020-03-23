001:   RAM mit speziellem "unused"-pattern initialisieren
---------------------------------------------------------
Modify ...\Rowley Associates Limited\CrossWorks for ARM\v4\packages\targets\STM32\STM32_Startup.s at "reset_handler" to:

reset_handler:

// RHB chgd: Preset all RAM with an uniform pattern
#ifdef DO_RAM_PRESET

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
  ldr r0, =___non_init_start__
  ldr r1, =__RAM_segment_end__

r02:
  cmp r0, r1
  beq r03
  str r2, [r0]
  adds r0, r0, #4
  b r02

r03:
#endif
// end of inserted code

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__RAM_segment_end__
  mov sp, r0
  bl SystemInit
#endif

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
  

temp

