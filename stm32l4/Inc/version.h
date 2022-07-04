/******************************************************************************
 * Set the version number
 *****************************************************************************/

#ifndef __VERSION_H_
#define __VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config/config.h"
#include <stddef.h>

#if defined(NOEXTENSION) 
    #define APP_NAME "NoExtension"
#elif defined(UNIVERSAL) 
    #define APP_NAME "Universal"
#elif defined(ENVIRONMENTAL) 
    #define APP_NAME "Environment"
#elif defined(TX18LISTENER)        
    #define APP_NAME "TxListener"
#else
    #define APP_NAME "Unknown App"
#endif

#if defined(STM32L476xx)
    #define MCU "STM32L476xx"
#elif defined(STM32L496xx)
    #define MCU "STM32L496xx"
#elif defined(STM32L4Sxxx)
    #define MCU "STM32L4Sxxx"
#elif defined(STM32L4Pxxx)
    #define MCU "STM32L4Pxxx"
#else
    #define MCU "Unknown"
#endif

#if defined(BL475IOT)
    #define BOARD   "BL475IOT"
#elif defined(DRAGONFLY476)
    #define BOARD   "Dragonfly476"
#elif defined(STM32L476NUCLEO)
    #define BOARD   "STM32L476 Nucleo"
#elif defined(STM32L4R9DISCOVERY)
    #define BOARD   "STM32L4R9 Discovery"
#elif defined(STM32L4S9ZXXREF)
    #define BOARD   "STM32L4S9ZXX Reference"
#elif defined(STM32L4P5BAREMETAL)
    #define BOARD   "STM32L4P5 bare metal"
#elif defined(STM32L476EVAL)
    #define BOARD   "STM32L476 Eval"
#elif defined(STM32L476BAREMETAL)
    #define BOARD   "STM32L476 bare metal"
#else
    #define BOARD   "Unknown Board"
#endif


/*-----------------------------------------------------------------------------
 * Don't change below
 *-----------------------------------------------------------------------------
 */

#define VERSION_NUMBER  "V" STR(MAJOR_VERSION) "." STR(MINOR_VERSION)

#define MCU_TYPE "for " MCU " family" 

#define BOARD_STRING "on board " BOARD

#define VERSION_STRING1  APP_NAME " " VERSION_NUMBER " " MCU_TYPE " " BOARD_STRING

#define VERSION_STRING2  "Built " __DATE__ " " __TIME__ 

#define VERSION_STRING   VERSION_STRING1 "\r\n" VERSION_STRING2 

extern const char VersionString[];
void        Dump_VersionInfo(void);
uint32_t    GetConfigNumLines(void);
char        *GetConfigLine(char *retbuf, size_t maxlen, uint32_t idx, bool bAppendCrlf);


#ifdef __cplusplus
}
#endif

#endif // __VERSION_H
