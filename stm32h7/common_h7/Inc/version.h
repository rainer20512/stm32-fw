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

#if defined(UNIVERSAL) 
    #define APP_NAME "Universal"
#else
    #define APP_NAME "Unknown App"
#endif

#if defined(STM32H742xx)
    #define MCU "STM32H742xx"
#elif defined(STM32H743xx)
    #define MCU "STM32H743xx"
#elif defined(STM32H723xx) || defined(STM32H733xx) || defined(STM32H725xx) || defined(STM32H735xx) || defined(STM32H730xx)
    #if   defined(STM32H723xx) 
        #define MCU "STM32H723xx"
    #elif defined(STM32H733xx) 
        #define MCU "STM32H733xx"
    #elif defined(STM32H725xx) 
        #define MCU "STM32H725xx"
    #elif defined(STM32H735xx) 
        #define MCU "STM32H735xx"
    #elif defined(STM32H730xx) 
        #define MCU "STM32H730xx"
    #else
        #define MCU "Unknown"
    #endif
#elif defined(STM32H745xx) || defined(STM32H747xx)
    #if defined(STM32H745xx) 
        #define MCU "STM32H745xx"
    #else
        #define MCU "STM32H747xx"
    #endif

    #if defined(CORE_CM4)
        #define MCORE "Core CM4"
    #else
        #define MCORE "Core CM7"
    #endif

#else
    #define MCU "Unknown"
#endif

#if defined(STM32H742REF)
    #define BOARD   "STM32H7_100_Ref"
#elif defined(STM32H743EVAL2)
    #define BOARD   "STM32H743_EVAL2"
#elif defined(STM32H745NUCLEO)
    #define BOARD   "STM32H745NUCLEO"
#elif defined(STM32H7_DEVEBOX)
    #define BOARD   "DevEBox STM32H7XX"
#elif defined(STM32H7_OPENMV)
    #define BOARD   "OpenMV STM32H743IIT"
#else
    #define BOARD   "Unknown Board"
#endif

/*-----------------------------------------------------------------------------
 * Don't change below
 *-----------------------------------------------------------------------------
 */

#define VERSION_NUMBER  "V" STR(MAJOR_VERSION) "." STR(MINOR_VERSION)

#if defined(MCORE)
    #define MCU_TYPE "for " MCU " family " MCORE
#else  
    #define MCU_TYPE "for " MCU " family" 
#endif

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
