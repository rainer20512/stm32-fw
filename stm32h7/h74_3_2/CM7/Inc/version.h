/******************************************************************************
 * Set the version number
 *****************************************************************************/

#ifndef __VERSION_H_
#define __VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config/config.h"

#define MAJOR_VERSION   1
#define MINOR_VERSION   04

#if defined(UNIVERSAL) 
    #define APP_NAME "Universal"
#else
    #define APP_NAME "Unknown App"
#endif

#if defined(STM32H742xx)
    #define MCU "STM32H742xx"
#elif defined(STM32H743xx)
    #define MCU "STM32H743xx"
#else
    #define MCU "Unknown"
#endif

#if defined(STM32H742REF)
    #define BOARD   "STM32H7_100_Ref"
#elif defined(STM32H743EVAL2)
    #define BOARD   "STM32H743_EVAL2"
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

#define VERSION_STRING2  __DATE__ " " __TIME__ 

#define VERSION_STRING1  APP_NAME " " VERSION_NUMBER 

#define VERSION_STRING   VERSION_STRING1 " " MCU_TYPE " " BOARD_STRING "\nBuilt " VERSION_STRING2 

extern const char VersionString[];
#ifdef __cplusplus
}
#endif

#endif // __VERSION_H
