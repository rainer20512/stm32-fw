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

/*-----------------------------------------------------------------------------
 * Don't change below
 *-----------------------------------------------------------------------------
 */

#define VERSION_NUMBER  "V" STR(MAJOR_VERSION) "." STR(MINOR_VERSION)

#define VERSION_STRING2  __DATE__ " " __TIME__ 

#define VERSION_STRING1  APP_NAME " " VERSION_NUMBER 

#define VERSION_STRING   VERSION_STRING1 " Built " VERSION_STRING2

#ifdef __cplusplus
}
#endif

#endif // __VERSION_H
