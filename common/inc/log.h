/*
 ******************************************************************************
 * @file    log.h 
 * @author  Rainer
 * @brief   log and debug info to serveral destinations
 *
 ******************************************************************************
 */
#if ! defined(_LOG_H_)
#define _LOG_H_

#include "config/config.h"

/*
 * Define different levels of logging detail depth 
 * the higher the loglevel, the more debug/log info is generated/displayed
 */

#define LOGLEVEL_NOTHING   0                    /* Don't debug/log anything at all   */
#define LOGLEVEL_ALWAYS    1                    /* Log always, independent from console_debuglevel       */
#define LOGLEVEL_FATAL     2                    /* debug/log only severe errors      */
#define LOGLEVEL_ERROR     3                    /* debug/log all kinds errors        */
#define LOGLEVEL_WARN      4                    /* debug/log also warnings           */ 
#define LOGLEVEL_INFO      5                    /* debug/log all kind of useful info */
#define LOGLEVEL_VERBOSE   6                    /* debug/log even more               */
#define LOGLEVEL_PALAVER   7                    /* debug/log all you can imagine     */

#define LOGTS_NONE         0                    /* No timestamp at start of line     */
#define LOGTS_T            1                    /* hh:mm:ss.iii at start of line     */
#define LOGTS_U            2                    /* ss.iii.uuu   at start of line     */

/* Different types of log destinations, can be or'ed to log to many destinations     */
#define LOGDEST_CONSOLE    (1<<0)               /* Log to console                    */
#define LOGDEST_FATFS      (1<<8)               /* Log to FAT filesystem             */
                                                /* Log to all destinations           */
#define LOGDEST_ALL        ( LOGDEST_CONSOLE | LOGDEST_FATFS )

/*
 * Logging function types:
 * LOG_<Severity>    : Log without timestamp, only with X- prefix
 * LOGT_<Severity>   : Log with timestamp in format hh:mm:ss.iiiX- prefix  
 * LOGU_<Severity>   : Log with timestamp in format ss.iii.uuuX- prefix
 * 
 * where
 * X-Severity Letter: F=fatal E=error W=Warning I=Info <no letter> = verbose
 * hh,mm,ss hour, minute and second resp
 * iii milliseconds
 * uuu microseconds
 */

#define LOG_ALWAYS(...)     _LOG(LOGLEVEL_ALWAYS,  LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
#define LOGT_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
#define LOGU_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )

#if STATIC_LOGLIMIT >= LOGLEVEL_FATAL
    #define LOG_FATAL(...)      _LOG(LOGLEVEL_FATAL,   LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_FATAL(...)
    #define LOGT_FATAL(...)
    #define LOGU_FATAL(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_ERROR
    #define LOG_ERROR(...)      _LOG(LOGLEVEL_ERROR,   LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_ERROR(...)
    #define LOGT_ERROR(...)
    #define LOGU_ERROR(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_WARN
    #define LOG_WARN(...)       _LOG(LOGLEVEL_WARN,    LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_WARN(...)
    #define LOGT_WARN(...)
    #define LOGU_WARN(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_INFO
    #define LOG_INFO(...)       _LOG(LOGLEVEL_INFO,    LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_INFO(...)
    #define LOGT_INFO(...)
    #define LOGU_INFO(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_VERBOSE
    #define LOG_VERBOSE(...)    _LOG(LOGLEVEL_VERBOSE, LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_VERBOSE(...)
    #define LOGT_VERBOSE(...)
    #define LOGU_VERBOSE(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_PALAVER
    #define LOG_PALAVER(...)    _LOG(LOGLEVEL_PALAVER, LOGTS_NONE, LOGDEST_ALL, __VA_ARGS__ )
    #define LOGT_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_T,    LOGDEST_ALL, __VA_ARGS__ )
    #define LOGU_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_U,    LOGDEST_ALL, __VA_ARGS__ )
#else
    #define LOG_PALAVER(...)
    #define LOGT_PALAVER(...)
    #define LOGU_PALAVER(...)
#endif

/* ------------------------------ Log console only ----------------------------------------*/
#define DBG_ALWAYS(...)     _LOG(LOGLEVEL_ALWAYS,  LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
#define DBGT_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
#define DBGU_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )

#if STATIC_LOGLIMIT >= LOGLEVEL_FATAL
    #define DBG_FATAL(...)      _LOG(LOGLEVEL_FATAL,   LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_FATAL(...)
    #define DBGT_FATAL(...)
    #define DBGU_FATAL(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_ERROR
    #define DBG_ERROR(...)      _LOG(LOGLEVEL_ERROR,   LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_ERROR(...)
    #define DBGT_ERROR(...)
    #define DBGU_ERROR(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_WARN
    #define DBG_WARN(...)       _LOG(LOGLEVEL_WARN,    LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_WARN(...)
    #define DBGT_WARN(...)
    #define DBGU_WARN(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_INFO
    #define DBG_INFO(...)       _LOG(LOGLEVEL_INFO,    LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_INFO(...)
    #define DBGT_INFO(...)
    #define DBGU_INFO(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_VERBOSE
    #define DBG_VERBOSE(...)    _LOG(LOGLEVEL_VERBOSE, LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_VERBOSE(...)
    #define DBGT_VERBOSE(...)
    #define DBGU_VERBOSE(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_PALAVER
    #define DBG_PALAVER(...)    _LOG(LOGLEVEL_PALAVER, LOGTS_NONE, LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGT_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_T,    LOGDEST_CONSOLE, __VA_ARGS__ )
    #define DBGU_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_U,    LOGDEST_CONSOLE, __VA_ARGS__ )
#else
    #define DBG_PALAVER(...)
    #define DBGT_PALAVER(...)
    #define DBGU_PALAVER(...)
#endif

/* ------------------------------ Log console only ----------------------------------------*/
#define FLOG_ALWAYS(...)     _LOG(LOGLEVEL_ALWAYS,  LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
#define FLOGT_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
#define FLOGU_ALWAYS(...)    _LOG(LOGLEVEL_ALWAYS,  LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )

#if STATIC_LOGLIMIT >= LOGLEVEL_FATAL
    #define FLOG_FATAL(...)      _LOG(LOGLEVEL_FATAL,   LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_FATAL(...)     _LOG(LOGLEVEL_FATAL,   LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_FATAL(...)
    #define FLOGT_FATAL(...)
    #define FLOGU_FATAL(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_ERROR
    #define FLOG_ERROR(...)      _LOG(LOGLEVEL_ERROR,   LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_ERROR(...)     _LOG(LOGLEVEL_ERROR,   LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_ERROR(...)
    #define FLOGT_ERROR(...)
    #define FLOGU_ERROR(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_WARN
    #define FLOG_WARN(...)       _LOG(LOGLEVEL_WARN,    LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_WARN(...)      _LOG(LOGLEVEL_WARN,    LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_WARN(...)
    #define FLOGT_WARN(...)
    #define FLOGU_WARN(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_INFO
    #define FLOG_INFO(...)       _LOG(LOGLEVEL_INFO,    LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_INFO(...)      _LOG(LOGLEVEL_INFO,    LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_INFO(...)
    #define FLOGT_INFO(...)
    #define FLOGU_INFO(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_VERBOSE
    #define FLOG_VERBOSE(...)    _LOG(LOGLEVEL_VERBOSE, LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_VERBOSE(...)   _LOG(LOGLEVEL_VERBOSE, LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_VERBOSE(...)
    #define FLOGT_VERBOSE(...)
    #define FLOGU_VERBOSE(...)
#endif

#if STATIC_LOGLIMIT >= LOGLEVEL_PALAVER
    #define FLOG_PALAVER(...)    _LOG(LOGLEVEL_PALAVER, LOGTS_NONE, LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGT_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_T,    LOGDEST_FATFS, __VA_ARGS__ )
    #define FLOGU_PALAVER(...)   _LOG(LOGLEVEL_PALAVER, LOGTS_U,    LOGDEST_FATFS, __VA_ARGS__ )
#else
    #define FLOG_PALAVER(...)
    #define FLOGT_PALAVER(...)
    #define FLOGU_PALAVER(...)
#endif

void _LOG( uint32_t loglvl,  uint32_t logts,  uint32_t logdest, const char* format, ... );
void Log_SetDebugLevels(uint32_t config_value);


#endif /* if ! defined(_LOG_H_) */