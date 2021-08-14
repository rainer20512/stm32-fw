#include "config/config.h"
#include <stdarg.h>
#include <stdio.h>

#include "log.h"
#include "rtc.h"

#if LOGTO_FATFS > 0
    #include "logfile.h"
#endif

#if LOGTO_CONSOLE > 0
    #include "debug_outbuf.h"
#endif

#define LOGLINE_LEN     256      /* max ength of one line of debug/log output */
static char logline[LOGLINE_LEN];

static const char severity_char[] = "FEWI";  /* Severity designators for Fatal, Error, Warning and Info */

static void _logto ( uint32_t logdest, uint32_t loglvl, char *buf, uint32_t buflen, uint32_t bWithCRLF )
{
    #if LOGTO_FATFS > 0
        if ( logdest & LOGDEST_FATFS ) {
            if ( loglvl <= fatfs_debuglevel || loglvl == LOGLEVEL_ALWAYS ) {
                LogFile_Write(buf, buflen);
                if ( bWithCRLF ) LogFile_CRLF();
            }
        }
    #endif
    #if LOGTO_CONSOLE > 0
        if ( logdest & LOGDEST_CONSOLE ) {
            if ( loglvl <= console_debuglevel || loglvl == LOGLEVEL_ALWAYS ) {
                Console_Write(buf, buflen);
                if ( bWithCRLF ) Console_CRLF();
            }
        }
    #endif
}

void _LOG( uint32_t loglvl,  uint32_t logts,  uint32_t logdest, const char* format, ... ) 
{
    char *start;
    uint32_t prefixlen;
    uint32_t reslen;

    /* Only generate output, if message severity is at or below global loglevel value */
    if ( loglvl != LOGLEVEL_ALWAYS && loglvl > console_debuglevel &&  loglvl > fatfs_debuglevel ) return;

    /* First, generate the timestamp, if desired */
    switch(logts) {
        case LOGTS_T:
            start = RTC_GetStrTimeMillis(&reslen);
            break;
        case LOGTS_U:
             start = RTC_GetStrSecMicros(&reslen);
            break;
        default:
            start = NULL;
    }

    if ( start ) _logto( logdest, loglvl, start, reslen, false );

    /* then, generate the severity letter */
    if ( loglvl >= LOGLEVEL_FATAL && loglvl <= LOGLEVEL_INFO ) {
        *logline     = severity_char[loglvl-LOGLEVEL_FATAL];
        *(logline+1) = '-';
        start     = logline+2;
        prefixlen = 2;
    } else {
        start     = logline;
        prefixlen = 0;
    }

    /* and append the log message */
    va_list args;
    va_start(args, format);
    reslen = vsnprintf(start, LOGLINE_LEN-prefixlen, format, args);
    va_end( args );

    _logto( logdest, loglvl, logline, reslen+prefixlen, true );
}

void Log_SetDebugLevels(uint32_t config_value)
{
    console_debuglevel = config_value & 0x0f;
    fatfs_debuglevel   = config_value >> 4;
}
