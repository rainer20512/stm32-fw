/**
  ******************************************************************************
  * @file    debug_util.h
  * @author  Rainer
  * @brief   Miscellaneous functions for debugging
  ******************************************************************************
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_OUTBUF_H
#define __DEBUG_OUTBUF_H

#if DEBUG_DEBUGIO > 0
  #include <debugio.h>
  #include "debug_stdin.h"
#endif

#include <stdio.h>

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if DEBUG_FEATURES > 0
  #if DEBUG_DEBUGIO > 0
    extern LinBuffT dbg_inbuf;
    #define DEBUG_PRINTF(...)     debug_printf (__VA_ARGS__ )
    #define DEBUG_PUTS(a)         debug_puts(a)
    #define DEBUG_PUTC(a)         debug_putchar(a)
    #define DEBUG_VPRINTF(...)    debug_vprintf (__VA_ARGS__ )
    #define DEBUG_TX_ACTIVE()     1
    #define DEBUG_RXBUF_GET()     ( &stdin_line )
  #else
    #define DEBUG_PRINTF(...)     printf (__VA_ARGS__ )
    #define DEBUG_PUTS(a)         puts(a)
    #define DEBUG_PUTC(a)         putchar(a)
    #define DEBUG_VPRINTF(...)    vprintf (__VA_ARGS__ )
    #define DEBUG_RXBUF_GET()     ( DebugUart->in )
  #endif
#else
  #define DEBUG_PRINTF(...)   
  #define DEBUG_PUTS(a)     
  #define DEBUG_PUTC(a)       
  #define DEBUG_VPRINTF(...)  
  #define DEBUG_TX_ACTIVE()       1
#endif

#if DEBUG_FEATURES
  void DBG_dump_clocksetting(void);
  void DBG_dump_powersetting(void);
  void DBG_dump_peripheralclocksetting(void);
  void DBG_dump_peripheralclocksetting_insleepmode(void);
  void DBG_dump_peripheralclockconfig(void);
  #if DEBUG_SLEEP_STOP > 0
    void DebugOutbufGetPtrs(void);
  #endif
#endif

#if LOGTO_CONSOLE > 0
    uint8_t Console_Write       (const char *data, uint32_t len );
    uint8_t Console_Write_CRLF  (const char *data, uint32_t len);
    void    Console_CRLF        (void);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_OUTBUF_H */
