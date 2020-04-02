/**
  ******************************************************************************
  * @file    debug_helper.h
  * @author  Rainer
  * @brief   Miscellaneous functions for debugging
  ******************************************************************************
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_HELPER_H
#define __DEBUG_HELPER_H

#include "config/config.h"
#include "debug.h"
#include "dev/uart_dev.h"

#ifdef DEBUG_DEBUGIO
  #include <debugio.h>
#endif
#if DEBUG_PROFILING > 0
    #include "system/profiling.h"
#endif

#include <stdio.h>
#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if DEBUG_FEATURES > 0 && DEBUG_DEBUGIO == 0
  void DebugAssignUart(const HW_DeviceType *dev, void *arg);
  void DebugDeAssignUart(const HW_DeviceType *dev);
  void DebugOutputCompleteCB ( uint32_t size );
  extern UsartHandleT *DebugUart;
#endif

#if DEBUG_FEATURES > 0
  #if DEBUG_DEBUGIO > 0
    #define DEBUG_PRINTTS(...)  do { debug_printf("%s:",ProfilerGetTS()); debug_printf (__VA_ARGS__ ); } while (0)
    #define DEBUG_PRINTF(...)   debug_printf (__VA_ARGS__ )
    #define DEBUG_PUTS(a)       debug_puts(a)
    #define DEBUG_PUTC(a)       debug_putchar(a)
    #define DEBUG_VPRINTF(...)  debug_vprintf (__VA_ARGS__ )
  #else
    #define DEBUG_PRINTTS(...)  do { printf("%s:",ProfilerGetTS()); printf (__VA_ARGS__ ); } while (0)
    #define DEBUG_PRINTF(...)   printf (__VA_ARGS__ )
    #define DEBUG_PUTS(a)       puts(a)
    #define DEBUG_PUTC(a)       putchar(a)
    #define DEBUG_VPRINTF(...)  vprintf (__VA_ARGS__ )
  #endif
#else
  #define DEBUG_PRINTTS(...)   
  #define DEBUG_PRINTF(...)   
  #define DEBUG_PUTS(a)       
  #define DEBUG_PUTC(a)       
  #define DEBUG_VPRINTF(...)  
#endif

int DBG_setPadLen ( int len );
int  DBG_setIndentRel ( int delta );
int  DBG_setIndentAbs ( int absval );
void DBG_dump_number (const char *text, uint32_t num );
void DBG_dump_textvalue(const char *text, const char *content );
void DBG_printf_indent(const char *format, ...);
void DBG_strpadright(const char *text, uint32_t desiredlen, char padchar );
void DBG_strpadright2(const char *text1, const char *text2, uint32_t desiredlen, char padchar );
void DBG_dump_uint32_hex (const char *text, uint32_t num );
void DBG_dump_uint32_kb (const char *text, uint32_t num );
void DBG_dump_section(const char *text, uint32_t start, uint32_t end, bool bIn_KByte  );
void DBG_dump_section_useage (const char *text, uint32_t used, uint32_t total );
void DBG_dump_stack_useage( uint32_t act_sp, uint32_t max_useage, uint32_t start, uint32_t end);
void DBG_dump_bitvalue(const char *text, uint32_t regval, uint32_t bitval ); 
void DBG_dump_bitvalue2(const char *text1, const char *text2, uint32_t regval, uint32_t bitval ); 
void DBG_dump_onoffvalue(const char *text, uint32_t regval, uint32_t bitval );
void DBG_dump_setresetvalue(const char *text, uint32_t regval, uint32_t bitval );
void DBG_dump_po2 (const char *text, uint32_t exp );

void CRLF(void);
void print_hexXX(uint8_t i);
void print_hexXXXX(uint16_t i);
#if ( DEBUG_RFM_STATUS > 0 || DEBUG_DUMP_RFM > 0 || DEBUG_DUMP_KEYS > 0 || DEBUG_CHECK_SEC > 0 || DEBUG_PRINT_RTC_TICKS > 0 || DEBUG_MEM_CHECK > 0 || DEBUG_SLEEP_MODE > 0 || DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0 ) 
    void print_dec_number ( uint16_t num, uint8_t digits, bool bLeadingZeros);
    void print_decSXX(int8_t i);
    char *my_itoa ( uint16_t num, char *retbuf, uint8_t digits, bool bLeadingZeros);
    #define print_decXX(i) 	   (print_dec_number(i,3, false))
    #define print_dec0XX(i)	   (print_dec_number(i,2, true))
    #define print_decXXXXX(i)  (print_dec_number(i,5, false))
    #define print_dec0XXXX(i)  (print_dec_number(i,4, true))
    #define print_dec0XXXXX(i) (print_dec_number(i,5, true))
#endif

#if ( DEBUG_DUMP_RFM > 0 || DEBUG_DUMP_KEYS > 0 )
    void COM_dump_packet(uint8_t *d, uint8_t len, uint8_t mac_ok);
    // void COM_mac_ok(void);
#else 
    #define COM_dump_packet(d, len, mac_ok)
    // #define COM_mac_ok() ()
#endif


#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
	void dump_rfm_ringbuf(void);
#else
	#define dump_rfm_ringbuf()
#endif

#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
    void COM_print_time(uint8_t c, uint32_t bWithCRLF);
    char *COM_store_time(char *retbuf, uint8_t c );
#else
    #define COM_print_time(c,b)
    #define COM_store_time(a,b)  (a)
#endif

#if DEBUG_MODE > 0 && DEBUG_PULSES > 0
    void COM_print_time_short(uint8_t c);
#else 
    #define COM_print_time_short(c)
#endif

#if DEBUG_SLEEP_STOP > 0
    void store_time(char *str );
    void store_decXX (uint8_t i);
    void store_hexXX (uint8_t i); 
    void store_hexXXXX(uint16_t i); 
    void store_str(char *); 
    void store_chr(char); 
    void store_dump(void); 
#else
    #define store_time(a)
    #define store_decXX(a)
    #define store_hexXX(a)
    #define store_hexXXX(a)
    #define store_str(a) 
    #define store_chr(a) 
    #define store_dump()
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_HELPER_H */
