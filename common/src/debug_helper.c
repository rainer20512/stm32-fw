/**
  ******************************************************************************
  * @file    debug_helper.c
  * @author  Rainer
  * @brief   Miscellaneous functions to format debug output
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/** @addtogroup DEBUG_UTIL
  * @{
  */

#include "config/config.h"
#include "debug_helper.h"

#if  DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0
  #include "rtc.h"
#endif

#if DEBUG_FEATURES > 0 || DEBUG_PROFILING  > 0

#include <stdarg.h>
#include <stdio.h>
#include <string.h>


#if ( DEBUG_PROFILING > 0 && DEBUG_MODE > 0 ) 
    #include "system/profiling.h"
#endif

#define MAX_INDENTLEN 12

static int myIndent=0;
static int myDesiredLen=0;


/*
 *************************************************************
 * Helper functions to dump controller settings
 *************************************************************
 */
int DBG_setIndentRel ( int delta ) {
  int ret = myIndent;
  myIndent += delta;
  return ret;
}

int DBG_setIndentAbs ( int absval ) {
  int ret = myIndent;
  myIndent = absval;
  return ret;
}

int DBG_setPadLen ( int len ) {
  int ret = myDesiredLen;
  myDesiredLen = len;
  return ret; 
}


static void DBG_do_indent ( void )
{
  for ( int i=0; i < myIndent; i++ )
    DEBUG_PUTC(' ');
}

void DBG_strpadright(const char *text, uint32_t desiredlen, char padchar )
{
  int padlen = desiredlen - strlen(text);
  DEBUG_PRINTF("%s", text);
  if ( padlen > 0 ) {
    for ( int i=0; i < padlen; i++ )
      DEBUG_PUTC(padchar);
  }
}

void DBG_printf_indent(const char *format, ...)
{
  DBG_do_indent();
  va_list args;
  va_start(args, format);
  DEBUG_VPRINTF(format, args);
  va_end(args);

}

void DBG_dump_textvalue(const char *text, const char *content ) 
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %s\n", content);
}


void DBG_dump_number (const char *text, uint32_t num )
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %u\n", num );
}

void DBG_dump_uint32_hex (const char *text, uint32_t num )
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" 0x%08x\n", num );
}

void DBG_dump_uint32_kb (const char *text, uint32_t num )
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %dkB\n", num >> 10 );
}

void DBG_dump_section(const char *text, uint32_t start, uint32_t end, bool bIn_KByte )
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  if ( bIn_KByte )
    DEBUG_PRINTF(" 0x%08x...0x%08x, Length=%dk Byte\n",start, end, (end - start)>>10); 
  else
    DEBUG_PRINTF(" 0x%08x...0x%08x, Length=%d Byte\n",start, end, end - start); 
}
void DBG_dump_section_useage (const char *text, uint32_t used, uint32_t total ) 
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %d Bytes (%d%%)\n", used, (used*100+(total>>1))/total );
}

void DBG_dump_stack_useage( uint32_t act_sp, uint32_t max_useage, uint32_t start, uint32_t end)
{
  uint32_t total = end - start;

  DBG_do_indent();
  DBG_strpadright("Actual SP", myDesiredLen, '.');
  DEBUG_PRINTF(" 0x%08x\n", act_sp );
  DBG_do_indent();
  DBG_strpadright("Max Stack Depth", myDesiredLen, '.');
  DEBUG_PRINTF(" %d Bytes (%d%%)\n", max_useage, (max_useage*100+(total>>1))/total );
}

 
void DBG_dump_bitvalue(const char *text, uint32_t regval, uint32_t bitval ) 
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %s\n", regval & bitval ? "Yes" : "No");
}

void DBG_dump_onoffvalue(const char *text, uint32_t regval, uint32_t bitval ) 
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %s\n", regval & bitval ? "On" : "Off");
}

void DBG_dump_setresetvalue(const char *text, uint32_t regval, uint32_t bitval ) 
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %s\n", regval & bitval ? "Set" : "Reset");
}

/*
 * Dump a power-of-2 value
 * @param  exp - exponent of 2
 *
 */
void DBG_dump_po2 (const char *text, uint32_t exp )
{
  DBG_do_indent();
  DBG_strpadright(text, myDesiredLen, '.');
  DEBUG_PRINTF(" %d\n", 1 << exp );
}


#endif // #if DEBUG_FEATURES > 0


void CRLF(void) {
	 DEBUG_PUTC('\r'); DEBUG_PUTC('\n');
}


void print_hexXX(uint8_t i) 
{
  uint8_t x = i>>4;
  if (x>=10) {
    DEBUG_PUTC(x+'a'-10);	
  } else {
    DEBUG_PUTC(x+'0');
  }	
  x = i & 0xf;
  if (x>=10) {
    DEBUG_PUTC(x+'a'-10);	
  } else {
    DEBUG_PUTC(x+'0');
  }	
}


#if DEBUG_DUMP_RFM > 0 || DEBUG_MODE > 0 || DEBUG_MEM_CHECK > 0
  /*!
   *******************************************************************************
   *  \brief helper function print 4 digit dec number
   *
   *  \note only unsigned numbers
   ******************************************************************************/

    void print_hexXXXX(uint16_t i) {
        print_hexXX(i>>8);
        print_hexXX(i&0xff);
    }

#endif

#if ( DEBUG_FEATURES > 0 || DEBUG_RFM_STATUS > 0 || DEBUG_DUMP_RFM > 0 || DEBUG_DUMP_KEYS > 0 || DEBUG_CHECK_SEC > 0 || DEBUG_PRINT_RTC_TICKS > 0 || DEBUG_MEM_CHECK > 0 || DEBUG_SLEEP_MODE > 0 || DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0 ) 
        #define MAX_NUMBUF 33
        static char numbuf[MAX_NUMBUF];
        static char *numptr;
        static bool _print_number_int ( uint16_t num, uint8_t digits, bool bLeadingZeros)
        {
                bool ret=false;
                if ( --digits && (num || bLeadingZeros) ) 
                        ret = _print_number_int( num/10, digits, bLeadingZeros);
	
                uint8_t digit= num % 10;
                if ( bLeadingZeros || ret || digit ) {
                        // uart_putc( digit+'0');
                        *numptr++ = '0'+digit;
                        return true;
                } else {
                        return false;
                }
        }


        static void print_str ( char *str ) {
            while ( *str ) {
               DEBUG_PUTC(*str);
               str++;
            }
        }

        void print_dec_number ( uint16_t num, uint8_t digits, bool bLeadingZeros)
        {
                numptr = numbuf;
                if  ( !bLeadingZeros && !num )
                    *numptr++ = '0';
                else
                    (void)_print_number_int ( num, digits, bLeadingZeros);
                *numptr='\0';
                print_str(numbuf);
        }

        char *my_itoa ( uint16_t num, char *retbuf, uint8_t digits, bool bLeadingZeros)
        {
                numptr = retbuf;
                if  ( !bLeadingZeros && !num )
                    *numptr++ = '0';
                else
                    (void)_print_number_int ( num, digits, bLeadingZeros);

                // Terminate string and return pointer to terminating \0
                *numptr='\0';
                return numptr;
        }


	/*!
	 *******************************************************************************
	 *  \brief helper function print 2 digit dec number
	 *
	 *  \note Signed number ( i.e. -128 .. +127 )
	 ******************************************************************************/
	void print_decSXX(int8_t i) {
		if ( i < 0 ) {
			DEBUG_PUTC('-');
			print_decXX( (uint8_t)(i*-1) );
		} else {
			print_decXX( (uint8_t)i );
		}
	}

#endif


#if ( DEBUG_DUMP_RFM > 0 || DEBUG_DUMP_KEYS > 0 )
/*!
 *******************************************************************************
 *  \brief dump data from *d length len
 *
 *  \note
 ******************************************************************************/
static uint16_t seq=0;
void COM_dump_packet(uint8_t *d, uint8_t len, uint8_t mac_ok) {
    /*
    print_dec0XX((task & TASK_RTC)?RTC_GetSecond()+1:RTC_GetSecond());
	DEBUG_PUTC('.');
    print_hexXX(RTC_s256());
    */
    if (mac_ok && (len>=(2+4))) {
        DEBUG_PRINTF(" PKT");
        len-=4; // mac is correct and not needed
    } else {
        DEBUG_PRINTF(" ERR");
    }
    print_hexXXXX(seq++);
    DEBUG_PUTC(':');
    uint8_t dots=0;
    if (len > 24) {
      len=24; // debug output limitation
      dots=1;
    }
    while ((len--)>0) {
        DEBUG_PUTC(' ');
        print_hexXX(*(d++));
    }
    if (dots) {
      DEBUG_PRINTF("...");
    }
    DEBUG_PUTC('\n');
}
#endif 

#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
#include "rfm/rfm_ringbuffer.h"
void dump_rfm_ringbuf(void)
{
  rfm_xbuf_dataT type, type2;
  uint16_t data, data2; 
   
  while ( rfm_xbuf_has_data() ) {
    rfm_xbuf_get(&type, &data);
    DEBUG_PRINTF("R:");
    switch ( type ) {
        case xbuf_uint8_out:
            DEBUG_PRINTF("U8 out %02x", (uint8_t)data);
            break;
        case xbuf_uint8_in:
            DEBUG_PRINTF("U8 in %02x", (uint8_t)data);
            break;
        case xbuf_uint16_out:
            DEBUG_PRINTF("U16 out %04x", data);
            break;
        case xbuf_uint16_in:
            DEBUG_PRINTF("U16 in %04x", data);
            break;
        case xbuf_uint24_out:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint24_out ) 
                DEBUG_PRINTF("U24 out: Wrong consecutive type");
            else
                DEBUG_PRINTF("U24 out %06x", ((uint32_t)data)<<16 | data2 );
            break;
        case xbuf_uint24_in:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint24_in ) 
                DEBUG_PRINTF("U24 in: Wrong consecutive type");
            else
                DEBUG_PRINTF("U24 in %06x", ((uint32_t)data)<<16 | data2 );
            break;
        case xbuf_uint88_out:
            DEBUG_PRINTF("U8 U8 out %02x %02x", (uint8_t)(data>>8), (uint8_t)data);
            break;
        case xbuf_uint88_in:
            DEBUG_PRINTF("U8 U8 in %02x %02x", (uint8_t)(data>>8), (uint8_t)data);
            break;
        case xbuf_uint816_out:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint816_out ) 
                DEBUG_PRINTF("U8 U16 out: Wrong consecutive type");
            else
                DEBUG_PRINTF("U8 U16 out %02x %04x", (uint8_t)data, data2 );
            break;
        case xbuf_uint816_in:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint816_in ) 
                DEBUG_PRINTF("U8 U16 out: Wrong consecutive type");
            else
                DEBUG_PRINTF("U8 U16 in %02x %04x", (uint8_t)data, data2 );
            break;
        case xbuf_uint824_out:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint824_out ) 
                DEBUG_PRINTF("U8 U24 out: Wrong consecutive type");
            else
                DEBUG_PRINTF("U8 U24 out %02x %04x", (uint8_t)(data>>8), (uint32_t)((data&0xff)<<16) | data2 );
            break;
        case xbuf_uint824_in:
            rfm_xbuf_get(&type2, &data2);
            if ( type2 != xbuf_uint824_in ) 
                DEBUG_PRINTF("U8 U24 in: Wrong consecutive type");
            else
                DEBUG_PRINTF("U8 U24 in %02x %04x", (uint8_t)(data>>8), (uint32_t)((data&0xff)<<16) | data2 );
            break;
        default:          
                DEBUG_PRINTF("Uncoded type %d", type );
    }
    CRLF();	 
  }
  DEBUG_PUTS("Done\r\n");
}
#endif

#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0
    void COM_print_time(uint8_t c, uint32_t bWithCRLF) {
        print_dec0XX(RTC_GetSecond());
        DEBUG_PUTC('.');
        print_hexXX(RTC_GetS256());
        DEBUG_PUTC('-');
        DEBUG_PUTC(c);
        if (bWithCRLF) 
          DEBUG_PUTS("");
        else
          DEBUG_PUTC(' ');
   }
#endif

#if DEBUG_MODE > 0 && DEBUG_PULSES > 0
    void COM_print_time_short(uint8_t c) {
        print_hexXX(RTC_GetS256());
        DEBUG_PUTC(c);
    }
#endif

#if DEBUG_SLEEP_STOP > 0

  #define SLEEPBUFSIZE  512
  static char buf[SLEEPBUFSIZE];
  static uint32_t bufidx=0;

  /* Increment so, that buffer index will not point to location behind buffer */
  #define INC_BUFIDX(delta)   ( bufidx = ( bufidx+delta < SLEEPBUFSIZE ?  bufidx+delta : bufidx ) )
  /* Store element and increment idx, if possible */
  #define BUF_STORE(c)        do { buf[bufidx]=c; if ( bufidx < SLEEPBUFSIZE-1 ) bufidx++; } while(0)


void store_init(void)
{ 
   bufidx=0;
}

  void store_decXX(uint8_t i) 
  {
    uint8_t x = i/10;
    BUF_STORE(x+'0');

    x = i % 10;
   BUF_STORE(x+'0');
  }

  void store_hexXX(uint8_t i) 
  {
    uint8_t x = i>>4;
    if (x>=10) {
      BUF_STORE(x+'a'-10);	
    } else {
      BUF_STORE(x+'0');
    }	
    x = i & 0xf;
    if (x>=10) {
      BUF_STORE(x+'a'-10);	
    } else {
      BUF_STORE(x+'0');
    }	
  }

  void store_hexXXXX(uint16_t i) 
  {
    store_hexXX((uint8_t)(i>>8));
    store_hexXX((uint8_t)(i&0xff));
  }
   
  void store_str(char *str)
  {
    while ( *str ) {
      BUF_STORE(*(str++));
    }
  }
    
  void store_chr(char c)
  {
     BUF_STORE(c);
  }

  void store_time(char *str ) 
  {
    // store_decXX((uint8_t)RTC_GetSecond());
    #if DEBUG_PROFILING > 0 && DEBUG_MODE > 0
        store_str(ProfilerGetTS());
    #else
        BUF_STORE('.');
        store_hexXX((uint8_t)RTC_GetS256());
    #endif
    BUF_STORE('-');
    store_str(str);
   }

   void store_dump(void)
   {
      BUF_STORE('\0');
      DEBUG_PUTS(buf);
      bufidx=0;
   }

#endif /* DEBUG_SLEEP_STOP > 0 */


/**
  * @}
  */
