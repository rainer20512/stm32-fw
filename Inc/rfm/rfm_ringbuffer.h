
#include <stdint.h>

#include "debug.h"

#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )

/** Size of the circular receive buffer, must be power of 2 */
#ifndef RFM_X_BUFFER_SIZE
  #define RFM_X_BUFFER_SIZE 64
#endif

/** Size of the circular receive buffer, must be power of 2 */
#if (RFM_X_BUFFER_SIZE) & ( RFM_X_BUFFER_SIZE-1) != 0
    #error "RFM_X_BUFFER_SIZE not a power of 2"
#endif

typedef enum rfm_xbuf_enum {
  xbuf_uint8_out   = 0,
  xbuf_uint8_in    = 1,
  xbuf_uint16_out  = 2,
  xbuf_uint16_in   = 3,
  xbuf_uint24_out  = 4,
  xbuf_uint24_in   = 5,
  xbuf_uint88_out  = 6,    /* two uint8 tweeked uinto uint16 */
  xbuf_uint88_in   = 7, 
  xbuf_uint816_out = 8,    /* uint8 followed by uint16  in two consecutive entries*/
  xbuf_uint816_in =  9,
  xbuf_uint824_out = 10,    /* uint8 followed by uint24  in two consecutive entries*/
  xbuf_uint824_in  = 11,
} rfm_xbuf_dataT;

/* function prototypes */


unsigned char rfm_xbuf_put(rfm_xbuf_dataT type, uint16_t data);
uint16_t rfm_xbuf_get(rfm_xbuf_dataT *type, uint16_t *data);
unsigned char rfm_xbuf_has_data(void);

#endif
