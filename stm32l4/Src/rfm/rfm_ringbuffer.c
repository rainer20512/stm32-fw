#include "rfm/rfm_ringbuffer.h"

#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )

/* size of RX/TX buffers */
#define RFM_X_BUFFER_MASK ( RFM_X_BUFFER_SIZE - 1)

#if ( RFM_X_BUFFER_SIZE & RFM_X_BUFFER_MASK )
  #error RFM Ringbuffer size is not a power of 2
#endif

static volatile uint16_t RFM_XBufData[RFM_X_BUFFER_SIZE];
static volatile uint16_t RFM_XBufType[RFM_X_BUFFER_SIZE];
static volatile unsigned char RFM_XHead=0;
static volatile unsigned char RFM_XTail=0;


inline unsigned char rfm_xbuf_has_data() { return RFM_XHead != RFM_XTail; }


unsigned char rfm_xbuf_put(rfm_xbuf_dataT type, uint16_t data)
{
    unsigned char tmphead;
    
    tmphead  = (RFM_XHead + 1) & RFM_X_BUFFER_MASK;
    
    if ( tmphead == RFM_XTail ){
        /* Buffer full */
        return 0;
    } else {
        RFM_XBufType[tmphead] = type;
        RFM_XBufData[tmphead] = data;
    	RFM_XHead = tmphead;
        return 1;
    }
}

uint16_t rfm_xbuf_get(rfm_xbuf_dataT *type, uint16_t *data)
{
    unsigned char tmptail;

    if ( RFM_XHead == RFM_XTail ) {
        return 0;   /* no data available */
    }
    
    /* calculate /store buffer index */
    tmptail = (RFM_XTail + 1) & RFM_X_BUFFER_MASK;
    RFM_XTail = tmptail; 
    
    /* get data from receive buffer */
    *type = RFM_XBufType[tmptail];
    *data = RFM_XBufData[tmptail];
    return 1;
}

#endif
