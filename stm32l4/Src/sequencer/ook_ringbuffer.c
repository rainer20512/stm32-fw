#include "config/config.h"

#if USE_PULSE_SEQUENCER > 0


#include "sequencer/ook_ringbuffer.h"

#include "debug_helper.h"

/* size of buffer must be power of 2 */
#define PULSE_BUFFER_MASK ( PULSE_BUFFER_SIZE - 1)

#if ( PULSE_BUFFER_SIZE & PULSE_BUFFER_MASK )
#error Pulse Ringbuffer size is not a power of 2
#endif

static volatile uint16_t PULSE_mBuf[PULSE_BUFFER_SIZE];
static volatile uint16_t PULSE_uBuf[PULSE_BUFFER_SIZE];
static volatile unsigned char PULSE_Head=0;
static volatile unsigned char PULSE_Tail=0;

void pulse_buf_reset() 
{ 
    PULSE_Head = PULSE_Tail = 0; 
}


unsigned char pulse_buf_has_data() { return PULSE_Head != PULSE_Tail; }

/******************************************************************************
 * Store a pules-record
 * mdata - signal high time
 * udata - total pulse time ( from start of high to next high
 *****************************************************************************/
unsigned char pulse_buf_put(uint16_t mdata, uint16_t udata)
{
//    DEBUG_PRINTF("H-time=%d,pulse=%d\n", mdata, udata);
    unsigned char tmphead;
    
    tmphead  = (PULSE_Head + 1) & PULSE_BUFFER_MASK;
    
    if ( tmphead == PULSE_Tail ){
        /* Buffer full */
	return 0;
    } else {
	PULSE_mBuf[tmphead] = mdata;
	PULSE_uBuf[tmphead] = udata;
    	PULSE_Head = tmphead;
	return 1;
    }
}
unsigned char pulse_buf_get(uint16_t* mdata, uint16_t* udata)
{
    unsigned char tmptail;

    if ( PULSE_Head == PULSE_Tail ) {
        return 0;   /* no data available */
    }
    
    /* calculate /store buffer index */
    tmptail = (PULSE_Tail + 1) & PULSE_BUFFER_MASK;
    PULSE_Tail = tmptail; 
    
    /* get data from receive buffer */
	*mdata = PULSE_mBuf[tmptail];
	*udata = PULSE_uBuf[tmptail];
    return 1;
}

#endif //#if USE_PULSE_SEQUENCER > 0