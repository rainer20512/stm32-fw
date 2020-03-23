/******************************************************************************
 * Access Dallas 1-Wire Devices via STM32 U(S)ART
 * Author of the initial code: Peter Dannegger (danni(at)specs.de)
 * modified by Martin Thomas (mthomas(at)rhrk.uni-kl.de)
 * migrated to STM32 by rainer
 *
 ******************************************************************************/

#include "config/config.h"

#include "dev/uart_dev.h"
#include "debug_helper.h"
#include "onewire.h"
#include "timer.h"
#include "rtc.h"

#if USE_ONEWIRE > 0

#if USE_DS18X20 > 0
    #include "ds18xxx20.h"
#endif

/* Timeout values */
#define OW_TIMEOUT_RESET       8       /* Timeout for reset-command */
#define OW_TIMEOUT_NORMALMIN   4       /* minimum timeout for normal commands */
#define OW_TIMEOUT_NORMAL      1       /* Timeout per byte for normal commands */


#define CALC_TIMEOUT_NORMAL(a)  MAX(OW_TIMEOUT_NORMALMIN, (a*OW_TIMEOUT_NORMAL) )

/* Assignment of U(S)ART to use ----------------------------------------------
 * USART1 ALTN1(Tx=A9 Rx=A10)
 */

/* remember U(S)ART to use ----------------------------------------------
 * this is currently USART1 ALTN1(Tx=A9 Rx=A10)
 */
static UsartHandleT *myUHandle=NULL;

//#define MY_UART (&HandleCOM1)

/* 
 * We need two different baudrates: 
 * 9600 Bd   - to generate the Reset Pulse and read the presence pulse 
 * 115200 Bd - to transmit/receive Data Bits
 */
#define BR_RESET            9600
#define BR_DATA             115200
#define RESET_PATTERN       0xF0
#define HIGH_PATTERN        0xFF
#define LOW_PATTERN         0x00
#define OW_BUFSIZE          80      /* 1 byte match ROM or Skip ROM, 8 bytes ID, 1 byte command */

/* private structs and typedefs ---------------------------------------------*/

static uint8_t  ow_iobuf[OW_BUFSIZE];
static uint32_t ow_bufptr;                  /* current index into iobuf */
static uint8_t  ow_iochar;                  /* Send/Receive a single bit (e.g. reset ) */
static volatile bool    ow_terminate_wait;  /* flag to terminate active wait, set by usart CB */
static int8_t   ow_myTimerID;               /* Timeout TimerID used in async functions */

static OneWireCB ow_uCB = NULL;              /* callback for asynchronous I/O operations */
static OneWireCB ow_tCB = NULL;              /* callback for asynchronous timeouts       */

// Nrof Sensors and their IDs
uint8_t ow_nSensors;
uint8_t ow_SensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];


/* internal callbacks--------------------------------------------------------*/

/******************************************************************************
 * Terminate an active wait, used as internal callback
 *****************************************************************************/
static void OneWire_RxWaitCallback( UsartHandleT* u, uint8_t ch)
{
    UNUSED(u); UNUSED(ch);
    ow_terminate_wait = true;
}

/******************************************************************************
 * Async communication done callback: 
 * Delete Timeout timer, if specified and call user Callback if set
 *****************************************************************************/
static void OneWire_RxAsyncCallback( UsartHandleT* u, uint8_t ch)
{
    UNUSED(u); UNUSED(ch);
    if ( ow_myTimerID >= 0 ) MsTimerDelete(ow_myTimerID);
    #if DEBUG_MODE > 0  && DEBUG_ONEWIRE > 1
        DEBUG_PRINTTS("ow async cb\n");
    #endif
    if ( ow_uCB ) ow_uCB();
}

/******************************************************************************
 * Async timeout callback: 
 * just call user function, if set
 *****************************************************************************/
static void OneWire_AsyncTmoCallback( uint32_t timerID )
{
    UNUSED(timerID);
    UsartAbortOp(myUHandle);
    #if DEBUG_MODE > 0  && DEBUG_ONEWIRE > 0
        DEBUG_PRINTTS("ow tmo cb Rx=%d,Tx=%d\n",myUHandle->hRxDma->Instance->CNDTR,myUHandle->hTxDma->Instance->CNDTR);
    #endif
    if ( ow_tCB ) ow_tCB();
}



/* private functions --------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 - Utility functions, public --------------------------------------------------
 ----------------------------------------------------------------------------*/

/******************************************************************************
 * Reset encoder to begin encoding at first byte
 *****************************************************************************/
void ow_encoder_reset(void )
{
    ow_bufptr = 0;
}

/******************************************************************************
 * Encode one Byte
 * and store it into ow_iobuf at the current write position
 *****************************************************************************/
bool ow_encode_byte( const uint8_t b )
{
    uint8_t *wrptr = ow_iobuf+ow_bufptr;
    uint8_t mask = 1;
    if ( ow_bufptr + 8 > OW_BUFSIZE ) return false;
    for ( uint32_t j = 0; j < 8; j++ ) {
        *wrptr = ( b & mask ? HIGH_PATTERN : LOW_PATTERN ); 
        wrptr++;
        mask <<= 1;
    }
    ow_bufptr += 8;
    return true;
}

/******************************************************************************
 * Encode bits into Sequence of HIGH_PATTERN or LOW_PATTERN
 * and store them into ow_iobuf
 *****************************************************************************/
bool ow_encode_vector( uint8_t *in, uint32_t len )
{
    uint8_t *rdptr = in;
    bool ret = true;
    assert(ow_bufptr + len * 8 <= OW_BUFSIZE );
    for ( uint32_t i = 0; ret && i < len; i++ ) {
        ret = ow_encode_byte(*(rdptr++));
    }
    return ret;
}

/******************************************************************************
 * Decode one byte at position "ofs" within "ow_iobuf"
 * and return result in "byte"
 *****************************************************************************/
void ow_decode_byte ( uint8_t *byte, uint32_t ofs )
{
    uint8_t *rdptr = ow_iobuf+ofs;
    uint8_t mask = 1;

    *byte = 0x00;
    for ( uint32_t j = 0; j < 8; j++ ) {
        if ( *rdptr == HIGH_PATTERN ) *byte |= mask;
        rdptr++;
        mask <<= 1;
    }
}

/******************************************************************************
 * Decode ow_iobuf into "vector"
 * the whole actual length of buffer will be decoded, if length of encoded buffer
 * is not a multiple of 8, the last partial byte is ignored
 * @note No checking of bounds for "vector!
 *****************************************************************************/
void ow_decode_vector ( uint8_t *vector )
{
    uint8_t *dptr = vector;
    for ( uint32_t i = 0; i < ow_bufptr / 8; i++ ) {
        ow_decode_byte(dptr, i*8);
        dptr++;
    }
}

/******************************************************************************
 * Preset the encoder bytes with a constant value, i.e. HIGH_PATTERN
 * for a following read operation
 * Preset is always started at the beginning
 *****************************************************************************/
void ow_encoder_preset(uint32_t len )
{
    if ( len * 8 > OW_BUFSIZE ) return;
    ow_bufptr = len * 8;
    memset(ow_iobuf, HIGH_PATTERN, ow_bufptr );
}

/******************************************************************************
 * Get the actual length of the encode buffer "ow_iobuf" in bytes
 * One data byte will be decoded with 8 bytes in "ow_iobuf"
 *****************************************************************************/
uint32_t ow_encoder_getbuflen(void)
{
    return ow_bufptr / 8;
}



/*-----------------------------------------------------------------------------
 - private functions ----------------------------------------------------------
 ----------------------------------------------------------------------------*/

/******************************************************************************
 * Change the UART's baudrate
 *****************************************************************************/
static void SetBaudrate(uint32_t baudRate)
{
    /* if baudrate already set to desired value, then do nothing */
    if ( myUHandle->baudrate == baudRate ) return;

    if (!Usart_SetCommParams(myUHandle, baudRate, false) ) {
        #if DEBUG_MODE > 0 
            DEBUG_PUTS("Error: Failed to change baudrate");
        #endif
    }
}


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
- Synchronous functions, i.e. all following functions will wait for termination
-------------------------------------------------------------------------------
 ----------------------------------------------------------------------------*/

/******************************************************************************
 * write one bit via Polling
 * Does not set/chack baudrate, has to be done by caller
 *****************************************************************************/
static bool ow_write_bit_wait(bool b)
{
    uint8_t txrx = (b ? HIGH_PATTERN : LOW_PATTERN );

    UsartTxRxOneByteWait (myUHandle, &txrx, CALC_TIMEOUT_NORMAL(1) );
    return txrx == HIGH_PATTERN;
}

/******************************************************************************
 * read one bit via Polling
 * Reding is done by writing the high pattern, slave would pull down in case
 * of zero, so read result is one (true), if 0xff is read
 *
 * Does not set/check baudrate, has to be done by caller
 *****************************************************************************/
#define ow_read_bit_wait()   ow_write_bit_wait(true)


/******************************************************************************
 * write one byte via active wait
 * i.e. the function does not return before the byte is not sent completely
 *****************************************************************************/
static bool ow_write_byte_wait( uint8_t b, uint32_t tmo )
{
    uint32_t ticktmo = HAL_GetTick() + tmo;
    bool ret;

    ow_encoder_reset();
    if (! ow_encode_byte(b) ) return false;

    ow_terminate_wait = false;
    UsartAssignCallbacks(myUHandle,  NULL, NULL, OneWire_RxWaitCallback );

    /* Enable Receiver before transmitter starts! */
    UsartStartRx(myUHandle,ow_iobuf,8);
    UsartStartTx(myUHandle,ow_iobuf,8);

    /* wait for termination or timeout */
    while ( !ow_terminate_wait && HAL_GetTick() != ticktmo ) ;
    ret = HAL_GetTick() != ticktmo;

    /* Restore async callback, if set */
    UsartAssignCallbacks(myUHandle,  NULL, NULL, OneWire_RxAsyncCallback );


    return ret;

}

/******************************************************************************
 * read one byte via active wait
 * this is done by writing the HIGH_PATTERN and then examinating the response
 * the byte read will be returned.
 *****************************************************************************/
 static uint8_t ow_read_byte_wait( uint32_t tmo )
{
    uint8_t ret;

    // read by sending only "1"s, so bus gets released
    // after the init low-pulse in every slot
    ow_write_byte_wait( HIGH_PATTERN, tmo ); 
    ow_decode_byte(&ret, 0);

    return ret;
}

/******************************************************************************
 * Send an reset pulse and scan for presence pulse in active wait mode
 * This is done by setting the baudrate to 9600 Bd, ie. a bit time of 104 us.
 * So, startbit and 4 data bits of zero will form a 520 us low pulse for reset
 * ( reqired minimum is 480us ).
 * The presence pulse of the device ( 60-240 us ) will result in reception
 * of the fifth bit as zero, ie. 0XE0 is expected to be received
 * if no device is answering, the received data is identical to the sent data
 *****************************************************************************/
static bool ow_reset_wait(void)
{
    uint32_t old_br = myUHandle->baudrate;
    bool ret;
    SetBaudrate(BR_RESET);
    ow_iochar = RESET_PATTERN;

    if (!UsartTxRxOneByteWait (myUHandle, &ow_iochar, OW_TIMEOUT_RESET)) 
        ret =  false;
    else 
        ret = ow_reset_successful();

    /* Restore previous baudrate */
    if ( old_br != BR_RESET) SetBaudrate(old_br);

    return ret;
}


/******************************************************************************
 * do one rom search pass and return the position of first difference
 *****************************************************************************/
static uint8_t ow_one_search_pass( uint8_t diff, uint8_t *id )
{
    uint8_t i, j, next_diff;
    bool b;
    
    if( !ow_reset_wait() ) {
        return OW_PRESENCE_ERR;         // error, no device found <--- early exit!
    }
    
    if ( !ow_write_byte_wait( OW_SEARCH_ROM, CALC_TIMEOUT_NORMAL(1) ) )         // ROM search command
        return OW_DATA_ERR;

    next_diff = OW_LAST_DEVICE;         // unchanged on last device
    i = OW_ROMCODE_SIZE * 8;            // 8 bytes
    
    do {
        j = 8;                          // 8 bits
        do {
            b = ow_read_bit_wait();     // read bit
            if( ow_read_bit_wait() ) {  // read complement bit
                if( b ) {               // 0b11 is not possible
                    return OW_DATA_ERR; // data error <--- early exit!
                }
            } else {
                if( !b ) {              // 0b00 = more than one device
                    if( diff > i || ((*id & 1) && diff != i) ) {
                        b = 1;          // now 1
                        next_diff = i;  // next pass 0
                    }
                }
            }
            ow_write_bit_wait( b );     // write bit
            *id >>= 1;
            if( b ) *id |= 0x80;        // store bit
            i--;
        } while( --j );
        
        id++;                           // next byte

    } while( i );
    
    return next_diff;                   // to continue search
}



/* public functions ---------------------------------------------------------*/

/******************************************************************************
 * Check, if a previous reset pulse generated a presence pulse
 *****************************************************************************/
bool ow_reset_successful ( void )
{
    return ow_iochar != RESET_PATTERN;
}

/******************************************************************************
 * Check, whether the last asny bit read delivered 1 or 0 
 * will be returned as true or false;
 *****************************************************************************/
bool ow_get_bitval(void)
{
    return ow_iochar != HIGH_PATTERN;
}

/******************************************************************************
 * Reset asynchronous callbacks for successful execution
 * and for timeout
 * If synchronous functions are called after setting the async callbacks,
 * the async callbacks will be overwritten!
 *****************************************************************************/
void ow_set_callbacks(OneWireCB onCommDone, OneWireCB onTimeout)
{
    ow_uCB = onCommDone;
    ow_tCB = onTimeout;
}

/******************************************************************************
 * Send an reset pulse and scan for presence pulse in callback mode
 * This is done by setting the baudrate to 9600 Bd, ie. a bit time of 104 us.
 * So, startbit and 4 data bits of zero will form a 520 us low pulse for reset
 * ( reqired minimum is 480us ).
 * The presence pulse of the device ( 60-240 us ) will result in reception
 * of the fifth bit as zero, ie. 0XE0 is expected to be received
 * if no device is answering, the received data is identical to the sent data
 * Callback function must have been set before!
 *****************************************************************************/
void ow_reset_with_cb(bool bWithTimeout)
{
    SetBaudrate(BR_RESET);
    ow_iochar = RESET_PATTERN;

    UsartStartRx(myUHandle, &ow_iochar,1);
    UsartStartTx(myUHandle, &ow_iochar,1);

    if ( bWithTimeout) 
        ow_myTimerID = MsTimerSetRel(MILLISEC_TO_TIMERUNIT(OW_TIMEOUT_RESET), false, OneWire_AsyncTmoCallback, 0);
    else
        ow_myTimerID = -1;
}

/******************************************************************************
 * Write "ow_iobuf" of length "ow_bufptr" to OneWire with nowait (async)
 * callbacks must have been set before via "ow_set_callbacks"
 * if requested, timeout is calculated and set automatically. Timeout period
 * depends from length of buffer. 
 * If the buffer is preset with HIGH_PATTERN, this function can also be used
 * to read from OneWire
 *****************************************************************************/
void ow_write_buffer_with_cb(bool bWithTimeout)
{
    SetBaudrate(BR_DATA);
    UsartStartRx(myUHandle, ow_iobuf,ow_bufptr);
    UsartStartTx(myUHandle, ow_iobuf,ow_bufptr);

    if ( bWithTimeout) {
        ow_myTimerID = MsTimerSetRel(MILLISEC_TO_TIMERUNIT(CALC_TIMEOUT_NORMAL(ow_bufptr)), false, OneWire_AsyncTmoCallback, 0);
        // DEBUG_PRINTTS("SetTMO to +%d\n",CALC_TIMEOUT_NORMAL(ow_bufptr));
    } else
        ow_myTimerID = -1;
}

/***********************************************************************
 * Scan the bus for all connected OneWire devices and return the number
 * of sensors found
 * This functions operates synchronously, i.e. will wait for termination
 * of every onewire-transfer.
 * This may be very time consuming when many onewire devices are found
 ***********************************************************************/
uint8_t ow_rom_search(void)
{
    uint8_t diff;
     
    ow_reset_wait();

    ow_nSensors = 0;
     
    diff = OW_SEARCH_FIRST;
    while ( diff != OW_LAST_DEVICE && ow_nSensors < MAXSENSORS ) {
        diff = ow_one_search_pass( diff, &ow_SensorIDs[ow_nSensors][0] ); 
        if( diff == OW_PRESENCE_ERR ) {
            #if DEBUG_MODE > 0
                if ( debuglevel > 0 ) DEBUG_PUTS( "No OW Sensor found");
            #endif
            break;
        }
         
        if( diff == OW_DATA_ERR ) {
            #if DEBUG_MODE > 0 
                if ( debuglevel > 0 ) DEBUG_PUTS( "OW Bus Error");
            #endif
            break;
        }
                     
        ow_nSensors++;
    }
     
     return ow_nSensors;
}

/******************************************************************************
 * Dump the content of "ow_SensorIDs", i.e. show all found ROMs 
 *****************************************************************************/
void ow_dump_rom(void)
{
    printf("found %d OneWire devices\n", ow_nSensors );
    for ( uint8_t i = 0; i < ow_nSensors; i++ ) {
        printf("%2d) ",i);
        for ( uint8_t j=0; j < OW_ROMCODE_SIZE; j++ ) {
            print_hexXX(ow_SensorIDs[i][j]);putchar(' ');
        }
        putchar('\n');
    }       
}

/******************************************************************************
 * write one bit async
 *****************************************************************************/
void ow_write_bit_with_cb(bool bitval, bool bWithTimeout)
{
    ow_iochar = (bitval ? HIGH_PATTERN : LOW_PATTERN );
    SetBaudrate(BR_DATA);

    /* The RxComplete callback will be called very soon, so first activate timeout */
    if ( bWithTimeout) 
        ow_myTimerID = MsTimerSetRel(MILLISEC_TO_TIMERUNIT(CALC_TIMEOUT_NORMAL(OW_TIMEOUT_NORMALMIN*2)), false, OneWire_AsyncTmoCallback, 0);
    else
        ow_myTimerID = -1;

    UsartStartRx(myUHandle, &ow_iochar,1);
    UsartStartTx(myUHandle, &ow_iochar,1);

}

/******************************************************************************
 * read one bit async
 * Caller has to use 'ow_get_bit_value' on termination to readout the bit value
 *****************************************************************************/
void ow_read_bit_with_cb(bool bWithTimeout)
{   
    ow_write_bit_with_cb(true, bWithTimeout);
}

/******************************************************************************
 * returns true, if underlying serial device is initialzed
 *****************************************************************************/
bool ow_Initialized(void)
{
    return ( myUHandle != NULL );
}

/******************************************************************************
 * Actions to be taken after initialization of underlying serial device
 *****************************************************************************/
void OW_PostInit(const HW_DeviceType *dev, void *arg)
{
    UNUSED(arg);
    myUHandle = USART_GetHandleFromDev(dev);

    SetBaudrate(BR_DATA);
    /* 
     * Assign Async callback as default, wait functions will use their 
     * own callback, but will restore to Async when done
     */
    UsartAssignCallbacks(myUHandle,  NULL, NULL, OneWire_RxAsyncCallback );
    
    /* Perform ROM search and store all found devices in ow_SensorIDs */
    if ( ow_rom_search() == 0 ) {
        DEBUG_PUTS("No OneWire devices found!");
    } else {
        ow_dump_rom();
    }

    /* Init all devices, which make use of onewire */
    #if USE_DS18X20 > 0
        DS18X20_Init();
    #endif
}

/******************************************************************************
 * Actions to be taken before deinitialization of underlying serial device
 *****************************************************************************/
void OW_PreDeInit(const HW_DeviceType *dev)
{
    UNUSED(dev);

    /* DeInit all devices, that make use of OneWire */
    // ... 
    
    /* invalidate ROM IDs and clear link to U(S)ART */
    ow_nSensors = 0;
    myUHandle   = 0;
}







#if OW_TEST > 0


/******************************************************************************
 * Test USART data transfer with loopback
 *****************************************************************************/

#define MAX_TESTBUF 255

static uint8_t      testbuf[MAX_TESTBUF];
static uint8_t      recvbuf[MAX_TESTBUF];
static UsartHandleT *testUHandle;
static int8_t       test_myTimerID;               /* Timeout TimerID used in async functions */
static uint8_t      test_actlen;

#define MIN(a,b) (a<b ? a : b )

void test_showdiff(void)
{
    uint8_t rxlen = testUHandle->RxCount;
    if (rxlen != test_actlen)
        printf("different lengths: Tx=%d, Rx=%d\n", test_actlen, rxlen);
    
    uint8_t limit = MIN(rxlen, test_actlen);
    uint8_t nodiff = 1;

    register uint8_t *tptr = testbuf;
    register uint8_t *rptr = recvbuf;
    for ( register uint8_t i = 0; i < limit; i++ )
        if ( *(tptr++) != *(rptr++) ) { nodiff = 0; break; }

    if ( nodiff ) {
        printf("No difference within first %d bytes\n", limit);
        return;
    }

    tptr = testbuf;
    rptr = recvbuf;
    for ( register uint8_t i = 0; i < limit; i++ )
        printf("%02x ", *(tptr++));
    puts("");

    for ( register uint8_t i = 0; i < limit; i++ )
        printf("%02x ", *(rptr++));
    puts("");

    
}


/******************************************************************************
 * Async communication done callback: 
 * Delete Timeout timer, if specified and call user Callback if set
 *****************************************************************************/
static void Test_RxAsyncCallback( UsartHandleT* u, uint8_t ch)
{
    UNUSED(u); UNUSED(ch);
    if ( test_myTimerID >= 0 ) MsTimerDelete(test_myTimerID);
    DEBUG_PRINTTS("test async cb\n");
    test_showdiff();
}

/******************************************************************************
 * Async timeout callback: 
 * just call user function, if set
 *****************************************************************************/
static void Test_AsyncTmoCallback( uint32_t timerID )
{
    UNUSED(timerID);
    UsartAbortOp(testUHandle);
    DEBUG_PRINTTS("test tmo cb Rx=%d,Tx=%d\n",testUHandle->hRxDma->Instance->CNDTR, testUHandle->hTxDma->Instance->CNDTR);
    test_showdiff();
}

/******************************************************************************
 * Actions to be taken after initialization of underlying serial device
 *****************************************************************************/
void OWTEST_PostInit(const HW_DeviceType *dev, void *arg)
{
    UNUSED(arg);
    testUHandle = USART_GetHandleFromDev(dev);


    if (!Usart_SetCommParams(testUHandle, BR_DATA, true) ) {
        #if DEBUG_MODE > 0 
            DEBUG_PUTS("Error: Failed to change baudrate");
        #endif
    }
    /* 
     * Assign Async callback as default, wait functions will use their 
     * own callback, but will restore to Async when done
     */
    UsartAssignCallbacks(testUHandle,  NULL, NULL, Test_RxAsyncCallback );
    
}
static uint8_t testbuf_init(uint8_t len)
{
    register uint8_t *ptr = testbuf;
    register uint8_t limit = MIN(MAX_TESTBUF,len);
    for ( register uint8_t i = 0; i < limit; i++ )
       *(ptr++) = i;

    return limit;
}


void test_write_buffer_with_cb(uint8_t len)
{
    test_actlen = testbuf_init(len);
    UsartStartRx(testUHandle, recvbuf, test_actlen);
    UsartStartTx(testUHandle, testbuf,test_actlen);

    test_myTimerID = MsTimerSetRel(MILLISEC_TO_TIMERUNIT(CALC_TIMEOUT_NORMAL(test_actlen)), false, Test_AsyncTmoCallback, 0);
    DEBUG_PRINTTS("SetTMO to +%d\n",CALC_TIMEOUT_NORMAL(test_actlen));
}
#endif // #if OW_TEST > 0


#endif // #if USE_ONEWIRE > 0
