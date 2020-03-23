/******************************************************************************
 * rfm69.c
 *
 * Created: 12.12.2019 
 *  Author: rainer
 *
 * This file contains all code portions, which are specific for RFM69 radio device
 * to work in RFM12 compatibility mode
 * 
 * The RFM69 interconnect for FSK mode consists of 5 lines
 * - MOSI, MISO, SCK and NSEL
 * - plus DIO0 as interrupt line to signal "fifo not empty" in packet rx mode 
 * 
 * The list of specific function prototypes can be found in "rfm_specific.h"
 *
 ******************************************************************************/
#include "config/config.h"

#if USE_RFM69 > 0

#include "task/minitask.h"
#include "global_flags.h"
#include "eeprom.h"
#include "rfm/rfm.h"
#include "rfm/rfm69_lowlevel.h"
#include "rfm/rfm_spi_interface.h"
#include "debug_helper.h"

/* Status read Command */
#define RFM_READ_STATUS()                               rfm_spi_read8( REG_OPMODE )

uint8_t  rfmXX_device_present(void)
{
     return RFM_READ_STATUS() != 0xFF;
}

/******************************************************************************
 * Initialize RF module for normal FSK-Mode ( HR20-like )
 *****************************************************************************/
void rfmXX_init(void)
{
	// Sequencer on .listenmode off, transceiver in stdby mode  ( default after powerup )
	rfm_spi_write8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY );
	
	// Packet Mode, FSK, no Shapening  ( default after powerup )
	rfm_spi_write8( REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 );
	
	// // Datarate 19157 Bd, that is the RFM12-Datarate, that is closest to 19200
	rfm_spi_write16( REG_BITRATE, RFM69_DATARATE(19157) );

	// Frequency deviation 50kHz for RFM12 Compatibility
	rfm_spi_write16( REG_FDEV, RF_FDEV_50000 );

	// Carrier Frequency
	rfm_spi_write24( REG_FRF, RFM69_FRQ(FSK_CARRIER_FREQUENCY) );

	// ***C017***
	#if defined ( RFM69_NO_H )
	// PA0 is the only allowed on non highpower types
	// all other modes won't work
	rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA0_ON |RF_OUTPUTPOWER(0x10) );
	#else
	// Output Power to maximum, PA0 does not work!
	// rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_OUTPUTPOWER(RF_PALEVEL_OUTPUTPOWER_MAX) );
	rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_OUTPUTPOWER(0x10) );
	#endif

	// Receiver input imp 200 ohms, automatic gain select
	rfm_spi_write8( REG_LNA, RF_LNA_ZIN_200 | RF_LNA_GAINSELECT_AUTO );

	// Receiver bandwith depends from baudrate
	rfm_spi_write8( REG_RXBW, RF_GET_RXBW(19200) );

	// Enable Automatic Frq. correction
	// rfm_spi_write8( REG_AFCFEI, RF_AFCFEI_AFCAUTO_ON );

	// Begin send immediate after the first byte in fifo
	// This should be the default value, says manual, but isn't
	// So set manually **** C008 ****
	rfm_spi_write8( REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | 0x01 );

	// RSSI Threshold to 220
	rfm_spi_write8( REG_RSSITHRESH, 220 );
	
	// 2 preamble bytes for rfm12 compatibility
	rfm_spi_write16( REG_PREAMBLE, 2);

	// sync bytes 0x2d 0xd4 for rfm12-compatibility
	rfm_spi_write8( REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 );
	rfm_spi_write16( REG_SYNCVALUE1, 0x2dd4 );

	// Variable packet size, no coding, no crc, no adress filtering
	rfm_spi_write8( REG_PACKETCONFIG1, RF_PACKET_FORMAT_VARIABLE | RF_PACKET_DCFREE_OFF
	| RF_PACKET_CRC_OFF | RF_PACKET_CRCAUTOCLEAR_ON | RF_PACKET_ADRSFILTERING_OFF );

	// 40 Bytes payload
	rfm_spi_write8( REG_PAYLOADLENGTH, 0x40 );

	// CLKCOUT
	#if RFM_CLK_OUTPUT > 0
		rfm_spi_write8( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_00 | RF_DIOMAPPING2_CLKOUT_1 );
	#else
		rfm_spi_write8( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_00 | RF_DIOMAPPING2_CLKOUT_OFF );
	#endif
}

/*!
 *******************************************************************************
 * Switch RFM12 to "Off" stat and disable further interrupts
 ******************************************************************************/
void rfmXX_OFF(void)
{
/* Debug
    switch ( rfm_basemode) {
        case rfm_basemode_rx:
            COM_print_time('r',0);
            break;
        case rfm_basemode_tx:
            COM_print_time('t',0);
            break;
        default:
            COM_print_time('#',0);
            break;
    }
Debug */
    RFM_RXTX_OFF();
    RFM_SPI_DESELECT();
}

/*-----------------------------------------------------------------------------
 * The receive part of rfm69 driver
 *---------------------------------------------------------------------------*/

/*!
 *******************************************************************************
 * Start the RFM169 receiver. 
 ******************************************************************************/
static inline __attribute__((always_inline))
void rfm69_do_receive(void)
{
    RFM_RX_ON();    //re-enable RX
    RFM_INT_EN();   // enable RFM interrupt
}

/*******************************************************************************
 * The common part of all receive routines
 ******************************************************************************/
void rfmXX_receiveBody(void)
{	
    // DBG_PUTS_PP("Recv:");
    // DBG_CRLF();
    
    rfm_CheckIdle();

    // do re-init only if receive after sleep/idle
    if ( ! bTxThenRx ) {
        /* Prepare Receiver for FSK-Mode */
        RFM_Abort(0);
        rfmXX_init();
    }

    rfm_basemode = rfm_basemode_rx;
    UserByteCnt = 0;

    /* Start RFM69 Receiver in FSK mode */
    rfm69_do_receive();
}

/*-----------------------------------------------------------------------------
 * The transmit part of rfm12 driver
 *---------------------------------------------------------------------------*/


/*!
 *******************************************************************************
 * Start the RFM69 transmitter. 
 ******************************************************************************/
static inline __attribute__((always_inline))
void rfm69_start_transmit(void) 
{
    RFM_TX_ON(); // enable Transmitter
    RFM_SPI_SELECT(); // set nSEL low: from this moment SDO indicate FFIT or RGIT
    RFM_INT_EN(); // enable interrupt on DIO0
}

/*!
 *******************************************************************************
 * Enable crystal oscillator an synthesizer in RFM chip. According to manual
 * doing this prior to transmission will shorten the latency time when
 * transmitter is enabled
 ******************************************************************************/
void rfmXX_HeatUpTransmitter(void)
{
    RFM_TX_ON_PRE();
}

/*******************************************************************************
 * Common part of all transmit variants
 * - Fill fifo completely, then start transmitter
 * Due to RFM69 operating in packet mode, Interrupt will be thrown when the 
 * whole packet has been sent.
 * NOTE: Make sure to deploy exactly as many bytes as the length byte ( first )
 * byte specifies. If more bytes are deployed, the RFM69 chip will treat the
 * next byte again as length byte and transmit arbitrary junk!!
 ******************************************************************************/
void rfmXX_TransmitBody(void)
{

	uint8_t TotalTxCount;	/* Number of all transmit bytes fetched so far */

        /*******************************************************************************
         * Callback for filling the transmit fifo by calling user callback
         * or by fetching from buffer.
         * returns RFM_TX_NOMOREDATA when no more data for transmission available
         * If both callback for fetching char and buffer are specified, 
         * the callback has priority!
         ******************************************************************************/
/*local*/inline __attribute__((always_inline))
        uint16_t  rfm_GetTransmitChar( void )
        {
            if ( cbTxFetchByte ) {
                TotalTxCount++;
                return cbTxFetchByte();
            }
            if ( BufPtr ) {
                if ( UserByteCnt >= TxBufLen ||TotalTxCount >= TxBufLen ) {
                    return RFM_TX_NOMOREDATA;
                } else {
                    TotalTxCount++;
                    return *(BufPtr+UserByteCnt++);
                }
            }

            // in case of neither buffer nor callback are set:
            return RFM_TX_NOMOREDATA;
/*local*/}

/*local*/void rfm_TxBufferFill(void)
         {
            uint16_t ch;        /* actual byte or RFM_TX_NOMOREDATA */
            ch=rfm_GetTransmitChar();

            #if DEBUG_MODE > 0
                uint8_t len;        /* store first byte =lengthbyte in variable length mode of RFM69 */
                uint8_t cnt = 0;    /* count the number of following bytes */
                len = (uint8_t)ch;
            #endif
            while ( ch!=RFM_TX_NOMOREDATA ) {
                #if DEBUG_MODE > 0
                    cnt++;
                #endif
                rfm_spi_write8(REG_FIFO, (uint8_t)ch);
                ch=rfm_GetTransmitChar();
            }

            #if DEBUG_MODE > 0 
                /* Check, whether length info byte is equal to length of message ( length byte itself does not count lo length ) */
                if ( len != cnt-1 ) DEBUG_PRINTF("RFM69 FillTxBuffer - Error: Length byte (%d) and real length(%d) differ!\n", len, cnt-1 );
            #endif
/*local*/}

/* ------------------ rfm_transmit_body ------------------------------- */
    // DBG_PUTS_PP("Xmit:");
    // DBG_CRLF();
    
    rfm_CheckIdle();

    rfm_basemode = rfm_basemode_tx;
    UserByteCnt    = 0;
    TotalTxCount = 0;

    /* 
     * RFM69 Fifo can be filled while in Sleep/Idle
     * NOTE: Make sure to deploy exactly as many bytes as the length byte ( first )
     * byte specifies. If more bytes are deployed, the RFM69 chip will treat the
     * next byte again as length byte and transmit arbitrary junk!!
     */
    rfm_TxBufferFill();

    rfm69_start_transmit();
}

/*iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 * Interrupt routines and interrupt handler
 *iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii*/

/*******************************************************************************
 * The RFM basic receive logic:
 * If callback is specified, use callback to deliver and stop, when callback returns 0
 * If buffer/maxlen is specified, store into buffer and stop when maxlen is reached
 ******************************************************************************/
static inline __attribute__((always_inline))
uint8_t rfm_ReceiveChar ( unsigned char rxchar)
{
    uint8_t rxcontinue=1;
    // DBG_PUTC('r');print_hexXX(rxchar);

    /* 
     * If callback is set, call callback and check return value 
     * if callback returns 0, this also will stop receiving
     */
    if ( cbRxByte ) {
        if ( !cbRxByte(rxchar) ) rxcontinue = 0;
    } else {
        /* Store into buffer, if specified and check for buffer full */
        if (BufPtr) {
            *(BufPtr+UserByteCnt) = rxchar;
            if ( ++UserByteCnt >= RxBufLen ) rxcontinue = 0;
        } 
    }
    return rxcontinue;
}


/*******************************************************************************
 * There is no real stram mode in RFM69. So, when packet receptio is complete
 * read the fifo and deliver byte by byte to the upper level until upper level
 * signals termination.
 * Fifo will be empty, on exit of this function
 * Note: This function is executed in Interrupt context!
 ******************************************************************************/
 void rfm_DeliverRxBytes(void)
{ 
#if DEBUG_MODE > 0
    uint8_t dropCount=0;
#endif
    uint8_t rxContinue = 1;
    while ( rxContinue && ( rfm_spi_read8(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) ) {
        rxContinue = rfm_ReceiveChar(rfm_spi_read8(REG_FIFO));
    } 
    if ( !rxContinue ) {
        /* 
         * Clear Fifo, it will not always be emptied for next operation after a receive,
         * see manual, ch. 5.2.2.4 
         */
        while ( rfm_spi_read8(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY ) 
        {
            rfm_spi_read8(REG_FIFO);
            #if DEBUG_MODE > 0
                dropCount++;
            #endif
        }
        #if DEBUG_MODE > 0
            DEBUG_PRINTF(" %d Rx bytes dropped\n",dropCount);
        #endif		
    }

}

/*******************************************************************************
 * Pinchange Interupt for MISO pin
 * This routine is executed in exti - interrupt context, so keep it short! 
 ******************************************************************************/
void HandleFSKInterrupt_RFMXX(uint16_t pin, uint16_t pinvalue, void *arg)
{
    UNUSED(pin); UNUSED(pinvalue); UNUSED(arg);
    // COM_print_time('i',1);
    // DBG_PUTC('i');

    switch ( rfm_basemode ) {
    case rfm_basemode_tx:
        /* Only Switch off to keep the basemode */
        RFM_SwitchOff();
        rfm_basemode = rfm_basemode_txdone;
        TaskNotify(TASK_RFM); // inform the rfm task about end of transmition
        return;
    case rfm_basemode_rx:
        /* Triggered, when the whole packet is received */
        RFM_SwitchOff();
        rfm_DeliverRxBytes();
        rfm_basemode = rfm_basemode_rxdone;
        TaskNotify(TASK_RFM); // inform the rfm task about end of transmition
        return;
    case rfm_basemode_rxook:
        // Not tested!
        // RFM_INT_DIS_OOK(); RHB commented out: need this?
        while ( rfm_spi_read8(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY ) {};
        if ( !rfm_ReceiveChar(rfm_spi_read8(REG_FIFO)) ) {
                /* Only Switch off to keep the basemode */
                RFM_SwitchOff();
                rfm_basemode = rfm_basemode_rxookdone;
                TaskNotify(TASK_RFM); // inform the rfm task about end of transmition
                return;
        } else {
                // RFM_INT_EN_OOK(); RHB commented out: need this?
        }
        break;
    default:
        DEBUG_PUTS("FSK-Intr: Unhandeled state");
        /* Switch off and basemode to idle */
        RFM_Abort(1); 
        return;

    } // switch	
}



/*ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
 o   The OOK-Stuff
 oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo*/
#ifdef USE_RFM_OOK

    /*
     ******************************************************************************
     * Initialise RF module for OOK receiver mode
     ******************************************************************************
     */
    void RFM_OOK_init(void)
    {
            // Sequencer on .listenmode off, transceiver in stdby mode  ( default after powerup )
            rfm_spi_write8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY );
	
            // Packet Mode, FSK, no Shapening  ( default after powerup )
            rfm_spi_write8( REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_01 );
	
            // Datarate 5000 Bd, for 200µs bit length
            // For FS20 this means: 1 = 111000 0 = 1100
            rfm_spi_write16( REG_BITRATE, RFM69_DATARATE(9600) );

            // Frequency deviation 5kHz 
            rfm_spi_write16( REG_FDEV, RF_FDEV_5000 );

            // Carrier Frequency
            rfm_spi_write24( REG_FRF, RFM69_FRQ(config.ook_frq > 0 ? OOK_CARRIER_FREQUENCY1 : OOK_CARRIER_FREQUENCY0) );

            // ***C017***
            #if defined ( RFM69_NO_H )
                    // PA0 is the only allowed on non highpower types
                    // all other modes won't work
                    rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA0_ON |RF_OUTPUTPOWER(0x10) );
            #else
                    // PA0 will not work on hi power types
                    rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_OUTPUTPOWER(0x10) );
                    // rfm_spi_write8(REG_PALEVEL, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_OUTPUTPOWER(config.tx_outpower) );
            #endif
		
            // Receiver input imp 200 ohms, automatic gain select 
            rfm_spi_write8( REG_LNA, RF_LNA_ZIN_200 | RF_LNA_GAINSELECT_AUTO );

            // Receiver bandwith depends from baudrate; 125 kHz is a good value
            // rfm_spi_write8( REG_RXBW, RF_RXBW_OOK_125 );
            rfm_spi_write8( REG_RXBW, RF_RXBW_OOK_62_5 );


            // Enable Automatic Frq. correction
            // rfm_spi_write8( REG_AFCFEI, RF_AFCFEI_AFCAUTO_ON );

            // OOK fix threshold ( 0x60 was a good experimental value )
            //rfm_spi_write8( REG_OOKFIX, config.ook_fixed_thres );
            rfm_spi_write8( REG_OOKFIX, 0x60 );

    
            // OOK Threshold type peak with RFM69 default parameters:
            // Threshold decrement by 0.5 dB
            // decrement once per chip
            rfm_spi_write8( REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK );

            // RSSI Threshold to -90 dB
            // rfm_spi_write8( REG_RSSITHRESH, 180 );
	
            // No preamble bytes 
            rfm_spi_write16( REG_PREAMBLE, 0);

      
            // 
            rfm_spi_write8( REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_5 | RF_SYNC_TOL_0 );
            rfm_spi_write16( REG_SYNCVALUE1, 0x3333 );
            rfm_spi_write16( REG_SYNCVALUE3, 0x3333 );
            rfm_spi_write8( REG_SYNCVALUE5, 0x38 );

            // Variable packet size, no coding, no crc, no adress filtering
            rfm_spi_write8( REG_PACKETCONFIG1, RF_PACKET_FORMAT_FIXED | RF_PACKET_DCFREE_OFF 
                                             | RF_PACKET_CRC_OFF | RF_PACKET_CRCAUTOCLEAR_ON | RF_PACKET_ADRSFILTERING_OFF );

            // Payload length will be set before send begins
            // 

            // CLKCOUT 
            #if RFM_CLK_OUTPUT > 0
                    rfm_spi_write8( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_00 | RF_DIOMAPPING2_CLKOUT_1 );
            #else
                    rfm_spi_write8( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_00 | RF_DIOMAPPING2_CLKOUT_OFF );
            #endif
    }

    /*
     ******************************************************************************
     * Switch on OOK Receiver
     ******************************************************************************
     */
    void RFM_OOK_on(void)
    {

        RFM_INT_DIS();

        rfm_basemode = rfm_basemode_rxook;

        RFM_RX_OOK_ON();    //enable RX in OOK mode

        /* enable RFM interrupt on DIO2-Output */
        OOK_DATA_IRQ_CLEAR();
        OOK_DATA_IRQ_ENABLE();
    }

    /*
     ******************************************************************************
     * Switch off OOK Receiver
     ******************************************************************************
     */
    void RFM_OOK_off(void)
    {
        rfmXX_OFF();
        /* Disable Data-Interrupts */
        OOK_DATA_IRQ_DISABLE();

        rfm_basemode = rfm_basemode_idle;
    }
#endif



/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Debug helper functions
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/

#if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0

#include "debug_helper.h"

#if DEBUG_RFM_STATUS > 1
    #define RFM69_VERBOSE
#endif

#define DEBUG_PUTS_NOCRLF(a)    DEBUG_PRINTF("%s",a)
#define DBG_CRLF()              CRLF()

#ifdef RFM69_VERBOSE
const char e01[] = "OpMode........ ";
const char e02[] = "DataModul..... ";
const char e03[] = "BitRate....... ";
const char e04[] = "Frq.deviation. ";
const char e05[] = "Base Frq...... ";
const char e10[] = "HW-Vers....... ";
const char e11[] = "PA-Lvl........ ";
const char e12[] = "PA-Ramp....... ";
const char e18[] = "LNA........... ";
const char e1b[] = "OOK-Peak...... ";
const char e1c[] = "OOK-Avg....... ";
const char e1d[] = "OOK-FixThres.. ";
const char e1f[] = "AFC-Value..... ";
const char e21[] = "FEI-Value..... ";
const char e24[] = "curr. RSSI.... ";
const char e25[] = "DIO-mapping... ";
const char e27[] = "IrqFlags1.... ";
const char e28[] = "IrqFlags2.... ";
const char e2c[] = "PreambleSize.. ";
const char e2e[] = "SyncConfig.... ";

static void rfm69_decode_opmode( uint8_t opmode )
{
	if (opmode & 0x80 ) DEBUG_PUTS_NOCRLF("Sequ.Off ");
	if (opmode & 0x40 ) DEBUG_PUTS_NOCRLF("Lstn.On ");
	
	opmode >>= 2;
	DEBUG_PUTS_NOCRLF("Mode=");
	switch ( opmode & 0x07 ) {
		case 0:
		DEBUG_PUTS_NOCRLF("Sleep");
		break;
		case 1:
		DEBUG_PUTS_NOCRLF("St.by");
		break;
		case 2:
		DEBUG_PUTS_NOCRLF("FS");
		break;
		case 3:
		DEBUG_PUTS_NOCRLF("TX");
		break;
		case 4:
		DEBUG_PUTS_NOCRLF("RX");
		break;
		default:
		DEBUG_PUTS_NOCRLF("Resvd");
	}
}


static void rfm69_decode_datamodul( uint8_t dmodul )
{
	switch ( dmodul & 0x60 ) {
		case 0x00:
		DEBUG_PUTS_NOCRLF("Packetmode");
		break;
		case 0x20:
		DEBUG_PUTS_NOCRLF("Resvd!");
		break;
		case 0x40:
		DEBUG_PUTS_NOCRLF("Cont. w/ BitSync");
		break;
		case 0x60:
		DEBUG_PUTS_NOCRLF("Cont. w/o BitSync");
		break;
	}

	DEBUG_PUTC(' ');
	switch ( dmodul & 0x18 ) {
		case 0x00:
		DEBUG_PUTS_NOCRLF("FSK");
		break;
		case 0x08:
		DEBUG_PUTS_NOCRLF("OOK");
		break;
		default:
		DEBUG_PUTS_NOCRLF("Resvd!");
	}

	DEBUG_PUTC(' ');
	dmodul &= 0x03;
	DEBUG_PUTS_NOCRLF("Shaping=");
	DEBUG_PUTC(dmodul+'0');
}

static void rfm69_decode_palevel( uint8_t opmode )
{
	if (opmode & 0x80 ) DEBUG_PUTS_NOCRLF("PA0 ");
	if (opmode & 0x40 ) DEBUG_PUTS_NOCRLF("PA1 ");
	if (opmode & 0x20 ) DEBUG_PUTS_NOCRLF("PA2 ");

	opmode &= 0x1f;
	DEBUG_PUTS_NOCRLF("Output-Pwr=");
	print_hexXX(opmode);
}

static void rfm69_decode_lna( uint8_t opmode )
{
	DEBUG_PUTS_NOCRLF("Inp. Imped. ");
	if (opmode & 0x80 )
	DEBUG_PUTS_NOCRLF("200");
	else
	DEBUG_PUTS_NOCRLF("50");

	DEBUG_PUTS_NOCRLF(" Curr. Gain ");
	print_hexXX( opmode >> 3 & 0x07 );

	DEBUG_PUTS_NOCRLF(" Sel. Gain ");
	print_hexXX( opmode  & 0x07 );
}

static void	rfm69_decode_ook_peak(uint8_t mode)
{
	DEBUG_PUTS_NOCRLF("Thres.type ");
	switch(mode >> 6 ) {
		case 0b00:
		DEBUG_PUTS_NOCRLF("fixed");
		break;
		case 0b01:
		DEBUG_PUTS_NOCRLF("peak");
		break;
		case 0b10:
		DEBUG_PUTS_NOCRLF("average");
		break;
		default:
		DEBUG_PUTS_NOCRLF("resvd!");
	}

	DEBUG_PUTS_NOCRLF(" Thres.stepwidth ");
	switch( (mode >> 3) & 0b111 ) {
		case 0b000:
		DEBUG_PUTS_NOCRLF("0.5");
		break;
		case 0b001:
		DEBUG_PUTS_NOCRLF("1");
		break;
		case 0b010:
		DEBUG_PUTS_NOCRLF("1.5");
		break;
		case 0b011:
		DEBUG_PUTS_NOCRLF("2");
		break;
		case 0b100:
		DEBUG_PUTS_NOCRLF("3");
		break;
		case 0b101:
		DEBUG_PUTS_NOCRLF("4");
		break;
		case 0b110:
		DEBUG_PUTS_NOCRLF("5");
		break;
		case 0b111:
		DEBUG_PUTS_NOCRLF("6");
	}
	DEBUG_PUTS_NOCRLF("dB ");

	DEBUG_PUTS_NOCRLF(" Peak decr=");
	print_hexXX(mode & 0b111 );
	
}

static void rfm69_decode_diomapping ( uint16_t map )
{
	uint8_t work;
	uint8_t fclk;
	uint8_t i;

	fclk = (uint8_t)(map & 0x07 );

	for ( i = 6; i; i-- ) {
		work = (uint8_t)( map >> 14 );
		map <<= 2;
		DEBUG_PUTC(work+'0');
		DEBUG_PUTC(' ');
	}
	
	DEBUG_PUTS_NOCRLF("ClkOut frq=");
	if ( fclk == 7 )
	DEBUG_PUTS_NOCRLF("Off");
	else if ( fclk == 6 )
	DEBUG_PUTS_NOCRLF("RC-Mode");
	else {
		DEBUG_PUTS_NOCRLF("FOSC/");
		print_decXX(1<<fclk);
	}
}

static void rfm69_decode_irqflags1( uint8_t flags )
{
	if (flags & 0x80 ) DEBUG_PUTS_NOCRLF("ModeRdy ");
	if (flags & 0x40 ) DEBUG_PUTS_NOCRLF("RxRdy ");
	if (flags & 0x20 ) DEBUG_PUTS_NOCRLF("TxRdy ");
	if (flags & 0x10 ) DEBUG_PUTS_NOCRLF("PLL-lock ");
	if (flags & 0x08 ) DEBUG_PUTS_NOCRLF("RSSI ");
	if (flags & 0x04 ) DEBUG_PUTS_NOCRLF("Timeout ");
	if (flags & 0x02 ) DEBUG_PUTS_NOCRLF("Automode ");
	if (flags & 0x01 ) DEBUG_PUTS_NOCRLF("Addr-match ");
}
static void rfm69_decode_irqflags2( uint8_t flags )
{
	if (flags & 0x80 ) DEBUG_PUTS_NOCRLF("Fifo-full ");
	if (flags & 0x40 ) DEBUG_PUTS_NOCRLF("FifoNotEmpty ");
	if (flags & 0x20 ) DEBUG_PUTS_NOCRLF("FifoLevel ");
	if (flags & 0x10 ) DEBUG_PUTS_NOCRLF("FifoOverrun ");
	if (flags & 0x08 ) DEBUG_PUTS_NOCRLF("PacketSent ");
	if (flags & 0x04 ) DEBUG_PUTS_NOCRLF("PayloadRdy");
	if (flags & 0x02 ) DEBUG_PUTS_NOCRLF("CrcOk");
}

static void rfm69_decode_syncconfig(uint8_t val)
{
	if (val & 0x80 ) {
		DEBUG_PUTS_NOCRLF("Sync On ");
		} else {
		DEBUG_PUTS_NOCRLF("No Sync");
		return;
	}
	DEBUG_PUTS_NOCRLF("FillCond=");
	DEBUG_PUTC(((val&0x40)>>6)+'0');
	
	DEBUG_PUTS_NOCRLF(" SyncSize=");
	DEBUG_PUTC(((val>>3)&0x07)+'1');
	
	DEBUG_PUTS_NOCRLF(" SyncTol=");
	DEBUG_PUTC((val&0x07)+'0');
}


#endif

static void rfm_dump_irq_status( void )
{
	uint8_t ret8;
	ret8 = rfm_spi_read8(REG_IRQFLAGS1);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e27);
	rfm69_decode_irqflags1(ret8);
	#else
	print_hexXX(REG_IRQFLAGS1);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_IRQFLAGS2);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e28);
	rfm69_decode_irqflags2(ret8);
	#else
	print_hexXX(REG_IRQFLAGS2);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();
}

void rfmXX_dump_status ( void )
{
	uint8_t ret8;
	uint16_t ret16;

	ret8 = rfm_spi_read8(REG_OPMODE);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e01);
	rfm69_decode_opmode(ret8);
	#else
	print_hexXX(REG_OPMODE);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_DATAMODUL);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e02);
	rfm69_decode_datamodul(ret8);
	#else
	print_hexXX(REG_DATAMODUL);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	ret16 = rfm_spi_read16(REG_BITRATE);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e03);
	#else
	print_hexXX(REG_BITRATE);DEBUG_PUTC('=');
	#endif

	print_hexXXXX(ret16);
	DBG_CRLF();

	ret16 = rfm_spi_read16(REG_FDEV);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e04);
	#else
	print_hexXX(REG_FDEV);DEBUG_PUTC('=');
	#endif
	print_hexXXXX(ret16);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_FRF);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e05);
	#else
	print_hexXX(REG_FRF);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8);
	ret16 = rfm_spi_read16(REG_FRF+1);
	print_hexXXXX(ret16);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_VERSION);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e10);
	#else
	print_hexXX(REG_VERSION);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_PALEVEL);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e11);
	rfm69_decode_palevel(ret8);
	#else
	print_hexXX(REG_PALEVEL);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_PARAMP) & 0x0f;
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e12);
	#else
	print_hexXX(REG_PARAMP);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_LNA);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e18);
	rfm69_decode_lna(ret8);
	#else
	print_hexXX(REG_LNA);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	#define OOK 1
	#if OOK > 0
	ret8 = rfm_spi_read8(REG_OOKPEAK);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e1b);
	rfm69_decode_ook_peak(ret8);
	#else
	print_hexXX(REG_OOKPEAK);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_OOKAVG);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e1c);
	#else
	print_hexXX(REG_OOKAVG);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8>>6);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_OOKFIX);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e1d);
	#else
	print_hexXX(REG_OOKFIX);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8);
	DBG_CRLF();

	#endif

	ret16 = rfm_spi_read16(REG_AFCMSB);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e1f);
	#else
	print_hexXX(REG_AFCMSB);DEBUG_PUTC('=');
	#endif
	print_hexXXXX(ret16);
	DBG_CRLF();

	ret16 = rfm_spi_read16(REG_FEIMSB);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e21);
	#else
	print_hexXX(REG_FEIMSB);DEBUG_PUTC('=');
	#endif
	print_hexXXXX(ret16);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_RSSIVALUE);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e24);
	#else
	print_hexXX(REG_RSSIVALUE);DEBUG_PUTC('=');
	#endif
	print_hexXX(ret8);
	DBG_CRLF();

	ret16 = rfm_spi_read16(REG_DIOMAPPING1);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e25);
	rfm69_decode_diomapping(ret16);
	#else
	print_hexXX(REG_RSSIVALUE);DEBUG_PUTC('=');
	print_hexXXXX(ret16);
	#endif
	DBG_CRLF();

	rfm_dump_irq_status();

	ret16 = rfm_spi_read16(REG_PREAMBLE);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e2c);
	#else
	print_hexXX(REG_PREAMBLE);DEBUG_PUTC('=');
	#endif
	print_hexXXXX(ret16);
	DBG_CRLF();

	ret8 = rfm_spi_read8(REG_SYNCCONFIG);
	#ifdef RFM69_VERBOSE
	DEBUG_PUTS_NOCRLF(e2e);
	rfm69_decode_syncconfig(ret8);
	#else
	print_hexXX(REG_SYNCCONFIG);DEBUG_PUTC('=');
	print_hexXX(ret8);
	#endif
	
	if ( ret8 & 0x80 ) {
		for ( ret16=0; ret16 <= ((ret8 >> 3) & 0x07	) ; ret16++ ) {
			DEBUG_PUTC(' ');print_hexXX( rfm_spi_read8(REG_SYNCVALUE1+ret16));
		}
	}
	DBG_CRLF();

}
#endif

#endif /* #if USE_RFM69 > 0 */
