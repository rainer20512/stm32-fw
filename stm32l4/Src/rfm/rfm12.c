/******************************************************************************
 * rfm12.c
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * This file contains all code portions, which are specific for RFM12 radio device
 * operated in FSK or OOK mode
 *
 * The RFM12 interconnect for FSK mode consists of 4 lines
 * - MOSI, MISO, SCK and NSEL
 * - "Data available" in FSK Rx mode is signalled by a falling edge on MISO when
 *   NSEL is active (LOW). So MISO is not only used as normal MISO, but also as
 *   Interrupt source in Rx mode.
 * 
 * The list of specific function prototypes can be found in "rfm_specific.h"
 *
 ******************************************************************************/
#include "config/config.h"

#if USE_RFM12 > 0

#include "task/minitask.h"
#include "global_flags.h"
#include "rfm/rfm.h"
#include "rfm/rfm12_lowlevel.h"
#include "rfm/rfm_spi_interface.h"
#include "debug_helper.h"

typedef enum {
	rfm_submode_normal=0,
	rfm_submode_tx_preamble=12,
	rfm_submode_tx_userdata=13,
	rfm_submode_tx_trailer=14,
} rfm_submode_t;

static rfm_submode_t  rfm_submode  = rfm_submode_normal;
static uint8_t ActBufIdx;		/* internal counter when accessing byte arrays during transmit/send */

static const uint8_t rfm_header[4] = {
    0xaa, 0xaa, 0x2d, 0xd4
};

#define RFM_PREAMBLE_SIZE	sizeof(rfm_header)
#define RFM_TRAILER_SIZE	1



/* Status read Command */
#define RFM12_READ_STATUS()                               rfm12_spi16_ret(0x0000)


uint8_t  rfm12_Device_present(void)
{
     return RFM12_READ_STATUS() != 0xFFFF;
}

/******************************************************************************
 * Initialize RF module for normal FSK-Mode ( HR20-like )
 *****************************************************************************/
void rfm12_Init(void)
{
	RFM12_READ_STATUS();

	// 1. Configuration Setting Command
	rfm12_spi16(
		RFM_CONFIG_EL           |
		RFM_CONFIG_EF           |
		RFM_CONFIG_BAND_433     |
		RFM_CONFIG_X_12_0pf  
	 );

	// 2. Power Management Command 
	//rfm12_spi16(
	//	 RFM_POWER_MANAGEMENT     // switch all off
	//	 );

	// 3. Frequency Setting Command
	rfm12_spi16(
		RFM_FREQUENCY            | 
		/**** Chng 056 ****/
		RFM_FREQ_433Band(FSK_CARRIER_FREQUENCY)
	 );

	// 4. Data Rate Command
#if ( RFM_BAUD_RATE == 19200 )
	rfm12_spi16(RFM_DATA_RATE_19200);
#else
	#error "This Baudrate has no predefined setting, pls add" 
#endif
//  RFM_SET_DATARATE produces another value than the predefined for 19200
//	rfm12_spi16(RFM_SET_DATARATE(RFM_BAUD_RATE));

	// 5. Receiver Control Command
	rfm12_spi16(
		RFM_RX_CONTROL_P20_VDI  | 
		RFM_RX_CONTROL_VDI_FAST |
		RFM_RX_CONTROL_BW(RFM_BAUD_RATE) |
		RFM_RX_CONTROL_GAIN_0   |
		RFM_RX_CONTROL_RSSI_85
	 );

	// 6. Data Filter Command
	rfm12_spi16(
		RFM_DATA_FILTER         |
		RFM_DATA_FILTER_AL      |
		RFM_DATA_FILTER_ML      |
		RFM_DATA_FILTER_DQD(3)             
	 );

	// 7. FIFO and Reset Mode Command
	rfm12_spi16(
		RFM_FIFO_IT(8) |
		RFM_FIFO_DR
	 );

	// 8. Receiver FIFO Read

	// 9. AFC Command
	rfm12_spi16(
		RFM_AFC_AUTO_VDI        |
		RFM_AFC_RANGE_LIMIT_7_8 |
		RFM_AFC_EN              |
		RFM_AFC_OE              |
		RFM_AFC_FI     
	 );

	// 10. TX Configuration Control Command
	rfm12_spi16(
		RFM_TX_CONTROL_MOD(RFM_BAUD_RATE) |
		RFM_TX_CONTROL_POW_0
	 );

	// 11. Transmitter Register Write Command

	// 12. Wake-Up Timer Command

	// 13. Low Duty-Cycle Command

	// 14. Low Battery Detector Command

	//rfm12_spi16(
	//	 RFM_LOW_BATT_DETECT |
	//	 3      // 2.2V + v * 0.1V
	//	 );

	// 15. Status Read Command
}

/*!
 *******************************************************************************
 * Switch off the RFM12 transceiver to the lowest possible mode
 ******************************************************************************/
void rfm12_OFF(void)
{
    RFM12_RXTX_OFF();
    RFMxx_SPI_DESELECT();
}

/*-----------------------------------------------------------------------------
 * The receive part of rfm driver
 *---------------------------------------------------------------------------*/
/*!
 *******************************************************************************
 * Neccessary to initialize the RFM12 receiver correctly
 * sequence found by trial and error
 ******************************************************************************/
static void rfm12_prepare_recv( void )
{
	rfm12_spi16(RFM_FIFO_IT(8) |               RFM_FIFO_DR);
	rfm12_spi16(RFM_FIFO_IT(8) | RFM_FIFO_FF | RFM_FIFO_DR);
}

/*!
 *******************************************************************************
 * Start the RFM12 receiver. 
 ******************************************************************************/
static inline __attribute__((always_inline))
void rfm12_do_receive(void)
{
        // The follwing sequnce will clear FFIT/RGIT RGUR/FFOV to avoid junk being received
        // by wrong status readout
        RFM12_WRITE(0xAA);
        (void)RFM12_READ_FIFO();
        RFM12_RX_ON();    //re-enable RX

        RFM12_INT_EN(); // enable RFM interrupt
}

/*!
 *******************************************************************************
 * RFM chip dependant implementation of Receive-routine
 ******************************************************************************/
void rfm12_ReceiveBody(void)
{	
    // DEBUG_PUTS("Recv:");
    // 
    
    rfm_CheckIdle();

    // do re-init only if receive after sleep/idle
    if ( ! bTxThenRx ) {
        RFM_Abort(0);
        rfm12_Init();
    }


    // Prepare Receiver for FSK-Mode
    rfm_basemode = rfm_basemode_rx;
    rfm_submode = rfm_submode_normal;
    UserByteCnt = ActBufIdx = 0;

    rfm12_prepare_recv();
    rfm12_do_receive();
}

/*-----------------------------------------------------------------------------
 * The transmit part of rfm driver
 *---------------------------------------------------------------------------*/
/*!
 *******************************************************************************
 * Start the RFM12 transmitter. 
 * The follwing code sequence has been found by trial and error
 * The transmitter will come up faster, if it is is prepared with
 * "rfm12_HeatUpTransmitter" shortly before transmission will start
 ******************************************************************************/
static void rfm12_start_transmit(void) {
        // The follwing sequnce will clear FFIT/RGIT RGUR/FFOV to avoid junk being received
        // by wrong status readout
        RFM12_WRITE(0xAA);
        (void)RFM12_READ_FIFO();

        RFM12_TX_ON();
        RFMxx_SPI_SELECT(); // set nSEL low: from this moment SDO indicate FFIT or RGIT
        RFM12_INT_EN(); // enable RFM interrupt
}

/*!
 *******************************************************************************
 * Enable crystal oscillator an synthesizer in RFM chip. According to manual
 * doing this prior to transmission will shorten the latency time when
 * transmitter is enabled
 ******************************************************************************/
void rfm12_HeatUpTransmitter(void) {
	RFM12_TX_ON_PRE();
}

/*!
 *******************************************************************************
 * The RFM chip specific part for transmission of data
 ******************************************************************************/
void rfm12_TransmitBody(void)
{
	// DEBUG_PUTS("Xmit:");
	// 
	
	rfm_CheckIdle();

	RFM_Abort(0);

	rfm12_Init();

	// Prepare Receiver for FSK-Mode
	rfm_basemode = rfm_basemode_tx;
	UserByteCnt = ActBufIdx = 0;

        // Start with Preamble
        rfm_submode  = rfm_submode_tx_preamble;
        rfm12_start_transmit();
}

/*iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 * Interrupt routines and interrupt handler
 *iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii*/


/*******************************************************************************
 * Internal transmit callback to transmit the neccessary RFM12 preamble bytes
 ******************************************************************************/
 static inline __attribute__((always_inline))
 uint16_t rfm_GetPreambleChar ( void ) {
	if ( ActBufIdx >= RFM_PREAMBLE_SIZE ) 
		return RFM_TX_NOMOREDATA;
	else {
		/* Preamble bytes do not count into BlockLen, do not increment UserByteCnt */
		return (uint16_t)*(rfm_header+ActBufIdx++);
	}
 }

/*******************************************************************************
 * Internal transmit callback to transfer the user bytes, either by calling the
 * user callback or by fetching from buffer
 * returns RFM_TX_NOMOREDATA when no more data for transmission available
 * If both callback for fetching char and buffer are specified, 
 * the callback has priority!
 ******************************************************************************/
static inline __attribute__((always_inline))
uint16_t rfm_GetUserChar ( void )
 {
	if ( UserByteCnt >= TxBufLen) 
		return RFM_TX_NOMOREDATA;

	if ( cbTxFetchByte ) {
		UserByteCnt++;
		return cbTxFetchByte();
	}

	if ( BufPtr ) {
		UserByteCnt++;
		return *(BufPtr+ActBufIdx++);
	}

	// in case of neither buffer nor callback are set:
	return RFM_TX_NOMOREDATA;

}

static inline __attribute__((always_inline))
uint16_t rfm_GetTrailerChar ( void )
{
	if ( ActBufIdx >= RFM_TRAILER_SIZE)
		return RFM_TX_NOMOREDATA;

	ActBufIdx++;
	return 0xcc;
}

 /************************************************************************************
 * Internal transmit callback for the whole transmit sequence, which consists of three
 * phases:
 * 1. Preamble bytes
 * 2. payload bytes
 * These phases are stepped thru by the variable "rfm_submode" and the corresponding
 * callback ( see above ) is called.
 * Finally, RFM_TX_NOMOREDATA will be returned
 ************************************************************************************/
static inline __attribute__((always_inline))
uint16_t  rfm_GetTransmitChar( void )
{
	uint16_t ret;
	switch ( rfm_submode )
	{
		case rfm_submode_tx_preamble:
			ret = rfm_GetPreambleChar();
			if ( ret != RFM_TX_NOMOREDATA ) return ret;
			rfm_submode = rfm_submode_tx_userdata;
			ActBufIdx = 0;
                        __attribute__ ((fallthrough));
			// no break here
		case rfm_submode_tx_userdata:
		    ret = rfm_GetUserChar();
			if ( ret != RFM_TX_NOMOREDATA ) return ret;
			rfm_submode = rfm_submode_tx_trailer;
			ActBufIdx = 0;
                        __attribute__ ((fallthrough));
			// No break here
		case rfm_submode_tx_trailer:
			ret = rfm_GetTrailerChar();
			if ( ret != RFM_TX_NOMOREDATA ) return ret;
			break;
		default:		
			ret = RFM_TX_NOMOREDATA;
	}
//	DBG_PUTC('T');DBG_HEXXX((uint8_t)ret);
	return ret;
}

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
            /* Store into buffer, if specified */
            if (BufPtr) {
                    *(BufPtr+ActBufIdx++) = rxchar;
            } 
    }

    /*  check for buffer full */
    if ( ++UserByteCnt >= RxBufLen ) rxcontinue = 0;

    return rxcontinue;
}


/*******************************************************************************
 * Pinchange Interupt for MISO pin
 * This routine is executed in exti - interrupt context, so keep it short! 
 ******************************************************************************/
void HandleFSKInterrupt_RFM12(uint16_t pin, uint16_t pinvalue, void *arg)
{
  UNUSED(pin); UNUSED(pinvalue); UNUSED(arg);

  register uint16_t work;

  // RFM module interupt
  RFM12_INT_DIS();
  while (RFM12_MISO_GET()) {
	switch ( rfm_basemode )
	{
		case rfm_basemode_tx:
			work = rfm_GetTransmitChar();
			if ( work != RFM_TX_NOMOREDATA ) {
                            RFM12_WRITE((uint8_t)work);
			} else {
                            RFM_SwitchOff();
                            rfm_basemode = rfm_basemode_txdone;
                            TaskNotify(TASK_RFM); // inform the rfm task about end of transmition
			    return;
			}
			break;
		case rfm_basemode_rx:
			/* The whole processing is done in "rfm_ReceiveChar", because stub will call it too in stub-mode */
			if ( !rfm_ReceiveChar(RFM12_READ_FIFO()) ) {
                            RFM_SwitchOff();
                            rfm_basemode = rfm_basemode_rxdone;
                            TaskNotify(TASK_RFM); // inform the rfm task about end of transmition
                            return;
			}
			break;
		default:
			DEBUG_PUTS("FSK-Intr: Unhandeled state");
			/* Switch off and basemode to idle */
			RFM_Abort(1); 
			return;

	 } // switch	
  }  // while 

  RFM12_INT_CLR();   // RHB Clear any pending interrupts due to  PinChanges during shift
  RFM12_INT_EN();  // enable RFM interrupt
}


/*ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
 o   The OOK-Stuff
 oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo*/
#ifdef USE_RFM_OOK
	/******************************************************************************
	 *
	 *****************************************************************************/
	void rfm12_OOK_init(void)
	{

		RFM_READ_STATUS();

		// 1. Configuration Setting Command
	/*
		;RFM12B - Configuration Command
		;el : Enable TX register = 0
		;ef : Enable RX FIFO buffer = 0
		;Select 433 MHz band
		;Select 12 pF crystal load capacitor
		8017
	*/
		rfm12_spi16(
			RFM_CONFIG_BAND_433     |
			RFM_CONFIG_X_12_0pf  
		 );

		// 2. Enable Receive chain
		/*
		;RFM12B - Power Management Command
		;er : Enable receiver chain = 1
		;ebb : Enable baseband circuit = 1
		;et : Enable PLL,PA, TX = 0
		;es : Enable synthesizer = 1
		;ex : Enable crystal oscillator = 1
		;eb : Enable low battery detector = 0
		;ew : Enable wake-up timer = 0
		;dc : Disable clock output = 0
		82D8
		*/
		// RFM12_RX_ON(); Activation of receiver is done later

		// 3. Frequency Setting Command
		/*
		;RFM12B - Frequency Command
		;Set center frequency to 433.92 MHz
		A608 --- A604 RHB
		*/
		rfm12_spi16(
			RFM_FREQUENCY            | 
			/**** Chng 056 ****/
			RFM_FREQ_433Band(config.ook_frq > 0 ? OOK_CARRIER_FREQUENCY1 : OOK_CARRIER_FREQUENCY0)
		);

		// 4. Data Rate Command
		// 4. Data Rate Command
		rfm12_spi16(RFM_DATA_RATE_9600);



		// 5. Receiver Control Command
		/*
		;RFM12B - Receiver Control Command
		;p16 : 0=INTin, 1=VDIout = 1
		;Receiver bandwidth = 67 kHz
		;LNA gain = 0 dB
		 **** Chng 056 ****
		;RSSI threshold = -91 dBm
		94C1 
		*/

		rfm12_spi16(
			RFM_RX_CONTROL_P20_VDI  | 
			RFM_RX_CONTROL_BW_67 	|
			RFM_RX_CONTROL_GAIN_0   |
			RFM_RX_CONTROL_RSSI_91
		 );

		// 6. Data Filter Command
		/*
		;RFM12B - Datafilter Command - undocumented code C220 instead of C228 makes the difference
		;al : Clock recovery auto lock control = 0
		;ml : Clock recovery lock control = 0
		;s : (0=digital, 1=analog filter) = 0
		;DQD threshold = 0
		C220
		*/
		rfm12_spi16(
			RFM_DATA_FILTER_UNDOC   |
			RFM_DATA_FILTER_DIG     |
			RFM_DATA_FILTER_DQD(0)             
		 );

		// 7. FIFO and Reset Mode Command
		/*
		;RFM12B - FIFO / Reset Mode Command
		;sp : Select single Byte Sync pattern = 0
		;al : Disable Sync pattern recognition = 0
		;ff : FIFO fill = 0
		;dr : Disable highly sensitive Reset = 0
		;FIFO IT level = 0
		CA00
		*/
		rfm12_spi16(
			RFM_FIFO
		 );

		// 8. Receiver FIFO Read

		// 9. AFC Command
		/*
		;RFM12B - AFC Command
		;AFC automatic mode = Run once
		;st : Strobe edge = 0
		;fi : Fine mode = 0
		;oe : Offset register enable = 1
		;en : Calculate offset = 1
		;Range limit = +3fres to -4fres
		;Max. Deviation = +7.5 kHz to -10 kHz
		C431
		*/
		rfm12_spi16( 
			RFM_AFC_RANGE_LIMIT_3_4 |
			RFM_AFC_EN            
		 );

		// 10. PLL Command
		/*
		;RFM12B - PLL Command
		;lpx : Enable low power crystal oscillator = 0
		;ddy : Enable phase detector delay = 0
		;ddit : Disable PLL loop dithering = 1
		;Select clock output slew rate control for 2.5 MHz or less
		;Select PLL bandwidth for max. 256 kbps
		CC67
		*/
		rfm12_spi16(
			RFM_PLL_OB_5_10			 |
			RFM_PLL_DDIT			 |
			RFM_PLL_BW0
		);

	}

	void rfm12_OOK_on(void)
	{
		// Prepare for OOK-Modus
		rfm12_OFF();
		RFM_OOK_init();
	
		// Enable RFM12 Receiver
		RFM12_RX_ON();
		RFMxx_SPI_DESELECT(); // RHB ***0001***

		// Enable Int0-Interrupte
                OOK_DATA_IRQ_CLEAR();
		OOK_DATA_IRQ_ENABLE();
	}		

	void rfm12_OOK_off(void)
	{
		// Stop RFM12 Receiver
		rfm12_OFF();

		// Disable Int0-Interrupte
		OOK_DATA_IRQ_DISABLE();
	}
#endif

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Debug helper functions
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/

#if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0
	
	#include "debug_helper.h"

	const char digits[] ="0123456789abcdef"; 

	const char e15[] = "FFIT/RGIT";
	const char e14[] = "POR";
	const char e13[] = "FFOV/RGUR";
	const char e12[] = "WKUP";
	const char e11[] = "EXT";
	const char e10[] = "LBD";
	const char e09[] = "FFEM";
	const char e08[] = "ATS/RSSI";
	const char e07[] = "DQD";
	const char e06[] = "CRL";
	const char e05[] = "ATGL";

	const char *const status_text[]  = {e15, e14, e13, e12, e11, e10, e09, e08, e07, e06,e05 };

	void rfm12_Dump_status( void )
	{
                uint16_t status = RFM12_READ_STATUS();
		DEBUG_PRINTF("RFM-status=%04x\n", status);
		uint16_t work;
		unsigned char i;
		for (work=0x8000,i=0;i<11;i++) {
		   if ( status & work ) {
			 DEBUG_PUTS(*(status_text+i));
			 DEBUG_PUTC(' ');
	       }
		   work /= 2;
		}
		DEBUG_PUTC( (status & RFM_STATUS_OFSS ? '-' : '+') );
		DEBUG_PUTC(digits[status & 0x0f] );
		CRLF();

	}
#endif /* #if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0 */

const RFM_DeviceType rfm12_driver = {
    /* ------ Presence check ------*/
    .rfmXX_Device_present = rfm12_Device_present,
    /* ------ FSK functions ------*/
    .rfmXX_OFF                  = rfm12_OFF,
    .rfmXX_Init                 = rfm12_Init,
    .rfmXX_ReceiveBody          = rfm12_ReceiveBody, 
    .rfmXX_TransmitBody         = rfm12_TransmitBody,
    .rfmXX_HeatUpTransmitter    = rfm12_HeatUpTransmitter,
    .HandleFSKInterrupt_RFMXX   = HandleFSKInterrupt_RFM12,
    /* ------ OOK functions ------*/
    #ifdef USE_RFM_OOK
        .rfmXX_OOK_init         = rfm12_OOK_Init,
        .rfmXX__OOK_on          = rfm12_OOK_On,
        .rfmXX__OOK_off)        = rfm12_OOK_Off.
    #endif
    /* ------ Debug ------*/    
    #if DEBUG_RFM_STATUS > 0 || DEBUG_MODE > 0
        .rfmXX_Dump_status      = rfm12_Dump_status,
    #endif
    .rfmID                      = RFM12_ID,
    .name                       = "RFM12",
};


#endif /* if USE_RFM12 > 0 */