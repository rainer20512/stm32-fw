/******************************************************************************
 * rfm.c
 *
 * Created: 11.12.2019 14:06:25
 *  Author: rainer
 *
 * hardware device independent part of RFM code
 *
 ******************************************************************************/

#include "config/config.h"

#include "rfm/rfm_specific.h"
#include <stddef.h>

RFM_DeviceType *rfm=NULL;

#if USE_RFM12 > 0 || USE_RFM69 > 0

#include "rfm/rfm_spi_interface.h"
#include "rfm/rfm.h"
#include "eeprom.h"
#include "debug_helper.h"
#include "global_flags.h"
#include "system/status.h"
#include "system/periodic.h"
#include "wireless.h"
#include "com.h"
#include "timer.h"


#if USE_RFM_OOK > 0
    #include "sequencer/pulses.h"
#endif

/******************************************************************************
 * Both receiver and transmitter can be feeded by callback functions or by
 * arrays to write to/read from. 
 * Whenever both callback functions and buffer variables are specified, 
 * the callback will be used and the buffer ptr is ignored. It is ok to change
 * or remove callbacks during running operation. In this case, the buffers
 * are used to write to/read from for all following bytes. 
 * 
 * internal variables:
 *
 * - cbTxFetchByte - callback to get the next byte to transfer, must not be NULL
 * - cbRxByte      - callback when a byte is received
 * When the following two parameters are set, 
 * they must remain valid until transmission finishes
 *  - BufPtr      - Pointer to buffer to transmited
 *  - Buflen      - Length of buffer to be transmitted
 * The following two callbacks may be NULL
 * - cbOnComplete - Callback when operation is complete
 * - cbOnAbort    - Callback when transmission is aborted before completely sent
 * - ByteCount    - internal only: will count the number of bytes from begin of transmission
 * The following two items are used for Transmit, then Receive
 * - bTxThenRx    - 0 for normal receive, != 0 when Transmit, 
 * - cbTxDone     - Callback after Transmit, before switching to Receive
 *****************************************************************************/

RFM_TxByteFetchCB cbTxFetchByte;
RFM_RxByteCB cbRxByte;
uint8_t *BufPtr;
uint8_t TxBufLen;
uint8_t RxBufLen;
RFM_DoneCB cbOnComplete;

bool bTxThenRx;
RFM_DoneCB cbTxDone;
rfm_basemode_t rfm_basemode = rfm_basemode_idle;

uint8_t UserByteCnt;		/* internal total packet length counter used in transmitting and receiving data */

static uint8_t xmit_period=0;   /* downcounter for FSK sync period, will be decremented every minute 
                                 * and reinitialized on successful sync */


/*!
 *******************************************************************************
 * Switch RFMxx to "Off" stat and disable further interrupts
 * This routine is executed in interrupt context.
 * \note Although this routine resides in the hardware independet part of code,
 *       it contains a harware specific call
 ******************************************************************************/
void RFM_SwitchOff(void)
{
    if ( !rfm ) return;
    /* 
     * Disable MISO interrupts first. Otherwise echoing MOSI on the MISO line
     * Would generate sensless interrupts
     */
     switch(rfm->rfmID) {
#if USE_RFM12 > 0
        case RFM12_ID:
            RFM12_INT_DIS();
            break;
#endif
#if USE_RFM69 > 0
        case RFM69_ID:
            RFM69_INT_DIS();
            break;
#endif
        default:
            DEBUG_PRINTF("RFM_SwitchOff: Unknown RFM ID %d\n",rfm->rfmID);
            
     } 

     rfm->rfmXX_OFF();
}

/******************************************************************************
 * Abort any action and/or bring rfm chip to a defined state
 * \param bSleep  only relevant in stub mode: controls, whether
 *                remote can go to sleep after abort or does
 *                an deferred abort/sleep after transmit
 * \note the "SwitchOffLocal" seems to be redundant, but RFM_Abort may be called
 *       in abnormal situations without a prior SwitchOff. So leave it here, too.
 * \note When sleeping, remote has to be woken up before normal
 *       operation may be resumed
 *****************************************************************************/ 
void RFM_Abort(uint8_t bSleep)
{
    RFM_SwitchOff();
    ClearFlagBit(gflags, GFLAG_FSK_BIT);
    UNUSED(bSleep);
    rfm_basemode = rfm_basemode_idle;
}


/*-----------------------------------------------------------------------------
 * General Functions for setting buffers / Getting status
 *---------------------------------------------------------------------------*/
#if DEBUG_MODE > 0
    bool rfm_CheckIdle(void)
    {
        if ( rfm_basemode != rfm_basemode_idle ) {
            DEBUG_PUTS("ChkIdle: Not in Idle");
        }
        return rfm_basemode == rfm_basemode_idle;
    }
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * All following setters can safely called during an active receive
 * or transmit operation
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/ 

/*******************************************************************************
 * Set Buffer and length for transmission and/or reception stream.
 * buffer may also be set to NULL
 ******************************************************************************/
void RFM_SetBuffer ( uint8_t *buf, uint8_t maxtxlen, uint8_t maxrxlen )
{
    BufPtr	= buf;
    TxBufLen	= maxtxlen;
    RxBufLen	= maxrxlen;
}

/*******************************************************************************
 * Set the receive stream length, requires ( does not check ) an receive buffer
 * of sufficient size
 ******************************************************************************/
void RFM_SetRxLen( uint8_t maxrxlen )
{
    RxBufLen	= maxrxlen;
}

/*******************************************************************************
 * Set the final "operation complete" callback
 ******************************************************************************/
void RFM_SetDoneCB(RFM_DoneCB doneCB)
{
    cbOnComplete = doneCB;
}

/*******************************************************************************
 * Set the callback for delivering the received bytes in an receive operation
 ******************************************************************************/
void RFM_SetRxCB ( RFM_RxByteCB rxCB )
{
    cbRxByte	 = rxCB;
}

/*******************************************************************************
 * Set the callback for fetching the next user byte in an transmission
 ******************************************************************************/
void RFM_SetTxCB ( RFM_TxByteFetchCB txCB )
{
    cbTxFetchByte= txCB;
}

/*******************************************************************************
 * Set the callback for "tranmission done" in TxThenRx mode
 * if callback is NULL, we notify in bTxThenRx flag, that _no_ TxThenRx operation,
 * but just an Tx operation takes place
 ******************************************************************************/
void RFM_SetRxAfterTx(RFM_DoneCB cbTxD )
{
    cbTxDone		= cbTxD;
    bTxThenRx		= cbTxD != NULL;
}


/*!
 *******************************************************************************
 * Receive an arbitrary number of bytes and deliver them via callback
 * or put them into buffer. The reception is terminated when either the
 * callback returns 0 or the maximal buffer length is reached.
 *
 * \param buf    - ptr to buffer the received characters will be stored into
 *                 may be NULL, in this case maxlen parameter is ignored
 * \param maxlen - number of bytes to receive. Cancel reception when reached
 * \param rxCB   - notifier will be called with every received byte. If this
 *                 notifier will return 0, reception is cancelled
 *                 may be NULL
 * \param doneCB - Callback to be called when reception is terminated. 
 *                 may be NULL
 *
 * \note If both callback and buffer are specified, the callback is used and
 *       buffer/maxlen are ignored.
 * \note Either callback or buffer/maxlen must be not NULL
 * \note Buffer/maxlen and/or callback can be changed/set to NULL during an
 *       active reception
 ******************************************************************************/
void RFM_Receive(uint8_t *buf, uint8_t maxlen, RFM_RxByteCB rxCB, RFM_DoneCB doneCB )
{
    /* Reset bTxThenRx */
    RFM_SetRxAfterTx(NULL);

    /* Init the rest */
    RFM_SetRxCB (rxCB);
    RFM_SetBuffer(buf, 0, maxlen);
    RFM_SetDoneCB(doneCB);

    rfm->rfmXX_ReceiveBody();

    // Set FSK-Transceiver-running-bit
    SetFlagBit(gflags, GFLAG_FSK_BIT);
}

/*!
 *******************************************************************************
 * Send an arbitrary number of bytes via rfm12 transmitter
 * These bytes are either fetched from buffer or requested by callback.
 * Transmission ends if either "maxlen" bytes are send 
 * or callback returns "RFM_TX_NOMOREDATA"
 *
 * \param buf    - ptr to buffer the bytes to transmit will be read from
 *                 may be NULL, in this case maxlen parameter is ignored
 * \param maxlen - number of bytes to transmit. Cancel transmit when reached
 * \param TxByteFetchCB
 *			     - Callback to fetch the next byte to transmit. When callback
 *                 returns RFM_TX_NOMOREDATA, transmission is cancelled
 *                 may be NULL
 * \param doneCB - Callback to be called when reception is terminated. 
 *                 may be NULL
 *
 * \note If both callback and buffer are specified, the callback is used and
 *       buffer/maxlen are ignored.
 * \note Either callback or buffer/maxlen must be not NULL
 * \note Buffer/maxlen and/or callback can be changed/set to NULL during an
 *       active transmission
 ******************************************************************************/
void RFM_Transmit(uint8_t *buf, uint8_t maxlen, RFM_TxByteFetchCB txCB, RFM_DoneCB doneCB )
{
	RFM_SetBuffer(buf, maxlen, 0);
	RFM_SetTxCB(txCB);
	RFM_SetDoneCB(doneCB);
	RFM_SetRxAfterTx(NULL);

	rfm->rfmXX_TransmitBody();

	// Set FSK-Transceiver-running-bit
	SetFlagBit(gflags, GFLAG_FSK_BIT);

}


/*!
 *******************************************************************************
 * First transmit an arbitrary number of bytes, when done, txDoneCallback 
 * is called ( if not NULL) and a receive is started immediately
 *
 *                 may be NULL, in this case maxlen parameter is ignored
 * \param txDoneCB - Notifier called when transmission is done and before
 *                   reception starts. Can be used to re-set any parameters, eg
 * All other parameters are explained in the Transmit and receive routines 
 *
 * \note If both callback and buffer are specified, the callback is used and
 *       buffer/maxlen are ignored.
 * \note Either callback or buffer/maxlen must be not NULL
 * \note Buffer/maxlen and/or callback can be changed/set to NULL during an
 *       active transmission
 ******************************************************************************/
void RFM_TxThenRx(uint8_t *buf, uint8_t maxtxlen, uint8_t maxrxlen, RFM_TxByteFetchCB txCB, RFM_DoneCB txDoneCB,  RFM_RxByteCB cbRxByte, RFM_DoneCB allDoneCB )
{
	RFM_SetTxCB(txCB);
	RFM_SetBuffer(buf, maxtxlen, maxrxlen);
	RFM_SetRxAfterTx(txDoneCB);
	RFM_SetRxCB(cbRxByte);
	RFM_SetDoneCB(allDoneCB);

	rfm->rfmXX_TransmitBody();

	// Set FSK-Transceiver-running-bit
	SetFlagBit(gflags, GFLAG_FSK_BIT);
}

/******************************************************************************
 * test all compiled rfm drivers for presence of driver chip
 * use the driver with mist specific determination method first,
 * use that with the least specific method last ( RFM12 )
 * 
 * a ptr to correct driver is returned.
 * if no device is found, NULL will be returned
 *****************************************************************************/
static RFM_DeviceType *rfm_assign_driver(void)
{
    #if USE_RFM69 > 0
        if ( rfm69_driver.rfmXX_Device_present() )  return (RFM_DeviceType *)&rfm69_driver;
    #endif
    #if USE_RFM12 > 0
        if ( rfm12_driver.rfmXX_Device_present() )  return (RFM_DeviceType *)&rfm12_driver;
    #endif
    return NULL;

}

void RFM_PostInit(const HW_DeviceType *dev, void *arg)
{
    UNUSED(arg);

    /* Assign the selected SPI Device ( may be BB or HW SPI */
    SetSPIDevice(dev);

    /* Assign correct rfm chip driver by calling devices' "Device_present" method */
    rfm = rfm_assign_driver();
    if ( rfm ) {
        SetFskDataAvailableCB(rfm, rfm->HandleFSKInterrupt_RFMXX);
    } else {
        CTL_error |= ERR_RFM_INOP;
        SetFSKStatus(FSK_STATUS_NORFM,false);
    }

    #if USE_RFM_OOK > 0
        SetOokDataAvailableCB(HandleOOKInterrupt);
    #endif

}

void RFM_PreDeInit(const HW_DeviceType *dev)
{
    UNUSED(dev);
    if ( rfm ) SetFskDataAvailableCB( rfm, NULL );
    RFM_Abort(1);
    CTL_error |= ERR_RFM_INOP;
    SetFSKStatus(FSK_STATUS_NORFM,false);

    rfmSpi = NULL;
}

/******************************************************************************
 * Check RFM module for being installed. This is done by calling the device
 * specific function "rfmXX_device_present".
 * If no RFM module seems to be present, a global error is set and the device
 * is deinitialized to reduce power consumption 
 *****************************************************************************/
bool RFM_IsPresent(void)
{
  
  if ( !rfm ) return false;

  bool ret = rfm->rfmXX_Device_present();

  if ( ret ) {
    CTL_error &= ~ERR_RFM_INOP;
  } else {
    DEBUG_PUTS("RFM device not found");
    CTL_error |= ERR_RFM_INOP;
    SetFSKStatus(FSK_STATUS_NORFM,false);
  }

  return ret;

}

/*!
 *******************************************************************************
 * Task handler for RFM-Events (called by main loop )
 * ***C021*** 
 ******************************************************************************/
void task_handle_rfm(uint32_t arg)
{
    UNUSED(arg);
    switch ( rfm_basemode )
    {
        case rfm_basemode_txdone:
            RFM_Abort(!bTxThenRx);
            #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
                    dump_rfm_ringbuf();
            #endif
            /* Check for Tx then Rx */
            if ( bTxThenRx ) {
                // Notify via callback
                if(cbTxDone) cbTxDone();
                // keep all settings about buffers and callbacks and start receiving
                rfm->rfmXX_ReceiveBody();

                // Set FSK-Transceiver-running-bit
                SetFlagBit(gflags, GFLAG_FSK_BIT);
            } else {
                // No Rx after Tx: Done, notify via callback
                if ( cbOnComplete ) cbOnComplete();
            }
            break;
        case rfm_basemode_rxdone:
            RFM_Abort(1);
            #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
                    dump_rfm_ringbuf();
            #endif
            // Done, notify via callback
            if ( cbOnComplete ) cbOnComplete();
            break;
        default:				
            RFM_Abort(1);
            DEBUG_PUTS("Handle_RFM: Illegal base mode");
            #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
                    dump_rfm_ringbuf();
            #endif
    }		
}

/*******************************************************************************
 * OOK-Receiver has Prio 1, so  start FSK Receive or transmit only 
 * - if OOK-Receiver is not running
 * - and there is sufficient time to finish the required FSK-action
 * \note As at second 0 or 30 a time correction may occur, don't use neither 
 *       the Milliseconds nor the millisecond timer at this seconds!
 ******************************************************************************/
static void FSK_action(void)
{
  if (  ( 
      ( RTC_GetSecond() == config.RFM_devaddr && WL_BufPtr)  ||
      (
          (
              RTC_GetSecond()>30 
              && ( (RTC_GetSecond()&1) ? wl_force_addr1==config.RFM_devaddr : wl_force_addr2==config.RFM_devaddr )
          ) || (
              wl_force_addr1==0xff 
              && RTC_GetSecond() % 30 == config.RFM_devaddr 
              && ((wl_force_flags>>config.RFM_devaddr)&1)
          )
      )
    )
        && RFM_is_idle()
#ifdef TX18LISTENER
        && FreeTimeSlot() > 1 
#endif
    ) // collission protection: send only when the second counter is equal to it's own address.
       // and RFM is unused and enough time to send before OOK-Receiver starts
  {
    MsTimerSetAbs(  WLTIME_START, wirelessTimerEvent, WL_TIMER_FIRST );
  }

  if ( ( RTC_GetSecond() == 59 || RTC_GetSecond() == 29 ) 
         && RFM_is_idle() 
#ifdef TX18LISTENER
         && FreeTimeSlot() > 1 
#endif
     )
  {
      #if (WL_SKIP_SYNC)
      if (wl_skip_sync > 0) {
           wl_skip_sync--;
      } else 
      #endif
      {
        MsTimerSetAbs(  WLTIME_SYNC, wirelessTimerEvent, WL_TIMER_SYNC );
      }
  }
}

/*******************************************************************************
 * checks to be performed every second
 * - if FSK mode is on, check for neccessary FSK actions
 * - in initial OOK mode after time has synced, try to sync with OOK transmitter
 * \note As at second 0 or 30 a time correction may occur, don't use neither 
 *       the Milliseconds nor the millisecond timer at this seconds!
 ******************************************************************************/
static void rfm_every_second ( void * arg )
{
    UNUSED(arg);
    if ( config.FSK_mode==1 ) {
        // if FSK-Mode is activated
        // Start FSK Transceiver if possible
        if ( RFM_is_idle() && config.RFM_devaddr!=0 && time_sync_tmo>0 ) FSK_action();
    }
    #ifdef TX18LISTENER
            // If Time-synced and OOK-Receiver in initial state: trigger initial OOK-Receiving
            if ( FSK_status == FSK_STATUS_INSYNC && OOK_status == OOK_STATUS_WAITFORTSYNC ) {
                    SetOOKStatus(OOK_STATUS_WAITFORSYNC, false);
                    Start_OOK_Receiver(0);
            }
    #endif
}

/*******************************************************************************
 * checks to be performed at second 0
 * - check for fsk receiver still sync'ed
 ******************************************************************************/
static void rfm_at_0 ( void *arg )
{
    UNUSED(arg);

    // Only if FSK-Mode is activated
    DEBUG_PRINTF("XmitPeriod=%d\n", xmit_period);
    if ( config.FSK_mode==1 ) wirelesTimeSyncCheck();

    if ( xmit_period == 0 ) {
        // if FSK-Mode is activated, send routine data packet to master
        if ( config.FSK_mode==1 ) COM_print_debug(true);
        #if defined(UNIVERSAL) && USE_DUMPER > 0 && USE_DS18X20 > 0
            // Currently not working. DumpBlock is only able to dump one type of data 
            Dump_Block((abstemp+5)/10,(uint16_t)bat_average);
        #endif
        xmit_period=config.TransmitPeriod;
    } else {
        #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
            COM_print_debug(false);
        #endif
    }
    xmit_period--;
}

#include "encoding.h"
void task_init_rfm( void )
{
    encoder_init(); 
    EverySecond(rfm_every_second, (void *)0, "RFM every second");
    AtSecond(0, rfm_at_0, (void *)0, "RFM time sync check");
}
#endif /* if USE_RFM12 > 0 || USE_RFM69 > 0 */