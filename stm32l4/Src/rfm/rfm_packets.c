/******************************************************************************
 * rfm_packets.c
 *
 * Created: 20.09.2018 09:06:25
 *  Author: rainer
 *
 * transmit and receive of packet oriented data blocks, independent from the
 * underlying rfm chip. 
 * packets contain a minimal integrity block which consists
 * of [<length byte>, <inverted length byte>]
 *
 ******************************************************************************/
#include "config/config.h"

#include <stdio.h>

#include "rfm/rfm.h"
#include "rfm/rfm_packets.h"
#include "task/minitask.h"
#include "global_flags.h"
#include "debug_helper.h"

#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
	#include "rfm/rfm_ringbuffer.h"
#endif

/*!
 *******************************************************************************
 *  the internal receive or transmit state:
 *  will be either indicate transmission/reception of length block
 *  or transmission/reception of userdata block
 *  \note This enum is internal only.the external useable status is defined 
 *        in rfm_status_t
 ******************************************************************************/
typedef enum {
	pkt_packetmode_unknown=0,
	pkt_packetmode_tx_lengthblock=1,	// Normal transmit w/o receive afterwards
	pkt_packetmode_tx_userdata=2,		
	pkt_packetmode_txrx_lengthblock=3,  // transmit w receive afterwards
	pkt_packetmode_txrx_userdata=4,
	pkt_packetmode_rx_lengthblock=5,    // receive
	pkt_packetmode_rx_userdata=6,
} pkt_PacketModeType;

/******************************************************************************
 * internal variables
 * - cbUserTxFetchByte - temporary storage to store the user transmit callback 
 *                       as long as the intrininsic callback (for transmission of
 *                       length block) is used
 * - pkt_cbTxDone      - user callback to notfy about end of transmission in a
 *                       transmit/receive operation
 * - cbUserRxByteCB    - temporary storage to store the user receie callback 
 *                       as long as the intrininsic callback (for reception of
 *                       length block) is used
 * - pkt_cbUserDone	   - the users final callback
 * - pkt_mode          - internal variable to keep track of the current 
 *                       transceiver state
 * - pkt_status        - externally reported status after termination of a
 *                       transmit, receive or transmit+receive operation
 * - ByteCounter       - internally used to count transmitted/received bytes
 *****************************************************************************/
 
static PKT_TxByteFetchCB	pkt_cbUserTxByteFetch;
static RFM_DoneCB			pkt_cbTxDone;
static PKT_RxByteCB			pkt_cbUserRxByteCB;
static PKT_DoneCB			pkt_cbUserDone;
static pkt_PacketModeType	pkt_mode		= pkt_packetmode_unknown;
static PKT_StatusType		pkt_status;
static uint8_t				ByteCounter;
static uint8_t				pkt_MaxRxSize;
/* 
 * for convenience we define a transmit/receive buffer, that can be used by the application
 * Not in full, but only the part behind the length block. So an application should never
 * access the buffer directly, only via the function PKT_GetBuffer()
 */
 uint8_t pkt_buf[PKT_MAXSIZE];
 uint8_t * const PKT_Userdata_ptr = pkt_buf + PKT_LENGTHSIZE;
 uint8_t PKT_Userdata_idx;
 /*----------------------------------------------------------------------------
  - General helper functions
  ---------------------------------------------------------------------------*/

/*!
 *******************************************************************************
 *  Write one userdata byte at the position "PKT_Userdata_idx"
 *  and then increase PKT_Userdata_idx by one
 *  \returns the new value of PKT_Userdata_idx
 ******************************************************************************/
uint8_t PKT_PutUserByte(uint8_t byte)
{
	PKT_Userdata_ptr[PKT_Userdata_idx++] = byte;
	return PKT_Userdata_idx;
}

/*!
 *******************************************************************************
 *  compute the length block in the packet buffer for transmission
 ******************************************************************************/
static void pkt_SetPacketLength(uint8_t len)
 {
	uint8_t *ptr = pkt_buf;
	*(ptr++) = len;
	*ptr = ~len;
 }

/*!
 *******************************************************************************
 *  check the length block in the packet buffer for correctness
 * \returns != 0 if ok, 0 on failure
 ******************************************************************************/
static uint8_t pkt_CheckPacketLength(void)
 {
	return (pkt_buf[0] + pkt_buf[1]) == 0xff;
 }

/*!
 *******************************************************************************
 * Check buflen for being within size limits
 * and set mode to the passed mode
 *******************************************************************************/
static uint8_t pkt_CheckAndSetMode(pkt_PacketModeType mode, uint8_t buflen)
{
    /* check buflen for being in allowable range */
    if ( buflen > PKT_USEFUL_SIZE ) {
        DEBUG_PUTS("PKT_Check: packet too large!");
        return 0;
    }

    ByteCounter	= 0;
    pkt_mode	= mode;

    // Last byte of length byte counts to payload, so packet size is one byte less!
    pkt_SetPacketLength(buflen-1);

    return 1;
}

/*!
 *******************************************************************************
 * Save User callback temporarily, we need our own callback first 
 *******************************************************************************/
static void pkt_SaveCallbacks(PKT_TxByteFetchCB cbTxFetchByte, RFM_DoneCB txDoneCB,  PKT_RxByteCB cbRxByte, PKT_DoneCB allDoneCB)
{
    pkt_cbUserTxByteFetch	= cbTxFetchByte;
    pkt_cbTxDone			= txDoneCB;
    pkt_cbUserRxByteCB		= cbRxByte;
    pkt_cbUserDone			= allDoneCB;
}

/*!
 *******************************************************************************
 * Underlying layer reports "done". 
 * So, if callback to upper layer is given, inform him and return the Status
 * DO NOT CALL user callback in other places than here. Only set "pkt_status"
 * in erroneous situations!
 *******************************************************************************/
void pkt_RfmDone ( void )
{
    if ( pkt_cbUserDone ) pkt_cbUserDone(pkt_status);
}

/*----------------------------------------------------------------------------
- Receive functions
---------------------------------------------------------------------------*/

/*!
 *******************************************************************************
 *  Get the length of the last received data block or the transmit block that
 * is going to b transmitted. The length is stored at offset 0
 *  of the general receive/transmit buffer, even if user specified an own 
 *  receive buffer 
 *  
 *  \returns length of userdata portion 
 *  \note    Valid only after successful receive or before transmit
 ******************************************************************************/
uint8_t PKT_GetPacketLength ( void ) 
{
    return pkt_buf[0];
}
/*!
 *******************************************************************************
 * The internal callback for character reception during reception of length block
 * If the length block is invalid, the reception is aborted and the upper layer
 * will be informed by notifier ( by returning a faulty status )
 *******************************************************************************/
uint8_t pkt_LengthBlockChecker ( uint8_t rxchar )
{
    /* "Empty" packets, ie. length byte = 0, are not allowed */
    if ( ByteCounter == 0 && rxchar== 0 ) {
        pkt_status = pkt_status_rx_badlength;
        return 0;
    }

    pkt_buf[ByteCounter++] = rxchar;
 
    if (ByteCounter >= PKT_LENGTHSIZE ) {
        // reset byte counter and switch mode to receive userdata
        // first check integrity of length block
        if (!pkt_CheckPacketLength() ) {
            // if not valid, set return status and return 0 into lower level. 
            // This will terminate lower level reception. Don't call user callback from here!
            pkt_status = pkt_status_rx_badlength;
            return 0;
        }
        // check for length of received block within limits:
        // a) whether received size not greater than maximum size
        // b) if a return buffer size has been set: whether received size will fit into it.
        if (pkt_buf[0] > PKT_MAXSIZE || ( pkt_MaxRxSize > 0 && pkt_buf[0] > pkt_MaxRxSize ) ) {
            // if not valid, set return status and return 0 into lower level.
            // This will terminate lower level reception. Don't call user callback from here!
            pkt_status = pkt_status_rx_toolong;
            return 0;
        }

        /* 
         * if everything is fine, reset byte counter, limit the raw receiver to
         * the known packet size, switch mode to rx_userdata 
         * and switch Rx callback to user callback
         * Last Lengthblock byte counts to payload, so set bytecounter to 1
         */
        ByteCounter = 1;
        pkt_mode = pkt_packetmode_rx_userdata;
        // last byte of length block counts as payload, so subtract one
        RFM_SetRxLen(pkt_buf[0]+PKT_LENGTHSIZE-1);
        RFM_SetRxCB(pkt_cbUserRxByteCB);
    }

    return 1;
}

/******************************************************************************
 * Receive a rfm packet
 * 
 * \param buf		- buffer to store received data into      
 * \param buflen	- expected length of data / maximal length of buffer
 *                    When received packet length exceedes buflen, an error
 *                    status is returned by callback
 *                    Value of 0 means: No limitations ( use with caution ) 
 * \param cbRxByte  - Callback for every received byte
 * \param cbAllDone - Callback on termination or receive error
 * \note  either buf or cbRxByte must be specified!
 *****************************************************************************/
 void PKT_Receive( uint8_t *buf, uint8_t buflen, PKT_RxByteCB cbRxByte, PKT_DoneCB cbAllDone)
{
    /* 
     * Store the size of return buffer. It will be checked after reception of length info
     * for being sufficient in length
     */
    pkt_MaxRxSize = buflen;
    /*
     * set packet mode and reset Bytecounter 
     */
    ByteCounter	= 0;
    pkt_mode	= pkt_packetmode_rx_lengthblock;

    /*
     * Set preliminary return status. Will be changed in error conditions
     */
    pkt_status = pkt_status_rx_ok;
    pkt_SaveCallbacks(NULL, NULL, cbRxByte, cbAllDone);

    RFM_Receive(buf, buflen+PKT_LENGTHSIZE, pkt_LengthBlockChecker, pkt_RfmDone );
}

/******************************************************************************
 * Receive a rfm packet with a minimal parameter setup, 
 * ie only callback on receive termination
 * 
 * All other parameters are set to default values as follwing
 * - Byte receive Callback is NULL
 * - Receive buffer is set to the internal packet buffer
 * - buffer length is set to the maximum 
 *****************************************************************************/
void PKT_ReceiveDefault(PKT_DoneCB cbAllDone)
{
    PKT_Receive(PKT_Userdata_ptr, PKT_USEFUL_SIZE, NULL, cbAllDone);
}

/*----------------------------------------------------------------------------
- Transmit functions
---------------------------------------------------------------------------*/
/*!
 *******************************************************************************
 * internal callback for transmitting the packet length in front of userdata
 * When packet length has been transmitted, this routine will change the mode to
 * pkt_packetmode_tx_userdata and the the callback to the user specified callback
 ******************************************************************************/
uint16_t pkt_LengthTransmitter (void)
{
    uint8_t ret = pkt_buf[ByteCounter];

    if (++ByteCounter >= PKT_LENGTHSIZE ) {
        if ( pkt_mode == pkt_packetmode_tx_lengthblock )
            pkt_mode = pkt_packetmode_tx_userdata;
        else if ( pkt_mode == pkt_packetmode_txrx_lengthblock )
            pkt_mode = pkt_packetmode_txrx_userdata;
        else {
            DEBUG_PUTS("pkt_LengthXmitter: illegal state");
        }
        RFM_SetTxCB(pkt_cbUserTxByteFetch);
        ByteCounter = 0;
    }

    return (uint16_t)ret;
}

/*!
 *******************************************************************************
 * Internal callback for "Tx Done in Tx/Rx operation"
 * Switch internal state to start of receiving, reset byte counter and, 
 * if callback to upper layer is given, finally inform him
 *******************************************************************************/
void pkt_RfmTxDone(void)
{
    ByteCounter	= 0;
    pkt_mode	= pkt_packetmode_rx_lengthblock;
    if ( pkt_cbTxDone ) pkt_cbTxDone();
}

/*!
 *******************************************************************************
 * Start a transmission. Bytes are feed from Callback or buffer, whichever is 
 * specified. If both buffer and callback are given, callback will be used

 * \param bufptr 
 *     buffer to fetch the transmitted bytes from
 * \param buflen 
 *     number of byte to transmit. Must always be specified,
 *     independent from using buffer or callback
 * \param cbTxFetchByte 
 *     callback to fetch next byte for transmission
 * \param cbAllDone    
 *     callback after competion, transmission status will be returned
 *
 * \returns != 0 on success, 0 on failure   
 ******************************************************************************/
 uint8_t PKT_Transmit( uint8_t *buf, uint8_t buflen, PKT_TxByteFetchCB cbTxFetchByte, PKT_DoneCB cbAllDone)
{
    uint8_t rawTxSize = buflen+PKT_LENGTHSIZE;
    if ( !pkt_CheckAndSetMode( pkt_packetmode_tx_lengthblock, rawTxSize) ) return 0;
    pkt_SaveCallbacks(cbTxFetchByte, NULL, NULL, cbAllDone);

    /*
     * Set preliminary return status. Will be changed in error conditions
     */
    pkt_status = pkt_status_tx_ok;
    
    RFM_Transmit(buf, rawTxSize, pkt_LengthTransmitter, pkt_RfmDone);

    return 1;
}
/*!
 *******************************************************************************
 * Start a transmission with all possible parameters set to default values:
 * specified. If both buffer and callback are given, callback will be used

 * - Used buffer is this modules user buffer 
 * - buffer length is the current value of "PKT_Userdata_idx"
 * - no TxFetchCallback
 *  \param cbAllDone    
 *     callback after competion, transmission status will be returned
 *
 * \returns != 0 on success, 0 on failure   
 ******************************************************************************/
uint8_t PKT_TransmitDefault( PKT_DoneCB cbAllDone)
{
    return PKT_Transmit(PKT_Userdata_ptr, PKT_Userdata_idx, NULL, cbAllDone );
}


/*!
 *******************************************************************************
 * Start a transmission, followed by a receive 
 * Bytes to transmit are feed from Callback or buffer, whichever is specified. 
 * also bytes to receive are put to callback or buffer, , whichever is specified. 
 * If both buffer and callback are given, callback will be used
 *
 * \param bufptr 
 *     buffer to fetch the transmitted bytes from 
       _AND_ to write the received bytes into
 * \param txbuflen 
 *     number of byte to transmit. Must always be specified,
 *     independent from using buffer or callback
 * \param rxbuflen
 *     maximum number of bytes to receive. Must always be specified,
 *     independent from using buffer or callback
 * \param cbTxFetchByte 
 *     callback to fetch next byte for transmission
 * \param txDoneCB
 *     callback to notify upper layer, that transmission part is done
 * \param
 *     cbRxByte callback to notify upper layer about byte reception 
 * \param cbAllDone    
 *     callback after competion, transmission status will be returned
 *
 * \returns != 0 on success, 0 on failure   
 ******************************************************************************/
uint8_t PKT_TxThenRx( uint8_t *buf, uint8_t  txbuflen, uint8_t rxbuflen, PKT_TxByteFetchCB cbTxFetchByte, RFM_DoneCB txDoneCB,  PKT_RxByteCB cbRxByte, PKT_DoneCB allDoneCB )
{
    uint8_t rawTxSize = txbuflen+PKT_LENGTHSIZE;

    if ( !pkt_CheckAndSetMode( pkt_packetmode_txrx_lengthblock, rawTxSize) ) return 0;
    pkt_SaveCallbacks(cbTxFetchByte, txDoneCB, cbRxByte, allDoneCB);

    RFM_TxThenRx(buf,  rawTxSize, rxbuflen+PKT_LENGTHSIZE, pkt_LengthTransmitter, pkt_RfmTxDone, pkt_LengthBlockChecker, pkt_RfmDone );	

    return 1;
}


/******************************************************************************
 * Transmit and then receive a rfm packet with a minimal parameter setup, 
 * ie only callback on transmit and receive termination
 * 
 * All other parameters are set to default values as follwing
 * - Byte receive Callback is NULL
 * - Byte transmit callback is NULL
 * - Receive buffer is set to the internal packet buffer
 * - buffer length is set to "PKT_Userdata_idx" on transmit
 *   and to maximum length on receive
 *****************************************************************************/
uint8_t PKT_TxThenRxDefault( RFM_DoneCB txDoneCB, PKT_DoneCB allDoneCB )
{
    return PKT_TxThenRx(PKT_Userdata_ptr, PKT_Userdata_idx, PKT_USEFUL_SIZE, NULL, txDoneCB, NULL, allDoneCB );
}
