/******************************************************************************
 * rfm_packets.h
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

#ifndef __RFM_PACKETS_H
#define __RFM_PACKETS_H

#include "config/config.h"
#include "rfm/rfm.h"
/* 
 * the maximum packet size correspondents to RFM69 FIFO size. 
 * There are listeners, that support ony receiving into buffer
 * As long as this is the case, this size should not be increased
 */
#define PKT_MAXSIZE			66	
/*
 * size of block to store integrity data. This block consists of 
 * two bytes with [<length byte>, <inverted length byte>]
 */
#define PKT_LENGTHSIZE		2	
/* The length block resides in front of packet */
#define PKT_LENGTHOFFSET	0

/* Resulting net useful packet size */
#define PKT_USEFUL_SIZE		(PKT_MAXSIZE-PKT_LENGTHSIZE)


/*!
 *******************************************************************************
 *  Status returned by any type of "complete"-callback
 ******************************************************************************/
 typedef enum {
	pkt_status_none=0,
	pkt_status_tx_ok=1,
	pkt_status_rx_ok=2,
	pkt_status_rx_badlength=3,
	pkt_status_rx_toolong=4,
} PKT_StatusType;

/* Callback type for OnComplete callbacks */
typedef void (*PKT_DoneCB) (PKT_StatusType );

/* Callback type to fetch a transmission character */
typedef uint16_t (*PKT_TxByteFetchCB) (void);

/* Callback type to signal reception of character */
typedef uint8_t  (*PKT_RxByteCB) (uint8_t);


/* pointer to application useable rx/tx buffer */
extern  uint8_t * const PKT_Userdata_ptr;

/* application level index to the default receive/transmit buffer */
/* not maintained/intialized by packet driver, ie user must       */
/* initialize properly before use!                                */
extern uint8_t PKT_Userdata_idx;

/* put on userdata byte at the next write position in the default receive/transmit buffer */
uint8_t PKT_PutUserByte(uint8_t byte);

/* Return the userdata length of last received packet */
uint8_t PKT_GetPacketLength ( void ) ;

void PKT_Receive( uint8_t *buf, uint8_t buflen, PKT_RxByteCB cbRxByte, PKT_DoneCB cbAllDone);
void PKT_ReceiveDefault(PKT_DoneCB cbAllDone);

uint8_t PKT_Transmit( uint8_t *buf, uint8_t buflen, PKT_TxByteFetchCB cbTxFetchByte, PKT_DoneCB cbAllDone);
uint8_t PKT_TransmitDefault(PKT_DoneCB cbAllDone);

uint8_t PKT_TxThenRx( uint8_t *buf, uint8_t  txbuflen, uint8_t rxbuflen, PKT_TxByteFetchCB cbTxFetchByte, RFM_DoneCB txDoneCB,  PKT_RxByteCB cbRxByte, PKT_DoneCB allDoneCB );
uint8_t PKT_TxThenRxDefault( RFM_DoneCB txDoneCB, PKT_DoneCB allDoneCB );

#endif /*  __RFM_PACKETS_H  */

