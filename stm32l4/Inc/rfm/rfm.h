/******************************************************************************
 * rfm.h
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * Select RFM chip dependent from project settings
 * currently supported:
 *  - rfm12b
 *  - rfm69(h)cw 
 *
 ******************************************************************************/

#ifndef __RFM_H
#define __RFM_H

#include "config/config.h"
#include "dev/spi.h"
#include "rfm/rfm_specific.h"

#if defined(USE_RFM12)
	#include "rfm/rfm12.h"
#elif defined(USE_RFM69)
	#include "rfm/rfm69.h"
#else
	#error "No RFM chip driver"
#endif


/* Macro to check presence/operability of RFM   module */
#define RFM_OPERABLE()     ( !(CTL_error & ERR_RFM_INOP) )
#define RFM_TX_NOMOREDATA	0x0100

/* Callback type for OnComplete, OnAbort */
typedef void (*RFM_DoneCB) (void);

/* Callback type to fetch a transmission character */
typedef uint16_t (*RFM_TxByteFetchCB) (void);

/* Callback type to signal reception of character */
typedef uint8_t  (*RFM_RxByteCB) (uint8_t);

/* Different operation status of RFM chip */
typedef enum {
	rfm_basemode_idle=0,
	rfm_basemode_tx=1,
	rfm_basemode_rx=2,
	rfm_basemode_rxdone=3,
	rfm_basemode_txdone=4,
	rfm_basemode_txook=5,			// RHB ToDo: neccessary? can be combined w rfm_basemode_tx, rfm_basemode_rx ?
	rfm_basemode_rxook=6,
	rfm_basemode_txookdone=7,		
	rfm_basemode_rxookdone=8,
} rfm_basemode_t;

/* Public Functions to be used by higher system levels ----------------------*/

bool RFM_IsPresent  (void);
void RFM_SetBuffer  ( uint8_t *buf, uint8_t maxtxlen, uint8_t maxrxlen );
void RFM_SetRxLen   ( uint8_t maxrxlen );
void RFM_SetDoneCB  ( RFM_DoneCB doneCB );
void RFM_SetRxCB    ( RFM_RxByteCB rxCB );
void RFM_SetTxCB    ( RFM_TxByteFetchCB txCB );
void RFM_Receive    ( uint8_t *buf, uint8_t maxlen, RFM_RxByteCB rxCB, RFM_DoneCB doneCB );
void RFM_Transmit   ( uint8_t *buf, uint8_t maxlen, RFM_TxByteFetchCB txCB, RFM_DoneCB doneCB );
void RFM_TxThenRx   ( uint8_t *buf, uint8_t maxtxlen, uint8_t maxrxlen, RFM_TxByteFetchCB txCB, RFM_DoneCB txDoneCB,  RFM_RxByteCB cbRxByte, RFM_DoneCB allDoneCB );

void task_handle_rfm(uint32_t);
void task_init_rfm( void );

#ifdef USE_RFM_OOK
	void RFM_OOK_init(void);
	void RFM_OOK_on(void);
	void RFM_OOK_off(void);
#endif

/* Private Functions to be used by lower system levels -----------------------*/
void RFM_SwitchOff(void);
void RFM_Abort(uint8_t nAction);
#if DEBUG_MODE > 0
    bool rfm_CheckIdle(void);
#else
    #define rfm_CheckIdle()
#endif

/* Private Variables to be used by lower system levels -----------------------*/
extern RFM_TxByteFetchCB cbTxFetchByte;
extern RFM_RxByteCB cbRxByte;
extern uint8_t *BufPtr;
extern uint8_t TxBufLen;
extern uint8_t RxBufLen;
extern RFM_DoneCB cbOnComplete;

extern bool bTxThenRx;
extern RFM_DoneCB cbTxDone;
extern rfm_basemode_t rfm_basemode;
extern uint8_t UserByteCnt;


/* Device PostInit and PreDeInit Funcions -----------.-----------------------*/
#include "dev/hw_device.h"
void RFM_PostInit(const HW_DeviceType *dev, void *arg);
void RFM_PreDeInit(const HW_DeviceType *dev);


#endif /* ifndef __RFM_H */