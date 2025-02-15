/**
  ******************************************************************************
  * @file    wireless.c
  * @author  Rainer 
  * @brief   Handling the wireless send and receive stuff
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup Wireless
  * @{
  */
/* Includes ------------------------------------------------------------------*/

#include "config/config.h"
#include "debug_helper.h"
#include "error.h"
#include "eeprom.h"
#include "system/status.h"
#include "system/util.h"
#include "dev/spi.h"
#include "ui/lcd.h"
#include "rfm/rfm_packets.h"
#include "rtc.h"
#include "com.h"
#include "timer.h"
#include "wireless.h"
#include "global_flags.h"
#include "encoding.h"
#if USE_DISPLAY > 0
    #include "ui/lcd_status.h"
#endif
#if USE_RFM_OOK > 0
    #include "sequencer/analyzer.h"
#endif

#include <string.h>

#if USE_RFM12 > 0 || USE_RFM69 > 0

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* First two bytes in framebuffer data area are reserved for Device Addr [0] and Flags [1] */
#define WLESS_PREFIX_DEVADDR            0
#define WLESS_PREFIX_FLAGS		1
#define WLESS_PREFIX_SIZE 		2

#define WLESS_TIMEINFO_SIZE		4     /* Time in SYNC Packets has size 4 */

#define RFM_FLAG_MASTER_SYNC            0x80
#define RFM_FLAG_SLAVE_ADDR		0x40
#define RFM_FLAG_SLAVE_FLAGS            0x20

/* Also need space for the CMAC Bytes */
/***C007*** enable RFM69 useage: RFM_FRAME max is the maximum payload size, it is determined
 * #define WIRELESS_BUF_MAX (RFM_FRAME_MAX - WLESS_PREFIX_SIZE - CMAC_BLOCK_SIZE)
 * by the RFM69h fifo size. The useable payload is reduced by Size Block and CMAC-Block
 **************************************************************************************/
#define WIRELESS_BUF_MAX (PKT_USEFUL_SIZE - WLESS_PREFIX_SIZE - CMAC_BLOCK_SIZE)

      
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* 
 * wireless buffer structure:
 * - x bytes plain data
 * all other things like DevAddr, Falgs, Size and CMAC bytes are added in framebuf, not here
 */
static uint8_t wireless_framebuf[WIRELESS_BUF_MAX]; 
static uint8_t wTmrTmo;                           /* TimerID of timeout timer */
static int8_t waitmin = WAITMIN_INITIAL;
static wirelessTimerCase_t wirelessTimerCase=WL_TIMER_NONE;

/* Exported variables --------------------------------------------------------*/
/* public pointer to the actual wireless buffer write position */
/* a value of 0 means "Buffer is empty"                        */
uint32_t WL_BufPtr=0;

uint8_t wl_force_addr1;
uint8_t wl_force_addr2;
uint32_t wl_force_flags;
int8_t time_sync_tmo=0;

#if (WL_SKIP_SYNC)
	uint8_t wl_skip_sync=0;
#endif

/* Forward declarations -------------------------------------------------------*/
static void wirelessSendPacket(uint8_t *buf, uint8_t bufsize);
void wirelessReceivePacket(PKT_StatusType rxStatus);
static void wirelessSetMasterTime(uint8_t *test);



/*!
 *******************************************************************************
 *  wireless send Done
 ******************************************************************************/
void wirelessSendDone(void) {
    COM_print_time('x', false);

    #ifdef ALARM
      // check for previous packet having been an alarm packet
      // if so: do nothing.
		if ( wirelessTimerCase == WL_TIMER_ALARM )  {
			return;
		}
    #endif	

	// wireless buffer can be reused
    WL_BufPtr=0;
    // Listen for answer from Master ( or Slave )
    COM_print_time('L', false);
	/***C030*** rfm_begin_receive(RECVMODE_AFTERSEND); */

    wTmrTmo = MsTimerSetRel(WLTIME_TIMEOUT, 0, wirelessTimerEvent, WL_TIMER_RX_TMO);    
}




void wirelessTimerEvent(uint32_t action )
{
  /* RHB added: If RFM is inoperabel, do nothing */
  wirelessTimerCase = WL_TIMER_NONE;
  if ( CTL_error & ERR_RFM_INOP ) return;

  /* Check for RFM still operable */
  if ( wirelessTimerCase == WL_TIMER_SYNC && !RFM_IsPresent() ) return;

  switch ( action ) {
      #ifdef ALARM
        case WL_TIMER_ALARM:
            // Send alarm packet, after that do not wait for answer from master
            Finalize_Alarm_Packet();
            wirelessTimerCase = WL_TIMER_ALARM;
            wirelessSendPacket(alarmbuf, alarm_buf_idx, WL_PACKET_ALARM);
            Reset_Alarm_Buf_Ptr();
            break;
      #endif
    case WL_TIMER_FIRST:
    case WL_TIMER_NEXT:
        // COM_print_time in wirelessSendPacket(), so not here
        wirelessSendPacket(wireless_framebuf, WL_BufPtr );
        break;
    case WL_TIMER_SYNC:
        wTmrTmo =  MsTimerSetRel ( WLTIME_SYNC_TIMEOUT, 0, wirelessTimerEvent, WL_TIMER_RX_TMO );
        // Prepare to receive a sync packet
        COM_print_time('Y', true);
        RFM_Abort(0);
        wl_force_addr1=0;
        wl_force_addr2=0;
        missed_syncs++;	
        PKT_ReceiveDefault(wirelessReceivePacket);
        break;
    case WL_TIMER_RX_TMO:
        COM_print_time('w', true);
	RFM_Abort(1);
        break;
    default:
        COM_print_time('?', true);
  } // switch
}

/*!
 *******************************************************************************
 *  wireless send data packet
 ******************************************************************************/
static void wirelessSendPacket(uint8_t *buf, uint8_t bufsize) 
{

    COM_print_time('S', false);

    rfm->rfmXX_HeatUpTransmitter();

    PKT_Userdata_idx = 0;
	
    // Changed RHB: Within Data Packets, the packet counter is also included
    // *** Chng 050 *** //
    PKT_PutUserByte(config.RFM_devaddr | rtc.pkt_cnt << PACKET_COUNTER_SHIFT);

    // set flags
    PKT_PutUserByte(0x00);

    // copy content
    memcpy(PKT_Userdata_ptr+PKT_Userdata_idx, buf, bufsize);
    
    // adjust size Information
    PKT_Userdata_idx += bufsize;

    #if ( DEBUG_DUMP_RFM > 0 )
        DEBUG_PRINTF("Send data:");
        COM_dump_packet(PKT_Userdata_ptr, PKT_Userdata_idx, false);
    #endif

    encrypt_decrypt (PKT_Userdata_ptr+WLESS_PREFIX_SIZE, PKT_Userdata_idx-WLESS_PREFIX_SIZE );
    cmac_calc(PKT_Userdata_ptr, PKT_Userdata_idx,(uint8_t*)&rtc,false);

    PKT_Userdata_idx += CMAC_BLOCK_SIZE;
    rtc.pkt_cnt++;

    if (!PKT_TxThenRxDefault(wirelessSendDone, wirelessReceivePacket) ) 
    {
            WL_BufPtr = 0;
            DEBUG_PUTS("Wireless packet dropped!");
    }
}





static bool wirelessHandleSync(uint8_t *data_ptr, uint8_t data_size)
{


    uint32_t mac_ok=cmac_calc(data_ptr, data_size-CMAC_BLOCK_SIZE, NULL, true);


    #if ( DEBUG_DUMP_RFM > 0 )
        DEBUG_PRINTF("Received Sync:");
        COM_dump_packet(data_ptr, data_size, mac_ok);
    #endif



    if (mac_ok) {
        MsTimerDelete(wTmrTmo);			

        if (*(data_ptr+WLESS_PREFIX_FLAGS )& RFM_FLAG_SLAVE_ADDR) {
            wl_force_addr1=*(data_ptr + WLESS_PREFIX_SIZE + WLESS_TIMEINFO_SIZE);
            wl_force_addr2=*(data_ptr + WLESS_PREFIX_SIZE + WLESS_TIMEINFO_SIZE + 1);
        } else if (*(data_ptr+WLESS_PREFIX_FLAGS)& RFM_FLAG_SLAVE_FLAGS) {
            wl_force_addr1=0xff;
            wl_force_addr2=0xff;
            memcpy(&wl_force_flags,(data_ptr + WLESS_PREFIX_SIZE + WLESS_TIMEINFO_SIZE),4);
        } else {
            #if (WL_SKIP_SYNC)
                 wl_skip_sync=WL_SKIP_SYNC;
            #endif
        }
        wirelessSetMasterTime(data_ptr);
    } else {
        // Sync packet, mac not ok
        #if ( DEBUG_DUMP_RFM > 0 )
            DEBUG_PUTS("MAC check failed on Sync");
        #endif
    }
    return mac_ok;
}

static void wirelessSetMasterTime(uint8_t *test)
{
    // Restore out_of_sync countdown and clear Out-of-sync-error
    time_sync_tmo=config.timeout;
    waitmin = WAITMIN_INITIAL;
    missed_syncs--; 	/*** Chng 052 ***/
    CTL_error &= ~ERR_RFM_SYNC;

    uint8_t old_subsec = RTC_GetS256();

    #if USE_DISPLAY > 0
        uint8_t old_hh = RTC_GetHour();
        uint8_t old_mm = RTC_GetMinute();
    #endif

    uint8_t YY = *(test+2);
    uint8_t MM = *(test+3)>>4;
    uint8_t DD = (*(test+4)>>5)+((*(test+3)<<3)&0x18);
    uint8_t hh = *(test+4)&0x1f;
    uint8_t mm = *(test+5)>>1;
    uint8_t ss = *(test+5)&1?29:59;
    uint8_t subsec = TIMERUNIT_TO_SUBS256(WLTIME_SYNC_AT)+(*(test+5)&1?2:6); 

    #if DEBUG_MODE > 0 
        /* Store current seconds, they may be altered by RTC_SetDateTime */
        uint8_t secs = rtc.ss;
    #endif
    /* set new time */
    RTC_SetDateTime(DD, MM, YY, hh, mm, ss, subsec);

    SetFSKStatus(FSK_STATUS_INSYNC, false);
 
    #if USE_DISPLAY > 0
        if ( old_hh != hh || old_mm != mm ) {
            LCD_DisplayStatus(LCD_STATUS_TIME);
        }
    #endif

    int16_t *avg = (*(test+5)&1?&avg_skew_30:&avg_skew_00);
    /**** Common Chng010 ****/
    *avg = average3(*avg, (int16_t)(old_subsec - RTC_GetS256()) << 8 );
    #if ( DEBUG_MODE > 0 )
        if ( console_debuglevel > 1 ) {
            DEBUG_PRINTF("Time sync'd @ %02d.%03d (old=%03d)",secs, subsec, old_subsec);
            DEBUG_PUTS("");
        }
    #elif DEBUG_PRINT_ADDITIONAL_TIMESTAMPS > 0
        DEBUG_PRINTF("Sync@ %03d (old=%03d)",RTC_GetS256(), old_subsec);
        DEBUG_PUTS("");
    #endif

}



static bool wirelessHandleData(uint8_t *data_ptr, uint8_t data_size)
{
    // We received a data packet:
    // Temporarily Change my own packet counter to ensure the mac is regarded as correct
    uint8_t pkt_cnt_save = rtc.pkt_cnt;

    // *** Chng 050 *** //
    rtc.pkt_cnt = *(data_ptr) >> PACKET_COUNTER_SHIFT;
    #if ( DEBUG_DUMP_RFM > 0 )
        DEBUG_PRINTF("Received data:");
        COM_dump_packet(data_ptr, data_size,false);
    #endif
    uint32_t mac_ok = cmac_calc(data_ptr,data_size-CMAC_BLOCK_SIZE,(uint8_t*)&rtc,true);
    encrypt_decrypt (data_ptr+WLESS_PREFIX_SIZE,data_size-WLESS_PREFIX_SIZE);
    #if ( DEBUG_DUMP_RFM > 0 )
        if (!mac_ok ) DEBUG_PUTS("Mac check failed\r\n");
    #endif

    // Change 0001: if-clause added
    if ( mac_ok ) {
        // Mask out the packet counter bits
        // *** Chng 050 *** // 
        *(data_ptr) &= PACKET_COUNTER_MASK;
        // Use my own Packet Counter again, do not increment here, is done in "wireless_send_packet"
        rtc.pkt_cnt =  pkt_cnt_save;

        // Prepare for sending the next data packet
		WL_BufPtr=0;
    }

    if (mac_ok && (*data_ptr==0)) { // Accept commands from master only
        // Timeout Deaktivieren
        MsTimerDelete(wTmrTmo);			
        if ( data_size <= CMAC_BLOCK_SIZE+WLESS_PREFIX_SIZE ) { 
           // empty packet don't need reply 
			/***C030 *** rfm_mode = rfmmode_stop; */
			return true;
        } 
        // !! hack
        // move input to top of buffer, beginning of buffer will be used for output
        // When Copying, omit the 4 mac bytes at the end and the two flag bytes at the beginning
        int8_t i;
		uint8_t j=PKT_USEFUL_SIZE-1;
        for (i=data_size-CMAC_BLOCK_SIZE;i>=WLESS_PREFIX_SIZE;i--) {
            *(data_ptr+j--) = *(data_ptr+i);
        }
        // COM_wireless_command_parse will write to wireless_buf
        // and is copied to framebuffer by wirelessSendPacket
        // Next Transmission is started via timer
        COM_wireless_command_parse(data_ptr+(++j), data_size-WLESS_PREFIX_SIZE-CMAC_BLOCK_SIZE);
        MsTimerSetRel(WLTIME_NEXT,0, wirelessTimerEvent, WL_TIMER_NEXT);
        return true;
    } // if (mac_ok && (*data_ptr==0))
    
    return false;

}

/*!
 *******************************************************************************
 * Callback after complete reception of an rfm packet:
 * check intrgrity, check type, ananlyze content and handle
 * according to content
 ******************************************************************************/
void wirelessReceivePacket(PKT_StatusType rxStatus) {


    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
          COM_print_time('R', true);
    #endif
    // Irqs are disabled already by interrupt-service routine
    // status maybe rfmmode_rx_ok or rfmmode_rx_badcrc or rfmmode_rx_toolong ( ***C009*** )

    // Last byte of lengthblock counts to payload, so pure data size is returned length - 1       
	uint8_t data_size = PKT_GetPacketLength()-1;

    // status is one of pkt_status_rx_okpkt_status_rx_badlength, pkt_status_rx_toolong, pkt_status_rx_buff_tooshort 
    if ( rxStatus ==pkt_status_rx_badlength || rxStatus == pkt_status_rx_toolong ) { 
        #if ( DEBUG_DUMP_RFM > 0 ) 
            switch (rxStatus ) {
                case pkt_status_rx_badlength:
                    DEBUG_PUTS("Bad Length Info");
                    break;
                case pkt_status_rx_toolong:
                    DEBUG_PRINTF("Pckt exceedes buf size, len=%d\n",PKT_GetPacketLength());
                break;
                default:
                    DEBUG_PRINTF("Unknown rxStatus=%d\n",rxStatus);
                break;
            }
        #endif
    } else {		
		/* 
		 * correct packet: handle it, if handler suceedes, we are done 
		 * otherwise, continue listening 
		 */				
        if ( *(PKT_Userdata_ptr+WLESS_PREFIX_FLAGS) & RFM_FLAG_MASTER_SYNC) {
			if (wirelessHandleSync(PKT_Userdata_ptr, data_size)) return;
        } else if ( time_sync_tmo > 0 ) {
            /* use data packets only if sync'ed ! */
			if ( wirelessHandleData(PKT_Userdata_ptr, data_size) ) return;
        }

    }

    // The received packet did not contain valid data, so listen again
	PKT_ReceiveDefault(wirelessReceivePacket);
}

/*!
 *******************************************************************************
 *  wireless Check time synchronization
 *  time_sync_tmo is restored with every succesful sync
 *  and parallel decremented every minute
 *  When reaching 0, we assume that synchronisation ist lost and try to resync
 *  (void:)After 5 minutes of unsuccessful resync attempts we will wait for 25 min before we try again
 *  After 1 minutes of unsuccessful resync attempts we will wait for 10 min before we try again
 ******************************************************************************/
void wirelesTimeSyncCheck(void) 
{
    /* If RFM is inoperabel, do nothing */
    if ( CTL_error & ERR_RFM_INOP ) return;

    #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
        COM_print_time('W', true);
    #endif
    
    time_sync_tmo--;
    if (time_sync_tmo<=0) {
        // RHB Changed due to C-Command     if ((time_sync_tmo>=0) || (time_sync_tmo<-30)) {
        if ((time_sync_tmo>=-1) || (time_sync_tmo < waitmin)) {
            // First phase of out of sync: Try to resync
            if ( time_sync_tmo < waitmin ) {
                // Increase the waittime for next out of sync interval:
                // Next wait interval <WAITMIN_DELTA> min longer
                // maximum wait time until next sync try is 120 min
                // ( cannot be longer due to 8 bit signed int )
                if ( waitmin >= -125+WAITMIN_DELTA ) waitmin -= WAITMIN_DELTA;
            }
            time_sync_tmo=-1;	// RHB was time_sync_tmo=0;
            #if defined(TX18LISTENER) 
                /* 
                 * When trying to resync with Master, stop OOK-Receiver, delete all OOK-Timers and set that receiver 
                 * to initial state. This is the only case that FSK-Receiver has a higher prio than OOK-Receiver
                 */
                if ( OOK_status != OOK_STATUS_INHIBITED ) Reset_OOK_Receiver();
            #endif
            // Added for Debug purposes
            #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
                COM_print_time('y', true);
            #endif
            SetFSKStatus(FSK_STATUS_WAITFORSYNC,false);
	    PKT_ReceiveDefault(wirelessReceivePacket);
        } else if (time_sync_tmo<= -1 - WL_RESYNC_INTERVAL ) {
            // Second phase of out of sync: switch off RFM for <waitmin> minutes
	    RFM_Abort(1);		// turn everything off
            CTL_error |= ERR_RFM_SYNC;
            SetFSKStatus(FSK_STATUS_TIMEOUTWAIT, false);
            #if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
                COM_print_time('z', true);
            #endif
        } 
    }
}

/*!
 *******************************************************************************
 *  wireless put one byte into buffer
 ******************************************************************************/
void wireless_putchar(uint8_t b) {
	if (WL_BufPtr<WIRELESS_BUF_MAX) {
	    wireless_framebuf[WL_BufPtr++] = b;
	}
} 

#endif

/**
  * @}
  */
