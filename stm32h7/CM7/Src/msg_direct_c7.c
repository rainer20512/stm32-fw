/*
 ******************************************************************************
 * @file    msg_direct_c7.c 
 * @author  rainer
 *
 * @brief  direct interprocess communication w/o RTOS on STM32H745 
 *         CM7 part
 *         
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "config/config.h"
#include "hardware.h"
#include "msg_direct.h"

#include "dev/hw_device.h"
#include "task/minitask.h"
#include "eeprom.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/* External references------------------------------------------------------ */
void Ipc_CM7_SendDirect( void );
int32_t AddDeviceRemote(const HW_DeviceType *dev);
bool    CheckUniqueRemote(const HW_DeviceType *dev);

/* Forward declarations ---------------------------------------------------- */
void Handle_Receive_1(void);
void Handle_Receive_2(void);
void Handle_Receive_3(void);
void Handle_Receive_4(void);
void Handle_Receive_5(void);


/******************************************************************************
 * Callback for CM4 message reception on CM7 core
 * This callback consists of two parts: The first one, which is called in
 * interrupt context ( this routine ) should be used for all message types
 * which only requre a very short respones ( setting a "done" flag e.g. )
 * Respones, which will take a longer execution time, should be executed
 * in the remote task.
 * 
 * @note  this routine will be executed in interrupt context.
 *
 *****************************************************************************/
void CM7_handle_remote_direct(void)
{
    assert(AMP_DirectBuffer.id == DIRECTMSG_ID);


    /* Neccessary action depends on msg_id */
    switch ( AMP_DirectBuffer.msg_id ) {
        /* handle all message types which only require a "done" status to be set */
        /* and the peer waits for completion by polling                          */
        /* case keks: uncomment the next statement as soon as the first case appears here */
            /* Set status to "received" */
            /* AMP_DirectBuffer.msg_status = MSGSTATUS_CM7_TO_CM4_DONE;
            break; */
        default:
            /* For all other message types: activate handler task */
            TaskNotify(TASK_REMOTE_CM7);
    }
}

/******************************************************************************
 * Thread function to handle "complex" messages from CM4 on CM7 core, 
 * which are too time consuming to handle within interrupt context
 *****************************************************************************/
void CM7_handle_remote( uint32_t arg )
{
    assert(AMP_DirectBuffer.id == DIRECTMSG_ID);

    UNUSED(arg);

    /* Neccessary action depends on msg_id                      */
    /* Note: all handlers must set msg_status finally and       */
    /* call IPC handler, if an answer is expected               */

    switch ( AMP_DirectBuffer.msg_id ) {
    case  MSGTYPE_REGISTER_DEVICE:
        /* do remote device registration */
        Handle_Receive_1();
        break;
    case  MSGTYPE_CHKUNIQUE_DEVICE:
        /* do remote device registration */
        Handle_Receive_2();
        break;
    case  MSGTYPE_TASKLIST_CM7:
        /* return a CM/ task list dump line by line */
        Handle_Receive_3();
        break;
    case  MSGTYPE_SETTINGS_GET_CM7:
        /* return a CM7 task list dump line by line */
        Handle_Receive_4();
        break;
    case  MSGTYPE_SETTINGS_SET_CM7:
        /* Set one CM7 settings item */
        Handle_Receive_5();
        break;
    default:
        DEBUG_PRINTF("CM7_handle_remote: unknown msg_id %d\n", AMP_DirectBuffer.msg_id );
    } /* switch */

}

/******************************************************************************
 * CM7 Handler for remote device registration
 *****************************************************************************/
void Handle_Receive_1(void)
{
    /* 
     * get Address of remote flash-stored device description. As all device
     * infos are stored in flash, direct read is possible
     */
    const HW_DeviceType *dev = (HW_DeviceType *)AMP_DirectBuffer.msg1.dev;
    AMP_DirectBuffer.msg1.response = AddDeviceRemote(dev);
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

/******************************************************************************
 * CM7 Handler for remote device uniqueness check
 *****************************************************************************/
void Handle_Receive_2(void)
{
    /* 
     * get Address of remote flash-stored device description. As all device
     * infos are stored in flash, direct read is possible
     */
    const HW_DeviceType *dev = (HW_DeviceType *)AMP_DirectBuffer.msg1.dev;
    AMP_DirectBuffer.msg1.response = CheckUniqueRemote(dev);
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

/******************************************************************************
 * CM7 Handler for remote (CM7) task list
 *****************************************************************************/
void Handle_Receive_3(void)
{
    static const char *prefixstr;
    /* first call ? */  /* Then Initialize list generation */
    if ( AMP_DirectBuffer.msg_sub_id == ACTIONID_INIT ) {
        /* Then Initialize list generation */
        TaskSetListStartStop(AMP_DirectBuffer.msg3.init.startstop[0],AMP_DirectBuffer.msg3.init.startstop[1]);
        prefixstr = AMP_DirectBuffer.msg3.init.prefixstr;
    }
    
    AMP_DirectBuffer.msg_sub_id = TaskIterateList ( AMP_DirectBuffer.msg_sub_id, AMP_DirectBuffer.msg3.buffer, TYPE3_BUFLEN, prefixstr );
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

/******************************************************************************
 * CM7 Handler for remote (CM7) Settings list
 *****************************************************************************/
void Handle_Receive_4(void)
{
    EE_LimitsT eelt;
    if ( AMP_DirectBuffer.msg_sub_id == ACTIONID_INIT ) {
        /* Then Initialize Settings list generation */
        AMP_DirectBuffer.msg_sub_id = 0;
    }
    AMP_DirectBuffer.msg4.max_idx  = Config_GetCnt();
    AMP_DirectBuffer.msg4.bIsValid = Config_GetValMinMax(AMP_DirectBuffer.msg_sub_id, &eelt );
    if (AMP_DirectBuffer.msg4.bIsValid) {
        AMP_DirectBuffer.msg4.idx  = AMP_DirectBuffer.msg_sub_id;
        AMP_DirectBuffer.msg4.val  = eelt.deflt;
        AMP_DirectBuffer.msg4.min  = eelt.min;
        AMP_DirectBuffer.msg4.max  = eelt.max;
        AMP_DirectBuffer.msg4.help = eelt.help;
        AMP_DirectBuffer.msg4.type = eelt.type;
        AMP_DirectBuffer.msg_sub_id++;
    }
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

/******************************************************************************
 * CM7 Handler for remote (from CM4) Setting set
 * Uses also MSgSettingItemT to exchange the data
 * Settings index is expected in member idx, newvalue is expected in  member val
 * result will be returned in member bIsValid
 *****************************************************************************/
void Handle_Receive_5(void)
{
    AMP_DirectBuffer.msg4.bIsValid = Config_SetVal(AMP_DirectBuffer.msg4.idx, AMP_DirectBuffer.msg4.val );
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

///////////////////////////////////////////////////////////////////////////////
// Stubs
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * CM7 Stub for clock change event on CM4 core
 * @note  The message field is unused in this type of message
 *****************************************************************************/
void MSGD_DoClockChange(void)
{
    AMP_DirectBuffer.msg_id      = MSGTYPE_CLOCKCHANGE_CM4;
    AMP_DirectBuffer.msg_status  = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}

bool MSGD_WaitForRemoteClockChange(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DirectBuffer.msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait for CM4 clk chng timed out!");
            return false;
        }
    }
    return true;
}
