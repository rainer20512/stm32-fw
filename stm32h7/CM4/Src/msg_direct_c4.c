/*
 ******************************************************************************
 * @file    msg_direct_c4.c 
 * @author  rainer
 *
 * @brief  direct interprocess communication w/o RTOS on STM32H745 
 *         CM4 part
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

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/* Ptr to the direct message buffer, allocated by CM7 core and passed as reference to CM4 */
AMPDctBufPtr_t      AMP_DctBuf_Ptr;

/* External references------------------------------------------------------ */
void Ipc_CM4_SendDirect( void );
bool DevicesInhibitFrqChange(void);

/* Forward declarations ---------------------------------------------------- */
void Handle_Receive_1025(void);



/******************************************************************************
 * Callback for CM7 message reception on CM4 core
 * This callback consists of two parts: The first one, which is called in
 * interrupt context ( this routine ) should be used for all message types
 * which only requre a very short respones ( setting a "done" flag e.g. )
 * Respones, which will take a longer execution time, should be executed
 * in the remote task.
 * 
 * @note  this routine will be executed in interrupt context.
 *
 *****************************************************************************/
void CM4_handle_remote_direct(void)
{
    assert(AMP_DctBuf_Ptr->id == DIRECTMSG_ID);

    /* Neccessary action depends on msg_id */
    switch ( AMP_DctBuf_Ptr->msg_id ) {
        /* handle all message types which only require a "done" status to be set */
        /* and the peer waits for completion by polling                          */
        case MSGTYPE_REGISTER_DEVICE:
        case MSGTYPE_CHKUNIQUE_DEVICE:
        case MSGTYPE_TASKLIST_CM7:
        case MSGTYPE_SETTINGS_GET_CM7:
        case MSGTYPE_SETTINGS_SET_CM7:
            /* Set status to "received"                                          */
            AMP_DctBuf_Ptr->msg_status = MSGSTATUS_CM7_TO_CM4_DONE;
            break;
        default:
            /* For all other message types: activate handler task */
            TaskNotify(TASK_REMOTE_CM4);
    }
}

/******************************************************************************
 * Thread function to handle "complex" messages from CM7 on CM4 core, 
 * which are too time consuming to handle within interrupt context
 *****************************************************************************/
void CM4_handle_remote(uint32_t arg )
{
    assert(AMP_DctBuf_Ptr->id == DIRECTMSG_ID);

    /* 
     * handle all messages, that require more action than just ACK'ing the reception 
     * Note: all handlers must set msg_status finally and call IPC handler, 
     * if an answer is expected 
     */
    switch ( AMP_DctBuf_Ptr->msg_id ) {
        case MSGTYPE_CLOCKCHANGE_CM4:
            Handle_Receive_1025();
            break;
        default:
            DEBUG_PRINTF("CM4_handle_remote: unknown msg_id %d\n", AMP_DctBuf_Ptr->msg_id );
    }
}

/******************************************************************************
 * CM4 Handler for remote (from CM7) clock change event
 * Notify all devices about clock change
 *****************************************************************************/
void Handle_Receive_1025(void)
{
    if ( DevicesInhibitFrqChange() ) {
        DEBUG_PUTS("Error: One or more CM4 devices disagreed to frq change!");
    }
    /* Set status to "received" */
    AMP_DctBuf_Ptr->msg_status = MSGSTATUS_CM7_TO_CM4_DONE;
}

///////////////////////////////////////////////////////////////////////////////
// Stubs
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * CM4 Stub for remote device registration
 *****************************************************************************/
void MSGD_DoRemoteRegistration(void *dev)
{
    /* 
     * get Address of remote flash-stored device description. As all device
     * infos are stored in flash, direct read is possible
     */
    AMP_DctBuf_Ptr->msg1.dev    = dev;
    AMP_DctBuf_Ptr->msg_id      = MSGTYPE_REGISTER_DEVICE;
    AMP_DctBuf_Ptr->msg_status  = MSGSTATUS_CM4_TO_CM7_ACTIVE;
    Ipc_CM4_SendDirect();
}

int32_t MSGD_WaitForRemoteRegistration(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DctBuf_Ptr->msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait #1 timed out!");
            return -1;
        }
    }
    return AMP_DctBuf_Ptr->msg1.response;
}

/******************************************************************************
 * CM4 Stub for remote device uniqueness check
 *****************************************************************************/
void MSGD_DoCheckUniqueRemote(void *dev)
{
    /* 
     * get Address of remote flash-stored device description. As all device
     * infos are stored in flash, direct read is possible
     */
    AMP_DctBuf_Ptr->msg1.dev    = dev;
    AMP_DctBuf_Ptr->msg_id      = MSGTYPE_CHKUNIQUE_DEVICE;
    AMP_DctBuf_Ptr->msg_status  = MSGSTATUS_CM4_TO_CM7_ACTIVE;
    Ipc_CM4_SendDirect();
}

bool MSGD_WaitForCheckUniqueRemote(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DctBuf_Ptr->msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait #2 timed out!");
            return -1;
        }
    }
    return (bool)AMP_DctBuf_Ptr->msg1.response;
}

/******************************************************************************
 * CM4 Stub for remote task list from CM7
 *****************************************************************************/
void MSGD_GetTasklistLine(bool bInitCall, const char *prefixstr)
{
    if ( bInitCall ) {
        AMP_DctBuf_Ptr->msg3.init.startstop[0] = LISTMODE_HEADER;
        AMP_DctBuf_Ptr->msg3.init.startstop[1] = LISTMODE_FOOTER;
        AMP_DctBuf_Ptr->msg3.init.prefixstr    = prefixstr;
        AMP_DctBuf_Ptr->msg_sub_id             = ACTIONID_INIT;
    }

    AMP_DctBuf_Ptr->msg_id      = MSGTYPE_TASKLIST_CM7;
    AMP_DctBuf_Ptr->msg_status  = MSGSTATUS_CM4_TO_CM7_ACTIVE;
    Ipc_CM4_SendDirect();
}

char *MSGD_WaitForTasklistLine(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DctBuf_Ptr->msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait #3 timed out!");
            return NULL;
        }
    }
    return  *(AMP_DctBuf_Ptr->msg3.buffer) == '\0' ? NULL : AMP_DctBuf_Ptr->msg3.buffer;
}

/******************************************************************************
 * CM4 Stub for Settings list from CM7
 *****************************************************************************/
void MSGD_GetSettingsLine(bool bInitCall)
{
    if ( bInitCall ) {
        AMP_DctBuf_Ptr->msg_sub_id = ACTIONID_INIT;
    }

    AMP_DctBuf_Ptr->msg_id      = MSGTYPE_SETTINGS_GET_CM7;
    AMP_DctBuf_Ptr->msg_status  = MSGSTATUS_CM4_TO_CM7_ACTIVE;
    Ipc_CM4_SendDirect();
}

MSgSettingItemT *MSGD_WaitForGetSettingsLine(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DctBuf_Ptr->msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait #4 timed out!");
            return 0;
        }
    }
    return  &AMP_DctBuf_Ptr->msg4;
}

/******************************************************************************
 * CM4 Stub for set one CM7 settings item
 *****************************************************************************/
void MSGD_SetSettingsLine(uint8_t idx, uint8_t newval)
{
    AMP_DctBuf_Ptr->msg_id      = MSGTYPE_SETTINGS_SET_CM7;
    AMP_DctBuf_Ptr->msg_status  = MSGSTATUS_CM4_TO_CM7_ACTIVE;
    AMP_DctBuf_Ptr->msg4.idx    = idx;
    AMP_DctBuf_Ptr->msg4.val    = newval;
    Ipc_CM4_SendDirect();
}

bool MSGD_WaitForSetSettingsLine(void)
{
    uint32_t tickstart = HAL_GetTick();
    while ( AMP_DctBuf_Ptr->msg_status  != MSGSTATUS_CM7_TO_CM4_DONE ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DIRECTMSG_WAIT_TIMEOUT) {
            DEBUG_PUTS("DirectMessage Wait #5 timed out!");
            return false;
        }
    }
    return  AMP_DctBuf_Ptr->msg4.bIsValid;
}
