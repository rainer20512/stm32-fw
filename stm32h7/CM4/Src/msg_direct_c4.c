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

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/* Ptr to the direct message buffer, allocated by CM7 core and passed as reference to CM4 */
AMPDctBufPtr_t      AMP_DctBuf_Ptr;

/* External references------------------------------------------------------ */
void Ipc_CM4_SendDirect( void );



/******************************************************************************
 * Callback for CM7 message reception 
 * @note  will be executed in interrupt context
 *****************************************************************************/
void cm4_msg_direct_received(void)
{
    assert(AMP_DctBuf_Ptr->id == DIRECTMSG_ID);

    /* Set status to "received" */
    AMP_DctBuf_Ptr->msg_status = MSGSTATUS_CM7_TO_CM4_DONE;

    /* Neccessary action depends on msg_id */
    switch ( AMP_DctBuf_Ptr->msg_id ) {
        case  MSGTYPE_REGISTER_DEVICE:
        case MSGTYPE_CHKUNIQUE_DEVICE:
        case MSGTYPE_TASKLIST_CM7:
            /* All the previous will get their results by polling */
            break;
        default:
            DEBUG_PRINTF("cm4_direct_msg: unknown msg_id %d\n", AMP_DctBuf_Ptr->msg_id );
    }
}

/******************************************************************************
 * CM4 Handler for remote device registration
 * @note  will be executed in interrupt context
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
 * CM4 Handler for remote device uniqueness check
 * @note  will be executed in interrupt context
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

