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

void CM7_handle_remote( uint32_t arg )
{
    assert(AMP_DirectBuffer.id == DIRECTMSG_ID);

    /* Set status to "received" */
    AMP_DirectBuffer.msg_status = MSGSTATUS_CM4_TO_CM7_DONE;

    /* Neccessary action depends on msg_id */
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
    case  MSGTYPE_SETTINGS_CM7:
        /* return a CM/ task list dump line by line */
        Handle_Receive_4();
        break;
    default:
        DEBUG_PRINTF("cm7_handle_remote: unknown msg_id %d\n", AMP_DirectBuffer.msg_id );
    } /* switch */
}

/******************************************************************************
 * CM7 Handler for remote device registration
 * @note  will be executed in interrupt context
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
 * @note  will be executed in interrupt context
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
 * @note  will be executed in interrupt context
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
 * @note  will be executed in interrupt context
 *****************************************************************************/
void Handle_Receive_4(void)
{
    EE_LimitsT eelt;
    if ( AMP_DirectBuffer.msg_sub_id == ACTIONID_INIT ) {
        /* Then Initialize Settings list generation */
        AMP_DirectBuffer.msg_sub_id = 0;
    }
    AMP_DirectBuffer.msg4.bIsValid = Config_GetValMinMax(AMP_DirectBuffer.msg_sub_id, &eelt );
    if (AMP_DirectBuffer.msg4.bIsValid) {
        AMP_DirectBuffer.msg4.val  = eelt.deflt;
        AMP_DirectBuffer.msg4.min  = eelt.min;
        AMP_DirectBuffer.msg4.max  = eelt.max;
        AMP_DirectBuffer.msg4.help = eelt.help;
        AMP_DirectBuffer.msg_sub_id++;
    }
    AMP_DirectBuffer.msg_status    = MSGSTATUS_CM7_TO_CM4_ACTIVE;
    Ipc_CM7_SendDirect();
}
