/*
 ******************************************************************************
 * @file    msg_direct.h 
 * @author  rainer
 *
 * @brief  interprocess communication between different core w/o RTOS
 *         via shared memory and Hardware semaphores
 *
 * @note   Any changes in AMP control block requires recompile of both parts!
 * 
 ******************************************************************************
 */
#ifndef __MSG_DIRECT_H__
#define __MSG_DIRECT_H__

/* Constant Identifier of direct message buffer */
#define DIRECTMSG_ID                            0x4443544D

#define DIRECTMSG_WAIT_TIMEOUT                  10
/*
 * different status of a message buffer
 */
#define MSGSTATUS_NONE                          0  /* unused */
#define MSGSTATUS_CM7_TO_CM4_ACTIVE             7  /* Sender=CM7, Receiver=CM4 */
#define MSGSTATUS_CM7_TO_CM4_DONE               8  /* Sender=CM7, Receiver=CM4 */
#define MSGSTATUS_CM4_TO_CM7_ACTIVE             4  /* Sender=CM4, Receiver=CM7 */
#define MSGSTATUS_CM4_TO_CM7_DONE               5  /* Sender=CM4, Receiver=CM7 */


/*
 * different types of data transfer 
 */
#define MSGTYPE_REGISTER_DEVICE                 1   /* Register a CM4 device on CM7 core */
#define MSGTYPE_CHKUNIQUE_DEVICE                2   /* Check for Pin uniqueness of CM4 device on CM7 core */
#define MSGTYPE_TASKLIST_CM7                    3   /* Get a line by line tasklist for CM4 from CM7*/


/* 
 * Transfer type 1: cm4-device registration on cm7 core. As all device data is 
 * stored in flash, we can directly pass the flash address to CM7, as CM7 is able
 * to access the CM4 flash as well
 * Also used for Transfer type 2: CheckUnique for a device 
 * response is a boolean type
 */
typedef struct {
    void *dev;                        /* CM4->CM7: HW-Device address */
    int32_t response;                 /* CM7->CM4: >= 0 ok, -1 error for type 1, 0 = false, != 0 = true for type 2*/
} MsgRegisterDeviceT;

#define TYPE3_BUFLEN        80
typedef union {
        struct {
            uint8_t startstop[2];     /* Set start and stop item when sub_id == 0    */
            const char *prefixstr;    /* Prefix for every generated line             */
        } init;
        char buffer[TYPE3_BUFLEN];    /* Return buffer for the actual task list line */
} MsgTaskItemU;                       /* List is finished, when buffer == ""         */ 

/*
 * Direct message buffer: consists of constant ID, msg type identifier,
 * sub identifiert für that id (if needed) and a
 * message buffer as an union of all possible message buffers 
 */
typedef struct {
  uint32_t id;
  uint32_t msg_id;
  uint32_t msg_sub_id;
  uint32_t msg_status;
  union {
    MsgRegisterDeviceT msg1;  
    MsgTaskItemU       msg3;
  }; 
} AMPDctBuf_t;


/* Used by CM7 */
#if defined ( CORE_CM7 )
    extern AMPDctBuf_t          AMP_DirectBuffer;
    void cm7_msg_direct_received(void);
#endif

/* Used by CM4 */
#if defined ( CORE_CM4 )
    typedef AMPDctBuf_t *       AMPDctBufPtr_t;
    extern  AMPDctBufPtr_t      AMP_DctBuf_Ptr;
    void cm4_msg_direct_received(void);
    void MSGD_DoRemoteRegistration(void *dev);
    int32_t MSGD_WaitForRemoteRegistration(void);
    void MSGD_DoCheckUniqueRemote(void *dev);
    bool MSGD_WaitForCheckUniqueRemote(void);
    void MSGD_GetTasklistLine(bool bInitCall, const char *prefixstr);
    char *MSGD_WaitForTasklistLine(void);
#endif


#endif // __MSG_DIRECT_H__