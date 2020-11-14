#ifndef __NEXHARDWARE_H__
#define __NEXHARDWARE_H__
#include "NexConfig.h"
#include "NexTouch.h"
#include "NexObject.h"
#include "util/Utilities.h"

#define NEX_RET_INVALID_CMD                 (0x00)
#define NEX_RET_CMD_FINISHED                (0x01)
#define NEX_RET_EVENT_LAUNCHED              (0x88)
#define NEX_RET_EVENT_UPGRADED              (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_USERDEF_HEAD                (0x75)  /* Userdefined return, in TouchMove event, e.g. */
#define NEX_RET_INVALID_COMPONENT_ID        (0x02)
#define NEX_RET_INVALID_PAGE_ID             (0x03)
#define NEX_RET_INVALID_PICTURE_ID          (0x04)
#define NEX_RET_INVALID_FONT_ID             (0x05)
#define NEX_RET_INVALID_FILE                (0x06)  
#define NEX_RET_INVALID_CRC                 (0x09)  
#define NEX_RET_INVALID_BAUD                (0x11)
#define NEX_RET_INVALID_WAV_OR_CH           (0x12)
#define NEX_RET_INVALID_VARIABLE            (0x1A)
#define NEX_RET_INVALID_OPERATION           (0x1B)
#define NEX_RET_INVALID_ASSIGN              (0x1C)
#define NEX_RET_INVALID_EEPROM_OP           (0x1D)
#define NEX_RET_INVALID_PARAM_QTY           (0x1E)
#define NEX_RET_INVALID_IO_OP               (0x1F)
#define NEX_RET_INVALID_ESC_CHAR            (0x20)
#define NEX_RET_VARNAME_TOO_LONG            (0x23)
#define NEX_RET_SERBUF_OVERFLOW             (0x24)

#define TERMLEN                             3       /* Length of nextion comm terminator ( 3x 0xff)  */
#define STR_ANSWER_LEN                      32      /* Max length a nextion answer string may have   */
#define NUMBER_ANSWER_LEN                   8       /* num identifier + 4 num bytes + 3 x terminator */
#define ONE_BYTE_ANSWER_LEN                 4       /* one byte ID, 3 x terminator                   */

extern const uint8_t termstr[];

typedef struct nexAnswerStr {
    uint8_t strlen;
    char txt[STR_ANSWER_LEN];
} nexStr;

struct nexAnswer {
    union {
        uint32_t Number;                   /* number answer                     */
        uint8_t  OneByteAnswer;            /* One byte answer                   */
        nexStr   Str;                      /* returned string                   */  
    };
    uint8_t bOneByteValid;                 /* != 0, if one byte answer is valid */                 
    uint8_t bNumberValid;                  /* != 0, if number answer is valid   */
    uint8_t bStringValid;                  /* != 0, if String answer is valid   */
    uint8_t bUserDefValid;                 /* != 0, if user defined answer is valid   */
    uint8_t bAnswerValid;                  /* != 0, if we found the terminator  */
};

/* Reset all types of answers */
#define RESET_VALID(a) \
    do  {                   \
    (a)->bAnswerValid   = 0;  \
    (a)->bOneByteValid  = 0;  \
    (a)->bNumberValid   = 0;  \
    (a)->bStringValid   = 0;  \
    (a)->bUserDefValid  = 0;  \
    } while (0)


extern struct nexAnswer answer;

/**
 * Init Nextion.  
 * 
 * @return true if success, false for failure. 
 */
uint8_t nexInit(void);

/**
 * Listen touch event and calling callbacks attached before.
 * 
 * Supports push and pop at present. 
 *
 * @param nex_listen_list - index to Nextion Components list. 
 * @return none. 
 *
 * @warning This function must be called repeatedly to response touch events
 *  from Nextion touch panel. Actually, you should place it in your loop function. 
 */
void nexLoop(struct NexObject *nex_listen_list[]);

/**
 * @}
 */
//timeout=100
uint8_t recvRetNumber(uint32_t *number);
uint16_t recvRetString(char *buffer, uint16_t len);
uint8_t recvRetCommandFinished();
void sendCommand(char *command);

#define CreateNexObject(obj, pid, id, name) \
    obj.__pid = pid;                        \
    obj.__cid = id;                         \
    StringCopy(obj.__name, name)

#endif /* #ifndef __NEXHARDWARE_H__ */