/**
 * @file NexHardware.cpp
 *
 * The implementation of base API for using Nextion. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/11
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include "NexHardware.h"
#include "util/Utilities.h"

#include "debug_helper.h"





/* Nextion termination byte sequence */
const uint8_t termstr[TERMLEN] = { 0xff, 0xff, 0xff };

char cmd[64];
char buf[12];
struct nexAnswer    answer    = {0 };

#define ANSWER_IS_TMO       0
#define ANSWER_IS_SUCCESS   1 
#define ANSWER_IS_NUM       2
#define ANSWER_IS_STR       3
#define ANSWER_IS_USERDEF   4
#define ANSWER_IS_ERR       (-1)
#define ANSWER_IS_UNDECODED (-2)

/* forward declarations  -----------------------------------------------------*/
int8_t nexOneNumStr(uint16_t answer_len);

/*
 * Receive uint32_t data. 
 * 
 * @param number - save uint32_t data. 
 * @param 100 - set 100 time. 
 *
 * @retval 1 - success. 
 * @retval 0 - failed.
 *
 */
uint8_t recvRetNumber(uint32_t *number)
{

    /* check for reception of answer of expected chars */
    int8_t ret = nexOneNumStr(NUMBER_ANSWER_LEN);

    /* check for expected answer of type NUM */ 
    if ( ret != ANSWER_IS_NUM ) return 0;
       
    *number = answer.Number;
    return 1;
}

/*
 * Receive string data. 
 * 
 * @param buffer - save string data. 
 * @param len - string buffer length. 
 * @param 100 - set 100 time. 
 *
 * @return the length of string buffer.
 *
 */
uint16_t recvRetString(char *buffer, uint16_t len)
{
    /* check for reception of answer of expected chars: 32 bytes pure str + indicator byte + termination str  */
    int8_t ret = nexOneNumStr(STR_ANSWER_LEN+1+TERMLEN);

    /* check for expected answer of type NUM */ 
    if ( ret != ANSWER_IS_STR ) return 0;

    uint16_t work = answer.Str.strlen;
    if ( work > len ) work = len;
    memcpy(buffer, answer.Str.txt, work);

    return work;
}

/*
 * Command is executed successfully. 
 *
 * @param 100 - set 100 time.
 *
 * @retval 1 - success.
 * @retval 0 - failed. 
 *
 */
uint8_t recvRetCommandFinished()
{

    /* Wait for answer and check for reception of answer of expected chars */
    int8_t ret = nexOneNumStr(ONE_BYTE_ANSWER_LEN);
   
    /* check for expected OK answer */ 
    return ret == ANSWER_IS_SUCCESS ? 1 : 0;
}

void sendCommand(char *command)
{
    /* reset all types of answer-valid-bytes */
    RESET_VALID(&answer);

    nexSerial_start();
    nexSerial_print((unsigned char*)command);
    nexSerial_terminate();
}

uint8_t nexInit(void)
{
    uint8_t ret1 = 0;
    uint8_t ret2 = 0;

    //dbSerialBegin(9600);
    nexSerial_init(9600);
    sendCommand("");
    sendCommand("bkcmd=1");
    ret1 = recvRetCommandFinished();
    sendCommand("page 0");
    ret2 = recvRetCommandFinished();
    return ret1 && ret2;
}


static void nexStoreOneByte(uint8_t onebyte)
{
    answer.OneByteAnswer  = onebyte;
    answer.bOneByteValid  = 1; 
}

static void nexStoreNumber()
{
    /* Answer length w/o first header (0x71) byte */
    #define NUMBER_BUF      4
    char temp[NUMBER_BUF];
 
    /* Check for number prefix and decode number */
    if ( nexSerial_readBytes((char *)temp, NUMBER_BUF) < NUMBER_BUF ) return;

    answer.Number       = ((uint32_t)temp[3] << 24) | ((uint32_t)temp[2] << 16) | (temp[1] << 8) | (temp[0]);
    answer.bNumberValid = 1;
}

static void nexStoreString(void)
{
    uint8_t work = nexSerial_available();
    if ( work > STR_ANSWER_LEN ) work = STR_ANSWER_LEN;
    nexSerial_readBytes(answer.Str.txt, work);
    answer.Str.strlen = work;
}


/******************************************************************************
 * Handle One Byte, Number or String answers from Nextion 
 * This function should be called synchronously by the issuer of the
 * corrensponding command after the command has been sent to nextion
 *****************************************************************************/
int8_t nexOneNumStr(uint16_t answer_len)
{
    uint8_t c;
    int8_t ret;

    /* check for reception of answer of expected chars */
    if ( nexSerial_WaitFor(answer_len, &answer.bAnswerValid, 1)== 0 ) {
        ret = ANSWER_IS_TMO;
    } else {

        c = nexSerial_read();
        switch (c) {
            case NEX_RET_INVALID_CMD:
            case NEX_RET_CMD_FINISHED:
            case NEX_RET_INVALID_COMPONENT_ID:
            case NEX_RET_INVALID_PAGE_ID:
            case NEX_RET_INVALID_PICTURE_ID:
            case NEX_RET_INVALID_FONT_ID:
            case NEX_RET_INVALID_FILE:
            case NEX_RET_INVALID_CRC:
            case NEX_RET_INVALID_BAUD:
            case NEX_RET_INVALID_WAV_OR_CH:
            case NEX_RET_INVALID_VARIABLE:
            case NEX_RET_INVALID_OPERATION:
            case NEX_RET_INVALID_ASSIGN:
            case NEX_RET_INVALID_EEPROM_OP:
            case NEX_RET_INVALID_PARAM_QTY:
            case NEX_RET_INVALID_IO_OP:
            case NEX_RET_INVALID_ESC_CHAR:
            case NEX_RET_VARNAME_TOO_LONG:
            case NEX_RET_SERBUF_OVERFLOW:
                nexStoreOneByte(c);
                ret = ( c == NEX_RET_CMD_FINISHED ? ANSWER_IS_SUCCESS : ANSWER_IS_ERR );
                DEBUG_PRINTF("Got byte %02x\n",c);
                break;
            case NEX_RET_NUMBER_HEAD:
                nexStoreNumber();
                ret = ANSWER_IS_NUM;
                DEBUG_PRINTF("Got number %02d\n",answer.Number);
                break;
            case NEX_RET_STRING_HEAD:
                nexStoreString();
                answer.bStringValid = 1;
                ret = ANSWER_IS_STR;
                DEBUG_PRINTF("Got Str of len %d\n", answer.Str.strlen);
                break;
            default:
                DEBUG_PRINTF("nexOneNumStr cannot handle %02x\n", c);
                ret = ANSWER_IS_UNDECODED;
        }
    }
    /* Reset input ( and output buffer to get rid of partial or undecoded receive data ) */
    nexSerial_start();
    
    return ret;
}

static void nexHandleTouch(struct NexObject *nex_listen_list[])
{
    /* Answer length w/o first header (0x65) byte and w/o 3 trailer bytes*/
    #define TOUCH_LEN      3
    char tbuf[TOUCH_LEN];
    
    /* check for expected OK answer */ 
    if ( nexSerial_readBytes((char *)tbuf, TOUCH_LEN) == TOUCH_LEN ) {
        NexTouch_iterate(nex_listen_list, tbuf[0], tbuf[1], (int32_t)tbuf[2]);
    }
}

/******************************************************************************
 * Handle Touch Events.
 * This function is called asynchronously by serial in tasl
 *****************************************************************************/
void nexLoop(struct NexObject *nex_listen_list[])
{
    uint8_t c;

    c = nexSerial_read();
    switch (c) {
        case NEX_RET_EVENT_TOUCH_HEAD:
            nexHandleTouch(nex_listen_list);
            break;  
        case NEX_RET_USERDEF_HEAD:
            nexStoreString();
            answer.bUserDefValid = 1;
            // TODO handle user defined answer
            break;
        default:
            DEBUG_PRINTF("nexLoop cannot handle %02x\n", c);
    }
    /* Reset input ( and output buffer to get rid of partial or undecoded receive data ) */
    nexSerial_start();
}