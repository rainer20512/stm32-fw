/**
  ******************************************************************************
  * @file    circbuff.c
  * @author  Rainer 
  * @brief   Implementation of a circular buffer of uint8_t values
  *
  * @note    Mainly used for USART input and output
  * 
  ******************************************************************************
  */

#include "config/config.h"
#include "error.h"
#include "circbuf.h"

#include <string.h>

/*********************************************************************************
  * Implementation of a circular buffer 
  ********************************************************************************/
bool CircBuff_Init(CircBuffT *cbuff, uint32_t size, uint8_t *bufptr )
{
    cbuff->size = size;
    cbuff->mask = size-1;
    if ( cbuff->size & cbuff->mask ) {
        Error_Handler_XX(-10, __FILE__, __LINE__);
        return false;
    }
    cbuff->buf = bufptr;
    cbuff->wrptr = cbuff->rdptr = 0;

    return true;
}

bool CircBuff_Put(CircBuffT *b, uint8_t ch )
{
  if ( CHECK_CBUF_FREE(*b, 1) ) {    
    b->buf[b->wrptr]=ch;
    b->wrptr = CBUFPTR_INCR(*b, wrptr, 1);
    return true;
  } else {
    b->buf[CBUFPTR_DECR(*b, wrptr, 1)] = '*';
    return false;
  }
}

bool CircBuff_Get(CircBuffT *b, uint8_t *ch )
{
  if ( CBUF_EMPTY(*b) ) return false;

  *ch = b->buf[b->rdptr];
  b->rdptr = CBUFPTR_INCR(*b, rdptr, 1); 
  return true;  
}

/******************************************************************************
 * Write 2 bytes. we assume buffer pointer is at an even position before (and 
 * after ) so no rollover of the underlying linear buffer may occur
 *****************************************************************************/
bool CircBuff_Put2(CircBuffT *b, uint16_t w )
{
  if ( CHECK_CBUF_FREE(*b, 2) ) {
    assert ( ( b->wrptr & 0b1 )  == 0 );  
    *(uint16_t*)(b->buf+b->wrptr) = w;
    b->wrptr = CBUFPTR_INCR(*b, wrptr, 2);
    return true;
  } else {
    return false;
  }
}

/******************************************************************************
 * Get 2 bytes. we assume buffer pointer is at an even position before (and 
 * after ) so no rollover of the underlying linear buffer may occur
 *****************************************************************************/
bool CircBuff_Get2(CircBuffT *b, uint16_t *w )
{
  if ( CBUF_EMPTY(*b) ) return false;

  assert ( ( b->rdptr & 0b1 )  == 0 );  
  *w = *(uint16_t*)(b->buf+b->rdptr);
  b->rdptr = CBUFPTR_INCR(*b, rdptr, 2);
  return true;  
}

/******************************************************************************
 * Peek 2 bytes. we assume buffer pointer is at an even position before (and 
 * after ) so no rollover of the underlying linear buffer may occur
 *****************************************************************************/
bool CircBuff_Peek2(CircBuffT *b, uint16_t *w )
{
  if ( CBUF_EMPTY(*b) ) return false;

  assert ( ( b->rdptr & 0b1 )  == 0 );  
  *w = *(uint16_t*)(b->buf+b->rdptr);
  return true;  
}



/******************************************************************************
 * Put a byte vector to circular buffer, which is guaranteed to produce no wrap
 * and no overflow. 
 * !! These both prerequisites must have been checked before !!
 *****************************************************************************/
static void CB_PutNowrap(CircBuffT *b, uint8_t *buf, uint32_t buflen)
{
    memcpy(b->buf+b->wrptr, buf, buflen);
    b->wrptr = CBUFPTR_INCR(*b, wrptr, buflen);
}

/******************************************************************************
 * Put a byte vector to circular buffer. If a buffer rollover occurs, this
 * vector write is executed in two pieces.
 * If the circular buffer does not have enough room to store the entire buffer,
 * only the fitting first bytes will be stored
 * 
 * The number of stored bytes will be returned
 *****************************************************************************/
#include "debug_helper.h"
uint32_t CircBuff_PutStr(CircBuffT *b, uint8_t *buf, uint32_t buflen)
{
    /* chek for storeable bytes and reduce write size, if neccessary */
    uint32_t temp = CBUF_GET_FREE(*b);
    if ( buflen > temp )  buflen = temp;

    /*
     * Check, whether a wraparound will occur during write. If so, 
     * write in two parts: First up to wraparound, then the rest
     */
    temp = CBUF_GET_LINEARWRITESIZE(*b);
    if ( temp < buflen ) {
        DEBUG_PRINTF("Write in 2 portions: %d and %d\n", temp, buflen-temp);
        CB_PutNowrap(b, buf, temp);
        buf += temp;
        temp = buflen - temp;
    } else {
        DEBUG_PRINTF("Write in 1 portion: %d \n", buflen);
        temp = buflen;
    }
    CB_PutNowrap(b, buf, temp);

    return buflen;
}

bool CircBuff_Get_Indexed(CircBuffT *b, uint32_t idx, uint8_t *ch )
{
  if ( !CBUF_VALID_RDPOS(*b, idx) ) return false;

  *ch = b->buf[b->rdptr+idx];
  return true;  
}

void CircBuff_Mark(CircBuffT *b)
{
    b->wrback_ptr = b->wrptr;
}
void CircBuff_PutBack(CircBuffT *b, uint8_t ch )
{
    b->buf[b->wrback_ptr]=ch;
    b->wrback_ptr = CBUFPTR_INCR(*b, wrback_ptr, 1);
}

/******************************************************************************
 * Returns a pointer to the read buffer and the length of that buffer
 * the read buffer extents up to the write pointer or the upper end of the 
 * underlying linear buffer, whichever is smaller
 *****************************************************************************/
void CircBuff_GetLinearReadBuffer(CircBuffT *b, uint8_t **readbuf, uint32_t *size )
{
    *readbuf = b->buf+b->rdptr;
    *size = CBUF_GET_LINEARREADSIZE(*b);
}

/*********************************************************************************
  * Implementation of a linear buffer 
  ********************************************************************************/
void LinBuff_Init(LinBuffT *lbuff, uint32_t size, uint8_t *bufptr )
{
    lbuff->size = size;
    lbuff->buf = bufptr;
    lbuff->wrptr = lbuff->rdptr = lbuff->ovrrun = 0;
}

bool LinBuff_Putc(LinBuffT *b, uint8_t ch )
{
  if ( CHECK_LBUF_FREE(*b, 1) ) {    
    b->buf[b->wrptr]=ch;
    b->wrptr++;
    b->rdptr=0;
    return true;
  } else {
    b->rdptr=1;
    return false;
  }
}

bool LinBuff_Peekc(LinBuffT *b, uint8_t *ch )
{
  if ( LBUF_EMPTY(*b) ) return false;

  *ch = b->buf[b->rdptr];
  return true;  
}

bool LinBuff_Getc(LinBuffT *b, uint8_t *ch )
{
  bool ret = LinBuff_Peekc(b, ch);
  if ( ret ) b->rdptr++;
  return ret;  
}

#if 0
bool LinBuff_Getc_Indexed(LinBuffT *b, uint32_t idx, uint8_t *ch )
{
  if ( !LBUF_VALID_RDPOS(*b, idx) ) return false;

  *ch = b->buf[b->rdptr+idx];
  return true;  
}
#endif

/*********************************************************************************
  * @brief  Return the whole current input buffer as string
  *         
  * @param  **str - ptr to string, will point to input buffer on return
  * @param  *len  - ptr to size_t to receive the strlen
  *         
  * @note   there will be non \0 character at the end. the consumer
  *         has to check the len parameter to determine the length of returned string
  *
  * @retval None
  ********************************************************************************/
void LinBuff_Gets(LinBuffT *b, uint8_t **str, size_t *len )
{

    /* Return ptr to whole input string */
    *str = b->buf + b->rdptr;
    *len = b->wrptr - b->rdptr;

    /* proceed rdptr */
    b->rdptr = b->wrptr;
}

/*********************************************************************************
  * @brief  Return the first <numtoread> chars from input buffer
  *         
  * @param  *buf       - ptr to string, will point to input buffer on return
  * @param  numtoread  - number of characters to read
  *         
  * @note   if there are less chars in input buffer than requested, only
  *         the number of available characters is returned
  * @note   there will be no \0 character at the end. the consumer
  *         has to check the return len parameter to determine the length of returned string
  *
  * @retval number of characters actually available
  ********************************************************************************/
size_t LinBuff_GetN(LinBuffT *b, uint8_t **buf, size_t numtoread )
{
    size_t avail = LBUF_USED(*b);
    size_t num = ( avail < numtoread ? avail : numtoread );

    /* Return ptr to input string */
    *buf = b->buf + b->rdptr;

    /* proceed rdptr */
    b->rdptr += num;

    return num;
}


/*********************************************************************************
  * @brief  Return the input buffer as string up to the first occurence of 'delim'
  *         If 'delim' is not contained, the whole buffer will be returned
  *
  * @param  **str - ptr to string, will point to input buffer on return
  * @param  *len  - ptr to size_t to receive the strlen
  * @param  delim - delimiter character
  *         
  * @note   there will be non \0 character at the end. the consumer
  *         has to check the len parameter to determine the length of returned string
  * @note   the delimiter is not part of the string returned. It will remain in the
  *         buffer and will be the next character to read. If not used, user must skip
  *         over this delimiter
  *
  * @retval returns true, iff delimiter is found in input buffer
  ********************************************************************************/
bool LinBuff_GetsTo(LinBuffT *b, uint8_t **str, size_t *len, char delimiter )
{
    uint32_t work = b->rdptr;

    /* scan the valid portion of buffer for delimiter */
    while ( work < b->wrptr ) {
      if ( b->buf[work] == delimiter ) {
        /* 
         * we found at position 'work'. Return the portion 
         * from 'rdptr' to one char before delimiter 
         */
        *str = b->buf + b->rdptr;
        *len = work - b->rdptr;

        /* proceed rdptr */
        b->rdptr = work;
        return true;
      }
      work++;
    }

    /* Nothing found: return the whole buffer */
    LinBuff_Gets(b, str, len );
    return false;
}



/*********************************************************************************
  * @brief  Set Input Buffer to empty
  *
  * @retval None
  *
  ********************************************************************************/
void LinBuff_SetEmpty(LinBuffT *b)
{
  b->rdptr = b->wrptr = b->ovrrun = 0;
}


/*********************************************************************************
  * @brief  Remove the first "num" elements from linear buffer. the remaining
  *         content is moved to the begin of the buffer, read and write pointer
  *         are adjustet
  *         
  * @param  num - number of elements to remove from linear buffer
  *         
  * @retval None
  *
  * @note   removal only up to current read pointer
  ********************************************************************************/
bool LinBuff_CleanUp(LinBuffT *b, uint32_t num )
{
  if ( num > b->rdptr ) return false;

  if ( b->wrptr-num > 0 ) memmove(b->buf, b->buf+num, b->wrptr-num);

  b->rdptr-= num;
  b->wrptr-= num;
  b->rdptr = 0;
  return true;
}
  
