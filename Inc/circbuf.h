/**
  ******************************************************************************
  * @file    circbuff.h
  * @author  Rainer 
  * @brief   Implementation of a circular and linear buffer of uint8_t values
  *
  * @note    Mainly used for USART input and output
  * 
  ******************************************************************************
  */

#ifndef __CIRCBUF_H
#define __CIRCBUF_H

#include "config/config.h"

#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif


/* Public typedef -----------------------------------------------------------*/
typedef struct circular_buffer {
	uint8_t *buf;                     /* Pointer to buffer array */
        uint32_t size;                    /* Size of buffer, must be a power of 2 */
        uint32_t mask;                    /* Size of buffer-1 */
        uint32_t wrptr;                   /* write position within output buffer */
        uint32_t wrback_ptr;              /* write position for writeback operation */
        uint32_t rdptr;                   /* position of next character to be transferred to output device */
} CircBuffT; 

/* Public typedef -----------------------------------------------------------*/
typedef struct linear_buffer {
	uint8_t *buf;                     /* Pointer to buffer array */
        uint32_t size;                    /* Size of buffer, no size restrictions */
        uint32_t wrptr;                   /* write position within output buffer */
        uint32_t rdptr;                   /* position of next character to be transferred to output device */
} LinBuffT; 


/* increment circular pointer by 'num' */
#define CBUFPTR_INCR(cbuf,ptr,num)    ( ((cbuf).ptr+num) & (cbuf).mask )      
/* decrement circular pointer by 'num' */
#define CBUFPTR_DECR(cbuf,ptr,num)    ( ((cbuf).ptr-num) & (cbuf).mask )      
/* true, if circular bufferis empty */
#define CBUF_EMPTY(cbuf)              ( (cbuf).wrptr == (cbuf).rdptr )
/* true, if circular buffer wraps around */
#define CBUF_WRAPAROUND(cbuf)         ( (cbuf).wrptr < (cbuf).rdptr )
/* check for 'num' free elements in buffer */
#define CHECK_CBUF_FREE(cbuf,num)     ( CBUF_WRAPAROUND(cbuf) ? (cbuf).wrptr + num < (cbuf).rdptr : (cbuf).wrptr + num < (cbuf).rdptr + (cbuf).size )
/* get the number of used elements */
#define CBUF_GET_USED(cbuf)           ( CBUF_WRAPAROUND(cbuf) ? (cbuf).wrptr + (cbuf).size - (cbuf).rdptr : (cbuf).wrptr - (cbuf).rdptr )
/* true, if r is a valid read index ( ie between rdptr..wrptr-1  */
#define CBUF_VALID_RDPOS(cbuf,r)      ( CBUF_WRAPAROUND(cbuf) ? (cbuf).rdptr+r < (cbuf).wrptr + (cbuf).size : (cbuf).rdptr+r < (cbuf).wrptr )
/* return the number of bytes that can be read consecutively up to the end of the underlying linear buffer */
#define CBUF_GET_LINEARREADSIZE(cbuf) ( CBUF_WRAPAROUND(cbuf) ? (cbuf).size - (cbuf).rdptr : (cbuf).wrptr - (cbuf).rdptr )


/* true, if linear buffer is empty */
#define LBUF_EMPTY(lbuf)              ( (lbuf).wrptr == (lbuf).rdptr )
/* check for 'num' free elements in buffer, leave one element free for terminating \0 */
#define CHECK_LBUF_FREE(lbuf,num)     ( (lbuf).wrptr + num < (lbuf).size )
/* true, if r is a valid read index ( ie between rdptr..wrptr-1  */
#define LBUF_VALID_RDPOS(lbuf,r)      ( (lbuf).rdptr+r < (lbuf).wrptr )
/* remove last written element */
#define LBUF_DEL(lbuf)                do { if ( (lbuf).wrptr > 0 ) (lbuf).wrptr--; } while (0) 



/* Public functions ---------------------------------------------------------*/
bool CircBuff_Init(CircBuffT *cbuff, uint32_t size, uint8_t *bufptr );
bool CircBuff_Put(CircBuffT *b, uint8_t ch );
bool CircBuff_Get(CircBuffT *b, uint8_t *ch );
bool CircBuff_Put2(CircBuffT *b, uint16_t w );
bool CircBuff_Get2(CircBuffT *b, uint16_t *w );
bool CircBuff_Peek2(CircBuffT *b, uint16_t *w );
bool CircBuff_Get_Indexed(CircBuffT *b, uint32_t idx, uint8_t *ch );
void CircBuff_GetlinearReadBuffer(CircBuffT *b, uint8_t **readbuf, uint32_t *size );
void CircBuff_Mark(CircBuffT *b);
void CircBuff_PutBack(CircBuffT *b, uint8_t ch );

void LinBuff_Init(LinBuffT *cbuff, uint32_t size, uint8_t *bufptr );
bool LinBuff_Putc(LinBuffT *b, uint8_t ch );
bool LinBuff_Getc(LinBuffT *b, uint8_t *ch );
void LinBuff_Gets(LinBuffT *b, uint8_t **str, size_t *len );
bool LinBuff_GetsTo(LinBuffT *b, uint8_t **str, size_t *len, char delimiter );
void LinBuff_SetEmpty(LinBuffT *b);
bool LinBuff_CleanUp(LinBuffT *b, uint32_t num );


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CIRCBUF_H */
