/**
  ******************************************************************************
  * @file    debug_stdin.h
  * @author  Rainer
  * @brief   handle debug input via standard input i.e. SWDIO line
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_STDIN_H
#define __DEBUG_STDIN_H

#include "circbuf.h"

extern LinBuffT stdin_line; /* buffer to collect characters until CRLF is read */

#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEBUG_OUTBUF_H */
