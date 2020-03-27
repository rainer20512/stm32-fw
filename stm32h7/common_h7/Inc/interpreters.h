/**
  ******************************************************************************
  * @file    interpreters.h 
  * @author  Rainer
  * @brief   Debug Input handling and command interpreter 
  * @note    The header file is common to CM7 and CM4, the implementation isn't!
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERPRETERS_H
#define __INTERPRETERS_H

#include "config/config.h"

#include "cmdline.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* The first commandset must be visible to the command line interpreter */
extern const InterpreterModuleT mdlBasic;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INTERPRETERS_H */
