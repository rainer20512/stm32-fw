/**
  ******************************************************************************
  * @file    com.h 
  * @author  Rainer
  * @brief   Debug Input handling and command interpreter 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMDLINE_H
#define __CMDLINE_H

#include "config/config.h"

#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* Public types---- ---------------------------------------------------------*/

typedef struct InterpreterModule InterpreterModuleT;

/*
 * Function type to return a submenus prompt
 */
typedef const char * ( *pFnGetPrompt) (void);

/*
 * Function type to execute a command
 */
typedef bool ( *pFnInterpreterExec ) ( char *cmd, size_t len, const void * arg );

/*
 * Enumeration for different types of menu entries
 */
typedef enum {
  ctype_fn  = 0,            // Normal call of execution function
  ctype_sub = 1,            // push a new submenu
} CEnumT;

/*
 * execution type according to cType
 */
typedef union {
  pFnInterpreterExec fn;          // Normal function for cType == ctype_fn
  const InterpreterModuleT *sub;  // submenu for cType = ctype_sub
} CExecT;
    
typedef struct CommandSet {
  const char * const command;
  const CEnumT cType;
  const CExecT exec;
  const void * arg;
  const char * const helptext;
} CommandSetT;

typedef struct InterpreterModule {
  const pFnGetPrompt       prompt; 
  const CommandSetT * const commandlist;
  const uint32_t           num_cmd;
} InterpreterModuleT;

void CMD_PrintHelp(void);
void CMD_Prompt(void);
bool CMD_Push ( const InterpreterModuleT * mdlNew );
void CMD_Pop ( void );
void CMD_Init ( void );
bool CMD_get_one_word ( char **start, size_t *len );
uint16_t CMD_argc ( void );
bool CMD_is_numeric( char *word, size_t wordlen );
uint32_t CMD_to_number ( char *word, size_t wordlen );

void task_handle_com(uint32_t);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CMDLINE_H */
