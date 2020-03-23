/**
  ******************************************************************************
  * @file    cmdline.c
  * @author  Rainer 
  * @brief   Functions to handle / execute command line commands
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup CmdLine
  * @{
  */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <inttypes.h>

#include "config/config.h"

#include "error.h"
#include "timer.h"
#include "task/minitask.h"
#include "system/profiling.h"
#include "rfm/rfm12.h"
#include "rtc.h"
#include "com.h"
#include "system/clockconfig.h"
#include "wireless.h"
#include "debug_helper.h"
#include "debug_sram.h"
#include "debug_outbuf.h"
#include "debug_gpio_exti.h"
#include "debug_pwr_rcc.h"
#include "system/status.h"

#include "cmdline.h"
#include "interpreters.h"

/** @addtogroup CmdLine
  * @{
  */ 

/* Private define ------------------------------------------------------------*/
#define CMD_SEPARATOR       ';' /* character to separate cmds from each other */
#define MODULE_MAXDEPTH      5  /* Interpreter modules maybe stacked to this  */ 

/* Private macro -------------------------------------------------------------*/

/* Private types---- ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/*
 * Ptr to the line buffer, that will be used 
 */
static LinBuffT *inbuf;

/*
 * A "stack" of interpreter modules, 
 * only the first level is initialized to a basic commandline interpreter
 */
static const InterpreterModuleT * modules[MODULE_MAXDEPTH+1];
/*
 * The "stack pointer" to the interpreter modules. 
 * Initially empty. Has to be initialized to the Basic Interpreter module
 * This is done in CMD_Init()
 */
static int32_t module_ptr = -1;

/*
 * Pointer to the actual interpreter module, just a shortcut to
 * avoid typing of modules[module_ptr]
 * is maintained solely by CMD_Push and CMD_Pop
 */
static const InterpreterModuleT *act_module;


/* Private functions ---------------------------------------------------------*/


/*********************************************************************************
 * @brief  returns the maximum length of current command set commands
 *
 * @retval maximum length of all commands
 *
 ********************************************************************************/
static size_t CMD_GetMaxCmdLen ( void )
{
  size_t ret = 0;
  size_t tmp;
  for ( uint32_t i= 0 ; i < act_module->num_cmd; i++ ) {
      tmp = strlen(act_module->commandlist[i].command);
      if ( tmp > ret ) ret = tmp;
  }

  return ret;
}

/*********************************************************************************
 * @brief  print 'text' rightpadded with blanks to achieve length 'len'
 * @param text   - text to be printed
 * @param texlen - size of text
 * @param len    - size to achieve
 *
 * @note If text is longer than len, the text will be truncated
 *
 ********************************************************************************/
static void CMD_print_text_rightpadded ( const char *text, size_t textlen, size_t len )
{

#define min(a,b) (a<b? a : b )

    size_t stop = min(textlen, len);
    for ( size_t i=0; i < stop;i++ )
      putchar(*(text++));
    for ( size_t i=stop; i < len; i++ )
      putchar(' ');
}

/* Exported functions --------------------------------------------------------*/


/*********************************************************************************
 * @brief  print the help text for the actual interpreter module
 *         the help text consists of a numerated list of all command words
 *         followed by the commands help text. The help text may span over more
 *         than one line, newlines are denotated by \n in the help text
 ********************************************************************************/
void CMD_PrintHelp(void)
{
  size_t cmdlen = CMD_GetMaxCmdLen();
  for ( uint32_t i = 0; i <  act_module->num_cmd; i++ ) {
      printf("%2d) ",i );
      CMD_print_text_rightpadded(act_module->commandlist[i].command, strlen(act_module->commandlist[i].command), cmdlen );
      printf(" - %s\n", act_module->commandlist[i].helptext );
  }
}

/*********************************************************************************
 * @brief  print the current input prompt
 *         This input prompt consists of all stacked module prompts followed
 *         by a '>'
 ********************************************************************************/
void CMD_Prompt(void)
{
  for ( int32_t i= 0 ; i <= module_ptr; i++ ) {
    printf("%s>", modules[i]->prompt() );
  }
}

/*********************************************************************************
 * @brief Push a new command module on top of stack
 * @param promptFn    - the new modules prompt text
 * @param commands    - the new modules list of commands, executers help texts
 * @param num_entries - number of entries in the commmand list
 *
 * @note On successful Push, the help text of the new module is displayed
 ********************************************************************************/
bool CMD_Push ( const InterpreterModuleT * mdlNew )
{
  if ( module_ptr + 1 == MODULE_MAXDEPTH -1 ) {
    DEBUG_PUTS("Interpreter module stack overflow");
    return false;
  }
 
  module_ptr++;
  modules[module_ptr] = mdlNew;
  act_module = modules[module_ptr];
  CMD_PrintHelp();
  return true;
}

/*********************************************************************************
 * @brief Pop the actual Module from Stack. 
 *
 * @note The initial module (BasicModule) cannot be popped and will remain on TOS
 *       when trying to pop
 * @note On successful Pop, the help text of the previous module is displayed
 ********************************************************************************/
void CMD_Pop ( void )
{
  if ( module_ptr == 0 ) {
    DEBUG_PUTS("Interpreter module stack underflow");
    return;
  }

  module_ptr--;
  act_module = modules[module_ptr];
  CMD_PrintHelp();
}

/*********************************************************************************
 * @brief Initialize the commandline interpreter by pushing the (unremovable)
 *        basic interpreter module onto stack
 ********************************************************************************/
void CMD_Init ( void )
{
  CMD_Push(&mdlBasic);
  CMD_Prompt();
}

/*--------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------
 * Functions to cut a commandline into single words. 
 * Words are separated by one or more blanks
 *--------------------------------------------------------------------------------
 *------------------------------------------------------------------------------*/
#define MAX_WORDS     10
#define IS_LC(a)      ((a) >= 'a' && (a) <= 'z')
#define TO_UC(a)      (a) &= ~0x20
/*
 * Actual command line and length of command line
 */
static char * act_line;
static size_t act_line_len;

/*
 * List of beginning of words ( index into 'act_line' 
 */
static uint16_t words[MAX_WORDS];


/*
 * total number of words and actual word in command line
 */
static uint16_t word_num, act_word; 

/*
 * list of partial matches and number of partial matches
 * set by 'get_matches'
 */
static uint16_t match_list[MAX_WORDS];
static uint16_t match_cnt;


/*********************************************************************************
 * @brief  Returns the next word (according to 'act_word' fom the command line
 *
 * @param  start - return value: ptr to start of word 
 * @param   len  - return value: length of word
 * @retval true on success, false if no more words available
 * @note   start and len only valid, if true is returned
 ********************************************************************************/
bool CMD_get_one_word ( char **start, size_t *len )
{
  if ( act_word >= word_num ) return false;

  uint32_t p    = words[act_word];
  char *cmdline = act_line + p;
  *start        = cmdline;
  
  /* find end of word, search for next blank */
  while ( p < act_line_len && *cmdline != ' ' ) {
    p++; cmdline++;
  }
  
  *len = p - words[act_word];

  /* increment word counter */
  act_word++;

  return true;
}

/*********************************************************************************
 * @brief  Returns the number of words actually available in command line
 *
 ********************************************************************************/
uint16_t CMD_argc ( void )
{
  if ( act_word >= word_num ) 
    return 0;
  else 
    return word_num - act_word;
}

/*********************************************************************************
 * @brief  Parse line into single words.
 *         Words are separated by one or meore blanks
 *         
 * @retval true on success. false will be returned, if more than MAX_WORDS are found
 *
 * @note   when false is returned, the list of words will be truncated
 ********************************************************************************/
static bool parse_line ( char *cmdline, size_t len )
{
  uint32_t p = 0;
  
  word_num     = 0;
  act_word     = 0;
  act_line     = cmdline;
  act_line_len = len;

  while ( 1 ) {

    /* skip blanks */
    while ( p < len && *cmdline == ' ' ) {
      p++; cmdline++;
    }

    /* No more words */
    if ( p >= len ) return true;

    /* We are at the beginning of one word: */

    if ( word_num == MAX_WORDS ) {
      /* maximum number of words reached */
      return false;
    }

    /* Store beginning of word */
    words[word_num++] = p;
  
    /* search for next blank */
    while ( p < len && *cmdline != ' ' ) {
      p++; cmdline++;
    }

    /* Word extends to end of line */
    if ( p >= len ) return true;
  }

}

/********************************************************************************
 * @brief convert a given word to upper case letters
 ********************************************************************************/
static void word_to_uc ( char *word, size_t wordlen ) 
{
  uint32_t p=0;
  
  while ( p < wordlen ) {
    if ( IS_LC(*word) ) TO_UC(*word);
    p++; word++;
  }

}

/********************************************************************************
 * @brief  compare word1 and word2 on an case insensitve base. word1 is expected
 * @param  word1    - ptr to first word, HAS TO BE IN UC !
 * @param  word1len - length of first word
 * @param  word2    - ptr to second word
 * @param  word2len - length of second word
 * 
 * @retval 0 - no match
 *         1 - partial match ( word 1 is identical to the beginning of word2 )
 *         2 - full match ( word 1 is identical to word2 )
 ********************************************************************************/
static uint32_t compare_words( const char * word1, size_t word1len,  const char * word2, size_t word2len )
{
  size_t len = min(word1len, word2len );
  char c;

  /* check for identical up to the lenth of the shorter word */
  for ( uint32_t i = 0; i < len; i++ ) {
    c = *(word2++);
    if (IS_LC(c) ) TO_UC(c);
    if ( c != *(word1++) ) return 0;
  }

  /* words match up to the length of the shorter word */
  /* if length of both words is identical, the we have a full match */
  if ( word1len == word2len ) return 2;

  /* if the words match up to the length of the first word, we have a partial match */
  if ( len == word1len ) return 1;

  /* If word 1 is longer than word2, no match */
  return 0;
}



/********************************************************************************
 * @brief returns true, if the given word is the word for help
 *        i.e. 'help' or '?'
 ********************************************************************************/
static bool is_helpword( char *word, size_t wordlen ) 
{
  /* check for '?' */
  if ( wordlen == 1 && *word == '?' ) return true;
  
  /* check for 'HELP' */
  if ( compare_words(word, wordlen, "HELP", 4 ) > 0 ) return true;

  return false;
}

/********************************************************************************
 * @brief returns true, if the given word is the word for exit 
 *        i.e. 'exit' or '^D'
 ********************************************************************************/
static bool is_exitword( char *word, size_t wordlen ) 
{
  /* check for '^D' */
  if ( wordlen == 1 && *word == 0x04 ) return true;
  
  /* check for 'EXIT' */
  if ( compare_words(word, wordlen, "EXIT", 4 ) > 0 ) return true;

  return false;
}

/********************************************************************************
 * @brief returns true, if the given word is a decimal number
 ********************************************************************************/
bool CMD_is_numeric( char *word, size_t wordlen ) 
{
  uint32_t p=0;
  char c;
  bool bHex = wordlen > 2 && *word == '0' && UCASE(*(word+1))=='X';
  
  while ( p < wordlen ) {
    c = UCASE(*word);
    if ( bHex ) {
       if ( (c < '0' || c > '9') && (c < 'A' || c > 'F') ) return false;
    } else {
        if ( c < '0' || c > '9' ) return false;
    }
    p++; word++;
  }
  return true;
}

/********************************************************************************
 * @brief  convert the given word to a number
 * @note   conversion stops, if a non numeric char is found
 * @retval numeric value
 ********************************************************************************/
uint32_t CMD_to_number ( char *word, size_t wordlen ) 
{ uint8_t base = 10;
  uint32_t p=0;
  uint32_t ret = 0;
  uint32_t digit;
  char c;
  bool bHex = wordlen > 2 && *word == '0' && UCASE(*(word+1))=='X';

/*local*/bool CMD_GetDecimal(char digit, uint32_t *retval )
        {
            if ( digit < '0' || digit > '9' ) return false;
            *retval = digit - '0';
            return true;
        }
/*local*/bool CMD_GetHex(char digit, uint32_t *retval )
        {
            if ( digit >= '0' && digit <= '9' ) {
                *retval = digit - '0';
                return true;
            }
            digit  &= ~0x20;
            if ( digit >= 'A' && digit <= 'F' ) {
                *retval = digit - 'A'+10;
                return true;
            }
            return false;
        }

    // Check for Hex: ie '0' followed by x
    if ( bHex ) {
        base = 16;
        word += 2;
        wordlen -=2;
    }

    while ( p < wordlen ) {
        c = *word;
        if (!( base == 10 ? CMD_GetDecimal(c, &digit) : CMD_GetHex(c, &digit) ) ) return ret;
        ret = ret * base + digit;
        p++; word++;
    }
    return ret;
}

/********************************************************************************
 * @brief  find the matches of the geiven word with the current command list
 *         All comares are exxuted on a no case sensitive base.
 * 
 *         if an exact match (length of word identical to command list ) or ONE
 *         partial match is found, then the corresponding index is returned
 *         
 *         if more than one partial matches are found, then the indexes are
 *         stored to 'match_list' and -1 is returned
 * @retval as described above
 ********************************************************************************/
static int16_t get_matches( char *word, size_t wordlen )
{

  match_cnt = 0;
  for ( uint32_t i = 0; i < act_module->num_cmd; i++ ) {
    switch( compare_words( word, wordlen, act_module->commandlist[i].command, strlen(act_module->commandlist[i].command)) ) {
      case 2:
        /* exact match: return with corresponding index */
        return i;
        break;
      case 1:
        /* partial match: Store in matchlist */
        if ( match_cnt == MAX_WORDS ) {
          printf("Too many matches, max is %d\n", MAX_WORDS);
          return -1;
        }
        match_list[match_cnt++] = i;
        break;
      case 0:
        // no match: no action
        break;
     } // switch
  } // for

  return -1;
}

static bool execute_entry ( char *word, size_t wordlen, uint32_t idx )
{
   if ( idx < act_module->num_cmd ) {
      switch(act_module->commandlist[idx].cType ) {
        case ctype_fn:
          return act_module->commandlist[idx].exec.fn(word, wordlen, act_module->commandlist[idx].arg);
          break;
        case ctype_sub:
          return CMD_Push(act_module->commandlist[idx].exec.sub);
          break;
        default:
          DEBUG_PRINTF("Exec type: %d out of range\n", act_module->commandlist[idx].cType );
      }
   } else {
      DEBUG_PRINTF("Exec entry: %d out of range 0 .. %d\n", idx, act_module->commandlist[idx] );
   }
   return false;
}
/********************************************************************************
 * @brief  Handle a given word as a command. If the word is a number and a valid
 *         command index ( as displayed in help text ), the denotated command is
 *         executed. If the word is an unambigous command from the command list,
 *         the denotated command is executed. If it is an ambiguous word, all
 *         possible completions are listed
 * @note   word matching is executed on an uppercase base
 ********************************************************************************/
static void handle_word( char *word, size_t wordlen )
{
   /* first convert given word to UC */
   word_to_uc( word, wordlen );

   /* handle help command */
   if ( is_helpword( word, wordlen ) ) {
      CMD_PrintHelp();
      return;
   }

   /* handle exit command */
   if ( is_exitword( word, wordlen ) ) {
      CMD_Pop();
      return;
   }

   /* handle numeric command */
   if ( CMD_is_numeric( word, wordlen ) ) {
      uint32_t idx = CMD_to_number( word, wordlen );
      execute_entry(word, wordlen, idx );
      return;
   }

   /* handle text matches */
   int16_t xmatch = get_matches(word, wordlen);
   
   /* exact match or one partial match ? -> execute */
   if ( xmatch >= 0 || match_cnt == 1) {
     if ( match_cnt == 1 ) xmatch = match_list[0];
     execute_entry(word, wordlen, xmatch );
     return;
   }
  
   /* partial matches? print possible completions */
   if ( match_cnt > 0 ) {
     puts("possible completions:");
     for ( uint32_t i = 0; i < match_cnt; i++ ) {
       printf(" %s", act_module->commandlist[match_list[i]].command );
     }
   }

/*
   printf("%d ",act_word );
    for ( uint32_t j = 0; j < wordlen; j++ ){
      putchar(word[j]);
    }
*/
   printf("\n");
}

static void handle_line ( char *cmdline, size_t len )
{

  char *word;
  size_t wordlen;
  if (!parse_line ( cmdline, len ) ) {
    printf("Too many words, max is %d\n", MAX_WORDS);
  }

  while ( act_word < word_num ) {
   CMD_get_one_word ( &word, &wordlen );
   handle_word(word, wordlen );
  }

}


/*********************************************************************************
  * @brief  Scan input for list of commands and parameters
  *         
  * @retval None
  *
  * @note   will try to read separated commands to the end of input
  ********************************************************************************/
void task_handle_com(uint32_t arg)
{
  char *cmdline;
  size_t len;
  bool more;
  char drop;

  UNUSED(arg);

  /* Get input buffer and be sure, it exists */
  inbuf = DEBUG_RXBUF_GET();
  assert ( inbuf );

  /* read all portions of the input buffer */
  do {
    more = LinBuff_GetsTo(inbuf, (uint8_t **)&cmdline, &len, CMD_SEPARATOR );
    if ( len > 0 ) handle_line( cmdline, len );
    if ( more ) LinBuff_Getc(inbuf, (uint8_t *)&drop);
  } while ( more );

  /* Reset input buffer to 'empty' */
  LinBuff_SetEmpty(inbuf);
  CMD_Prompt();
}


/**
  * @}
  */
