/**
  ******************************************************************************
  * @file    version.c
  * @author  Rainer 
  * @brief   retrieve the version string and other versioning informations
  *
  ******************************************************************************
  */

#define BUILD_CONFIG_STR  1
#include "config/config.h"

#include "version.h"
#include "log.h"

#include <stdio.h>


const char VersionString[] = VERSION_STRING;

void Dump_VersionInfo(void)
{
    #define MAXLINE     80
    char line[MAXLINE+1];
    uint32_t idx;
    LOG_ALWAYS("\r\nBuild Information:");
    LOG_ALWAYS("%s",VERSION_STRING1);
    LOG_ALWAYS("%s",VERSION_STRING2);
    LOG_ALWAYS("\r\nConfig Information:");
    for ( idx = 0; idx < GetConfigNumLines(); idx++ ) 
       LOG_ALWAYS(GetConfigLine(line, MAXLINE,idx,false));
       // LOG_ALWAYS("IDX=%d",idx);
    
}

uint32_t GetConfigNumLines(void)
{
    return MAX_CONFIGSTR;
}

char *GetConfigLine(char *retbuf, size_t maxlen, uint32_t idx, bool bAppendCrlf)
{
    snprintf( retbuf, maxlen,bAppendCrlf ? "%s = %d\n": "%s = %d" ,ConfigStrings[idx], ConfigValues[idx]);
    
  /* append terminating \0 in any case */
  retbuf[maxlen] = '\0';

  return retbuf;
}

