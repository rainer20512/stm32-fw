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


const char VersionString[] = VERSION_STRING;

void Dump_VersionInfo(void)
{
    LOG_ALWAYS("\r\nBuild Information:");
    LOG_ALWAYS("%s",VERSION_STRING1);
    LOG_ALWAYS("%s",VERSION_STRING2);
    LOG_ALWAYS("\r\nConfig Information:");
    LOG_ALWAYS("%s",CONFIG_STRING);

}