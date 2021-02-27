/**
  ******************************************************************************
  * @file    logfat.c
  * @author  Rainer 
  * @brief   functions for Logging to a FAT filesystem
  *          
  *
  * @note    The logging is done thru FATFS device number 0. This device MUST
  *          contain a valid file system
  * 
  ******************************************************************************
  *
  * @addtogroup LOGFAT
  * @{
  */
#include "config/config.h"
#include "debug_helper.h"
#include "rtc.h"
#include "logfile.h"

#include "ff_gen_drv.h"
#include "ff.h"

#define DETAILED_FSERR      1

static FATFS LogFileFs;   /* File system object for logging device */
static FIL   LogFile;     /* Logfile File object */
static char  DevPath[4];  /* SD card logical drive path */

#define FILENAMELEN         13
#define LOGFILEPATTERN      "log%04d.log"
static char logfilename[FILENAMELEN]; /* Logfilename in 8.3 format  */
static uint8_t logOpen = 0;           /* != 0 if logfile is open for write */


extern Diskio_drvTypeDef FatFsQspi_Driver;

/* forward declarations -----------------------------------------------------*/
static void     LogFile_ExplainError     (FRESULT res, const char * const op );
static int32_t  LogFile_OpenNextLogFile  ( void );
static void     LogFile_Timestamp        (void);

/******************************************************************************
 * Mount the logging volume and open the logfile for write/append
 *  0 is returned on success, 
 * -1 on any failure 
 *****************************************************************************/
int32_t LogFile_Init(void)
{
  FRESULT res;

  if(FATFS_LinkDriver(&FatFsQspi_Driver, DevPath) == 0)
  /* Register the file system object to the FatFs module */
  res = f_mount(&LogFileFs, (TCHAR const*)DevPath, 0);
  if ( res != FR_OK ) {
    LogFile_ExplainError (res, "mount");
    return -1;
  }

  /* Open next available logfile in name schema logXXXX.log */  
  if ( LogFile_OpenNextLogFile() != 0 ) return -1;
   
  LogFile_Info("Logging started");
  return 0;      
}

/******************************************************************************
 * Returns a value != 0, if logfile is open for write
 *****************************************************************************/
uint8_t LogFile_IsOpen(void)
{
    return logOpen;
}


int32_t LogFile_Flush(void)
{
   if ( !logOpen ) return -1;

    FRESULT res = f_sync(&LogFile);
    if ( res != FR_OK ) LogFile_ExplainError(res, "flush");

    return res == FR_OK ? 0 : - 1;
}

/******************************************************************************
 * Close an currently open logfile.
 *  0 is returned on success
 * -1 on any failure 
 *****************************************************************************/
int32_t LogFile_Close ( void )
{
    if ( !logOpen ) return -1;

    LogFile_Info("Logging stopped");
    FRESULT res = f_close(&LogFile);
    logOpen     = 0;

    return res == FR_OK ? 0 : - 1;
}


/******************************************************************************
 * Write one line to logfile, the newline character is appended
 *  0 is returned on success
 * -1 on any failure 
 *****************************************************************************/
void LogFile_Info(const char *line)
{
    UINT rlen;
    LogFile_Timestamp();
    f_write(&LogFile, line, strlen(line), &rlen);
}


/******************************************************************************
 * Dump a timestamp to file, currently timestamp contains not date info
 *****************************************************************************/
static void LogFile_Timestamp(void)
{
    char *strtim = RTC_GetStrTimeMillis();
    uint32_t len = strlen(strtim);
    UINT rlen;

    f_write(&LogFile,strtim, len, &rlen );
    f_write(&LogFile," ",1, &rlen);
}


/******************************************************************************
 * Find the first unused filename, that complies to the LOGXXXX.LOG format,
 * where XXXX is an ongoing number starting from 0000 up to 9999
 *  0 is returned on success, the file logfile's name is stored in "logfilename",
 *     the logfile is open for write
 * -1 on any failure 
 *****************************************************************************/
static int32_t LogFile_OpenNextLogFile( void )
{
    FILINFO fno;
    FRESULT res;

    for ( uint32_t num = 0; num <= 9999; num++ ) {
        snprintf(logfilename, FILENAMELEN, LOGFILEPATTERN, num);
        res = f_stat(logfilename, &fno );
        switch(res) {
            case FR_OK: continue;
            case FR_NO_FILE:
                res = f_open(&LogFile, logfilename, FA_OPEN_APPEND | FA_WRITE );
                if ( res == FR_OK ) {
                    logOpen = 1;
                    return 0;
                } else {
                    LogFile_ExplainError(res, "open");
                    break;
                }
            default:
                LogFile_ExplainError(res, "scan directory");
                return -1;
        } // switch
    }  // for

    DEBUG_PUTS("Too many logfiles on device");
    return -1;
}


/******************************************************************************
 * Explain any FatFS error in human readable form
 *****************************************************************************/
static void LogFile_ExplainError( FRESULT res, const char * const op )
{
#if DETAILED_FSERR > 0 
    const char *errtxt;
    switch ( res ) {
        case FR_OK:				/* (0) Succeeded */
            errtxt = "no";break; 
	case FR_DISK_ERR:			/* (1) A hard error occurred in the low level disk I/O layer */
            errtxt = "low level access";break; 
	case FR_INT_ERR:			/* (2) Assertion failed */
            errtxt = "internal";break; 
	case FR_NOT_READY:			/* (3) The physical drive cannot work */
            errtxt = "drive not ready";break; 
	case FR_NO_FILE:			/* (4) Could not find the file */
            errtxt = "file not found";break; 
	case FR_NO_PATH:			/* (5) Could not find the path */
            errtxt = "path not found";break; 
	case FR_INVALID_NAME:                   /* (6) The path name format is invalid */
            errtxt = "bad filename";break; 
	case FR_DENIED:				/* (7) Access denied due to prohibited access or directory full */
            errtxt = "acess or fs full";break; 
	case FR_EXIST:				/* (8) Access denied due to prohibited access */
            errtxt = "file exists";break; 
	case FR_INVALID_OBJECT:                 /* (9) The file/directory object is invalid */
            errtxt = "invalid object";break; 
	case FR_WRITE_PROTECTED:		/* (10) The physical drive is write protected */
            errtxt = "write protection";break; 
	case FR_INVALID_DRIVE:                  /* (11) The logical drive number is invalid */
            errtxt = "invalid drive";break; 
	case FR_NOT_ENABLED:			/* (12) The volume has no work area */
            errtxt = "volume not ebanled";break; 
	case FR_NO_FILESYSTEM:                  /* (13) There is no valid FAT volume */
            errtxt = "no filesystem";break; 
	case FR_MKFS_ABORTED:                   /* (14) The f_mkfs() aborted due to any problem */
            errtxt = "aborted";break; 
	case FR_TIMEOUT:                        /* (15) Could not get a grant to access the volume within defined period */
            errtxt = "timeout";break; 
	case FR_LOCKED:				/* (16) The operation is rejected according to the file sharing policy */
            errtxt = "file locked";break; 
	case FR_NOT_ENOUGH_CORE:		/* (17) LFN working buffer could not be allocated */
            errtxt = "no heap mem";break; 
	case FR_TOO_MANY_OPEN_FILES:            /* (18) Number of open files > _FS_LOCK */
            errtxt = "too many open files";break; 
	case FR_INVALID_PARAMETER:              /* (19) Given parameter is invalid */
            errtxt = "invalid param";break; 
        default: errtxt = "unknown"; 
    }
    DEBUG_PRINTF("LogFile %s error on %s\n", errtxt, op );
#else
    DEBUG_PRINTF("LogFile %d error on %s\n", rest, op );
#endif
}



/**
  * @}
  */
