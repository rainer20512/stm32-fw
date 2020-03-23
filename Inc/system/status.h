/**
  ******************************************************************************
  * @file    system/status.h
  * @author  Rainer
  * @brief   Enumerations for status and their printable output
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STATUS_H
#define __STATUS_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

// Status for OOK-Receiver and FSK-Transciever

typedef enum OokStatusEnum {
  OOK_STATUS_INHIBITED=0,		// Don't use OOK-Receiver at all. 
  OOK_STATUS_WAITFORTSYNC,		// Wait for HR20-Time synchronsation
  OOK_STATUS_WAITFORSYNC,		// No valid signal found so far
  OOK_STATUS_INITIALPHASE,		// In Sync Phase, i.e. repeating the signal every 8 sec
  OOK_STATUS_NORMALPHASE, 		// In normal Phase, i.e. repeating the signal every 128 sec
  OOK_STATUS_NO_PEER,	 		// We did not find any peer
  OOK_STATUS_NROF_ELEMENTS              /* Keep this element as last entry in enum ! */
} OokStatusEnumType;

extern OokStatusEnumType OOK_status;			

typedef enum FskStatusEnum {
  FSK_STATUS_WAITFORSYNC=0,		// Waiting for sync
  FSK_STATUS_INSYNC,			// Currently synchronzied with master
  FSK_STATUS_TIMEOUTWAIT,		// Timeout before Master Resync, waiting before next resync
  FSK_STATUS_NORFM,			// No RFM module found
  FSK_STATUS_NROF_ELEMENTS              /* Keep this element as last entry in enum ! */
} FskStatusEnumType;

extern FskStatusEnumType FSK_status;			

const char *Get_FSK_status_text     (void);
const char *Get_OOK_status_text     (void);
void        StatusPrintGlobalStatus (void);
void        StatusPrintOOKStatus    (void);
void        SetFSKStatus            ( FskStatusEnumType newStatus, uint8_t bForce );
void        SetOOKStatus            ( OokStatusEnumType newStatus, uint8_t bForce );

bool CanStop ( void );
void StopMode_Before(void);
void StopMode_After(uint32_t StopMode);

void UserPinSignal1(void);
void UserPinSignalN(uint32_t n, uint32_t period);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STATUS_H */
