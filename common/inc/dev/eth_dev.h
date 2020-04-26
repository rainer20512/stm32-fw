/*
 ******************************************************************************
 * @file    can_dev.h 
 * @author  Rainer
 * @brief   CAN device functions. 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ETH_DEV_H
#define __ETH_DEV_H

#include "config/config.h"

#if USE_ETH > 0 

#include "hardware.h"
#include "hw_device.h"
#include "config/devices_config.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct CanHandleType {
	CAN_HandleTypeDef hCan;           /* CAN handle structure is included */
        CanSpeedModeEnum  baudrate;       /* current CAN baudrate             */
        CanOpmode         mode;           /* current CAN Operating mode ( normal, init, sleep ) */
        uint32_t          last_error;     /* last error code                  */
        /* --------------------- Callback functions ------------------------- */
        void (*CanOnRx)(uint32_t, CanTxRxDataT*);
        void (*CanOnTx)(void);
        void (*CanOnErr)(uint32_t);
} CanHandleT; 

typedef void (*CanRxCb)(uint32_t, CanTxRxDataT*);
typedef void (*CanTxCb)(void);
typedef void (*CanErrCb)(uint32_t);

/* Public functions ---------------------------------------------------------*/
void         CAN_Setup32BitFilter ( CanFilterT *flt, uint32_t id, uint8_t rtrId, uint32_t mask, uint8_t rtrMask, uint8_t bIsIsList, uint8_t bIsEID, uint8_t fifonum);
void         CAN_Setup16BitFilter ( CanFilterT *flt, uint8_t fltIdx, uint32_t id, uint8_t rtrId, uint32_t mask, uint8_t rtrMask, uint8_t bIsIsList, uint8_t bIsEID, uint8_t fifonum);
#define      CAN_Setup16BitFilter0( flt, id, rtrId, mask, rtrMask, bIsIsList, bIsEID, fifonum) CAN_Setup16BitFilter(flt, 0, id, rtrId, mask, rtrMask, bIsIsList, bIsEID, fifonum)
#define      CAN_Setup16BitFilter1( flt, id, rtrId, mask, rtrMask, bIsIsList, bIsEID, fifonum) CAN_Setup16BitFilter(flt, 1, id, rtrId, mask, rtrMask, bIsIsList, bIsEID, fifonum)
bool         CAN_SetFilter        (CAN_TypeDef *hcan, CanFilterT *flt, uint8_t fltnum, uint8_t bActivate);
void         CAN_RegisterCallbacks(CanHandleT *myHandle, CanRxCb RxCb, CanTxCb TxCb, CanErrCb ErrCb);
bool         CAN_Start            ( CanHandleT *me );
bool         CAN_Stop             ( CanHandleT *me );
bool         CAN_IsTxMboxFree     (CanHandleT *me);
CanTxMboxNum CAN_Transmit         ( CanHandleT *me, CanTxRxDataT *tx );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // USE_ETH > 0 

#endif /* __ETH_DEV_H */
