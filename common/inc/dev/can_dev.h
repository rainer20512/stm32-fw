/*
 ******************************************************************************
 * @file    can_dev.h 
 * @author  Rainer
 * @brief   CAN device functions. 
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_DEV_H
#define __CAN_DEV_H

#include "config/config.h"

#if USE_CAN > 0 

#include "hw_device.h"
#include "config/devices_config.h"
#include "devices.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct I2cHandleType I2cHandleT; 
typedef void (*I2cCB) ( I2cHandleT * );

/* Public typedef ---------------------------------------------------------------*/
typedef enum {
      CanSpeed1000 = 0,
      CanSpeed500  = 1,
      CanSpeed250  = 2,
      CanSpeed200  = 3,
      CanSpeed125  = 4,
      CanSpeed100  = 5,
      CanSpeed50   = 6,
      CanSpeed20   = 7,
      CanSpeed10   = 8,     /* Last Entry acts as delimiter/Array size designator */
      CanSpeedMaxNum
} CanSpeedModeEnum;

/******************************************************************************
 * enumeration for can modes 
 * Note: the enumeration values are according to the SLAK and INAK bits in MSR
 *       register. Don't change
 *****************************************************************************/
typedef enum {
    CanOpmodeNormal             = 0b00,
    CanOpmodeInit               = 0b01,
    CanOpmodeSleep              = 0b10,
} CanOpmode;

/******************************************************************************
 * enumeration for can modes 
 * Note: the enumeration values are according to the SLIM and LBKM bits in BTR
 *       register. Don't change
 *****************************************************************************/
typedef enum {
    CanBusModeNormal            = 0b00,
    CanBusModeLoopback          = 0b01,
    CanBusModeSilent            = 0b10,
    CanBusModeSilentLoopback    = 0b11,
} CanBusMode;

typedef enum {
    CanTxMboxNone               =0,
    CanTxMbox0                  =1 << 0,
    CanTxMbox1                  =1 << 1,
    CanTxMbox2                  =1 << 2,
} CanTxMboxNum;



typedef union {
     struct {
        uint16_t flt16Id1;
        uint16_t flt16Id2;
     };
    uint32_t flt32Id;
} CanFilterElementU;


typedef struct CanFilterType {
    CanFilterElementU flt[2];
    uint8_t bIsEID;
    uint8_t bIsIdList;
    uint8_t bIs32Bit;
    uint8_t fifonum;
} CanFilterT;

typedef struct {
    uint32_t Id;        /*!< Message id, depending from IDE, this parameter must be a number between 0 and 0x7FF or 0x1FFFFFFF resp. */
    uint8_t IDE;          /*!< 0 for Standard ID, != 0 for extended ID */
    uint8_t RTR;          /*!< != 0 for RTR frame, 0 otherwise  */
    uint8_t FltMatchIDx;  /*!< Idx of matching receive filter */
    uint8_t DLC;          /*!< Length of data ( 0 ... 8 ) */
    uint8_t *data;        /*!< ptr to data bytes vector of length DLC */
    uint16_t Timestamp;   /*!< Specifies the timestamp counter value captured on start of frame reception. @note: Time Triggered Communication Mode must be enabled. */                     
} CanTxRxDataT;

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

/* asseble an 16 bit filter with std ID */
#define MKSTDFLT16(id, rtr) ( ((uint16_t)id) << 5 | ( rtr ? 1 << 4 : 0 ) )
/* asseble an 16 bit filter with ext ID */
#define MKEXTFLT16(id, rtr) ( ((uint16_t)id) << 5 | ( rtr ? 1 << 4 : 0 ) | 1 << 3 | (( id >> 26 ) &0b111 ) )

#define MKSTDFLT32(id, rtr) ( ((uint32_t)id) << 21                                                 | (rtr ? 1 << 1 : 0) )
#define MKEXTFLT32(id, rtr) ( ((uint32_t)id) << 21 | ((((uint32_t)id) >> 8) & 0x001FFFF8) | 1 << 2 | (rtr ? 1 << 1 : 0) )
 


#if defined(CAN1) && defined(USE_CAN1)
    extern CanHandleT CAN1Handle;
    extern const HW_DeviceType HW_CAN1;
#endif


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

#endif // USE_CAN > 0 

#endif /* __CAN_DEV_H */
