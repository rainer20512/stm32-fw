/******************************************************************************
 * rfm_specific.h
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * This file specifies the RFM-driver Interface
 *
 * The device specific implementation has to implement all these routines for
 * the specific RFM device
 *
 * currently supported RFM devices:
 *  - rfm12b
 *  - rfm69(h)cw 
 *
 ******************************************************************************/

#ifndef __RFM_SPECIFIC_H_
#define __RFM_SPECIFIC_H_

#define RFM12_ID        12
#define RFM69_ID        69

typedef struct RFMT {
    /* ------ Presence check ------*/
    uint8_t (*rfmXX_Device_present)     (void);     /* returns != 0, if RFM device has been detected           */
    /* ------ FSK functions ------*/
    void    (*rfmXX_OFF)                (void);     /* Switch device off to lowest power state                 */
    void    (*rfmXX_Init)               (void);     /* Initialization for operation in FSK mode                */
    void    (*rfmXX_ReceiveBody)        (void);     /* RFM chip specific receiver routine                      */
    void    (*rfmXX_TransmitBody)       (void);     /* RFM chip specific transmitter routine                   */ 
    void    (*rfmXX_HeatUpTransmitter)  (void);     /* Prepare the RFM transmitter before transmission         */
    void    (*HandleFSKInterrupt_RFMXX) (uint16_t pin, uint16_t pinvalue, void *arg);    /* Data available interrupt handler */
    /* ------ OOK functions ------*/
    void    (*rfmXX_OOK_Init)           (void);
    void    (*rfmXX_OOK_On)             (void);
    void    (*rfmXX_OOK_Off)            (void);
    /* ------ Debug ------*/    
    void    (*rfmXX_Dump_status)        (void);     /* Dump the rfm internal status for debug purposes         */
    uint32_t rfmID;                                 /* ID of used RFM type, ( 12 or 69) see above              */
    char     *name;                                 /* user friendly name of RFM type                          */
} RFM_DeviceType;

/* global pointer to rfm driver */
extern const RFM_DeviceType rfm12_driver;
extern const RFM_DeviceType rfm69_driver;
extern RFM_DeviceType *rfm;

#endif /* #ifndef __RFM_SPECIFIC_H_ */