/******************************************************************************
 * rfm_specific.h
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * This file contains all code portions, which are specific to certain RFM devices
 *
 * The device specific implementation has to implement all these routines for
 * the specific RFM device
 * currently supported RFM devices:
 *  - rfm12b
 *  - rfm69(h)cw 
 *
 ******************************************************************************/

#ifndef __RFM_SPECIFIC_H_
#define __RFM_SPECIFIC_H_

uint8_t rfmXX_device_present(void);    /* returns != 0, if RFM device has been detected           */
void    rfmXX_OFF(void);               /* Switch device off to lowest power state                 */
void    rfmXX_init(void);              /* Initialization for operation in FSK mode                */
void    rfmXX_receiveBody(void);       /* RFM chip specific receiver routine                      */
void    rfmXX_TransmitBody(void);      /* RFM chip specific transmitter routine                   */ 
void    rfmXX_HeatUpTransmitter(void); /* Prepare the RFM transmitter before transmission         */
void    HandleFSKInterrupt_RFMXX(uint16_t pin, uint16_t pinvalue, void *arg);    /* Data available interrupt handler */


void    rfmXX_dump_status( void );     /* Dump the rfm internal status for debug purposes         */

#endif /* #ifndef __RFM_SPECIFIC_H_ */