#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

/*************************************************************************
* MACROS
*************************************************************************/

/*************************************************************************
* FUNCTIONS
*************************************************************************/
void Serial_Init(long baudrate);
void Serial_Terminate(void);
unsigned char Serial_Read();
unsigned char Serial_Available();
unsigned char Serial_ReadBytes(char *buf, unsigned char len);
void Serial_Print(unsigned char *txt);
void Serial_Start(void);
void Serial_Terminate(void);
unsigned char Serial_WaitFor( uint8_t nrOfChars, uint8_t *ref, uint8_t limit);
unsigned char Serial_IsWriteActive();


#endif