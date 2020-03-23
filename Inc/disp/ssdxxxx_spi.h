/**
  ******************************************************************************
  * @file    ssdxxxx_spi.h
  * @author  Rainer
  *
  * @brief   Implements a very simple interface for epaper/LCD spi-controllers
  *          of type SSDxxxx with optional MISO line, with D/C and nRst line in      
  *          addition to the normal MOSI, SCK and nSEL lines
  *          details see .c file
  ******************************************************************************
  */

#ifndef __SSDXXXX_SPI_H
#define __SSDXXXX_SPI_H

#include "config/config.h"
#include "dev/hw_device.h"




/* the following two routines will write directly onto the SPI bus */
void SSD_WriteCmd           ( uint8_t data );
void SSD_WriteData          ( uint8_t data );

void SSD_WriteC             ( uint8_t cmd);
void SSD_WriteD             ( uint8_t data );
void SSD_WriteCnD           ( uint8_t cmd, uint8_t data );
void SSD_WriteCnDD          ( uint8_t cmd, uint8_t d1, uint8_t d2 );
void SSD_WriteCnDataVector  ( uint8_t cmd, const uint8_t *vector, uint16_t length );
void SSD_WriteCnConstant    ( uint8_t cmd, const uint8_t value, uint16_t length );

void SSD_StartConsecWrite   ( uint8_t cmd );
void SSD_ConsecWrite        ( uint8_t data);
void SSD_StopConsecWrite    ( void );


void SSD_DevReset           (void);
void SSD_DevInit            (const HW_DeviceType *dev);

bool SSD_CanRead            (void);
bool SSD_IsBusy             (void);
bool SSD_IsInitialized      (void);
void SSD_SetBusyIrqCB       (void (*)( uint16_t, void * ));

/* Start transmission in asynchronous mode */
void SSD_AsynchronousWrite  (void);

/* Flush internal buffer  (only relevant with asynchronous write */
void SSD_Flush              (void);

void task_handle_epd ( uint32_t arg );

#endif /* ifndef __SSDXXXX_SPI_H */