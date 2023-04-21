#ifndef __W25QXX_H
#define __W25QXX_H
//////////////////////////////////////////////////////////////////////////////////	 

  
/*
 ******************************************************************************
 * Winbond W25QXX QSPI-flash family
 ******************************************************************************
 */	

// Flash Chip IDs of different family members
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17
#define W25Q256 0XEF18

//RHB todo  extern u16 W25QXX_TYPE;	// actual type
 
/*
 ******************************************************************************
 * Winbond W25QXX QSPI-flash family
 ******************************************************************************
 */
/* W25Qxx commands --------------------------------------------------------- */	
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg1		0x05 
#define W25X_ReadStatusReg2		0x35 
#define W25X_ReadStatusReg3		0x15 
#define W25X_WriteStatusReg1            0x01 
#define W25X_WriteStatusReg2            0x31 
#define W25X_WriteStatusReg3            0x11 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown           0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID           0x90 
#define W25X_JedecDeviceID		0x9F 

#define W25X_Enable4ByteAddr            0xB7
#define W25X_Exit4ByteAddr              0xE9

#define W25X_SetReadParam		0xC0 
#define W25X_EnterQPIMode               0x38
#define W25X_ExitQPIMode                0xFF

#include <stdint.h>

void W25QXX_Init(void);
void W25QXX_Qspi_Enable(void);
void W25QXX_Qspi_Disable(void);
uint16_t  W25QXX_ReadID(void);
uint8_t W25QXX_ReadSR(uint8_t regno);
void W25QXX_4ByteAddr_Enable(void);
void W25QXX_Write_SR(uint8_t regno,uint8_t sr);
void W25QXX_Write_Enable(void);
void W25QXX_Write_Disable(void);
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Erase_Chip(void);
void W25QXX_Erase_Sector(uint32_t Dst_Addr);
void W25QXX_Wait_Busy(void);

#endif // __W25QXX_H
