/*
 ******************************************************************************
 * @file    qspecific.h 
 * @author  Rainer
 * @brief   QSPI chip specific functions. Have to be implemented indiviually
 *          for every qspi device or deqice family
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QSPECIFIC_H
#define __QSPECIFIC_H

char*       XSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize);
void        XSpecific_DumpStatusInternal(XSpiHandleT *myHandle);

bool        XSpecific_BasicInit(XSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size, bool bFirstInit );
bool        XSpecific_SpecificInit(const HW_DeviceType *self,XSpiHandleT *myHandle, uint32_t clk_frq);

bool        XSpecific_ResetMemory(XSpiHandleT *myHandle);
uint32_t    XSpecific_GetID ( XSpiHandleT *me );

bool        XSpecific_EnterDeepPowerDown(XSpiHandleT *myHandle);
bool        XSpecific_LeaveDeepPowerDown(XSpiHandleT *myHandle);

bool        XSpecific_HPerfMode ( XXSPI_HandleTypeDef *hqspi, bool bHPerf );

bool        XSpecific_EnableMemoryMappedMode(XSpiHandleT *myHandle);

bool        XSpecific_SuspendErase(XXSPI_HandleTypeDef *hqspi);
bool        XSpecific_ResumeErase(XXSPI_HandleTypeDef *hqspi);

bool        XSpecific_ReadCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);
bool        XSpecific_WriteCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);


bool        XSpecific_WriteEnable        (XXSPI_HandleTypeDef *hqspi);
bool        XSpecific_WaitForWriteDone   (XXSPI_HandleTypeDef *hqspi, uint32_t Timeout);

bool        XSpecific_GetEraseParams(uint32_t erasemode, uint32_t *timeout_ms, uint8_t *opcode);
bool        XSpecific_EraseCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t eraseMode );
bool        XSpecific_WriteEnable(XXSPI_HandleTypeDef *hqspi);
bool        XSpecific_WaitForWriteDone(XXSPI_HandleTypeDef *hqspi, uint32_t Timeout);
bool        XSpecific_WaitForWriteDone_IT(XXSPI_HandleTypeDef *hqspi);


#endif /* __QSPECIFIC_H */

