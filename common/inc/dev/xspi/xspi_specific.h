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

char*       QSpecific_GetChipTypeText(uint8_t *idbuf, char *retbuf, const uint32_t bufsize);
void        QSpecific_DumpStatusInternal(XSpiHandleT *myHandle);

bool        QSpecific_BasicInit(XSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size, bool bFirstInit );
bool        QSpecific_SpecificInit(const HW_DeviceType *self,XSpiHandleT *myHandle, uint32_t clk_frq);

bool        QSpecific_ResetMemory(XSpiHandleT *myHandle);
uint32_t    QSpecific_GetID ( XSpiHandleT *me );

bool        QSpecific_EnterDeepPowerDown(XSpiHandleT *myHandle);
bool        QSpecific_LeaveDeepPowerDown(XSpiHandleT *myHandle);

bool        QSpecific_HPerfMode ( XXSPI_HandleTypeDef *hqspi, bool bHPerf );

bool        QSpecific_EnableMemoryMappedMode(XSpiHandleT *myHandle);

bool        QSpecific_SuspendErase(XXSPI_HandleTypeDef *hqspi);
bool        QSpecific_ResumeErase(XXSPI_HandleTypeDef *hqspi);

bool        QSpecific_ReadCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);
bool        QSpecific_WriteCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);


bool        QSpecific_WriteEnable        (XXSPI_HandleTypeDef *hqspi);
bool        QSpecific_WaitForWriteDone   (XXSPI_HandleTypeDef *hqspi, uint32_t Timeout);

bool        QSpecific_GetEraseParams(uint32_t erasemode, uint32_t *timeout_ms, uint8_t *opcode);
bool        QSpecific_EraseCMD(XXSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t eraseMode );
bool        QSpecific_WriteEnable(XXSPI_HandleTypeDef *hqspi);
bool        QSpecific_WaitForWriteDone(XXSPI_HandleTypeDef *hqspi, uint32_t Timeout);
bool        QSpecific_WaitForWriteDone_IT(XXSPI_HandleTypeDef *hqspi);


#endif /* __QSPECIFIC_H */

