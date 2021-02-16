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
void        QSpecific_DumpStatusInternal(QSpiHandleT *myHandle);

bool        QSpecific_BasicInit(QSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size, bool bFirstInit );
bool        QSpi_SpecificInit(const HW_DeviceType *self,QSpiHandleT *myHandle, uint32_t clk_frq);

bool        QSpecific_ResetMemory(QSpiHandleT *myHandle);
uint32_t    QSpecific_GetID ( QSpiHandleT *me );

bool        QSpecific_EnterDeepPowerDown(QSpiHandleT *myHandle);
bool        QSpecific_LeaveDeepPowerDown(QSpiHandleT *myHandle);

bool        QSpecific_HPerfMode ( QSPI_HandleTypeDef *hqspi, bool bHPerf );

bool        QSpecific_EnableMemoryMappedMode(QSpiHandleT *myHandle);

bool        QSpecific_SuspendErase(QSPI_HandleTypeDef *hqspi);
bool        QSpecific_ResumeErase(QSPI_HandleTypeDef *hqspi);

bool        QSpecific_ReadCMD(QSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);
bool        QSpecific_WriteCMD(QSPI_HandleTypeDef *hqspi, uint32_t Addr, uint32_t Size);


bool        QSpecific_WriteEnable        (QSPI_HandleTypeDef *hqspi);
bool        QSpecific_WaitForWriteDone   (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);

bool        QSpecific_GetEraseParams(uint32_t erasemode, uint32_t *timeout_ms, uint8_t *opcode);
bool        QSpecific_EraseCMD(QSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t eraseMode );
bool        QSpecific_WriteEnable(QSPI_HandleTypeDef *hqspi);
bool        QSpecific_WaitForWriteDone(QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
bool        QSpecific_WaitForWriteDone_IT(QSPI_HandleTypeDef *hqspi);


#endif /* __QSPECIFIC_H */

