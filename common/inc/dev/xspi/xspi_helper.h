/*******************************************************************************
 * @file    xspi_helper.h
 * @author  Rainer
 * @brief   QSPI/OSPI helper functions for all kinds of specific Flash drivers
 ******************************************************************************/
#include "config/config.h"

#include "dev/xspi/flash_interface.h"

/* Different modes of QUADSPI operation */
#define XSPI_MODE_POLL      0       // Execute function in Polling/active wait mode
#define XSPI_MODE_IRQ       1       // Execute function in Interrupt/NoWalt mode
#define XSPI_MODE_DMA       2       // Execute function in DMA/NoWalt mode, makes only sense for read/write operations

#ifdef __cplusplus
 extern "C" {
#endif

bool XHelper_SetupCommand           (XSPI_CommandTypeDef *sCmd, const NOR_FlashCmdT *cmd, uint32_t arglen, uint32_t arg, uint32_t retlen );
void XHelper_SetupAltBytes          (XSPI_CommandTypeDef *sCmd, const NOR_RWModeTypeT *rd, uint32_t alt_bytes);
bool XHelper_SendCommandBytesSerial (XSpiHandleT *myHandle, uint8_t *bytes, uint32_t num);
bool XHelper_WaitForBitsSetOrReset  (XSpiHandleT *myHandle, const NOR_FlashQueryT *qry, uint32_t Timeout, uint32_t opmode, bool bSet);
bool XHelper_CmdArgRead             (XSpiHandleT *myHandle, const NOR_FlashCmdT *cmd, uint32_t arg, uint8_t *retbuf, int32_t retlen);
bool XHelper_HasDeepPowerDown       (XSpiHandleT *myHandle );
bool XHelper_HasHighPerformanceMode (XSpiHandleT *myHandle );
void XHelper_SetGeometry            (XSpiHandleT *myHandle, XSpiGeometryT *geometry);
const NOR_RWModeTypeT* XHelper_FindReadCmd  (XSpiHandleT *myHandle);
const NOR_RWModeTypeT* XHelper_FindWriteCmd (XSpiHandleT *myHandle);

#ifdef __cplusplus
 }
#endif
