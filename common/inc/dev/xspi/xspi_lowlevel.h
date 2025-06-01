/*******************************************************************************
 * @file    xspi_lowlevel.h
 * @author  Rainer
 * @brief   QSPI/OSPI lowlevel functions which are independent from specific
 *          Flash device, all access is done via interface definition
 ******************************************************************************/
#include "config/config.h"

#include "dev/xspi_dev.h"

#ifdef __cplusplus
 extern "C" {
#endif

bool        XSpiLL_ClockInit(XSpiHandleT *myHandle, uint32_t clk_frq, uint32_t desired_frq, uint32_t flash_size );
uint32_t    XSpiLL_ExecuteCmd( XSpiHandleT *myHandle, const NOR_FlashCmdT *cmd, uint32_t arg, uint8_t *retbuf, uint32_t *retlen, uint32_t *exec_time );
bool        XSpiLL_WaitForWriteDone(XSpiHandleT *myHandle, uint32_t Timeout);
bool        XSpiLL_WaitForWriteDone_IT(XSpiHandleT *myHandle);
int32_t     XSpiLL_SetHPMode  (XSpiHandleT *myHandle, bool bEna);
bool        XSpiLL_WriteEnable(XSpiHandleT *myHandle, bool bEna);
bool        XSpiLL_ResetMemory(XSpiHandleT *myHandle);
bool        XSpiLL_GetID(XSpiHandleT *myHandle);
bool        XSpiLL_Erase(XSpiHandleT *myHandle, uint32_t Address,const NOR_FlashCmdT  *ecmd );
bool        XSpiLL_ReadCMD(XSpiHandleT *myHandle, uint32_t Addr, uint32_t Size);
bool        XSpiLL_WriteCMD(XSpiHandleT *myHandle, uint32_t Addr, uint32_t Size);

#if DEBUG_MODE > 0
    const char *XSpiLL_GetChipManufacturer(uint8_t mf_id );
    void XSpiLL_DumpChipInfo(uint8_t *idbuf);
    void XSpiLL_DumpGeometry(XSpiGeometryT *geo);
#endif


#ifdef __cplusplus
 }
#endif
