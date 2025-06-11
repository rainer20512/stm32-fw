/*
 ******************************************************************************
 * @file    lv_4wire_spi.h
 * @author  Rainer
 * @brief   low level functions for 4-wire-spi
 *
 ******************************************************************************
 */

#include "config/config.h"
#include "dev/spi_dev.h"
#include "dev/spi.h"

#if EPAPER_DEV==SPIDEV1_DEV
    #define MYHANDLE  (&SPI1Handle)
#elif EPAPER_DEV==SPIDEV2_DEV
    #define MYHANDLE  (&SPI2Handle)
#else
    #error "No SPI-Handle for GC9A01 defined!"
#endif

#define LV_DRV_DISP_SPI_CS(lvl)             ( lvl == 0 ? SpiNSelLow(MYHANDLE) : SpiNSelHigh(MYHANDLE) )
#define LV_DRV_DISP_RST(lvl)                ( lvl == 0 ? SpiRstLow(MYHANDLE)  : SpiRstHigh(MYHANDLE) )
#define LV_DRV_DISP_CMD_DATA(lvl)           ( lvl == 0 ? SpiDnCLow(MYHANDLE)  : SpiDnCHigh(MYHANDLE) )
#define LV_DRV_DISP_SPI_WR_BYTE(byte)       Spi8TxByte(MYHANDLE, byte)


void LV_DRV_DELAY_MS(uint32_t wait_ms);
void LV_DRV_DISP_SPI_WR_ARRAY(char *arr, uint32_t len);
