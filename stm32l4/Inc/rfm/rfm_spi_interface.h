/******************************************************************************
 * rfm_spi_interface.h
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * All the functions that are neccessary to control a SPI device from rfm module
 * These are the data transfer functions and two setter functions to assign the
 * (BB)SPI device and to set the data available interrupt in FSK mode and to 
 * Enable, Disable and Clear Interrupt flag of that interrupt.
 * 
 * Note: While RFM12 uses the MISO line also for signalling an DataAvailable
 *       Interrupt ( only when NESL is active), RFM69 requires an dedicated
 *       signal line for that. We use the SPI's busy line and interrupt for
 *       that. So keep in mind, that, in FSK mode,  RFM12 requires a 4-wire
 *       interface minimum, RFM69 requires at least a 5-wire interface.
 * 
 * The  data transfer functions differ between RFM12 and RMF69: While the
 * first one has no register clear addressing and all data transfer is done
 * thru 16bit read and writes, the RFM69 has a throughout register addressing
 * and access is done by specifying register address and an one to three byte
 * data value.
 *
 ******************************************************************************/
#ifndef __RFM_SPI_INTERFACE_H
#define __RFM_SPI_INTERFACE_H

#include "config/config.h"


#include "dev/spi.h"
#include "dev/devices.h"

void SetSPIDevice           ( const HW_DeviceType *dev);
void SetFskDataAvailableCB  (pFnIrqCB cb);

#if USE_RFM12 > 0
    uint16_t rfm_spi16_ret  (uint16_t outval);
    #define  rfm_spi16(a)   (void)(rfm_spi16_ret(a))

    #define RFM_INT_DIS()           MISO_IRQ_Disable(rfmSpi);
    #define RFM_INT_EN()            MISO_IRQ_Enable(rfmSpi)
    #define RFM_INT_CLR()           MISO_IRQ_Clear(rfmSpi)
#endif
#if USE_RFM69 > 0
    void     rfm_spi_write8     (uint8_t addr, uint8_t val8);
    void     rfm_spi_write16    (uint8_t addr, uint16_t val16);
    void     rfm_spi_write24    (uint8_t addr, uint32_t val32);
    void     rfm_spi_write_bulk (uint8_t addr, uint8_t *data, uint8_t size);
    uint8_t  rfm_spi_read8      (uint8_t addr);
    uint16_t rfm_spi_read16     (uint8_t addr);
    #define RFM_INT_DIS()           BUSY_IRQ_Disable(rfmSpi);
    #define RFM_INT_EN()            BUSY_IRQ_Enable(rfmSpi)
    #define RFM_INT_CLR()           BUSY_IRQ_Clear(rfmSpi)
#endif

#if USE_RFM_OOK > 0
    void SetOokDataAvailableCB (pFnIrqCB cb);
    #define OOK_DATA_IRQ_ENABLE()   INP_IRQ_Enable(rfmSpi)
    #define OOK_DATA_IRQ_DISABLE()  INP_IRQ_Disable(rfmSpi)
    #define OOK_DATA_IRQ_CLEAR()    INP_IRQ_Clear(rfmSpi)
    #define OOK_DATA_GET()          SpiInpGet(rfmSpi)
#endif

/* Controlling/Checking SPI-Interface ---------------------------------------*/
extern SpiHandleT *rfmSpi;
#define RFM_SPI_SELECT()        SpiNSelLow(rfmSpi)
#define RFM_SPI_DESELECT()      SpiNSelHigh(rfmSpi)
#define RFM_MISO_GET()          SpiMisoGet(rfmSpi)

#endif /* __RFM_SPI_INTERFACE_H */