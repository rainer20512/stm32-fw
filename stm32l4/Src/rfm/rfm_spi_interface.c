/******************************************************************************
 * rfm_spi_interface.h
 *
 * Created: 11.12.2019 
 *  Author: rainer
 *
 * All the functions that are neccessary to control a SPI device from rfm module
 * These are the data transfer functions and two setter functions to assign the
 * (BB)SPI device and to set the data available interrupt in FSK mode.
 * 
 * The  data transfer functions differ between RFM12 and RMF69: While the
 * first one has no register clear addressing and all data transfer is done
 * thru 16bit read and writes, the RFM69 has a throughout register addressing
 * and access is done by specifying register address and an one to three byte
 * data value.
 *
 ******************************************************************************/
#include "config/config.h"
#include "debug.h"

#if USE_RFM12 > 0 || USE_RFM69 > 0

#include "rfm/rfm_spi_interface.h"

#if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
	#include "rfm/rfm_ringbuffer.h"
#endif

SpiHandleT *rfmSpi=NULL;

/*!
 *******************************************************************************
 *  Assign a SPI device to the RFM module. 
 *  \note this my be an BitBang or Hardware SPI device
 ******************************************************************************/
void SetSPIDevice( const HW_DeviceType *dev) {
    rfmSpi = SPI_GetHandleFromDev(dev);
}

/*!
 *******************************************************************************
 *  Assign a callback function when the MISO line changes its level
 *  \note this indicates "data available" in FSK mode
 ******************************************************************************/
void SetFskDataAvailableCB(pFnIrqCB cb) {
    #if USE_RFM12 > 0
        SpiSetMisoCB(rfmSpi, cb );
    #elif USE_RFM69 > 0 
        SpiSetBusyCB(rfmSpi, cb );
    #else
        #error "No DataAvail-Interrupt configuration!"
    #endif
}
#if USE_RFM_OOK
    /*!
     *******************************************************************************
     *  Assign a callback function when the OOK data line changes its level
     ******************************************************************************/
    void SetOokDataAvailableCB(pFnIrqCB cb) {
        SpiSetInpCB(rfmSpi, cb );
    }
#endif

#if USE_RFM12 > 0
/*!
 *******************************************************************************
 *  RFM SPI access
 *  \note 16bit SPI TX and RX transfer
 *  \param outval the value that shall be clocked out to the RFM
 *  \returns the value that is clocked in from the RFM
 *   
 ******************************************************************************/
uint16_t rfm_spi16_ret(uint16_t outval)
{
  // uint8_t i;
  uint16_t ret; // =0; <- not needeed will be shifted out

  #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
    rfm_xbuf_put(xbuf_uint16_out, outval);
  #endif
  
  RFM_SPI_SELECT();
 
  ret = Spi16TxRx(rfmSpi, outval );

  RFM_SPI_DESELECT();
  RFM_SPI_SELECT();		// nSEL back to Low to enable RFM12 to signal pending data on MISO-Line ( SDO on RFM12 )

  #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
    rfm_xbuf_put(xbuf_uint16_in, ret);
  #endif

  return ret;

}
#endif

#if USE_RFM69 > 0

/*
 * used to serialize the different types and length of register/data access for RFM69
 */
static uint8_t vector[4];

/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Write an 8-bit value via SPI
 *  \param addr register address
 *  \param val8 the 8bit value to be written to register
 *   
 ******************************************************************************/
void rfm_spi_write8(uint8_t addr, uint8_t val8)
{

    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put(xbuf_uint88_out,((uint16_t)addr) << 8 | val8);
    #endif
	
    // MSB to 1 for write access
    vector[0] = addr | 0x80 ;
    vector[1] = val8;

    RFM_SPI_SELECT();
    Spi8TxVector(rfmSpi, vector, 2);			
    RFM_SPI_DESELECT();
}

/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Write an 16-bit value via SPI
 *  \param addr register address
 *  \param val16 the 16bit value to be written to register
 *   
 ******************************************************************************/
void rfm_spi_write16(uint8_t addr, uint16_t val16)
{
    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put( xbuf_uint816_out, ((uint16_t)addr)  );
        rfm_xbuf_put( xbuf_uint816_out, val16 );
    #endif
  
		
    // MSB to 1 for write access
    vector[0] = addr | 0x80 ;
    vector[1] = val16  >> 8;
    vector[2] = val16 & 0xff;

    RFM_SPI_SELECT();
    Spi8TxVector(rfmSpi, vector, 3);			
    RFM_SPI_DESELECT();
}

/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Write an 24-bit value via SPI
 *  \param addr register address
 *  \param val24 the 24bit value to be written to register
 *   
 ******************************************************************************/
void rfm_spi_write24(uint8_t addr, uint32_t val24)
{

    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put( xbuf_uint824_out, ((uint16_t)addr)<<8 | ((uint8_t)((val24 >> 16) & 0xff )));
        rfm_xbuf_put( xbuf_uint824_out, (uint16_t)(val24 &0xffff) );
    #endif
  	
    // MSB to 1 for write access
    vector[0] = addr | 0x80 ;
    vector[1] = val24 >> 16;
    vector[2] = val24 >> 8;
    vector[3] = val24 & 0xff;

    RFM_SPI_SELECT();
    Spi8TxVector(rfmSpi, vector, 4);			
    RFM_SPI_DESELECT();
}

/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Write an byte vector via SPI
 *  \param addr rfm69 register address
 *  \param data ptr to the byte vector that shall be clocked out
 *  \param size number of bytes to be written
 *
 ******************************************************************************/
void rfm_spi_write_bulk(uint8_t addr, uint8_t *data, uint8_t size)
{

    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put( xbuf_uint8_out, ((uint16_t)addr) << 8 );
    #endif
  
    RFM_SPI_SELECT();
    // MSB to 1 for write access
    Spi8TxByte(rfmSpi, addr | 0x80);
    Spi8TxVector(rfmSpi, data, size);
    RFM_SPI_DESELECT();
}


/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Read an 8 bit value 
 *  \param addr rfm69 register address to read
 *  \returns content of register
 *
 ******************************************************************************/
uint8_t rfm_spi_read8(uint8_t addr)
{
    uint8_t ret;
    RFM_SPI_SELECT();
    // MSB to 0 for read access
    Spi8TxByte(rfmSpi, addr & 0x7f);
    ret = Spi8TxRxByte(rfmSpi, 0) ;
    RFM_SPI_DESELECT();

    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put( xbuf_uint88_in, ((uint16_t)addr)<<8 | ret  );
    #endif
    return ret ;
}

/*!
 *******************************************************************************
 *  RFM69 SPI access
 *  \note Read an 16 bit value 
 *  \param addr rfm69 register address to read
 *  \returns content of register
 *
 ******************************************************************************/
uint16_t rfm_spi_read16(uint8_t addr)
{
    uint16_t ret;

    RFM_SPI_SELECT();
    // MSB to 0 for read access
    Spi8TxByte(rfmSpi, addr & 0x7f);
    ret = Spi16TxRx(rfmSpi, 0) ;
    RFM_SPI_DESELECT();
            

    #if ( DEBUG_DUMP_RFM > 0 && DEBUG_RFM_HARDCORE > 0 )
        rfm_xbuf_put( xbuf_uint816_in, ((uint16_t)addr)  );
        rfm_xbuf_put( xbuf_uint816_in, ret );
    #endif

  return ret;
}
#endif /* if USE_RFM69 > 0 */

#endif /* if USE_RFM12 > 0 || USE_RFM69 > 0 */
