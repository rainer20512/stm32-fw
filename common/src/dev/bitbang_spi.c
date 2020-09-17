/**
  ******************************************************************************
  * @file    bitbang_spi.c
  * @author  Rainer
  *
  * @brief   Implements a bitbang spi on the bitbang spi device
  *          This implementation can be configured to implement/cut off
  *          the following functionalities
  *          - Uni- or bidirectional SPI, ie with or w/o MISO line
  *          - Generate an interrupt when MISO goes high
  *          - implement a D/C line ( used with displays 
  *          - implement a reset line
  *
  * @note:   ----------------------------------------------------
  *          to be performant, this file has to be compiled with 
  *          optimization on, even in DEBUG config
  *          ----------------------------------------------------
  *
  ******************************************************************************
  */

#include "config/devices_config.h"

#if defined(USE_BBSPI1) || defined(USE_BBSPI2) || defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4)

#include "dev/spi_dev.h"
#include "dev/spi.h"
#include "debug_helper.h"
#include "system/exti_handler.h"

#if defined(USE_BBSPI1) || defined(USE_BBSPI2)


#ifdef USE_BIT_BANDING
    #include "system/hw_util.h"
#endif


#define BBSPI_MOSI_LOW(a)      	(*(a->bb.mosi_bsrr) = ((uint32_t)a->bb.mosi_bitpos)<<16)
#define BBSPI_MOSI_HIGH(a)     	(*(a->bb.mosi_bsrr) = a->bb.mosi_bitpos)

#define BBSPI_SCK_LOW(a)      	(*(a->bb.sck_bsrr)  = ((uint32_t)a->bb.sck_bitpos)<<16)
#define BBSPI_SCK_HIGH(a)     	(*(a->bb.sck_bsrr)  = a->bb.sck_bitpos)

#define BBSPI_MISO_GET(a)       (*(a->miso_idr) & (a->miso_bitpos))


/*******************************************************************************
 *  Bitbanged 16-bit SPI 
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 ******************************************************************************/
uint16_t Spi16TxRx_bb(SpiHandleT *self, uint16_t outval)
{
  uint32_t i;

  // DEBUG_PRINTF("Spi16: %04x\n", outval);
  
  for (i=16;i!=0;i--) {
    if (0x8000 & outval) {
      BBSPI_MOSI_HIGH(self->data);
    } else {
      BBSPI_MOSI_LOW(self->data);
    }
	
	outval <<= 1;

    BBSPI_SCK_HIGH(self->data);

    if (self->data->use_miso)
        if (BBSPI_MISO_GET(self->data)) outval |= 1;
    

    BBSPI_SCK_LOW(self->data);
//    asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop");
  } // for loop
  
  return outval;
}

/*******************************************************************************
 *  Bitbanged 8-bit SPI 
 *  Unidirectional: Transmit only
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 *
 *  Timing considerations:
 *  pulse and clock cycle width vary widelly with the clock frequency
 *  These are typical values with optimized ( level 3 ) code:
 *
 *  SYSCLK   Hi Pulse Width   Lo Pulse Width   Clk Cycle Width
 *  [MHz]        [ns]               [ns]            [ns]
 *   80           90                190             280
 *   32           160               310             470
 *   16           310               630             940
 *    8           630              1250            1880
 *
 *  Pls consult your SPI devices manual, whether the minimal clock high pulse
 *  width and clock cycle width are met.
 *  
 ******************************************************************************/
void Spi8TxByte_bb(SpiHandleT *self, uint8_t outval)
{
  uint32_t i;

  BBSPI_SCK_LOW(self->data);

  for (i=8;i!=0;i--) {
    if (0x80 & outval) {
      BBSPI_MOSI_HIGH(self->data);
	  // uart_putc('1');
    } else {
      BBSPI_MOSI_LOW(self->data);
	  // uart_putc('0');
    }
	
    BBSPI_SCK_HIGH(self->data);
	outval <<= 1;

    BBSPI_SCK_LOW(self->data);
  } // for loop
}

/*******************************************************************************
 *  Bitbanged 8-bit SPI 
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 ******************************************************************************/
uint8_t Spi8TxRxByte_bb(SpiHandleT *self, uint8_t outval)
{
  uint32_t i;


  // print_s_p(PSTR("Spi8:")); print_hexXX(outval);CRLF();

  BBSPI_SCK_LOW(self->data);

  for (i=8;i!=0;i--) {
    if (0x80 & outval) {
      BBSPI_MOSI_HIGH(self->data);
	  // uart_putc('1');
    } else {
      BBSPI_MOSI_LOW(self->data);
	  // uart_putc('0');
    }
	
    BBSPI_SCK_HIGH(self->data);
	outval <<= 1;

	if (self->data->use_miso)
            if (BBSPI_MISO_GET(self->data)) outval |= 1;
	
    BBSPI_SCK_LOW(self->data);
  } // for loop
  
  return outval;
}

/*******************************************************************************
 *  Bitbanged 8-bit SPI Transmit of a vector of bytes
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 *  Either vectorIn or vectorOut may be NULL; in wich case data is only shifted 
 *  out or shifted in respectively. 
 ******************************************************************************/
bool Spi8TxRxVector_bb(SpiHandleT *self, uint8_t *vectorOut, uint8_t *vectorIn, uint16_t size)
{
    uint32_t i;
    if ( !vectorIn && !vectorOut ) return;

    if        ( !vectorIn ) {
        for ( i = 0; i < size; i++ ) Spi8TxByte_bb(self, vectorOut[i]);
    } else if ( !vectorOut ) {
        for ( i = 0; i < size; i++ ) vectorIn[i] = Spi8TxRxByte_bb(self, 0);
    } else {
        for ( i = 0; i < size; i++ ) vectorIn[i] = Spi8TxRxByte_bb(self, vectorOut[i]);
    }
    return true;
}

/*******************************************************************************
 *  Bitbanged 9 bit SPI 
 *  Transfer may be uni- or bidirectional, this can be configured in bb_spi_config.h
 *  Setting of ChipSelect and so on has to be done outside of this routine
 *  Shift Direction is from MSB to LSB, Signal not inverted
 *  Can be used for LCD-Controllers, where the nine'th (highest) bit will
 *  be interpreted as Data/notCommand
 ******************************************************************************/
void Spi9TxByte_bb(SpiHandleT *self, uint16_t outval)
{
  uint32_t i;


  // print_s_p(PSTR("Spi8:")); print_hexXX(outval);CRLF();

  BBSPI_SCK_LOW(self->data);

  for (i=9;i!=0;i--) {
    if (0x0100 & outval) {
      BBSPI_MOSI_HIGH(self->data);
    } else {
      BBSPI_MOSI_LOW(self->data);
    }
	
    BBSPI_SCK_HIGH(self->data);
	outval <<= 1;

	if (self->data->use_miso)
            if (BBSPI_MISO_GET(self->data)) outval |= 1;
	
    BBSPI_SCK_LOW(self->data);
  } // for loop
  
//   return outval;
}

const SpiFunctionT SpiFns_bb = {
   .Spi16TxRx          = Spi16TxRx_bb,
   .Spi9TxByte         = Spi9TxByte_bb,
   .Spi8TxByte         = Spi8TxByte_bb,
   .Spi8TxRxByte       = Spi8TxRxByte_bb,
   .Spi8TxRxVector     = Spi8TxRxVector_bb,
   .Spi8TxVector_IT    = NULL,                  /* all these are not implemented as BitBanging functions ! */
   .Spi9TxVector       = NULL,
   .Spi9TxConstant     = NULL,
   .Spi9TxVector_IT    = NULL,
   .Spi9TxVector_DMA   = NULL,
   .Spi9TxConstant_DMA = NULL,
};

#else
    const SpiFunctionT SpiFns_bb = { 0 };
#endif /* #if defined(USE_BBSPI1) || defined(USE_BBSPI2) */

#endif /* #if defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4) */