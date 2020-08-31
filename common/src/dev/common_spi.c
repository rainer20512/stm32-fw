/**
  ******************************************************************************
  * @file    common_spi.c
  * @author  Rainer
  *
  * @brief   That part of a SPI Impelemtation, that is common to harware and
  *          bit bang SPI
  *
  * @note:   ----------------------------------------------------
  *          to be performant, this file has to be compiled with 
  *          optimization on, even in DEBUG config
  *          ----------------------------------------------------
  *
  ******************************************************************************
  */

#include "config/devices_config.h"

#if defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_SPI4) || defined(USE_BBSPI1) || defined(USE_BBSPI2)

#include "dev/spi.h"
#include "system/exti_handler.h"


#define SPI_NSEL_LOW(a)      	(*(a->nsel_bsrr) = ((uint32_t)a->nsel_bitpos)<<16)
#define SPI_NSEL_HIGH(a)     	(*(a->nsel_bsrr) = a->nsel_bitpos)

#define SPI_DNC_LOW(a)      	(*(a->dnc_bsrr) = ((uint32_t)a->dnc_bitpos)<<16)
#define SPI_DNC_HIGH(a)     	(*(a->dnc_bsrr) = a->dnc_bitpos)

#define SPI_RST_LOW(a)      	(*(a->rst_bsrr) = ((uint32_t)a->rst_bitpos)<<16)
#define SPI_RST_HIGH(a)     	(*(a->rst_bsrr) = a->rst_bitpos)

#define SPI_MISO_GET(a)       (*(a->miso_idr) & (a->miso_bitpos))
#define SPI_INP_GET(a)        (*(a->inp_idr)  & (a->inp_bitpos) )
#define SPI_IS_BUSY(a)        (*(a->busy_idr) & (a->busy_bitpos))

/* Private or driver functions ------------------------------------------------------*/
static void SpiResetMyData ( SpiDataT *data ) 
{
    memset(data, 0, sizeof(SpiDataT) );
}

/* Public functions -----------------------------------------------------------------*/
void MISO_IRQ_Clear( SpiHandleT *self )
{
    EXTI_CLEAR_PR1(self->data->miso_bitpos); 
}

void MISO_IRQ_Enable( SpiHandleT *self )
{
    EXTI_ENABLE_IRQ(self->data->miso_bitpos); 

}

void MISO_IRQ_Disable( SpiHandleT *self  )
{
   EXTI_DISABLE_IRQ(self->data->miso_bitpos );
}

void BUSY_IRQ_Clear( SpiHandleT *self )
{
    EXTI_CLEAR_PR1(self->data->busy_bitpos); 
}

void BUSY_IRQ_Enable( SpiHandleT *self )
{
    EXTI_ENABLE_IRQ(self->data->busy_bitpos); 

}

void BUSY_IRQ_Disable( SpiHandleT *self  )
{
   EXTI_DISABLE_IRQ( self->data->busy_bitpos );
}

void INP_IRQ_Clear( SpiHandleT *self )
{
    EXTI_CLEAR_PR1(self->data->inp_bitpos); 
}

bool INP_IRQ_Enabled(SpiHandleT *self )
{
   return EXTI_IRQ_ENABLED( self->data->inp_bitpos ) != 0;
}


void INP_IRQ_Enable( SpiHandleT *self )
{
    EXTI_ENABLE_IRQ(self->data->inp_bitpos); 

}

void INP_IRQ_Disable( SpiHandleT *self  )
{
   EXTI_DISABLE_IRQ( self->data->inp_bitpos );
}


void SpiNSelLow(SpiHandleT *self)
{
    SPI_NSEL_LOW(self->data);
}

void SpiNSelHigh(SpiHandleT *self)
{
    SPI_NSEL_HIGH(self->data);
}

void SpiDnCLow(SpiHandleT *self)
{
    SPI_DNC_LOW(self->data);
}

void SpiDnCHigh(SpiHandleT *self)
{
    SPI_DNC_HIGH(self->data);
}

void SpiRstLow(SpiHandleT *self)
{
    SPI_RST_LOW(self->data);
}

void SpiRstHigh(SpiHandleT *self)
{
    SPI_RST_HIGH(self->data);
}

bool SpiIsBusy(SpiHandleT *self)
{
    return SPI_IS_BUSY(self->data);
}

bool SpiMisoGet(SpiHandleT *self)
{
    return SPI_MISO_GET(self->data);
}

bool SpiInpGet(SpiHandleT *self)
{
    return SPI_INP_GET(self->data);
}

/* Will be overwritten in hw_spi.c, unchanged for bitbang_spi.c */
__weak bool HwSpi_DefaultInit(SpiDataT *data, const HW_DeviceType *SpiDev)
{
    UNUSED(data); UNUSED(SpiDev);
    return true;
}

/******************************************************************************
 * Performs a Standard initialization of SPI Handle and link to SPI device
 * Standard means, that no DMA is configured and the SPI mode is
 *****************************************************************************/
bool SpiInit (SpiHandleT *hnd, const HW_DeviceType *SpiDev)
{
    /* Initialize the associated Handle */
    SpiResetMyData(hnd->data);
    SpiHandleInit(hnd, SpiDev);
    if ( SpiDev->devType == HW_DEVICE_HWSPI ) {
        /* Initialize hardware, if hadrware SPI is used */
        hnd->data->bInitialized = HwSpi_DefaultInit(hnd->data, SpiDev);
    } else {
        hnd->data->bInitialized = true;
    }
    return hnd->data->bInitialized;
}
/******************************************************************************
 * Deinitialization of handle: Reset to zero
 *****************************************************************************/
void SpiDeInit (SpiHandleT *hnd, const HW_DeviceType *self)
{
    UNUSED(self);
    /* NB: ResetHandle will also set hnd->bInitialized to false */
    SpiResetMyData(hnd->data);
}

/*
 * Register Interrupt Callback for MISO rising edge. 
 * The interrupt is not enabled automatically
 * This has to be done by MISO_IRQ_Enable
 */
void SpiSetMisoCB(SpiHandleT *self, pFnIrqCB cb)
{
    if ( cb )
        Exti_Register_Callback     ( self->data->miso_bitpos, self->data->miso_idr, cb, self);
    else
        Exti_UnRegister_Callback   ( self->data->miso_bitpos);
}

/*
 * Register Interrupt Callback for BUSY falling edge. 
 * The interrupt is not enabled automatically
 * This has to be done by BUSY_IRQ_Enable
 */
void SpiSetBusyCB(SpiHandleT *self, pFnIrqCB cb)
{
    if ( cb )
        Exti_Register_Callback     ( self->data->busy_bitpos, self->data->busy_idr, cb, self );
    else
        Exti_UnRegister_Callback   ( self->data->busy_bitpos);
}

/*
 * Register Interrupt Callback for BUSY falling edge. 
 * The interrupt is not enabled automatically
 * This has to be done by BUSY_IRQ_Enable
 */
void SpiSetInpCB(SpiHandleT *self, pFnIrqCB cb)
{
    if ( cb )
        Exti_Register_Callback     ( self->data->inp_bitpos, self->data->inp_idr, cb, self );
    else
        Exti_UnRegister_Callback   ( self->data->inp_bitpos);
}

#endif /* #if defined(USE_SPI1) || defined(USE_SPI2) || defined(USE_SPI3) || defined(USE_BBSPI1) || defined(USE_BBSPI2) */
