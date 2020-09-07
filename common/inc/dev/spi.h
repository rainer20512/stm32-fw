/*
 *  SPI Functions 
 *  There are two implementations of SPI functionalities
 *     a) via hardware SPI
 *     b) via bit banging SPI
 *  Both must implement the same interface, that is defined here
 */

#pragma once // multi-iclude prevention. gcc knows this pragma

#define USE_BIT_BANDING

#include "config/config.h"
#include "dev/spi_dev.h"
#include "hw_device.h"

/******************************************************************************
 * Portion of SpiHandle, that is specific for Bitbang SPI
 *****************************************************************************/
typedef struct SpiDataTypeBB {
    __IO uint32_t * mosi_bsrr;
    __IO uint32_t *sck_bsrr;
    uint16_t mosi_bitpos;
    uint16_t  sck_bitpos;
} SpiDataBB;

/******************************************************************************
 * Portion of SpiHandle, that is specific for Hardware SPI
 *****************************************************************************/
typedef struct SpiDataTypeHW {
    SPI_HandleTypeDef myHalHandle;
    // SPI_TypeDef      *mySpi;
    uint32_t          myBaudrate;
    void    ( *OnTxComplete )   ( void );
    void    ( *OnTxRxComplete ) ( void );
    void    ( *OnError )        ( void );
    uint8_t           use_nss;
    uint8_t           use_hw_irq;
} SpiDataHW;

typedef struct SpiDataType {
    const HW_DeviceType *mySpiDev;
    __IO uint32_t *nsel_bsrr;
    __IO uint32_t *dnc_bsrr;
    __IO uint32_t *rst_bsrr;
    __IO uint32_t *busy_idr;
    __IO uint32_t *inp_idr;
    __IO uint32_t *miso_idr;
    uint16_t miso_bitpos;
    uint16_t busy_bitpos;
    uint16_t inp_bitpos;
    uint16_t nsel_bitpos;
    uint16_t  dnc_bitpos;
    uint16_t  rst_bitpos;
    uint8_t bInitialized;              /* flag for "device is Initialized/operable"     */
    uint8_t use_miso;                  /* Use the MISO line                             */
    uint8_t use_miso_irq;              /* Use the MISO line IRQ on rising edge          */
    uint8_t use_dnc;                   /* Use the DataNotCommand line (also called SPI4 */
    uint8_t use_rst;                   /* Use the reset line                            */
    uint8_t use_busy;                  /* Use the busy input line                       */
    uint8_t use_busy_irq;              /* Busy line IRQ on any edge                     */
    uint8_t use_inp;                   /* Use an additional input line                  */ 
    uint8_t use_inp_irq;               /* IRQ on Inp line level change                  */
    uint8_t bIsMaster;
    uint8_t datasize;
    uint8_t filler;
    union {
        SpiDataBB bb;
        SpiDataHW hw;
    };
} SpiDataT;


/******************************************************************************7
 * The following functions are hardware specific and must be linked via 
 * SPIfunctionType structure for every different implementation           
 *****************************************************************************/
typedef struct SpiFunctionType {
    uint16_t ( *Spi16TxRx          ) (SpiHandleT *, uint16_t );
    void     ( *Spi9TxByte         ) (SpiHandleT *, uint16_t );
    void     ( *Spi8TxByte         ) (SpiHandleT *, uint8_t  );
    uint8_t  ( *Spi8TxRxByte       ) (SpiHandleT *, uint8_t  );
    void     ( *Spi8TxRxVector     ) (SpiHandleT *self, uint8_t  *, uint8_t  *, uint16_t);
    void     ( *Spi8TxVector_IT    ) (SpiHandleT *self, uint8_t  *,             uint16_t);
    void     ( *Spi9TxVector       ) (SpiHandleT *self, uint16_t *,             uint16_t);
    void     ( *Spi9TxConstant     ) (SpiHandleT *self, uint16_t  ,             uint16_t);
    void     ( *Spi9TxVector_IT    ) (SpiHandleT *self, uint16_t *,             uint16_t);
    void     ( *Spi9TxVector_DMA   ) (SpiHandleT *self, uint16_t *,             uint16_t);
    void     ( *Spi9TxConstant_DMA ) (SpiHandleT *self, uint16_t  ,             uint16_t);
} SpiFunctionT;


/******************************************************************************7
 * The SPI Handle consist of neccessary variables and the block of
 * implementation specific functions        
 *****************************************************************************/
typedef struct SpiHandleType {
    SpiDataT *data;             /* my runtime data                           */
    const SpiFunctionT *fns;    /* my hardware specific implementation       */
} SpiHandleT;


/******************************************************************************7
 * The following functions are independet from any hardware specific implementation
 * and have to be implemented only once. Youe will find the implementation
 * in file "common_spi.c"
 *****************************************************************************/
SpiHandleT *SPI_GetHandleFromDev(const HW_DeviceType *self);

/*
 * The IRQ-Callback fns must have the following signature: 
 * uint16_t pin, uint16_t pinvalue, void *arg 
 * where pin is the pinnumber, that is associated with the interrupt
 *       pinvalue is the current pin input value ( 0 or 1 )
 *       arg is an optional  user defineable argument to the callback
 */
typedef void ( *pFnIrqCB )( uint16_t, uint16_t, void * );

bool SpiInit            (SpiHandleT *, const HW_DeviceType *);
void SpiDeInit          (SpiHandleT *, const HW_DeviceType *);
bool SpiClockInit       (const HW_DeviceType *self, bool bDoInit);
void HwSpiSetPrescaler  (SPI_TypeDef *hspi, uint32_t baudrate );

void SpiSetMisoCB       (SpiHandleT *, pFnIrqCB);
void SpiSetBusyCB       (SpiHandleT *, pFnIrqCB);
void SpiSetInpCB        (SpiHandleT *, pFnIrqCB);
void MISO_IRQ_Enable    (SpiHandleT *);
void MISO_IRQ_Clear     (SpiHandleT *);
void MISO_IRQ_Disable   (SpiHandleT *);
void BUSY_IRQ_Enable    (SpiHandleT *);
void BUSY_IRQ_Clear     (SpiHandleT *);
void BUSY_IRQ_Disable   (SpiHandleT *);
bool INP_IRQ_Enabled    (SpiHandleT *);
void INP_IRQ_Enable     (SpiHandleT *);
void INP_IRQ_Clear      (SpiHandleT *);
void INP_IRQ_Disable    (SpiHandleT *);
bool SpiIsBusy          (SpiHandleT *);
void SpiNSelLow         (SpiHandleT *);
void SpiNSelHigh        (SpiHandleT *);
void SpiDnCLow          (SpiHandleT *);
void SpiDnCHigh         (SpiHandleT *);
void SpiRstLow          (SpiHandleT *);
void SpiRstHigh         (SpiHandleT *);
uint32_t SpiMisoGet     (SpiHandleT *);
uint32_t SpiInpGet      (SpiHandleT *);

#define Spi16TxRx(hnd, ... )          hnd->fns->Spi16TxRx           (hnd, __VA_ARGS__ )
#define Spi9TxByte(hnd, ... )         hnd->fns->Spi9TxByte          (hnd, __VA_ARGS__ )
#define Spi8TxByte(hnd, ... )         hnd->fns->Spi8TxByte          (hnd, __VA_ARGS__ )  
#define Spi8TxRxByte(hnd, ... )       hnd->fns->Spi8TxRxByte        (hnd, __VA_ARGS__ )
#define Spi8TxRxVector(hnd, ... )     hnd->fns->Spi8TxRxVector      (hnd, __VA_ARGS__ )
#define Spi8TxVector_IT(hnd, ... )    hnd->fns->Spi8TxVector_IT     (hnd, __VA_ARGS__ )
#define Spi9TxVector(hnd, ... )       hnd->fns->Spi9TxVector        (hnd, __VA_ARGS__ )
#define Spi9TxConstant(hnd, ... )     hnd->fns->Spi9TxConstant      (hnd, __VA_ARGS__ )
#define Spi9TxVector_IT(hnd, ... )    hnd->fns->Spi9TxVector_IT     (hnd, __VA_ARGS__ )
#define Spi9TxVector_DMA(hnd, ... )   hnd->fns->Spi9TxVector_DMA    (hnd, __VA_ARGS__ )
#define Spi9TxConstant_DMA(hnd, ... ) hnd->fns->Spi9TxConstant_DMA  (hnd, __VA_ARGS__ )

