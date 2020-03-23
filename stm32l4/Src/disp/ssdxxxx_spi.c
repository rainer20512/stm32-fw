/**
  ******************************************************************************
  * @file    ssdxxxx_spi.c
  * @author  Rainer
  *
  * @brief   Implements a very simple interface for epaper/LCD spi-controllers
  *          of type SSDxxxx with optional MISO line, with D/C and nRst line in      
  *          addition to the normal MOSI, SCK and nSEL lines
  *          
  *          In general, a BitBang SPI is faster as the Hardware SPI due
  *          to the overhead of HAL.
  * 
  *          With SPI4 (ie 8 Bit + D/C line ) and D/C line it is difficult to
  *          implement a mechanism, that makes use of DMA-Transfer, because the
  *          D/C line has to be switched by Software. 
  *
  *          So, when using SPI4, the best way to to this is using BitBang SPI
  *          when using HW SPI, the best way is to use polling mode. The 8 bit
  *          implementation for two separate tasks, one for filling the output 
  *          queue, one for sending the output via SPI to the ePaper, was 
  *          removed due to heavy overhead compared with the direct polling
  *          method.
  * 
  *          For SPI3 (i.e.9 Bit, where the MSB is the D/C selector ) we implement
  *          a polling method with active wait and an asynchronous method, which
  *          uses a separate output task. The data exchange is done via a circular
  *          buffer and done by DMA. 
  * 
  *          There is another implementation method, which implements the whole
  *          frame buffer as shadow frame buffer in system RAM. The advantage of
  *          this method is, that we get rid of wiping the whole display ram before
  *          filling in the useful data. This method is not yet implemented.
  *          
  *
  * @note:   The SPI-Device to use is assigned in code in "SSD_DevInit"
  *
  * @note:   ----------------------------------------------------
  *          to be performant, this file has to be compiled with 
  *          optimization on, even in DEBUG config
  *          ----------------------------------------------------
  *
  ******************************************************************************
  */
#include "config/config.h"

#if USE_EPAPER > 0 || USE_DOGM132 > 0

#include "dev/spi.h"
#include "error.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

#define USE_9BIT_SPI     0
#define DO_ASYNC_WRITE   0  

/* Forward declarations --------------------------------------------------------*/
#if DO_ASYNC_WRITE > 0 
    static void OnTxComplete (void );
#endif

static SpiHandleT *mySpi;
static SpiDataT   *mySpiData;

#if DO_ASYNC_WRITE > 0
    /* In asynchronous mode, we need no direct control on NSEL line */
    #define NSEL_LOW() 
    #define NSEL_HIGH()
#else
    #define NSEL_LOW()      SpiNSelLow (mySpi)
    #define NSEL_HIGH()     SpiNSelHigh (mySpi)
#endif

#if DO_ASYNC_WRITE > 0
    /* What we need for anychronous operation */
    #include "task/minitask.h"
    #include "circbuf.h"

    /* 
     * The circular buffer should be large enough to keep all the operations until
     * next update. Uniform data byte writes should be done via "SSD_WriteCnConstant"
     * This will be coded specially in circular buffer and save lots of space
     */

    #define EPD_BUFSIZE     2048
    
    /* 
     * Coding for 9-bit SPI Command and data bytes 
     */
    #define SPI9_DATABIT    0x100
    #define MKDATA(d)       (((uint16_t)d) | SPI9_DATABIT )
    #define MKCMD(c)        ((uint16_t)c) 

    /* "normal" values in the circular buffer are in the range 0x0000 .. 0x0100 */
    /* we use  higher values to mark special operation fields                   */
    #define EPD_SPECIAL_MASK        0x8000U
    #define EPD_OPERATION_MASK      0x7FFFU
    #define EPD_OPERATION_SHIFT     0
    /* All special operation identifier                                         */
    #define EPD_OP_FILL_CONSTANT    0x0001U

    /* Circular buffer structure and the circular buffer itself */
    static uint8_t buf[EPD_BUFSIZE];
    static CircBuffT  epdCbuf;  


    /* Special Operation "Write Uniform byte vector" */
    static void Encode_WriteCnConstant ( uint8_t cmd, const uint8_t value, uint16_t length )
    {
        
        CircBuff_Put2(&epdCbuf, EPD_SPECIAL_MASK | EPD_OP_FILL_CONSTANT );
        CircBuff_Put2(&epdCbuf, (uint16_t)cmd << 8 | value );
        CircBuff_Put2(&epdCbuf, length );
    }

    static void Decode_WriteCnConstant ( void ) 
    {
        uint8_t cmd, value;
        uint16_t length;

        CircBuff_Get2(&epdCbuf, &length);
        cmd = length >> 8;
        value = (uint8_t)length;
        CircBuff_Get2(&epdCbuf, &length);

        Spi9TxByte(mySpi, MKCMD(cmd));
        Spi9TxConstant_DMA(mySpi, MKDATA(value), length);

        /* Must call OnTxComplete, if not using IT or DMA functions! 
        OnTxComplete();
        */
    }

    /* The central start point to decode special commands in circbuf: */
    /* read the special word and find out the operation ID            */
    /* The control of the NSEL line is done by the caller, be sure    */
    /* to call "OnTxComplete" after execution, either as callback on  */
    /* spi interrupt functions ( that is preset ) or do it manually   */
    /* when using spi busy wait functions                             */
    static void Decode_Special ( void )
    {
        uint16_t spec;
        CircBuff_Get2(&epdCbuf, &spec);
        spec = spec & EPD_OPERATION_MASK;
        switch ( spec ) 
        {
            case EPD_OP_FILL_CONSTANT:
                Decode_WriteCnConstant();
                break;
            default:
                #if DEBUG_MODE > 0 
                    DEBUG_PRINTF("Unknown special code %02d", spec);
                #endif
                OnTxComplete();
        }
    }
#endif



#if USE_9BIT_SPI == 0
    /**************************************************************************************
     * @brief Output a byte via associated SPI. The NSEL-line setting is done manually
     *        per byte
     * @param data    - byte to send
     * @note  SPI is accessed via polling, i.e. wait for end of previous transmission 
     *        could occur
     *************************************************************************************/
    static void SSD_SPI_out(uint8_t data)
    {
      NSEL_LOW();
      Spi8TxByte     (mySpi, data);
      NSEL_HIGH();
    }
#else

    static void Spi9_out(uint16_t c_or_d )
    {
    #if DO_ASYNC_WRITE > 0
        #if DEBUG_MODE > 0
            bool bRet = 
        #endif
        CircBuff_Put2(&epdCbuf, c_or_d );
        #if DEBUG_MODE > 0
            if ( !bRet ) DEBUG_PUTS("Spi9_out: Buffer overflow!");
        #endif
    #else
        Spi9TxByte  (mySpi, out);
    #endif
    }
    /**************************************************************************************
     * SSD1606 supports alternatively SPI9, where MSBit is D/C bit
     * this allows to omit the D/C line
     * Use teh following code for that
     *************************************************************************************/
    static void EPD_SPI9_out(uint16_t c_or_d )
    {
      NSEL_LOW();
      Spi9_out    (c_or_d);
      NSEL_HIGH();
    }
#endif

/******************************************************************************
 * Write one command byte                                                     *
 * Write directly to SPI device with polling mode, i.e. wait could occur      *
 *****************************************************************************/
void SSD_WriteCmd(uint8_t cmd)
{
#if USE_9BIT_SPI > 0
    EPD_SPI9_out(MKCMD(cmd));
#else
    SpiDnCLow(mySpi);
    SSD_SPI_out(cmd);
#endif
}

/******************************************************************************
 * Write one data byte                                                        *
 * Write directly to SPI device with polling mode, i.e. wait could occur      *                                               *
 *****************************************************************************/
void SSD_WriteData(uint8_t data)
{
#if USE_9BIT_SPI > 0
    EPD_SPI9_out(MKDATA(data));
#else
    SpiDnCHigh(mySpi);
    SSD_SPI_out(data);
#endif
}

/******************************************************************************
 * Write one command byte                                                     *
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void SSD_WriteC( uint8_t cmd)
{
#if USE_9BIT_SPI > 0
    EPD_SPI9_out(MKCMD(cmd));
#else
    SpiDnCLow(mySpi);
    NSEL_LOW();
    Spi8TxByte     (mySpi, cmd);
    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one data byte                                                        *
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void SSD_WriteD( uint8_t data )
{
#if USE_9BIT_SPI > 0
    NSEL_LOW();
    Spi9_out(MKDATA(data));
    NSEL_HIGH();
#else
    SpiDnCHigh(mySpi);
    NSEL_LOW();
    Spi8TxByte(mySpi, data);
    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one command and one data byte                                        *
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void SSD_WriteCnD( uint8_t cmd, uint8_t data )
{
#if USE_9BIT_SPI > 0
    NSEL_LOW();
    Spi9_out(MKCMD(cmd));
    Spi9_out(MKDATA(data));
    NSEL_HIGH();
#else
    SpiDnCLow(mySpi);
    NSEL_LOW();
    Spi8TxByte(mySpi, cmd);

    SpiDnCHigh(mySpi);
    Spi8TxByte(mySpi, data);
    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one command and two data bytes                                       *
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void SSD_WriteCnDD( uint8_t cmd, uint8_t d1, uint8_t d2 )
{
#if USE_9BIT_SPI > 0
    NSEL_LOW();
    Spi9_out(MKCMD(cmd));
    Spi9_out(MKDATA(d1));
    Spi9_out(MKDATA(d2));
    NSEL_HIGH();
#else
    SpiDnCLow(mySpi);
    NSEL_LOW();
    Spi8TxByte     (mySpi, cmd);

    SpiDnCHigh(mySpi);
    Spi8TxByte     (mySpi, d1);
    Spi8TxByte     (mySpi, d2);
    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one command and a vector of data bytes                               *
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void SSD_WriteCnDataVector(uint8_t cmd, uint8_t *vector, uint16_t length )
{
#if USE_9BIT_SPI > 0
    NSEL_LOW();
    Spi9_out(MKCMD(cmd));

    for ( uint16_t i = 0; i < length; i++)
        Spi9_out(MKDATA(*(vector++)));

    NSEL_HIGH();
#else
    SpiDnCLow(mySpi);
    NSEL_LOW();
    Spi8TxByte     (mySpi, cmd);

    SpiDnCHigh(mySpi);
    for ( uint16_t i = 0; i < length; i++)
        Spi8TxByte (mySpi, *(vector++));

    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one command and a vector of unique data bytes
 * Write indirectly via FIFO-Buffer                                           *
 *****************************************************************************/
void  SSD_WriteCnConstant ( uint8_t cmd, const uint8_t value, uint16_t length )
{
#if USE_9BIT_SPI > 0
    #if DO_ASYNC_WRITE > 0
        Encode_WriteCnConstant ( cmd, value, length );
    #else
        NSEL_LOW();
        Spi9_out(MKCMD(cmd));
        Spi9TxConstant(mySpi, MKDATA(value), length);
        NSEL_HIGH();
    #endif
#else
    SpiDnCLow(mySpi);
    NSEL_LOW();
    Spi8TxByte     (mySpi, cmd);

    SpiDnCHigh(mySpi);
    for ( uint16_t i = 0; i < length; i++)
        Spi8TxByte (mySpi, value);

    NSEL_HIGH();
#endif
}

/******************************************************************************
 * Write one command and a variable number of following databytes
 *****************************************************************************/
void SSD_StartConsecWrite ( uint8_t cmd )
{
    SSD_WriteCmd(cmd);
}
/******************************************************************************
 * Write one data byte in the sequence of data bytes 
 *****************************************************************************/
void SSD_ConsecWrite ( uint8_t data)
{
    SSD_WriteData(data);
}

/******************************************************************************
 * Terminate the writing of consecutive data bytes
 *****************************************************************************/
void SSD_StopConsecWrite ( void )
{
    /* Nothing to do here */
}


void SSD_DevReset(void)
{
    SpiNSelHigh(mySpi);
    HAL_Delay(1);
    SpiRstLow(mySpi);
    HAL_Delay(2);
    SpiRstHigh(mySpi);
    HAL_Delay(3);
}

bool SSD_CanRead(void)
{
   return mySpiData->use_miso != 0;
}

bool SSD_IsBusy(void)
{
    return mySpiData->use_busy && SpiIsBusy(mySpi);
}
bool SSD_IsInitialized (void)
{
    return mySpiData->bInitialized;
}

void SSD_SetBusyIrqCB(pFnIrqCB cb )
{

    if ( mySpiData->use_busy ) {
        SpiSetBusyCB(mySpi, cb);
        if ( cb ) {
            BUSY_IRQ_Clear(mySpi);
            BUSY_IRQ_Enable(mySpi);
        } else {
            BUSY_IRQ_Disable(mySpi);
        }
    } else {
    #if DEBUG_MODE > 0
        DEBUG_PUTS("Error: Cannot assign Busy-Change Interrupt, no busy pin defined");
    #endif
    }
        
}

void SSD_DevInit(const HW_DeviceType *dev)
{
    if ( dev->devType != HW_DEVICE_BBSPI  && dev->devType != HW_DEVICE_HWSPI ) {
        #if DEBUG_MODE > 0 && ( DEBUG_EPAPER > 0 || DEBUG_LCD > 0 )
            DEBUG_PUTS("LCD device not of type SPI, init failed"); 
        #endif
        mySpi     = NULL;
        mySpiData = NULL;
        return;
    }

    mySpi     = SPI_GetHandleFromDev(dev);
    mySpiData = mySpi->data;

    /* check for neccessary pins assigned */
#if USE_9BIT_SPI < 1
    #if DEBUG_MODE > 0 && ( DEBUG_EPAPER > 0 || DEBUG_LCD > 0 )
        if (!mySpiData->use_dnc) {
            DEBUG_PUTS("8bit SPI without D/C line - program stopped");
        }
    #endif
    assert(mySpi->data->use_dnc);
#endif    
    /* 
     * If there is no dedicated Reset-Pin, the Reset of the ePaper
     * has to be tied to Vdd                 
     */
    #if DEBUG_MODE > 0 && ( DEBUG_EPAPER > 0 || DEBUG_LCD > 0 )
        if (!mySpiData->use_rst) {
            DEBUG_PUTS("Warning: LCD has no nRST connection, tie to Vdd\n");
        }
    #endif
#if DO_ASYNC_WRITE > 0
    bool bRet = CircBuff_Init(&epdCbuf, EPD_BUFSIZE, buf);
    #if DEBUG_MODE > 0
        if ( !bRet ) DEBUG_PUTS("SSD_DevInit: CircBuff_Init failed!");
    #endif
    mySpiData->hw.OnTxComplete = OnTxComplete;
#endif
    /* Will be initialized with NSEL=1, NRST=1, SCK=0, DnC Low */
}


/* --------------------------------------------------------------------------*/
/* Functions for asynchronous mode of epaper spi interface                   */
/* --------------------------------------------------------------------------*/
#if DO_ASYNC_WRITE > 0 
static uint32_t readsize; 

/* Start asynchronous write */
void SSD_Flush(void)
{
    #if DEBUG_MODE > 0 && DEBUG_EPAPER > 0
        DEBUG_PRINTF("Circular buffer usage before flush %d/%d\n", CBUF_GET_USED(epdCbuf), EPD_BUFSIZE);
    #endif

    TaskNotify(TASK_EPD);
}

/******************************************************************************
 * Callback for asynchronous data transfer
 *****************************************************************************/
void SSD_AsynchronousWrite(void)
{
    /* Do nothing, if circular buffer is empty */
    if ( CBUF_EMPTY(epdCbuf) ) return;

    /* First check for special command */
    uint16_t firstword;
    CircBuff_Peek2(&epdCbuf, &firstword);
    if ( firstword & EPD_SPECIAL_MASK ) {
        SpiNSelLow(mySpi);
        readsize = 0;
        Decode_Special();
        return;
    }

    /* 
     * Otherwise get the maximum read size and scan for special commands
     * if found, reduce the read size up to (but excluding) this word
     */
    uint16_t *p = (uint16_t*)(epdCbuf.buf + epdCbuf.rdptr);

    readsize = CBUF_GET_LINEARREADSIZE(epdCbuf);

    /* Convert readsize to count words for the following lines*/
    readsize /= 2;
    for ( uint32_t i=0; i < readsize; i++ ) if ( *(p+i) & EPD_SPECIAL_MASK ) {
        readsize = i;
        break;
    }
    /* Now convert readsize back to bytes for the addition in OnTxComplete" */
    readsize *= 2;

    SpiNSelLow(mySpi);
    Spi9TxVector_DMA(mySpi, (uint16_t*)(epdCbuf.buf+epdCbuf.rdptr), readsize/2);    
}

static void OnTxComplete (void )
{
     SpiNSelHigh(mySpi);
      epdCbuf.rdptr = CBUFPTR_INCR(epdCbuf, rdptr, readsize);
      /* Check for more */
      TaskNotify(TASK_EPD);
}
#else
    void SSD_AsynchronousWrite(void) {}
    void SSD_Flush(void) {}
#endif /* if DO_ASYNC_WRITE > 0 */

void task_handle_epd ( uint32_t arg )
{
    UNUSED(arg);

    if (SSD_IsInitialized() ) SSD_AsynchronousWrite();
}

#if 0
//-------------------------------------------------------------------------
const unsigned char lut_data[]=
{
   0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,0xAA,
   0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,0x55,0xAA,
   0xAA,0x00,0x55,0x55,0x55,0x55,0xAA,0xAA,0xAA,
   0xAA,0x55,0x55,0x55,0x55,0xAA,0xAA,0xAA,0xAA,
   0x15,0x15,0x15,0x15,0x05,0x05,0x05,0x05,0x01,
   0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,
   0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,0x00,0x00
};

void EPD_TestInit(void)
{
   unsigned char i;

   SSD_WriteCmd(0x10);   //do not enter deep sleep mode
   SSD_WriteData(0x00);

   SSD_WriteCmd(0x11);  //data entry mode setting,0x01,Y decrement,X increment
   SSD_WriteData(0x01);
   SSD_WriteCmd(0x44);  //set RAM X-address start/end position
   SSD_WriteData(0x00); //RAM X -address start at 00H
   SSD_WriteData(0x11); //RAM X-address end at 11H->(17D),that is (17+1*4=72)
   SSD_WriteCmd(0x45);  //set RAM Y-address start/end position
   SSD_WriteData(0xAB); //RAM Y-address start at ABH->(171D)
   SSD_WriteData(0x00); //RAM Y-address end at 00H
   SSD_WriteCmd(0x4E);  //set RAM x address count to 0;
   SSD_WriteData(0x00);
   SSD_WriteCmd(0x4F);  //set RAM Y address count to 172->0;
   SSD_WriteData(0xAB);
   SSD_WriteCmd(0xF0);  //booster feedback selection,0x1F->internal feedback is used
   SSD_WriteData(0x1F); //0x83
   SSD_WriteCmd(0x21);  //bypass the RAM data into the display,enable pass
   SSD_WriteData(0x03);
   SSD_WriteCmd(0x2C);  //write VCOM register
   SSD_WriteData(0xA0);
   SSD_WriteCmd(0x3C);  //board waveform, board voltage
   SSD_WriteData(0x63);
   SSD_WriteCmd(0x22);  //enable sequence, CLK->CP->
   SSD_WriteData(0xC4);


   SSD_WriteCmd(0x32);  //write LUT register
   for(i=0;i<90;i++)
       SSD_WriteData(lut_data[i]);
}

//-------------------------------------------------------------------------
EPD_FillDisplay(uint8_t dat) //0xFF=white, 0x00=black, 0x55=gray 1, 0xAA=gray 2
{
   unsigned int i;

   SSD_WriteCmd(0x24);//data write into RAM after this command

   for(i=0;i<3096;i++) //3096 = 172x72/8x2, (2-Bit per dot)
   {
      SSD_WriteData(dat);
   }
   SSD_WriteCmd(0x20);

   //Booster diable
   SSD_WriteCmd(0x22); //display updata sequence option ,in page 33
   SSD_WriteData(0x02);
   SSD_WriteCmd(0x20);
}
#endif

#endif // #if USE_EPAPER > 0
//-------------------------------------------------------------------------


