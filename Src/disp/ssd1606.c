/*
 ******************************************************************************
 * @file    ssd1606.c
 * @author  Rainer
 * @brief   Driver for SSD1606 epaper driver
 ******************************************************************************
 * @attention
 *    The ssd1606 supports 4 colours ( greyscales ). the maximum resolution is
 *    128x180 pixel. In the documentation the X is used for the smaller side,
 *    i.e. X-Address range is 0 .. 127, Y address range is 0..179
 *    One ssd-ram byte holds the pixel values for 4 consecutive bytes in X-direction
 * 
 *    Whithin this driver X is used for the larger range, i.e. 0..179 and
 *    Y is ised for the smaller range, i.e. 0..127. As you would normally use the
 *    display in an landscape orientation, this seems more intuitive.
 *    But, as a drawback, the code is somewhat complicated, due to two different
 *    meanings of X and Y. We tried to handle all the translation effort between
 *    hardware (X,Y) and driver (X,Y) within the MIN, MAX and ACT-Macros
 *    and within the function ssd1606_SetDisplayWindow.
 *
 *    The display can operate in normal or rotated landscape orientation, In normal
 *    landscape orientation, the hole in the glass is located in the bottom right
 *   
 *    Moreover, the ssd1606 has two display memories, one active ( which will be
 *    mapped to the display on the update command, and one historical. 
 *    With every update command, these memories are swapped. Swapping does not
 *    change the content of the memory. So after an update, you will find the image
 *    from before two updates. Keep that in mind or blank the display memory and
 *    fill completely after and update
 *
 ******************************************************************************
 */

#include "config/config.h"

#if USE_EPAPER > 0

/* Includes ------------------------------------------------------------------*/
#include "disp/ssd1606.h"
#include "disp/ssdxxxx_spi.h"


/* Look-up table for the epaper (90 bytes) */
const unsigned char WF_LUT[]={
  0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,
  0xAA,0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,
  0x55,0xAA,0xAA,0x00,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x15,0x15,0x15,0x15,
  0x05,0x05,0x05,0x05,0x01,0x01,0x01,0x01,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x41,0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,
  0x00,0x00,};

EPD_DrvTypeDef   ssd1606_drv =
{
  ssd1606_Init,
  ssd1606_WriteOnePixel,
  ssd1606_StartConsecutiveWrite,
  ssd1606_ConsecutiveWrite,
  ssd1606_StopConsecutiveWrite,
  ssd1606_WriteConstantByteVector,

  ssd1606_SetDisplayWindow,
  ssd1606_RefreshDisplay,
  ssd1606_CloseChargePump,
  ssd1606_IsBusy,
  ssd1606_SetBusyIrqCB,
  ssd1606_GetEpdPixelWidth,
  ssd1606_GetEpdPixelHeight,
  ssd1606_DrawImage,
};

static bool bRotate;     /* reverse addressing in X and Y-direction */

/**
  * @brief  Initialize the pixel addressing accroding to selected orientation
  * @param  selected orientation
  */
void ssd1606_SetupOrientation(bool bDoRotate)
{
    bRotate = bDoRotate;
}

/* 
 * Notice: X and Y seem to be swapped in column / row addressing,
 * i.e. x i the row address, y is the column address 
 */
#define MIN_Y()  ( bRotate ? SSD1606_EPD_PIXEL_WIDTH-1 : 0 )
#define MIN_X()  ( bRotate ? SSD1606_EPD_PIXEL_HEIGHT-1 : 0 )
#define MAX_Y()  ( bRotate ? 0 : SSD1606_EPD_PIXEL_WIDTH-1  )
#define MAX_X()  ( bRotate ? 0 : SSD1606_EPD_PIXEL_HEIGHT-1 )

/* Return the x- and -y coordinate in dependance of orientation */
#define ACT_Y(y) ( bRotate ? SSD1606_EPD_PIXEL_WIDTH-1-y : y )
#define ACT_X(x) ( bRotate ? SSD1606_EPD_PIXEL_HEIGHT-1-x : x )



/**
  * @brief  Sets a display window.
  * @param  Xpos: specifies the X bottom left position.
  * @param  Ypos: specifies the Y bottom left position.
  * @param  Width: display window width.
  * @param  Height: display window height.
  * @retval None
*/
void ssd1606_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Set Y position and the height */
  SSD_WriteCnDD( SSD_CMD_SET_RAMX_START_END, ACT_X(Ypos), ACT_X(Height) );
  /* Set X position and the width */
    SSD_WriteCnDD( SSD_CMD_SET_RAMY_START_END, ACT_Y(Xpos), ACT_Y(Width) );
  /* Set the height counter */
  SSD_WriteCnD( SSD_CMD_SET_RAMX_ACTUAL, ACT_X(Ypos) );
  /* Set the width counter */
  SSD_WriteCnD( SSD_CMD_SET_RAMY_ACTUAL, ACT_Y(Xpos) );
}

/**
  * @brief  Initialize the SSD1606 EPD Component.
  * @param  the orientation to use
  * @retval None
  */
void ssd1606_Init(const HW_DeviceType *dev, bool bRotate)
{
  /* Store the orientation and initialize pixel adressing */
  ssd1606_SetupOrientation(bRotate);

  /* Initialize the SSD1606 */
  SSD_DevInit(dev);
  SSD_WriteCnD(SSD_CMD_DEEPSLEEP, 0x00 );                   /* Deep sleep mode disable */
  SSD_WriteCnD(SSD_CMD_DATAENTRY, bRotate ? 0x00 : 0x03);   /* Data Entry Mode Setting */

  /* Display Window to whole display */
  ssd1606_SetDisplayWindow(0,0,  SSD1606_EPD_PIXEL_WIDTH-1,  SSD1606_EPD_PIXEL_HEIGHT-1);

  SSD_WriteCnD(SSD_CMD_BOOSTER_FEEDBACK, 0x1F);             /* Booster Set Internal Feedback Selection */
  SSD_WriteCnD(SSD_CMD_UPDATE_CTL1, 0x03);                  /* Disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3 */
  SSD_WriteCnD(SSD_CMD_VCOM_WRITE, 0xA0);                   /* Write VCOMregister */
  SSD_WriteCnD(SSD_CMD_0x3C, 0x64);                         /* Border waveform */


  SSD_WriteCnDataVector(SSD_CMD_WRITE_LUT, WF_LUT, 90);     /* Write LUT register */
}

/**
  * @brief  Writes 4 dots.
  * @param  HEX_Code: specifies the Data to write.
  * @retval None
  * @note Use this, when just writing one Pixel or in general
  *       one byte to RAM. In case of bulk write, 
  *       use the following
  */
void ssd1606_WriteOnePixel(uint8_t HEX_Code)
{
  SSD_WriteCnD(SSD_CMD_WRITE_RAM, HEX_Code);
}

/**
  * @brief  Bulk write of Pixels or generally RAM bytes
  *         Initiate the write sequence with "Start..."
  *         then write to consecutive cells with "ConsecutiveWrite"
  */
void ssd1606_StartConsecutiveWrite( void )
{
  /* Setup SSD1606 to write data on the RAM */
  /* as long as Data byts follow */
  SSD_StartConsecWrite(SSD_CMD_WRITE_RAM);
}

void ssd1606_ConsecutiveWrite ( uint8_t byte )
{
  /* Send the data to write */
  SSD_ConsecWrite(byte);
}

void ssd1606_StopConsecutiveWrite(void)
{
    SSD_StopConsecWrite();
}

/**
  * @brief  Writes an uniform byte vector of size 'length' and
  *         with byte value 'byte' current ram position
  * @param  None
  * @retval The EPD Pixel Width
  */
void ssd1606_WriteConstantByteVector( uint8_t byte, uint16_t length )
{
    SSD_WriteCnConstant(SSD_CMD_WRITE_RAM, byte, length);
}
/**
  * @brief  Gets the EPD pixel Width.
  * @param  None
  * @retval The EPD Pixel Width
  */
uint16_t ssd1606_GetEpdPixelWidth(void)
{
  return (uint16_t)SSD1606_EPD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the EPD pixel Height.
  * @param  None
  * @retval The EPD Pixel Height
  */
uint16_t ssd1606_GetEpdPixelHeight(void)
{
  return (uint16_t)SSD1606_EPD_PIXEL_HEIGHT;
}

/**
  * @brief  Writes to the selected EPD register.
  * @param  EPD_Reg: Address of the selected register.
  * @param  EPD_RegValue: value to write to the selected register.
  * @retval None
  */
void ssd1606_WriteReg(uint8_t EPD_Reg, uint8_t EPD_RegValue)
{
  SSD_WriteCnD(EPD_Reg, EPD_RegValue);
}

#if 0
  * @brief  Reads the selected EPD Register.
  * @param  EPD_Reg: address of the selected register
  * @retval EPD Register Value
  * @Note   read operations are not supported with
  *         serial access mode!
  */
  

uint8_t ssd1606_ReadReg(uint8_t EPD_Reg)
{

  // Write 8-bit Index (then Read Reg)
  SSD_WriteCmd(EPD_Reg);

  // Read 8-bit Reg 
  return (EPD_ReadData());
  return 0;
}
#endif

/******************************************************************************
 * @brief  Do a Refresh of the ePaper display, i.e. copy the content of
 *         pixel buffer to the display. 
 * @param  None
 * @retval None
 * @note   The display is kept running
 *****************************************************************************/
void ssd1606_RefreshDisplay(void)
{
  /* Write on the Display update control register with the update option*/
  SSD_WriteCnD(SSD_CMD_UPDATE_CTL2, 0xC4 );

  /* Launch the update: Nothing should interrupt this sequence in order to avoid display corruption */
  SSD_WriteC(SSD_CMD_MASTERUPDATE);

  /* In asynchronous write mode, start transfer now */
  SSD_Flush();
}

/******************************************************************************
 * @brief  disable charge pump and display clock to save energy
 * @param  None
 * @retval None
 *****************************************************************************/
void ssd1606_CloseChargePump(void)
{
  /* Write on the Display update control register: Disable CP then Disable Clock signal */
  SSD_WriteCnD(SSD_CMD_UPDATE_CTL2, 0x03);

  /* Launch the update: Nothing should interrupt this sequence in order to avoid display corruption */
  SSD_WriteC(SSD_CMD_MASTERUPDATE);
}

bool ssd1606_IsBusy(void)
{
    return SSD_IsBusy();
}

void ssd1606_SetBusyIrqCB ( void ( *pCb )(uint16_t, void *) )
{   
    SSD_SetBusyIrqCB( pCb );
}

/**
  * @brief  Displays picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the EPD
  * @param  Ypos:  Image Y position in the EPD
  * @param  Xsize: Image X size in the EPD
  * @note   Xsize have to be a multiple of 4
  * @param  Ysize: Image Y size in the EPD
  * @retval None
  */
void ssd1606_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t i, j = 0;
  uint8_t pixels_4 = 0;
  uint8_t pixels_4_grey[4] = {0};
  uint8_t nb_4_pixels, data_res = 0;

  (void)Xpos;(void)Ypos;
  /* Prepare the register to write data on the RAM */
  ssd1606_StartConsecutiveWrite();

  /* X size is a multiple of 8 */
  if ((Xsize % 8) == 0)
  {
    for (i= 0; i< ((((Ysize) * (Xsize/4)))/2) ; i++)
    {
      /* Get the current data */
      pixels_4 = pdata[i];
      if (pixels_4 !=0)
      {
        /* One byte read codes 8 pixels in 1-bit bitmap */
        for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
             from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* Two LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* Two LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }

          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
             EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

          /* Send the data to the EPD's RAM through SPI */
          ssd1606_ConsecutiveWrite(data_res);
        }
      }
      else
      {
        /* 1 byte read from xbm files is equivalent to 8 pixels in the
           other words 2 bytes to be transferred */
        ssd1606_ConsecutiveWrite(0xFF);
        ssd1606_ConsecutiveWrite(0xFF);
      }
    }
  }

  /* X size is a multiple of 4 */
  else
  {
    for (i= 0; i< ((((Ysize) * ((Xsize/4)+1))/2)) ; i++)
    {
      /* Get the current data */
      pixels_4 = pdata[i];
      if (((i+1) % (((Xsize/4)+1)/2)) != 0)
      {
        if (pixels_4 !=0)
        {
          /* One byte read codes 8 pixels in 1-bit bitmap */
          for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
          {
            /* Processing 8 pixels */
            /* Preparing the 4 pixels coded with 4 grey level per pixel
               from a monochrome xbm file */
            for (j= 0; j<4; j++)
            {
              if (((pixels_4) & 0x01) == 1)
              {
                /* Two LSB is coding black in 4 grey level */
                pixels_4_grey[j] &= 0xFC;
              }
              else
              {
                /* Two LSB is coded white in 4 grey level */
                pixels_4_grey[j] |= 0x03;
              }
              pixels_4 = pixels_4 >> 1;
            }

            /* Processing 4 pixels */
            /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
               EPD topology */
            data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

            /* Send the data to the EPD's RAM through SPI */
            ssd1606_ConsecutiveWrite(data_res);
          }
        }
        else if (pixels_4 == 0)
        {
          /* One byte read from xbm files is equivalent to 8 pixels in the
             other words Two bytes to be transferred */
          ssd1606_ConsecutiveWrite(0xFF);
          ssd1606_ConsecutiveWrite(0xFF);
        }
      }

      else if (((i+1) % (((Xsize/4)+1)/2)) == 0)
      {
        if (pixels_4 !=0xf0)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
             from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* 2 LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* 2 LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }

          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
             EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

          /* Send the data to the EPD's RAM through SPI */
          ssd1606_ConsecutiveWrite(data_res);
        }
        else if (pixels_4 == 0xf0)
        {
          /* One byte to be transferred */
          ssd1606_ConsecutiveWrite(0xFF);
        }
      }
    }
  }
   ssd1606_StopConsecutiveWrite();
}

void EPD_fill_display(uint8_t dat) //0xFF=white, 0x00=black, 0x55=gray 1, 0xAA=gray 2
{
    uint16_t length = 3096; //3096 = 172x72/8x2, (2-Bit per dot)

    SSD_WriteCnConstant   ( SSD_CMD_WRITE_RAM, dat, length );

    ssd1606_RefreshDisplay();

    //Booster diable
    ssd1606_CloseChargePump();

}

#endif // #if USE_EPAPER > 0