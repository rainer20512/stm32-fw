/*
 ******************************************************************************
 * @file    epaper.c
 * @author  Rainer
 * @brief   Implements an Interface for an epaper display
 ******************************************************************************
 * @attention This implementation bases on an ssd1606 driver
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "config/config.h"

#if USE_EPAPER > 0

#include <stdio.h>
#include "system/profiling.h"
#include "debug_helper.h"
#include "rtc.h"

#include "disp/epaper.h"
#include "disp/ssd1606.h"

/* Specify path relative to include directory */
#include "../src/disp/fonts/epd/font20epd.c"
#include "../src/disp/fonts/epd/font16epd.c"
#include "../src/disp/fonts/epd/font12epd.c"
#include "../src/disp/fonts/epd/font8epd.c"


#if USE_DS18X20 > 0
    #include "ds18xxx20.h"
#endif

static sFONT           *pFont;
static EPD_DrvTypeDef  *epd_drv;

static void BusyCB ( uint16_t pin, void *arg )
{
    (void)arg;(void)pin;
    if ( epd_drv->IsBusy() )
       COM_print_time('B', true);
    else
       COM_print_time('b', true);
}

static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);

/**
  * @brief  Initializes the EPD.
  * @param  None
  * @retval EPD state
  */
void EPD_PostInit(const HW_DeviceType *dev, void *bRotate)
{
    
  /* Default value for the Font */
  pFont = &Font16;

  epd_drv = &ssd1606_drv;

  /* EPD Init */
  epd_drv->Init(dev, (uint32_t)bRotate != 0);

  /* Clear the EPD screen */
  EPD_Clear(EPD_COLOR_WHITE);

  /* Initialize the font */
  EPD_SetFont(&EPD_DEFAULT_FONT);

  epd_drv->SetBusyIrqCB(BusyCB);

}

/**
  * @brief  Gets the EPD X size.
  * @param  None
  * @retval EPD X size
  */
uint32_t EPD_GetXSize(void)
{
  return(epd_drv->GetEpdPixelWidth());
}

/**
  * @brief  Gets the EPD Y size.
  * @param  None   
  * @retval EPD Y size
  */
uint32_t EPD_GetYSize(void)
{
  return(epd_drv->GetEpdPixelHeight());
}

/**
  * @brief  Sets the Text Font.
  * @param  pFonts: specifies the layer font to be used.
  * @retval None
  */
void EPD_SetFont(sFONT *pFonts)
{
  pFont = pFonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used layer font.
  */
sFONT *EPD_GetFont(void)
{
  return pFont;
}

/**
  * @brief  Clears the hole EPD.
  * @param  Color: Color of the background
  * @retval None
  */
void EPD_Clear(uint16_t Color)
{
  epd_drv->SetDisplayWindow(0, 0, 171, 17);
  epd_drv->WriteConstantByteVector( Color, 3096 );
}

/**
  * @brief  Displays one character.
  * @param  Xpos: start column address.
  * @param  Ypos: the Line where to display the character shape.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void EPD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  Ascii -= 32;
  
  DrawChar(Xpos, Ypos, &pFont->table[Ascii * ((pFont->Height) * (pFont->Width))]);
}

/**
  * @brief  Displays characters on the EPD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Text: Pointer to string to display on EPD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE  
  * @retval None
  */
void EPD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0; 
  uint8_t  *ptr = Text;
  
  /* Get the text size */
  while (*ptr++) size ++ ;
  
  /* Characters number per line */
  xsize = (EPD_GetXSize()/pFont->Width);
  
  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size)* pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn =  - Xpos + ((xsize - size)*pFont->Width);
      break;
    }    
  default:
    {
      refcolumn = Xpos;
      break;
    }
  }
  
  /* Send the string character by character on EPD */
  while ((*Text != 0) & (((EPD_GetXSize() - (i*pFont->Width)) & 0xFFFF) >= pFont->Width))
  {
    /* Display one character on EPD */
    EPD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}

/**
  * @brief  Displays a character on the EPD.
  * @param  Line: Line where to display the character shape
  *          This parameter can be one of the following values:
  *            @arg  0..8: if the Current fonts is Font8
  *            @arg  0..5: if the Current fonts is Font12
  *            @arg  0..3: if the Current fonts is Font16
  *            @arg  0..2: if the Current fonts is Font20
  * @param  ptr: Pointer to string to display on EPD
  * @retval None
  */
void EPD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  EPD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
  * @brief  Draws an horizontal line.
  * @param  Xpos: X position 
  * @param  Ypos: Y position
  * @param  Length: line length
  * @retval None
  */
void EPD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

  epd_drv->SetDisplayWindow(Xpos, Ypos, Xpos + Length, Ypos);

  epd_drv->StartConsecutivePixelWrite();
  for(index = 0; index < Length; index++)
  {
    /* Prepare the register to write data on the RAM */
    epd_drv->WriteConsecutivePixel(0x3F);
  }
  epd_drv->StopConsecutivePixelWrite();
}

/**
  * @brief  Draws a vertical line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: line length.
  * @retval None
  */
void EPD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;
  
  epd_drv->SetDisplayWindow(Xpos, Ypos, Xpos, Ypos + Length);
  
  epd_drv->StartConsecutivePixelWrite();
  for(index = 0; index < Length; index++)
  {
    /* Prepare the register to write data on the RAM */
    epd_drv->WriteConsecutivePixel(0x00);
  }
  epd_drv->StopConsecutivePixelWrite();
}

/**
  * @brief  Draws a rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Height: rectangle height
  * @param  Width: rectangle width
  * @retval None
  */
void EPD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  EPD_DrawHLine(Xpos, Ypos, Width);
  EPD_DrawHLine(Xpos, (Ypos + Height), (Width + 1));
  
  /* Draw vertical lines */
  EPD_DrawVLine(Xpos, Ypos, Height);
  EPD_DrawVLine((Xpos + Width), Ypos , Height);
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: X position.
  * @param  Ypos: Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void EPD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint16_t index = 0;

  /* Set the rectangle */
  epd_drv->SetDisplayWindow(Xpos, Ypos, (Xpos + Width), (Ypos + Height));

  epd_drv->StartConsecutivePixelWrite();
  for(index = 0; index < 3096; index++)
  {
    epd_drv->WriteConsecutivePixel(0xFF);
  }
  epd_drv->StopConsecutivePixelWrite();
}

/**
  * @brief  Draws an Image.
  * @param  Xpos: X position in the EPD
  * @param  Ypos: Y position in the EPD
  * @param  Xsize: X size in the EPD
  * @param  Ysize: Y size in the EPD
  * @param  pdata: Pointer to the Image address
  * @retval None
  */
void EPD_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  /* Set display window */
  epd_drv->SetDisplayWindow(Xpos, Ypos, (Xpos+Ysize-1), (Ypos+(Xsize/4)-1));
  
  if(epd_drv->DrawImage != NULL)
  {
    epd_drv->DrawImage(Xpos, Ypos, Xsize, Ysize, pdata);
  }
  epd_drv->SetDisplayWindow(0, 0, EPD_GetXSize(), EPD_GetYSize());
}

/**
  * @brief  Disables the clock and the charge pump.
  * @param  None
  * @retval None
  */
void EPD_CloseChargePump(void)
{
  /* Close charge pump */
  epd_drv->CloseChargePump();

  /* Add a 400 ms delay */
  // RHB todo? HAL_Delay(400);
}

/**
  * @brief  Updates the display from the data located into the RAM.
  * @param  None
  * @retval None
  */
void EPD_RefreshDisplay(void)
{
  /* Refresh display sequence */
  epd_drv->RefreshDisplay();

  /* Poll on the BUSY signal and wait for the EPD to be ready */
  // while (HAL_GPIO_ReadPin(EPD_BUSY_GPIO_PORT, EPD_BUSY_PIN) != (uint16_t)RESET);

  /*  EPD reset pin mamagement */

  /* Add a 10 ms Delay after EPD pin Reset */
  // RHB todo ?? HAL_Delay(10);
}

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Draws a character on EPD.
  * @param  Xpos: specifies the X position, can be a value from 0 to 171
  * @param  Ypos: specifies the Y position, can be a value from 0 to 17
  * @param  c: pointer to the character data
  * @retval None
  */
static void  DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t index = 0;
  uint32_t data_length = 0;
  uint16_t height = 0;
  uint16_t width = 0;

  width  = pFont->Width;
  height = pFont->Height;
  
  /* Set the Character display window */
  epd_drv->SetDisplayWindow(Xpos, Ypos, (Xpos + width - 1), (Ypos + height - 1));
  
  data_length = (height * width);
    
  epd_drv->StartConsecutivePixelWrite();
  for(index = 0; index < data_length; index++)
  {
    epd_drv->WriteConsecutivePixel(c[index]);
  }
  epd_drv->StopConsecutivePixelWrite();
}

void EPD_Test(uint8_t num)
{
    #define TXTBUFSIZE  15
    char txtbuf[TXTBUFSIZE];

    ProfilerPush(JOB_TASK_EPD);
    EPD_Clear(EPD_COLOR_WHITE);
#if USE_DS18X20 > 0
    DS18X20_GetTempStr(DS18X20_GetTemp(), txtbuf, TXTBUFSIZE);
#else
    strcpy(txtbuf, "Uhrzeit");
#endif
    EPD_DisplayStringAtLine(num+1, (uint8_t *)txtbuf);
    EPD_DisplayStringAtLine(num,   (uint8_t *)RTC_GetStrDateTime());
    EPD_RefreshDisplay();
    ProfilerPop();
}

#endif // #if USE_EPAPER > 0
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
