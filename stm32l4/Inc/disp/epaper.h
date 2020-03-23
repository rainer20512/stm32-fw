/*
 ******************************************************************************
 * @file    epaper.h
 * @author  Rainer
 * @brief   Implements an Interface for an ePaper Display 
 *          
 ******************************************************************************
 */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EPAPER_H
#define __EPAPER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "disp/fonts/epd_fonts.h"

typedef enum
{
  EPD_OK = 0,
  EPD_ERROR = 1,
  EPD_TIMEOUT = 2
} EPD_StatusTypeDef;

/**
  * @brief  Line mode structures definition
  */
typedef enum
{
  CENTER_MODE             = 0x01,    /*!< Center mode */
  RIGHT_MODE              = 0x02,    /*!< Right mode  */
  LEFT_MODE               = 0x03     /*!< Left mode   */
} Text_AlignModeTypdef;


/**
  * @brief  EPD color
  */
#define EPD_COLOR_BLACK         0x00
#define EPD_COLOR_DARKGRAY      0x55
#define EPD_COLOR_LIGHTGRAY     0xAA
#define EPD_COLOR_WHITE         0xFF

/**
  * @brief EPD default font
  */
#define EPD_DEFAULT_FONT         Font12


void     EPD_PostInit(const HW_DeviceType *dev, void *bRotate);
uint32_t EPD_GetXSize(void);
uint32_t EPD_GetYSize(void);

void     EPD_SetFont(sFONT *pFonts);
sFONT    *EPD_GetFont(void);

void     EPD_Clear(uint16_t Color);

void     EPD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr);
void     EPD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *pText, Text_AlignModeTypdef mode);
void     EPD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);

void     EPD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     EPD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     EPD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     EPD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

void     EPD_RefreshDisplay(void);

void     EPD_CloseChargePump(void);

void     EPD_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

void     EPD_Test(uint8_t num);

#ifdef __cplusplus
}
#endif

#endif /* __EPAPER_H */

