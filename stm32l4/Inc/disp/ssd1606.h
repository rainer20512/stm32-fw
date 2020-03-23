/**
  ******************************************************************************
  * @file    gde021a1.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the gde021a1.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSD1606_H
#define __SSD1606_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "disp/epd.h"


/**
  * @brief  SSD1606 Size
  */
#define  SSD1606_EPD_PIXEL_WIDTH    172
#define  SSD1606_EPD_PIXEL_HEIGHT   18

/**
  * @brief  SSD1606 Registers
  */
#define SSD_CMD_STATUSREAD          0x00   /* Status Read */
#define SSD_CMD_DRV_OUT_CTL         0x01   /* Driver Output Control */
#define SSD_CMD_GATE_DRV_V_CTL      0x03   /* Gate driving voltage control */
#define SSD_CMD_SRC_DRV_V_CTL       0x04   /* Source driving coltage control */
#define SSD_CMD_DISP_CTL            0x07   /* Display Control */
#define SSD_CMD_0x0B                0x0B   /* Gate and Sorce non overlap period COntrol */
#define SSD_CMD_0x0F                0x0F   /* Gate scan start */
#define SSD_CMD_DEEPSLEEP           0x10   /* Deep Sleep mode setting */
#define SSD_CMD_DATAENTRY           0x11   /* Data Entry Mode Setting */
#define SSD_CMD_SOFT_RESET          0x12   /* SWRESET */
#define SSD_CMD_TSENDOR_WR          0x1A   /* Temperature Sensor Control (Write to Temp Register) */
#define SSD_CMD_TSENDOR_RD          0x1B   /* Temperature Sensor Control(Read from Temp Register) */
#define SSD_CMD_TSENDOR_WR_CTL      0x1C   /* Temperature Sensor Control(Write Command  to Temp sensor) */
#define SSD_CMD_TSENDOR_LOAD        0x1D   /* Temperature Sensor Control(Load temperature register with temperature sensor reading) */
#define SSD_CMD_MASTERUPDATE        0x20   /* Master activation */
#define SSD_CMD_UPDATE_CTL1         0x21   /* Display update */
#define SSD_CMD_UPDATE_CTL2         0x22   /* Display update control 2 */
#define SSD_CMD_WRITE_RAM           0x24   /* write RAM */
#define SSD_CMD_READ_RAM            0x25   /* Read RAM */
#define SSD_CMD_VCOM_SENSE          0x28   /* VCOM sense */
#define SSD_CMD_VCOM_SENSE_DUR      0x29   /* VCOM Sense duration */
#define SSD_CMD_VCOM_OTP_PGM        0x2A   /* VCOM OTP program */
#define SSD_CMD_VCOM_WRITE          0x2C   /* Write VCOMregister */
#define SSD_CMD_READ_OTP_REG        0x2D   /* Read OTP registers */
#define SSD_CMD_WS_OTP_PGM          0x30   /* Program WS OTP */
#define SSD_CMD_WRITE_LUT           0x32   /* Write LUT register */
#define SSD_CMD_READ_LUT            0x33   /* Read LUT register */
#define SSD_CMD_0x36                0x36   /* Program OTP selection */
#define SSD_CMD_0x37                0x37   /* Proceed OTP selection */
#define SSD_CMD_0x3A                0x3A   /* Set dummy line pulse period */
#define SSD_CMD_0x3B                0x3B   /* Set Gate line width */
#define SSD_CMD_0x3C                0x3C   /* Select Border waveform */
#define SSD_CMD_SET_RAMX_START_END  0x44   /* Set RAM X - Address Start / End Position */
#define SSD_CMD_SET_RAMY_START_END  0x45   /* Set RAM Y - Address Start / End Position */
#define SSD_CMD_SET_RAMX_ACTUAL     0x4E   /* Set RAM X Address Counter */
#define SSD_CMD_SET_RAMY_ACTUAL     0x4F   /* Set RAM Y Address Counter */
#define SSD_CMD_BOOSTER_FEEDBACK    0xF0   /* Booster Set Internal Feedback Selection */
#define SSD_CMD_NOP                 0xFF   /* NOP */



void     ssd1606_Init(const HW_DeviceType *dev, bool bRotate);
void     ssd1606_WriteReg(uint8_t EPD_Reg, uint8_t EPD_RegValue);

/* ! Not supported with serial access ! */
/* uint8_t  ssd1606_ReadReg(uint8_t EPD_Reg); */

void     ssd1606_WriteOnePixel(uint8_t byte);
void     ssd1606_StartConsecutiveWrite( void );
void     ssd1606_ConsecutiveWrite ( uint8_t byte );
void     ssd1606_StopConsecutiveWrite(void);
void     ssd1606_WriteConstantByteVector( uint8_t byte, uint16_t length );

void     ssd1606_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);
void     ssd1606_RefreshDisplay(void);
void     ssd1606_CloseChargePump(void);
bool     ssd1606_IsBusy(void);
void     ssd1606_SetBusyIrqCB ( void ( *pCB ) (uint16_t, void * ) );

void     ssd1606_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
uint16_t ssd1606_GetEpdPixelWidth(void);
uint16_t ssd1606_GetEpdPixelHeight(void);

/* EPD driver structure */
extern EPD_DrvTypeDef   ssd1606_drv;

/* Read operations are not supported with serial access */
uint16_t  EPD_ReadData(void);

/* Testing */
void EPD_fill_display       (uint8_t dat); //0xFF=white, 0x00=black, 0x55=gray 1, 0xAA=gray 2
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SSD1606_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
