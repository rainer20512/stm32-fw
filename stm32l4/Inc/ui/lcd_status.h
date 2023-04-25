#include "ui/lcd_interface.h"

/******************************************************************************
 * define the various display items                                           
 *****************************************************************************/
#define	LCD_STATUS_BATTERY		(1<<0)
#define LCD_STATUS_TEMP			(1<<1)
#define LCD_STATUS_RFM			(1<<2)
#define LCD_STATUS_TIME			(1<<3)
#define LCD_STATUS_ATEMP0		(LCD_STATUS_BATTERY | LCD_STATUS_TEMP | LCD_STATUS_RFM | LCD_STATUS_TIME)

#if USE_BMP085 > 0
	#define LCD_STATUS_PRESSURE	(1<<4)
	#define LCD_STATUS_ATEMP1	(LCD_STATUS_ATEMP0 | LCD_STATUS_PRESSURE)
#elif USE_BME280 > 0
	#define LCD_STATUS_PRESSURE	(1<<4)
        /**** 007 ****/
	#define LCD_STATUS_RELHUM	(1<<5)
	#define LCD_STATUS_ATEMP1	(LCD_STATUS_ATEMP0 | LCD_STATUS_PRESSURE | LCD_STATUS_RELHUM )
#elif USE_CCS811 > 0
        /**** 002 ****/
	#define LCD_STATUS_CO2          (1<<4)
        #define LCD_STATUS_TVOC         (1<<5)
	#define LCD_STATUS_ATEMP1	(LCD_STATUS_ATEMP0 | LCD_STATUS_CO2 | LCD_STATUS_TVOC)
#else
	#define LCD_STATUS_ATEMP1	LCD_STATUS_ATEMP0
#endif

#if USE_SERIALSTORAGE > 0
	#define LCD_STATUS_EXTMEM	(1<<6)
	#define LCD_STATUS_ATEMP2	(LCD_STATUS_ATEMP1 | LCD_STATUS_EXTMEM)
#else
	#define LCD_STATUS_ATEMP2	LCD_STATUS_ATEMP1
#endif

#if defined(GASSENSOR)
	#define LCD_STATUS_VIN		(1<<6)
	#define LCD_STATUS_ATEMP3	(LCD_STATUS_ATEMP2 | LCD_STATUS_VIN)
#elif defined(TX18LISTENER)
	#define LCD_STATUS_RADIO	(1<<7)
	#define LCD_STATUS_ATEMP3	(LCD_STATUS_ATEMP2 | LCD_STATUS_RADIO)
#else
	#define LCD_STATUS_ATEMP3	LCD_STATUS_ATEMP2
#endif

// Draw all of the above status items
#define LCD_STATUS_ALL			LCD_STATUS_ATEMP3


EXPORT_SCREEN(Status);


uint8_t LCD_ComputeSigned(char *writebuf, int16_t val, uint8_t bAppendBlank);

#if 0
#if defined(GASSENSOR) || defined(STROMSENSOR)
	uint8_t LCD_ComputeCounter(char *writebuf, uint16_t val, uint8_t bWithDP, uint8_t bAppendBlank);
#endif

#if USE_DOGM132 > 0
	void LCD_InitStatus(void);
	uint8_t LCD_RedrawStatus(uint8_t);
	void LCD_EraseEeprom (void );

/*	void LCD_DisplaySigned(int16_t val);
	void LCD_DisplayNumberTest(int8_t number);
	#define LCD_IncMenuLevel() { LCD_MenuLevel++; }
	#define LCD_DecMenuLevel() { if (LCD_MenuLevel)LCD_MenuLevel--; }
*/
#else
	#define LCD_InitStatus()	
	#define LCD_RedrawStatus(a)			0
	#define LCD_EraseEeprom()
/*
	#define LCD_DisplaySigned(a)
	#define LCD_DisplayNumberTest(a) 
	#define LCD_PWM(a)
	#define LCD_IncMenuLevel()
	#define LCD_DecMenuLevel()
*/
#endif

#endif
