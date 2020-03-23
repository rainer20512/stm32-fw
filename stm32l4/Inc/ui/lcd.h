/******************************************************************************
 * Implements functionality for an pixel lcd
 *****************************************************************************/
#ifndef __LCD_H
#define __LCD_H

#include "config/config.h"

#if USE_PWMTIMER > 0
    #define USE_LCD_BACKLIGHT           1
#endif

#include "dev/qencode.h"         /* Needed for rotary encoder callback types */
#include "ui/lcd_interface.h"

// multi purpose buffer for all kinds to text or number display in the whole LCD-context
#define LCD_NUMBUFLEN	12
extern char LCD_numbuf[];


#if USE_DISPLAY > 0
    uint32_t LCD_ReDraw(void);
    void LDC_Display_Splash(const char * splash);
    void LCD_ClearDisplay(void);
    void LDC_Display_Progress(uint8_t progress);
    void LCD_DisplayStatus(uint32_t lcd_items);
    void LCD_DisplayValueEdit(uint32_t lcd_items);
    void LCD_SwitchToSleep(void);
    void LCD_SwitchToDefault(void);
    void LCD_WakeUp(void);
    void LCD_On(uint8_t uOnTime);

    typedef struct HWDTS HW_DeviceType;
    void LCD_PostInit(const HW_DeviceType *dev, void *bRotate);
    void LCD_AssignQenc(const HW_DeviceType *dev, void *arg);
    void task_handle_lcd ( uint32_t arg );
    void task_init_lcd ( void );

#else
    #define LCD_ReDraw()	0
    #define LCD_DisplayStatus(a)
    #define LCD_DisplayValueEdit(a)
    #define LCD_DisplaySelector(a)
    #define LCD_DisplayText(a)
    #define LDC_Display_Splash(a)
    #define LCD_ClearDisplay(a)
    #define LDC_Display_Progress(a)
    #define LCD_SwitchToStatus()
    #define LCD_SwitchToMenu()
    #define LCD_SwitchToDateTime()
    #define LCD_SwitchToSelector(...)
    #define LCD_SwitchToText(a,b,c)
    #define LCD_SwitchToSleep()
    #define LCD_SwitchToDefault()
    #define WakeUp()
    #define LCD_On(a)
#endif // #if USE_DISPLAY > 0

#if USE_LCD_BACKLIGHT > 0
    void LCD_SetPWMDev( const HW_DeviceType *devPWM, uint32_t chPWM );
    void LCD_SetBacklightPWM(uint32_t int_idx);
    #define LCD_PWM(int) LCD_SetBacklightPWM(int)
#else   
    #define LCD_PWM(int)    
#endif

#endif /* ifndef __LCD_H */