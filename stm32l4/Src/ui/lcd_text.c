/******************************************************************************
 * Stuff do display a static Text on Display
 *****************************************************************************/

#include "config/config.h"
#if USE_DISPLAY > 0

#include "task/minitask.h"
#include "eeprom.h"
#include "version.h"
#include "timer.h"

#include "disp/fonts/lcd_fonts.h"
#include "ui/lcd.h"
#include "ui/lcd_text.h"

#if DEBUG_LCD_MENU > 0
	#include "debug_helper.h"
#endif

#define TXT_NOTIMPL     "Not implemented"
#define TEXT_TIMEOUT    30                  /* Timeout [s] when displaying text */

static uint8_t display_mode;

/*****************************************************************************
* Display Version string in line 1 and line 2
* Display eeprom useage centered in line 3 and line 4
****************************************************************************/
static uint8_t LDC_Display_EEPROM_useage(uint8_t redraw_bits) 
{

    uint16_t pixlen;
    #if USE_25X512_EEPROM > 0 || USE_FM24V10 > 0
            char *text;
    #endif
    const char *ptext;
    
    if ( redraw_bits & LCD_TEXT_LINE0 ) {
        ptext = APP_STRING;
        pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, ptext);
        dogm_moveto_xy(0,(131-pixlen)/2);
        lcd_put_string(FONT_PROP_8, NORMAL, ptext);
        redraw_bits &= ~LCD_TEXT_LINE0;
    } else if ( redraw_bits & LCD_TEXT_LINE1 ) {
        ptext = BUILD_STRING;
        pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, ptext);
        dogm_moveto_xy(1,(131-pixlen)/2);
        lcd_put_string(FONT_PROP_8, NORMAL, ptext);
        redraw_bits &= ~LCD_TEXT_LINE1;
    } else if ( redraw_bits & LCD_TEXT_LINE2 ) {
        #if USE_SERIALSTORAGE > 0	 
            text = GetExtmemBlockUseageStr(true);
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(2,(131-pixlen)/2);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
        #endif
        redraw_bits &= ~LCD_TEXT_LINE2;
    } else if ( redraw_bits & LCD_TEXT_LINE3 ) {
        #if USE_SERIALSTORAGE > 0
            text = GetExtmemByteUseageStr(true);
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(3,(131-pixlen)/2);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
        #endif
        redraw_bits &= ~LCD_TEXT_LINE3;
    }
    return redraw_bits;
}

/*****************************************************************************
* Display a "not implemented" message in line 1
****************************************************************************/
static uint8_t LDC_Display_not_implemented(uint8_t redraw_bits) 
{

    uint16_t pixlen;
    #if USE_25X512_EEPROM > 0 || USE_FM24V10 > 0
            char *text;
    #endif
    const char *ptext;
    
    if ( redraw_bits & LCD_TEXT_LINE0 ) {
        redraw_bits &= ~LCD_TEXT_LINE0;
    } else if ( redraw_bits & LCD_TEXT_LINE1 ) {
        ptext = TXT_NOTIMPL;
        pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, ptext);
        dogm_moveto_xy(1,(131-pixlen)/2);
        lcd_put_string(FONT_PROP_8, NORMAL, ptext);
        redraw_bits &= ~LCD_TEXT_LINE1;
    } else if ( redraw_bits & LCD_TEXT_LINE2 ) {
        redraw_bits &= ~LCD_TEXT_LINE2;
    } else if ( redraw_bits & LCD_TEXT_LINE3 ) {
        redraw_bits &= ~LCD_TEXT_LINE3;
    }
    return redraw_bits;
}

#if LCD_TEXT_TESTING > 0
    /*****************************************************************************
    * Display Test Text in line 0-3
    ****************************************************************************/
    static uint8_t LCD_DisplayTestText(uint8_t redraw_bits)
    {

        uint16_t pixlen;
        const char *text;

        if ( redraw_bits & LCD_TEXT_LINE0 ) {
            text = "Line0";
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(0,0);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
            redraw_bits &= ~LCD_TEXT_LINE0;
        } else if ( redraw_bits & LCD_TEXT_LINE1 ) {
            text = "Line1";
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(1,(132-pixlen)/2);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
            redraw_bits &= ~LCD_TEXT_LINE1;
        } else if ( redraw_bits & LCD_TEXT_LINE2 ) {
            text = "Line2";
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(2,132-pixlen);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
            redraw_bits &= ~LCD_TEXT_LINE2;
        } else if ( redraw_bits & LCD_TEXT_LINE3 ) {
            text = "LongLongLine3";
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, text);
            dogm_moveto_xy(3,(131-pixlen)/2);
            lcd_put_string(FONT_PROP_8, NORMAL, text);
            redraw_bits &= ~LCD_TEXT_LINE3;
        }
        return redraw_bits;
    }
#endif

void LCD_TextClick ( void )
{
    LCD_PWM(0);
    LCD_SwitchToDefault();
}

uint32_t LCD_InitText ( void *InitArg )
{
    display_mode = (uint32_t)InitArg;

    DisableRotary ();
    LCD_PWM(config.def_intensity);

    // Return to Default screen on Timeout
    LCD_StartPwrOffTmr(TEXT_TIMEOUT);

    return LCD_TEXT_ALL;
};







/*****************************************************************************
 * The following functions is called by LCD_ReDraw, when the LCD_TEXT flag is set
 ****************************************************************************/
uint32_t LCD_RedrawText(uint32_t redraw_bits)
{

    #if DEBUG_LCD_MENU > 0
        if ( config.dbg_level>2) {
            DEBUG_PRINTF("ReDrawText(%02x)\n",redraw_bits);
        }
    #endif

    switch ( display_mode )
    {
        case LCD_TEXT_EEPROM_USEAGE:
            redraw_bits = LDC_Display_EEPROM_useage(redraw_bits);
            break;
        case LCD_TEXT_NOT_IMPLEMENTED:
            redraw_bits = LDC_Display_not_implemented(redraw_bits);
            break;
#if LCD_TEXT_TESTING > 0
        case LCD_TEXT_TEST:
            redraw_bits = LCD_DisplayTestText(redraw_bits);
            break;
#endif
        default:
            redraw_bits = 0;
            LCD_TextClick();
    }
    return redraw_bits;
}

DEFINE_SCREEN(Text, LCD_InitText, LCD_TextClick, NULL, NULL, LCD_RedrawText, NULL, false);

#endif // if USE_DISPLAY > 0

