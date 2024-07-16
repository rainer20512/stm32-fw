/******************************************************************************
 * All the LCD Menu Stuff
 *****************************************************************************/

#include "config/config.h"

#if USE_DISPLAY > 0

#include <stdint.h>

#include "disp/fonts/lcd_fonts.h"
#include "ui/lcd.h"
#include "ui/lcd_valueedit.h"


#include "debug_helper.h"
#include "eeprom.h"

#define MENU_TIMEOUT 12

void LCD_ValueEditClick(void);
void LCD_ValueEditChangeValue(int32_t delta);
void EditTerminate(void);

static uint32_t bEditValue;			// Flag for "Editing of item value is active"
static MenuFeeder *feeder;                      // actual edit item feeder 

MenuEdit editItem;                              // actual (currently displayed ) edit item


/*****************************************************************************
 * convert uint8_t to hexbyte str in LCD_numbuf
 ****************************************************************************/
static void LCD_hexXX(uint8_t i) 
{
    /*****************************************************************************
     * Display hex nibble
     ****************************************************************************/
    uint8_t LCD_hexX(uint8_t n) {
            if (n>=10) {
                    return n + ('a'-10);	
            } else {
                    return n + '0';
            }	            
    }

    LCD_numbuf[0]= LCD_hexX(i>>4);
    LCD_numbuf[1]= LCD_hexX(i & 0xf );
    LCD_numbuf[2]= '\0';
}


/*
static void LCD_ValueEditDisplayNumber ( uint8_t num, uint8_t style )
{ 
	uint8_t t = (num/10);
	uint8_t e = num % 10;
	lcd_put_char(FONT_PROP_16, style, '0'+t);
	lcd_put_char(FONT_PROP_16, style, '0'+e);
} */

/*****************************************************************************
 * Display the ItemHelpText for editItem
 ****************************************************************************/
static void LCD_ValueEditDisplayItemHelp(void)
{
    const char *ofs = editItem.itemHelpText;

    // Check for NULL, which means: No helptext
    if ( !ofs ) return;

    const char *nl = strchr(ofs,'\n');
    dogm_moveto_xy(2,0);
    if ( nl ) {
        lcd_put_string_length(FONT_PROP_8, NORMAL, ofs, nl-ofs);
        dogm_clear_area(2,132, NORMAL);
        nl++;
    } else {
        dogm_clear_area(2,132, NORMAL);
        nl = ofs;
    }
    dogm_moveto_xy(3,0);
    lcd_put_string(FONT_PROP_8, NORMAL, nl );
    dogm_clear_area(3,132, NORMAL);
}

/*****************************************************************************
 * Display the Item value for  editItem
 ****************************************************************************/
static void LCD_ValueEditDisplayItemValue(void)
{
	dogm_moveto_xy(0,80);
	if ( editItem.bAsHex ) {
		LCD_hexXX(editItem.actual);
	} else { 
		my_itoa ( (uint16_t)editItem.actual, LCD_numbuf, 3, false);
	}
	lcd_put_string(FONT_PROP_16, (bEditValue ? INVERT : NORMAL ), LCD_numbuf);
	dogm_clear_area(2, 132, NORMAL);
}

/*****************************************************************************
 * Display the Item  text for editItem
 *****************************************************************************/
static void LCD_ValueEditDisplayItem(void)
{
    dogm_moveto_xy(0,0);
    lcd_put_string(  FONT_PROP_16, (bEditValue ? NORMAL : INVERT), editItem.itemText);
    dogm_clear_area(2, 79, NORMAL);
}

/*****************************************************************************
 * Initialization of the LCD-Stuff
 ****************************************************************************/
uint32_t Init(const void *InitArg)
{
    #if DEBUG_LCD_MENU > 0
            DEBUG_PUTS("ValueEdit Init");
    #endif

    feeder = (MenuFeeder *)InitArg;
    if ( !feeder ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("ValueEdit Init - Error: No feeder specified!");
        #endif
        LCD_SwitchToDefault();
        return 0;
    }

    if ( !feeder->EditDone || !feeder->Scroll ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("ValueEdit Init - Error: mandatory EditDone or Scroll Fn specified");
        #endif
        LCD_SwitchToDefault();
        return 0;
    }


    if ( feeder->Init ) feeder->Init();

    // Return to Main Display after some time of inactivity
    LCD_StartPwrOffTmr( MENU_TIMEOUT);
    EnableRotary (MENU_TIMEOUT);

    LCD_PWM(config.def_intensity);
    return feeder->EditItemList;
}

/*****************************************************************************
 * Return to main Menu, either by Timeout or by Button DblClick
 ****************************************************************************/
void LCD_ValueEditReturn(void)
{
    #if DEBUG_LCD_MENU > 0
            DEBUG_PUTS("ValueEdit Return");
    #endif
    // SEC_timer_destroy(SEC_TIMER_LICHTAUS);
    LCD_PWM(0);

    if ( bEditValue ) {
            // Save actual value, if changed 
            if ( editItem.actual != editItem.initial ) 
                feeder->EditDone(editItem.actual);
            bEditValue = false;
    }

    // Call Termination CB, if specified, otherwise switch to default screen
    if ( feeder->EditTerminate) 
        feeder->EditTerminate();
    else
        LCD_SwitchToDefault();
}

void EditTerminate(void)
{
    /* Will call LCD_ValueEditChangeValue with parameter 0, which will call the */
     /* terminate function LCD_ValueEditReturn                                  */
    DisableRotary();
}

/*****************************************************************************
 * Callback for the rotary's rotary knob in menu mode
 * i.e. change the menu item or the menu value
 ****************************************************************************/
void OnRotary(int32_t delta)
{
    int16_t work;
    if ( delta ) {
        if ( bEditValue ) {
            work = editItem.actual + delta;

            /* Changed the rollover behaviour: When at limit, stay at limit */
            /* if ( work > editItem.maxval ) work = editItem.minval + work - editItem.maxval - 1;
            if ( work < editItem.minval ) work = editItem.maxval - ( editItem.minval - work - 1); */
            if ( work > editItem.maxval ) work = editItem.maxval;
            if ( work < editItem.minval ) work = editItem.minval;
            if ( editItem.actual != work ) {
                editItem.actual = (uint8_t)work;
                if ( editItem.OnUpdate ) editItem.OnUpdate( (uint32_t) work );
                #if DEBUG_LCD_MENU > 0
                        DEBUG_PRINTF("NewValue=%d\n", work);
                #endif
            LCD_DisplayValueEdit( LCD_EDIT_ITEM_VALUE );
            }
        } else {
            // Scroll Mode : Call Feeder to update edit data and display
            feeder->Scroll(delta);
            LCD_DisplayValueEdit(feeder->EditItemList);
        }
        ReEnableRotary();
        LCD_RestartPwrOffTmr();
    } else {
        // delta == 0 means: Final call
        LCD_ValueEditReturn();
    }
}

/*****************************************************************************
 * Callback for the rotary's on/off switch while in menu display mode
 * Called with argument != 0 when on, and with argument=0 when off
 ****************************************************************************/
void OnClick(void)
{
    if ( bEditValue ) {
        // Call Edit Done Callback
        if ( editItem.actual != editItem.initial ) feeder->EditDone(editItem.actual);
    }
    bEditValue = !bEditValue;

    LCD_DisplayValueEdit( LCD_EDIT_ITEM_NAME | LCD_EDIT_ITEM_VALUE | LCD_EDIT_ITEM_HELP );

    // Restart ValueEdit Timeout Timer
    ReEnableRotary();
    LCD_RestartPwrOffTmr();
}



/*****************************************************************************
 * The following functions is called by LCD_ReDraw, when the LCD_MENU falg is set
 ****************************************************************************/
uint32_t OnRedraw(uint32_t redraw_bits)
{
    #if DEBUG_LCD_MENU > 0
        if ( config.dbg_level>2) {
            DEBUG_PRINTF("ReDrawValueEdit(%02x)\n",redraw_bits);
        }
    #endif

    if      (redraw_bits & LCD_EDIT_ITEM_NAME ) { redraw_bits &= ~LCD_EDIT_ITEM_NAME ; LCD_ValueEditDisplayItem(); }
    else if (redraw_bits & LCD_EDIT_ITEM_VALUE) { redraw_bits &= ~LCD_EDIT_ITEM_VALUE; LCD_ValueEditDisplayItemValue(); }
    else if (redraw_bits & LCD_EDIT_ITEM_HELP ) { redraw_bits &= ~LCD_EDIT_ITEM_HELP ; LCD_ValueEditDisplayItemHelp(); }

    return redraw_bits;
}


DEFINE_SCREEN(ValueEdit,    Init, OnClick, EditTerminate, OnRotary, OnRedraw, EditTerminate, false);

#endif // if USE_DISPLAY > 0

