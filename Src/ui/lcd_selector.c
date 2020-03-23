/******************************************************************************
 * All the LCD Menu Stuff
 *****************************************************************************/

#include "config/config.h"
#if USE_DISPLAY > 0

#include <stdint.h>
#include <stdio.h>


#include "debug.h"
#include "eeprom.h"

#include "disp/fonts/lcd_fonts.h"
#include "ui/lcd.h"
#include "ui/lcd_valueedit.h"
#include "ui/lcd_selector.h"
#include "debug_helper.h"

#define SEL_TIMEOUT 12

#if DEBUG_LCD_MENU > 0
    #include "debug_helper.h"
#endif

void LCD_SelectorClick(void);
void LCD_SelectorChangeValue(int32_t delta);

MenuSelector sel;
const MenuExecutor *myExecutor;

static uint32_t InitSelector ( const void *InitArg  )
{

    if ( !InitArg ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("InitSelector - Error: InitArg must be specified!");
        #endif
        LCD_SwitchToDefault();
        return 0;
    }

    /* Get Executor and fill in selection texts */
    myExecutor = (const MenuExecutor *)InitArg;
    myExecutor->TextFiller();
    if ( sel.items_used < 1 ) {
        #if DEBUG_MODE > 0
            DEBUG_PUTS("InitSelector - Error: No selections");
        #endif
        LCD_SwitchToDefault();
        return 0;
    }

    EnableRotary (SEL_TIMEOUT);
    LCD_StartPwrOffTmr(SEL_TIMEOUT);
    LCD_PWM(config.def_intensity);
    return LCD_SELECTOR_FULL;
};


/*****************************************************************************
 * Display the Item value for  "item"
 ****************************************************************************/
static void LCD_SelectorDisplayItem(uint8_t idx)
{
    uint8_t row = idx % 4;
    uint8_t col = ( idx < 4 ? 0 : 66 );
    dogm_moveto_xy(row, col);
    lcd_put_string(FONT_PROP_8, (idx==sel.selected ? DOGM_INVERT : DOGM_NORMAL ), sel.items[idx] );
}


/*****************************************************************************
 * Callback for the rotary's on/off switch while in selector display mode
 * Called with argument != 0 when on, and with argument=0 when off
 * Used to terminate selector mode
 ****************************************************************************/
static void SelectorClick(void)
{
    #if DEBUG_LCD_MENU > 0
        puts("SelectorClick=Return");
    #endif
    // SEC_timer_destroy(SEC_TIMER_LICHTAUS);
    LCD_PWM(0);
    DisableRotary();
    if ( myExecutor && myExecutor->Executor ) myExecutor->Executor(sel.selected);

}


/*****************************************************************************
 * Callback for the rotary's rotary knob in menu mode
 * i.e. change the menu item or the menu value
 ****************************************************************************/
static void SelectorChangeValue(int32_t delta)
{
    #if DEBUG_LCD_MENU > 0
        if ( config.dbg_level > 2 ) {
            printf("SelectorChng(%d)\n",delta);
        }
    #endif

    int32_t work;
    if ( delta ) {
        work = sel.selected+delta;
        sel.last_selected = sel.selected;

        // check range
        if ( work > sel.items_used-1 )  work -= sel.items_used;
        if ( work < 0 )                 work += sel.items_used;
        sel.selected = work;
        #if DEBUG_LCD_MENU > 0
            DEBUG_PRINTF("Selected=%d\n", work);
        #endif
        LCD_DisplaySelector( LCD_SELECTOR_DELTA );

        // Restart Menu Timeout Timer
        ReEnableRotary();
        LCD_RestartPwrOffTmr();
    }
}


/*****************************************************************************
 * Draw the selector Elements
 ****************************************************************************/
static void LCD_SelectorDisplayItems(uint8_t bDrawAll)
{
    uint8_t i;
    if ( bDrawAll) {
        for ( i = 0; i < sel.items_used; i++ )
            LCD_SelectorDisplayItem( i );
    } else {
        if ( sel.last_selected != sel.selected ) LCD_SelectorDisplayItem( sel.last_selected);
        LCD_SelectorDisplayItem( sel.selected);
    }
}

/*****************************************************************************
 * The following functions is called by LCD_ReDraw, when the LCD_SELECTOR flag is set
 ****************************************************************************/
static uint32_t RedrawSelector(uint32_t redraw_bits)
{
    #if DEBUG_LCD_MENU > 0
        if ( config.dbg_level>2) {
            printf("ReDrawStatus=%02x\n",redraw_bits);
        }
    #endif

    if      (redraw_bits & LCD_SELECTOR_FULL )  { redraw_bits &= ~LCD_SELECTOR_FULL  ; LCD_SelectorDisplayItems(1); }
    else if (redraw_bits & LCD_SELECTOR_DELTA ) { redraw_bits &= ~LCD_SELECTOR_DELTA ; LCD_SelectorDisplayItems(0); }

    return redraw_bits;
}

DEFINE_SCREEN(Selector, InitSelector, SelectorClick, NULL, SelectorChangeValue, RedrawSelector, NULL, false);


#endif // if USE_DISPLAY > 0

