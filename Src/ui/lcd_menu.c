/******************************************************************************
 * All the LCD Menu Stuff
 *****************************************************************************/

#include "config/config.h"
#if USE_DISPLAY > 0

#include "eeprom.h"
#include "global_flags.h"


#include "disp/fonts/lcd_fonts.h"
#include "ui/lcd.h"
#include "ui/lcd_valueedit.h"

#define MENU_TIMEOUT 12


#if DEBUG_LCD_MENU > 0
	#include "debug_helper.h"
#endif

static uint8_t item=0;				// Currently selected Menu Item

static char itemText[]="Config[xx]";
#define ITEM_TEXT_NUM_OFFSET 7		// offset of "xx"-Portion in above String

static void insert_indexXX(uint8_t i) {
    uint8_t x = i>>4;
    if (x>=10) 
        x += 'a'-10;
    else 
        x+= '0';
    itemText[ITEM_TEXT_NUM_OFFSET] = x;
    x = i & 0xf;
    if (x>=10)
        x += 'a'-10;
    else
        x+= '0';
    itemText[ITEM_TEXT_NUM_OFFSET+1] = x;
}

/*****************************************************************************
 * Setup all dynamic elements of MenuEdit-Element
 ****************************************************************************/
static void LCD_MenuSetup(uint8_t item)
{
	insert_indexXX(item);
	editItem.initial		= editItem.actual = Config_GetVal(item);
	editItem.maxval			= eelimits[item].max;
	editItem.minval			= eelimits[item].min;
	editItem.bAsHex			= true;
	editItem.itemText		= itemText;
	editItem.itemHelpText           = eelimits[item].help;
	#if DEBUG_LCD_MENU > 2
		printf("HelpText[%d]=%s\n",item, editItem.itemHelpText);
	#endif
}

/*****************************************************************************
 * Initialization of the LCD-Stuff
 ****************************************************************************/
static void LCD_MenuInit(void)
{
	item = 0;
	LCD_MenuSetup(item);
}

/*****************************************************************************
 * Callback for Termination of ValueEdit
 * Return value has been checked for being within min..max-range
 ****************************************************************************/
static void LCD_MenuEditDone ( uint8_t retval)
{
    #if DEBUG_LCD_MENU > 0
        bool ret = 
    #endif
    Config_SetVal(item,retval);

    #if DEBUG_LCD_MENU > 0
        printf("NewValue[%d]=%s, Status %s\n", item, retval, ( ret ? "ok" : "fail" ) );
    #endif
}

/*****************************************************************************
 * Callback for Scroll within list of editable values
 * scroll forward(positive) or backward(negative) by "delta"
 ****************************************************************************/
static void LCD_MenuScroll ( int32_t delta)
{
    int16_t work;
    if ( delta ) {
        work = item + delta;
        if ( work < 0 )  work += Config_GetCnt();
        if ( work >= (int32_t)Config_GetCnt() )  work -= Config_GetCnt();
        #if DEBUG_LCD_MENU > 0
                printf("NewItem=%d\n",work);
        #endif
        item=work;
        LCD_MenuSetup(item);
    }
}

const MenuFeeder ConfigEditFeeder = {
    .Init           = LCD_MenuInit,
    .Scroll         = LCD_MenuScroll,
    .EditDone       = LCD_MenuEditDone,
    .EditTerminate  = NULL,
    .EditItemList   = LCD_EDIT_ALL,
};

#endif // if USE_DISPLAY > 0

