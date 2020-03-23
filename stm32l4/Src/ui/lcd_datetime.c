/******************************************************************************
 * All the LCD Menu Stuff
 *****************************************************************************/

#include "config/config.h"
#if USE_DISPLAY > 0

#include "eeprom.h"
#include "rtc.h"
#include "disp/fonts/lcd_fonts.h"
#include "ui/lcd.h"
#include "ui/lcd_valueedit.h"

#if DEBUG_LCD_MENU > 0 || DEBUG_MODE > 0
    #include "debug_helper.h"
#endif


#define MAX_DT_ITEMS	6
// 6 Elements: DD, MM, YY, HH, MI, SS

static uint8_t item=0;				// Currently selected Menu Item

const char dt00[] = "Tag";
const char dt01[] = "Monat";
const char dt02[] = "Jahr";
const char dt03[] = "Stunde";
const char dt04[] = "Min";
const char dt05[] = "Sek";


static const char * const dt_item_text[] = { dt00, dt01, dt02, dt03,dt04, dt05 };
static const uint8_t config_min[]= { 1, 1, 0, 0, 0, 0};
static const uint8_t config_max[]= {31,12,99,23,59,59};


#define MAX_ITEMTXTLEN	12
static char itemText[MAX_ITEMTXTLEN];

/*****************************************************************************
 * Copy item text from Flash to RAM
 ****************************************************************************/
static char *get_item_text(uint8_t item)
{
    const char *textptr;
    // Get Ptr to correct flash location from flash	
    textptr = dt_item_text[item];
    // and copy to RAM
    strncpy(itemText, textptr, (size_t)MAX_ITEMTXTLEN);
    itemText[MAX_ITEMTXTLEN-1] = '\0'; // Add terminating \0 in case of too long strings
    return itemText;
}

/*****************************************************************************
 * Get the item value ( date_time-Element )
 ****************************************************************************/
static uint8_t get_item_value ( uint8_t item )
{
    switch ( item ) {
        case 0: // Day
            item = RTC_GetDay();
            break;
        case 1: // Month
            item = RTC_GetMonth();
            break;
        case 2: // Year
            item = RTC_GetYearYY();
            break;
        case 3: // Hour
            item = RTC_GetHour();
            break;
        case 4: // Minute
            item = RTC_GetMinute();
            break;
        case 5: // Second
            item = RTC_GetSecond();
            break;
        default:
            #if DEBUG_MODE > 0
                DEBUG_PUTS("get_item_value: Item out of Range[0..5]");
            #endif
                ;
    }
    return item;
}

/*****************************************************************************
 * Set the item value ( date_time-Element )
 ****************************************************************************/
static uint8_t set_item_value ( uint8_t item, uint8_t value )
{
    UNUSED(value);
    switch ( item ) {
        case 0: // Day
            //RHB todo RTC_SetDay(value);
            break;
        case 1: // Month
            //RHB todo RTC_SetMonth(value);
            break;
        case 2: // Year
            //RHB todo RTC_SetYear(value);
            break;
        case 3: // Hour
            //RHB todo RTC_SetHour(value);
            break;
        case 4: // Minute
            //RHB todo RTC_SetMinute(value);
            break;
        case 5: // Second
            //RHB todo RTC_SetSecond(value);
            break;
        default:
            #if DEBUG_MODE > 0
                DEBUG_PUTS("get_item_value: Item out of Range[0..5]");
            #endif
            ;
    }
    return item;
}

/*****************************************************************************
 * Setup all dynamic elements of MenuEdit-Element
 ****************************************************************************/
static void LCD_DTSetup(uint8_t item)
{
    editItem.initial	= editItem.actual = get_item_value( item );
    editItem.maxval	= config_max[item];
    editItem.minval	= config_min[item];
    editItem.bAsHex	= false;
    editItem.itemText	= get_item_text(item);
    editItem.itemHelpText= NULL;
    #if DEBUG_MODE > 0
            DEBUG_PRINTF("Actual=%d\n", editItem.actual);
            DEBUG_PRINTF("Min=%d\n",    editItem.minval);
            DEBUG_PRINTF("Max=%d\n",    editItem.maxval);
    #endif
}

/*****************************************************************************
 * Initialization of the LCD-Stuff
 ****************************************************************************/
static void LCD_DateTimeInit(void)
{
	item = 0;
	LCD_DTSetup(item);
}

/*****************************************************************************
 * Callback for Termination of ValueEdit
 * Return value has been checked for being within min..max-range
 ****************************************************************************/
static void LCD_DTEditDone ( uint8_t retval)
{
	set_item_value(item,retval);
	#if DEBUG_LCD_MENU > 0
		DEBUG_PRINTF("NewValue[%d]=%02x\n",item,retval);
	#endif
}


/*****************************************************************************
 * Callback for Scroll within list of editable values
 * scroll forward(positive) or backward(negative) by "delta"
 ****************************************************************************/
static void LCD_DTScroll ( int32_t delta)
{
    int16_t work;
    if ( delta ) {
        work = item + delta;
        if ( work < 0 )  work +=  MAX_DT_ITEMS;
        if ( work >=  MAX_DT_ITEMS )  work -= MAX_DT_ITEMS;
        #if DEBUG_LCD_MENU > 0
                DEBUG_PRINTF("NewItem=%d\n",work);
        #endif
        item=work;
        LCD_DTSetup(item);
    }
}

const MenuFeeder DateTimeEditFeeder = {
    .Init           = LCD_DateTimeInit,
    .Scroll         = LCD_DTScroll,
    .EditDone       = LCD_DTEditDone,
    .EditTerminate  = NULL,
    .EditItemList   = LCD_EDIT_ITEM_NAME | LCD_EDIT_ITEM_VALUE,
};

#endif // if USE_DISPLAY > 0

