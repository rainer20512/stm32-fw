#ifndef __LCD_VALUEEDIT_H
#define __LCD_VALUEEDIT_H

#include "ui/lcd.h"

#define LCD_EDIT_ITEM_HELP		(1<<0) 				// One Menu item's help text
#define LCD_EDIT_ITEM_VALUE		(1<<1) 				// One Menu item's value
#define LCD_EDIT_ITEM_NAME		(1<<2) 				// One Menu item's Name
#define LCD_EDIT_ALL 			( LCD_EDIT_ITEM_HELP | LCD_EDIT_ITEM_VALUE | LCD_EDIT_ITEM_NAME )

typedef struct MenuEdit_t {
	char* itemText;			// Pointer to Item text ( RAM-ptr ), must not be NULL
	const char *itemHelpText;	// Pointer to Item help text ( EEPROM-ptr ), may be NULL
	uint8_t actual;			// actual value					
	uint8_t initial;		// initial value when edit started
	uint8_t minval;			// minimum of allowed range
	uint8_t maxval;			// maximum of allowed range
	uint8_t bAsHex;			// Display as Hex or as Decimal value ?
} MenuEdit;

/******************************************************************************
 * Struct of functions to dynamically feed the ValueEdit-Screen
 * The follwoing elements are contained
 * - Init - called once on Initialization, can be used to init the feeder 
 *          structures, may be NULL
 * - Scroll - called on every scroll thru the edit items. Called function
 *            is responsible to change the edit items on scroll, mandatory
 * - EditDone - called on change of edited value, mandatory
 * - EditTerminate - If not NULL this fn is called instead of switching back
 *                   to default screen on edit termination
 * - EditItemList - determines the elements to be displayed on edit screen
 *                  any set of 
                    LCD_EDIT_ITEM_NAME, LCD_EDIT_ITEM_VALUE, LCD_EDIT_ITEM_HELP
 *****************************************************************************/
typedef struct MenuFeeder_t {
    void     ( *Init ) ( void );        // Init function for Feeder
    void     ( *Scroll ) ( int32_t);    // Callback for scroll elements list
    void     ( *EditDone ) ( uint8_t ); // Writeback Callback
    void     ( *EditTerminate) (void);  // Termination callback
    uint32_t EditItemList;              // List of items itmes to be editable
} MenuFeeder;

extern MenuEdit editItem;

extern const MenuFeeder ConfigEditFeeder;
extern const MenuFeeder DateTimeEditFeeder;

EXPORT_SCREEN(ValueEdit);

#endif /* #ifndef __LCD_VALUEEDIT_H */