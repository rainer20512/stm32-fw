#ifndef __LCD_SELECTOR_H
#define __LCD_SELECTOR_H

#define SELECTOR_PASS_READ_TXT  0                       // Read the text to be displayed
#define SELECTOR_PASS_EXECUTE   1                       // Execute the associated function

#define SELECTOR_ITEMS_MAX	8			// Maximum number of selector Items
typedef struct MenuSelector_t {
	const char* items[SELECTOR_ITEMS_MAX];		// Pointer to selector texts ( PROGMEM-ptr )
	PFN_SelectorCallback cb;			// Callback when finished
	uint8_t items_used;				// Number of Items used
	uint8_t selected;				// Selected Item
	uint8_t last_selected;				// previous selected item
} MenuSelector;

typedef struct MenuExecutor_t {
        void ( *TextFiller ) ( void );                  // Fill in all item texts into "sel" and set the total number of items
        void ( *Executor   ) ( uint32_t );              // Callback on selection of one item 
} MenuExecutor;

#define MAKE_EXECUTOR( Filler, Exec )   \
{                                       \
    .TextFiller = Filler,               \
    .Executor   = Exec,                 \
} 

#define ADD_ITEM(idx, item)     sel.items[idx] = item

extern MenuSelector sel;
extern const MenuExecutor MainExecutor;

EXPORT_SCREEN(Selector);

#endif /* #ifndef __LCD_SELECTOR_H */