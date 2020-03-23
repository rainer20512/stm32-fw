/******************************************************************************
 * Implements functionality for an pixel lcd
 *****************************************************************************/
#ifndef __LCD_INTERFACE_H
#define __LCD_INTERFACE_H

#include "dev/qencode.h"         /* Needed for rotary encoder callback types */
#include "ui/lcd.h"
#include "task/minitask.h"                             

#define LCD_EDIT			(1<<7)				// Draw one menu item
#define LCD_SELECTOR			(1<<6)				// Draw an Selector
#define LCD_STATUS			(1<<5)				// Draw Status screen
#define LCD_TEXT			(1<<4)				// Display a static text
#define LCD_SLEEP			(1<<0)				// LCD is in Sleep Mode




#define LCD_SELECTOR_FULL		(1<<0)			// Print the whole selector 
#define LCD_SELECTOR_DELTA		(1<<1)			// Print only changed items

#define LCD_TEXT_LINE0			(1<<0)			// Fill Line 0
#define LCD_TEXT_LINE1			(1<<1)			// Fill Line 1
#define LCD_TEXT_LINE2			(1<<2)			// Fill Line 2
#define LCD_TEXT_LINE3			(1<<3)			// Fill Line 3
#define LCD_TEXT_ALL			( LCD_TEXT_LINE0 | LCD_TEXT_LINE1 | LCD_TEXT_LINE2 | LCD_TEXT_LINE3 )


// Callback for Selector
typedef void (*PFN_SelectorCallback)(uint32_t);

void ReEnableRotary       (void);
void EnableRotary         (uint32_t rot_timeout_sec);
void DisableRotary        (void);
void LCD_StartPwrOffTmr   ( uint32_t secs_to_pwroff );
void LCD_RestartPwrOffTmr ( void );

typedef struct LCDScreenType {
    uint32_t ( *OnInit      ) ( const void *InitArg); /* Init function */
    void     ( *OnClick     ) ( void );               /* Click Callback */
    void     ( *OnDblClick  ) ( void );               /* DoubleClick Callback */
    void     ( *OnRotary    ) ( int32_t );            /* Rotary callback */
    uint32_t (*OnRedraw     ) ( uint32_t );           /* Redraw_callback */
    void     ( *OnPwrOff    ) ( void );               /* Callback on display power off */
    const void  *InitArg;                             /* optional Argument to Init Fn */
    bool     bDefaultScr;                             /* true for default screen */
} LCDScreenT;

void LCD_Activate ( const LCDScreenT *scr, const void *InitArg );

#define DEFINE_SCREEN(Name, OnInitFn, OnClickFn, OnDblClickFn, OnRotaryFn, OnRedrawFn, OnPwrOffFn, bIsDefault) \
const LCDScreenT Scr_##Name = {      \
      .OnInit       = OnInitFn,      \
      .OnClick      = OnClickFn,     \
      .OnDblClick   = OnDblClickFn,  \
      .OnRotary     = OnRotaryFn,    \
      .OnRedraw     = OnRedrawFn,    \
      .OnPwrOff     = OnPwrOffFn,    \
      .bDefaultScr  = bIsDefault,    \
}; \
\
extern const LCDScreenT *act_lcd_screen;            \
extern uint32_t redraw_items;                       \
void LCD_Display##Name(uint32_t lcd_items)          \
{                                                   \
    if ( act_lcd_screen != &Scr_##Name ) return;    \
                                                    \
    redraw_items |= lcd_items;                      \
    TaskNotify(TASK_LCD);                           \
}                                                   \
                                                    \
void LCD_SwitchTo##Name ( const void *InitArg )     \
{                                                   \
    LCD_Activate(&Scr_##Name, InitArg);             \
}                                                   
 
#define EXPORT_SCREEN(Name)                         \
    extern const LCDScreenT Scr_##Name;             \
    void LCD_SwitchTo##Name(const void *InitArg);   \
    void LCD_Display##Name(uint32_t lcd_items);

#define ADD_SCREEN(Name) &Scr_##Name

#endif /* ifndef __LCD_INTERFACE_H */