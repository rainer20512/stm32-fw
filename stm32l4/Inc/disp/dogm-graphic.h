#ifndef DOGMGRAPHIC_H_INCLUDED
#define DOGMGRAPHIC_H_INCLUDED

#include "config/config.h"

/*****************************************************************************
 * BEGIN CONFIG BLOCK
 *****************************************************************************/
// Select the display type: DOGS102: 102, DOGM128/DOGL128: 128, DOGM132: 132, DOGXL160: 160
#define DISPLAY_TYPE                    132

// Display Orientation: Normal (0) or upside-down (1)?
#define ORIENTATION_UPSIDEDOWN          0

#define DOGM_USE_RESET                  0		// Reset by RC

// Define this if LCD Output should continue in next line when reaching edge of display
// Used for all outputs. To enable this feature for text only, use the appropriate flag in font.h
#define DOGM_WRAP_AROUND                1

// Include graphic functions, i.e. dogm_draw_image_P, dogm_draw_image_xy_P, dogm_clear_area ? 
#define DOGM_INCLUDE_GRAPHIC_FUNCTIONS  1

/*****************************************************************************
 * END CONFIG BLOCK
 *****************************************************************************/



/*****************************************************************************
 * Public Functions
 *****************************************************************************/

//initializes the display in standard settings
//for DOGS, DOGM, DOGL and DOGXL
typedef struct HWDTS HW_DeviceType;
void dogm_init(const HW_DeviceType *dev, bool bRotate);

/* clear the display ( write with fixed pattern ) */
void dogm_clear_display( uint8_t bckgnd );

/* write data word or command to the LCD */
void dogm_data     (uint8_t data);
void dogm_command  (uint8_t cmd);

/* Get the current position */
extern uint8_t dogm_current_page;
extern uint8_t dogm_current_column;
static inline uint8_t dogm_get_position_page(void)   {return dogm_current_page;}
static inline uint8_t dogm_get_position_column(void) {return dogm_current_column;}

/* Copy Raw data from Flash to display */
#if DOGM_INCLUDE_GRAPHIC_FUNCTIONS >= 1
  void dogm_draw_image(const uint8_t *image, uint8_t pages, uint8_t columns, uint8_t style);
  void dogm_draw_image_xy(const uint8_t *image, uint8_t x, uint8_t y, uint8_t pages, uint8_t columns, uint8_t style);
#endif

/* Clear (partial) display */
void dogm_clear_area(uint8_t pages, uint8_t columns, uint8_t style);
void dogm_clear_area_xy(uint8_t pages, uint8_t columns, uint8_t style, uint8_t page, uint8_t col);
  

/* Move Cursor */
void dogm_moveto_xy  (uint8_t page, uint8_t column);
void dogm_move_xy    (int8_t pages, int16_t columns);


void Set_Background_PWM( uint8_t pwm_idx );

//Text functions are included in font.c / font.h

/*****************************************************************************
 * LCD Size, based on type selection above
 *****************************************************************************/
#if DISPLAY_TYPE == 160
  #define DOGM_WIDTH            160 //width of the LCD
  #define DOGM_HEIGHT           104 //height of the LCD
  #define DOGM_RAM_PAGES        26  //size of LCD RAM
  #define DOGM_PIXEL_PER_BYTE   4   //using double pixels
  #define DOGM_DOUBLE_PIXEL     1   
  #define SHIFT_ADDR_DOGM_NORMAL     0   //column offset for normal orientation
  #define SHIFT_ADDR_TOPVIEW    0   //column offset for bottom view orientation
#endif
 
 #if DISPLAY_TYPE == 132
  #define DOGM_WIDTH          132 //width of the LCD
  #define DOGM_HEIGHT         32  //height of the LCD
  #define DOGM_RAM_PAGES      4   //size of LCD RAM
  #define DOGM_PIXEL_PER_BYTE 8   //using single pixels
  #define SHIFT_ADDR_DOGM_NORMAL   0   //column offset for normal orientation
  #define SHIFT_ADDR_TOPVIEW  0   //column offset for bottom view orientation
#endif

#if DISPLAY_TYPE == 128
  #define DOGM_WIDTH          128 //width of the LCD
  #define DOGM_HEIGHT         64  //height of the LCD
  #define DOGM_RAM_PAGES      8   //size of LCD RAM
  #define DOGM_PIXEL_PER_BYTE 8   //using single pixels
  #define SHIFT_ADDR_DOGM_NORMAL   0   //column offset for normal orientation
  #define SHIFT_ADDR_TOPVIEW  4   //column offset for bottom view orientation
#endif

#if DISPLAY_TYPE == 102
  #define DOGM_WIDTH          102 //width of the LCD
  #define DOGM_HEIGHT         64  //height of the LCD
  #define DOGM_RAM_PAGES      8   //size of LCD RAM
  #define DOGM_PIXEL_PER_BYTE 8   //using single pixels
  #define SHIFT_ADDR_DOGM_NORMAL   0   //column offset for normal orientation
  #define SHIFT_ADDR_TOPVIEW  30  //column offset for bottom view orientation
#endif

#if ORIENTATION_UPSIDEDOWN == 0
  #define SHIFT_ADDR   SHIFT_ADDR_DOGM_NORMAL
#endif

#if ORIENTATION_UPSIDEDOWN == 1
  #define SHIFT_ADDR   SHIFT_ADDR_TOPVIEW
#endif

/*****************************************************************************
 * Command Codes
 *****************************************************************************/
#if DISPLAY_TYPE == 128 || DISPLAY_TYPE == 132 || DISPLAY_TYPE == 102
  #define DOGM_DISPLAY_ENABLE    0xAE  //1: Display on/off
  #define DOGM_START_LINE        0x40  //2: display start line set 
  #define DOGM_PAGE_ADDRESS      0xB0  //3: Page address set (lower 4 bits select one of the pages)
  #define DOGM_COL_ADDRESS       0x10  //4: column address 
  #define DOGM_BOTTOMVIEW        0xA0  //8: select orientation 
  #define DOGM_DISPLAY_DOGM_INVERT    0xA6  //9: inverted display
  #define DOGM_ALL_PIXEL         0xA4  //10: show memory content or switch all pixels on
  #define DOGM_BIAS              0xA2  //11: lcd bias set
  #define DOGM_RESET_CMD         0xE2  //14: Reset Controller
  #define DOGM_SCAN_DIR          0xC0  //15: output mode select (turns display upside-down)
  #define DOGM_POWER_CONTROL     0x28  //16: power control set 
  #define DOGM_VOLTAGE           0x20  //17: voltage regulator resistor ratio set 
  #define DOGM_VOLUME_MODE       0x81  //18: Volume mode set 
  #define DOGM_NO_OP             0xE3  //22: NOP command
#endif

#if DISPLAY_TYPE == 128 || DISPLAY_TYPE == 132
  #define DOGM_INDICATOR         0xAC  //19: static indicator (2-byte command)
  #define DOGM_BOOSTER_SET       0xF8  //20: booster ratio set
#endif
  
#if DISPLAY_TYPE == 102
  #define DOGM_ADV_PROG_CTRL     0xFA  //25: advanced program control 
  #define DOGM_ADV_PROG_CTRL2    0x10  //25: advanced program control 
#endif

#if DISPLAY_TYPE == 160
  #define DOGM_COL_ADDRESS       0x10  //4: column address 
  #define DOGM_TEMP_COMP         0x24  //5: Set Temperature Compensation
  #define DOGM_PANEL_LOAD        0x28  //6: Set Panel loading
  #define DOGM_PUMP_CTRL         0x2C  //7: Set pump control
  #define DOGM_ADV_PROG_CTRL     0x30  //8: advanced program control first word
  #define DOGM_START_LINE        0x40  //9: display scroll line set LSB
  #define DOGM_START_LINE2       0x50  //9: display scroll line set MSB
  #define DOGM_PAGE_ADDRESS      0x60  //10: Page address set
  #define DOGM_VOLTAGE_BIAS      0x81  //11: Bias set
  #define DOGM_PARTIAL_CTRL      0x84  //12: Set partial display control
  #define DOGM_RAM_ADDR_CTRL     0x88  //13: Set RAM address control
  #define DOGM_FIXED_LINES       0x90  //14: Set display fixed lines
  #define DOGM_LINE_RATE         0xA0  //15: Set line rate
  #define DOGM_ALL_PIXEL         0xA4  //16: show all points
  #define DOGM_INVERSE           0xA6  //17: Inverse display
  #define DOGM_DISPLAY_ENABLE    0xAE  //18: Display enable
  #define DOGM_MAPPING_CTRL      0xC0  //19: LCD mapping control
  #define DOGM_GRAY_SHADE        0xD0  //20: LCD gray shade
  #define DOGM_RESET_CMD         0xE2  //21: System reset
  #define DOGM_NO_OP             0xE3  //22: NOP
  #define DOGM_BIAS_RATIO        0xE8  //24: Bias Ratio
  #define DOGM_CURSOR_MODE_RESET 0xEE  //25: Reset cursor update mode
  #define DOGM_CURSOR_MODE_SET   0xEF  //26: Set cursor update mode
  #define DOGM_COM_END           0xF1  //27: Set COM End
  #define DOGM_PARTIAL_START     0xF2  //28: Set partial display start
  #define DOGM_PARTIAL_END       0xF3  //29: Set partial display end
  #define DOGM_WINDOW_START_COL  0xF4  //30: Window program start column
  #define DOGM_WINDOW_START_PAGE 0xF5  //31: Window program start column
  #define DOGM_WINDOW_END_COL    0xF6  //32: Window program start column
  #define DOGM_WINDOW_END_PAGE   0xF7  //33: Window program start column
  #define DOGM_WINDOW_PROGRAM    0xF8  //34: Enable window programming
#endif 

/*****************************************************************************
 * Makros to execute commands 
 *****************************************************************************/
 
 #if DISPLAY_TYPE == 160
  #define DOGM_SET_TEMP_COMP(i)          dogm_command(DOGM_TEMP_COMP | ((i) & 0x3))
  #define DOGM_SET_PANEL_LOAD(i)         dogm_command(DOGM_PANEL_LOAD | ((i) & 0x3))
  #define DOGM_SET_PUMP_CTRL(i)          dogm_command(DOGM_PUMP_CTRL | ((i) & 0x3))
  #define DOGM_SET_ADV_PROG_CTRL(i)      dogm_command(DOGM_ADV_PROG_CTRL); \
                                         dogm_command((i)&0x3F)  
  #define DOGM_SET_START_LINE(i)         dogm_command(DOGM_START_LINE | ((i)&0xF)); \
                                         dogm_command(DOGM_START_LINE2 | (((i)>>4)&0x7))
  #define DOGM_SET_VOLTAGE_BIAS(i)       dogm_command(DOGM_VOLTAGE_BIAS); \
                                         dogm_command((i)&0xFF)
  #define DOGM_SET_PARTIAL_DISPLAY_CTRL(i) dogm_command(DOGM_PARTIAL_CTRL | ((i) & 0x3))
  #define DOGM_SET_RAM_ADDR_CTRL(i)      dogm_command(DOGM_RAM_ADDR_CTRL | ((i) & 0x7))
  #define DOGM_SET_FIXED_LINES(i)        dogm_command(DOGM_FIXED_LINES | ((i) & 0xF))
  #define DOGM_SET_LINE_RATE(i)          dogm_command(DOGM_LINE_RATE | ((i) & 0x3))
  #define DOGM_SHOW_ALL_PIXELS_ON()      dogm_command(DOGM_ALL_PIXEL | 1)  
  #define DOGM_SHOW_ALL_PIXELS_OFF()     dogm_command(DOGM_ALL_PIXEL | 0)  
  #define DOGM_DOGM_INVERT_DISPLAY(i)         dogm_command(DOGM_INVERSE | ((i)&1))
  #define DOGM_SWITCH_ON()               dogm_command(DOGM_DISPLAY_ENABLE | 1)
  #define DOGM_SWITCH_OFF()              dogm_command(DOGM_DISPLAY_ENABLE | 0)
  #define DOGM_SET_MAPPING_CTRL(i)       dogm_command(DOGM_MAPPING_CTRL | ((i) & 0x7))  
  #define DOGM_SET_BOTTOM_VIEW()         dogm_command(DOGM_MAPPING_CTRL | 0)  
  #define DOGM_SET_TOP_VIEW()            dogm_command(DOGM_MAPPING_CTRL | 6)  
  #define DOGM_SET_GRAY_SHADE(i)         dogm_command(DOGM_GRAY_SHADE | ((i) & 0x3))
  #define DOGM_SET_PAGE_ADDR(i)          dogm_command(DOGM_PAGE_ADDRESS | ((i) & 0x1F))
  #define DOGM_SET_COLUMN_ADDR(col)      dogm_command(DOGM_COL_ADDRESS | ((((col)+SHIFT_ADDR)>>4) & 0x0F)); \
                                         dogm_command((((col)+SHIFT_ADDR) & 0x0F))
  #define DOGM_GOTO_ADDRESS(page,col)    dogm_command(DOGM_PAGE_ADDRESS | ((page) & 0x1F)); \
                                         dogm_command(DOGM_COL_ADDRESS | ((((col)+SHIFT_ADDR)>>4) & 0x0F)); \
                                         dogm_command((((col)+SHIFT_ADDR) & 0x0F))

  #define DOGM_NOP()                     dogm_command(DOGM_NO_OP)
  #define DOGM_SET_BIAS_RATIO(i)         dogm_command(DOGM_BIAS_RATIO | ((i) & 0x3))
  #define DOGM_SET_CURSOR_UPDATE_MODE    dogm_command(DOGM_CURSOR_MODE_SET)
  #define DOGM_RESET_CURSOR_UPDATE_MODE  dogm_command(DOGM_CURSOR_MODE_RESET)
  #define DOGM_SET_COM_END(i)            dogm_command(DOGM_COM_END); \
                                         dogm_command(i)
  #define DOGM_SET_PARTIAL_DISPLAY(start,end) \
                                         dogm_command(DOGM_PARTIAL_START); \
                                         dogm_command((start) & 0x7F) \
                                         dogm_command(DOGM_PARTIAL_END); \
                                         dogm_command((end) & 0x7F)
  #define DOGM_SET_PROGRAM_WINDOW(startpage,startcol,endpage,endcol) \
                                         dogm_command(DOGM_WINDOW_START_PAGE); \
                                         dogm_command(startpage); \
                                         dogm_command(DOGM_WINDOW_START_COL); \
                                         dogm_command(startcol); \
                                         dogm_command(DOGM_WINDOW_END_PAGE); \
                                         dogm_command(endpage); \
                                         dogm_command(DOGM_WINDOW_END_COL); \
                                         dogm_command(endcol)
  #define DOGM_ENABLE_WINDOW_PROGRAM     dogm_command(DOGM_WINDOW_PROGRAM | 1)
  #define DOGM_DISABLE_WINDOW_PROGRAM    dogm_command(DOGM_WINDOW_PROGRAM | 0)

  /* The following two are not tested with DISPLAY_TYPE == 160 */
  #define DOGM_POWERSAVE_ON()			do { DOGM_SWITCH_OFF(); DOGM_SHOW_ALL_PIXELS_ON(); } while(0)
  #define DOGM_POWERSAVE_OFF()			do { DOGM_SHOW_ALL_PIXELS_OFF(); DOGM_SWITCH_ON(); } while(0)

#endif 
 
#if DISPLAY_TYPE == 128 || DISPLAY_TYPE == 132 || DISPLAY_TYPE == 102
  #define DOGM_SWITCH_ON()              dogm_command(DOGM_DISPLAY_ENABLE | 1)
  #define DOGM_SWITCH_OFF()             dogm_command(DOGM_DISPLAY_ENABLE | 0)
  #define DOGM_SET_FIRST_LINE(i)        dogm_command(DOGM_START_LINE | ((i) & 0x3F))
  #define DOGM_SET_PAGE_ADDR(i)         dogm_command(DOGM_PAGE_ADDRESS | ((i) & 0x0F))
  #define DOGM_SET_COLUMN_ADDR(col)     dogm_command(DOGM_COL_ADDRESS | ((((col)+SHIFT_ADDR)>>4) & 0x0F)); \
                                        dogm_command((((col)+SHIFT_ADDR) & 0x0F))
  #define DOGM_GOTO_ADDRESS(page,col)   dogm_command(DOGM_PAGE_ADDRESS | ((page) & 0x1F)); \
                                        dogm_command(DOGM_COL_ADDRESS | ((((col)+SHIFT_ADDR)>>4) & 0x0F)); \
                                        dogm_command((((col)+SHIFT_ADDR) & 0x0F))
  #define DOGM_SET_BOTTOM_VIEW()        dogm_command(DOGM_BOTTOMVIEW | 1)
  #define DOGM_SET_TOP_VIEW()           dogm_command(DOGM_BOTTOMVIEW | 0)
  #define DOGM_SET_MODE_POSITIVE()      dogm_command(DOGM_DISPLAY_DOGM_INVERT | 0)
  #define DOGM_SET_MODE_DOGM_INVERTED()      dogm_command(DOGM_DISPLAY_DOGM_INVERT | 1)
  #define DOGM_SHOW_ALL_PIXELS_ON()     dogm_command(DOGM_ALL_PIXEL | 1)
  #define DOGM_SHOW_ALL_PIXELS_OFF()    dogm_command(DOGM_ALL_PIXEL | 0)
  #define DOGM_SET_BIAS_RATIO_1_7()     dogm_command(DOGM_BIAS | 1)
  #define DOGM_SET_BIAS_RATIO_1_9()     dogm_command(DOGM_BIAS | 0)
  #define DOGM_SEND_RESET()             dogm_command(DOGM_RESET_CMD)
  #define DOGM_ORIENTATION_DOGM_NORMAL()     dogm_command(DOGM_SCAN_DIR | 0x0)
  #define DOGM_ORIENTATION_UPSIDEDOWN() dogm_command(DOGM_SCAN_DIR | 0x8)
  #define DOGM_SET_POWER_CONTROL(i)     dogm_command(DOGM_POWER_CONTROL | ((i) & 0x07))
  #define DOGM_SET_LOW_POWER()          dogm_command(DOGM_POWER_CONTROL | 0x7)
  #define DOGM_SET_WIDE_RANGE()         dogm_command(DOGM_POWER_CONTROL | 0x7)
  #define DOGM_SET_LOW_VOLTAGE()        dogm_command(DOGM_POWER_CONTROL | 0x3)
  #define DOGM_SET_BIAS_VOLTAGE(i)      dogm_command(DOGM_VOLTAGE | ((i) & 0x07))                                   
  #define DOGM_SET_VOLUME_MODE(i)       dogm_command(DOGM_VOLUME_MODE); \
                                        dogm_command(((i) & 0x3F))       
  #define DOGM_NOP()                    dogm_command(DOGM_NO_OP)
#endif

#if DISPLAY_TYPE == 128 || DISPLAY_TYPE == 132
  #define DOGM_SET_INDICATOR_OFF()      dogm_command(DOGM_INDICATOR | 0); \
                                        dogm_command(0x00)
  #define DOGM_SET_INDICATOR_STATIC()   dogm_command(DOGM_INDICATOR | 1); \
                                        dogm_command(0x11)
  #define DOGM_SET_INDICATOR_1HZ()      dogm_command(DOGM_INDICATOR | 1); \
                                        dogm_command(0x01)
  #define DOGM_SET_INDICATOR_2HZ()      dogm_command(DOGM_INDICATOR | 1); \
                                        dogm_command(0x10)
  #define DOGM_SET_INDICATOR(i,j)       dogm_command(DOGM_INDICATOR | ((i) & 1)); \
                                        dogm_command(((j) & 2))
  #define DOGM_SET_BOOSTER_MODE(i)      dogm_command(DOGM_BOOSTER_SET); \
                                        dogm_command(((i) & 0x03))  
  #define DOGM_SET_BOOSTER_MODE_234     dogm_command(DOGM_BOOSTER_SET); \
                                        dogm_command(0x0)  
  #define DOGM_SET_BOOSTER_MODE_5       dogm_command(DOGM_BOOSTER_SET); \
                                        dogm_command(0x1)  
  #define DOGM_SET_BOOSTER_MODE_6       dogm_command(DOGM_BOOSTER_SET); \
                                        dogm_command(0x3)  
  /* The follwoing power save commands assume, that no indicator is used */
  #define DOGM_POWERSAVE_ON()	        do { DOGM_SWITCH_OFF(); DOGM_SHOW_ALL_PIXELS_ON(); } while(0)
  #define DOGM_POWERSAVE_OFF()          do { DOGM_SHOW_ALL_PIXELS_OFF(); DOGM_SWITCH_ON(); } while(0)
#endif


#if DISPLAY_TYPE == 102
  #define DOGM_TEMPCOMP_HIGH  0x80
  #define DOGM_COLWRAP        0x02
  #define DOGM_PAGEWRAP       0x01
  #define DOGM_SET_ADV_PROG_CTRL(i)     dogm_command(DOGM_ADV_PROG_CTRL); \
                                       dogm_command(DOGM_ADV_PROG_CTRL2 & i)
#endif

//Bit positions for style settings
#define DOGM_NORMAL      0
#define DOGM_INVERT      4
#define DOGM_INVERT_BIT  4

#endif
