/******************************************************************************
 * Display Library
 * for EA-DOGS102  GLCD (102px x 64px)
 *     EA-DOGM128  GLCD (128px x 64px)
 *     EA-DOGM132  GLCD (132px x 32px)
 *     EA-DOGL128  GLCD (128px x 64px)
 *     EA-DOGXL160 GLCD (160px x 104px)
 * 
 * Provides all basic functions to access the display
 * Since no graphics ram is used, the memory footprint is rather small but
 * also does not allow to change single pixels. Data written to the LCD can
 * not be read back!
 * Text can be displayed using the attached font generator containing several
 * character sets and font variants.
 * Thanks to Oliver Schwaneberg for adding several functions to this library!
 * 
 * Author:  Jan Michel (jan at mueschelsoft dot de)
 * License: GNU General Public License, version 3
 * Version: v0.95a November 2012
 * ****************************************************************************
 * New features in v0.95
 *   - added initialization for top-view 
 *   - automatic shifting column addresses (e.g. top-view)
 * New features in v0.94
 *   - corrected order of arguments in dogm_clear_area_xy()
 *   - added command definitions for DOGXL160
 *   - cleaned up dogm_graphic.h command area
 * New features in v0.93
 *   - modified initialization for DOGM128 and DOGS102
 * New features in v0.92
 *   - Added initialization for DOGS102 and DOGL128
 * New features in v0.91
 *   - Control of chip select pin
 *   - backlight & chip select are optional - can be turned of in header file
 *   - added function dogm_draw_image_P 
 *   - added function dogm_draw_image_xy_P
 *   - added function dogm_clear_area
 *   - added SPI initialization
 * Changes by Rainer 29.10.19
 *   - port to STM32: All _P routines removed
 *   - SPI access via Spi-Device
 *   - changed prefix LCD_ to DOGM_
 *****************************************************************************/

#include "config/config.h"

#if USE_DOGM132 > 0

#include "debug_helper.h"
#include "global_flags.h"
#include "disp/ssdxxxx_spi.h"
#include "disp/dogm-graphic.h"
#include "dev/spi.h"





/*****************************************************************************
 * Output pin controlling makros
 *****************************************************************************/

//Control reset input of LCD
#if DOGM_USE_RESET == 1
	#define DOGM_RESET()           SSD_DevReset()
#else
	#define DOGM_RESET() 
#endif

//=============================================================================
//keeping track of current position in ram - necessary for big fonts & bitmaps
//=============================================================================

uint8_t dogm_current_page = 0;
uint8_t dogm_current_column = 0;

/******************************************************************************
 * Changes the internal cursor by s pages
 * s             - number of pages to move
 */ 
uint8_t dogm_inc_page(int8_t s) {
  uint8_t p = dogm_current_page;
  p += s;
  p %= DOGM_RAM_PAGES;    //all lcd have dogm_ram_pages which is power of two
  dogm_current_page = p;
  return p;
  }

/******************************************************************************
 * Changes the internal cursor by s columns, including wrapping (if selected)
 * s             - number of columns to move
 */ 
uint8_t dogm_inc_column(int16_t s) {
  uint16_t c = dogm_current_column;
  c += s;
#if DOGM_WRAP_AROUND == 1
  while (c >= DOGM_WIDTH) {
    if (s > 0) dogm_inc_page(1);
    else       dogm_inc_page(-1);
    if (s > 0) c -= DOGM_WIDTH;
    else       c += DOGM_WIDTH;
    }
#endif
  dogm_current_column = c;
  return c;
  }
  
  
/******************************************************************************
 * Moves the cursor to the given position
 * pages         - page to move to
 * columns       - column to move to
 */ 
void dogm_moveto_xy(uint8_t page, uint8_t column) {
  DOGM_GOTO_ADDRESS(page,column);
  dogm_current_column = column; 
  dogm_current_page = page;
  }

/******************************************************************************
 * Moves the cursor relative to the current position
 * pages         - number of pages to move
 * columns       - number of columns to move
 */  
void dogm_move_xy(int8_t pages, int16_t columns) {
  dogm_moveto_xy(dogm_inc_page(pages),dogm_inc_column(columns));
  }


//=============================================================================
//Basic Byte Access to Display
//=============================================================================

/******************************************************************************
 * Writes one data byte
 * data          - the data byte
 */
void dogm_data(uint8_t data) {
  SSD_WriteD(data);
  dogm_inc_column(1);
}

/******************************************************************************
 * Writes one command byte
 * cmd           - the command byte
 */
void dogm_command(uint8_t cmd) {
  SSD_WriteC(cmd);
}
  

//=============================================================================
//Puts raw data from Flash to the Display
//=============================================================================
#if DOGM_INCLUDE_GRAPHIC_FUNCTIONS >= 1
/******************************************************************************
 * This function draws a bitmap from the current position on the screen.
 * Parameters:
 * progmem_image - prog_uint8_t array of columns aka the bitmap image
 * pages         - height of image in pages
 * columns       - width of image in pixels (or columns)
 * style         - Bit2: sets inverse mode
 */  
void dogm_draw_image(const uint8_t *image, uint8_t pages, uint8_t columns, uint8_t style) 
{
    uint8_t i,j = 0;
    uint8_t inv = (style & DOGM_INVERT_BIT);
    while(j<pages && (dogm_get_position_page() < DOGM_RAM_PAGES)) {
        for (i=0; i<columns && (dogm_get_position_column() < DOGM_WIDTH); i++) {
            uint8_t tmp = *(image++);
            if(inv) tmp = ~tmp;
            dogm_data(tmp);
        }
        if(++j != pages && dogm_get_position_column() != 0) dogm_move_xy(1,-columns);
    }
}

  
/******************************************************************************
 * This function draws a bitmap at any xy-position on the screen. 
 * Be aware that some pixels are deleted due to memory organization!
 * Parameters:
 * progmem_image - prog_uint8_t array of columns aka the bitmap image
 * x             - x start coordinate on the screen (in pixel)
 * y             - y start coordinate on the screen (in pixel)
 * pages         - height of image in pages
 * columns       - width of image in pixels
 * style         - Bit2: sets inverse mode
 */
void dogm_draw_image_xy(const uint8_t *image, uint8_t x, uint8_t y, uint8_t pages, uint8_t columns, uint8_t style) 
{
    uint16_t i,j;
    uint8_t data   = 0;
    uint8_t inv    = style & DOGM_INVERT_BIT;
    uint8_t offset = y & 0x7; //Optimized modulo 8
	
    //If there is an offset, we must use an additional page
    if(offset) pages++;

    //If there is not enough vertical space -> cut image
    if(pages > DOGM_RAM_PAGES - dogm_get_position_page()) 
        pages = DOGM_RAM_PAGES - dogm_get_position_page();
  
    //Goto starting point and draw
    dogm_moveto_xy((y>>3), x);
    for (j=0; j<pages; j++) {
	for (i=0; i<columns && (dogm_get_position_column() < DOGM_WIDTH); i++){
            data = 0;
            if (!offset || j+1 != pages)
                data = *(image+j*columns + i) << offset;
            if(j > 0 && offset)
                data |= *(image+(j-1)*columns + i) >> (8-offset);
            if(inv)	data = ~data;
            dogm_data(data);
	}
        if(j+1 != pages) dogm_move_xy(1,-columns);
    }
}
#endif


/******************************************************************************
 * This function clears an area of the screen
 * pages         - height of area in pages
 * columns       - width of area in pixels
 * style         - Bit2: sets inverse mode
 * Cursor is moved to start of area after clear
 */
void dogm_clear_area(uint8_t pages, uint8_t columns, uint8_t style) {
  uint8_t i,j,max;
  uint8_t inv = (style & DOGM_INVERT_BIT)?0xFF:0;
  
  if(pages > (max = DOGM_RAM_PAGES - dogm_get_position_page()))   
    pages = max;
  if(columns > (max = DOGM_WIDTH - dogm_get_position_column()))   
    columns = max;
  
  for(j=0; j<pages; j++) {
    for(i=0; i<columns; i++) {
      dogm_data(inv);
      }
    dogm_move_xy((dogm_get_position_column()?1:0),-columns);
    }
  dogm_move_xy(-pages,0);
  }

/******************************************************************************
 * This function clears an area of the screen starting at the given coordinates
 * pages         - height of area in pages
 * columns       - width of area in pixels
 * style         - style modifier
 * col           - column of upper left corner
 * page          - page of upper left corner
 * Cursor is moved to start of area after clear
 */
void dogm_clear_area_xy(uint8_t pages, uint8_t columns, uint8_t style, uint8_t page, uint8_t col) {
  dogm_moveto_xy(page,col);
  dogm_clear_area(pages,columns,style);
  }
 
/******************************************************************************
 * Clear the whole display
 ******************************************************************************/
void dogm_clear_display( uint8_t bckgnd )
{
   dogm_clear_area_xy(DOGM_RAM_PAGES, DOGM_WIDTH ,bckgnd ,0 ,0); 
}

/******************************************************************************
 * Initializes the display in 4x booster for 2.4-3.3V supply voltage
 * scheme according to datasheet
 * Suitable for all DOGS, DOGM, DOGL and DOGXL displays 
 * in both bottom or top-view orientation.
 ******************************************************************************/
void dogm_init(const HW_DeviceType *dev, bool bRotate)
{    
  /* Assign SPI device to driver layer */
  SSD_DevInit ( dev );  
  #if DOGM_USE_RESET	== 1
	  DOGM_RESET();                  //Apply Reset to the Display Controller
  #endif
  //Load settings
  #if DISPLAY_TYPE == 160
    DOGM_SET_COM_END(103);               //set last COM electrode
    if ( bRotate ) {
      DOGM_SET_BOTTOM_VIEW();            //6 o'clock mode, normal orientation
    } else {
      DOGM_SET_TOP_VIEW();               //12 o'clock mode, reversed orientation
    }
    DOGM_SET_START_LINE(0);              //set scrolling to 0
    DOGM_SET_PANEL_LOAD(3);              //set panel loading to 28-38nF
    DOGM_SET_BIAS_RATIO(3);              //set bias ratio
    DOGM_SET_VOLTAGE_BIAS(0x5F);         //set Vbias potentiometer for contrast
    DOGM_SET_RAM_ADDR_CTRL(1);           //set auto-increment
  #endif
  #if DISPLAY_TYPE == 132
    DOGM_SET_FIRST_LINE(0);              //first bit in RAM is on the first line of the LCD
    if ( bRotate ) {
      DOGM_SET_BOTTOM_VIEW();            //6 o'clock mode, normal orientation
      DOGM_ORIENTATION_DOGM_NORMAL();
    } else {
      DOGM_SET_TOP_VIEW();               //12 o'clock mode, reversed orientation
      DOGM_ORIENTATION_UPSIDEDOWN();
    }
    DOGM_SHOW_ALL_PIXELS_OFF();          //Normal Pixel mode
    DOGM_SET_MODE_POSITIVE();            //positive display
    DOGM_SET_BIAS_RATIO_1_9();           //bias 1/9
    DOGM_SET_POWER_CONTROL(7);           //power control mode: all features on
    DOGM_SET_BIAS_VOLTAGE(3);            //set voltage regulator R/R
    DOGM_SET_VOLUME_MODE(0x1F);          //volume mode set
    // DOGM_SET_INDICATOR_OFF();            //switch indicator off, no blinking
    DOGM_SET_INDICATOR_2HZ();
  #endif

  #if DISPLAY_TYPE == 128
    DOGM_SET_FIRST_LINE(0);              //first bit in RAM is on the first line of the LCD
    if ( bRotate ) {
      DOGM_SET_BOTTOM_VIEW();            //6 o'clock mode, normal orientation
      DOGM_ORIENTATION_DOGM_NORMAL();
    } else {
      DOGM_SET_TOP_VIEW();               //12 o'clock mode, reversed orientation
      DOGM_ORIENTATION_UPSIDEDOWN();
    }
    DOGM_SHOW_ALL_PIXELS_OFF();          //Normal Pixel mode
    DOGM_SET_MODE_POSITIVE();            //positive display
    DOGM_SET_BIAS_RATIO_1_7();           //bias 1/7
    DOGM_SET_POWER_CONTROL(7);           //power control mode: all features on
    DOGM_SET_BIAS_VOLTAGE(7);            //set voltage regulator R/R
    DOGM_SET_VOLUME_MODE(0x06);          //volume mode set
    DOGM_SET_INDICATOR_OFF();            //switch indicator off, no blinking
  #endif
  #if DISPLAY_TYPE == 102
    DOGM_SET_FIRST_LINE(0);              //first bit in RAM is on the first line of the LCD
    if ( bRotate ) {
      DOGM_SET_BOTTOM_VIEW();            //6 o'clock mode, normal orientation
      DOGM_ORIENTATION_DOGM_NORMAL();
    } else {
      DOGM_SET_TOP_VIEW();               //12 o'clock mode, reversed orientation
      DOGM_ORIENTATION_UPSIDEDOWN();
    }
    DOGM_SHOW_ALL_PIXELS_OFF();          //Normal Pixel mode
    DOGM_SET_MODE_POSITIVE();            //positive display
    DOGM_SET_BIAS_RATIO_1_9();           //bias 1/9
    DOGM_SET_POWER_CONTROL(7);           //power control mode: all features on
    DOGM_SET_BIAS_VOLTAGE(7);            //set voltage regulator R/R
    DOGM_SET_VOLUME_MODE(0x9);           //volume mode set
    DOGM_SET_ADV_PROG_CTRL(DOGM_TEMPCOMP_HIGH);
  #endif

  /* clear whole display */
  dogm_clear_display(DOGM_NORMAL); 

  DOGM_SWITCH_ON();                    //Switch display on
  return;
}


#endif	// if USE_DOGM132 > 0


