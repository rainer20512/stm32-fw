/**
 * @file lv_gc9a01.c
 *
 * Low level driver for GCA901 TFT driver chip
 *
 */


/*********************
 *      INCLUDES
 *********************/
#include "lv_gc9a01.h"
#include "lv_4wire_spi.h"
#include "debug_outbuf.h"

#if LV_USE_GC9A01

/*********************
 *      DEFINES
 *********************/

#define MY_DISP_HOR_RES         240
#define MY_DISP_VER_RES         240
#define BYTE_PER_PIXEL          2

/* GC9A01 Commands that we know of.  Limited documentation */
#define GC9A01_INVOFF		0x20
#define GC9A01_INVON		0x21
#define GC9A01_DISPON		0x29
#define GC9A01_CASET		0x2A
#define GC9A01_RASET		0x2B
#define GC9A01_RAMWR		0x2C
#define GC9A01_COLMOD		0x3A
#define GC9A01_MADCTL		0x36
#define GC9A01_MADCTL_MY  	0x80
#define GC9A01_MADCTL_MX  	0x40
#define GC9A01_MADCTL_MV  	0x20
#define GC9A01_MADCTL_RGB 	0x00
#define GC9A01_DISFNCTRL	0xB6

#define GC9A01_CMD_MODE		0
#define GC9A01_DATA_MODE     	1


#define GC9A01_DEBUG            0

/**********************
 *      TYPEDEFS
 **********************/

/* Init script function */
struct GC9A01_function {
	uint16_t cmd;
	uint16_t data;
};

/* Init script commands */
enum GC9A01_cmd {
	GC9A01_START,
	GC9A01_END,
	GC9A01_CMD,
	GC9A01_DATA,
	GC9A01_DELAY
};

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC CONSTANTS
 **********************/

/* init commands for 1.28" round display */
static const uint8_t init_cmd_list[] = {
    0xEF,       0,
    0xEB,       1,  0x14,
    0xFE,       0,          // Inter Register Enable1
    0xEF,       0,          // Inter Register Enable2
    0xEB,       1,  0x14,
    0x84,       1,  0x40,
    0x85,       1,  0xFF,
    0x86,       1,  0xFF,
    0x87,       1,  0xFF,
    0x88,       1,  0x0A,
    0x89,       1,  0x21,
    0x8A,       1,  0x00,
    0x8B,       1,  0x80,
    0x8C,       1,  0x01,
    0x8D,       1,  0x01,
    0x8E,       1,  0xFF,
    0x8F,       1,  0xFF,
    GC9A01_DISFNCTRL,      // Display Function Control
                2,  0x00, 0x00,
    GC9A01_MADCTL,        // Memory Access Control
                1,  0x48, // Set the display direction 0,1,2,3	four directions
    GC9A01_COLMOD,        // COLMOD: Pixel Format Set
                1,  0x05, // 16 Bits per pixel
    0x90,       4,  0x08, 0x08, 0x08, 0x08,
    0xBD,       1,  0x06,
    0xBC,       1,  0x00,
    0xFF,       3,  0x60, 0x01, 0x04,
    0xC3,               // Power Control 2
                1,  0x13,
    0xC4,               // Power Control 3
                1,  0x13,
    0xC9,               // Power Control 4
                1,  0x22,
    0xBE,       1 , 0x11,
    0xE1,       2,  0x10, 0x0E,
    0xDF,       3,  0x21, 0x0C, 0x02,
    0xF0,               // SET_GAMMA1
                6,  0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF1,               // SET_GAMMA2
                6,  0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xF2,               // SET_GAMMA3
                6,  0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF3,               // SET_GAMMA4
                6,  0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xED,       2,  0x1B, 0x0B,
    0xAE,       1,  0x77,
    0xCD,       1,  0x63,
    0x70,       9,  0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03,
    0xE8,       1,  0x34,
    0x62,       12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70,
                    0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63,       12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70,
                    0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64,       7,  0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07,
    0x66,       10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45,
                    0x10, 0x00, 0x00, 0x00,
    0x67,       10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01,
                    0x54, 0x10, 0x32, 0x98,
    0x74,       7,  0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
    0x98,       2,  0x3E, 0x07,
    0x35,       0,      // Tearing Effect Line ON
    0x21,       0,      // Display Inversion ON
    0x11,       0,      // Sleep Out Mode
    LV_LCD_CMD_DELAY_MS, 12,        // Delay in multiples of 10(!) ms
    GC9A01_DISPON, 0,   // Display ON
    LV_LCD_CMD_DELAY_MS, 26,
    LV_LCD_CMD_DELAY_MS, LV_LCD_CMD_EOF
};
/**********************
 *  STATIC VARIABLES
 **********************/
// Documentation on op codes for GC9A01 are very hard to find.
// Will document should they be found.
static struct GC9A01_function GC9A01_cfg_script[] = {
	{ GC9A01_START, GC9A01_START},
	{ GC9A01_CMD, 0xEF},

	{ GC9A01_CMD, 0xEB},
	{ GC9A01_DATA, 0x14},

	{ GC9A01_CMD, 0xFE}, // Inter Register Enable1
	{ GC9A01_CMD, 0xEF}, // Inter Register Enable2

	{ GC9A01_CMD, 0xEB},
	{ GC9A01_DATA, 0x14},

	{ GC9A01_CMD, 0x84},
	{ GC9A01_DATA, 0x40},

	{ GC9A01_CMD, 0x85},
	{ GC9A01_DATA, 0xFF},

	{ GC9A01_CMD, 0x86},
	{ GC9A01_DATA, 0xFF},

	{ GC9A01_CMD, 0x87},
	{ GC9A01_DATA, 0xFF},

	{ GC9A01_CMD, 0x88},
	{ GC9A01_DATA, 0x0A},

	{ GC9A01_CMD, 0x89},
	{ GC9A01_DATA, 0x21},

	{ GC9A01_CMD, 0x8A},
	{ GC9A01_DATA, 0x00},

	{ GC9A01_CMD, 0x8B},
	{ GC9A01_DATA, 0x80},

	{ GC9A01_CMD, 0x8C},
	{ GC9A01_DATA, 0x01},

	{ GC9A01_CMD, 0x8D},
	{ GC9A01_DATA, 0x01},

	{ GC9A01_CMD, 0x8E},
	{ GC9A01_DATA, 0xFF},

	{ GC9A01_CMD, 0x8F},
	{ GC9A01_DATA, 0xFF},

	{ GC9A01_CMD, GC9A01_DISFNCTRL}, // Display Function Control
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},

	{ GC9A01_CMD, GC9A01_MADCTL}, // Memory Access Control
	{ GC9A01_DATA, 0x48}, // Set the display direction 0,1,2,3	four directions

	{ GC9A01_CMD, GC9A01_COLMOD}, // COLMOD: Pixel Format Set
	{ GC9A01_DATA, 0x05}, // 16 Bits per pixel

	{ GC9A01_CMD, 0x90},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x08},

	{ GC9A01_CMD, 0xBD},
	{ GC9A01_DATA, 0x06},

	{ GC9A01_CMD, 0xBC},
	{ GC9A01_DATA, 0x00},

	{ GC9A01_CMD, 0xFF},
	{ GC9A01_DATA, 0x60},
	{ GC9A01_DATA, 0x01},
	{ GC9A01_DATA, 0x04},

	{ GC9A01_CMD, 0xC3}, // Power Control 2
	{ GC9A01_DATA, 0x13},
	{ GC9A01_CMD, 0xC4}, // Power Control 3
	{ GC9A01_DATA, 0x13},

	{ GC9A01_CMD, 0xC9}, // Power Control 4
	{ GC9A01_DATA, 0x22},

	{ GC9A01_CMD, 0xBE},
	{ GC9A01_DATA, 0x11},

	{ GC9A01_CMD, 0xE1},
	{ GC9A01_DATA, 0x10},
	{ GC9A01_DATA, 0x0E},

	{ GC9A01_CMD, 0xDF},
	{ GC9A01_DATA, 0x21},
	{ GC9A01_DATA, 0x0C},
	{ GC9A01_DATA, 0x02},

	{ GC9A01_CMD, 0xF0}, // SET_GAMMA1
	{ GC9A01_DATA, 0x45},
	{ GC9A01_DATA, 0x09},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x26},
	{ GC9A01_DATA, 0x2A},

	{ GC9A01_CMD, 0xF1}, // SET_GAMMA2
	{ GC9A01_DATA, 0x43},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x72},
	{ GC9A01_DATA, 0x36},
	{ GC9A01_DATA, 0x37},
	{ GC9A01_DATA, 0x6F},

	{ GC9A01_CMD, 0xF2}, // SET_GAMMA3
	{ GC9A01_DATA, 0x45},
	{ GC9A01_DATA, 0x09},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x26},
	{ GC9A01_DATA, 0x2A},

	{ GC9A01_CMD, 0xF3}, // SET_GAMMA4
	{ GC9A01_DATA, 0x43},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x72},
	{ GC9A01_DATA, 0x36},
	{ GC9A01_DATA, 0x37},
	{ GC9A01_DATA, 0x6F},

	{ GC9A01_CMD, 0xED},
	{ GC9A01_DATA, 0x1B},
	{ GC9A01_DATA, 0x0B},

	{ GC9A01_CMD, 0xAE},
	{ GC9A01_DATA, 0x77},

	{ GC9A01_CMD, 0xCD},
	{ GC9A01_DATA, 0x63},

	{ GC9A01_CMD, 0x70},
	{ GC9A01_DATA, 0x07},
	{ GC9A01_DATA, 0x07},
	{ GC9A01_DATA, 0x04},
	{ GC9A01_DATA, 0x0E},
	{ GC9A01_DATA, 0x0F},
	{ GC9A01_DATA, 0x09},
	{ GC9A01_DATA, 0x07},
	{ GC9A01_DATA, 0x08},
	{ GC9A01_DATA, 0x03},

	{ GC9A01_CMD, 0xE8},
	{ GC9A01_DATA, 0x34},

	{ GC9A01_CMD, 0x62},
	{ GC9A01_DATA, 0x18},
	{ GC9A01_DATA, 0x0D},
	{ GC9A01_DATA, 0x71},
	{ GC9A01_DATA, 0xED},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x18},
	{ GC9A01_DATA, 0x0F},
	{ GC9A01_DATA, 0x71},
	{ GC9A01_DATA, 0xEF},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x70},

	{ GC9A01_CMD, 0x63},
	{ GC9A01_DATA, 0x18},
	{ GC9A01_DATA, 0x11},
	{ GC9A01_DATA, 0x71},
	{ GC9A01_DATA, 0xF1},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x18},
	{ GC9A01_DATA, 0x13},
	{ GC9A01_DATA, 0x71},
	{ GC9A01_DATA, 0xF3},
	{ GC9A01_DATA, 0x70},
	{ GC9A01_DATA, 0x70},

	{ GC9A01_CMD, 0x64},
	{ GC9A01_DATA, 0x28},
	{ GC9A01_DATA, 0x29},
	{ GC9A01_DATA, 0xF1},
	{ GC9A01_DATA, 0x01},
	{ GC9A01_DATA, 0xF1},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x07},

	{ GC9A01_CMD, 0x66},
	{ GC9A01_DATA, 0x3C},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0xCD},
	{ GC9A01_DATA, 0x67},
	{ GC9A01_DATA, 0x45},
	{ GC9A01_DATA, 0x45},
	{ GC9A01_DATA, 0x10},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},

	{ GC9A01_CMD, 0x67},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x3C},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x01},
	{ GC9A01_DATA, 0x54},
	{ GC9A01_DATA, 0x10},
	{ GC9A01_DATA, 0x32},
	{ GC9A01_DATA, 0x98},

	{ GC9A01_CMD, 0x74},
	{ GC9A01_DATA, 0x10},
	{ GC9A01_DATA, 0x85},
	{ GC9A01_DATA, 0x80},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x00},
	{ GC9A01_DATA, 0x4E},
	{ GC9A01_DATA, 0x00},

	{ GC9A01_CMD, 0x98},
	{ GC9A01_DATA, 0x3E},
	{ GC9A01_DATA, 0x07},

	{ GC9A01_CMD, 0x35}, // Tearing Effect Line ON
	{ GC9A01_CMD, 0x21}, // Display Inversion ON

	{ GC9A01_CMD, 0x11}, // Sleep Out Mode
	{ GC9A01_DELAY, 120},
	{ GC9A01_CMD, GC9A01_DISPON}, // Display ON
	{ GC9A01_DELAY, 255},
	{ GC9A01_END, GC9A01_END},
};


/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *      Forward decls
 **********************/
static void GC9A01_hard_reset( void );
static void GC9A01_run_cfg_script(void);
static void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map);


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
#define GRMEM __attribute((section(".bss2")))
#define NUMROWS     60          // Number of rows in partial buffer
lv_display_t * lv_gc9a01_create(uint32_t hor_res, uint32_t ver_res, lv_lcd_flag_t flags,
                                lv_gc9a01_send_cmd_cb_t send_cmd_cb, lv_gc9a01_send_color_cb_t send_color_cb)
{
    GC9A01_hard_reset();
    GC9A01_run_cfg_script();

    lv_display_t * disp = lv_lcd_generic_mipi_create(hor_res, ver_res, flags, send_cmd_cb, send_color_cb);
    // lv_lcd_generic_mipi_send_cmd_list(disp, init_cmd_list);

     /* One buffer for partial rendering*/
    LV_ATTRIBUTE_MEM_ALIGN
    static GRMEM uint8_t buf_1_1[MY_DISP_HOR_RES * NUMROWS * BYTE_PER_PIXEL];            /*A buffer for 10 rows*/
    lv_display_set_buffers(disp, buf_1_1, NULL, sizeof(buf_1_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    uint16_t temp = (uint16_t)BASTMR_GetMicrosecond(&BASTIM_HANDLE);
    GC9A01_fillScreen(GC9A01_Color565(temp, temp>>4, temp>>8)); // ?
    GC9A01_fillRect( 60, 60, 80, 80, GC9A01_Color565(0xFF,0x00,0x00));
    return disp;
}

void lv_gc9a01_set_gap(lv_display_t * disp, uint16_t x, uint16_t y)
{
    lv_lcd_generic_mipi_set_gap(disp, x, y);
}

void lv_gc9a01_set_invert(lv_display_t * disp, bool invert)
{
    lv_lcd_generic_mipi_set_invert(disp, invert);
}

void lv_gc9a01_set_gamma_curve(lv_display_t * disp, uint8_t gamma)
{
    lv_lcd_generic_mipi_set_gamma_curve(disp, gamma);
}

void lv_gc9a01_send_cmd_list(lv_display_t * disp, const uint8_t * cmd_list)
{
    lv_lcd_generic_mipi_send_cmd_list(disp, cmd_list);
}

/******************************************************************************
*******************************************************************************
* 4W-SPI-Driver for GC9A01
*******************************************************************************

******************************************************************************/
// hard reset of the tft controller
// ----------------------------------------------------------
static void GC9A01_hard_reset( void )
{

    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(0);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);
}

/**
 * Write a command to the GC9A01
 * @param cmd the command
 */
static void GC9A01_command(uint8_t cmd)
{
	LV_DRV_DISP_CMD_DATA(GC9A01_CMD_MODE);
	LV_DRV_DISP_SPI_WR_BYTE(cmd);
}

/**
 * Write data to the GC9A01
 * @param data the data
 */
static void GC9A01_data(uint8_t data)
{
	LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);
	LV_DRV_DISP_SPI_WR_BYTE(data);
}

static int GC9A01_data_array(uint8_t *buf, uint32_t len)
{
	uint8_t *pt = buf;

	for (uint32_t lp = 0; lp < len; lp++, pt++)
	{
		LV_DRV_DISP_SPI_WR_BYTE(*pt);
	}
	return 0;
}

static int GC9A01_databuf(uint32_t len, uint8_t *buf)
{
	uint32_t byte_left = len;
	uint8_t *pt = buf;

	while (byte_left)
	{
		if (byte_left > 64)
		{
			LV_DRV_DISP_SPI_WR_ARRAY((char*)pt, 64);
			byte_left = byte_left - 64;
			pt = pt + 64;
		}
		else
		{
			LV_DRV_DISP_SPI_WR_ARRAY((char*)pt, byte_left);
			byte_left=0;
		}
	}

	return 0;
}

// Configuration of the tft controller
// ----------------------------------------------------------
static void GC9A01_run_cfg_script(void)
{
	int i = 0;
	int end_script = 0;

        LV_DRV_DISP_SPI_CS(0); // Low to listen to us

	do {
		switch (GC9A01_cfg_script[i].cmd)
		{
			case GC9A01_START:
				break;
			case GC9A01_CMD:
				GC9A01_command( GC9A01_cfg_script[i].data & 0xFF );
				break;
			case GC9A01_DATA:
				GC9A01_data( GC9A01_cfg_script[i].data & 0xFF );
				break;
			case GC9A01_DELAY:
                                LV_DRV_DELAY_MS(GC9A01_cfg_script[i].data);
				break;
			case GC9A01_END:
				end_script = 1;
		}
		i++;
	} while (!end_script);

        LV_DRV_DISP_SPI_CS(1); // Deselect
}


/******************************************************************************
* lowlevel routine for writing n command and m data bytes, n,m >= 0
******************************************************************************/
void gc9a01_send_cmd(lv_display_t * disp, const uint8_t * cmd, size_t cmd_size, 
                       const uint8_t * param, size_t param_size)
{
    const uint8_t *p;
    size_t i;
    LV_UNUSED(disp);

   LV_DRV_DISP_SPI_CS(0);  // Listen to us

   if ( cmd_size > 0 ) {
	LV_DRV_DISP_CMD_DATA(GC9A01_CMD_MODE);
        for ( i = 0, p=cmd; i < cmd_size; p++,i++ ) {
            LV_DRV_DISP_SPI_WR_BYTE(*p);
            #if GC9A01_DEBUG > 0
                DEBUG_PRINTF("C 0x%02x ", *p);
            #endif
        }
	LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);
    }
    if ( param_size > 0 ) {
        #if GC9A01_DEBUG > 0
            DEBUG_PRINTF("%d data bytes starting with 0x%02x", param_size, *param);
        #endif
        for ( i = 0, p=param; i < param_size; p++,i++ ) {
            LV_DRV_DISP_SPI_WR_BYTE(*p);
        }
    }

    LV_DRV_DISP_SPI_CS(1);

    #if GC9A01_DEBUG > 0
        DEBUG_PRINTF("\n");
    #endif

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_display_flush_ready(disp);
}

void gc9a01_send_mass_data(lv_display_t * disp, const uint8_t * cmd, size_t cmd_size, 
                       const uint8_t * param, size_t param_size)
{
    LV_UNUSED(disp);
    uint32_t byte_left;
    uint8_t *pt;
    size_t i;

    LV_DRV_DISP_SPI_CS(0);  // Listen to us

   if ( cmd_size > 0 ) {
	LV_DRV_DISP_CMD_DATA(GC9A01_CMD_MODE);
        for ( i = 0, pt=cmd; i < cmd_size; pt++,i++ ) {
            LV_DRV_DISP_SPI_WR_BYTE(*pt);
            #if GC9A01_DEBUG > 0
                DEBUG_PRINTF("C 0x%02x ", *pt);
            #endif
        }
	LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);
    }
    if ( param_size > 0 ) {
        #if GC9A01_DEBUG > 0
            DEBUG_PRINTF("Blockwise: %d data bytes starting with 0x%02x", param_size, *param);
        #endif
        byte_left = param_size;
        pt = param;
        while (byte_left)
        {
                if (byte_left > 64)
                {
                        LV_DRV_DISP_SPI_WR_ARRAY(pt, 64);
                        byte_left = byte_left - 64;
                        pt = pt + 64;
                }
                else
                {
                        LV_DRV_DISP_SPI_WR_ARRAY(pt, byte_left);
                        byte_left=0;
                }
        }
    }

    LV_DRV_DISP_SPI_CS(1);

    #if GC9A01_DEBUG > 0
        DEBUG_PRINTF("\n");
    #endif

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_display_flush_ready(disp);
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map)
{
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

        int32_t x;
        int32_t y;
        for(y = area->y1; y <= area->y2; y++) {
            for(x = area->x1; x <= area->x2; x++) {
                /*Put a pixel to the display. For example:*/
                /*put_px(x, y, *px_map)*/
                px_map++;
            }
        }
    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_display_flush_ready(disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif  /*LV_USE_GC9A01*/
