/******************************************************************************
 * Display the Status
 *****************************************************************************/

#include "config/config.h"

/* -------------------- Forward Declarations --------------------------------*/
uint8_t LCD_ComputeSigned(char *writebuf, int16_t val, uint8_t bAppendBlank);
uint8_t LCD_ComputeCounter(char *writebuf, uint16_t val, uint8_t bWithDP, uint8_t bAppendBlank);


#if USE_DISPLAY > 0

#include "dev/adc_dev.h"
#include "rtc.h"
#include "eeprom.h"
#include "ds18xxx20.h"
#include "system/status.h"

#include "ui/lcd_interface.h"
#include "ui/lcd_status.h"
#include "ui/lcd_text.h"
#include "ui/lcd_selector.h"
#include "debug_helper.h"

#include "disp/fonts/lcd_fonts.h"

#if USE_DUMPER > 0
    #include "Dumper.h"
#endif

#if USE_SERIALSTORAGE > 0
    #include "SerialStorage.h"
#endif
#if USE_LSM303D > 0
    #include "lsm303d.h"
#endif

#if USE_THPSENSOR > 0
    #include "sensors/thp_sensor.h"
#endif


static int16_t last_temp=0xffff;

extern int16_t mintemp, maxtemp;

void LCD_ClickCallback(void);
void LCD_DblClickCallback(void);

/******************************************************************************
 * Display a signed 16 bit number with one digit behind the dp
 * if bAppendBlank != 0, one blank is appended
 * returns the length of the returned string [excluding \0]
 *****************************************************************************/
static uint8_t LCD_ComputeTemp(char *writebuf, int16_t val, uint8_t bAppendBlank)
{
    uint8_t digit;
    char *p = writebuf;

    if ( val < 0 ) {
        *(p++)= '-';
        val *= -1;	
    } 
    // Tenth
    digit = val % 10;

    p = my_itoa(val/10, p, 4, false ); 

    // p points to the terminating \0 after call
    // Append DP and tenth digit
    *(p++)= '.';	
    *(p++)= '0' + digit;
    if ( bAppendBlank ) {
        *(p++)= ' ';
        /**** Chng 056 ****/
        *(p++)= ' ';
    }
    *p = '\0';
    return ( p - writebuf );

}	

/*****************************************************************************
 * Initialization of the LCD-Screen
 * Display is aready cleared by caller
 ****************************************************************************/
static uint32_t Init(const void *InitArg)
{
    UNUSED(InitArg);
    #if DEBUG_LCD_MENU > 0
            DEBUG_PUTS("LCD_StatusInit");
    #endif
    LCD_StartPwrOffTmr(config.lcdOnTime);
    return LCD_STATUS_ALL;
}


#if USE_SERIALSTORAGE > 0
	static uint8_t last_useage=0xff;

	/******************************************************************************
	 * Display the pressure in format pppp.p
	 ******************************************************************************/
	static void LCD_Display_ExtMem_Useage(uint8_t scheme, uint8_t force) 
	{
		uint16_t pixlen;


		/* Status text is 6 chars long, so prepare for 6 chars */
	
		if ( SetStor_IsStatusOK() ) {
			/* if Status=OK, display useage in % */
			uint8_t val = SerStor_GetUseagePercent();
			if ( !force && last_useage == val )return;

			last_useage=val;
			char *p = my_itoa(val, LCD_numbuf, 3, false);
		
			//  Add '%'
			*p = '%';
			*(++p)='\0';
		} else {
			/* Otherwise display Status text */
			strcpy_P(LCD_numbuf, SerStor_GetStatusText());
			last_useage = 0xff;
		}

		switch ( scheme ) {
		case 0:
			lcd_moveto_xy(3,105);
			lcd_clear_area(1,37,NORMAL);
			pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
			lcd_moveto_xy(3,132-pixlen-1);
			lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
			break;	
		case 1:
		case 2:
			lcd_moveto_xy(2,100);
			lcd_clear_area(2,32,NORMAL);
			pixlen = lcd_get_strlen(FONT_PROP_16, NORMAL, LCD_numbuf);
			lcd_moveto_xy(2,132-pixlen-1);
			lcd_put_string(FONT_PROP_16, NORMAL, LCD_numbuf);
			break;	
		default:
			break;
		}
	}
#endif


static uint16_t last_battery=0xffff;

/******************************************************************************
 * Display the battery Voltage in the format V.vvv
 ******************************************************************************/
static void LCD_Display_VBatt(uint8_t scheme, uint8_t force) 
{
    uint16_t val;
    uint16_t pixlen;	
    #if defined(ADC1) && defined(USE_ADC1)
        val = ADC_GetVdda(&HW_ADC1);
    #else   
        val = 0:
    #endif
    
    // Paint only, if changed
    if ( !force && val == last_battery ) return;
    last_battery = val;
            
    // leave first place in buffer unused to insert dp later
    my_itoa(val, LCD_numbuf+1, 4, true);
    
    //  Insert dp
    *LCD_numbuf = *(LCD_numbuf+1);
    *(LCD_numbuf+1) = '.';

    switch ( scheme ) {
        case 0:
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
            dogm_moveto_xy(0,132-pixlen-1);
            lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
            break;	
        case 1:
        case 2:
            pixlen = lcd_get_strlen(FONT_PROP_16, NORMAL, LCD_numbuf);
            dogm_moveto_xy(0,132-pixlen-1);
            lcd_put_string(FONT_PROP_16, NORMAL, LCD_numbuf);
            break;	
        case 3:
        case 4:
            pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
            dogm_moveto_xy(3,132-pixlen-1);
            lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
            break;
        default:
            break;
    }
}


static void LCD_Display_Number ( uint8_t num )
{
    uint8_t t = (num/10);
    uint8_t e = num % 10;
    lcd_put_char(FONT_PROP_8, NORMAL, '0'+t);
    lcd_put_char(FONT_PROP_8, NORMAL, '0'+e);
}

static void LCD_Display_Time(uint8_t scheme)
{
    switch ( scheme ) {
        case 0:
        case 1:
        case 3:
        case 4:
            dogm_moveto_xy(3,55);
            break;
        default:
            return;
            break;
    }
    LCD_Display_Number(RTC_GetHour());
    lcd_put_char(FONT_PROP_8, NORMAL, ':');
    LCD_Display_Number(RTC_GetMinute());
}

	


/******************************************************************************
 * Display the System Status
 ******************************************************************************/
static void LCD_Display_StatusText(uint8_t scheme) 
{
    switch ( scheme ) {
            case 0:
            case 1:
            case 3:
            case 4:
                dogm_moveto_xy(3,0);
                break;
            default:
                return;
                break;
    }
    lcd_put_string(FONT_PROP_8, NORMAL, Get_FSK_status_text());
#if USE_RFM_OOK > 0
    lcd_put_char(FONT_PROP_8, NORMAL, '/');
    lcd_put_string(FONT_PROP_8, NORMAL, Get_OOK_status_text());
#endif
}


#if defined(GASSENSOR) || defined(STROMSENSOR)
#include "zaehler.h"

static void LCD_Display_Temp(uint8_t scheme, uint8_t force )
{
	lcd_set_font(FONT_PROP_16, NORMAL);

	lcd_moveto_xy(1,0);
	LCD_ComputeCounter(LCD_numbuf, countervalueHi, 0, 0);
	lcd_putstr(LCD_numbuf);
	LCD_ComputeCounter(LCD_numbuf, countervalue, 1, 1);
	lcd_putstr(LCD_numbuf);

#if defined(GASSENSOR) 
	if ( scheme == 3) {
		lcd_moveto_xy(0,0);
		lcd_clear_area(1,132, NORMAL);

		lcd_set_font(FONT_PROP_8, ( lsm303Flags & (1 << LSM303_MAG_X_NONZERO_FLAG ) ? INVERT : NORMAL));
		lcd_moveto_xy(0,0);
		LCD_ComputeSigned(LCD_numbuf, magData.mx, 0);
		lcd_putstr(LCD_numbuf);

		lcd_set_font(FONT_PROP_8, ( lsm303Flags & (1 << LSM303_MAG_Y_NONZERO_FLAG ) ? INVERT : NORMAL));
		lcd_moveto_xy(0,43);
		LCD_ComputeSigned(LCD_numbuf, magData.my, 0);
		lcd_putstr(LCD_numbuf);

		lcd_set_font(FONT_PROP_8, ( lsm303Flags & (1 << LSM303_MAG_Z_NONZERO_FLAG ) ? INVERT : NORMAL));
		lcd_moveto_xy(0,87);
		LCD_ComputeSigned(LCD_numbuf, magData.mz, 0);
		lcd_putstr(LCD_numbuf);
	}
#elif USE_OPTICAL > 0
	#include "optical.h"
	if ( scheme == 3) {
		lcd_moveto_xy(0,0);
		lcd_clear_area(1,132, NORMAL);

		lcd_set_font(FONT_PROP_8, INVERT);
		lcd_moveto_xy(0,0);
		my_itoa(opt_dark, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);

		lcd_set_font(FONT_PROP_8, NORMAL);
		lcd_moveto_xy(0,43);
		my_itoa(opt_prescaler, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);

		lcd_set_font(FONT_PROP_8, ( !OPTICAL_IS_RED()  ? INVERT : NORMAL));		
		lcd_moveto_xy(0,87);
		my_itoa(opt_dark-opt_light, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);
	}
#elif USE_OPTICAL_EMETER > 0
	extern uint16_t p1;
	extern uint16_t p2;
	extern uint16_t p3;
	if ( scheme == 3) {
		lcd_moveto_xy(0,0);
		lcd_clear_area(1,132, NORMAL);
		lcd_set_font(FONT_PROP_8, NORMAL);

		lcd_moveto_xy(0,0);
		my_itoa(p1, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);

		lcd_moveto_xy(0,43);
		my_itoa(p2, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);

		lcd_moveto_xy(0,87);
		my_itoa(p3, LCD_numbuf, 4, true);
		lcd_putstr(LCD_numbuf);
	}
#endif

	// Display Temp, Draw only, if changed
	if ( force || last_temp != abstemp ) {
		last_temp = abstemp;
		if ( scheme == 3 ) {
			lcd_set_font(FONT_PROP_16, NORMAL);
			lcd_moveto_xy(1,90);
		} else {
			// Scheme 0,1,2
			lcd_set_font(FONT_PROP_8, NORMAL);
			lcd_moveto_xy(0,0);
		}
		LCD_ComputeTemp(LCD_numbuf,(abstemp+5)/10, 1);
		lcd_putstr(LCD_numbuf);
	}
}

#else
/******************************************************************************
 * Display the outdoor temp value 
 ******************************************************************************/
static void LCD_Display_Temp(uint8_t scheme, uint8_t force ) 
{
#if USE_DS18X20 > 0 || defined(TX18LISTENER) || USE_BME280 > 0
    #include "system/util.h"
    uint8_t strlen;

    // Draw only, if changed
    if ( !force && last_temp == abstemp ) return;
    last_temp = abstemp;

    strlen = LCD_ComputeTemp(LCD_numbuf,(abstemp+5)/10, scheme >= 3);
    switch ( scheme  ) {
        case 0:
        case 1:
            dogm_moveto_xy(0,20);
            lcd_put_string(FONT_DIGITS_24, NORMAL, LCD_numbuf);
            // There is no blank included in ths charset, so clear character rectangle
            dogm_clear_area(3,19,NORMAL);
            break;
        case 2:
            dogm_moveto_xy(0,2);
            lcd_put_string(FONT_DIGITS_32, NORMAL, LCD_numbuf);
            // There is no blank included in ths charset, so clear character rectangle
            if ( strlen<4) dogm_clear_area(4,28,NORMAL);
            break;
        case 3:
            lcd_set_font(FONT_PROP_8, INVERT);
            dogm_moveto_xy(0,0);
            lcd_putstr("Min");
            dogm_moveto_xy(0,43);
            lcd_putstr("Temp");
            dogm_moveto_xy(0,87);
            lcd_putstr("Max");
            dogm_moveto_xy(1,43);
            lcd_set_font(FONT_PROP_16, NORMAL);
            lcd_putstr(LCD_numbuf);
            if ( mintemp != MINMAX_UNSET ) {
                LCD_ComputeTemp(LCD_numbuf,(mintemp+5)/10, 1);
                dogm_moveto_xy(1,0);
                lcd_putstr(LCD_numbuf);
            }
            if ( maxtemp != MINMAX_UNSET ) {
                LCD_ComputeTemp(LCD_numbuf,(maxtemp+5)/10, 1);
                dogm_moveto_xy(1,87);
                lcd_putstr(LCD_numbuf);
            }
            break;
        case 4:
            lcd_set_font(FONT_PROP_8, NORMAL);
            dogm_moveto_xy(0,100);
            lcd_putstr("Temp");
            lcd_set_font(FONT_PROP_16, NORMAL);
            dogm_moveto_xy(1,100);
            lcd_putstr(LCD_numbuf);
        default:
            break;
    }
#endif
}
#endif

#if defined(LCD_STATUS_CO2) || defined(LCD_STATUS_TVOC)
    static uint16_t last_co2=0xffff;
    static uint16_t last_tvoc = 0xffff;

    /******************************************************************************
     * Display CO2 ppm and/or TVOC ppb as decimal integer 
     ******************************************************************************/
    static void LCD_Display_Environmental(uint8_t scheme, uint8_t force) 
    {
        uint16_t val;
        uint8_t style;

        switch ( scheme ) {
        case 0:
        case 1:
        case 2:
        case 3:
            break;
        case 4:
            /**** 002 ****/ 
            /* On measurement error, print headline inverted */
            if ( CTL_error & ERR_THPSENSOR ) {
                 style = INVERT;
                 force = true;
            } else 
                 style =NORMAL;
            /* CO2 */
            if ( THPSENSOR_GetCapability() & THPSENSOR_HAS_CO2 ) {
                val = (uint16_t)THPSENSOR_GetCO2();
                // Display only, if changed
                if ( force || last_co2 != val ){
                    last_co2=val;
                    my_itoa(val, LCD_numbuf, 5, false);
                    lcd_set_font(FONT_PROP_8, style);
                    dogm_moveto_xy(0,0);
                    lcd_putstr("CO2 ppm");
                    lcd_set_font(FONT_PROP_16, NORMAL);
                    dogm_moveto_xy(1,0);
                    lcd_putstr(LCD_numbuf);
                    /* Clear one char position behind */
                    dogm_clear_area(2,11,NORMAL);
                }
            }
            /* TVOC */
            if ( THPSENSOR_GetCapability() & THPSENSOR_HAS_TVOC ) {
                val = (uint16_t)THPSENSOR_GetTVOC();
                // Display only, if changed
                if ( force || last_tvoc != val ){
                    last_tvoc=val;
                    my_itoa(val, LCD_numbuf, 5, false);
                    lcd_set_font(FONT_PROP_8, style);
                    dogm_moveto_xy(0,50);
                    lcd_putstr("TVOC ppb");
                    lcd_set_font(FONT_PROP_16, NORMAL);
                    dogm_moveto_xy(1,50);
                    lcd_putstr(LCD_numbuf);
                    /* Clear one char position behind */
                    dogm_clear_area(2,11,NORMAL);
                }
            }
            break;
        } // switch
    }
#endif
#if defined(LCD_STATUS_PRESSURE)
    static uint16_t last_pressure=0xffff;

    /******************************************************************************
     * Display the pressure in format pppp.p
     ******************************************************************************/
    static void LCD_Display_Pressure(uint8_t scheme, uint8_t force) 
    {
        /**** 006 **** 
         * THPSENSOR_GetP will return the local pressure. It has to be compensated
         * to get the MSL pressure
         */          
        uint16_t val = (uint16_t)(int16_t)THPSENSOR_GetP_MSL();
        uint16_t pixlen;	

        // Display only, if changed
        if ( !force && last_pressure == val )return;
        last_pressure=val;

        // leave first place in buffer unused to insert dp later
        char *p = my_itoa(val, LCD_numbuf, 5, false);

        //  Insert dp 
        *p = *(p-1);
        *(p-1) = '.';
        *(++p)='\0';

        switch ( scheme ) {
            case 0:
                pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
                dogm_moveto_xy(2,132-pixlen-1);
                lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
                break;	
            case 1:
            case 2:
                pixlen = lcd_get_strlen(FONT_PROP_16, NORMAL, LCD_numbuf);
                dogm_moveto_xy(2,132-pixlen-1);
                lcd_put_string(FONT_PROP_16, NORMAL, LCD_numbuf);
                break;	
            default:
                break;
        }
    }
#endif
#if defined(LCD_STATUS_RELHUM)
    static uint16_t last_rh=0xffff;

    /******************************************************************************
     * Display the pressure in format ppp%
     ******************************************************************************/
    static void LCD_Display_RelHum(uint8_t scheme, uint8_t force) 
    {

        /* Relative Humidity is only displayed in scheme 0 */
        if ( scheme != 0 ) return;

        /* Original value is promille, so convert to % */
        uint16_t val = ((uint16_t)(int16_t)THPSENSOR_GetH())/10;
        uint16_t pixlen;	

        // Display only, if changed
        if ( !force && last_rh == val )return;
        last_rh=val;

        char *p = my_itoa(val, LCD_numbuf, 5, false);

        //  Append % 
        *p = '%';
        *(++p)='\0';
        /* we need no switch(scheme) here, rel. humidity is displayed only in scheme 0 */
        pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
        dogm_moveto_xy(3,132-pixlen-1);
        lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
    }
#endif

#if defined(TX18LISTENER)
    #include "global_flags.h"
    /******************************************************************************
     * Display the RFM12 Radio OOK status
     ******************************************************************************/
    static void LCD_Display_Radio( uint8_t scheme, uint8_t force ) 
    {
        UNUSED(force);
        static char letter=' ';
        char bNormal = true;
        if  (OOK_Receiver_running()) {
            // OOK-Receiver is running: Check for retry
            if ( OOK_Receiver_retrying() )
                letter = 'r';
            else 
                letter = 'o';
        } else {
            // OOK Receiver not running
            if ( FSK_Transceiver_running() ) {
                // if FSK-receiver is running, LCD updates are inhibited, so this code
                // should not be executed!
                letter = '!';
            } else {
                // RFM12 transceiver is idle
                bNormal=false;
            }
        }

        switch ( scheme  ) {
        case 0:
        case 1:
            dogm_moveto_xy(0,0);
            break;
        case 3:
            dogm_moveto_xy(0,125);
            break;
        default:
            return;
            break;
        }

        if ( bNormal) 
            lcd_put_char(FONT_PROP_8, INVERT, letter);
        else
            lcd_put_char(FONT_PROP_8, NORMAL, letter);
            // lcd_clear_area(0,2, NORMAL);
    }
#endif

void LCD_DisplayNumberTest(int8_t number)
{
    LCD_ComputeTemp(LCD_numbuf,number,0);
    dogm_moveto_xy(0,20);
    lcd_put_string(FONT_DIGITS_24, NORMAL, LCD_numbuf);
}


static uint8_t  old_scheme=0;
static uint8_t  force=0;


/*****************************************************************************
 * The following functions is called by main(), when then TASK_LCD-Falg is set
 * The function is called only, if RFM12 is idling to avoid mixing SPI 
 * instructions between RFM12 and DOGM132
 ****************************************************************************/
static uint32_t OnRedraw(uint32_t redraw_bits)
{
    uint8_t work = config.displayScheme;
    if ( old_scheme != work ) {
        // Clear the entire display
        dogm_clear_display(DOGM_NORMAL);
        old_scheme=work;
        // Repaint all items
        redraw_bits=LCD_STATUS_ALL;
        return redraw_bits;
    }

    if ( redraw_bits == LCD_STATUS_ALL ) force = true;

    #if DEBUG_LCD_MENU > 0
        if ( config.dbg_level>2) {
            DEBUG_PRINTF("ReDrawStatus(%02x)\n",redraw_bits);
        }
    #endif


    if      (redraw_bits & LCD_STATUS_TEMP) 	{ redraw_bits &= ~LCD_STATUS_TEMP;	LCD_Display_Temp(work,force); }
    else if (redraw_bits & LCD_STATUS_BATTERY) 	{ redraw_bits &= ~LCD_STATUS_BATTERY;	LCD_Display_VBatt(work,force); }
    else if (redraw_bits & LCD_STATUS_TIME)	{ redraw_bits &= ~LCD_STATUS_TIME; 	LCD_Display_Time(work); }
    else if (redraw_bits & LCD_STATUS_RFM)	{ redraw_bits &= ~LCD_STATUS_RFM; 	LCD_Display_StatusText(work); }
#if defined(LCD_STATUS_PRESSURE)
    else if (redraw_bits & LCD_STATUS_PRESSURE)	{ redraw_bits &= ~LCD_STATUS_PRESSURE;  LCD_Display_Pressure(work,force); }
#endif
#if defined(LCD_STATUS_RELHUM)
    else if (redraw_bits & LCD_STATUS_RELHUM)	{ redraw_bits &= ~LCD_STATUS_RELHUM;    LCD_Display_RelHum(work,force); }
#endif
#if defined(LCD_STATUS_CO2)
    else if (redraw_bits & LCD_STATUS_CO2)	{ redraw_bits &= ~LCD_STATUS_CO2;  LCD_Display_Environmental(work,force); }
#endif
#if defined(LCD_STATUS_TVOC)
    else if (redraw_bits & LCD_STATUS_TVOC)	{ redraw_bits &= ~LCD_STATUS_TVOC;  LCD_Display_Environmental(work,force); }
#endif
#if USE_SERIALSTORAGE > 0
    else if (redraw_bits & LCD_STATUS_EXTMEM)	{ redraw_bits &= ~LCD_STATUS_EXTMEM;	LCD_Display_ExtMem_Useage(work,force); }
#endif
#if defined(GASSENSOR)
    else if (redraw_bits & LCD_STATUS_VIN)	{ redraw_bits &= ~LCD_STATUS_VIN; 	LCD_Display_Vin(work,force); }
#elif defined(TX18LISTENER)
    else if (redraw_bits & LCD_STATUS_RADIO)	{ redraw_bits &= ~LCD_STATUS_RADIO; 	LCD_Display_Radio(work,force); }
#endif
    if ( !redraw_bits ) force = false;
 
    return redraw_bits;
}


/*****************************************************************************
 * Callback for the rotary's rotary knob in normal mode
 * i.e. change the display scheme
 ****************************************************************************/
static void OnRotary(int32_t delta)
{
    #if DEBUG_LCD_MENU > 0
        DEBUG_PRINTF("Status-ChngScheme(%d)\n",delta);
    #endif
    if ( delta ) {
        int16_t work = config.displayScheme + delta;
        if ( work >= eelimits[EEPROM_DISPLAY_SCHEME_IDX].min && work <= eelimits[EEPROM_DISPLAY_SCHEME_IDX].max ) {
            #if DEBUG_MODE > 0 && DEBUG_LCD > 0 
                DEBUG_PRINTF("NewScheme=%d\n",work);
            #endif
            config.displayScheme = work;
            ReEnableRotary();
            LCD_RestartPwrOffTmr();
            LCD_DisplayStatus(LCD_STATUS_ALL);
        }
    } else {
        // delta == 0 means: Rotary Deactivated due to timeout
        LCD_PWM(0);
    }
}

/*****************************************************************************
 * Callback for the rotary's on/off switch Double Click
 * Called only once at the second depress
 ****************************************************************************/
static void OnDblClick(void)
{
   LCD_SwitchToSelector(&MainExecutor);
}


/*****************************************************************************
 * Callback for the rotary's on/off switch
 * Called with argument != 0 when on, and with argument=0 when off
 ****************************************************************************/
static void OnClick(void)
{
    #if DEBUG_LCD_MENU > 0
        DEBUG_PUTS("Status-Click");
    #endif

    /* Rotary and BkLight are enabled for the configured BkLight OnTime */
    LCD_PWM(config.def_intensity);
    EnableRotary((uint32_t)config.bklightOnTIme);
    LCD_RestartPwrOffTmr();
}


/******************************************************************************
 * Display a signed 16 bit number with five digits, no leading zeroes
 * if bAppendBlank != 0, one blank is appended
 * returns the length of the returned string [excluding \0]
 ******************************************************************************/
uint8_t LCD_ComputeSigned(char *writebuf, int16_t val, uint8_t bAppendBlank)
{
	char *p = writebuf;

	if ( val < 0 ) {
		*(p++)= '-';
		val *= -1;	
	} else {
		*(p++)= '+';
	}

	p = my_itoa(val, p, 5, false ); 

	if ( bAppendBlank ) {
		*(p++)= ' ';
		*(p++)= ' ';
		*p = '\0';
	}

	return ( p - writebuf );

}	

DEFINE_SCREEN(Status, Init, OnClick, OnDblClick, OnRotary, OnRedraw, NULL, true);

#endif // if USE_DISPLAY > 0

#if defined(GASSENSOR) || defined(STROMSENSOR)
	/******************************************************************************
	 * Display an unsigned 16 bit number with leading zeros and one digit 
	 * behind the dp 
	 * if bAppendBlank != 0, one blank is appended
	 * returns the length of the returned string [excluding \0]
	 ******************************************************************************/
	uint8_t LCD_ComputeCounter(char *writebuf, uint16_t val, uint8_t bWithDP, uint8_t bAppendBlank)
	{
		uint8_t digit;
		char *p = writebuf;

		// Tenth
		digit = ( bWithDP ? val % 10 : val );

		if ( bWithDP )
			p = my_itoa(val/10, p, 3, true ); 
		else
			p = my_itoa(val, p, 4, true );

		// p points to the terminating \0 after call
		if ( bWithDP) {
			// Append DP and tenth digit
			*(p++)= '.';	
			*(p++)= '0' + digit;
		}

		if ( bAppendBlank ) {
			*(p++)= ' ';
			/**** Chng 056 ****/
			*(p++)= ' ';
		}
		*p = '\0';
		return ( p - writebuf );

	}	
#endif

