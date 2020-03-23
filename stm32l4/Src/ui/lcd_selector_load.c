/******************************************************************************
 * Fill the text and the response action for lcd_selector
 *****************************************************************************/

#include "config/config.h"

#if USE_DISPLAY > 0

#include <stdarg.h>

#include "rtc.h"
#include "eeprom.h"
#include "ds18xxx20.h"
#include "system/status.h"

#include "ui/lcd_interface.h"
#include "ui/lcd_selector.h"
#include "ui/lcd_valueedit.h"
#include "ui/lcd_text.h"
#include "debug_helper.h"

#if USE_DUMPER > 0
    #include "Dumper.h"
#endif

#if USE_SERIALSTORAGE > 0
    #include "SerialStorage.h"
#endif
#if USE_LSM303D > 0
    #include "lsm303d.h"
#endif

void LCD_InitSelectorText (  uint8_t num, ...)
{
    // Copy Parameter List to sel
    uint32_t i;
    va_list args;
    va_start(args, num);
     if ( num > SELECTOR_ITEMS_MAX ) num = SELECTOR_ITEMS_MAX;
    sel.items_used = num;
    for (i = 0; i < num; i++) {
        sel.items[i] = va_arg(args, const char *);
        #if DEBUG_LCD_MENU > 0
            printf("Item[%d]=%s\n",i,sel.items[i]);
    #endif
    }
    va_end(args);
}


#if USE_DUMPER > 0
	void EraserCallback ( uint8_t ret );

	void LCD_EraseEeprom (void )
	{
		RTC_gen_timer_set_abs( (uint8_t)(RTC_s256()+RTC_TIMER_CALC(950)), LCD_SwitchToStatus );
		LDC_Display_Splash(PSTR("Erasing ext Mem...."));
		EraserCallback(1);
	}
#endif

#if defined(GASSENSOR) || defined(STROMSENSOR)

	#include "lsm303d.h"
	#include "zaehler.h"

	struct MagDataSum_s sum;

	void LCD_SaveCounter (void )
	{
		RTC_gen_timer_set_abs( (uint8_t)(RTC_s256()+RTC_TIMER_CALC(950)), LCD_SwitchToStatus );
		LDC_Display_Splash(PSTR("Save Counter to EEPROM"));
		Zaehler_StoreToEeprom();
	}

#if defined(GASSENSOR)
	static void ReferenceFinished(uint8_t ret, struct MagData_s *xxx)
	{
		TWI_Unlock();
		RTC_gen_timer_set_abs( (uint8_t)(RTC_s256()+RTC_TIMER_CALC(950)), LCD_SwitchToStatus );
		if ( ret )
			LDC_Display_Splash(PSTR("Calibration ok"));
		else
			LDC_Display_Splash(PSTR("Calibration failed"));
	}

	void LSM303_Recalibrate(void)
	{
		if ( TWI_Lock() ) LSM303_ReferenceMagnetic ( &sum, ReferenceFinished, true );

	}
#elif defined(STROMSENSOR)
	static void Opt_CalFinished(void)
	{
		RTC_gen_timer_set_abs( (uint8_t)(RTC_s256()+RTC_TIMER_CALC(950)), LCD_SwitchToStatus );
		if ( 1 )
			LDC_Display_Splash(PSTR("Calibration ok"));
		else
			LDC_Display_Splash(PSTR("Calibration failed"));
	}

	void Opt_Recalibrate(void)
	{
		Opt_CalFinished();

	}

#endif

	void SelectorExecMain( uint32_t ret)
	{
		switch (ret) {
			case 0:
				 LCD_SwitchToSelector(MainExecutor);
				 break;
			case 1:
				LDC_Display_Splash(PSTR("Recalibrating..."));
				#if defined(GASSENSOR)
					LSM303_Recalibrate();
				#else
					Opt_Recalibrate();
				#endif
				break;
			case 2:
				LCD_SwitchToSleep();
				break;
			default:
				#if DEBUG_LCD_MENU > 0
					uart_puts_P("SelectorCallback: Unexpected RetVal");CRLF();
				#endif
			LCD_SwitchToDefault()();
		}
	}


	void LCD_InitSelectorMore ( void )
	{
            LCD_InitSelectorText(3, "1-Back", "2-Calibrate", "3-LCD Off" );
	}
    
    const MenuExecutor MoreExecutor = MAKE_EXECUTOR(FillSelectorItemsMore, SelectorExecMore );
#endif


void FillSelectorItemsMain ( void )
{
    #if defined(GASSENSOR) || defined(STROMSENSOR)
        LCD_InitSelectorText( 8, "1-Back", "2-Config Menu", "3-Set Date", "4-DumpEepr", "5-EraseEepr", "6-InfoEepr", "7-SaveCntr", "8-More" );
    #else
        LCD_InitSelectorText( 7, "1-Back", "2-Config Menu", "3-Set Date", "4-DumpEepr", "5-EraseEepr", "6-InfoEepr", "7-LCD Off" );
    #endif
    
}
/*****************************************************************************
 * Callback after termination of selector
 * The Selected index will be passed as argument
 ****************************************************************************/
void SelectorExecMain ( uint32_t idx )
{
    switch (idx) {
        case 1:
            LCD_SwitchToValueEdit(&ConfigEditFeeder);
            break;
        case 2:
            LCD_SwitchToValueEdit(&DateTimeEditFeeder);
            break;
#if USE_DUMPER > 0
            case 3:
                LDC_Display_Splash(PSTR("Dumping EEPROM..."));
                Dumper_PrintAll(NULL);
                break;
            case 4:
                LCD_EraseEeprom();
                break;
#else
            case 3:
            case 4:
                LCD_SwitchToText((const void *)LCD_TEXT_NOT_IMPLEMENTED);
                break;
#endif
        case 5:
            LCD_SwitchToText(LCD_TEXT_EEPROM_USEAGE);
            break;
#if defined(GASSENSOR) || defined(STROMSENSOR)
            case 6:
                LCD_SaveCounter();
                break;
            case 7:
                LCD_SwitchToSelector(MoreExecutor);
                break;
#else
            case 6:
                LCD_SwitchToSleep();
                break;
#endif
        default:
            #if DEBUG_LCD_MENU > 0
                DEBUG_PUTS("SelectorCallback: Unexpected RetVal");
            #endif
            LCD_SwitchToDefault();
    }

}

const MenuExecutor MainExecutor = MAKE_EXECUTOR(FillSelectorItemsMain, SelectorExecMain );

#endif // if USE_DISPLAY > 0