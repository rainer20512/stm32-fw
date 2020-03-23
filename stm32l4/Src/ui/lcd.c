/******************************************************************************
 * Implements functionality for an pixel lcd
 *****************************************************************************/
#include "config/config.h"


#if USE_DISPLAY > 0


#include "ui/lcd.h"

// multi purpose buffer for all kinds to text or number display in the whole LCD-context
#define LCD_NUMBUFLEN	12
extern char LCD_numbuf[];
char LCD_numbuf[LCD_NUMBUFLEN];

#include <stdarg.h>

#include "task/minitask.h"
#include "timer.h"
#include "eeprom.h"
#include "global_flags.h"
#include "debug_helper.h"
#include "system/periodic.h"
#include "disp/fonts/lcd_fonts.h"


#include "ui/lcd.h"
#include "ui/lcd_status.h"
#include "ui/lcd_interface.h"
#include "ui/lcd_Selector.h"
#include "ui/lcd_Text.h"
#include "ui/lcd_ValueEdit.h"

uint32_t redraw_items = 0;

char LCD_numbuf[LCD_NUMBUFLEN];

/* My associated quadrature encoder */
static const HW_DeviceType *myQenc            = NULL;

/* List of all LCD screens, actual LCD screen and default LCD screen */ 
static const LCDScreenT* lcd_screens[]= {
    ADD_SCREEN(Status),
    ADD_SCREEN(ValueEdit),
    ADD_SCREEN(Selector),
    ADD_SCREEN(Text),
};

const LCDScreenT     *act_lcd_screen;    /* ptr to actual LCD screen definition */
static const LCDScreenT *default_lcd_screen;    /* ptr to default LCD screen definition */
 
static uint32_t rotoff_time;    /* last passed rotary off time */
static uint32_t pwroff_time;    /* last passed poweroff time */

/* Fixed timer ID ( second timer )                                           */
static int8_t RotaryTimerID= NO_TIMER_ID;   /* ID of my rotary timeout timer */
       int8_t LcdOnTimerID = NO_TIMER_ID;   /* ID of my On-Timer             */


/* - Forward declarations ---------------------------------------------------*/
void LCD_WakeUp(void);
void LCD_SwitchToSleep(void);
void RotaryRotateCallback(int32_t delta );

/*****************************************************************************
 * The following functions is called by main(), when then TASK_LCD-Falg is set
 * The function is called only, if RFM12 is idling to avoid mixing SPI 
 * instructions between RFM12 and DOGM132
 ****************************************************************************/
uint32_t LCD_ReDraw(void)
{
    redraw_items = ( act_lcd_screen && act_lcd_screen->OnRedraw ? act_lcd_screen->OnRedraw(redraw_items) : 0 );
    return redraw_items;
}



/*****************************************************************************
 * Clear entire Display
 ****************************************************************************/
void LCD_ClearDisplay(void)
{
	// Clear the entire display
	dogm_clear_display(DOGM_NORMAL);
}

/*****************************************************************************
 * Clear entire Display and write text centered in line 1
 ****************************************************************************/
void LDC_Display_Splash(const char *splash) {

	LCD_ClearDisplay();

	uint16_t pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, splash);
	dogm_moveto_xy(1,(131-pixlen)/2);
	lcd_put_string(FONT_PROP_8, NORMAL, splash);
}

/*****************************************************************************
 * Write a percentage value ( aka Progress ) centered on Display
 ****************************************************************************/
void LDC_Display_Progress(uint8_t progress) {

	char *p = my_itoa(progress, LCD_numbuf, 3, false);
			
	//  Add '%'
	*p = '%';
	*(++p)='\0';

	uint16_t pixlen = lcd_get_strlen(FONT_PROP_8, NORMAL, LCD_numbuf);
	dogm_moveto_xy(1,(131-pixlen)/2);
	lcd_put_string(FONT_PROP_8, NORMAL, LCD_numbuf);
}


/*****************************************************************************
 * activate the screen "active" :
 * - clear the entire display
 * - call "active"'s OnInit function with "InitArg" as argument
 ****************************************************************************/
void LCD_Activate( const LCDScreenT *active, const void *InitArg )
{
    act_lcd_screen = active;
    LCD_ClearDisplay();
    if ( act_lcd_screen->OnInit) 
        redraw_items = act_lcd_screen->OnInit(InitArg);
    else    
        redraw_items = 0;

    TaskNotify(TASK_LCD);
}   

void LCD_SwitchToDefault(void)
{
    LCD_Activate(default_lcd_screen, NULL);
}

/*----------------------------------------------------------------------------
 -----------------------------------------------------------------------------
 - All the rotary encoder stuff
 -----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/

/******************************************************************************
 * Rotary timeout callback
 * - Disable Rotary
 * - if set call Rotary Callback with a delta of 0 ( that indicates termination )
 *****************************************************************************/
static void RotTimeoutCB(uint32_t arg)
{
    UNUSED(arg);
    QEnc_DeActivate(myQenc);
}

/******************************************************************************
 * Restart the Rotaries OnTime-interval
 *****************************************************************************/
void ReEnableRotary(void)
{
    if ( rotoff_time ) SecTimerReSetRel(RotaryTimerID, rotoff_time);
}

/******************************************************************************
 * Enable Rotary with a specific callback and timeout
 *****************************************************************************/
void EnableRotary(uint32_t rot_timeout_sec)
{
    if ( myQenc ) {
       /* rotate callback is deleted on deactivation and must be re-set */
        QEnc_SetRotateCallback  (myQenc, RotaryRotateCallback); 
        rotoff_time = rot_timeout_sec;
        QEnc_Activate(myQenc);
        SecTimerReUseRel (RotaryTimerID, rot_timeout_sec, false, RotTimeoutCB, 0 );
    } else {
        rotoff_time = 0;
    }
}

/******************************************************************************
 * Disable Rotary before timeout
 *****************************************************************************/
void DisableRotary(void)
{
    SecTimerDelete(RotaryTimerID);
    QEnc_DeActivate(myQenc);
}

/******************************************************************************
 * Set Rotary click callback
 *****************************************************************************/
void RotaryClickCallback(void)
{
    if ( act_lcd_screen->OnClick ) act_lcd_screen->OnClick();
}

/******************************************************************************
 * Set Rotary DblClick callback
 *****************************************************************************/
void RotaryDblClickCallback(void)
{
    if ( act_lcd_screen->OnDblClick ) act_lcd_screen->OnDblClick();
}

/******************************************************************************
 * Rotary Rotate callback
 *****************************************************************************/
void RotaryRotateCallback(int32_t delta )
{
    if ( act_lcd_screen->OnRotary ) act_lcd_screen->OnRotary(delta);
}


/*****************************************************************************
 * Wrap the LCD_SwitchToSleep into Timer Callback
 ****************************************************************************/
static void LCD_EnterPowerSaveCB( uint32_t arg )
{
    UNUSED(arg);
    if ( act_lcd_screen->OnPwrOff ) 
        act_lcd_screen->OnPwrOff();
    else
        LCD_SwitchToSleep();
}

/*****************************************************************************
 * if an lcdOnTime > 0 is configured, start the pwroff timer for that interval
 * The timeout fn can be passed, if NULL, the standard poweroff fn is used
 ****************************************************************************/
void LCD_StartPwrOffTmr ( uint32_t secs_to_pwroff )
{
    pwroff_time = secs_to_pwroff;

    if  ( secs_to_pwroff ) {
        SecTimerReUseRel(LcdOnTimerID, secs_to_pwroff, false, LCD_EnterPowerSaveCB, 0  );
    }
}

/******************************************************************************
 * Restart the Rotaries OnTime-interval
 *****************************************************************************/
void LCD_RestartPwrOffTmr ( void )
{
    if (pwroff_time) SecTimerReSetRel(LcdOnTimerID, pwroff_time);
}


/*****************************************************************************
 * find the default screeen in all defined screens (execute once at start )
 ****************************************************************************/
static void LCD_DisplayInit(void) 
{
      act_lcd_screen    = NULL;
      default_lcd_screen = NULL;
      uint32_t num_screens = sizeof(lcd_screens)/sizeof(LCDScreenT*);

      if ( num_screens == 0 ) {
        #if DEBUG_MODE > 0 
            DEBUG_PUTS("LCD.c - Error: No screens defined!");
        #endif
        return;
      }
      for ( uint32_t i = 0; i < num_screens; i++ ) {
          if ( lcd_screens[i]->bDefaultScr ) {
            default_lcd_screen = lcd_screens[i];
            break;
          }
      }

      /* if no screen is assigned the default screen, take the first one as default */
      if ( default_lcd_screen == NULL ) default_lcd_screen = lcd_screens[0];
}


/******************************************************************************
* @brief  Initializes the LCD at .
* @param  None
* @retval EPD state
 *****************************************************************************/
void LCD_PostInit(const HW_DeviceType *dev, void *bRotate)
{

  #if USE_DOGM132 > 0
     #include "disp/dogm-graphic.h"
     /* lcd Init and clear */
     dogm_init(dev, (uint32_t)bRotate != 0);
  #endif
  RotaryTimerID = SecTimerAllocate(RotaryTimerID);
  LcdOnTimerID  = SecTimerAllocate(LcdOnTimerID);
  LCD_DisplayInit();
  LCD_PWM(0);
  LDC_Display_Splash("Starting...");    
  #if USE_QENCODER > 0
      myQenc = &QENC_DEV;
      QEnc_SetDblClickCallback(myQenc, RotaryDblClickCallback);
      QEnc_SetClickCallback   (myQenc, RotaryClickCallback);
//      QEnc_SetRotateCallback  (myQenc, RotaryRotateCallback);
      /* Rotate callback will be enabled when rotary is enabled */
  #endif
}


/*****************************************************************************
 * WakeUp the LCD after sleep
 ****************************************************************************/
void LCD_WakeUp(void)
{
    #if DEBUG_LCD_MENU > 0
        DEBUG_PUTS("LCD WakeUp");
    #endif
    DOGM_POWERSAVE_OFF();
    ClearFlagBit(gflags, GFLAG_DISPOFF_BIT);
    #if USE_QENCODER > 0
       QEnc_SetClickCallback(myQenc, RotaryClickCallback);
    #endif
    LCD_SwitchToDefault();
}

/*****************************************************************************
 * Put the LCD into sleep mode and notice this in "current_lcd_topic
 * Does not write anything at this time
 ****************************************************************************/
void LCD_SwitchToSleep(void) 
{
    if ( config.lcdOnTime > 0 ) {
        #if DEBUG_LCD_MENU > 0
            DEBUG_PUTS("LCD Switch to Sleep: Off");
        #endif
        DisableRotary();
        #if USE_QENCODER > 0
            QEnc_SetClickCallback   (myQenc, LCD_WakeUp );
            /* Rotate callback is reset when going to sleep, so set again here */
            QEnc_SetRotateCallback  (myQenc, RotaryRotateCallback);
        #endif
        LCD_PWM(0);
        DOGM_POWERSAVE_ON();
        SetFlagBit(gflags, GFLAG_DISPOFF_BIT);
        act_lcd_screen = NULL;
    } else {
        #if DEBUG_LCD_MENU > 0
            DEBUG_PUTS("LCD Switch to Sleep: Status");
        #endif
        LCD_SwitchToDefault();
    }
}

/******************************************************************************
 * wrapper to display the time on display
 *****************************************************************************/
static void task_lcd_time( void *arg)
{
    UNUSED(arg);
    LCD_DisplayStatus(LCD_STATUS_TIME);
}

/******************************************************************************
 * Initialization for LCD task: WakeUp LCD and schedule an periodic update
 * at second 0 to actualize the time
 *****************************************************************************/
void task_init_lcd ( void )
{
    LCD_WakeUp();
    AtSecond(0, task_lcd_time, (void *)0, "Actualize Time on display");
}

/******************************************************************************
 * task handle for lcd updates
 *****************************************************************************/
void task_handle_lcd ( uint32_t arg )
{
    UNUSED(arg);
    if ( LCD_ReDraw() ) TaskNotify(TASK_LCD);
}

/*********************************************************************************************
 * Additional Code for Background LED PWM operation. For that Timer0 is used
 ********************************************************************************************/

#if USE_LCD_BACKLIGHT > 0

    #include "dev/timer_dev.h"

    /* My associated quadrature encoder */
    static const HW_DeviceType *myPWM       = NULL;
    static uint8_t              myPWMCh;
    static uint8_t              old_pwm_idx = 0;

    const uint8_t pwmtable_832[32]  =
    {
        0,  1,  1,  2,  3,  3,  4,  5,  6,  7,   8,   9,  10,  12,  14, 16, 
       19, 23, 27, 32, 38, 45, 54, 64, 76, 91, 108, 128, 152, 181, 215, 255
    };
 

    /**************************************************************************
     * Assign PWM timer device and PWM channel
     *************************************************************************/
    void LCD_SetPWMDev( const HW_DeviceType *devPWM, uint32_t chPWM )
    {
        myPWM   = devPWM;
        myPWMCh = chPWM;
    }

    void LCD_SetBacklightPWM( uint32_t pwm_idx )
    {
        if ( pwm_idx == old_pwm_idx ) return;

        if ( pwm_idx == 0 ) {
            TMR_StopPWMCh(myPWM, myPWMCh);
        } else {
            if ( old_pwm_idx == 0 ) TMR_InitPWMCh(myPWM, myPWMCh, false);
            uint32_t work = pwmtable_832[pwm_idx];

            /* Instead of 99,5% duy cycle, switch permanent on */
            if ( work == 255 ) work++;
            #if DEBUG_MODE > 0 && DEBUG_LCD > 0
                    DEBUG_PRINTF("PWM=");print_decXX(work);CRLF();
            #endif 
            TMR_StartPWMChS256(myPWM, myPWMCh,work);
        }

        old_pwm_idx = pwm_idx;
     }

#endif // USE_LCD_BACKLIGHT > 0

#endif // USE_DISPAY > 0

