/**
  ******************************************************************************
  * 
  * @file    portenta_h7.c
  * @author  Rainer
  * @brief   minimal BSP for Portenta H7 board, mainly for debug purposes
  *          only used in the boot phase, later we access the LEDs via IO-device
  *
  ******************************************************************************
  */

#include "config/config.h"
#include "bsp/arduino_wrapper.h"
#include "bsp/portenta_h7.h"
#include "hardware.h"

static const ARD_PinT myLeds [MAX_LED]= {
    { GPIOK, 5, UNDEF }, // PK5, rd
    { GPIOK, 6, UNDEF }, // PK6, gn
    { GPIOK, 7, UNDEF }, // PK7, bl
};

/* Set to != 0 for every LED, which is powered in inverted mode, i.e. light when pin is LOW and dark when pin is high */
static const uint8_t invertedLeds [MAX_LED]= {
    1,
    1,
    1,
};

#if defined(CORE_CM4)
    #define MULTIPLIER  1024
#else
    #define MULTIPLIER  2048
#endif    
static uint32_t _wait(uint32_t duration );uint32_t _wait(uint32_t duration )
{
    uint32_t k=0;
    for ( uint32_t j=0; j < duration * MULTIPLIER; j++ ) {
        k += j;
        asm("nop");
    }
    return k;
}


void BSP_LED_Init(uint32_t lednum)
{
     if ( lednum < MAX_LED ) {

        /* inverted LEDs are driven by OD-Pin in order to allow higher LED voltages */ 
        if ( invertedLeds[lednum] ) {
            pinInitOd(myLeds[lednum], PULLUP, UNDEF);
        } else {
            pinInitPp(myLeds[lednum], NOPULL,UNDEF);
        }
     }
}

void BSP_LED_Toggle(uint32_t lednum)
{
     if ( lednum < MAX_LED ) {
        GPIO_TypeDef *g = myLeds[lednum].gpio;
        g->ODR ^= ( 1 << myLeds[lednum].pin );
     }
}

static void _onoff ( uint32_t lednum, uint32_t OnOffVal )
{
    /* be sure, that range of lednum has been checked */
    GPIO_TypeDef *g = myLeds[lednum].gpio;

    /* Set the corresponding Set- or Reset-Bit in BSRR */
    uint32_t bitpos = (OnOffVal ? 1 : 1 << 16) << myLeds[lednum].pin;
    g->BSRR = bitpos;
}


void BSP_PinHigh(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, 1 );    
}

void BSP_PinLow(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, 0 );    
}

void BSP_LED_On(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, invertedLeds[lednum] ? 0 : 1 );    
}

void BSP_LED_Off(uint32_t lednum)
{
    if ( lednum >= MAX_LED ) return;
    _onoff ( lednum, invertedLeds[lednum] ? 1 : 0 );    
}

    
const ARD_PinT oscOnOff = {GPIOH, 1, HIGH};
void BSP_Start_25MHz_Osc(void)
{
/* 25 MHz Oscillator must be started by setting PH1 to 1 */
    pinInitPp(oscOnOff, NOPULL, HIGH);
    /* Wait for osciallator to stabilize */
    _wait(250);
}

/* 
 * LAN8742 nRST is tied to a dedicated GPIO pin ( PJ15 ) 
 * nReset pulse has to be applied manually
 * MODE[2:0] of LAN8742 is bootstrapped during nReset Pulse
 * via CRS_DV, RXD1, RXD0 pins. 
 * So be sure to set them appropriately before applying nReset
 * minimal duration for nReset Pulse is 100 microsecs
 */
const ARD_PinT lan8742_nReset = {GPIOJ, 15, HIGH};
const ARD_PinT lan8742_Bootstrap[] = {
    { GPIOC, 0, HIGH },               // Mode[0], RXD0
    { GPIOC, 1, HIGH },               // Mode[1], RXD1
    { GPIOA, 7, HIGH },               // Mode[2], CRS_DV
    { NULL,  0, 0    },               // List Deilimter
};

void BSP_LAN7842_Bootstrap(void)
{
    const ARD_PinT *ptr = lan8742_Bootstrap;
    while ( ptr->gpio != NULL ) {
        pinInitPp( *ptr, NOPULL, ptr->initVal);
        ptr++;
    }
    pinInitOd(lan8742_nReset, PULLUP, HIGH);

    _wait(25);
    digitalWrite(lan8742_nReset, LOW);
    _wait(250);
    digitalWrite(lan8742_nReset, HIGH);
    _wait(250);

}

const ARD_PinT pmicModePin = { GPIOJ, 0, LOW };

/* 
 * PMIC PF1550 standby pin is tied to PJ0
 * which is initially set to "Run", i.e. tied low.
 */
void PMIC_SetMode( PMIC_RunmodeT mode )
{
   digitalWrite(pmicModePin, mode == PMIC_STBY ? HIGH : LOW );
}

static void PMIC_ModeInit(void)
{
    pinInitPp( pmicModePin, NOPULL, pmicModePin.initVal );
}
void BSP_Board_Init ( void )
{
    BSP_Start_25MHz_Osc();
    /* Set PMIC Mode to "Run" */
    PMIC_ModeInit();
}

