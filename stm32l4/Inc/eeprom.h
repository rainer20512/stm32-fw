/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  Rainer
  * @brief   eeprom or simulated eeprom access functions
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#include "config/config.h"

#if USE_PULSE_SEQUENCER > 0
    #include "sequencer/analyzer.h"
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************
 * Allowed types of persistent config parameters
 *****************************************************************************/
typedef enum {
    EEType_OnOff    = 0,
    EEType_YesNo,
    EEType_Uint8_Dec,
    EEType_Uint8_Hex,
    EEType_Uint16_Dec,
    EEType_Uint16_Hex,
    EEType_Uint32_Dec,
    EEType_Uint32_Hex,
    EEType_String,
}EETypeT;

/******************************************************************************
 * A type to specify default value and minimal/maximal allowed value for an
 * eeprom item
 *****************************************************************************/
typedef struct EELT {
  uint8_t deflt;
  uint8_t min;
  uint8_t max;
  uint8_t type;
  const char *help;
} EE_LimitsT;

extern const EE_LimitsT eelimits[];

/******************************************************************************
 * Define the limits for every single eeprom item
 *****************************************************************************/
    #define EELIMITS01 \
    /* 00 */ {                 0,   0,   1, EEType_YesNo,       "Reset System\nat next hh:mm:10" },                        \
    /* 01 */ {                 5,   1,  30, EEType_Uint8_Dec,   "Radio Timeout [min]" },                                        
#if USE_DS18X20 > 0
    #define EELIMITS02  EELIMITS01 \
    /* 02 */ {                30,  10, 255, EEType_Uint8_Dec,   "Interval [s] of\ntemperature measurement" },               
#elif USE_RFM_OOK > 0
    #define EELIMITS02  EELIMITS01 \
    /* 02 */ {                 1,   0,   1, EEType_OnOff,       "OOK-Mode (0=off, 1=On)" },               
#else
    #define EELIMITS02  EELIMITS01 \
    /* 02 */ {                 0,   0, 255, EEType_Uint8_Dec,   "Unused 02" },                                             
#endif
    #define EELIMITS03  EELIMITS02 \
    /* 03 */ {                 1,   0,   1, EEType_OnOff,       "FSK-Mode (0=off, 1=On)" },                                \
    /* 04 */ {                 5,   1,  60, EEType_Uint8_Dec,   "Send data interval [min]" },                              \
    /* 05 */ {                 1,   0,   1, EEType_OnOff,       "Dump detailled\nTimestamps (0=off, 1=On)" },              \
    /* 06 */ {                 1,   1,   9, EEType_Uint8_Dec,   "Debug-Level, reqires\nDEBUG_MODE > 0" },                  \
    /* 07 */ {                 9,   0, 255, EEType_Uint8_Dec,   "Correction value to correct\nlocal pressure to MSL" },    \
    /* 08 */ {                 1,   0,  31, EEType_Uint8_Dec,   "Backlight intensity" },                                   \
    /* 09 */ {                 1,   0,   4, EEType_Uint8_Dec,   "LCD display scheme to use" },                             \
    /* 0a */ {                 5,   1,  30, EEType_Uint8_Dec,   "Backlight on time [s]" },                                 \
    /* 0b */ {                 0,   0, 255, EEType_Uint8_Dec,   "LCD on time [s]" },                                       \
    /* 0c */ {               125,  80, 160, EEType_Uint8_Dec,   "threshold for battery\nwarning [unit 0.02V]" },           \
    /* 0d */ {               115,  80, 160, EEType_Uint8_Dec,   "threshold for battery\nLOW [unit 0.02V]" },                
#if defined(TX18LISTENER)
    #define EELIMITS0E EELIMITS03 \
    /* 0e */ {    TX18_SKIP_INIT,   0,  32, EEType_Uint8_Dec,   "OOK-Skips in init mode" },                                \
    /* 0f */ {    TX18_SKIP_NORM,   0,   9, EEType_Uint8_Dec,   "OOK-Skips in normal mode" },               
#else
    #define EELIMITS0E EELIMITS03 \
    /* 0e */ {                 0,   0, 255, EEType_Uint8_Dec,   "Unused 0e" },                                             \
    /* 0f */ {                 0,   0, 255, EEType_Uint8_Dec,   "Unused 0f" },                                             
#endif
    #define EELIMITS10 EELIMITS0E \
    /* 10 */ {                28,   1,  28, EEType_Uint8_Hex,   "Device Address" },                                        \
    /* 11 */ {                01,  00, 255, EEType_Uint8_Hex,   "Security Key[0]" },                                       \
    /* 12 */ {              0x23,  00, 255, EEType_Uint8_Hex,   "Security Key[1]" },                                       \
    /* 13 */ {              0x45,  00, 255, EEType_Uint8_Hex,   "Security Key[2]" },                                       \
    /* 14 */ {              0x67,  00, 255, EEType_Uint8_Hex,   "Security Key[3]" },                                       \
    /* 15 */ {              0x89,  00, 255, EEType_Uint8_Hex,   "Security Key[4]" },                                       \
    /* 16 */ {              0xab,  00, 255, EEType_Uint8_Hex,   "Security Key[5]" },                                       \
    /* 17 */ {              0xcd,  00, 255, EEType_Uint8_Hex,   "Security Key[6]" },                                       \
    /* 18 */ {              0xef,  00, 255, EEType_Uint8_Hex,   "Security Key[7]" },                                       \
    /* 19 */ {                 0,   0,   1, EEType_YesNo,       "Enable Periodic Dump\nof values to ext. EEPROM" },        \
    /* 1a */ {                 1,   0,   1, EEType_YesNo,       "Led as Sleep\nIndicator (0=no, 1=yes)" },                 \
    /* 1b */ {                 0,   0,   1, EEType_YesNo,       "Allow StopMode\n(0=no, 1=yes)" },                         \
    /* 1c */ { DEFAULT_STOP_MODE,   0,   2, EEType_Uint8_Dec,   "StopMode to enter on stop" },                             \
    /* 1d */ { USER_CLOCKCONFIG,    0,  25, EEType_Uint8_Dec,   "Clock configuration to use" },                            
#if defined(TX18LISTENER)
    #define EELIMITS1E EELIMITS10 \
    /* 1e */ {        TX_OOK_FRQ,   0,  1, EEType_Uint8_Dec,   "OOK-Frequency\n0=434.000, 1=433.850" },                    
#else
    #define EELIMITS1E EELIMITS10 \
    /* 1e */ {                 0,   0, 255, EEType_Uint8_Dec,   "Unused 1e" },                                             
#endif
    #define EELIMITS1F EELIMITS1E \
    /* 1f */ {                 0,   0, 255, EEType_Uint8_Dec,   "Unused 1f" },                                             \

#define EELIMITS { EELIMITS1F }

#define EEPROM_FORCE_RESET_IDX		0				// Index of the reset flag
#define EEPROM_DISPLAY_SCHEME_IDX       9                               // Index of display scheme

/******************************************************************************
 * all persistent setting items in RAM
 *****************************************************************************/
typedef struct { // each variables must be uint8_t or int8_t without exception
    /* 00 */ uint8_t ForceReset;	  //!< Reset System at next hh:mm:10 - MUST BE the FIRST ELEMENT!
    /* 01 */ uint8_t timeout;  		  //!< HR20-Master Timeout <min>
#if USE_DS18X20 > 0
    /* 02 */ uint8_t temp_interval;       //!< Interval [s] of temperature measurement
#elif USE_RFM_OOK > 0
    /* 02 */ uint8_t OOK_mode;		  //!< OOK-Mode (0=off, 1=TX18@433Mhz, 
#else
    /* 02 */ uint8_t unused02;		  //!< not used
#endif
    /* 03 */ uint8_t FSK_mode; 		  //!< FSK-Mode (0=off, 1=HR20-like,
    /* 04 */ uint8_t TransmitPeriod;      //!< Interval <min> in which sensor values are transmitted to master
    /* 05 */ uint8_t dbg_timestamps;      //!< Only in Debugmode: Dump detailled Timestamps every second
    /* 06 */ uint8_t dbg_level;  	  //!< Debug-Level, only used if compiled with DEBUG_MODE > 0
    /* 07 */ uint8_t PressureComp;  	  //!< Correction value to correct local pressure value to MSL
    /* 08 */ uint8_t def_intensity;       //!< Backlight intensity, 0..1F
    /* 09 */ uint8_t displayScheme;       //!< LCD display scheme to USE
    /* 0a */ uint8_t bklightOnTIme;       //!< Backlight on time [s] 
    /* 0b */ uint8_t lcdOnTime;           //!< Backlight on time [s] 
    /* 0c */ uint8_t bat_warning_thld;    //!< threshold for battery warning [unit 0.02V]=[unit 0.01V per cell]
    /* 0d */ uint8_t bat_low_thld;        //!< threshold for battery low [unit 0.02V]=[unit 0.01V per cell]
#if defined(TX18LISTENER)
    /* 0e */ uint8_t tx18_skip_init;      //!< how many transmissions will be skipped in intial phase
    /* 0f */ uint8_t tx18_skip_norm;       //!< how many transmissions will be skipped in normal phase
#else
    /* 0e */ uint8_t unused_0e;           //!< Unused
    /* 0f */ uint8_t unused_0f;           //!< Unused
#endif
    /* 10 */ uint8_t RFM_devaddr;	  //!< HR20's own device address in RFM radio networking. =0 mean disable radio
    /* 11...18 */ uint8_t security_key[8];//!< key for encrypted radio messasges
    /* 19 */ uint8_t doDump;              //!< Enable Periodic Dump of values to ext. EEPROM ( 0 = no, 1 = yes )
    /* 1a */ uint8_t SleepIndicator;      //!< Use Led as Sleep Indicator ( 0 = no, 1 = yes )
    /* 1b */ uint8_t allow_Stop;          //!< Allow Stop-Mode
    /* 1c */ uint8_t StopMode;            //!< Stop Mode to enter on stop 
    /* 1d */ uint8_t clk_config;          //!< clock configuration to use
#if defined(TX18LISTENER)
    /* 1e */ uint8_t ook_frq;             //!< ook frequency select
#else
    /* 1e */ uint8_t unused_1e;           //!< Unused
#endif
    /* 1f */ uint8_t unused_1f;           //!< Unused
} EE_ConfigT;


#if defined(STROMSENSOR)
	#define EE_LAYOUT (0xD2) //!< EEPROM layout version (Stromsensor)
#elif defined(GASSENSOR) 
	#define EE_LAYOUT (0xD1) //!< EEPROM layout version (Gassensor)
#elif defined(UNIVERSAL) || defined(NOEXTENSION) || defined(ENVIRONMENTAL)
	#define EE_LAYOUT (0xD0) //!< EEPROM layout version (Universal 0) 
#elif defined(TX18LISTENER)
	#define EE_LAYOUT (0xA0) //!< EEPROM layout version (Außen 0)
#else
	#error "No EEPROM Layout defined"
#endif

extern EE_ConfigT config;

void     Config_Init  (void);
void     Config_Dump  (void);
bool     Config_SetVal(uint8_t idx, uint8_t newval);
uint8_t  Config_GetVal(uint8_t idx);
uint32_t Config_GetCnt(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __EEPROM_H */
