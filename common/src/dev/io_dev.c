/*
 ******************************************************************************
 * @file    io_dev.h 
 * @author  Rainer
 * @brief   configure plain io ports, either as normal inputg or output ports
 *          Parameters as in  ..._hal_gpio.h
 *          in case of input, an EXTI iterrupt may be specified
 *          EXTI0 .. EXTI4 have individual interrupts (and priorities)
 *          whereas EXIT5..EXTI9 and EXT10..EXTI15 share one interrupt and
 *          one Interrupt prio. The resulting interrupt prio for these 
 *          shared interrupts is the interrupt prio of the last assigned
 *          input pin of the corresponding group EXTI5..EXTI9 and EXTI10..EXTI15 resp.
 *
 * @note:   ----------------------------------------------------
 *          to be performant, this file has to be compiled with 
 *          optimization on, even in DEBUG config
 *          ----------------------------------------------------
 *
 *****************************************************************************/

/** @addtogroup IO device
  * @{
  */

#include "config/config.h"
#include "config/gpio_config.h"
#include "error.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"
#include "dev/devices.h"
#include "dev/io_dev.h"

#include "system/exti_handler.h"
#include "debug_helper.h"



typedef struct {
    uint8_t led_num;       /* Number of elements in the following array */    
    uint8_t led_idx[];     /* Array of io pins, that are used as user LEDs */
} IO_AdditionalDataType;


/* Forward declarations -------------------------------------------------------------*/

static IO_AdditionalDataType * IO_GetAdditionalData(const HW_DeviceType *self)
{
    return (IO_AdditionalDataType *)(self->devData);
}



/* Private or driver functions ------------------------------------------------------*/

/*******************************************************************************************
 * Find the HW_Gpio_IO_Type record number "Idx" in sequence
 ******************************************************************************************/
static const HW_Gpio_IO_Type *IO_GetGpio(const HW_DeviceType *self, uint32_t idx)
{
    assert( idx < self->devGpioIO->num );
    return &self->devGpioIO->gpio[idx];
}

/*******************************************************************************************
 * Find the HW_Gpio_IO_Type record, that is associated with user LED no. "idx"
 ******************************************************************************************/
static const HW_Gpio_IO_Type *GetGpioFromLedIdx( uint8_t idx )
{
    #if DEBUG_MODE > 0
        if ( idx >= IO_GetAdditionalData(&HW_IO)->led_num ) {
            DEBUG_PUTS("ERROR LED index too big");
            return NULL;
        }
    #endif
    uint8_t pinidx = IO_GetAdditionalData(&HW_IO)->led_idx[idx];
    return IO_GetGpio(&HW_IO, pinidx);
}

typedef enum { 
    InputPin  = 0,
    OutputPin = 1,
    IrqPin    = 2,
} PinSearchEnum;

/*******************************************************************************************
 * Find the HW_Gpio_IO_Type record, that is associated with "pin" and is Input, Output ir IRQ pin
 ******************************************************************************************/
static const HW_Gpio_IO_Type *IO_FindPinInternal(const HW_DeviceType *self, uint16_t pin, PinSearchEnum SearchMode)
{
   assert ( self->devType == HW_DEVICE_IO );
   const HW_GpioList_IO *list = self->devGpioIO;
   for ( uint32_t i = 0; i < list->num; i++ ) 
     if ( list->gpio[i].pin == pin ) {
        switch ( SearchMode ) {
            case InputPin:
                /* check for input pin */
                if ( list->gpio[i].gpio_mode == GPIO_MODE_INPUT ) return &list->gpio[i];
                break;
            /* check for output pin */
            case OutputPin:
                if ( list->gpio[i].gpio_mode == GPIO_MODE_OUTPUT_PP || list->gpio[i].gpio_mode == GPIO_MODE_OUTPUT_OD ) return &list->gpio[i];
                break;
            /* check for output pin */
            case IrqPin:
                if ( HW_IsIrqMode(list->gpio[i].gpio_mode) ) return &list->gpio[i];
                break;
        }
     } 

   return NULL;
}

#define IO_FindInputPin(self, pin)          IO_FindPinInternal(self, pin, InputPin)
#define IO_FindOutputPin(self, pin)         IO_FindPinInternal(self, pin, OutputPin)
#define IO_FindIRQPin(self, pin)            IO_FindPinInternal(self, pin, IrqPin)

/*******************************************************************************************
 * Set an output pin to active High
 ******************************************************************************************/
static void OutputLow( const HW_Gpio_IO_Type *gpio )
{
    gpio->gpio->BSRR = (uint32_t)gpio->pin << 16;
}

/*******************************************************************************************
 * Set an output pin to active Low
 ******************************************************************************************/
static void OutputHigh( const HW_Gpio_IO_Type *gpio )
{
    gpio->gpio->BSRR = gpio->pin;
}

/*******************************************************************************************
 * Toggle an output pin
 ******************************************************************************************/
static void OutputToggle( const HW_Gpio_IO_Type *gpio )
{
    gpio->gpio->ODR ^= gpio->pin;
}



   
/* Public or driver functions --------------------------------------------------------*/
bool IO_Init(const HW_DeviceType *self)
{
    GpioIOInitAll(self->devGpioIO);
    return true;
}

void IO_DeInit(const HW_DeviceType *self)
{
    GpioIOInitAll(self->devGpioIO);
}

/*******************************************************************************************
 * Assign an EXIT-Interrupt to a pin configured as Input with interrupt before
 * Will fail, if not an input pin or not configured as Input with interrupt
 * If cb is NULL, callback will be deassigned
 ******************************************************************************************/
void IO_AssignInterrupt(uint16_t pin, ExtIrqCB cb )
{
    const HW_Gpio_IO_Type *gpio = IO_FindIRQPin(&HW_IO, pin);
    if ( gpio ) {
        if ( cb ) {
            Exti_Register_Callback(pin, &gpio->gpio->IDR, cb, NULL);    
        } else {
            Exti_UnRegister_Callback(pin);    
        }
    } else {
        /* No Interrupt or undefined IO pin */
        #if DEBUG_MODE > 0
            DEBUG_PRINTF("Cannot assign exti interrupt to no-interrupt pin %d\n", HW_GetIdxFromPin(pin));
        #endif
    }
}


/*******************************************************************************************
 * Get the number of configured user LEDs
 * Will fail, if not an input pin or not configured as Input with interrupt
 ******************************************************************************************/
uint8_t IO_UseLedGetNum( void )
{
    return IO_GetAdditionalData(&HW_IO)->led_num;
}

void IO_OutputHigh( uint16_t pinnum )
{
    const HW_Gpio_IO_Type *gpio = IO_FindOutputPin(&HW_IO, pinnum);
    #if DEBUG_MODE > 0 
        if ( !gpio || !HW_IsOutputMode ( gpio->gpio_mode ) ) {
            DEBUG_PUTS("Error: Illegal Pin number");
            return;
        }
    #endif
    assert(gpio);
    OutputHigh(gpio);
}

void IO_OutputLow ( uint16_t pinnum )
{
    const HW_Gpio_IO_Type *gpio = IO_FindOutputPin(&HW_IO, pinnum);
    #if DEBUG_MODE > 0 
        if ( !gpio || !HW_IsOutputMode ( gpio->gpio_mode ) ) {
            DEBUG_PUTS("Error: Illegal Pin number");
            return;
        }
    #endif
    assert(gpio);
    OutputLow(gpio);
}

void IO_OutputToggle ( uint16_t pinnum )
{
    const HW_Gpio_IO_Type *gpio = IO_FindOutputPin(&HW_IO, pinnum);
    #if DEBUG_MODE > 0 
        if ( !gpio || !HW_IsOutputMode ( gpio->gpio_mode ) ) {
            DEBUG_PUTS("Error: Illegal Pin number");
            return;
        }
    #endif
    assert(gpio);
    OutputToggle(gpio);
}



void IO_UserLedOn( uint8_t idx )
{
    const HW_Gpio_IO_Type *gpio = GetGpioFromLedIdx( idx );
    if ( gpio ) {
        if ( gpio->drv == HW_IO_NORMAL )
            OutputHigh(gpio);
        else
            OutputLow(gpio);
    }
}

void IO_UserLedOff( uint8_t idx )
{
    const HW_Gpio_IO_Type *gpio = GetGpioFromLedIdx( idx );
    if ( gpio ) {
        if ( gpio->drv == HW_IO_INVERTED )
            OutputHigh(gpio);
        else
            OutputLow(gpio);
    }
}

void IO_UserLedToggle ( uint8_t idx )
{
    const HW_Gpio_IO_Type *gpio = GetGpioFromLedIdx( idx );
    OutputToggle(gpio);
}

void IO_UserLedBlink (uint8_t idx, uint32_t toggles, uint32_t ms )
{
  const HW_Gpio_IO_Type *gpio = GetGpioFromLedIdx( idx );
  while(toggles-- > 0 )
  {
      OutputToggle(gpio);
      HAL_Delay(ms);
  }  
}

/* 
 * Currently, max 10 IO lines are supported
 * but that is only a configuration issue
 */
static const HW_GpioList_IO gpio_io = {
    .num  = IO_NUM,
    .gpio = { 
        #if defined(IO_00 )
            IO_00, 
        #endif
        #if defined(IO_01)
            IO_01,
        #endif
        #if defined(IO_02)
            IO_02,
        #endif
        #if defined(IO_03)
            IO_03,
        #endif
        #if defined(IO_04)
            IO_04,
        #endif
        #if defined(IO_05)
            IO_05,
        #endif
        #if defined(IO_06)
            IO_06
        #endif
        #if defined(IO_07)
            IO_07,
        #endif
        #if defined(IO_08)
            IO_08,
        #endif
        #if defined(IO_09)
            IO_09,
        #endif
     },
};

    static const IO_AdditionalDataType additional_io = {
        .led_num = USERLEDNUM,
        .led_idx = USERLEDS,
    };
    const HW_DeviceType HW_IO = {
        .devName        = "HW_IO",
        .devBase        = NULL,
        .devGpioAF      = NULL,
        .devGpioIO      = &gpio_io,
        .devType        = HW_DEVICE_IO,
        .devData        = &additional_io,
        .devIrqList     = NULL,
        .devDmaTx       = NULL,
        .devDmaRx       = NULL,
        .Init           = IO_Init,
        .DeInit         = IO_DeInit,
        .OnFrqChange    = NULL,
        .AllowStop      = NULL,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };




/**
  * @}
  */


