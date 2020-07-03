/*
 ******************************************************************************
 * @file    hw_device.c
 * @author  Rainer
 * @brief   Abstract description of a hardware device
 *          Implementation for GPIO Init/DeInit
 *
 ******************************************************************************
 */

#include "config/config.h"
#include "dev/hw_device.h"
#include "system/hw_util.h"
#include "system/exti_handler.h"
#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif

/******************************************************************************
 * Initialize one Pin of GPIO "gpio" to the values Given in Init
 * If is's a special pin, which has to be enabled explitely in PWR registers,
 * this will be done
 * Special pins and there associated bits in PWR registers are
 *    GPIOG Bit 2 .. 15 -> requires PWR-CR.IOSV to be set
 *    GPIOA BIT 9 .. 12 -> requires PWR-CR.USV to be set
 *****************************************************************************/
void GpioInitHW( GPIO_TypeDef *gpio, GPIO_InitTypeDef *Init )
{
    #if defined( PWR_CR2_IOSV )
        /* check for IOSV */
        if ( gpio == GPIOG && Init->Pin > GPIO_PIN_1 ) {

            /* Get clock status of PWR domain and switch on, if not already on*/
            uint32_t pwrbit = __HAL_RCC_PWR_IS_CLK_ENABLED();
            if ( !pwrbit ) __HAL_RCC_PWR_CLK_ENABLE();

            /* Check IOSV Bit and set */
            if ( !READ_BIT(PWR->CR2, PWR_CR2_IOSV) )SET_BIT(PWR->CR2, PWR_CR2_IOSV); 

            /* Switch PWR domain clock off again, if it was off before */
            if ( !pwrbit ) __HAL_RCC_PWR_CLK_DISABLE();
        }
    #endif

    /* Set hardware Pin */
    HAL_GPIO_Init(gpio, Init);
}

/******************************************************************************
 * Initialize one Pin to the values Given in Init
 * Init has to be properly initialized before call. 
 * I.e. Fields Mode, and Speed have to be set in Init upon call 
 * Only Pin Number, Pull and AF code are set by this function according to gpio field
 * values. The pullup is set, the output value isn't. Use "GpioAFInitOneWithPreset"
 * to set the output value, too.
 *****************************************************************************/
void GpioAFInitOne( const HW_Gpio_AF_Type *gpio, GPIO_InitTypeDef *Init )
{
 
    HW_SetHWClock ( gpio->gpio, 1 );
    Init->Pin       = gpio->pin;
    Init->Alternate = gpio->af_mode;
    Init->Pull      = gpio->pull;
   GpioInitHW(gpio->gpio, Init);
}

/******************************************************************************
 * Same as "GpioAFInitOne" but outputs are preset to preset value
 * Init has to be properly initialized before call. 
 * I.e. Fields Mode, Pull and Speed have to be set in Init upon call 
 * Only Pin Number and AF code are set by this function according to gpio field
 * values
 *****************************************************************************/
void GpioAFInitOneWithPreset( const HW_Gpio_AF_Type *gpio, GPIO_InitTypeDef *Init, HW_PinOutInitial preset )
{
 
    HW_SetHWClock ( gpio->gpio, 1 );
    Init->Pin       = gpio->pin;
    Init->Alternate = gpio->af_mode;
    Init->Pull      = gpio->pull;
    switch(preset) {
        case HW_OUTPUT_LOW:
            /* Set output pin low */
            gpio->gpio->BSRR = gpio->pin << 16;
            break;
        case HW_OUTPUT_HIGH:
            /* Set output pin high */
            gpio->gpio->BSRR = gpio->pin;
            break;
        default:
            /* do nothing */
            ;
    }
   GpioInitHW(gpio->gpio, Init);
}

void GpioAFInitAll   ( const HW_GpioList_AF *gpioList, GPIO_InitTypeDef *Init)
{
   for ( uint32_t i=0; i < gpioList->num; i++ )
       GpioAFInitOne(&gpioList->gpio[i], Init);
}

void GpioAFDeInitOne   ( const HW_Gpio_AF_Type *gpio )
{
        HAL_GPIO_DeInit(gpio->gpio, gpio->pin);
}


void GpioAFDeInitAll ( const HW_GpioList_AF *gpioList )
{
    for ( uint32_t i=0; i < gpioList->num; i++ )
        GpioAFDeInitOne(&gpioList->gpio[i]);
}

/******************************************************************************
 * Enable or disable all associated interrupts of a device 
 *****************************************************************************/
void HW_SetAllIRQs(const HW_IrqList *irqlist, bool bDoEna)
{
    const HW_IrqType *irq;
    for ( uint8_t i = 0; i < irqlist->num; i++ ) {
        irq = &irqlist->irq[i];
        if ( bDoEna ) {
            HAL_NVIC_SetPriority(irq->irq_num, irq->irq_prio, irq->irq_subprio);
            HAL_NVIC_EnableIRQ(irq->irq_num);
        } else { 
            HAL_NVIC_DisableIRQ(irq->irq_num);
        }
    }
}




/******************************************************************************
 * Returns true, if the pin mode is one that may trigger an interrupt
 *****************************************************************************/
bool HW_IsIrqMode ( uint32_t mode )
{
    return    mode == GPIO_MODE_IT_RISING
           || mode == GPIO_MODE_IT_FALLING          
           || mode == GPIO_MODE_IT_RISING_FALLING;
}
/******************************************************************************
 * Returns true, if the pin mode is one that may trigger an event
 *****************************************************************************/
bool HW_IsEvtMode ( uint32_t mode )
{
    return    mode == GPIO_MODE_EVT_RISING
           || mode == GPIO_MODE_EVT_FALLING          
           || mode == GPIO_MODE_EVT_RISING_FALLING;
}

/******************************************************************************
 * Returns true, if the pin mode is any of the input modes
 *****************************************************************************/
bool HW_IsInputMode ( uint32_t mode )
{
    return  mode == GPIO_MODE_INPUT
         || HW_IsIrqMode(mode)
         || HW_IsEvtMode(mode);
}

/******************************************************************************
 * Returns true, if the pin mode is any of the output modes
 *****************************************************************************/
bool HW_IsOutputMode ( uint32_t mode )
{
    return  mode == GPIO_MODE_OUTPUT_PP 
         || mode == GPIO_MODE_OUTPUT_OD;
}


/******************************************************************************
 * The same for GPIO pins in standard input or output mode
 * Alle GPIO_InitTypeDef-structure members are defined in the corresponding
 * gpio-structure.
 *****************************************************************************/
static void GpioIOInitOne(const HW_Gpio_IO_Type *gpio)
{
    GPIO_InitTypeDef Init;

    Init.Pin    = gpio->pin;
    Init.Mode   = gpio->gpio_mode;
    Init.Speed  = gpio->speed;
    Init.Pull   = gpio->pull;

    HW_SetHWClock ( gpio->gpio, 1 );

    /* Set Output Pin to predefined value BEFORE configuring as output */
    if ( gpio->initial != HW_INPUT )
        HAL_GPIO_WritePin(gpio->gpio, gpio->pin, ( gpio->initial == HW_OUTPUT_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET));
   GpioInitHW( gpio->gpio, &Init);

    /* Enable Interrupt, if Input and Interrupt mode configured and interrupt configured*/
    const GPIO_IrqType *irq = &gpio->gpio_irq;
    if(HW_IsIrqMode(gpio->gpio_mode)) {
        if ( irq->irq_prio >= 0 ) {
            uint16_t irq_num=ExtiGetIrqNumFromPin(gpio->pin);
            HAL_NVIC_SetPriority(irq_num, irq->irq_prio, irq->irq_subprio);
            HAL_NVIC_EnableIRQ(irq_num);
        }
    } else {
        // Pin has no interrupt configuration, but interrupt is specified: Issue warning
        #if DEBUG_MODE > 0
            if ( irq->irq_prio >= 0 ) {
                DEBUG_PRINTF("Warning: Pin %d of GPIO%c has no interrupt capability, interrupt ignored\n", HW_GetIdxFromPin(gpio->pin),HW_GetGPIOLetter(gpio->gpio) );
            }
        #endif
    }
}

void GpioIOInitAll ( const HW_GpioList_IO *gpioList )
{
   for ( uint32_t i=0; i < gpioList->num; i++ )
        GpioIOInitOne(&gpioList->gpio[i]);
}

static void GpioIODeInitOne   ( const HW_Gpio_IO_Type *gpio )
{
    HAL_GPIO_DeInit(gpio->gpio, gpio->pin);

    /* Disable Interrupt, if Input and Interrupt mode configured and interrupt configured */
    /* and interrupt is not shared between more than one pin ( PIN5 .. PIN15 have shared interrupts ) */
    if(HW_IsIrqMode(gpio->gpio_mode)) {
        const GPIO_IrqType *irq = &gpio->gpio_irq;
        if ( irq->irq_prio >= 0 ) {
            HAL_NVIC_DisableIRQ( ExtiGetIrqNumFromPin(gpio->pin) );
        }
    }
     HAL_GPIO_DeInit(gpio->gpio, gpio->pin);
}

void GpioIODeInitAll ( const HW_GpioList_IO *gpioList )
{
    for ( uint32_t i=0; i < gpioList->num; i++ )
        GpioIODeInitOne(&gpioList->gpio[i]);
}



/**************************************************************************************************
 * Analog ADC input pins
 *************************************************************************************************/
void GpioADCInitAll ( const HW_GpioList_ADC *gpioList )
{
    const HW_Gpio_ADC_Type *gpio;

    GPIO_InitTypeDef Init= {0};
    #if defined(GPIO_MODE_ANALOG_ADC_CONTROL)
        Init.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    #else
        Init.Mode = GPIO_MODE_ANALOG;
    #endif
    Init.Pull     = GPIO_NOPULL;

    for ( uint32_t i=0; i < gpioList->num; i++ ) {
        gpio = &gpioList->gpio[i];
        if ( gpio->gpio ) {
            /* Exclude internal channels, they have gpio==NULL */
            HW_SetHWClock ( gpio->gpio, 1 );
            Init.Pin = gpio->pin;
           GpioInitHW( gpio->gpio, &Init);
        }
    }
}


void GpioADCDeInitAll ( const HW_GpioList_ADC *gpioList )
{
    const HW_Gpio_ADC_Type *gpio;

    for ( uint32_t i=0; i < gpioList->num; i++ ) {
        gpio = &gpioList->gpio[i];
        if ( gpio->gpio ) {
            /* Exclude internal channels, they have gpio==NULL */
            HAL_GPIO_DeInit( gpio->gpio, gpio->pin);
        }
    }
}
