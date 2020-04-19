/**
 ******************************************************************************
 * @file    exti_handler.c
 * @author  Rainer
 * @brief   Do all the handling of EXTI pin change interrupts. 
 *          As the lines 5-9 and 10-15 share one interrupt, we must find out
 *          manually, which line generated the interrupt.
 *
 *          Internally maximum one Interrupt handler may be assigned to each
 *          of the 16 exti pin interrupt lines.
 *
 *          Limitations: It is not possible to handle or distuingish interrupts
 *          from the same pin number of different ports
 ******************************************************************************
 */

/** @addtogroup EXTI pin interrupt handling
  * @{
  */

#include "config/config.h"
#include "error.h"
#include "system/exti_handler.h"
#include "debug_helper.h"
#include "system/profiling.h"
#include "system/hw_util.h"

static ExtIrqCB        handlers[EXTI_LINENUM];
static __IO uint32_t * idr     [EXTI_LINENUM];
static void *          fnArgs  [EXTI_LINENUM];

bool Exti_Register_Callback     ( uint16_t GPIO_Pin, __IO uint32_t *gpioidr, ExtIrqCB cb, void *arg )
{
    /* Must specify a callback */
    assert(cb);

    uint16_t line = HW_GetIdxFromPin(GPIO_Pin);
    /* Must specify a valid line */
    assert ( line < EXTI_LINENUM);

    /* Error if handler already set */
    if ( handlers[line] != NULL ) {
        DEBUG_PRINTF("Error: Exti-Handler for line %d already set\n", line );
        return false;
    }

    handlers[line] = cb;
    idr[line]      = gpioidr;
    fnArgs[line]   =arg;
    return true;
}

void Exti_UnRegister_Callback   ( uint16_t GPIO_Pin )
{
    uint16_t line = HW_GetIdxFromPin(GPIO_Pin);

    /* Must specify a valid line */
    assert ( line < EXTI_LINENUM);

    handlers[line] = NULL;
}

/*******************************************************************************************************
 * returuns true, if "line" is a valid line and has an Interrupt handler assigned
 ******************************************************************************************************/
bool Exti_Has_Callback( uint16_t line )
{
    return line < EXTI_LINENUM && handlers[line];
}

#define EXTI_15_10_MASK 0b1111110000000000
#define EXTI_9_5_MASK   0b0000001111100000
#define EXTI_15_10_MSB  0b1000000000000000
#define EXTI_15_10_LSB  0b0000010000000000

#define EXTI_9_5_MASK   0b0000001111100000
#define EXTI_9_5_MSB    0b0000001000000000
#define EXTI_9_5_LSB    0b0000000000100000

/*******************************************************************************************************
 * returns the EXTI Interrupt number for 'pin'
 * -1 if not defined
 ******************************************************************************************************/
int16_t ExtiGetIrqNumFromPin ( uint16_t pin )
{
   
    if ( pin >= EXTI_15_10_LSB && pin <= EXTI_15_10_MSB  ) return EXTI15_10_IRQn;
    if ( pin >= EXTI_9_5_LSB && pin <= EXTI_9_5_MSB  ) return EXTI9_5_IRQn;

    uint8_t pinidx = HW_GetIdxFromPin(pin);
    switch ( pinidx ) {
        case 0: return EXTI0_IRQn;
        case 1: return EXTI1_IRQn;
        case 2: return EXTI2_IRQn;
        case 3: return EXTI3_IRQn;
        case 4: return EXTI4_IRQn;
       default: 
            #if DEBUG_MODE > 0
                DEBUG_PRINTF("Error: No EXTI_IRQ for pin %d\n",HW_GetIdxFromPin(pin) );
            #endif
            return -1;
    }
}

/*******************************************************************************************************
 * configure exti interrupt for pin "pin" of GPIO port "gpio"
 * The interrupt is configured, but not enabled yet !
 ******************************************************************************************************/
void Exti_ConfigIrq( GPIO_TypeDef *gpio, uint16_t pin, uint32_t extiTrigger )
{
    uint32_t gpioidx;       /* Index of GPIO structure (0..9) */
    uint32_t pinidx;        /* index of pin ( 0..15) */
    uint32_t regval;
    /* configure EXTI for rising and/or falling edge */
    if ( extiTrigger &  EXTI_TRIGGER_RISING ) {
        SET_BIT(EXTI->RTSR1, (uint32_t)pin);
    } else { 
        CLEAR_BIT(EXTI->RTSR1, (uint32_t)pin);
    }
    if ( extiTrigger &  EXTI_TRIGGER_FALLING ) {
        SET_BIT(EXTI->FTSR1, (uint32_t)pin);
    } else { 
        CLEAR_BIT(EXTI->FTSR1, (uint32_t)pin);
    }

    /* Get index of GPIO structure */
    gpioidx = HW_GetGPIOIdx(gpio);
    pinidx =  HW_GetIdxFromPin(pin);

    /* Read actual content of SYSCFG->EXTICR */
    regval = SYSCFG->EXTICR[pinidx >> 2u];

    /* Clear all corresponding bits */
    regval &= ~(SYSCFG_EXTICR1_EXTI0 << (SYSCFG_EXTICR1_EXTI1_Pos * (pinidx & 0x03u)));
    
    /* Select to correct GPIO port and write back */
    regval |= gpioidx << (SYSCFG_EXTICR1_EXTI1_Pos * (pinidx & 0x03u));
    SYSCFG->EXTICR[pinidx >> 2u] = regval;

}

/*******************************************************************************************************
 * Disable one pin change interrupt
 * Use only for temporarily disable, e.g. debounce of keys
 * returns the old activation status
 ******************************************************************************************************/
uint16_t Exti_DisableIrq( uint16_t pin )
{
    uint16_t ret = EXTI->IMR1 & pin;
    CLEAR_BIT(EXTI->IMR1, (uint32_t)pin );
    return ret;
}

/*******************************************************************************************************
 * Re-enable a previously disabled pin change interrupt
 ******************************************************************************************************/
void Exti_EnableIrq( uint16_t pin )
{
    /* clear pending interrupts */
    EXTI->PR1 = pin;
    
    /* Enable interrupts */
    SET_BIT(EXTI->IMR1, (uint32_t)pin );

}


/*
 *******************************************************************************************************
 * Interrupt handlers 
 *******************************************************************************************************
 */

/**
  * @brief  Handle EXTI interrupt request.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
static void EXTI_HandleIRQ(uint16_t GPIO_Pin)
{
  // DEBUG_PRINTF("h%02d", HW_GetIdxFromPin(GPIO_Pin));
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != 0x00u) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    uint16_t line = HW_GetIdxFromPin(GPIO_Pin);
    uint16_t pinvalue = *idr[line] & GPIO_Pin;
    // DEBUG_PRINTF("H%02d", line);
    if ( handlers[line] ) handlers[line](GPIO_Pin, pinvalue, fnArgs[line]);
  }
}

/*******************************************************************************************************
 * @brief  This function handles External External lines 10 to 15 interrupt request.
 * @param  None
 * @retval None
 ******************************************************************************************************/
void EXTI15_10_IRQHandler(void)
{
  ProfilerPush(JOB_IRQ_EXTI);
  /* Get the Interrupt source */
  uint32_t src = EXTI->PR1 & EXTI_15_10_MASK; 
  
  /* And for every single bit call the HAL Handler ( which will call the user callback for every bit ) */
  uint32_t position = EXTI_15_10_MSB;
  while (1) {
    if ( position & src ) EXTI_HandleIRQ(position);
    if ( position == EXTI_15_10_LSB ) break;
    position >>= 1;
  }
  ProfilerPop();
}

/*******************************************************************************************************
 * @brief  This function handles External External lines 5 to 9 interrupt request.
 * @param  None
 * @retval None
 ******************************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
  ProfilerPush(JOB_IRQ_EXTI);
  /* Get the Interrupt source */
  uint32_t src = EXTI->PR1 & EXTI_9_5_MASK; 
  
  /* And for every single bit call the HAL Handler ( which will call the user callback for every bit ) */
  uint32_t position = EXTI_9_5_MSB;
  while (1) {
    if ( position & src ) EXTI_HandleIRQ(position);
    if ( position == EXTI_9_5_LSB ) break;
    position >>= 1;
  }
  ProfilerPop();
}

/*******************************************************************************************************
 * @brief  Generate the EXTI IRQ handlers for Line 0 .. 4 by Macro
 * @param  None
 * @retval None
 ******************************************************************************************************/
#define EXTI_IRQ(num)               \
void EXTI##num##_IRQHandler(void)   \
{                                   \
  uint16_t pin  = GPIO_PIN_##num;   \
  ProfilerPush(JOB_IRQ_EXTI);       \
  EXTI_HandleIRQ(pin);              \
  ProfilerPop();                    \
}

//RHB todo EXTI_IRQ(0)
//RHB todo EXTI_IRQ(1)
EXTI_IRQ(2)
EXTI_IRQ(3)
EXTI_IRQ(4)

/**
  * @}
  */

