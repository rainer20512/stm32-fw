/**
  ******************************************************************************
  * @file    clockconfig_cm4.c
  * @author  Rainer
  * @brief   This is the clock configuration part of the CM4 core in a dual core
  *          environment. The only thing to do is to react on clock change event
  *          from master and then notify own devices an callback functions
  *          
  ******************************************************************************
  */


/** @addtogroup CLOCK_CONFIG
  * @{
  */

#include "config/config.h"
#include "system/clockconfig_cm4.h"
#include "dev/devices.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif
/*
 *************************************************************************************
 * All the stuff for clock change notification callback management
 * ( aside of callbacks in devices )
 ************************************************************************************/
#define MAX_CLKCHG_CB                4
static ClockChangeCB clkCB[MAX_CLKCHG_CB];/* Array of registered clk chng callbacks */
static int32_t numClkchangeCB = 0;      /* Number of "    "    "      "      "      */
void   ClockNotifyCallbacks(uint32_t);  /* forward declaration                      */

/******************************************************************************
 * @brief Register a clock change callback
 * @param changeCB callback function to be notified on clock changes
 * @returns 0  on success
 *          -1 ERR_: no more room to register callback
 *          -2 error: callback must no be NULL
 * @note Once registered a function, it can never be unregistered again
 *****************************************************************************/
int32_t ClockRegisterForClockChange ( ClockChangeCB changeCB )
{
    /* Space left in array ? */
    if ( numClkchangeCB >= MAX_CLKCHG_CB - 1 ) return -1;

    /* CB must not be NULL */
    if ( changeCB == 0 ) return -2;

    clkCB[numClkchangeCB++] = changeCB;
    return 0;
}

/******************************************************************************
 * @brief Notify all registered callbacks on Clock change
 * @param newclk  new clock frequncy in Hz
 *****************************************************************************/
void   ClockNotifyCallbacks(uint32_t newclk)
{
    for ( int32_t i = 0; i < numClkchangeCB; i++ ) 
        clkCB[i](newclk);
}


/******************************************************************************
 * @brief Execution of clok change action after notification from CM7 core
 *        via direct message
 *****************************************************************************/
void    ClockChangePerform ( void )
{
    /* Notify all devices */
    if ( DevicesInhibitFrqChange() ) {
        DEBUG_PUTS("Error: One or more CM4 devices disagreed to frq change!");
    }

    /* Notify all registered callbacks */
    ClockNotifyCallbacks(HAL_RCC_GetSysClockFreq());
}



/**
  * @}
  */


