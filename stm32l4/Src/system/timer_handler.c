/**
 ******************************************************************************
 * @file    timer_handler.c
 * @author  Rainer
 * @brief   Implements a list of input capture handlers for timers
 *          currently only used by quuadrature encoders
 *
 ******************************************************************************
 */

/** @addtogroup EXTI pin interrupt handling
  * @{
  */

#include "config/config.h"
#include "error.h"
#include "system/timer_handler.h"
#include "debug_helper.h"
#include "system/profiling.h"
#include "system/hw_util.h"

#define TIMER_CAPT_NUM    8      

typedef struct TmrBaseCaptureType {
    void         *hwBase;           /* TIM peripheral address */
    TimCaptureCB captureCB;         /* captureCB              */
} TmrBaseCaptureT;

static TmrBaseCaptureT tmr_capture_handlers[TIMER_CAPT_NUM];
static uint8_t tmr_capture_handlers_used = 0;


bool    Tim_Register_CaptureCB      ( void *hwBase, TimCaptureCB cb );
void    Tim_UnRegister_CaptureCB    ( void *hwBase );
bool    Tim_Has_CaptureCB           ( void *hwBase );


/* Private functions ----------------------------------------------------------------*/

/******************************************************************************
 * Find a Callback via associated peripheral timer address 
 * returns the array index in tmr_capture_handlers
 * if not found, TIMER_CAPT_NUM is returned
 *****************************************************************************/
static uint32_t Find_captureCB( void *hwBase )
{
    for ( uint8_t i = 0; i < tmr_capture_handlers_used; i++ )
        if ( tmr_capture_handlers[i].hwBase == hwBase ) return i;

    return TIMER_CAPT_NUM;
}

/******************************************************************************
 * Remove the "idx"-th entry by moving all following entries down by one
 *****************************************************************************/
static void Remove_captureCB( uint32_t idx )
{
    if (tmr_capture_handlers_used == 0 ) return;

    for ( uint8_t i=idx; i < tmr_capture_handlers_used-1; i++ )
        tmr_capture_handlers[i] = tmr_capture_handlers[i+1];
 
    tmr_capture_handlers_used--;
}

/* Public functions -----------------------------------------------------------------*/
/******************************************************************************
 * Register a timer capture callback for a specific timer. The timer is
 * identified by its peripheral address
 *****************************************************************************/
bool Tim_Register_CaptureCB( void *hwBase, TimCaptureCB cb )
{
    /* Look for already registered */
    uint32_t idx = Find_captureCB(hwBase);

    /* found none, then append new entry */
    if ( idx == TIMER_CAPT_NUM ) {
        /* Free entries left ? */
        if ( tmr_capture_handlers_used == TIMER_CAPT_NUM ) {
            DEBUG_PUTS("Error: Cannot register TmrCaptureCB");
            return false;
        }
        idx = tmr_capture_handlers_used++;
    }
    tmr_capture_handlers[idx].hwBase    = hwBase;
    tmr_capture_handlers[idx].captureCB = cb;
    return true;
}

/******************************************************************************
 * UnRegister a timer capture callback for a specific timer. The timer is
 * identified by its peripheral address
 *****************************************************************************/
void    Tim_UnRegister_CaptureCB    ( void *hwBase )
{
    /* Find Entry */
    uint32_t idx = Find_captureCB(hwBase);

    /* if nothing found, report error */
    if ( idx == TIMER_CAPT_NUM ) {
        DEBUG_PUTS("Error: Cannot unregister TmrCaptureCB");
        return;
    }

    Remove_captureCB( idx );
}

/******************************************************************************
 * returns true, if the timer identified by its peripheral addres
 * has an registered Callback
 *****************************************************************************/
bool    Tim_Has_CaptureCB           ( void *hwBase )
{
    return Find_captureCB(hwBase) < TIMER_CAPT_NUM;
}


/******************************************************************************
 * Interrupt Handler for Timer Capture
 *****************************************************************************/
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 {
    /* Look for registered Callback */
    uint32_t idx = Find_captureCB(htim->Instance);

    if ( idx < TIMER_CAPT_NUM ) {
        /* execute callback, if found */
        tmr_capture_handlers[idx].captureCB(htim);
    } else {
        DEBUG_PRINTF("No Capture Callback for Timer @%08x\n", htim->Instance);
    }
 }