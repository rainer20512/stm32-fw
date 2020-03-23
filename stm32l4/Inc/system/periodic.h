/**
 ******************************************************************************
 * @file    periodic.h
 * @author  Rainer
 * @brief   implementation for handling of all periodic items
 *          Periodic items occor at fixed seconds within a minute,
 *          at fixed minutes within an hour or at fixed hrs within one day
 ******************************************************************************
 */

#ifndef __PERIODIC_H
#define __PERIODIC_H

#include "config/config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Mandatory type of all periodic functions */
typedef void ( *PeriodicFn ) ( void *arg );


/* Add one Action list item and return the list index 
 * -1 is returned in case of error.
 * If the item is already contained in list, it will not */
#if DEBUG_PERIODIC > 0
    int32_t AddActionListItem  ( PeriodicFn, void *, const char *);
    int32_t AtSecond    ( uint32_t sec, PeriodicFn, void *, const char *);
    int32_t EverySecond (               PeriodicFn, void *, const char *); 
    int32_t AtMinute    ( uint32_t min, PeriodicFn, void *, const char *);
    int32_t AtHour      ( uint32_t hr,  PeriodicFn, void *, const char *);
#else
    int32_t AddActionListItemShort ( PeriodicFn, void *);
    int32_t AtSecondShort    ( uint32_t sec, PeriodicFn, void *);
    int32_t EverySecondShort (               PeriodicFn, void *); 
    int32_t AtMinuteShort    ( uint32_t min, PeriodicFn, void *);
    int32_t AtHourShort      ( uint32_t hr,  PeriodicFn, void *);
    #define AddActionListItem(a,b,c)     AddActionListItemShort(a,b)
    #define AtSecond(a,b,c,d)            AddSecondShort(a,b,c)
    #define EverySecond(b,c,d)           EverySecondShort(b,c)
    #define AtMinute(a,b,c,d)            AddMinuteShort(a,b,c)
    #define AtHour(a,b,c,d)              AddHourShort(a,b,c)
#endif

void task_periodic ( uint32_t arg );

#if DEBUG_MODE > 0 
    void PeriodicDumpList(void);
#else
    #define PeriodicDumpList()
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PERIODIC_H */

