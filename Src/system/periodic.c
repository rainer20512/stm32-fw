/**
 ******************************************************************************
 * @file    periodic.c
 * @author  Rainer
 * @brief   implementation for handling of all periodic items
 *          Periodic items occor at fixed seconds within a minute,
 *          at fixed minutes within an hour or at fixed hrs within one day
 ******************************************************************************
 */

#include "config/config.h"
#include "debug_helper.h"
#include "rtc.h"

#include "system/periodic.h"

#if DEBUG_MODE > 0 && DEBUG_PERIODIC > 0
    #define DBG_PUTS(a) DEBUG_PUTS(a)
#else
    #define DBG_PUTS(a)
#endif

/* Private define ------------------------------------------------------------*/
/* maximum size action list */
#define MAX_ACTION_LIST     32  /* max number of entries in action list       */
#define MAX_PERIODIC_SEC    15  /* max number of entries in seconds list      */
#define MAX_EVERY_SEC       10  /* max number of entries in every second list */
#define MAX_PERIODIC_MIN    10  /* max number of entries in mins list         */
#define MAX_PERIODIC_HR     4   /* max number of entries in hrs list          */

#define DEFAULT_EVERY_SECOND  0 /* dummy second enty for every second list    */

/* Private typedef -----------------------------------------------------------*/

/* An action list item consists of one Periodic function with one void * argument */
typedef struct {
    PeriodicFn func;    /* periodic function to call                         */
    void       *arg;    /* function argument                                 */
#if DEBUG_PERIODIC > 0
    const char *name;   /* explanational name, for debug purposes only       */
#endif
} ActionListItem;

/* An periodic list item consists of one "at"-value and one index into the
   action list */
typedef struct {
    uint8_t atValue;    /* "at" value the execution has to be triggered      */
    uint8_t actionIdx;  /* ptr into action list                              */
} PeriodicListItem;


/* Private variables ---------------------------------------------------------*/
ActionListItem action[MAX_ACTION_LIST];   /* list of all actions              */
PeriodicListItem atSec[MAX_PERIODIC_SEC]; /* period actions at certain seconds*/
PeriodicListItem everySec[MAX_EVERY_SEC]; /* period actions at every second   */
PeriodicListItem atMin[MAX_PERIODIC_MIN]; /* period actions at certain mins   */
PeriodicListItem atHr[MAX_PERIODIC_HR];   /* period actions at certain Hrs    */

uint32_t actionListNum  = 0; /* actual number of entries in action list       */
uint32_t periodicSecNum = 0; /* actual number of entries in seconds list      */
uint32_t everySecNum = 0;    /* actual number of e. in every second list      */
uint32_t periodicMinNum = 0; /* actual number of entries in minutes list      */
uint32_t periodicHrNum  = 0; /* actual number of entries in hrs list          */

/* Public functions -------------------------------------------------------- */


/******************************************************************************
 * Search the action list for one item. If found, the corresponding index wil
 * be returned. 
 * If not found, -1 will be returned
 *****************************************************************************/
static int32_t ActionListItemFind ( PeriodicFn fn, void *arg )
{
    for ( uint32_t i = 0; i < actionListNum; i++ ) 
        if ( action[i].func == fn && action[i].arg == arg ) return i;
        
    return -1;
}

/******************************************************************************
 * Add one Action list item and return the list index 
 * -1 is returned in case of error.
 * If the item is already contained in list ( fn and arg ), it will not be 
 * but the corresponding index will be returned
 *****************************************************************************/
#if DEBUG_PERIODIC > 0
    int32_t AddActionListItem  ( PeriodicFn fn, void *arg, const char *name)
#else
    int32_t AddActionListItemShort ( PeriodicFn fn, void *arg)
#endif
{
    int32_t ret;
    /* already in list? If yes, return index */
    if ( ret=ActionListItemFind(fn, arg), ret >= 0 ) return ret; 

    /* Add new element, if space left */
    if ( actionListNum >= MAX_ACTION_LIST ) {
        DBG_PUTS("Error: AddActionListItem: List full");
        return -1;
    }

    action[actionListNum].func = fn;
    action[actionListNum].arg  = arg;
#if DEBUG_PERIODIC > 0
    action[actionListNum].name = name;
#endif    
    return actionListNum++;
}

/******************************************************************************
 * Add one item to periodic list "list".
 * NOTE: this list must have been checked before to contain at least space for 
 * one more element
 *****************************************************************************/ 
static int32_t AddPeriodicListItem ( PeriodicListItem* list, uint32_t *actNum, uint8_t atValue, uint8_t actionIdx )
{
    list[*actNum].atValue   = atValue;
    list[*actNum].actionIdx = actionIdx;
    return (*actNum)++;
}

#if DEBUG_PERIODIC > 0
    int32_t AtSecond ( uint32_t sec, PeriodicFn fn, void *arg, const char *name)
#else
    int32_t AtSecondShort ( uint32_t sec, PeriodicFn fn, void *arg)
#endif
{
    if ( sec > 59 ) {
        DBG_PUTS("Error: AtSecond illegal Second");
        return -1;
    }

    /* if list is full, return with error */
    if ( periodicSecNum >= MAX_PERIODIC_SEC ) {
        DBG_PUTS("Error: AtSecond List full");
        return -1;
    }

    /* Add action item */
    #if DEBUG_PERIODIC > 0
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #else
        int32_t actionIdx = AddActionListItemShort(fn, arg);
    #endif

    /* If not successful return error */
    if ( actionIdx < 0 ) return -1;

    /* append to at list */
    return AddPeriodicListItem(atSec, &periodicSecNum, sec, actionIdx);
}

/*******************************************************************************
 * Things to do at certain seconds
 * \note As at second 0 or 30 a time correction may occur, don't use neither 
 *       the Milliseconds nor the millisecond timer at this seconds!
 ******************************************************************************/
 #if DEBUG_PERIODIC > 0
    int32_t EverySecond ( PeriodicFn fn, void *arg, const char *name)
#else
    int32_t EverySecondShort ( PeriodicFn fn, void *arg)
#endif
{
    /* if list is full, return with error */
    if ( everySecNum >= MAX_EVERY_SEC ) {
        DBG_PUTS("Error: EverySecond List full");
        return -1;
    }

    /* Add action item */
    #if DEBUG_PERIODIC > 0
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #else
        int32_t actionIdx = AddActionListItemShort(fn, arg);
    #endif

    /* If not successful return error */
    if ( actionIdx < 0 ) return -1;

    /* append to at list */
    return AddPeriodicListItem(everySec, &everySecNum, DEFAULT_EVERY_SECOND, actionIdx);
}



#if DEBUG_PERIODIC > 0
    int32_t AtMinute ( uint32_t min, PeriodicFn fn, void *arg, const char *name)
#else
    int32_t AtMinuteShort ( uint32_t min, PeriodicFn fn, void *arg)
#endif
{
    if ( min > 59 ) {
        DBG_PUTS("Error: AtMinute illegal minute");
        return -1;
    }

    /* if list is full, return with error */
    if ( periodicMinNum >= MAX_PERIODIC_MIN ) {
        DBG_PUTS("Error: AtMinute List full");
        return -1;
    }

    /* Add action item */
    #if DEBUG_PERIODIC > 0
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #else
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #endif

    /* If not successful return error */
    if ( actionIdx < 0 ) return -1;

    /* append to at list */
    return AddPeriodicListItem(atMin, &periodicMinNum, min, actionIdx);
    
}

#if DEBUG_PERIODIC > 0
    int32_t AtHour   ( uint32_t hr,  PeriodicFn fn, void *arg, const char *name)
#else
    int32_t AtHourShort   ( uint32_t hr,  PeriodicFn fn, void *arg)
#endif
{
    if ( hr > 23 ) {
        DBG_PUTS("Error: AtHour illegal hour");
        return -1;
    }

    /* if list is full, return with error */
    if ( periodicHrNum >= MAX_PERIODIC_HR ) {
        DBG_PUTS("Error: AtHour List full");
        return -1;
    }

    /* Add action item */
    #if DEBUG_PERIODIC > 0
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #else
        int32_t actionIdx = AddActionListItem(fn, arg, name);
    #endif

    /* If not successful return error */
    if ( actionIdx < 0 ) return -1;

    /* append to at list */
    return AddPeriodicListItem(atHr, &periodicHrNum, hr, actionIdx);
}

/******************************************************************************
 * iterate a list for a matching "at" value and -if matches- execute the 
 * associated action item
 *****************************************************************************/
static void CheckList ( PeriodicListItem *list, uint32_t cnt, uint8_t atValue )
{
    uint8_t actionIdx;
    /* For all list items */
    for ( uint32_t i = 0; i < cnt; i++ ) {
        /* if "at" value matches */
        if ( list[i].atValue  == atValue ) {
            /* execute associated action */
            actionIdx = list[i].actionIdx;
            action[actionIdx].func(action[actionIdx].arg);
        }
    }
}

static inline __attribute__((always_inline))
void DoPeriodicSec ( uint8_t sec )
{
    /* entries for specific second first */
    if ( periodicSecNum > 0 ) CheckList ( atSec,    periodicSecNum, sec );

    /* entries for every second thereafter */
    if ( everySecNum > 0 )    CheckList ( everySec, everySecNum,    DEFAULT_EVERY_SECOND);
}

static inline __attribute__((always_inline))
void DoPeriodicMin ( uint8_t min )
{
    if ( periodicMinNum > 0 ) CheckList ( atMin, periodicMinNum, min );
}

static inline __attribute__((always_inline))
void DoPeriodicHr ( uint8_t hr )
{
    if ( periodicHrNum > 0 )  CheckList ( atHr, periodicHrNum, hr );
}

/******************************************************************************
 * Do the periodic execution for the actual second.
 * if second == 0, do the periodc execution for the actual minute
 * if minute == 0 too, do the periodic execution for the actual hour
 *****************************************************************************/
void task_periodic ( uint32_t arg )
{
    UNUSED(arg);

    uint8_t ss = RTC_GetSecond();
    DoPeriodicSec(ss);
    if ( ss == 0 ) {
        uint8_t mm = RTC_GetMinute();
        DoPeriodicMin(mm);
        if ( mm == 0 ) {
            DoPeriodicHr(RTC_GetHour());
         }
     }
}

#if DEBUG_MODE > 0
void PeriodicDumpOneList ( PeriodicListItem *list, uint32_t used, char *numericTitle )
{
    uint32_t idx;
    if ( used == 0 ) {
        DBG_printf_indent("- no entries -\n");
        return;
    }
    if ( numericTitle ) 
        DBG_printf_indent("%s Description\n", numericTitle);
    for ( uint32_t i = 0; i < used; i++ ) {
        idx = list->actionIdx;
        if ( numericTitle )
            DBG_printf_indent("%3d %s\n", list->atValue, action[idx].name);
        else
            DBG_printf_indent(" o  %s\n", action[idx].name);
        list++;
    }

}


void PeriodicDumpList(void)
{
    DEBUG_PUTS  ("List of Periodic Actions -------------------------------------------------");

    int oldIndent = DBG_setIndentRel(+2);
    DBG_printf_indent("\nEvery Second List -----\n");
    DBG_setIndentRel(+2);
    PeriodicDumpOneList(everySec, everySecNum, NULL);
    DBG_setIndentRel(-2);
    DBG_printf_indent("\nAt Second List --------\n");
    DBG_setIndentRel(+2);
    PeriodicDumpOneList(atSec, periodicSecNum, "Sec");
    DBG_setIndentRel(-2);
    DBG_printf_indent("\nAt Minute List --------\n");
    DBG_setIndentRel(+2);
    PeriodicDumpOneList(atMin, periodicMinNum, "Min");
    DBG_setIndentRel(-2);
    DBG_printf_indent("\nAt Hour List ----------\n");
    DBG_setIndentRel(+2);
    PeriodicDumpOneList(atHr, periodicHrNum, "Hour");
    DBG_setIndentRel(-2);
 
    DBG_setIndentAbs(oldIndent);
}
    

#endif