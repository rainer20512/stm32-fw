
/**
 ******************************************************************************
 * @file    "minitask.c" 
 * @author  Rainer
 * @brief   Primitive Task functions
 * 
 * Tasking works only cooperative, i.e. any task will be executed only if
 * the currently running task will suspend itself by returning from its 
 * run-function 
 *
 * Moreover, all tasks share one stack and the whole RAM contents
 *
 ******************************************************************************
*/

#include "config/config.h"
#include "debug_helper.h"
#include "task/minitask.h"
#include "system/profiling.h"
#include "FreeRTOS.h"

#include "log.h"

/* Private define ------------------------------------------------------------*/

/* 
 * the maximum number of tasks we can handle 
 * ( one task requires approx 256 bytes  of SRAM )
 */
#define MAX_TASK            20

/* 
 * maximum number of semaphores the system supports. Semaphores can be allocated
 * and deallocated by any task
 * ( one semaphore requires 84 bytes of SRAM )
 */
// #define MAX_SEMA            5

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    MiniTaskInitFn Init;
    MiniTaskRunFn  Run;
    int32_t        PrID;        /* Profiler ID to count the execution time on */
    TaskHandle_t   TaskID;      /* ID of the created rtos task                */
    StaticTask_t   staticTCB;   /* buffer for the static task control block   */
    StackType_t*   stackMem;    /* Ptr to tasks private stack                 */
    uint32_t       stackSize;   /* Stack size of this stack                   */
#if DEBUG_MODE > 0
    const char *   Name;        /* Name of the task, just for debug purposes  */
#endif
} MiniTaskT;

typedef struct {
    StaticSemaphore_t staticSemaphore;
    SemaphoreHandle_t binaryTaskSemaphore; 
    uint32_t          bIsAllocated;
} SemaphoreT;

/* Private variables ---------------------------------------------------------*/

/* Array of registered tasks */
static MiniTaskT       tasks[MAX_TASK] = {0};

#if 0
    static SemaphoreT semaphores[MAX_SEMA] = {0};
#endif

#if DEBUG_MODE > 0
    /* flag variable to inhibit StopMode (for debug purposes) */
    bool bAllowStop=true; 
#endif
/* Public functions -------------------------------------------------------- */

/******************************************************************************
 * Not implemented on FreeRTOS
 *****************************************************************************/
bool TaskIsRunableTask(void)
{
    return true;
}

/******************************************************************************
 * RTOS Wrapper for all tasks:
 * - check the task semaphore
 * - if task action is required, execute
 * - suspend task
 *****************************************************************************/
void TaskRTOSWrapper ( void *taskArg ) {
    uint32_t myTaskID = (uint32_t)taskArg;
    assert( myTaskID < MAX_TASK );
    #if DEBUG_PROFILING >  0
        /* Accounting to the specified ID, or to main, if nothing specified */
        int32_t profID = tasks[myTaskID].PrID;
        if ( profID < 0 ) profID = JOB_TASK_UNCLASSIFIED;
    #endif
    while ( 1 ) {
        #if DEBUG_PROFILING >  0
            ProfilerSwitchTo(JOB_TASK_SEMWAIT);
        #endif
        /* Check semaphore for timeout, in this case retval == 0 */
        if ( ulTaskNotifyTake( pdFALSE,  portMAX_DELAY ) > 0 ) {
            #if DEBUG_PROFILING >  0
                ProfilerSwitchTo(profID);
            #endif
            tasks[myTaskID].Run(myTaskID);
        }
    } /* endless while */
}

/******************************************************************************
 * Register a new task with an optional Init function, a mamdatory run function
 * and an unique number or prio. Task#0 has the highest prio, Task#31 the lowest
 *****************************************************************************/
#if DEBUG_MODE > 0
void TaskRegisterTask( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, StackType_t* stackMem, uint32_t ulStackDepth, const char *Name )
#else 
void TaskRegisterTaskShort( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, StackType_t* stackMem, uint32_t ulStackDepth )
#endif
{
    assert( num < MAX_TASK);
    assert( r != NULL );


    if ( tasks[num].TaskID != NULL  ) {
        LOG_ERROR("Task %d already set", num);
        return;
    }

    tasks[num].Init      = i;
    tasks[num].Run       = r;
    tasks[num].PrID      = profilerID;
    tasks[num].stackMem  = stackMem;
    tasks[num].stackSize = ulStackDepth;
    tasks[num].TaskID    = NULL;
#if DEBUG_MODE > 0
    tasks[num].Name = Name;
#endif


}
 #if DEBUG_MODE > 0
    #define TASK_NAME(i)    tasks[i].Name
 #else
    #define TASK_NAME(i)    NULL
 #endif

/******************************************************************************
 * first, call the Init-Function for all registered tasks, if specified 
 * thereafter create all tasks
 *****************************************************************************/
void TaskInitAll ( void )
{


    TaskHandle_t  ret;
    ProfilerPush(JOB_TASK_INIT);
    for ( uint32_t i = 0; i < MAX_TASK; i++ ) {
        /* Check, whether task structure is set */
        if ( tasks[i].Run ) {
            /* Initialize task */
            if ( tasks[i].Init ) tasks[i].Init();


            /* start task */
            ret = xTaskCreateStatic( TaskRTOSWrapper, TASK_NAME(i), tasks[i].stackSize, (void *)i, i, tasks[i].stackMem, &tasks[i].staticTCB );
            LOGT_INFO("Started %s", TASK_NAME(i));
            if ( ret ) 
                tasks[i].TaskID = ret;
            else {
                #if DEBUG_MODE > 0
                    LOG_ERROR("Cannot initialize Task #%d (%s)", i, TASK_NAME(i) );
                #endif
            }
        }
    } // for

#if 0
    /* Initialize the pool of shared semaphores */
    for ( uint32_t i=0; i < MAX_SEMA; i++ ) {
        semaphores[i].binaryTaskSemaphore = xSemaphoreCreateBinaryStatic(&(semaphores[i].staticSemaphore));
        semaphores[i].bIsAllocated        = 0;
    }
#endif
    ProfilerPop();
}


/******************************************************************************
 * Notify another task. 
 * Can be called out of interrupt context or from normal execution path
 * Notifications may be cumulated, resulting in more than one execution loop
 * of that task
 *****************************************************************************/
void TaskNotify ( uint32_t num )
{
    if ( tasks[num].TaskID )  {
        /* Check for execution out of interrupt context */
        if ( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk )
            vTaskNotifyGiveFromISR(tasks[num].TaskID, NULL);
        else
            xTaskNotifyGive(tasks[num].TaskID);
    } else {
        // LOG_ERROR("Notify to unset Task #%d", num);
    }
}

#if 0
/******************************************************************************
 * Allocate one semaphore from the static pool of semaphores, execute "give" and
 * return ready for use to caller
* NULL is returned, if there is no unused semaphore in the pool
 *****************************************************************************/
SemaphoreHandle_t TaskSemaphoreAlloc(void)
{
    for ( uint32_t i = 0; i < MAX_SEMA; i++ ) if ( !semaphores[i].bIsAllocated ) {
       semaphores[i].bIsAllocated = 1;
       xSemaphoreGive(semaphores[i].binaryTaskSemaphore);
       return semaphores[i].binaryTaskSemaphore;
    } // for, if

    LOG_ERROR("TaskSemaphoreAlloc: No free semaphore");
    return NULL;
}

void TaskSemaphoreFree( SemaphoreHandle_t used_sem )
{
    for ( uint32_t i = 0; i < MAX_SEMA; i++ ) if ( semaphores[i].binaryTaskSemaphore == used_sem ) {
       semaphores[i].bIsAllocated = 0;
       return;
    } // for, if

    LOG_ERROR("TaskSemaphoreFree: Illegal semaphore");
    return;
}



/******************************************************************************
 * Notify another task out of an interrupt context 
 * Notifications my be cumulated, resulting in more than one execution loop
 * of that task
 *****************************************************************************/
void TaskNotifyFromISR ( uint32_t num )
{
    if ( tasks[num].TaskID )  {
        vTaskNotifyGiveFromISR(tasks[num].TaskID, NULL);
    } else {
        LOG_ERROR("ISR Notify to unset Task #%d", num);
    }
}
#endif

#if DEBUG_MODE > 0

static TaskStatus_t taskstatus[MAX_TASK];
static UBaseType_t nroftasks;

const char * TaskStateTxt[] = { "A", "R", "B", "S", "D", "I", };
static const char* TaskState( uint32_t state )
{
    if ( state > sizeof(TaskStateTxt) / sizeof(const char *) ) 
        return "?";
    else
        return TaskStateTxt[state];
}


/* Fill "taskstatus" array and return the number of entries ( number of tasks ) */
/* 0 is returned in case of more than MAX_TASK                                 */
uint32_t TaskGetTasks(void) 
{
    nroftasks = uxTaskGetNumberOfTasks();
    if ( nroftasks > MAX_TASK ) {
        LOG_ERROR("Too many tasks");
        return 0;
    }   
    uxTaskGetSystemState( taskstatus, MAX_TASK, NULL );
    return nroftasks;
}


void TaskFormatHeader( char* buffer, size_t buflen, const char *prefixstr, uint32_t i )
{
    switch ( i ) {
        case 0:
            snprintf(buffer ,buflen, "%sTask List (%2d Tasks)----------------------------------------------------", prefixstr, nroftasks);
            break;
        case 1:
            #if configGENERATE_RUN_TIME_STATS > 0
                snprintf(buffer ,buflen, "%s   Core No Prio St            Name  UnusedStack           Consumed time",prefixstr);
            #else
                snprintf(buffer ,buflen, "%s   Core No Prio St            Name  UnusedStack  ",prefixstr);
            #endif
            break;
        default:
            *buffer='\0';
    }
}
/* Macros to convert 100 microsecond ticks to seconds/milliseconds */
#define GET_SECONDS(time)   (   time / 10000UL )
#define GET_MILLIS(time)    ( ( time / 10 ) % 1000 )

void TaskFormatBody( char* buffer, size_t buflen, const char *prefixstr, uint32_t i, const char *corename )
{
    if ( i < nroftasks ) {
        snprintf(buffer, buflen,"%s   %s %3d %4d %2s %15s %4d",
                 prefixstr, corename, i,taskstatus[i].uxCurrentPriority, TaskState(taskstatus[i].eCurrentState),  taskstatus[i].pcTaskName,taskstatus[i].usStackHighWaterMark );
        /* Append 6 blanks and time statistics, if configured */
        #if configGENERATE_RUN_TIME_STATS > 0
            #define NUMBLANKS   14
            char *next = buffer + strlen(buffer);
            for ( uint32_t i=0; i < NUMBLANKS; i++ ) *(next++) = ' ';
            *next='\0';
            ProfilerFormatTime((uint64_t)taskstatus[i].ulRunTimeCounter/10, next, buflen - strlen(buffer), false);
        #endif
    } else {
        *buffer = '\0';
    }
}

void TaskFormatFooter( char* buffer, size_t buflen, const char *prefixstr, uint32_t i )
{
    switch ( i ) {
        case 0:
            snprintf(buffer ,buflen, "%s-------------------------------------------------------------------------", prefixstr);
            break;
        case 1:
            snprintf(buffer ,buflen, "%sA: Active, B : Blocked, R : Ready, D : Deleted, S : Suspended, I : Invalid", prefixstr);
            break;
        default:
            *buffer='\0';
    }
}


static uint8_t itemCnt;    /* counts the items when iterating thru header, body, footer */                 
static uint8_t listMode;   /* contains the actual part of list ( header, body, footer ) */
static uint8_t startItem   /* part to start the list generation with                    */
        = LISTMODE_HEADER; 
static uint8_t stopItem    /* part to end the list generation with                      */
        = LISTMODE_FOOTER;
/******************************************************************************
 * Set the Items the list will start ansd end with
 * Allowed are LISTMODE_HEADER, LISTMODE_BODY and LISTMODE_FOOTER for both
 *****************************************************************************/

void TaskSetListStartStop( uint8_t uStart, uint8_t uStop )
{
    startItem = uStart;
    stopItem  = uStop;
}

/* Macro returns true, if no more list items are available */
#define CHECKDONE()   ( *retbuf=='\0' && listMode == stopItem )
/* Macro returns true, if retbuf contains data */
#define MOREITEMS()   ( *retbuf!='\0' )

/******************************************************************************
 * Create the Tasklist by iterated calls to "TaskIterateList"
 * @param retbuf   - buffer to write one line into
 * @param buflen   - allowed max length of return buffer
 * @param actionId - 0 : Start with first line
 *                   1 : generate next line
 * @retval        >= 0 : more lines to follow
 *                  -1 : the returned line is the last one
 *****************************************************************************/
int32_t TaskIterateList ( uint32_t actionId, char *retbuf, size_t buflen, const char *prefixstr )
{
    switch ( actionId ) {
        case ACTIONID_INIT:
            TaskGetTasks();
            listMode = startItem;
            itemCnt  = 0;
            // no break here
            __attribute__((fallthrough));
        case ACTIONID_ITERATE:
            switch ( listMode ) {
                case LISTMODE_HEADER:
                    TaskFormatHeader(retbuf, buflen, prefixstr,itemCnt++);
                    if ( CHECKDONE() ) return -1;
                    if ( MOREITEMS() ) return ACTIONID_ITERATE;
                    /* no more lines in the current category, go to next category */
                    listMode++; itemCnt = 0;
                    // no break here
                    __attribute__((fallthrough));
                case LISTMODE_BODY:
                    #if defined(CORE_CM7)
                        TaskFormatBody(retbuf, buflen, prefixstr,itemCnt++, "CM7");
                    #else
                        TaskFormatBody(retbuf, buflen, prefixstr,itemCnt++, "CM4");
                    #endif
                    if ( CHECKDONE() ) return -1;
                    if ( MOREITEMS() ) return ACTIONID_ITERATE;
                    /* no more lines in the current category, go to next category */
                    listMode++; itemCnt = 0;
                    // no break here
                    __attribute__((fallthrough));
                case LISTMODE_FOOTER:
                    TaskFormatFooter(retbuf, buflen, prefixstr,itemCnt++);
                    if ( CHECKDONE() ) return -1;
                    if ( MOREITEMS() ) return ACTIONID_ITERATE;
                    /* no more lines in the current category, go to next category */
                    listMode++; itemCnt = 0;
                    // no break here
                    __attribute__((fallthrough));
                default:
                    *retbuf = '\0';
                    return -1;
            }
            break;
        default:
            return -1;
   }
}

void TaskDumpList(void)
{
    int32_t i;
    char line[80];

    TaskSetListStartStop(LISTMODE_HEADER, LISTMODE_FOOTER);

    i = ACTIONID_INIT;
    while ( i = TaskIterateList ( i, line, 80, "" ), i >= 0 ) {
        DBG_printf_indent("%s\n",line);
    }
}

#endif // DEBUG_MODE > 0
