
/**
 ******************************************************************************
 * @file    "minitask.v" 
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
#include "task.h"

#if DEBUG_MODE > 0
    #include "debug_helper.h"
#endif
/* Private define ------------------------------------------------------------*/

/* the maximum number of task is defined by the number of bits in the 
   following variables. */
#define MAX_TASK            32

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

/* Private variables ---------------------------------------------------------*/

/* Array of registered tasks */
static MiniTaskT tasks[MAX_TASK] = {0};

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
    uint32_t sema;
    assert( myTaskID < MAX_TASK );
    #if DEBUG_PROFILING >  0
        /* Accounting to the specified ID, or to main, if nothing specified */
        uint32_t profID = tasks[myTaskID].PrID;
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
void TaskRegisterTask( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, StackType_t* stackMem, uint32_t ulStackDepth )
#endif
{
    assert( num < MAX_TASK);
    assert( r != NULL );


    if ( tasks[num].TaskID > 0  ) {
        DEBUG_PRINTF("Task %d already set\n", num);
        return;
    }

    tasks[num].Init      = i;
    tasks[num].Run       = r;
    tasks[num].PrID      = profilerID;
    tasks[num].stackMem  = stackMem;
    tasks[num].stackSize = ulStackDepth;
    tasks[num].TaskID    = 0;
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
            if ( ret ) 
                tasks[i].TaskID = ret;
            else {
                #if DEBUG_MODE > 0
                    DEBUG_PRINTF("Error: Cannot initialize Task #%d (%s)\n", i, TASK_NAME(i) );
                #endif
            }
        }
    } // for


    ProfilerPop();
}


/******************************************************************************
 * Notify another task. 
 * Notifications my be cumulated, resulting in more than one execution loop
 * of that task
 *****************************************************************************/
void TaskNotify ( uint32_t num )
{
    if ( tasks[num].TaskID )  {
        xTaskNotifyGive(tasks[num].TaskID);
    } else {
        DEBUG_PRINTF("Notifying unset Task #%d\n", num);
    }
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
        DEBUG_PRINTF("Notifying unset Task #%d\n", num);
    }
}


#if DEBUG_MODE > 0
void TaskDumpList(void)
{
    uint32_t i;
    uint32_t mask;

    DEBUG_PUTS  ("Task List ----------------------------------------------------------------");

    int oldIndent = DBG_setIndentRel(+2);
    DBG_printf_indent(" ID          Name");
    #if DEBUG_PROFILING > 0
        DBG_printf_indent("           Consumed time");
        DBG_setPadLen(20);
    #endif
    DEBUG_PRINTF("\n");

    for ( i = 0, mask=1; i < MAX_TASK; i++, mask <<= 1 ) if ( tasks[i].Run ) {
        DBG_printf_indent("%3d",i);
        #if DEBUG_PROFILING > 0
            ProfilerDumpTime( ProfilerTimes[tasks[i].PrID], (char *)tasks[i].Name);
        #else
            DBG_printf_indent("   %20s",i, tasks[i].Name);
            DEBUG_PRINTF("\n");
        #endif
    }
 
    DBG_setIndentAbs(oldIndent);
}
#endif
