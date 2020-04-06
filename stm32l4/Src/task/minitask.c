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
#if DEBUG_MODE > 0
    const char *   Name;        /* Name of the task, just for debug purposes  */
#endif
} MiniTaskT;

/* Private variables ---------------------------------------------------------*/


/* bit vector of used tasks */ 
static uint32_t taskUsedBits = 0;

/* bit vector of runable tasks */ 
static uint32_t taskPendbits = 0;

/* Array of registered tasks */
static MiniTaskT tasks[MAX_TASK];

#if DEBUG_MODE > 0
    /* flag variable to inhibit StopMode (for debug purposes) */
    bool bAllowStop=true; 
#endif
/* Public functions -------------------------------------------------------- */

/******************************************************************************
 * Returns true, if any task is runable
 *****************************************************************************/
bool TaskIsRunableTask(void)
{
    return taskPendbits != 0;
}

/******************************************************************************
 * Register a new task with an optional Init function, a mamdatory run function
 * and an unique number or prio. Task#0 has the highest prio, Task#31 the lowest
 *****************************************************************************/
#if DEBUG_MODE > 0
void TaskRegisterTask( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, const char *Name )
#else 
void TaskRegisterTask( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID )
#endif
{
    assert( num < MAX_TASK);
    assert( r != NULL );

    uint32_t mask = 1 << num;

    if ( taskUsedBits & mask ) {
        DEBUG_PRINTF("Task %d already set\n", num);
        return;
    }

    tasks[num].Init = i;
    tasks[num].Run  = r;
    tasks[num].PrID = profilerID;
#if DEBUG_MODE > 0
    tasks[num].Name = Name;
#endif

    taskUsedBits |= mask;
    taskPendbits &= ~mask;
}

/******************************************************************************
 * Call the Init-Function for all registered tasks, if specified 
 *****************************************************************************/
void TaskInitAll ( void )
{
    uint32_t shift = 1 << 0;
    ProfilerPush(JOB_TASK_INIT);
    for ( uint32_t i = 0; i < MAX_TASK; i++ ) {
        if ( taskUsedBits & shift && tasks[i].Init ) tasks[i].Init();
        shift <<= 1;
    }
    ProfilerPop();
}

/******************************************************************************
 * Call the Run-Function for all runable tasks. The pending bit will be reset
 * before execution of run-function, so the run function may set the pending
 * bit again, if required
 *****************************************************************************/
void TaskRunAll ( void )
{
    uint32_t shift = 1 << 0;
    int32_t profID;
    for ( uint32_t i = 0; i < MAX_TASK; i++ ) {
        if ( taskPendbits & shift ) {
            taskPendbits &= ~shift;
            profID = tasks[i].PrID;
            if ( profID >= 0 ) { ProfilerPush(profID); }
            tasks[i].Run(i);
            if ( profID >= 0 ) { ProfilerPop(); }
        }
        shift <<= 1;
    }
}

/******************************************************************************
 * Make a sleeping task runable
 * As notification may take place even when task ist not initialized,
 * always allow pending bit to be set
 *****************************************************************************/
void TaskNotify ( uint32_t num )
{
    uint32_t mask = ( 1 << num );
    taskPendbits |= mask;
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

    for ( i = 0, mask=1; i < MAX_TASK; i++, mask <<= 1 ) if ( taskUsedBits & mask ) {
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
