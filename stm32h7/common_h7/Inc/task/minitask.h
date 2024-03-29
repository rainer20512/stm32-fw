/**
  ******************************************************************************
  * @file    "minitask.h" 
  * @author  Rainer
  * @brief   Definition of all used devices. 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration itme, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MINITASK_H
#define __MINITASK_H

#include "config/config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* 
 * every bit in the task-variable specifies a specfic task to be performed
 * the enumeration should be don with decreasing priorities, ie. bit 0 has
 * the highest priority and will be handled first, whilst bit31 has the 
 * lowest priority and will be handled last
 */

#define TASK_TMR              0
#define TASK_COM              1
#define TASK_OW               2
#define TASK_USB              3
#define TASK_RTC              4
#define TASK_LOG              5
#define TASK_RFM              6
#define TASK_PERIODIC         7
#define TASK_ADC              8
#define TASK_LCD              9
#define TASK_XSPI             10
#define TASK_REMOTE_CM7       11
#define TASK_REMOTE_CM4       12
#define TASK_SERIN            13
#define TASK_LOGFILE          14
#define TASK_ZZZ              15
#define TASK_YYY              16
#define TASK_XXX              17
#define TASK_WWW              18
#define TASK_VVV              19

/* functtion prototypes of Init and Run functions of task */
/* both will be passed the task number as parameter       */
typedef void ( *MiniTaskInitFn ) ( void );
typedef void ( *MiniTaskRunFn  ) ( uint32_t );

/* public functions ---------------------------------------------------------*/
bool TaskIsRunableTask  (void);
void TaskInitAll        ( void );
void TaskRunAll         ( void );
void TaskNotify         ( uint32_t num );

SemaphoreHandle_t TaskSemaphoreAlloc(void);
void              TaskSemaphoreFree (SemaphoreHandle_t used_sem);


// void TaskNotifyFromISR  ( uint32_t num );  use TaskNotify instead !

#if DEBUG_MODE > 0
    void TaskRegisterTask   ( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, StackType_t* stackMem, uint32_t ulStackDepth, const char *Name );
#if 0
    uint32_t TaskGetTasks   (void);
    void TaskFormatHeader   ( char* buffer, size_t buflen, const char *prefixstr, uint32_t i );
    void TaskFormatLine     ( char* buffer, size_t buflen, const char *prefixstr, uint32_t i, const char *corename);
#endif
    void TaskDumpList       (void);
    extern bool bAllowStop;         /* flag variable to inhibit stop by command ( only in debug mode ) */

    #define LISTMODE_HEADER         0
    #define LISTMODE_BODY           1
    #define LISTMODE_FOOTER         2
    #define ACTIONID_INIT           0
    #define ACTIONID_ITERATE        1

    void TaskSetListStartStop( uint8_t uStart, uint8_t uStop );
    int32_t TaskIterateList ( uint32_t actionId, char *retbuf, size_t buflen, const char *prefixstr );
#else
    void TaskRegisterTaskShort ( MiniTaskInitFn, MiniTaskRunFn, uint32_t num, int32_t PrID, StackType_t* stackMem, uint32_t ulStackDepth);
    #define TaskRegisterTask(i,r,n,p,s,d,a)     TaskRegisterTaskShort(i,r,n,p,s,d)
    #define TaskDumpList()
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MINITASK_H */
