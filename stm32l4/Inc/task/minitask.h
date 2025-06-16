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
#define TASK_USBD             1
#define TASK_OW               2
#define TASK_COM              3
#define TASK_RTC              4
#define TASK_LOG              5
#define TASK_RFM              6
#define TASK_PERIODIC         7
#define TASK_EPD              8
#define TASK_ADC              9
#define TASK_STDIN            10
#define TASK_XSPI             11
#define TASK_LCD              12
#define TASK_LVGL             13
#define TASK_PULSE            20
#define TASK_SEQUENCE         21
#define TASK_THP              30

/* functtion prototypes of Init and Run functions of task */
/* both will be passed the task number as parameter       */
typedef void ( *MiniTaskInitFn ) ( void );
typedef void ( *MiniTaskRunFn  ) ( uint32_t );

/* public functions ---------------------------------------------------------*/
bool TaskIsRunableTask  (void);

#if DEBUG_MODE > 0
    void TaskRegisterTask ( MiniTaskInitFn i, MiniTaskRunFn r, uint32_t num, int32_t profilerID, const char *Name );
    void TaskDumpList     (void);
    extern bool bAllowStop;         /* flag variable to inhibit stop by command ( only in debug mode ) */
#else
    void TaskRegisterTaskShort ( MiniTaskInitFn, MiniTaskRunFn, uint32_t num, int32_t PrID);
    #define TaskRegisterTask(i,r,n,d,a)     TaskRegisterTaskShort(i,r,n,d)
    #define TaskDumpList()
#endif
void TaskInitAll        ( void );
void TaskRunAll         ( void );
void TaskNotify         ( uint32_t num );


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MINITASK_H */
