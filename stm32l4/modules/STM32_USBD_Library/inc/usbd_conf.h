/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/usbd_conf.h
  * @author  MCD Application Team
  * @brief   General low level driver configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF_H
#define __USBD_CONF_H


/* Includes ------------------------------------------------------------------*/

#include "config/config.h"

#include "hardware.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Common Config */
#define USBD_MAX_NUM_INTERFACES               1
#define USBD_MAX_NUM_CONFIGURATION            1
#define USBD_MAX_STR_DESC_SIZ                 0x100
#define USBD_SUPPORT_USER_STRING              0 
#define USBD_SELF_POWERED                     1
#define USBD_DEBUG_LEVEL                      2

/* Exported macro ------------------------------------------------------------*/
/* Memory management macros */   
#define USBD_malloc               USBD_StaticMalloc
#define USBD_free(a)              USBD_static_free
#define USBD_memset               memset
#define USBD_memcpy               memcpy

/* declare the handcrafted static malloc function */
void *USBD_StaticMalloc( size_t size );
void USBD_static_free(void *p);

/* DEBUG macros */  

#if DEBUG_MODE > 0 && USBD_DEBUG_LEVEL > 0
    #include "debug_helper.h"
#endif

#if (DEBUG_MODE > 0 && USBD_DEBUG_LEVEL > 0)
#define  USBD_UsrLog(...)   do { DEBUG_PRINTF(__VA_ARGS__);\
                            DEBUG_PRINTF("\n"); } while (0)
#else
#define USBD_UsrLog(...)   
#endif                            
                            
#if (DEBUG_MODE > 0 && USBD_DEBUG_LEVEL > 1)

#define  USBD_ErrLog(...)   do { DEBUG_PRINTF("ERROR: ") ;\
                            DEBUG_PRINTF(__VA_ARGS__);\
                            DEBUG_PRINTF("\n"); } while (0)
#else
#define USBD_ErrLog(...)   
#endif 
                                                        
#if (DEBUG_MODE > 0 && USBD_DEBUG_LEVEL > 2)                         
#define  USBD_DbgLog(...)   do { DEBUG_PRINTF("DEBUG : ") ;\
                            DEBUG_PRINTF(__VA_ARGS__);\
                            DEBUG_PRINTF("\n"); } while (0)
#else
#define USBD_DbgLog(...)                         
#endif

/* Exported functions ------------------------------------------------------- */

#endif /* __USBD_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
