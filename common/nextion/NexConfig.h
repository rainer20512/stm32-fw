
#ifndef __NEXCONFIG_H__
#define __NEXCONFIG_H__
#include "Serial.h"
#include "FreeRTOS.h"
#include "task.h"

#define nexSerial_init(b)           Serial_Init(b)
#define nexSerial_available()       Serial_Available()
#define nexSerial_read()            Serial_Read()
#define nexSerial_start()           Serial_Start()
#define nexSerial_terminate(d)      Serial_Terminate(d)
#define nexSerial_print(p)          Serial_Print(p) 
#define nexSerial_readBytes(b,l)    Serial_ReadBytes(b, l)
#define nexSerial_WaitFor(n,r,l)    Serial_WaitFor(n,r,l)
#define nexDelay(d)                 vTaskDelay(d/portTICK_PERIOD_MS)

#endif /* #ifndef __NEXCONFIG_H__ */
