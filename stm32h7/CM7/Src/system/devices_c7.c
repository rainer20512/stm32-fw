/**
  ******************************************************************************
  * @file    devices.c
  * @author  Rainer
  * @brief   Initialization of all used peripheral devices
  *          For Dual Core, we have a slightly different implementation
  *          for both Cores
  *          CM7 will also be notfied about CM4 used devices to allow bookkeeping 
  *          of devices used in both cores
  *          CM4 will not implement a "dupicates check", this is done by CM7 core
  *          The communication between the two instances is done via common memory
  *          and hardware semaphores
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <debugio.h>


#include "config/config.h"
#include "config/devices_config.h"

#include "system/hw_util.h"
#include "debug_helper.h"
#include "dev/devices.h"
/** @addtogroup DEVICES
  * @{
  */

/* Private define -----------------------------------------------------------------------*/
#define MAX_DEV    16
#define MAX_PRIO 0x0f

#define DEV_CORE_CM7        0
#define DEV_CORE_CM4        1

/* Private typedef ----------------------------------------------------------------------*/
/* Private macro ------------------------------------------------------------------------*/

/* Private variables --------------------------------------------------------------------*/
static const HW_DeviceType* devices[MAX_DEV];   /* ptr to device descripter              */
static HW_DynInitBlock      devDyn [MAX_DEV];   /* Ptr to dynamic PostInit/PreDeInit Fns */
static uint8_t              devIsInited[MAX_DEV]; /* flag for "device is Initialized     */
static uint8_t              devCoreNum[MAX_DEV];  /* the core the devices is attached to */
static uint32_t             act_devices     = 0;

#define PRINTNAME(a)     ( a ? a : "" )
#define PRINTCOLON(a)    ( a ? " : " : "" )

const char * const corename[]={"CM7","CM4"};
static const char* get_core_txt(uint32_t sel )
{
  if ( sel < sizeof(corename)/sizeof(char *) ) 
    return corename[sel];
  else
    return "Illegal";
}

static void dump_one_device ( uint32_t idx, bool bLong, const HW_DeviceType *dev )
{
    const HW_GpioList_AF *gpioaf;
    const HW_GpioList_IO *gpioio;
    const HW_GpioList_ADC *gpioadc;

    DBG_printf_indent("%2d) %s %sInit'ed\n", idx, dev->devName, devIsInited[idx] ? "" : "NOT " );
    if ( bLong) {
        DBG_setIndentRel(+4);
        gpioaf = dev->devGpioAF;
        gpioio = dev->devGpioIO;
        gpioadc= dev->devGpioADC;
        if ( gpioaf ) 
            for ( uint8_t j = 0; j < gpioaf->num; j++ ) {
                if ( gpioaf->gpio[j].gpio )
                    DBG_printf_indent("  AF  %c%02d%s%s\n",HW_GetGPIOLetter(gpioaf->gpio[j].gpio), HW_GetIdxFromPin(gpioaf->gpio[j].pin)
                                      ,PRINTCOLON(gpioaf->gpio[j].dbg_name), PRINTNAME(gpioaf->gpio[j].dbg_name)                                 
                    );
            }
        if ( gpioio ) 
            for ( uint8_t j = 0; j < gpioio->num; j++ ) {
                if ( gpioio->gpio[j].gpio )
                    DBG_printf_indent("  IO  %c%02d%s%s\n",HW_GetGPIOLetter(gpioio->gpio[j].gpio), HW_GetIdxFromPin(gpioio->gpio[j].pin)
                                      ,PRINTCOLON(gpioio->gpio[j].dbg_name), PRINTNAME(gpioio->gpio[j].dbg_name)                                 
                    );
            }
        if ( gpioadc ) 
            for ( uint8_t j = 0; j < gpioadc->num; j++ ) {
                if ( gpioadc->gpio[j].gpio )
                    DBG_printf_indent("  ADC %c%02d%s%s\n",HW_GetGPIOLetter(gpioadc->gpio[j].gpio), HW_GetIdxFromPin(gpioadc->gpio[j].pin) 
                                      ,PRINTCOLON(gpioadc->gpio[j].dbg_name), PRINTNAME(gpioadc->gpio[j].dbg_name)
                    );
            }
      
        DBG_setIndentRel(-4);
    } // if ( bLong )
}

void DBG_dump_devices(bool bLong)
{
    const HW_DeviceType *dev;
    int oldIndent;

    DEBUG_PRINTF("List of hardware devices - Core %s -----------\n", get_core_txt(DEV_CORE_CM7));
    oldIndent = DBG_setIndentAbs(2);
    DBG_setPadLen(20);
    for ( uint32_t i = 0; i < act_devices; i++ ) if ( devCoreNum[i] == DEV_CORE_CM7 ) {
        dump_one_device(i, bLong, devices[i]);
    }
    
    DBG_setIndentAbs(oldIndent);

    DEBUG_PRINTF("List of hardware devices - Core %s -----------\n", get_core_txt(DEV_CORE_CM4));
    oldIndent = DBG_setIndentAbs(2);
    DBG_setPadLen(20);
    for ( uint32_t i = 0; i < act_devices; i++ ) if ( devCoreNum[i] == DEV_CORE_CM4 ) {
        dump_one_device(i, bLong, devices[i]);
    } 
    
    DBG_setIndentAbs(oldIndent);

}

/******************************************************************************
 * Check whether Pin "pin" of GPIO "gpio" is already contained in device list,
 * i.e. whether this pin is already used in another device
 * note: "gpio" may be NULL for any internal pins/signals
 * note: devices on all cores are checked
 *****************************************************************************/
bool CheckOnePin( const HW_DeviceType *newdev, uint8_t newdevCore, GPIO_TypeDef *gpio, uint16_t pin )
{
    const HW_DeviceType *dev;
    uint8_t devCore;

    const HW_GpioList_AF *gpioaf;
    const HW_GpioList_IO *gpioio;
    const HW_GpioList_ADC *gpioadc;

    if ( !gpio ) return true;

    for ( uint8_t i = 0; i < act_devices; i++ )  {
        dev     = devices[i];
        devCore = devCoreNum[i];
        if ( dev != newdev ) {
            gpioaf = dev->devGpioAF;
            gpioio = dev->devGpioIO;
            gpioadc= dev->devGpioADC;
            if ( gpioaf ) 
                for ( uint8_t j = 0; j < gpioaf->num; j++ ) {
                    if ( gpioaf->gpio[j].gpio && gpioaf->gpio[j].gpio == gpio && gpioaf->gpio[j].pin == pin) {
                        #if DEBUG_MODE > 0
                            DEBUG_PRINTF("GPIO%c%d of device %s on core %s already used in device %s on core %s\n",HW_GetGPIOLetter(gpio), HW_GetIdxFromPin(pin), 
                            newdev->devName, get_core_txt(newdevCore), dev->devName, get_core_txt(devCore));
                        #endif
                        return false;
                    }
                    
                }
            if ( gpioio ) 
                for ( uint8_t j = 0; j < gpioio->num; j++ ) {
                    if ( gpioio->gpio[j].gpio && gpioio->gpio[j].gpio == gpio && gpioio->gpio[j].pin == pin) {
                        #if DEBUG_MODE > 0
                            DEBUG_PRINTF("GPIO%c%d of device %s on core %s already used in device %s on core %s\n",HW_GetGPIOLetter(gpio), HW_GetIdxFromPin(pin), 
                            newdev->devName, get_core_txt(newdevCore), dev->devName, get_core_txt(devCore));
                        #endif
                        return false;
                    }
                }
            if ( gpioadc ) 
                for ( uint8_t j = 0; j < gpioadc->num; j++ ) {
                    if ( gpioadc->gpio[j].gpio && gpioadc->gpio[j].gpio == gpio && gpioadc->gpio[j].pin == pin) {
                        #if DEBUG_MODE > 0
                            DEBUG_PRINTF("GPIO%c%d of device %s on core %s already used in device %s on core %s\n",HW_GetGPIOLetter(gpio), HW_GetIdxFromPin(pin), 
                            newdev->devName, get_core_txt(newdevCore), dev->devName, get_core_txt(devCore));
                        #endif
                        return false;
                    }
                }
        } // if ( dev != newdev ) 
    } // for
    return true;

}

/******************************************************************************
 * retruns true, if "dev" has only GPIO-Pins, that are NOT used/allocated by 
 * other devices on any core
 *****************************************************************************/
 static bool CheckUnique ( const HW_DeviceType *dev, uint32_t devCore )
{
    const HW_GpioList_AF *gpioaf;
    const HW_GpioList_IO *gpioio;
    const HW_GpioList_ADC *gpioadc;
    bool ret;

    gpioaf = dev->devGpioAF;
    gpioio = dev->devGpioIO;
    gpioadc= dev->devGpioADC;
    if ( gpioaf ) 
        for ( uint8_t j = 0; j < gpioaf->num; j++ ) {
            ret = CheckOnePin(dev, devCore, gpioaf->gpio[j].gpio, gpioaf->gpio[j].pin );
            if ( !ret ) return ret;
        }
    if ( gpioio ) 
        for ( uint8_t j = 0; j < gpioio->num; j++ ) {
            ret = CheckOnePin(dev, devCore, gpioio->gpio[j].gpio, gpioio->gpio[j].pin );
            if ( !ret ) return ret;
        }
    if ( gpioadc ) 
        for ( uint8_t j = 0; j < gpioadc->num; j++ ) {
            ret = CheckOnePin(dev, devCore, gpioadc->gpio[j].gpio, gpioadc->gpio[j].pin );
            if ( !ret ) return ret;
        }

    return true;
}

/******************************************************************************
 * Stub for CheckUniqueRemote from CM4 core
 *****************************************************************************/
bool CheckUniqueRemote(const HW_DeviceType *dev)
{
    return CheckUnique( dev, DEV_CORE_CM4);
}

/******************************************************************************
 * returns true, if hardwarwe address is not used by any other device
 * i.e. if any hardware component is not used by more than one device
 * Hardware address NULL is used for devices, that do not use any hardware
 * any may occur multiple times
 *****************************************************************************/
static bool CheckHardwareUnique ( const HW_DeviceType *dev, uint8_t devCore )
{
    bool ret = true;
    for ( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devices[i]->devBase &&  devices[i]->devBase == dev->devBase ) {
            ret = false;
            DEBUG_PRINTF("Devices %s on core %s and %s on core %s share thesame hardware!\n", devices[i]->devName, get_core_txt(devCoreNum[i]), dev->devName, get_core_txt(devCore));
        }
    }

    return ret;
}
 


static int32_t add_one_device(const HW_DeviceType *dev, uint8_t devCore, HW_DynInitT postInit, HW_DynDeInitT preDeInit)
{
    if ( !dev->Init || !dev->DeInit ) {
        DEBUG_PUTS("AddDevice: Init and DeInit must be specified");
        return -1;
    }

    if ( act_devices >= MAX_DEV ) {
        DEBUG_PUTS("AddDevice: max. number of devices reached");
        return -1;
    }

    if ( !CheckHardwareUnique(dev, devCore) ) {
        DEBUG_PRINTF("AddDevice: %s from core %s causes hardware collision\n", dev->devName, get_core_txt(devCore));
    }

    devices[act_devices]     = dev;
    devIsInited[act_devices] = false;
    devCoreNum[act_devices]  = devCore;
    devDyn [act_devices].PostInitFn =postInit;
    devDyn [act_devices].PreDeInitFn=preDeInit;
    return (int32_t)act_devices++;
}

/******************************************************************************
 * @brief  Registers "dev" as new device together with device specific postInit 
 *         and preDeInit functions 
 *         Will return the assigned deviceID on success
 * @param  dev - Ptr to a device to be registered
 *         postInit - device specific postInit fn, may be NULL, will be called
 *                    automatically after mandatory devices Init function
 *         preDeInit - device specific preDeInit fn, may be NULL, will be called
 *                    automatically after mandatory devices DeInit function
 *
 * @return device ID on success
 *         -1 in case of failure
 * @note   Every device MUST implement an non-NULL Init and DeInit function
 *         in order to be successfuly registered
 *****************************************************************************/
int32_t AddDevice(const HW_DeviceType *dev, HW_DynInitT postInit, HW_DynDeInitT preDeInit)
{
    return add_one_device(dev, DEV_CORE_CM7, postInit, preDeInit);
}

/******************************************************************************
 * @brief  Registers "dev" as new remote device together with ist core ID
 *         Will return the assigned deviceID on success
 * @param  dev - Ptr to a device to be registered
 *         postInit and preDeinit are not needed here, because
 *         this is done by the owning core
 *
 * @return device ID on success
 *         -1 in case of failure
 * @note   Every device MUST implement an non-NULL Init and DeInit function
 *         in order to be successfuly registered
 *****************************************************************************/
int32_t AddDeviceRemote(const HW_DeviceType *dev)
{
    return add_one_device(dev, DEV_CORE_CM4, NULL, NULL);
}


/******************************************************************************
 * @brief Initialize the specified device by calling the devices Init-function
 *        In case a specific PostInit function was specified with registration
 *        of that device, this postInit function will be called after devices
 *        Init function
 *        
 * @param dev_idx - device ID returned by registration 
 * @return true, if successful, false otherwise
 *****************************************************************************/
bool DeviceInitByIdx(uint32_t dev_idx, void *arg)
{
    if ( dev_idx >= act_devices) return false;

    register const HW_DeviceType *dev = devices[dev_idx];

    devIsInited[dev_idx] = false;

    /* Check for own device */
    if ( devCoreNum[dev_idx] != DEV_CORE_CM7 ) {
        #if DEBUG_MODE > 0
            DEBUG_PRINTF("Device %s not initialized, is remote on core %s\n", dev->devName, get_core_txt(devCoreNum[dev_idx]));
        #endif
        return false;
    }


    /* check for unique pin assignments */
    if (!CheckUnique(dev, DEV_CORE_CM7) ) {
        #if DEBUG_MODE > 0
            DEBUG_PRINTF("Device %s not initialized\n", dev->devName);
        #endif
        return false;
    }

    /* try to init */
    if ( !dev->Init(dev) ) {
        DEBUG_PRINTF("DeviceInit: Init of Device %s failed!\n", dev->devName);
        return false;
    }
    
    /* execute specific postInit, if defined */
    if ( devDyn[dev_idx].PostInitFn ) devDyn[dev_idx].PostInitFn(dev,arg);
    return ( devIsInited[dev_idx] = true );

}

/******************************************************************************
 * @brief DeInitialize the specified device by calling the devices DeInit-
 *        function. In case a specific PreDeInit function was specified with 
 *        registration, of that device, this preDeInit function will be called 
 *        before devices DeInit function
 *        
 * @param dev_idx - device ID returned by registration 
 * @return nothing
 *****************************************************************************/
void DeviceDeInitByIdx(uint32_t dev_idx)
{
    if ( dev_idx >= act_devices) return;

    /* Check for own device */
    if ( devCoreNum[dev_idx] != DEV_CORE_CM7 ) return;

    register const HW_DeviceType *dev = devices[dev_idx];

    /* execute specific preDeInit, if defined */
    if ( devDyn[dev_idx].PreDeInitFn ) devDyn[dev_idx].PreDeInitFn(dev);

    /* Do the DeInit */
    dev->DeInit(dev);
    devIsInited[dev_idx] = false;
}

/******************************************************************************
 * Returns true, if any device does not allow to enter Stop2 mode due to      
 * activity of that device
 ******************************************************************************/
bool DevicesInhibitStop(void)
{
    /* Check every device with AllowStop method implemented for denial of stop */
    /* Only regard my own devices */
    for( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devCoreNum[i] == DEV_CORE_CM7 && devices[i]->AllowStop && !devices[i]->AllowStop(devices[i]) ) {
            #if DEBUG_SLEEP_STOP > 1
              store_chr('!');
              store_str(devices[i]->devName);
              // store_chr('?');
            #endif
            return true;
        }
    }

    return false;
}

/******************************************************************************
 * Returns true, if any device does NOT allow a system clock frequency change 
 * to "newSysClk"
 ******************************************************************************/
bool DevicesInhibitFrqChange(void)
{
    bool ret = false;
    /* Check every device with AllowStop method implemented for denial of stop */
    /* Only regard my own devices */
    for( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devCoreNum[i] == DEV_CORE_CM7 && devices[i]->OnFrqChange && !devices[i]->OnFrqChange(devices[i]) ) {
            #if DEBUG_MODE > 0
                DEBUG_PRINTF("Device %s inhibits change of SysClk\n", devices[i]->devName);
            #endif
            ret = true;
        }
    }

    return ret;
}


const HW_DeviceType *FindDevByBaseAddr(uint32_t dt, void *pBase )
{
    for( uint8_t i = 0; i < act_devices; i++ ) {
        if ( devCoreNum[i] == DEV_CORE_CM7 ){
            if ( devices[i]->devType == dt && devices[i]->devBase && devices[i]->devBase == pBase ) return devices[i];
        }
    }
    return NULL;
}
#ifndef HW_DEBUG_UART
    #error "DEBUG_UART is not defined!"
#endif

/******************************************************************************
 * DO the init of the mandatory devices: IO and debug uart
 ******************************************************************************/
void BasicDevInit(void)
{

  int32_t dev_idx;

  /* Init IO_DEV */
  dev_idx = AddDevice(&HW_IO,NULL ,NULL);
  DeviceInitByIdx(dev_idx, NULL);

  /* Init debug u(s)art ) */
  #if DEBUG_FEATURES > 0 && DEBUG_DEBUGIO == 0
     dev_idx = AddDevice(&HW_DEBUG_UART, DebugAssignUart, DebugDeAssignUart);
     DeviceInitByIdx(dev_idx, (void *)1);
  #endif
}

/**
  * @}
  */

