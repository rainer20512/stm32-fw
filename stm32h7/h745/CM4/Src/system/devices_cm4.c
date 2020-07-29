/**
  ******************************************************************************
  * @file    devices_cm4.c
  * @author  Rainer
  * @brief   Initialization of all used peripheral devices on Core CM4
  * @note    The implementation is somewhat different from single core
  *          implementation, 
  *          The housekeeping of used devices is done on CM7 core, so this
  *          core has to do an IPC call to register remotely and check against
  *          remote device database
  * @note    The devices are also kept locally to be able to do intialization
  *          and deinitialiation as well as react to clock frq changes
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
#include "msg_direct.h"


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
static uint32_t             devRmtIdx[MAX_DEV]; /* remote idx of this CM4 device         */
static uint32_t             act_devices     = 0;


/* Public functions ---------------------------------------------------------------------*/
#define PRINTNAME(a)     ( a ? a : "" )
#define PRINTCOLON(a)    ( a ? " : " : "" )
#define PRINTOWNERSHIP(i,g,p)   ( IsMyPin(i,g,p) ? "" : " Not assigned")

void DBG_dump_devices(bool bLong)
{
    const HW_DeviceType *dev;
    const HW_GpioList_AF *gpioaf;
    const HW_GpioList_IO *gpioio;
    const HW_GpioList_ADC *gpioadc;

    DEBUG_PUTS("List of hardware devices owned by core CM4 ---");
    int oldIndent = DBG_setIndentAbs(2);
    DBG_setPadLen(20);
    for ( uint32_t i = 0; i < act_devices; i++ ) {
        dev = devices[i];
        DBG_printf_indent("%2d) %s %sInit'ed\n", i, dev->devName, devIsInited[i] ? "" : "NOT " );
        if ( bLong) {
            DBG_setIndentRel(+4);
            gpioaf = dev->devGpioAF;
            gpioio = dev->devGpioIO;
            gpioadc= dev->devGpioADC;
            if ( gpioaf ) 
                for ( uint8_t j = 0; j < gpioaf->num; j++ ) {
                    if ( gpioaf->gpio[j].gpio )
                        DBG_printf_indent("  AF  %c%02d%s%s%s\n",HW_GetGPIOLetter(gpioaf->gpio[j].gpio), HW_GetIdxFromPin(gpioaf->gpio[j].pin)
                                          ,PRINTCOLON(gpioaf->gpio[j].dbg_name), PRINTNAME(gpioaf->gpio[j].dbg_name)
                                          ,PRINTOWNERSHIP(i,gpioaf->gpio[j].gpio,gpioaf->gpio[j].pin)
                        );
                }
            if ( gpioio ) 
                for ( uint8_t j = 0; j < gpioio->num; j++ ) {
                    if ( gpioio->gpio[j].gpio )
                        DBG_printf_indent("  IO  %c%02d%s%s%s\n",HW_GetGPIOLetter(gpioio->gpio[j].gpio), HW_GetIdxFromPin(gpioio->gpio[j].pin)
                                          ,PRINTCOLON(gpioio->gpio[j].dbg_name), PRINTNAME(gpioio->gpio[j].dbg_name)
                                          ,PRINTOWNERSHIP(i,gpioio->gpio[j].gpio,gpioio->gpio[j].pin)
                        );
                }
            if ( gpioadc ) 
                for ( uint8_t j = 0; j < gpioadc->num; j++ ) {
                    if ( gpioadc->gpio[j].gpio )
                        DBG_printf_indent("  ADC %c%02d%s%s%s\n",HW_GetGPIOLetter(gpioadc->gpio[j].gpio), HW_GetIdxFromPin(gpioadc->gpio[j].pin) 
                                          ,PRINTCOLON(gpioadc->gpio[j].dbg_name), PRINTNAME(gpioadc->gpio[j].dbg_name)
                                          ,PRINTOWNERSHIP(i,gpioadc->gpio[j].gpio,gpioadc->gpio[j].pin)
                        );
                }
      
            DBG_setIndentRel(-4);
        } // if ( bLong )
    } // for
    
    DBG_setIndentAbs(oldIndent);
}

/******************************************************************************
 * Check, whether Pin "Pin" of GPIO port "gpio" is still unassigned
 * if so, it is assigned to "newdev" and true is returned
 * if already assigned to any other device, false is returned
 *****************************************************************************/
bool AssignOnePin( uint32_t devIdx, GPIO_TypeDef *gpio, uint16_t pin )
{
    /* Do remote registration */
    MSGD_DoRemotePinAssignment(MSGTYPE2_SUBID_ASSIGN, devRmtIdx[devIdx], gpio, pin);

    return MSGD_WaitForRemotePinAssignment();
}

/******************************************************************************
 * returns true, if device owns that GPIO pin
 *****************************************************************************/
bool IsMyPin( uint32_t devIdx, GPIO_TypeDef *gpio, uint16_t pin )
{    
    /* Do remote query */
    MSGD_DoRemotePinAssignment(MSGTYPE2_SUBID_QUERY, devRmtIdx[devIdx], gpio, pin);

    return MSGD_WaitForRemotePinAssignment();
}

/******************************************************************************
 * Deassign one GPIO pin from a device
 * this is only possible, iff the device owned that pin before
 *****************************************************************************/
bool DeassignOnePin( uint32_t devIdx, GPIO_TypeDef *gpio, uint16_t pin )
{
    /* Do remote deassignment */
    MSGD_DoRemotePinAssignment(MSGTYPE2_SUBID_DEASSIGN, devRmtIdx[devIdx], gpio, pin);

    return MSGD_WaitForRemotePinAssignment();
}


/******************************************************************************
 * returns true, if hardwarwe address is not used by any other device
 * i.e. if any hardware component is not used by more than one device
 * Hardware address NULL is used for devices, that do not use any hardware
 * any may occur multiple times
 *****************************************************************************/
static bool CheckHardwareUnique ( const HW_DeviceType *dev )
{
    bool ret = true;
    for ( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devices[i]->devBase &&  devices[i]->devBase == dev->devBase ) {
            ret = false;
            DEBUG_PRINTF("Devices %s and %s share thesame hardware!\n", devices[i]->devName, dev->devName);
        }
    }

    return ret;
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
    if ( !dev->Init || !dev->DeInit ) {
        DEBUG_PUTS("AddDevice: Init and DeInit must be specified");
        return -1;
    }

    if ( act_devices >= MAX_DEV ) {
        DEBUG_PUTS("AddDevice: No more room to store CM4-deivces");
        return -1;
    }

    if ( !CheckHardwareUnique(dev) ) {
        DEBUG_PRINTF("AddDevice: %s causes hardware collision\n", dev->devName);
    }

    /* Do remote registration */
    MSGD_DoRemoteRegistration((void *)dev);
    int32_t remoteIdx = MSGD_WaitForRemoteRegistration();

    if ( remoteIdx  == -1 ) {
        DEBUG_PUTS("AddDevice: Remote registration failed");
        return -1;
    }

    devices[act_devices]            = dev;
    devRmtIdx[act_devices]          = remoteIdx;
    devIsInited[act_devices]        = false;
    devDyn [act_devices].PostInitFn = postInit;
    devDyn [act_devices].PreDeInitFn= preDeInit;


    return (int32_t)act_devices++;
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
    for( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devices[i]->AllowStop && !devices[i]->AllowStop(devices[i]) ) {
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
    /* Check every device with OnFrqChange method implemented for denial of stop */
    for( uint32_t i = 0; i < act_devices; i++ ) {
        if ( devices[i]->OnFrqChange && !devices[i]->OnFrqChange(devices[i]) ) {
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
        if ( devices[i]->devType == dt && devices[i]->devBase && devices[i]->devBase == pBase ) return devices[i];
    }
    return NULL;
}

/******************************************************************************
 * return the index of the device list for a given device
 * -1 will be returned when not foune
 ******************************************************************************/
int32_t GetDevIdx ( const HW_DeviceType *dev)
{
    for( uint8_t i = 0; i < act_devices; i++ ) {
        if ( devices[i] == dev ) return i;
    }
    DEBUG_PUTS("Error: GetDevIdx: Device not found");
    return -1;
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

  /* Init debug u(s)art ) first to be able to output error messages early */
  #if DEBUG_FEATURES > 0 && DEBUG_DEBUGIO == 0
     dev_idx = AddDevice(&HW_DEBUG_UART, DebugAssignUart, DebugDeAssignUart);
     DeviceInitByIdx(dev_idx, (void *)1);
  #endif

  /* Init IO_DEV */
  dev_idx = AddDevice(&HW_IO,NULL ,NULL);
  DeviceInitByIdx(dev_idx, NULL);

}

/**
  * @}
  */

