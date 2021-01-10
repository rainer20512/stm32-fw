/*
 ******************************************************************************
 * @file    hw_device.h
 * @author  Rainer
 * @brief   Abstract description of a hardware device
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_DEVICE_H
#define __HW_DEVICE_H

#include "config/config.h"
#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Forward declarations -----------------------------------------------------*/
typedef struct  HWDTS HW_DeviceType;

/******************************************************************************
 * defines for different types of hardware, don't use enum here, because
 * these defines are used in preprocessor conditionals! **** 005 ****
 *****************************************************************************/

#define     HW_DEVICE_UNKNOWN     0
#define     HW_DEVICE_IO          1
#define     HW_DEVICE_PWM         2
#define     HW_DEVICE_UART        3
#define     HW_DEVICE_I2C         4
#define     HW_DEVICE_HWSPI       5
#define     HW_DEVICE_BBSPI       6
#define     HW_DEVICE_ADC         7
#define     HW_DEVICE_QENC        8
#define     HW_DEVICE_PWMTIMER    9
#define     HW_DEVICE_BASETIMER   10
#define     HW_DEVICE_QSPI        11
#define     HW_DEVICE_CAN         12
#define     HW_DEVICE_ETH         13
#define     HW_DEVICE_USBD        14
#define     HW_DEVICE_FMC         15


/******************************************************************************
 * Select normal or inverted outpiut driver mode
 *****************************************************************************/
typedef enum {
    HW_IO_NORMAL        = 0,        
    HW_IO_INVERTED      = 1,
} HW_PinDrvEnum;

/******************************************************************************
 * Select Initial state of output pin
 *****************************************************************************/
typedef enum {
    HW_OUTPUT_LOW       = 0,        
    HW_OUTPUT_HIGH      = 1,
    HW_INPUT            = 2,
} HW_PinOutInitial;

/******************************************************************************
 * describe an interrupt for Hardware Devices
 *****************************************************************************/
typedef struct {
    uint16_t irq_num;            //!< Interrupt position as defined by NVIC 
    uint8_t irq_prio;            //!< Interrupt priority
    uint8_t irq_subprio;         //!< Interrupt subprio 
} HW_IrqType;

/******************************************************************************
 * List of interrupts for Hardware Devices
 *****************************************************************************/
typedef struct {
    uint8_t num;                 //!< Number of interrupts in List > 
    HW_IrqType irq[];            //!< List of interrupts
} HW_IrqList;

/******************************************************************************
 * describe an interrupt for a GPIO pin
 *****************************************************************************/
typedef struct {
    int8_t  irq_prio;            //!< Interrupt priority, -1 means: no interrupt
    uint8_t irq_subprio;         //!< Interrupt subprio 
} GPIO_IrqType;

#define GPIO_NO_IRQ              {-1,0,}

/******************************************************************************
 * describe a pin that is used in plain IO mode
 *****************************************************************************/
typedef struct {
    uint16_t pin;               //!< Pin name as defined in ..._hal_gpio.h
    GPIO_TypeDef *gpio;         //!< GPIO-Port ( GPIOA ... GPIOx )
    uint32_t gpio_mode;         //!< Pin mode as defined in ..._hal_gpio.h
    uint8_t  speed;             //!< speed as defined in ..._hal_gpio.h
    uint8_t  pull;              //!< pull as defined in ..._hal_gpio.h
    HW_PinDrvEnum drv;          //!< on output: drive normal or inverted, ignored on input
    HW_PinOutInitial initial;   //!< Initial output value, ignored on input
    GPIO_IrqType gpio_irq;      //!< associated interrupt ( in case of input pin with irq )
    const char *dbg_name;       //!< Pin function ( for debug purposes only )
} HW_Gpio_IO_Type;

/******************************************************************************
 * describe a pin that is used as ADC analog input
 *****************************************************************************/
typedef struct {
    uint32_t channelID;         //!< ADC channel ID
    uint32_t sampleTime;        //!< ADC channel Sample time
    GPIO_TypeDef *gpio;         //!< GPIO-Port ( GPIOA ... GPIOx ), NULL for internal channels
    uint16_t pin;               //!< associated Pin number as defined in ..._hal_gpio.h
    const char *dbg_name;       //!< Pin function ( for debug purposes only )
} HW_Gpio_ADC_Type;


/******************************************************************************
 * describe a pin that is used in AF mode or anyway special mode, eg BBSPI
 *****************************************************************************/
typedef struct {
    uint16_t pin;               //!< Pin name as defined in ..._hal_gpio-h
    GPIO_TypeDef *gpio;         //!< GPIO-Port ( GPIOA ... GPIOx )
    uint8_t af_mode;            //!< Alternate function as defined in ..._hal_gpio-h
    uint8_t  pull;              //!< pull as defined in ..._hal_gpio.h
    const char *dbg_name;       //!< Pin function ( for debug purposes only )
} HW_Gpio_AF_Type;


typedef struct HWGLAF {
  uint8_t num;
  HW_Gpio_AF_Type gpio[];
} HW_GpioList_AF;

typedef struct HWGLIO {
  uint8_t num;
  HW_Gpio_IO_Type gpio[];
} HW_GpioList_IO;

typedef struct HWGLADC {
  uint8_t num;
  HW_Gpio_ADC_Type gpio[];
} HW_GpioList_ADC;

/******************************************************************************
 * Dynamic PostInit and PreDeInit functions
 *****************************************************************************/
typedef void ( *HW_DynInitT )   ( const HW_DeviceType *, void *args);
typedef void ( *HW_DynDeInitT ) ( const HW_DeviceType *);
typedef struct HWINIT {
  HW_DynInitT   PostInitFn;       /* What to do after static Init Fn execution    */
  HW_DynDeInitT PreDeInitFn;      /* What to do before static DeInit Fn executiob */
} HW_DynInitBlock;

#define HW_DYN_NULL     { NULL, NULL }

/******************************************************************************
 * describe a DMA 
 *****************************************************************************/
typedef struct {
    DMA_HandleTypeDef   *dmaHandle;     //!< DMA handle to use at runtime
#if defined(STM32L476xx) || defined(STM32L496xx)
    DMA_Channel_TypeDef *dmaChannel;     //!< Associated DMA channel / Stream
#elif defined(STM32H745xx) || defined(STM32H742xx) || defined(STM32H743xx)
    void                *dmaChannel;     //!< Associated DMA channel / Stream
                                         //!< may be of type DMA_Stream_TypeDef *
                                         //!< or BDMA_Channel_TypeDef *
#else
    #error "No setup for DMA Channel or Stream"
#endif
    uint8_t              dmaRequest;    //!< Associated DMA Request    
    uint8_t              dmaIrqNum;     //!< IRQ for this channel/request
    uint32_t             dmaPrio;       //!< DMA channel priority as defined in stm32xxxxhal_dma.h
} HW_DmaType;

typedef struct HWDTS {
    bool (*Init)            ( const HW_DeviceType *self );
    void (*DeInit)          ( const HW_DeviceType *self );
    void (*BeforeFrqChange) ( const HW_DeviceType *self );
    bool (*OnFrqChange)     ( const HW_DeviceType *self );
    bool (*AllowStop)       ( const HW_DeviceType *self );
    bool (*OnSleep)         ( const HW_DeviceType *self );
    bool (*OnWakeUp)        ( const HW_DeviceType *self );
    
    const char              *devName;
    uint32_t                devType;
    const void              *devBase;                   /* Peripheral base address */
    const void              *devData;                   /* Additional data         */
    const HW_GpioList_AF    *devGpioAF;
    const HW_GpioList_IO    *devGpioIO;
    const HW_GpioList_ADC   *devGpioADC;
    const HW_IrqList        *devIrqList;
    const HW_DmaType        *devDmaRx;
    const HW_DmaType        *devDmaTx;
} HW_DeviceType;

bool GpioInitHW             ( uint32_t devIdx, GPIO_TypeDef *gpio, GPIO_InitTypeDef *Init );

bool GpioAFInitOneWithPreset( uint32_t devIdx, const HW_Gpio_AF_Type *gpio, GPIO_InitTypeDef *Init, HW_PinOutInitial preset );
bool GpioAFInitOne          ( uint32_t devIdx, const HW_Gpio_AF_Type *gpio, GPIO_InitTypeDef *Init );
bool GpioAFInitAll          ( uint32_t devIdx, const HW_GpioList_AF *gpioList, GPIO_InitTypeDef *Init );
void GpioAFDeInitOne        ( uint32_t devIdx, const HW_Gpio_AF_Type *gpio );
void GpioAFDeInitAll        ( uint32_t devIdx, const HW_GpioList_AF *gpioList );

void HW_SetAllIRQs          ( const HW_IrqList *irqlist, bool bDoEna );

bool HW_IsIrqMode           ( uint32_t mode );
bool HW_IsEvtMode           ( uint32_t mode );
bool HW_IsInputMode         ( uint32_t mode );
bool HW_IsOutputMode        ( uint32_t mode );

bool GpioIOInitAll          ( uint32_t devIdx, const HW_GpioList_IO *gpioList );
void GpioIODeInitAll        ( uint32_t devIdx, const HW_GpioList_IO *gpioList );

bool GpioADCInitAll         ( uint32_t devIdx, const HW_GpioList_ADC *gpioList );
void GpioADCDeInitAll       ( uint32_t devIdx, const HW_GpioList_ADC *gpioList );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HW_DEVICE_H */
