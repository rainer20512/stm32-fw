/**
  ******************************************************************************
  * @file    config.c
  * @author  Rainer 
  * @brief   Handle the configuration
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup config
  * @{
  */
#include "config/config.h"
#include "config/devices_config.h"
#include "system/periodic.h"
#include "debug_helper.h"
#include "eeprom.h"
#include "timer.h"
#if defined(USER_CLOCKCONFIG)
    #include "system/clockconfig.h"
#endif

#if USE_EEPROM_EMUL > 0
    #include "eeprom_emul_conf.h"
    #include "eeprom_emul.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define config_raw ((uint8_t *) &config)
#define eeprom_base_addr    0x0060

/* Private variables ---------------------------------------------------------*/

const EE_LimitsT eelimits[]= EELIMITS;   /* Array on default, min, max values */
            
static uint32_t ee_element_num;          /* number of elements in settings array */
static __IO uint32_t ErasingOnGoing = 0; /* flag for "erase operation in progress" */
uint8_t  bEeConfigValid;                 /* flag for "eeprom emulation is valid    */

#if USE_EEPROM_EMUL > 0
    static uint8_t ee_cleanup_required =  0; /* flag for "cleanup is required"    */
    bool eeprom_read_config_byte  (uint8_t cfg_idx, uint8_t *ret);
    bool eeprom_write_config_byte (uint8_t cfg_idx, uint8_t val);
    bool eeprom_update_config_byte(uint8_t cfg_idx, uint8_t newval);
#else
    #define eeprom_update_config_byte(a,b)  true
#endif

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
EE_ConfigT config;

/* Exported functions --------------------------------------------------------*/



void Config_Dump(void)
{
   uint32_t i;
   const char *cp;
   const EE_LimitsT *ptr;

   if ( !bEeConfigValid ) {
       DEBUG_PUTS("Emulated EEPROM invalid - No EEPROM persistence for config Data!");
   }

   DEBUG_PUTS("Config settings, all values are HEX!");
   DEBUG_PUTS("No. Actual  Minimum  Maximum  Help");
   ptr = eelimits;
   for ( i = 0; i < ee_element_num; i++ ) {
      DEBUG_PRINTF(" %02x     %02x       %02x       %02x  ",i, *(config_raw+i), ptr->min, ptr->max);
      /* Replace \n with blank */
      cp = ptr->help;
      while ( *cp ) {
        DEBUG_PUTC(*cp=='\n' ? ' ' : *cp );
        cp++;
      }
      DEBUG_PUTC('\n');  
      ptr++;
   }
}

bool Config_SetVal(uint8_t idx, uint8_t newval)
{
    /* check validity of index */
    if ( idx >= ee_element_num ) return false;

    /* check validity of value */
    const EE_LimitsT *ptr = eelimits + idx;
    if ( newval < ptr->min || newval > ptr->max ) return false;

    /* all checks ok, so update in EEPROM, except force-reset, which is only updated in RAM */
    if ( idx != EEPROM_FORCE_RESET_IDX ) {
       if ( !eeprom_update_config_byte(idx, newval ) ) return false;
    }

    /* Update in RAM */
    *(config_raw+idx) = newval;

    return true;
}

uint8_t Config_GetVal(uint8_t idx)
{
    /* check validity of index */
    if ( idx >= ee_element_num ) 
        return 0;
    else
        return *(config_raw+idx);
}

/******************************************************************************
 * return a complete EEprom setting element, consisting of
 * actual value
 * minimum allowed value
 * maximum allowed value
 * helptext
 * \note the "deflt" field of EE_LimitsT return var is abused to store the
 *       actual value!
 *****************************************************************************/
bool Config_GetValMinMax(uint8_t idx, EE_LimitsT *ret )
{
    /* check validity of index */
    if ( idx >= ee_element_num ) 
        return false;
    else {
        *ret  = eelimits[idx];
        ret->deflt = *(config_raw+idx);
        return true;
    }
}


uint32_t Config_GetCnt(void)
{
   /* check sanity of emulated eeprom */
   if ( !bEeConfigValid ) {
       DEBUG_PUTS("Invalid emulated EEPROM configuration!");
       return 0;
   }    
    return ee_element_num;
}

/******************************************************************************
 * at second 10 check for reset request via config byte
 *****************************************************************************/
static void configCheckForReset ( void *arg )
{
    UNUSED(arg);
    if ( config.ForceReset )  TimerWatchdogReset(10);
}

static void configInit(void)
{
  uint32_t i;
  /* Check that both limits and settings are of equal size */
  #if DEBUG_MODE > 0
    if ( sizeof(eelimits) / sizeof(EE_LimitsT) != sizeof(config) )
        DEBUG_PRINTF("config limits and config settings are of different size!");
  #endif
  ee_element_num = MIN(sizeof(eelimits) / sizeof(EE_LimitsT), sizeof(config));

  /* Read default settings */
  for ( i=0; i< ee_element_num; i++ )
    *(config_raw+i) = eelimits[i].deflt;

  #if USE_EEPROM_EMUL > 0
      if ( bEeConfigValid ) {
          uint8_t ret;
          /* Read eeprom settings */
          for ( i=0; i< ee_element_num; i++ )
            if ( eeprom_read_config_byte(i, &ret ) )
               *(config_raw+i) = ret;
      }
  #endif  
}

#if USE_EEPROM_EMUL > 0

/******************************************************************************
 * Before write starts, check for older write errors ( which can occur due to
 * program errors ( when trying to write to flash region ))
 * then unlock flash
 *****************************************************************************/
static void eeprom_prepare_write(void)
{
    EE_PrepareWrite();
}

/******************************************************************************
 * Lock flash again after write/page erases
 *****************************************************************************/
static void eeprom_finalize_write(void)
{
    EE_FinalizeWrite();
}

bool eeprom_read_config_byte(uint8_t cfg_idx, uint8_t *ret)
{
  EE_Status status = EE_ReadVariable8bits(eeprom_base_addr+cfg_idx, ret);
  return !(status & EE_STATUSMASK_ERROR);
}

/*!
 *******************************************************************************
 * Write a config byte
 * To avoid unneccessary write operations, it should be checked before, that the
 * new value differs from the stored one
 ******************************************************************************/
bool eeprom_write_config_byte(uint8_t cfg_idx, uint8_t val)
{
  /* Wait any cleanup is completed before accessing flash again */
  if ( ErasingOnGoing == 1 ) {
    DEBUG_PRINTTS("Start wait for Sector erase\n");
    while (ErasingOnGoing == 1);
    DEBUG_PRINTTS("Finished wait for Sector erase\n");
  }

  /* Check for write errors (due to programming errors ) and enable write */
  eeprom_prepare_write();

  EE_Status ee_status = EE_WriteVariable8bits(eeprom_base_addr+cfg_idx, val);
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) ee_cleanup_required = 1;
  eeprom_finalize_write();

  return !(ee_status & EE_STATUSMASK_ERROR);
}

/*!
 *******************************************************************************
 * Update a config byte: 
 * Write only, if value has changes or not stored before
 ******************************************************************************/
bool eeprom_update_config_byte(uint8_t cfg_idx, uint8_t newval)
{
   /* check sanity of emulated EEPROM first */
   if ( !bEeConfigValid ) {
       DEBUG_PUTS("Invalid emulated EEPROM configuration!");
       return false;
   }    
    uint8_t temp;
    if ( eeprom_read_config_byte(cfg_idx, &temp) && temp == newval ) return true;

    return eeprom_write_config_byte(cfg_idx, newval);
}

/*!
 *******************************************************************************
 * When a cleanup is required, this is not done immediately, but signaled by
 * variable "ee_cleanup_required"
 * This variable should be checked periodically and a cleanup initialized, when
 * set. The more eeprom writes are done, the more often this check has to be
 * performed.
 ******************************************************************************/
void eeprom_check_cleanup ( void *arg )
{
    UNUSED(arg);

    EE_Status ee_status;
    if ( ee_cleanup_required ) {
        ee_cleanup_required = 0;
        ErasingOnGoing = 1;
        eeprom_prepare_write();
        ee_status = EE_CleanUp_IT();
        if ((ee_status & EE_STATUSMASK_ERROR) )  {
            DEBUG_PUTS("EEPROM cleanup failed");
            ErasingOnGoing = 0;
        }
 
    }
}

static EE_Status ee_initstatus;  /* return status of EE_Init */

/* Virtual address Tab defined by the user: 0x0000 and 0xFFFF values are prohibited */
static uint16_t VirtAddVarTab[NB_OF_VARIABLES];

#if DEBUG_EEPROM_EMUL > 0
      static uint32_t VarDataTab[NB_OF_VARIABLES] = {0}; /* Used in Config_Test */
#endif
/**
  * @brief  Clean Up end of operation interrupt callback.
  * @param  None
  * @retval None
  */

static void EE_EndOfCleanup_UserCallback(void)
{
  ErasingOnGoing = 0;
  eeprom_finalize_write();
}

/**
  * @brief  FLASH end of operation interrupt callback.
  * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
  *                  Mass Erase: Bank number which has been requested to erase
  *                  Page Erase: Page which has been erased
  *                    (if 0xFFFFFFFF, it means that all the selected pages have been erased)
  *                  Program: Address which was selected for data program
  * @retval None
  */
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  /* Call CleanUp callback when all requested pages have been erased */
  if (ReturnValue == 0xFFFFFFFF) {
    EE_EndOfCleanup_UserCallback();
  }
}

#endif

void Config_Init(void)
{

  /* Assume emulated eeprom config is invalid as long as not positively validated */
  bEeConfigValid = false;

  #if USE_EEPROM_EMUL > 0

      ee_initstatus = EE_OK;
      /* Enable and set FLASH Interrupt priority */
      /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
      HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(FLASH_IRQn);


    #if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
      /* Clear OPTVERR bit and PEMPTY flag if set*/
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR) != RESET) 
      {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
      }
  
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != RESET) 
      {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY); 
      }
    #endif /* defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx) */
 
      /* Configure Programmable Voltage Detector (PVD) (optional) */
      /* PVD interrupt is used to suspend the current application flow in case
         a power-down is detected, allowing the flash interface to finish any
         ongoing operation before a reset is triggered. */
      #if defined(EEPROM_PVDLEVEL)
        EE_PVD_Config(EEPROM_PVDLEVEL);
      #endif  
    
      /* Set user List of Virtual Address variables: 0x0000 and 0xFFFF values are prohibited */
      for (uint32_t VarValue = 0; VarValue < NB_OF_VARIABLES; VarValue++)
      {
        VirtAddVarTab[VarValue] = (uint16_t)(VarValue + 1);
      }

      /* Unlock the Flash Program Erase controller */
      EE_PrepareWrite();

      /* Set EEPROM emulation firmware to erase all potentially incompletely erased
         pages if the system came from an asynchronous reset. Conditional erase is
         safe to use if all Flash operations where completed before the system reset */
      if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
      {    
        /* System reset comes from a power-on reset: Forced Erase */
        /* Initialize EEPROM emulation driver (mandatory) */
        ee_initstatus = EE_Init(VirtAddVarTab, EE_FORCED_ERASE);
        /* Debug output not initialized at this point, so just do termination work */
        if(ee_initstatus != EE_OK) goto ee_terminate;
      }
      else
      {    
        /* System reset comes from a STANDBY wakeup: Conditional Erase*/
        /* Initialize EEPROM emulation driver (mandatory) */
        ee_initstatus = EE_Init(VirtAddVarTab, EE_CONDITIONAL_ERASE);
        /* Debug output not initialized at this point, so just return */
        if(ee_initstatus != EE_OK) goto ee_terminate;
      }
  
      /* Lock the Flash Program Erase controller */
      EE_FinalizeWrite();

      /* Flag a correctly initialized emulated eeprom */
      bEeConfigValid = true;

      /* Schedule a periodic check for eeprom cleanup */
      AtSecond(29, eeprom_check_cleanup, (void *)0, "EEPROM emul check for cleanup");
  #endif

ee_terminate:
  /* Schedule the check for an reset request */
  AtSecond(10, configCheckForReset, (void *)0, "Check reset request");

  /* Init the configuration settings */
  configInit();

}

#if DEBUG_EEPROM_EMUL > 0
    bool Config_Test(void)
    {
      EE_Status ee_status = EE_OK;
      uint32_t VarValue = 0;
      uint32_t Index = 0;
      bool ret = true;
  
      /* Store 10 values of all variables in EEPROM, ascending order */
      for (VarValue = 1; VarValue <= 10; VarValue++)
      {
        for (Index = 0; Index < NB_OF_VARIABLES; Index++)
        {
          /* Wait any cleanup is completed before accessing flash again */
          if ( ErasingOnGoing == 1 ) {
            DEBUG_PRINTTS("Start wait for Sector erase\n");
            while (ErasingOnGoing == 1);
            DEBUG_PRINTTS("Finished wait for Sector erase\n");
          }
          ee_status = EE_WriteVariable32bits(VirtAddVarTab[Index], Index*VarValue);
          ee_status|= EE_ReadVariable32bits(VirtAddVarTab[Index], &VarDataTab[Index]);
          if (Index*VarValue != VarDataTab[Index]) { 
            printf("eeprom virt addr %04x: returned value %d != expected value %d\n", VirtAddVarTab[Index], VarDataTab[Index], Index*VarValue);
            ret = false;
          }

          /* Start cleanup IT mode, if cleanup is needed */
          if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
          if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {
            puts("Error on EE_CleanUp_IT");
            ret = false;
          }
        }
      }

      /* Read all the variables */
      for (Index = 0; Index < NB_OF_VARIABLES; Index++)
      {
        ee_status = EE_ReadVariable32bits(VirtAddVarTab[Index], &VarValue);
        if (ee_status != EE_OK) {
            puts("Error on ReadVariable");
            ret = false;
        }
        if (VarValue != VarDataTab[Index]) {
            printf("eeprom read:virt addr %04x: returned value %d != expected value %d\n", VirtAddVarTab[Index], VarValue, VarDataTab[Index]);
            ret = false;
        }
      }

      /* Store 1000 values of Variable1,2,3 in EEPROM */
      for (VarValue = 1; VarValue <= 1000; VarValue++)
      {
        if (ErasingOnGoing == 1) {
            DEBUG_PRINTTS("Start wait for Sector erase\n");
            while (ErasingOnGoing == 1);
            DEBUG_PRINTTS("Finished wait for Sector erase\n");
        }

        ee_status = EE_WriteVariable32bits(VirtAddVarTab[0], VarValue);
        ee_status|= EE_ReadVariable32bits(VirtAddVarTab[0], &VarDataTab[0]);
        if (VarValue != VarDataTab[0]) {
            puts("Error on write1000 index 0");
            ret = false;
        }

        ee_status|= EE_WriteVariable32bits(VirtAddVarTab[1], ~VarValue);
        ee_status|= EE_ReadVariable32bits(VirtAddVarTab[1], &VarDataTab[1]);
        if (~VarValue != VarDataTab[1]) {
            puts("Error on write1000 index 1");
            ret = false;
        }

        ee_status|= EE_WriteVariable32bits(VirtAddVarTab[2], VarValue << 1);
        ee_status|= EE_ReadVariable32bits(VirtAddVarTab[2], &VarDataTab[2]);
        if ((VarValue << 1) != VarDataTab[2]) {
            puts("Error on write1000 index 2");
            ret = false;
        }

        /* Start cleanup polling mode, if cleanup is needed */
        if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 0;ee_status|= EE_CleanUp();}
        if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {
            puts("Error in EE_CleanUp");
            ret = false;
        }
      }

      /* Read all the variables, second pass */
      for (Index = 0; Index < NB_OF_VARIABLES; Index++)
      {
        ee_status = EE_ReadVariable32bits(VirtAddVarTab[Index], &VarValue);
        if (ee_status != EE_OK) {
            puts("Error on ReadVariable, second pass");
            ret = false;
        }
        if (VarValue != VarDataTab[Index]) {
            printf("ReadVariable, second pass: virt addr %04x: returned value %d != expected value %d\n", VirtAddVarTab[Index], VarValue, VarDataTab[Index]);
            ret = false;
        }
      }

      /* Test is completed successfully */
      /* Lock the Flash Program Erase controller */
      HAL_FLASH_Lock();

      return ret;
    }
#endif //if DEBUG_EEPROM_EMUL > 0