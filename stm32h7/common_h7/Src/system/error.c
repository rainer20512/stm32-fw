#include "config/config.h"
#include "hardware.h"
#include "debug_helper.h"
#include "system/tm1637.h"

uint32_t My_Delay(uint32_t waittime )
{
  uint32_t tickstart = 0U;
  uint32_t upcount = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < waittime) {
    upcount++;
  }
  return upcount;
}

void Error_Handler(char *file, int line)
{
  /* Turn LED2 on */
  // BSP_LED_On(LED2);

  DEBUG_PRINTF("Error in %s line %d\n", file, line);
  
  while(1)
  {
    /* Error if LED2 is slowly blinking (1 sec. period) */
    // BSP_LED_Toggle(LED2); 
    DEBUG_PRINTF("Increments per second=%u\n",My_Delay(1000)); 
  }  
}

void Error_Handler_XX(int32_t code, char *file, int line)
{
    TM1637_displayInteger(code,0,99);
    Error_Handler( file, line );
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
