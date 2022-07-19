/**
  ******************************************************************************
  * @file    "config/sdmmc_config.h" 
  * @author  Rainer
  * @brief   Definition of SDMMC devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration items, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDMMC_CONFIG_H
#define __SDMMC_CONFIG_H

#include "hardware.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for SDMMC1 resources, no alternative assignments possible here
 * Function                     |   Pin
 *------------------------------+------------------------------
 * D0, D1, D2, D3               : PC8/AF12,  PC9/AF12,  PC10/AF12, PC11/AF12
 * D4, D5, D6, D7               : PB8/AF12,  PB9/AF12,  PC6/AF12,  PC7/AF12
 * CLK, CMD                     : PC12/AF12, PD2/AF12
 *
 * Definition for SDMMC2 resources, no alternative assignments possible here
 * Function                     |   Pin
 *------------------------------+------------------------------
 * D0, D1, D2, D3               : PB14/AF9,  PB15/AF9,  PB3/AF9,  PB4/AF9
 * D4, D5, D6, D7               : PB8/AF10,  PB9/AF10,  PC6/AF10, PC7/AF10
 * CLK, CMD                     : PD6/AF11,  PD7/AF11
 * CLK, CMD                     : PC1/AF9,   PA0/AF11
 

 *------------------------------+------------------------------
*/

#if USE_SDMMC > 0

    #if USE_SDMMC1 > 0
        #define SDMMC1_CK                         { GPIO_PIN_12, GPIOC, GPIO_AF12_SDMMC1, GPIO_PULLUP, "SDIO1 CK"  }
        #define SDMMC1_CMD                        { GPIO_PIN_2,  GPIOD, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 CMD"  }
        #define SDMMC1_D0                         { GPIO_PIN_8,  GPIOC, GPIO_AF12_SDMMC1, GPIO_PULLUP, "SDIO1 D0"  }
        #define SDMMC1_D1                         { GPIO_PIN_9,  GPIOC, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D1"  }
        #define SDMMC1_D2                         { GPIO_PIN_10, GPIOC, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D2"  }
        #define SDMMC1_D3                         { GPIO_PIN_11, GPIOC, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D3"  }
        #define SDMMC1_D4                         { GPIO_PIN_8,  GPIOB, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D4"  }
        #define SDMMC1_D5                         { GPIO_PIN_9,  GPIOB, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D5"  }
        #define SDMMC1_D6                         { GPIO_PIN_6,  GPIOC, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D6"  }
        #define SDMMC1_D7                         { GPIO_PIN_7,  GPIOC, GPIO_AF12_SDMMC1, GPIO_NOPULL, "SDIO1 D7"  }


        /* Definition for SDMMC1 NVIC */
        #if defined(SDMMC1_USE_IRQ)
                #define SDMMC1_IRQ                        { SDMMC1_IRQn, SDMMC1_IRQ_PRIO, 0    }
                #define SDMMC1_IRQHandler                 SDMMC1_IRQHandler
        #endif
    #endif // USE_SDMMC1 > 0 

    #if USE_SDMMC2 > 0
        #define SDMMC2_CK                         { GPIO_PIN_6,  GPIOD, GPIO_AF11_SDMMC2, GPIO_NOPULL, "SDIO2 CK"  }
        #define SDMMC2_CMD                        { GPIO_PIN_7,  GPIOD, GPIO_AF11_SDMMC2, GPIO_NOPULL, "SDIO2 CMD"  }
        #define SDMMC2_CK                         { GPIO_PIN_1,  GPIOC, GPIO_AF9_SDMMC2,  GPIO_NOPULL, "SDIO2 CK"  }
        #define SDMMC2_CMD                        { GPIO_PIN_0,  GPIOA, GPIO_AF11_SDMMC2, GPIO_NOPULL, "SDIO2 CMD"  }
        #define SDMMC2_D0                         { GPIO_PIN_14, GPIOB, GPIO_AF9_SDMMC2,  GPIO_NOPULL, "SDIO2 D0"  }
        #define SDMMC2_D1                         { GPIO_PIN_15, GPIOB, GPIO_AF9_SDMMC2,  GPIO_NOPULL, "SDIO2 D1"  }
        #define SDMMC2_D2                         { GPIO_PIN_3,  GPIOB, GPIO_AF9_SDMMC2,  GPIO_NOPULL, "SDIO2 D2"  }
        #define SDMMC2_D3                         { GPIO_PIN_4,  GPIOB, GPIO_AF9_SDMMC2,  GPIO_NOPULL, "SDIO2 D3"  }
        #define SDMMC2_D4                         { GPIO_PIN_8,  GPIOB, GPIO_AF10_SDMMC2, GPIO_NOPULL, "SDIO2 D4"  }
        #define SDMMC2_D5                         { GPIO_PIN_9,  GPIOB, GPIO_AF10_SDMMC2, GPIO_NOPULL, "SDIO2 D5"  }
        #define SDMMC2_D6                         { GPIO_PIN_6,  GPIOC, GPIO_AF10_SDMMC2, GPIO_NOPULL, "SDIO2 D6"  }
        #define SDMMC2_D7                         { GPIO_PIN_7,  GPIOC, GPIO_AF10_SDMMC2, GPIO_NOPULL, "SDIO2 D7"  }


        /* Definition for SDMMC1 NVIC */
        #if defined(SDMMC2_USE_IRQ)
                #define SDMMC2_IRQ                        { SDMMC2_IRQn, SDMMC2_IRQ_PRIO, 0    }
                #define SDMMC2_IRQHandler                 SDMMC2_IRQHandler
        #endif
    #endif // USE_SDMMC1 > 0 

    
#endif // USE_SDMMC > 0

#endif /* __SDMMC_CONFIG_H */

