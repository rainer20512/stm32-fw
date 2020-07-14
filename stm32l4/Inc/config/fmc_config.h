/**
  ******************************************************************************
  * @file    "config/fmc_config.h" 
  * @author  Rainer
  * @brief   Definition of FMC devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration items, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FMC_CONFIG_H
#define __FMC_CONFIG_H

#include "hardware.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for FMC resources, no alternative assignments possible here
 * Function                     |   Pin
 *------------------------------+------------------------------
 * D00, D01, D02, D03           : PD14, PD15, PD0,  PD1 
 * D04, D05, D06, D07           : PE7,  PE8,  PE9,  PE10
 * D08, D09, D10, D11           : PE11, PE12, PE13, PE14
 * D12, D13, D14, D15           : PE15, PD8,  PD9,  PD10
 * A00, A01, A02, A03           : PF0,  PF1,  PF2,  PF3
 * A04, A05, A06, A07           : PF4,  PF5,  PF11, PF12
 * A08, A09, A10, A11           : PF14, PF15, PG0,  PG1
 * A12, A13, A14, A15           : PG2,  PG3,  PG4,  PG5
 * A16, A17, A18, A19           : PD11, PD12, PD13, PE3
 * A20, A21, A22, A23           : PE4,  PE5,  PE6,  PE2
 * A24, A25, A22, A23           : PG13, PG14
 * CLK, NWAIT, NOE, NWE         : PD3,  PD4,  PD5,  PD6
 * NE1, NE2, NE3, NE4           : PD7,  PG9,  PG10, PG12
 * NBL0, NBL1, INT              : PE0,  PE1,  PG7
 *------------------------------+------------------------------
*/

#if USE_FMC > 0

#if USE_FMC_MUXED > 0 
    #define FMC_DA00                         { GPIO_PIN_14, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA00"  }
    #define FMC_DA01                         { GPIO_PIN_15, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA01"  }
    #define FMC_DA02                         { GPIO_PIN_0,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA02"  }
    #define FMC_DA03                         { GPIO_PIN_1,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA03"  }
    #define FMC_DA04                         { GPIO_PIN_7,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA04"  }
    #define FMC_DA05                         { GPIO_PIN_8,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA05"  }
    #define FMC_DA06                         { GPIO_PIN_9,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA06"  }
    #define FMC_DA07                         { GPIO_PIN_10, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA07"  }
    #define FMC_DA08                         { GPIO_PIN_11, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA08"  }
    #define FMC_DA09                         { GPIO_PIN_12, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA09"  }
    #define FMC_DA10                         { GPIO_PIN_13, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA10"  }
    #define FMC_DA11                         { GPIO_PIN_14, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA11"  }
    #define FMC_DA12                         { GPIO_PIN_15, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA12"  }
    #define FMC_DA13                         { GPIO_PIN_8,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA13"  }
    #define FMC_DA14                         { GPIO_PIN_9,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA14"  }
    #define FMC_DA15                         { GPIO_PIN_10, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC DA15"  }
#else
    #define FMC_D00                         { GPIO_PIN_14, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D00"  }
    #define FMC_D01                         { GPIO_PIN_15, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D01"  }
    #define FMC_D02                         { GPIO_PIN_0,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D02"  }
    #define FMC_D03                         { GPIO_PIN_1,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D03"  }
    #define FMC_D04                         { GPIO_PIN_7,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D04"  }
    #define FMC_D05                         { GPIO_PIN_8,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D05"  }
    #define FMC_D06                         { GPIO_PIN_9,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D06"  }
    #define FMC_D07                         { GPIO_PIN_10, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D07"  }
    #define FMC_D08                         { GPIO_PIN_11, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D08"  }
    #define FMC_D09                         { GPIO_PIN_12, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D09"  }
    #define FMC_D10                         { GPIO_PIN_13, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D10"  }
    #define FMC_D11                         { GPIO_PIN_14, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D11"  }
    #define FMC_D12                         { GPIO_PIN_15, GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D12"  }
    #define FMC_D13                         { GPIO_PIN_8,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D13"  }
    #define FMC_D14                         { GPIO_PIN_9,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D14"  }
    #define FMC_D15                         { GPIO_PIN_10, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D15"  }

    #define FMC_A00                         { GPIO_PIN_0,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A00"  }
    #define FMC_A01                         { GPIO_PIN_1,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A01"  }
    #define FMC_A02                         { GPIO_PIN_2,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A02"  }
    #define FMC_A03                         { GPIO_PIN_3,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A03"  }
    #define FMC_A04                         { GPIO_PIN_4,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A04"  }
    #define FMC_A05                         { GPIO_PIN_5,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A05"  }
    #define FMC_A06                         { GPIO_PIN_12,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A06"  }
    #define FMC_A07                         { GPIO_PIN_13, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A07"  }
    #define FMC_A08                         { GPIO_PIN_14, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A08"  }
    #define FMC_A09                         { GPIO_PIN_15, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A09"  }
    #define FMC_A10                         { GPIO_PIN_0,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A10"  }
    #define FMC_A11                         { GPIO_PIN_1,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A11"  }
    #define FMC_A12                         { GPIO_PIN_2,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A12"  }
    #define FMC_A13                         { GPIO_PIN_3,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A13"  }
    #define FMC_A14                         { GPIO_PIN_4,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A14"  }
    #define FMC_A15                         { GPIO_PIN_5,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A15"  }
#endif

    #define FMC_A16                         { GPIO_PIN_11, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A16"  }
    #define FMC_A17                         { GPIO_PIN_12, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A17"  }
    #define FMC_A18                         { GPIO_PIN_13, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A18"  }
    #define FMC_A19                         { GPIO_PIN_3,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A19"  }
    #define FMC_A20                         { GPIO_PIN_4,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A20"  }
    #define FMC_A21                         { GPIO_PIN_5,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A21"  }
    #define FMC_A22                         { GPIO_PIN_6,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A22"  }
    #define FMC_A23                         { GPIO_PIN_2,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A23"  }
    #define FMC_A24                         { GPIO_PIN_13, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A24"  }
    #define FMC_A25                         { GPIO_PIN_14, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A25"  }


	/* Definition for FMC NVIC */
	#if defined(FMC_USE_IRQ)
		#define FMC_IRQ                        { FMC_IRQn, FMC_IRQ_PRIO, 0    }
		#define FMC_IRQHandler                 FMC_IRQHandler
	#endif


#endif // USE_FMC 

#endif /* __FMC_CONFIG_H */

