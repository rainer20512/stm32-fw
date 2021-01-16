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

    /* Don't change the sequence, has to be in the order D00 ... D15 */
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
    #define FMC_D_MAX                        16

    /* Don't change the sequence, has to be in the order A00 ... A25 */
    #define FMC_A00                         { GPIO_PIN_0,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A00"  }
    #define FMC_A01                         { GPIO_PIN_1,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A01"  }
    #define FMC_A02                         { GPIO_PIN_2,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A02"  }
    #define FMC_A03                         { GPIO_PIN_3,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A03"  }
    #define FMC_A04                         { GPIO_PIN_4,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A04"  }
    #define FMC_A05                         { GPIO_PIN_5,  GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A05"  }
    #define FMC_A06                         { GPIO_PIN_12, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A06"  }
    #define FMC_A07                         { GPIO_PIN_13, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A07"  }
    #define FMC_A08                         { GPIO_PIN_14, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A08"  }
    #define FMC_A09                         { GPIO_PIN_15, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A09"  }
    #define FMC_A10                         { GPIO_PIN_0,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A10"  }
    #define FMC_A11                         { GPIO_PIN_1,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A11"  }
    #define FMC_A12                         { GPIO_PIN_2,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A12"  }
    #define FMC_A13                         { GPIO_PIN_3,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A13"  }
    #define FMC_A14                         { GPIO_PIN_4,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A14"  }
    #define FMC_A15                         { GPIO_PIN_5,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A15"  }
    #define FMC_A16                         { GPIO_PIN_11, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A16"  }
    #define FMC_A17                         { GPIO_PIN_12, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A17"  }
    #define FMC_A18                         { GPIO_PIN_13, GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A18"  }
    #define FMC_A19                         { GPIO_PIN_3,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A19"  }
    #define FMC_A20                         { GPIO_PIN_4,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A20"  }
    #define FMC_A21                         { GPIO_PIN_5,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A21"  }
    #define FMC_A22                         { GPIO_PIN_6,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A22"  }
    #define FMC_A23                         { GPIO_PIN_2,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A23"  }
    #define FMC_A24                         { GPIO_PIN_13, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A24"  }
    #define FMC_A25                         { GPIO_PIN_14, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC A25"  }
    #define FMC_A_MAX                       26

    /* Order of control pins in the following section */ 
    #define FMC_CLK_OFS                     0
    #define FMC_NWAIT_OFS                   1
    #define FMC_NOE_OFS                     2
    #define FMC_NWE_OFS                     3
    #define FMC_NE1_OFS                     4
    #define FMC_NE2_OFS                     5
    #define FMC_NE3_OFS                     6
    #define FMC_NE4_OFS                     7
    #define FMC_NBL0_OFS                    8
    #define FMC_NBL1_OFS                    9
    #define FMC_INT_OFS                     10
    #define FMC_NL_OFS                      11

    /* Last entry specifies number of CTL entries */
    #define FMC_CTL_MAX                     12    
    #define FMC_CTL_STR                     {"Clk", "NWait", "NOE", "NWE", "NE1", "NE2", "NE3", "NE4", "NBl0", "NBl1", "Int", "NL"}

    /* The order here ha so match the offset definitions above */
    #define FMC_CTL_CLK                     { GPIO_PIN_3,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC Clk"  }
    #define FMC_CTL_NWAIT                   { GPIO_PIN_6,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NWait"}
    #define FMC_CTL_NOE                     { GPIO_PIN_4,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NOE"  }
    #define FMC_CTL_NWE                     { GPIO_PIN_5,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NWE"  }
    #define FMC_CTL_NE1                     { GPIO_PIN_7,  GPIOD, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE1"  }
//  #define FMC_CTL_NE1                     { GPIO_PIN_7,  GPIOC, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE1"  }
    #define FMC_CTL_NE2                     { GPIO_PIN_9,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE2"  }
//  #define FMC_CTL_NE2                     { GPIO_PIN_8,  GPIOC, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE2"  }
//  #define FMC_CTL_NE3                     { GPIO_PIN_7,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE3"  }
    #define FMC_CTL_NE3                     { GPIO_PIN_10, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE3"  }
    #define FMC_CTL_NE4                     { GPIO_PIN_12, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NE4"  }
    #define FMC_CTL_NBL0                    { GPIO_PIN_0,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL1" }
    #define FMC_CTL_NBL1                    { GPIO_PIN_1,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL0" }
    #define FMC_CTL_INT                     { GPIO_PIN_7,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC INT"  }
    #define FMC_CTL_NL                      { GPIO_PIN_7,  GPIOB, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NL"  }


    /* Definition for FMC NVIC */
    #if defined(FMC_USE_IRQ)
            #define FMC_IRQ                        { FMC_IRQn, FMC_IRQ_PRIO, 0    }
            #define FMC_IRQHandler                 FMC_IRQHandler
    #endif

    /* SRAM timimg for CY62157EV30 MoBL SRAM with multiplexed adress/data line and 74hc373 address latches */
    /* very, very slow!!                                                                                   */

    /* Number of address bits                                                                              */
    #define ADDWID3                         24

    /* Address setup time is the 74hc373 minimum pulsewidth ( worst case ) in ns                           */
    #define ADDSET3                         2

    /* Address hold time is 74hc737 propagation delay plus output transistion time                         */
    #define ADDHLD3                         1

    /* Data setup time is the tDOE value from the CY62157EV30 datasheet                                    */
    #define DATASET3                        1

    #define ACCMODE3                        FMC_ACCESS_MODE_A

    #define TIMING3                         { ACCMODE3, ADDWID3, ADDSET3, ADDHLD3, DATASET3, }

    #if 0
        /* SRAM timimg for CY62157EV30 MoBL SRAM with multiplexed adress/data line and 74hc373 address latches */
        /* very, very slow!!                                                                                   */

        /* Number of address bits                                                                              */
        #define ADDWID1                         24

        /* Address setup time is the 74hc373 minimum pulsewidth ( worst case ) in ns                           */
        #define ADDSET1                         60
    
        /* Address hold time is 74hc737 propagation delay plus output transistion time                         */
        #define ADDHLD1                         100

        /* Data setup time is the tDOE value from the CY62157EV30 datasheet                                    */
        #define DATASET1                        50

        #define ACCMODE1                        FMC_ACCESS_MODE_D

        #define TIMING1                         { ACCMODE1, ADDWID1, ADDSET1, ADDHLD1, DATASET1, }
    #endif

#endif // USE_FMC 

#endif /* __FMC_CONFIG_H */

