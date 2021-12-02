/**
  ******************************************************************************
  * @file    "config/ltdc_config.h" 
  * @author  Rainer
  * @brief   Definition of LTDC devices, their GPIO pins and other configurations 
  * 
  * The selection of all neccessary devices is done by #ifdef'ing or #undef'ing
  * specific entries. If defining one element, there may be subortinated
  * configuration items, that have to be defined/undefined, too.
  * Check the corresponding ifdef-Block
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LTDC_CONFIG_H
#define __LTDC_CONFIG_H

#include "hardware.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/************************************************************************
 * Definition for LTDC resources, if no AF is stated, AF14 is assigned
 * Func. |   Pin
 *-------+------------------------------
 * R0    | G13, H2, I15
 * R1    | A2, H3, J0
 * R2    | A1, C10, H8, J1
 * R3    | B0/AF9, H9, J2
 * R4    | A5, A11, H10, J3
 * R5    | A9, A12, C0, H11, J4
 * R6    | A8, B1/AF9, H12, J5
 * R7    | E15, G6, J0/AF9, J6
 *
 * G0    | B1, E5, J7
 * G1    | B0, E6, J8
 * G2    | A6, H13, I15/AF9, J9
 * G3    | C9/AF10, E11, G10/AF9, H14, J10, J12/AF9
 * G4    | B10, H4, H15, J11
 * G5    | B11, H4/AF9, I0, K0
 * G6    | C7, I1, I11/AF9, K1
 * G7    | D3, G8, I2, K2
 *
 * B0    | E4, G14, J12
 * B1    | A10, G12, J13
 * B2    | A3/AF9, C9, D6, G10, J14
 * B3    | A8/AF13, D10, G11, J15
 * B4    | A10/AF12, E12, G12/AF9, K3, I4, J13/AF9
 * B5    | A3, I5, K4
 * B6    | B8, I6,K5
 * B7    | B9, I7, K6
 *
 * VSYNC | A4, I9, I13
 * HSYNC | C6, I10, I2 
 * CLK   | E14, G7, I14
 * DE    | E13, F10, K7
 *------------------------------+------------------------------
*/

#if USE_LTDC > 0

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
    #define FMC_D16                         { GPIO_PIN_8,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D16"  }
    #define FMC_D17                         { GPIO_PIN_9,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D17"  }
    #define FMC_D18                         { GPIO_PIN_10, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D18"  }
    #define FMC_D19                         { GPIO_PIN_11, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D19"  }
    #define FMC_D20                         { GPIO_PIN_12, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D20"  }
    #define FMC_D21                         { GPIO_PIN_13, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D21"  }
    #define FMC_D22                         { GPIO_PIN_14, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D22"  }
    #define FMC_D23                         { GPIO_PIN_15, GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D23"  }
    #define FMC_D24                         { GPIO_PIN_0,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D24"  }
    #define FMC_D25                         { GPIO_PIN_1,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D25"  }
    #define FMC_D26                         { GPIO_PIN_2,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D26"  }
    #define FMC_D27                         { GPIO_PIN_3,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D27"  }
    #define FMC_D28                         { GPIO_PIN_6,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D28"  }
    #define FMC_D29                         { GPIO_PIN_7,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D29"  }
    #define FMC_D30                         { GPIO_PIN_9,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D30"  }
    #define FMC_D31                         { GPIO_PIN_10, GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC D31"  }

    #if USE_FMC_SDRAM > 0
        #define FMC_D_MAX                        32
    #else
        #define FMC_D_MAX                        16
    #endif

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
    #define FMC_NBL2_OFS                    10
    #define FMC_NBL3_OFS                    11
    #define FMC_INT_OFS                     12
    #define FMC_NL_OFS                      13
    #define FMC_SDNWE_OFS                   14
    #define FMC_NCAS_OFS                    15
    #define FMC_NRAS_OFS                    16
    #define FMC_SDNE0_OFS                   17
    #define FMC_SDNE1_OFS                   18
    #define FMC_SDCKE0_OFS                  19
    #define FMC_SDCKE1_OFS                  20
    #define FMC_SDCLK_OFS                   21

    /* Last entry specifies number of CTL entries */
    #define FMC_CTL_MAX                     22    
    #define FMC_CTL_STR                     {"Clk", "NWait", "NOE",   "NWE",    "NE1",    "NE2",   "NE3",   "NE4",    "NBl0",   "NBl1", "NBl2", "NBl3", \
                                             "Int", "NL",    "SDNWE", "SDNCAS", "SDNRAS", "SDNE0", "SDNE1", "SDCKE0", "SDCKE1", "SDCLK" }

    /* The order here has to be so, that the offset definitions above are matched */
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
    #define FMC_CTL_NBL0                    { GPIO_PIN_0,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL0" }
    #define FMC_CTL_NBL1                    { GPIO_PIN_1,  GPIOE, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL1" }
    #define FMC_CTL_NBL2                    { GPIO_PIN_4,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL2" }
    #define FMC_CTL_NBL3                    { GPIO_PIN_5,  GPIOI, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NBL3" }
    #define FMC_CTL_INT                     { GPIO_PIN_7,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC INT"  }
    #define FMC_CTL_NL                      { GPIO_PIN_7,  GPIOB, GPIO_AF12_FMC, GPIO_PULLUP, "FMC NL"   }
    #define FMC_CTL_SDNWE                   { GPIO_PIN_5,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNWE"}
//    #define FMC_CTL_SDNWE                 { GPIO_PIN_0,  GPIOC, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNWE"}
    #define FMC_CTL_SDNCAS                  { GPIO_PIN_15, GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNCAS"}
    #define FMC_CTL_SDNRAS                  { GPIO_PIN_11, GPIOF, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNRAS"}
    #define FMC_CTL_SDNE0                   { GPIO_PIN_3,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNE0"}
//    #define FMC_CTL_SDNE0                 { GPIO_PIN_4,  GPIOC, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNE0"}
    #define FMC_CTL_SDNE1                   { GPIO_PIN_6,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNE1"}
//    #define FMC_CTL_SDNE1                 { GPIO_PIN_6,  GPIOB, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDNE1"}
    #define FMC_CTL_SDCKE0                  { GPIO_PIN_2,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDCKE0"}
//    #define FMC_CTL_SDCKE0                { GPIO_PIN_5,  GPIOC, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDCKE0"}
    #define FMC_CTL_SDCKE1                  { GPIO_PIN_7,  GPIOH, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDCKE1"}
//    #define FMC_CTL_SDCKE1                { GPIO_PIN_5,  GPIOB, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDCKE1"}
    #define FMC_CTL_SDCLK                   { GPIO_PIN_8,  GPIOG, GPIO_AF12_FMC, GPIO_PULLUP, "FMC SDCLK"}


    /* Definition for LTDC NVIC */
    #if defined(LTDC_USE_IRQ)
            #define LTDC_IRQ                        { LTDC_IRQn, LTDC_IRQ_PRIO, 0    }
            #define LTDC_IRQHandler                 LTDC_IRQHandler
            #define LTDC_ER_IRQ                     { LTDC_ER_IRQn, LTDC_IRQ_PRIO, 0    }
            #define LTDC_ER_IRQHandler              LTDC_ER_IRQHandler
    #endif

   
#endif // USE_LTDC 

#endif /* __LTDC_CONFIG_H */

