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


    /* Definition for FMC NVIC */
    #if defined(FMC_USE_IRQ)
            #define FMC_IRQ                        { FMC_IRQn, FMC_IRQ_PRIO, 0    }
            #define FMC_IRQHandler                 FMC_IRQHandler
    #endif

    /* SRAM timimg for CY62157EV30 MoBL SRAM with multiplexed adress/data line and 74hc373 address latches */
    /* very, very slow!!                                                                                   */

    /* Number of address bits                                                                              */
    #define ADDWID3                         20

    /* Number of data bits                                                                              */
    #define DATAWID3                        16

    /* Addr/data multipexed ? */
    #define ADMUXED3                        0

    /* Address setup time [ns]                            */
    #define ADDSET3                         10

    /* Address hold time [ns], only valid for mux'ed SRAM */
    #define ADDHLD3                         5

    /* Data setup time    [ns]                            */
    #define DATASET3                        10

    #define ACCMODE3                        FMC_ACCESS_MODE_A

    #define SRAM_TIMING3                    { ACCMODE3, ADDWID3, DATAWID3, ADMUXED3, ADDSET3, ADDHLD3, DATASET3, }
    #define FMC_TYPE3                       FMC_TYPE_SRAM

    /* Mode register definitions for IS42S32800G DRAM, see data sheet p.22 ff */
    #define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
    #define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
    #define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
    #define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
    #define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
    #define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
    #define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
    #define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
    #define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
    #define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
    #define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

    #define MODEREG4     (uint32_t)(   SDRAM_MODEREG_BURST_LENGTH_1   | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   \
                                    | SDRAM_MODEREG_CAS_LATENCY_2    | SDRAM_MODEREG_OPERATING_MODE_STANDARD \
                                    | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE \
                                   )

    /* Timing parameters for IS42S32800G-x DRAM according to datasheet */                                    
    #define T_MRD                   14      // tMRD - Mode Register program time -> 12ns
    #define T_XSR                   70      // tXSR - Self refresh exit time     -> 70ns
    #define T_RAS                   40      // min. self refresh period = tRAS   -> 40ns
    #define T_RC                    68      // tRC  - Command Period/Act-to-Act  -> 67.5ns
    #define T_WR                    20      // Wr. Recovery t.:tRAS-rRCD -> 40-20-> 20ns
    #define T_RP                    18      // tRP  - Precharge to Active Delay  -> 18ns 
    #define T_RCD                   18      // tRCD - Active to Read/Write Delay -> 18ns

    #define SDRAM_TIMES4           {T_MRD, T_XSR, T_RAS, T_RC, T_WR, T_RP, T_RCD, }

    /* Operational parameters for IS42S32800G DRAM */
    #define ROWWID4                 12
    #define COLWID4                 9
    #define DATAWID4                32
    #define CASLTCY4                2             /* CAS Latency may be 2 @ 100MHz SDRAM clock */
    #define SDBANKNUM4              2
    #define REFRESHRATE4            64            /* Refreshrate [ms] according to data sheet */
 
    #define FMC_TYPE4               FMC_TYPE_SDRAM
    #define SDRAM_TIMING4          { MODEREG4, ROWWID4, COLWID4, DATAWID4, SDBANKNUM4, CASLTCY4, REFRESHRATE4, SDRAM_TIMES4, } 
    
    
    #if 0
        /* SRAM timimg for CY62157EV30 MoBL SRAM with multiplexed adress/data line and 74hc373 address latches */
        /* very, very slow!!                                                                                   */

        /* Number of address bits                                                                              */
        #define ADDWID1                         24

        /* Number of data bits                                                                              */
        #define DATAWID1                        16

        /* Addr/data multipexed ? */
        #define ADMUXED1                        1

        /* Address setup time is the 74hc373 minimum pulsewidth ( worst case ) in ns                           */
        #define ADDSET1                         60
    
        /* Address hold time is 74hc737 propagation delay plus output transistion time                         */
        #define ADDHLD1                         100

        /* Data setup time is the tDOE value from the CY62157EV30 datasheet                                    */
        #define DATASET1                        50

        #define ACCMODE1                        FMC_ACCESS_MODE_D

        #define TIMING1                         { ACCMODE1, ADDWID1, DATAWID1, ADMUXED1, ADDSET1, ADDHLD1, DATASET1, }
    #endif

#endif // USE_FMC 

#endif /* __FMC_CONFIG_H */

