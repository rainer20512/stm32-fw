/*
 *  GPIO and Interrupt definitions for ETH
 */

#pragma once 

#include "config/devices_config.h"
#include "hardware.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

 
/******************************************************************************
 * ETH 
 *****************************************************************************/
#if USE_ETH > 0
    
  /* Definition for ETHERNET Pins, MII Mode
   *  Name           GPIO   ALTN      Data Dir Description
   * --------------------------------------------------------------------------
   * ETH_COL         PA3    PH3       input    Collision detection signal. MII only.
   * ETH_CRS         PA0    PH2       input    Carrier sense signal. MII only
   * ETH_RX_CLK      PA1              input    Timing reference for Rx data transfers
   * ETH_RX[0]       PC4              Input    Receive data.
   * ETH_RX[1]       PC5              Input    Receive data.
   * ETH_RX[2]       PB0    PH6       Input    Receive data.
   * ETH_RX[3]       PB1    PH7       Input    Receive data.
   * ETH_RX_DV       PA7              input    Receive data valid
   * ETH_RX_ER       PI10             input    Receive error
   * ETH_TX_CLK      PC3              input    Timing reference for Tx data transfers
   * ETH_TX[0]       PB12   PG13      output   Transmit data
   * ETH_TX[1]       PB13   PG12,PG14 output   Transmit data
   * ETH_TX[2]       PC2              output   Transmit data
   * ETH_TX[3]       PB8    PE2       output   Transmit data
   * ETH_TX_EN       PB11   PG11      output   Transmit Data Enable
   * ETH_TX_ER       PB10             output   Transmit error
   * ETH_MDC         PC1              output   Management Data Clock
   * ETH_MDIO        PA2              in/out   Management Data
   */

  /* Definition for ETHERNET Pins, RMII Mode
   *  Name           GPIO   ALTN      Data Dir Description
   * --------------------------------------------------------------------------
   * ETH_REF_CLK     PA1              input	   Continuous 50 MHz reference clock	
   * ETH_TX[0]	     PB12   PG13      output   Transmit data bit 0 (transmitted first)
   * ETH_TX[1]	     PB13   PG12,PG14 output   Transmit data
   * ETH_TX_EN	     PB11   PG11      output   Transmit data enable
   * ETH_RX[0]       PC4              input    Receive data 
   * ETH_RX[1]       PC5              input    Receive data 
   * ETH_CRS_DV	     PA7              input    Carrier Sense (CRS) and RX_Data Valid (RX_DV) multiplexed 
   * ETH_MDC         PC1              output   Management Data Clock
   * ETH_MDIO	     PA2              in/out   Management data
   */



  #if defined(ETH_USE_RMII)
    #define ETH_REF_CLK             { GPIO_PIN_1  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "Ref Clk" }
    #define ETH_RX0                 { GPIO_PIN_4  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "Rx0"     }
    #define ETH_RX1                 { GPIO_PIN_5  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "Rx1"     }
    #define ETH_CRS_DV              { GPIO_PIN_7  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "Crs/Dv"  }
    #define ETH_MDC                 { GPIO_PIN_1  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "MDc"     }
    #define ETH_MDIO                { GPIO_PIN_2  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "MDio"    }
    #if defined(STM32H745NUCLEO)
        // altn  #define ETH_TX0             { GPIO_PIN_12 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "Tx0"     } 
        #define ETH_TX0             { GPIO_PIN_13 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx0"     }
        #define ETH_TX1             { GPIO_PIN_13 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
        //altn    #define ETH_TX1          { GPIO_PIN_13 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
        //altn    #define ETH_TX1          { GPIO_PIN_14 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
        //altn    #define ETH_TX_EN	   { GPIO_PIN_11 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "TxEn"    }
        #define ETH_TX_EN           { GPIO_PIN_11 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "TxEn"    }
    #else
        #error "No Ethernet RMII config for selected board"
    #endif
  #elif defined(ETH_USE_MII)
    #define ETH_COL                 { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_CRS                 { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX_CLK              { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX[0]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX[1]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX[2]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX[3]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX_DV               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_RX_ER               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX_CLK              { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX[0]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
//altn    #define ETH_TX[0]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX[1]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }   
//altn    #define ETH_TX[1]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
//altn    #define ETH_TX[1]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX[2]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX[3]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
//altn    #define ETH_TX[3]               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX_EN               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
//altn    #define ETH_TX_EN               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_TX_ER               { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_MDC                 { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
    #define ETH_MDIO                { GPIO_PIN_ ,GPIO, GPIO_AF11_ETH, GPIO_NOPULL, ""    }
  #else
    #error "You must select either MII or RMII interface for ETH"
  #endif

  /* Definition for ETH Interrupts */
  #ifdef  ETH_USE_IRQ
      #define ETH_IRQ                       { ETH_IRQn,      ETH_IRQ_PRIO, 0 }
      #define ETH_WKUP_IRQ                  { ETH_WKUP_IRQn, ETH_IRQ_PRIO, 0 }
  #endif
#endif // USE_ETH > 0



