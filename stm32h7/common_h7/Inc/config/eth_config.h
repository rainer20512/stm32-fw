/*
 *  GPIO and Interrupt definitions for ETH
 */

#pragma once 

#include "config/devices_config.h"
#include "hardware.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define ETH_USE_RMII 
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
    #define ETH_REF_CLK      { GPIO_PIN_1  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "Ref Clk" }
    #define ETH_TX0	     { GPIO_PIN_12 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "Tx0"     } 
    //altn    #define ETH_TX0          { GPIO_PIN_13 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx0"     }
    #define ETH_TX1	     { GPIO_PIN_13 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
    //altn    #define ETH_TX1          { GPIO_PIN_13 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
    //altn    #define ETH_TX1          { GPIO_PIN_14 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "Tx1"     }
    #define ETH_TX_EN	     { GPIO_PIN_11 ,GPIOB, GPIO_AF11_ETH, GPIO_NOPULL, "TxEn"    }
    //altn    #define ETH_TX_EN        { GPIO_PIN_11 ,GPIOG, GPIO_AF11_ETH, GPIO_NOPULL, "TxEn"    }
    #define ETH_RX0          { GPIO_PIN_4  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "Rx0"     }
    #define ETH_RX1          { GPIO_PIN_5  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "Rx1"     }
    #define ETH_CRS_DV	     { GPIO_PIN_7  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "Crs/Dv"  }
    #define ETH_MDC          { GPIO_PIN_1  ,GPIOC, GPIO_AF11_ETH, GPIO_NOPULL, "MDc"     }
    #define ETH_MDIO	     { GPIO_PIN_2  ,GPIOA, GPIO_AF11_ETH, GPIO_NOPULL, "MDio"    }
  #elif defined(ETH_USE_MII)
  #else
    #error "You must select either MII or RMII interface for ETH"
  #endif

  /* Definition for CAN Interrupts */
  #ifdef  ETH_USE_IRQ
      #define CAN1_TX_IRQ                   { CAN1_TX_IRQn,  CAN_IRQ_PRIO, 0 }
      #define CAN1_RX0_IRQ                  { CAN1_RX0_IRQn, CAN_IRQ_PRIO, 0 }
      #define CAN1_RX1_IRQ                  { CAN1_RX1_IRQn, CAN_IRQ_PRIO, 0 }
      #define CAN1_SCE_IRQ                  { CAN1_SCE_IRQn, CAN_IRQ_PRIO, 0 }
  #endif
/******************************************
 Interrupt routine names are
           CAN1_TX_IRQHandler
           CAN1_RX0_IRQHandler
           CAN1_RX1_IRQHandler
           CAN1_SCE_IRQHandler
 *****************************************/
#endif // USE_ETH > 0



