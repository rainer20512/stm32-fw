/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/ethernetif.c
  * @author  MCD Application Team
  * @brief   This file implements Ethernet network interface drivers for lwIP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "config/config.h"

#if USE_ETH_PHY_ENC28J60 == 1 

#define DEBUG_IF            0

/* Includes ------------------------------------------------------------------*/
#include "enc28j60.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"
#include "lwip/pbuf.h"
#include <string.h>
#include "stm32h7xx_hal.h"
#include "debug_helper.h"

#if DEBUG_ENCTX > 0 
    #define IFDEBUG(...)   DEBUG_PRINTF(__VA_ARGS__)
#else
    #define IFDEBUG(...)   
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT                 ( osWaitForever )
#define TIME_WAITING_SPI_SEMAPHORE             ( osWaitForever ) 
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE            ( 350 )

/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_RX_BUFFER_SIZE                     (CONFIG_NET_ETH_MTU+36)

#define ETH_DMA_TRANSMIT_TIMEOUT                (20U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack,
          they will return back to ETH DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap, 
          then passed to ETH HAL driver.
*/

static uint8_t macaddress[6]= {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};

osSemaphoreId RxPktSemaphore = NULL; /* Semaphore to signal incoming packets */
osSemaphoreId SpiSemaphore   = NULL; /* Semaphore to guard SPI function access */

/* Private function prototypes -----------------------------------------------*/
static void ethernetif_input( void const * argument );
u32_t       sys_now(void);
static ENC_HandleTypeDef enc28j60;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH) 
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static void low_level_init(struct netif *netif)
{
  int32_t PHYLinkState;
    
  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;
  
  /* set MAC hardware address */
  netif->hwaddr[0] =  ETH_MAC_ADDR0;
  netif->hwaddr[1] =  ETH_MAC_ADDR1;
  netif->hwaddr[2] =  ETH_MAC_ADDR2;
  netif->hwaddr[3] =  ETH_MAC_ADDR3;
  netif->hwaddr[4] =  ETH_MAC_ADDR4;
  netif->hwaddr[5] =  ETH_MAC_ADDR5;
  
  /* maximum transfer unit */
  netif->mtu = CONFIG_NET_ETH_MTU;
  
  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    
  /* create a binary semaphore used for informing ethernetif of frame reception */
  RxPktSemaphore = xSemaphoreCreateBinary();

  SpiSemaphore   = xSemaphoreCreateBinary();
  xSemaphoreGive(SpiSemaphore);
  
  /* create the task that handles the ETH_MAC */
  osThreadDef(EthIf, ethernetif_input, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
  osThreadCreate (osThread(EthIf), netif);
  
  memset(&enc28j60, 0, sizeof(ENC_HandleTypeDef) );
  enc28j60.Init.ChecksumMode        = ETH_CHECKSUM_BY_HARDWARE;
  enc28j60.Init.DuplexMode          = ETH_MODE_HALFDUPLEX;
  enc28j60.Init.MACAddr             = macaddress;

  /* Initialize the ENC28J60, set MAC address, configure interrupts and enable receiver */
  ENC_Start(&enc28j60);
  
  PHYLinkState = ENC_GetLinkState(&enc28j60);
  
  /* Get link state */  
  if(PHYLinkState <= ENC_STATUS_LINK_DOWN) {
    IFDEBUG("Link initially down\n");
    netif_set_link_down(netif);
    netif_set_down(netif);
  } else {
    IFDEBUG("Link initially up\n");
    netif_set_up(netif);
    netif_set_link_up(netif);
  }
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  err_t errval = ERR_OK;
  UNUSED(netif);

  xSemaphoreTake(SpiSemaphore, TIME_WAITING_SPI_SEMAPHORE);    
  if ( !ENC_TransmitBuffer ( &enc28j60, p ) ) {
    IFDEBUG("ENC_Transmit failed\n");
    errval = ERR_IF;
  }
  xSemaphoreGive(SpiSemaphore);

  return errval;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  UNUSED(netif);
  struct pbuf *p = NULL;

  if( ENC_read_into_pbuf(&enc28j60, &p ) ) {
    #if DEBUG_IF > 1
        IFDEBUG("Rx: Got %d bytes\n", p->tot_len );
    #endif
  } else {
    /* if no frame is delivered to IP stack, rxBuff has to bee freed immediately */
    #if DEBUG_IF > 0
        IFDEBUG("Rx: No bytes\n");
    #endif
  }

  return p;
}

/**
  * @brief This function is the ethernetif_input task, it is processed when a packet 
  * is ready to be read from the interface. It uses the function low_level_input() 
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input( void const * argument )
{
    struct pbuf *p;
    struct netif *netif = (struct netif *) argument;

    for( ;; ){
        if (osSemaphoreWait( RxPktSemaphore, TIME_WAITING_FOR_INPUT)==osOK){
            #if DEBUG_IF > 2
                IFDEBUG("wait Receive\n");
            #endif
            LOCK_TCPIP_CORE();
            xSemaphoreTake(SpiSemaphore, TIME_WAITING_SPI_SEMAPHORE);    
            #if DEBUG_IF > 2
                IFDEBUG("Start Receive\n");
            #endif
            while ( ENC_RxPacketAvailable(&enc28j60) ) {
                p = low_level_input( netif );
                if (p != NULL) {
                    if (netif->input( p, netif) != ERR_OK ) {
                        pbuf_free(p);
                    }
                }
            } 
            #if DEBUG_IF > 2
                IFDEBUG("eeeee - End Receive\n");
            #endif

            ENC_RetriggerRxInterrupt(&enc28j60);

            xSemaphoreGive(SpiSemaphore);
            UNLOCK_TCPIP_CORE();

            // ENC_restore_irq(true);
        }
    }
}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */
  
  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  Returns the current time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  return HAL_GetTick();
}


void ENC_RxCpltCallback(void)
{
  osSemaphoreRelease(RxPktSemaphore);
  //
}


/**
  * @brief  Check the ETH link state and update netif accordingly.
  * @param  argument: netif
  * @retval None
  */
void ethernet_link_thread( void const * argument )
{
  int32_t PHYLinkState;
  struct netif *netif = (struct netif *) argument;
  
  for(;;)
  {
     
    xSemaphoreTake(SpiSemaphore, TIME_WAITING_SPI_SEMAPHORE);    

    /* Get Linkstate */
    PHYLinkState = ENC_GetLinkState(&enc28j60);
    
    if(netif_is_link_up(netif) && (PHYLinkState <= ENC_STATUS_LINK_DOWN))
    {
      ENC_Stop(&enc28j60);
      IFDEBUG("Link down\n");
      xSemaphoreGive(SpiSemaphore);
      netif_set_down(netif);
      netif_set_link_down(netif);
      xSemaphoreTake(SpiSemaphore, TIME_WAITING_SPI_SEMAPHORE);    
      if ( enc28j60.errstat ) enc28j60.errstat->linkChngCnt++;
    }
    else if(!netif_is_link_up(netif) && (PHYLinkState > ENC_STATUS_LINK_DOWN))
    {
        ENC_Restart(&enc28j60);
        IFDEBUG("Link up\n");
        xSemaphoreGive(SpiSemaphore);
        netif_set_up(netif);
        netif_set_link_up(netif);
        xSemaphoreTake(SpiSemaphore, TIME_WAITING_SPI_SEMAPHORE);    
        if ( enc28j60.errstat ) enc28j60.errstat->linkChngCnt++;
    }

    /* Check Rx Status */
    ENC_CheckRxStatus(&enc28j60);

    /* 
     * If Rx packet interrupt flag is set and more than 8 packets are waiting, 
     * we ass ume that the Rx packet interrupt was suppressed and we trigger
     * it manually
     */
    if ( enc28j60.PktCnt > 8 && ( enc28j60.eir & EIR_PKTIF ) != 0 ) {
        #if DEBUG_IF > 2
            IFDEBUG("-----\n");
        #endif
        ENC_RxCpltCallback();
    } 

    xSemaphoreGive(SpiSemaphore);

    osDelay(500);
  } // for
}

void ethernetif_statistic ( void )
{
   DEBUG_PRINTF("ENC28J60 Ethernet Interface Statistics");
   DEBUG_PRINTF("%5d restarts\n", enc28j60.restarts );
   ENC_DumpStatistic(&enc28j60);
}

#endif /* USE_ETH_PHY_ENC28J60  */
