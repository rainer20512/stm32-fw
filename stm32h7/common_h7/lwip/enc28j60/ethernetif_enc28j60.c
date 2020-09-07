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

#define DEBUG_ETHERNETIF       0

/* Includes ------------------------------------------------------------------*/
#include "enc28j60.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"
#include <string.h>

// RHB Added
#include "debug_helper.h"

/* Private typedef -----------------------------------------------------------*/

/* Types of action for any Queue Items */
typedef enum {
    Q_DO__INPUT          = 0,
    Q_DO__OUTPUT,
    Q_QRY_LINKSTATUS,
} IfQueueEntryEnum;

/* Queue structure for communication with the ethernet_io task */
typedef struct IfQueueEntryStruct {
    IfQueueEntryEnum    q_type;
    void *              q_arg;
} IfQueueEntryT;

/* Private define ------------------------------------------------------------*/

/* Size of the ethernet_io queue size */
#define IF_QUEUE_SIZE                          5
/* The time to block waiting for queue input. */
#define QUEUE_WAITING_FOR_INPUT                 ( osWaitForever )
/* The time to block waiting for queue output to be done. */
#define TIME_WAITING_FOR_TRANSMIT                 ( 100 )

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

/* Statically allocated memory for Queue entries and queue management information */
static uint8_t IfQueueStorage[IF_QUEUE_SIZE * sizeof(IfQueueEntryT)];
static StaticQueue_t    IfIoQueuePrivate;

osSemaphoreId TxPktSemaphore = NULL; /* Semaphore to signal incoming packets */
QueueHandle_t IfIoQueue;             /* Interface IO queue                   */

/* Private function prototypes -----------------------------------------------*/
static err_t if_trigger_output   (struct netif *netif, struct pbuf *p);
static void ethernetif_io        ( void const * argument );
static void ethernetif_input     ( struct netif *netif, void *arg ) ;
static void low_level_output     ( struct netif *netif, void *arg ) ;
static void ethernetif_linkstate ( struct netif *netif, void *arg ) ;
u32_t       sys_now(void);
static ENC_HandleTypeDef enc28j60;


/******************************************************************************
 * @brief Ethernet interface io task: 
 *  - get the type of entry and act according to entry type
 * the execution of any type of action sequentially within one task
 * will guarantee, that every access to ENC hardware via SPI is done
 * sequentially under any circumstances
 *
 * @param netif the lwip network interface structure for this ethernetif
 *****************************************************************************/
void ethernetif_io( void const * argument )
{
    IfQueueEntryT qEntry;
    struct netif *netif = (struct netif *) argument;

    for( ;; )  {
        if  ( xQueueReceive(IfIoQueue, &qEntry, QUEUE_WAITING_FOR_INPUT ) == pdTRUE ) {
            switch ( qEntry.q_type ) {
                case Q_DO__INPUT:
                    DEBUG_PRINTF("ETH_IO:Input\n");
                    ethernetif_input ( netif, qEntry.q_arg );
                    break;
                case Q_DO__OUTPUT:
                    DEBUG_PRINTF("ETH_IO:output\n");
                    low_level_output ( netif, qEntry.q_arg );
                    break;
                case Q_QRY_LINKSTATUS:
                    DEBUG_PRINTF("ETH_IO:linkstate\n");
                    ethernetif_linkstate ( netif, qEntry.q_arg );
                    break;
                default:
                    DEBUG_PRINTF("ethernetif_io: Illegal Queue entry type %d \n", qEntry.q_type);
            } // switch
            ENC_Check_RxIRQ();
        } // if
    } // for
}


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
  TxPktSemaphore = xSemaphoreCreateBinary();

  /* Create the IO queue */
  IfIoQueue = xQueueCreateStatic(IF_QUEUE_SIZE, sizeof(IfQueueEntryT), IfQueueStorage, &IfIoQueuePrivate);
  
  /* create the task that handles the ETH_MAC */
  osThreadDef(EthIf, ethernetif_io, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
  osThreadCreate (osThread(EthIf), netif);
  
  enc28j60.Init.ChecksumMode        = ETH_CHECKSUM_BY_HARDWARE;
  enc28j60.Init.DuplexMode          = ETH_MODE_FULLDUPLEX;
  enc28j60.Init.MACAddr             = macaddress;

  /* Initialize the ENC28J60, set MAC address, configure interrupts and enable receiver */
  ENC_Start(&enc28j60);
  
  PHYLinkState = ENC_GetLinkState(&enc28j60);
  
  /* Get link state */  
  if(PHYLinkState <= ENC_STATUS_LINK_DOWN) {
    DEBUG_PUTS("Link initially down\n");
    netif_set_link_down(netif);
    netif_set_down(netif);
  } else {
    DEBUG_PUTS("Link initially up\n");
    netif_set_up(netif);
    netif_set_link_up(netif);
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
  netif->linkoutput = if_trigger_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
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
static void low_level_output(struct netif *netif, void *arg)
{
    struct pbuf *p = (struct pbuf *) arg;

    UNUSED(netif);

    if ( !ENC_TransmitBuffer ( &enc28j60, p ) ) {
        DEBUG_PRINTF("ENC_Transmit failed\n");
    }

    /* 
     * Signal the ethernet thread, that transmission is done 
     * ( and pbuf may be freed )
     */
    osSemaphoreRelease(TxPktSemaphore);
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
    DEBUG_PRINTF("Rx: Got %d bytes\n", p->tot_len );
  } else {
    /* if no frame is delivered to IP stack, rxBuff has to bee freed immediately */
    DEBUG_PRINTF("Rx: No bytes\n");
  }

  return p;
}

void ethernetif_input( struct netif *netif, void *arg )
{
    struct pbuf *p;

    /* 
     * ethernetif_if input is triggered by hardware interrupt. 
     * So Reset interrupt line and clear interrupt flags 
     */

    DEBUG_PRINTF("Start Receive\n");
//    LOCK_TCPIP_CORE();
    while ( ENC_RxPacketAvailable(&enc28j60) ) {
        p = low_level_input( netif );
        if (p != NULL) {
            if (netif->input( p, netif) != ERR_OK ) {
                pbuf_free(p);
            }
        }
    } 
//    UNLOCK_TCPIP_CORE();
    DEBUG_PRINTF("eeeee - End Receive\n");

    ENC_restore_irq(true);

    ENC_DisableInterrupts();    
    ENC_clear_irqflags();
    ENC_EnableInterrupts();

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
static err_t if_trigger_output(struct netif *netif, struct pbuf *p)
{
  UNUSED(netif);
  IfQueueEntryT tx = { Q_DO__OUTPUT, p };

  if ( xQueueSend(IfIoQueue, &tx, 0 ) == pdFALSE ) {
    DEBUG_PRINTF("TriggerOutput: Cannout enqueue\n");
    return ERR_IF;
  }
   
  if (osSemaphoreWait( TxPktSemaphore, TIME_WAITING_FOR_TRANSMIT)!=osOK) {
    DEBUG_PRINTF("TriggerOutput: Timeout when waiting for transmit done\n");
    return ERR_IF;
  } 

  return ERR_OK;
}

void ENC_RxCpltCallback(bool bFromISR)
{
  IfQueueEntryT rx = { Q_DO__INPUT, NULL };
  bool result;

  if ( bFromISR)
    result = xQueueSendFromISR(IfIoQueue, &rx, 0 );
  else  
    result = xQueueSend(IfIoQueue, &rx, 0 );


  if ( result == pdFALSE ) DEBUG_PRINTF("RxCallback: Cannout enqueue\n");
}

static void ethernetif_linkstate ( struct netif *netif, void *arg )
{
    int32_t PHYLinkState = ENC_GetLinkState(&enc28j60);
    
    if(netif_is_link_up(netif) && (PHYLinkState <= ENC_STATUS_LINK_DOWN))
    {
      DEBUG_PUTS("Link down\n");
      netif_set_down(netif);
      netif_set_link_down(netif);
    }
    else if(!netif_is_link_up(netif) && (PHYLinkState > ENC_STATUS_LINK_DOWN))
    {
        DEBUG_PUTS("Link up\n");
        netif_set_up(netif);
        netif_set_link_up(netif);
    }   
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
    IfQueueEntryT linkstate = { Q_QRY_LINKSTATUS, NULL };

    for(;;) {
        if ( xQueueSend(IfIoQueue, &linkstate, 0 ) == pdFALSE ) {
            DEBUG_PRINTF("LinkState: Cannout enqueue\n");
        }
    
        osDelay(500);
    }
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



#endif /* USE_ETH_PHY_ENC28J60  */
