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
/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT                 ( osWaitForever )
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE            ( 350 )

/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_RX_BUFFER_SIZE                     (CONFIG_NET_ETH_MTU+36)

#define ETH_DMA_TRANSMIT_TIMEOUT                (20U)

#define RX_BUFF_CNT         8  /* number of Rx buffers */

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

/* my private rx buffers -----------------------------------------------------*/
typedef struct {
    /* Must be the first element in structure!                                */    
    uint8_t  rxBytes[ETH_RX_BUFFER_SIZE];      /* rx buffer                   */
    uint32_t bIsUsed;                          /* Flag for "buffer in use     */
    uint32_t rxBuffLen;                        /* actual length of buffer     */
    uint32_t rxID;
} RxBufferT;


/* Private function prototypes -----------------------------------------------*/
static void ethernetif_input( void const * argument );
u32_t       sys_now(void);
void        pbuf_free_custom(struct pbuf *p);
RxBufferT*  AllocRxBuffer(void);
void        FreeRxBuffer(RxBufferT *rx);

static ENC_HandleTypeDef enc28j60;
static RxBufferT RxBuffer[RX_BUFF_CNT]  __attribute__((section(".uncached100"))); 

LWIP_MEMPOOL_DECLARE(RX_POOL, 10, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool");

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
  uint32_t i;
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
    
  /* Initialize the RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);

  /* all RxBuffers are unused */
  for ( i = 0; i < RX_BUFF_CNT; i++ )
    RxBuffer[i].bIsUsed = false;
     
  /* create a binary semaphore used for informing ethernetif of frame reception */
  RxPktSemaphore = xSemaphoreCreateBinary();
  
  /* create the task that handles the ETH_MAC */
  osThreadDef(EthIf, ethernetif_input, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
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
#include "debug_helper.h"
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0, framelen = 0;
  struct pbuf *q;
  err_t errval = ERR_OK;

  UNUSED(netif);

  if ( !ENC_TransmitBuffer ( &enc28j60, p ) ) {
    DEBUG_PRINTF("ENC_Transmit failed\n");
    errval = ERR_IF;
  }

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
  struct pbuf_custom* custom_pbuf;
  RxBufferT *rxBuff;
  
  rxBuff = AllocRxBuffer();
  if ( !rxBuff ) {
    DEBUG_PRINTF("low_level_input - Error: No more RxBuffers\n");
    /* Get the rx packet from hardware to free rx buffer space - rx packet is lost */
    ENC_GetReceivedFrame(&enc28j60, NULL, 0);
    return NULL;
  }

  if( ENC_GetReceivedFrame(&enc28j60, rxBuff->rxBytes, &rxBuff->rxBuffLen) ) {
    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    custom_pbuf->custom_free_function = pbuf_free_custom;

    p = pbuf_alloced_custom(PBUF_RAW, rxBuff->rxBuffLen, PBUF_REF, custom_pbuf, rxBuff, ETH_RX_BUFFER_SIZE);
    DEBUG_PRINTF("Rx: Got %d bytes\n", rxBuff->rxBuffLen );
  } else {
    /* if no frame is delivered to IP stack, rxBuff has to bee freed immediately */
    FreeRxBuffer(rxBuff);
    DEBUG_PRINTF("Rx: No bytes\n");
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
            ENC_DisableInterrupts();    
            ENC_clear_irqflags();
            ENC_EnableInterrupts();
            DEBUG_PRINTF("Start Receive\n");
            LOCK_TCPIP_CORE();
            while ( ENC_RxPacketAvailable(&enc28j60) ) {
                p = low_level_input( netif );
                if (p != NULL) {
                    if (netif->input( p, netif) != ERR_OK ) {
                        pbuf_free(p);
                    }
                }
            } 
            DEBUG_PRINTF("eeeee - End Receive\n");
            ENC_restore_irq(enc28j60.irq_ena);
            UNLOCK_TCPIP_CORE();
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
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  RxBufferT *rxBuf = (RxBufferT *)p->payload;
    
  FreeRxBuffer(rxBuf);
#if defined(CORE_CM7)
  /* invalidate data cache: lwIP and/or application may have written into buffer */
  SCB_InvalidateDCache_by_Addr((uint32_t *)p->payload, p->tot_len);
#endif
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
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
    
/* 
    PHYLinkState = ENC_GetLinkState(&enc28j60);
    
    if(netif_is_link_up(netif) && (PHYLinkState <= ENC_STATUS_LINK_DOWN))
    {
      // RHB TODO HAL_ETH_Stop_IT(&EthHandle);
      DEBUG_PUTS("Link down\n");
      netif_set_down(netif);
      netif_set_link_down(netif);
    }
    else if(!netif_is_link_up(netif) && (PHYLinkState > ENC_STATUS_LINK_DOWN))
    {
        //RHB TODO HAL_ETH_Start_IT(&EthHandle);
        DEBUG_PUTS("Link up\n");
        netif_set_up(netif);
        netif_set_link_up(netif);
    }
*/   
    osDelay(500);
  }
}

/******************************************************************************
 * Find an unused RxBuffer 
 *****************************************************************************/
RxBufferT* AllocRxBuffer(void)
{
    for ( uint32_t i = 0; i < RX_BUFF_CNT; i ++ ) {
        if ( RxBuffer[i].bIsUsed == 0 ) {
            RxBuffer[i].bIsUsed = 1;
              #if DEBUG_ETHERNETIF > 1
                DEBUG_PRINTF("AllocRX #%d @0x%08x\n",i,&RxBuffer[i] );
              #endif
            RxBuffer[i].rxID = i;    
            return &RxBuffer[i];
        }
    }
    /* No free RxBuffer found */
    return NULL;
}

/******************************************************************************
 * Free a previously allocated RxBuffer
 *****************************************************************************/
void FreeRxBuffer(RxBufferT *rx)
{
    RxBufferT *work=NULL;
    uint8_t *start;

    /* Search for rx pointing to begin of RxBuffer-Element */
    for ( uint32_t i = 0; i < RX_BUFF_CNT; i ++ ) {    
        if ( RxBuffer+i == rx ) {
            work = rx;    
            break;
        }
    }
    /* if not found, search for rx pointing somewhere into RxBuffer-Element */
    if ( !work ) {
        for ( uint32_t i = 0; i < RX_BUFF_CNT; i ++ ) {    
            start = (uint8_t *)(RxBuffer+i);
            if ( (uint8_t*)rx <= start + sizeof(RxBufferT) - 1 ) {
                work = (RxBufferT *)start;    
                break;
            }
        }
    }

    if ( work ) {
        work->bIsUsed = 0;
        #if DEBUG_ETHERNETIF > 1
            DEBUG_PRINTF("FreeRX #%d @0x%08x\n", work->rxID, work);
        #endif
    } else {
        #if DEBUG_ETHERNETIF > 1
            DEBUG_PRINTF("FreeRX @0x%08x - illegal pointer\n", work);
        #endif
    }
}
   

#endif /* USE_ETH_PHY_ENC28J60  */
