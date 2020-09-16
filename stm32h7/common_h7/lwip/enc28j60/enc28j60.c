/**
  ******************************************************************************
  * @file    enc28j60.c
  * @author  Christian Schoffit, portions from Gregory Nutt:
  *          Copyright (C) 2010-2012, 2014 Gregory Nutt. All rights reserved.
  *          Author: Gregory Nutt <gnutt@nuttx.org>
  *
  * @version V1.0.0
  * @date    02-June-2015
  * @brief   This file provides a set of functions needed to manage the ENC28J60
  *          Stand-Alone Ethernet Controller with SPI Interface.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 Christian Schoffit</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Christian Schoffit nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "config/config.h"

#if USE_ETH_PHY_ENC28J60 == 1 

#define DO_ENC_STATISTIC        1

#if defined(STM32H745NUCLEO)
    #define SPI_HANDLE      (&SPI1Handle)
#elif defined(STM32H742REF)
    #define SPI_HANDLE      (&SPI3Handle)
#else
    #error "No SPI-Interface for ENC28J60 defined"
#endif
/*
Module         Feature        Issue Issue Summary                                             Affected Revisions
                                                                                              B1 B4 B5 B7
MAC Interface  -              1.    MAC registers unreliable with slow asynchronous SPI clock X  X
Reset          -              2.    CLKRDY set early                                          X  X  X  X
Core           Operating      3.    Industrial (-40캜 to +85캜) temperature range unsupported X  X
               Specifications
Oscillator     CLKOUT pin     4.    CLKOUT unavailable in Power Save mode                     X  X  X  X
Memory         Ethernet       5.    Receive buffer must start at 0000h                        X  X  X  X
               Buffer
Interrupts     -              6.    Receive Packet Pending Interrupt Flag (PKTIF) unreliable  X  X  X  X
PHY            -              7.    TPIN+/- automatic polarity detection and correction
                                    unreliable                                                X  X  X  X
PHY            -              8.    RBIAS resistor value differs between silicon revisions    X  X
PHY            -              9.    Internal loopback in half-duplex unreliable               X  X  X  X
PHY            -              10.   Internal loopback in full-duplex unreliable               X  X  X  X
PHY LEDs       -              11.   Combined Collision and Duplex Status mode unavailable     X  X  X  X
Transmit       -              12.   Transmit abort may stall transmit logic                   X  X  X  X
Logic
PHY            -              13.   Received link pulses potentially cause collisions               X  X
Memory         Ethernet       14.   Even values in ERXRDPT may corrupt receive buffer         X  X  X  X
               Buffer
Transmit       -              15.   LATECOL Status bit unreliable                             X  X  X  X
Logic
PHY LEDs       -              16.   LED auto-polarity detection unreliable                    X  X  X  X
DMA            -              17.   DMA checksum calculations will abort receive packets      X  X  X  X
Receive        -              18.   Pattern match filter allows reception of extra packets    X  X  X  X
Filter
SPI            -              19.   Reset command unavailable in Power Save mode              X  X  X  X
Interface

Only workaround relative to issues affecting B7 silicon revision are implemented. Therefore, issues
specific to Ethernet conformance are not addressed, since they only affect B1 and B3 silicon revisions.

Erratas 7, 8, 16... have workaround implemented by hardware

Errata 18 is implemented in lwip stack
*/

/* Includes ------------------------------------------------------------------*/
#include "enc28j60.h"
#include "dev/spi.h"
#include "debug_helper.h"

#if defined(DUAL_CORE) && defined(CORE_CM4)
    #include "system/clockconfig_cm4.h"
#else
    #include "system/clockconfig.h"
#endif

/* for struct pbuf */
#include "lwip/pbuf.h"

/* for osDelay */
#include "cmsis_os.h"

#define DEBUG_ENCRX         0       /* Debug Rx path */
#define DEBUG_ENCTX         0       /* Debug Tx path */
#define DEBUG_ENC           0       /* Debug common  */

//#define DEBUG_OUTPUT(lvl,...)             DEBUG_PRINTF(__VA_ARGS__)  

#define DEBUG_OUTPUT(lvl,...)             do if ( debuglevel > lvl ) { DEBUG_PRINTF(__VA_ARGS__); } while(0)  

#define ENCDEBUG(...)                     DEBUG_OUTPUT(0, __VA_ARGS__)
#define ENCRXDEBUG(...)                   DEBUG_OUTPUT(0, __VA_ARGS__)
#define ENCTXDEBUG(...)                   DEBUG_OUTPUT(0, __VA_ARGS__)

#if DEBUG_ENCTX > 0   /***** Tx Lvl 1 *****/
    #define ENCTX1DEBUG(...)              DEBUG_OUTPUT(1,__VA_ARGS__)
#else
    #define ENCTX1DEBUG(...)   
#endif
#if DEBUG_ENCTX > 1   /***** Tx Lvl 2 *****/
    #define ENCTX2DEBUG(...)              DEBUG_OUTPUT(2,__VA_ARGS__)
#else
    #define ENCTX2DEBUG(...)   
#endif
#if DEBUG_ENCTX > 2   /***** Tx Lvl 3 *****/
    #define ENCTX3DEBUG(...)              DEBUG_OUTPUT(3,__VA_ARGS__)
#else
    #define ENCTX3DEBUG(...)   
#endif
#if DEBUG_ENCTX > 3   /***** Tx Lvl 4 *****/
    #define ENCTX4DEBUG(...)              DEBUG_OUTPUT(4,__VA_ARGS__)
#else
    #define ENCTX4DEBUG(...)   
#endif

#if DEBUG_ENCRX > 0   /***** Rx Lvl 1 *****/
    #define ENCRX1DEBUG(...)              DEBUG_OUTPUT(1, __VA_ARGS__)
#else
    #define ENCRX1DEBUG(...)   
#endif
#if DEBUG_ENCRX > 1   /***** Rx Lvl 2 *****/
    #define ENCRX2DEBUG(...)              DEBUG_OUTPUT(2, __VA_ARGS__)
#else
    #define ENCRX2DEBUG(...)   
#endif
#if DEBUG_ENCRX > 2   /***** Rx Lvl 3 *****/
    #define ENCRX3DEBUG(...)              DEBUG_OUTPUT(3, __VA_ARGS__)
#else
    #define ENCRX3DEBUG(...)   
#endif
#if DEBUG_ENCRX > 3   /***** Rx Lvl 4 *****/
    #define ENCRX4DEBUG(...)              DEBUG_OUTPUT(4, __VA_ARGS__)
#else
    #define ENCRX4DEBUG(...)   
#endif

#if DEBUG_ENC > 0   /***** Lvl 1 *****/
    #define ENC1DEBUG(...)              DEBUG_OUTPUT(1, __VA_ARGS__)
#else
    #define ENC1DEBUG(...)   
#endif
#if DEBUG_ENC > 1   /***** Lvl 2 *****/
    #define ENC2DEBUG(...)              DEBUG_OUTPUT(2, __VA_ARGS__)
#else
    #define ENC2DEBUG(...)   
#endif
#if DEBUG_ENC > 2   /***** Lvl 3 *****/
    #define ENC3DEBUG(...)              DEBUG_OUTPUT(3, __VA_ARGS__)
#else
    #define ENC3DEBUG(...)   
#endif
#if DEBUG_ENC > 3   /***** Lvl 4 *****/
    #define ENC4DEBUG(...)              DEBUG_OUTPUT(4, __VA_ARGS__)
#else
    #define ENC4DEBUG(...)   
#endif

/* Poll timeout */

#define ENC_POLLTIMEOUT 50


/* Max Rx packet size ********************************************************/
#define ETH_MAX_RX_FRAMESIZE                   (CONFIG_NET_ETH_MTU+18)

/* Packet memory layout */

#define ALIGNED_BUFSIZE ((CONFIG_NET_ETH_MTU + 255) & ~255)

/* Work around Errata #5 (spurious reset of ERXWRPT to 0) by placing the RX
 * FIFO at the beginning of packet memory.
 */

#  define PKTMEM_RX_START 0x0000                            /* RX buffer must be at addr 0 for errata 5 */
#  define PKTMEM_RX_END   (PKTMEM_END-ALIGNED_BUFSIZE)      /* RX buffer length is total SRAM minus TX buffer */
#  define PKTMEM_TX_START (PKTMEM_RX_END+1)                 /* Start TX buffer after */
#  define PKTMEM_TX_ENDP1 (PKTMEM_TX_START+ALIGNED_BUFSIZE) /* Allow TX buffer for two frames */

/* Misc. Helper Macros ******************************************************/

#define enc_rdgreg(hnd,ctrlreg)              enc_rdgreg2(hnd, ENC_RCR | GETADDR(ctrlreg))
#define enc_wrgreg(hnd, ctrlreg, wrdata)     enc_wrgreg2(hnd, ENC_WCR | GETADDR(ctrlreg), wrdata)
#define enc_bfcgreg(hnd, ctrlreg,clrbits)    enc_wrgreg2(hnd, ENC_BFC | GETADDR(ctrlreg), clrbits)
#define enc_bfsgreg(hnd, ctrlreg,setbits)    enc_wrgreg2(hnd, ENC_BFS | GETADDR(ctrlreg), setbits)



#if DO_ENC_STATISTIC > 0
    /* Create an error statistic record if configured */
    static ENC_ErrStatusTypeDef errstat;
    #define INC(a) (handle->errstat->a)++ 
#else
    #define INC(a)
#endif

/* External functions --------------------------------------------------------*/
void HAL_Delay(volatile uint32_t Delay);
uint32_t HAL_GetTick(void);

/* Forward declarations *********************************************************/
static void    ENC_SPI_Select                 (bool select);
static uint8_t ENC_SPI_Send                   (uint8_t command);
static uint8_t ENC_SPI_SendWithoutSelection   (uint8_t command);
static void    ENC_SPI_SendBuf                (ENC_HandleTypeDef *handle, uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize);
static void    ENC_SPI_SendBufWithoutSelection(ENC_HandleTypeDef *handle, uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize);
static int32_t ENC_SPI_CheckFree              (ENC_HandleTypeDef *handle);
static void    ENC_IRQHandler                 ( uint16_t pin, uint16_t pinvalue , void *arg );

static __inline void up_udelay(uint32_t us);

/* Stores how many iterations the microcontroller can do in 1 탎 */
static uint32_t iter_per_us=0;

static void calibrate(uint32_t newclk)
{
    UNUSED(newclk);

    uint32_t time;
    volatile uint32_t i;

    iter_per_us = 1000000;

    time = HAL_GetTick();
    /* Wait for next tick */
    while (HAL_GetTick() == time) {
        /* wait */
    }
    for (i=0; i<iter_per_us; i++) {
    }
    iter_per_us /= ((HAL_GetTick()-time)*1000);
    ENCDEBUG("EncCalibrate: %d iter per us\n", iter_per_us);
}

/******************************************************************************
 * active delay function for very short delays 
 *  us: the number of 탎 to wait
 *****************************************************************************/
static __inline void up_udelay(uint32_t us)
{
    volatile uint32_t i;

    for (i=0; i<us*iter_per_us; i++) {
    }
}

/****************************************************************************
 * Function: enc_rdgreg2
 *
 * Description:
 *   Read a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the global address register.
 *
 * Parameters:
 *   cmd   - The full command to received (cmd | address)
 *
 * Returned Value:
 *   The value read from the register
 *
 * Assumptions:
 *
 ****************************************************************************/
static uint8_t enc_rdgreg2(ENC_HandleTypeDef *handle, uint8_t cmd)
{
    uint8_t cmdpdata[2];
    cmdpdata[0] = cmd;

  /* Send the read command and collect the data.  The sequence requires
   * 16-clocks:  8 to clock out the cmd + 8 to clock in the data.
   */

  ENC_SPI_SendBuf(handle, cmdpdata, cmdpdata, 2);

  return cmdpdata[1];
}


/****************************************************************************
 * Function: enc_wrgreg2
 *
 * Description:
 *   Write to a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the global address register.
 *
 * Parameters:
 *   cmd    - The full command to received (cmd | address)
 *   wrdata - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_wrgreg2(ENC_HandleTypeDef *handle, uint8_t cmd, uint8_t wrdata)
{
    uint8_t cmdpdata[2];
    cmdpdata[0] = cmd;
    cmdpdata[1] = wrdata;

    /* Send the write command and data.  The sequence requires 16-clocks:
     * 8 to clock out the cmd + 8 to clock out the data.
     */

    ENC_SPI_SendBuf(handle, cmdpdata, NULL, 2);
}


/****************************************************************************
 * Function: enc_waitgreg
 *
 * Description:
 *   Wait until grouped register bit(s) take a specific value (or a timeout
 *   occurs).
 *
 * Parameters:
 *   ctrlreg - Bit encoded address of banked register to check
 *   bits    - The bits to check (a mask)
 *   value   - The value of the bits to return (value under mask)
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static bool enc_waitgreg(ENC_HandleTypeDef *handle, uint8_t ctrlreg,
                        uint8_t bits, uint8_t value)
{
  uint32_t start = HAL_GetTick();
  uint32_t elapsed;
  uint8_t  rddata;

  /* Loop until the exit condition is met */

  do {
      /* Read the byte from the requested banked register */

      rddata  = enc_rdgreg(handle, ctrlreg);
      elapsed = HAL_GetTick() - start;
  } while ((rddata & bits) != value && elapsed < ENC_POLLTIMEOUT);

  return (rddata & bits) == value;
}


/****************************************************************************
 * Function: enc_waitwhilegreg
 *
 * Description:
 *   Wait while grouped register bit(s) have a specific value (or a timeout
 *   occurs).
 *
 * Parameters:
 *   ctrlreg - Bit encoded address of banked register to check
 *   bits    - The bits to check (a mask)
 *   value   - The value of the bits to return (value under mask)
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static bool enc_waitwhilegreg(ENC_HandleTypeDef *handle, uint8_t ctrlreg,
                        uint8_t bits, uint8_t value)
{
  uint32_t start = HAL_GetTick();
  uint32_t elapsed;
  uint8_t  rddata;

  /* Loop until the exit condition is met */

  do {
      /* Read the byte from the requested banked register */

      rddata  = enc_rdgreg(handle, ctrlreg);
      elapsed = HAL_GetTick() - start;
  } while ((rddata & bits) == value && elapsed < ENC_POLLTIMEOUT);

  return (rddata & bits) != value;
}

/******************************************************************************
 * @brief  Perform a hard reset on enc28j60 RST pin
 * Description:
 * @param  None
 * @retval None
 *****************************************************************************/
static void enc_hwreset(ENC_HandleTypeDef *handle) 
{
    UNUSED(handle);
    ENCDEBUG("HW-Reset\n");
    SpiRstLow(SPI_HANDLE);
    /* Minimal RST low pulse length is 400 ns, so 2us is safe */
    up_udelay( 2 );
    SpiRstHigh(SPI_HANDLE);
    /* Minimal 2us to next reset pulse */
    up_udelay( 2 );
}


/******************************************************************************
 * @brief  Perform a soft reset on enc28j60
 * Description:
 *   Send the single byte system reset command (SRC).
 *
 *   "The System Reset Command (SRC) allows the host controller to issue a
 *    System Soft Reset command.  Unlike other SPI commands, the SRC is
 *    only a single byte command and does not operate on any register. The
 *    command is started by pulling the CS pin low. The SRC opcode is the
 *    sent, followed by a 5-bit Soft Reset command constant of 1Fh. The
 *    SRC operation is terminated by raising the CS pin."
 *
 * @param  None
 * @retval None
 *****************************************************************************/
static void enc_swreset(ENC_HandleTypeDef *handle) 
{
  UNUSED(handle);
  ENCDEBUG("SW-Reset\n");
  /* Send the system reset command. */
  ENC_SPI_Send(ENC_SRC);

  /* Check CLKRDY bit to see when the reset is complete.  There is an errata
   * that says the CLKRDY may be invalid.  We'll wait a couple of msec to
   * workaround this condition.
   *
   * Also, "After a System Reset, all PHY registers should not be read or
   * written to until at least 50 탎 have passed since the Reset has ended.
   * All registers will revert to their Reset default values. The dual
   * port buffer memory will maintain state throughout the System Reset."
   */

  handle->bank = 0; /* Initialize the trace on the current selected bank */
  HAL_Delay(2); /* >1000 탎, conforms to errata #2 */
}


/****************************************************************************
 * Function: enc_setbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 * Assumption:
 *   The caller has exclusive access to the SPI bus
 *
 * Parameters:
 *   handle - Reference to the driver state structure
 *   bank   - The bank to select (0-3)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_setbank(ENC_HandleTypeDef *handle, uint8_t bank) {

  if (bank != handle->bank) {
      // ENCDEBUG("Set bank %d\n", bank);

      /* Select bank 0 (just so that all of the bits are cleared) */
      enc_bfcgreg(handle, ENC_ECON1, ECON1_BSEL_MASK);

      /* Then OR in bits to get the correct bank */
      if (bank != 0) {
          enc_bfsgreg(handle, ENC_ECON1, (bank << ECON1_BSEL_SHIFT));
      }

      /* Then remember the bank setting */
      handle->bank = bank;
  } 
}


/****************************************************************************
 * Function: enc_rdbreg
 *
 * Description:
 *   Read from a banked control register using the RCR command.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to read
 *
 * Returned Value:
 *   The byte read from the banked register
 *
 * Assumptions:
 *
 ****************************************************************************/
static uint8_t enc_rdbreg(ENC_HandleTypeDef *handle, uint8_t ctrlreg)
{
  uint8_t data[3];

  /* Set the bank */

  enc_setbank(handle, GETBANK(ctrlreg));

  /* Send the RCR command and collect the data.  How we collect the data
   * depends on if this is a PHY/CAN or not.  The normal sequence requires
   * 16-clocks:  8 to clock out the cmd and  8 to clock in the data.
   */

  data[0] = ENC_RCR | GETADDR(ctrlreg);

  /* The PHY/MAC sequence requires 24-clocks:  8 to clock out the cmd,
   * 8 dummy bits, and 8 to clock in the PHY/MAC data.
   */

  ENC_SPI_SendBuf(handle, data, data, (ISPHYMAC(ctrlreg))?3:2);
  return (ISPHYMAC(ctrlreg))?data[2]:data[1];
}


/****************************************************************************
 * Function: enc_wrbreg
 *
 * Description:
 *   Write to a banked control register using the WCR command.  Unlike
 *   reading, this same SPI sequence works for normal, MAC, and PHY
 *   registers.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to write
 *   wrdata  - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_wrbreg(ENC_HandleTypeDef *handle, uint8_t ctrlreg,
                       uint8_t wrdata)
{
  uint8_t data[2];

  /* Set the bank */

  enc_setbank(handle, GETBANK(ctrlreg));

  /* Send the WCR command and data.  The sequence requires 16-clocks:
   * 8 to clock out the cmd + 8 to clock out the data.
   */

  data[0] = ENC_WCR | GETADDR(ctrlreg);
  data[1] = wrdata;

  ENC_SPI_SendBuf(handle, data, NULL, 2);
}

/****************************************************************************
 * Function: enc_wrbreg16
 *
 * Description:
 *   Write to a banked 16bit control register using the WCR command.
 *   Make use of the fact, that all 16bit control registers are
 *   organized as both bytes of these registers being organized
 *   as low byte first at consecutive memory locations
 *
 * Parameters:
 *   handle   - Reference to the driver state structure
 *   ctrlreg  - Bit encoded address of banked LO-register to write
 *              the high byte is then automatically at next address
 *   wrdata16 - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_wrbreg16(ENC_HandleTypeDef *handle, uint8_t ctrlreg,
                       uint16_t wrdata16)
{
    enc_wrbreg(handle, ctrlreg, wrdata16 & 0xff);
    enc_wrbreg(handle, ctrlreg+1, wrdata16 >> 8);
}

/****************************************************************************
 * Function: enc_waitbreg
 *
 * Description:
 *   Wait until banked register bit(s) take a specific value (or a timeout
 *   occurs).
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to check
 *   bits    - The bits to check (a mask)
 *   value   - The value of the bits to return (value under mask)
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static bool enc_waitbreg(ENC_HandleTypeDef *handle, uint8_t ctrlreg,
                        uint8_t bits, uint8_t value)
{
  uint32_t start = HAL_GetTick();
  uint32_t elapsed;
  uint8_t  rddata;

  /* Loop until the exit condition is met */

  do
    {
      /* Read the byte from the requested banked register */

      rddata  = enc_rdbreg(handle, ctrlreg);
      elapsed = HAL_GetTick() - start;
    }
  while ((rddata & bits) != value && elapsed < ENC_POLLTIMEOUT);

  return (rddata & bits) == value;
}


/****************************************************************************
 * Function: enc_rdphy
 *
 * Description:
 *   Read 16-bits of PHY data.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   phyaddr - The PHY register address
 *
 * Returned Value:
 *   16-bit value read from the PHY
 *
 * Assumptions:
 *
 ****************************************************************************/
static uint16_t enc_rdphy(ENC_HandleTypeDef *handle, uint8_t phyaddr)
{
  uint16_t data = 0;

  /* "To read from a PHY register:
   *
   *   1. Write the address of the PHY register to read from into the MIREGADR
   *      register.
   */

  enc_wrbreg(handle, ENC_MIREGADR, phyaddr);

  /*   2. Set the MICMD.MIIRD bit. The read operation begins and the
   *      MISTAT.BUSY bit is set.
   */

  enc_wrbreg(handle, ENC_MICMD, MICMD_MIIRD);

  /*   3. Wait 10.24 탎. Poll the MISTAT.BUSY bit to be certain that the
   *      operation is complete. While busy, the host controller should not
   *      start any MIISCAN operations or write to the MIWRH register.
   *
   *      When the MAC has obtained the register contents, the BUSY bit will
   *      clear itself.
   */

  up_udelay(12);

  if (enc_waitbreg(handle, ENC_MISTAT, MISTAT_BUSY, 0x00)) {
      /* 4. Clear the MICMD.MIIRD bit. */

      enc_wrbreg(handle, ENC_MICMD, 0x00);

      /* 5. Read the desired data from the MIRDL and MIRDH registers. The
       *    order that these bytes are accessed is unimportant."
       */

      data  = (uint16_t)enc_rdbreg(handle, ENC_MIRDL);
      data |= (uint16_t)enc_rdbreg(handle, ENC_MIRDH) << 8;
  }

  return data;
}


/****************************************************************************
 * Function: enc_wrphy
 *
 * Description:
 *   write 16-bits of PHY data.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   phyaddr - The PHY register address
 *   phydata - 16-bit data to write to the PHY
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_wrphy(ENC_HandleTypeDef *handle, uint8_t phyaddr,
                      uint16_t phydata)
{
  /* "To write to a PHY register:
   *
   *    1. Write the address of the PHY register to write to into the
   *       MIREGADR register.
   */

  enc_wrbreg(handle, ENC_MIREGADR, phyaddr);

  /*    2. Write the lower 8 bits of data to write into the MIWRL register. */

  enc_wrbreg(handle, ENC_MIWRL, phydata);

  /*    3. Write the upper 8 bits of data to write into the MIWRH register.
   *       Writing to this register automatically begins the MIIM transaction,
   *       so it must be written to after MIWRL. The MISTAT.BUSY bit becomes
   *       set.
   */

  enc_wrbreg(handle, ENC_MIWRH, phydata >> 8);

  /*    The PHY register will be written after the MIIM operation completes,
   *    which takes 10.24 탎. When the write operation has completed, the BUSY
   *    bit will clear itself.
   *
   *    The host controller should not start any MIISCAN or MIIRD operations
   *    while busy."
   */

  /* wait for approx 12 탎 */
  up_udelay(12);
  enc_waitbreg(handle, ENC_MISTAT, MISTAT_BUSY, 0x00);
}


/****************************************************************************
 * Function: enc_rdbuffer
 *
 * Description:
 *   Read a buffer of data.
 *
 * Parameters:
 *   buffer  - A pointer to the buffer to read into
 *   buflen  - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Read pointer is set to the correct address
 *
 ****************************************************************************/
static void enc_rdbuffer(ENC_HandleTypeDef *handle, void *buffer, int16_t buflen)
{

  /* Select ENC28J60 chip */
  ENC_SPI_Select(true);

  /* Send the read buffer memory command (ignoring the response) */
  ENC_SPI_SendWithoutSelection(ENC_RBM);

  /* Then read the buffer data */
  ENC_SPI_SendBuf(handle, NULL, buffer, buflen);

  /* De-select ENC28J60 chip: done in ENC_SPI_SendBuf */
}


/******************************************************************************
 * @brief  Initialize the enc28j60 and configure the needed hardware resources
 * @param  handle:      Handle on data configuration.
 * @param  bFirstInit:  flag for "this is the first init"
 * @retval None
 *****************************************************************************/
bool ENC_Start(ENC_HandleTypeDef *handle, uint32_t bFirstInit)
{
    /* register value */
    uint8_t regval;

    /* Calibrate time constant */
    calibrate(HAL_RCC_GetSysClockFreq());

    /* If we have an hardware reset line, do hardware reset */
    if ( SPI_HANDLE->data->use_rst != 0 ) {
        enc_hwreset(handle);
    }
    /* System reset */
    enc_swreset(handle);

    /* Use bank 0 */
    enc_setbank(handle, 0);

    if ( bFirstInit ) {
        /* Register callback on clock changes for recalibration of wait loop */
        ClockRegisterForClockChange(calibrate);

        #if DO_ENC_STATISTIC > 0
                /* Reset error counters and assign counter record to handle */
                memset(&errstat,0, sizeof(ENC_ErrStatusTypeDef));
                handle->errstat = &errstat;
        #endif
    }

    /* Check if we are actually communicating with the ENC28J60.  If its
     * 0x00 or 0xff, then we are probably not communicating correctly
     * via SPI.
     */

    regval = enc_rdbreg(handle, ENC_EREVID);
    if (regval == 0x00 || regval == 0xff) {
        ENCDEBUG("ENC_Start: ENC28J60 not responding\n");
        return false;
    }
    /* ID 6 means: Revision 7 */
    if ( regval == 6 ) regval = 7;
    ENCDEBUG("ENC28J60 Rev. %d\n", regval);

#if DEBUG_ENC > 0
    uint16_t phyid1 = enc_rdphy(handle, ENC_PHID1);
    uint16_t phyid2 = enc_rdphy(handle, ENC_PHID2);
    ENC1DEBUG("ENC28J60 PhyID1,2= 0x%04x,0x%04x\n", phyid1, phyid2);
#endif

    /* Initialize ECON2: Enable address auto increment. */
    enc_wrgreg(handle, ENC_ECON2, ECON2_AUTOINC /* | ECON2_VRPS*/);

    /* Initialize receive buffer.
     * First, set the receive buffer start address.
     */
    handle->nextpkt = PKTMEM_RX_START;
    enc_wrbreg16(handle, ENC_ERXSTL, PKTMEM_RX_START);
    #if DEBUG_ENC > 1
        ENC2DEBUG("RX Start=0x%04x ", PKTMEM_RX_START);
    #endif

    /* Set the receive data pointer */
    /* Errata 14 */
    enc_wrbreg16(handle, ENC_ERXRDPTL, PKTMEM_RX_END);

    /* Set the receive buffer end. */
    enc_wrbreg16(handle, ENC_ERXNDL, PKTMEM_RX_END);

    #if DEBUG_ENC > 1
        ENC2DEBUG("RX End=0x%04x\n", PKTMEM_RX_END);
    #endif

    /* Set transmit buffer start. */
    enc_wrbreg16(handle, ENC_ETXSTL, PKTMEM_TX_START);

    /* Set filter mode: unicast OR broadcast AND crc valid */
    enc_wrbreg(handle, ENC_ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

    do {
            HAL_Delay(10); /* Wait for 10 ms to let the clock be ready */
            regval = enc_rdbreg(handle, ENC_ESTAT);
    } while ((regval & ESTAT_CLKRDY) == 0);

    /* Enable MAC receive */
    enc_wrbreg(handle, ENC_MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

    /* Enable automatic padding and CRC operations */
    if (handle->Init.DuplexMode == ETH_MODE_HALFDUPLEX) {
        enc_wrbreg(handle, ENC_MACON3,
        ( handle->Init.ChecksumMode == ETH_CHECKSUM_BY_HARDWARE ? MACON3_PADCFG0 | MACON3_TXCRCEN : 0 ) | MACON3_FRMLNEN);
        enc_wrbreg(handle, ENC_MACON4, MACON4_DEFER);        /* Defer transmission enable */

        /* Set Non-Back-to-Back Inter-Packet Gap */
        enc_wrbreg16(handle, ENC_MAIPGL, 0x0c12);

        /* Set Back-to-Back Inter-Packet Gap */
        enc_wrbreg(handle, ENC_MABBIPG, 0x12);
    } else {
        /* Set filter mode: unicast OR broadcast AND crc valid AND Full Duplex */
        enc_wrbreg(handle, ENC_MACON3,
        (handle->Init.ChecksumMode == ETH_CHECKSUM_BY_HARDWARE ? MACON3_PADCFG0 | MACON3_TXCRCEN : 0 ) | MACON3_FRMLNEN | MACON3_FULDPX);

        /* Set Non-Back-to-Back Inter-Packet Gap */
        enc_wrbreg(handle, ENC_MAIPGL, 0x12);

        /* Set Back-to-Back Inter-Packet Gap */
        enc_wrbreg(handle, ENC_MABBIPG, 0x15);
    }

    /* Set the maximum packet size which the controller will accept */
    enc_wrbreg16(handle, ENC_MAMXFLL, ETH_MAX_RX_FRAMESIZE);

    if (handle->Init.DuplexMode == ETH_MODE_HALFDUPLEX) {
        enc_wrphy(handle, ENC_PHCON1, 0x00);
        enc_wrphy(handle, ENC_PHCON2, PHCON2_HDLDIS); /* errata 9 workaround */
    } else {
        enc_wrphy(handle, ENC_PHCON1, PHCON1_PDPXMD); /* errata 10 workaround */
        enc_wrphy(handle, ENC_PHCON2, 0x00);
    }

    /* Set MAC address */
    ENC_SetMacAddr(handle);

    /* Since we not modify PHLCON register, we don't fall in errata 11 case */

    /* Reset all interrupt flags */
    enc_bfcgreg(handle,ENC_EIR, EIR_ALLINTS);

    /* Set Interrupt Callback and enable interrupts*/
    SpiSetInpCB(SPI_HANDLE, ENC_IRQHandler);
    INP_IRQ_Enable(SPI_HANDLE);

    /* Enable packet received interrupt only */
    enc_bfsgreg(handle,ENC_EIE, EIE_PKTIE | EIE_INTIE );

    osDelay(100);

    /* Enable the receiver */
    enc_bfsgreg(handle, ENC_ECON1, ECON1_RXEN);

    return true;
}

/******************************************************************************
 * Restart ENC28J60 Receiver after link up
 *****************************************************************************/
void ENC_Restart(ENC_HandleTypeDef *handle)
{
    /* Reset Rx interrupt flags */
    ENC_RetriggerRxInterrupt(handle);

    /* Reenable the receiver */
    enc_bfsgreg(handle, ENC_ECON1, ECON1_RXEN);

    /* Set Interrupt Callback and enable interrupts*/
    SpiSetInpCB(SPI_HANDLE, ENC_IRQHandler);
    INP_IRQ_Enable(SPI_HANDLE);

}

/******************************************************************************
 * Stop ENC28J60 Receiver on link down
 *****************************************************************************/
void ENC_Stop(ENC_HandleTypeDef *handle)
{
    /* Disable the receiver */
    enc_bfcgreg(handle, ENC_ECON1, ECON1_RXEN);

    /* Reset Interrupt Callback and disable interrupts*/
    INP_IRQ_Disable(SPI_HANDLE);
    SpiSetInpCB(SPI_HANDLE, NULL);
}

/****************************************************************************
 * Function: ENC_SetMacAddr
 *
 * Description:
 *   Set the MAC address to the configured value.  This is done after ifup
 *   or after a TX timeout.  Note that this means that the interface must
 *   be down before configuring the MAC addr.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void ENC_SetMacAddr(ENC_HandleTypeDef *handle)
{
  /* Program the hardware with it's MAC address (for filtering).
   *   MAADR1  MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1
   *   MAADR2  MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2
   *   MAADR3  MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3
   *   MAADR4  MAC Address Byte 4 (MAADR<23:16>)
   *   MAADR5  MAC Address Byte 5 (MAADR<15:8>)
   *   MAADR6  MAC Address Byte 6 (MAADR<7:0>)
   */

  enc_wrbreg(handle, ENC_MAADR1, handle->Init.MACAddr[0]);
  enc_wrbreg(handle, ENC_MAADR2, handle->Init.MACAddr[1]);
  enc_wrbreg(handle, ENC_MAADR3, handle->Init.MACAddr[2]);
  enc_wrbreg(handle, ENC_MAADR4, handle->Init.MACAddr[3]);
  enc_wrbreg(handle, ENC_MAADR5, handle->Init.MACAddr[4]);
  enc_wrbreg(handle, ENC_MAADR6, handle->Init.MACAddr[5]);
}


/****************************************************************************
 * Function: ENC_PrepareBuffer
 *
 * Description:
 *   Prepare the TX buffer by resetting TX buffer start, by setting
 *   ENC_ETXNDL to the passed buflen + 1 ( due to control byte )
 *   and by by setting ENC_EWRPT to TX buffer start
 *
 *   Thereafter write the control byte and the data bytes to the
 *   ENC transmit buffer
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to write from
 *   buflen  - The number of bytes to write
 *
 * Returned Value:
 *   ENC_ERR_OK         on success
 *   ENC_ERR_TIMEOUT    on timeout during wait for completion of ongoing transmisson
 *   END_ERR_MEM        on internal TX buffer overrun ( due to too large tx packet )
 *
 * Assumptions:
 *   Read pointer is set to the correct address
 *
 ****************************************************************************/
static int32_t ENC_PrepareTxBuffer(ENC_HandleTypeDef *handle, struct pbuf *p)
{
    uint16_t buflen, txend;

    struct pbuf *work;

    /* Wait while TX is busy */
    if (!enc_waitgreg(handle, ENC_ECON1, ECON1_TXRTS, 0)) {
        INC(tmoErrCnt);
        return ENC_ERR_TIMEOUT;
    }

    /* Verify that the hardware is ready to send another packet.  The driver
    * starts a transmission process by setting ECON1.TXRTS. When the packet is
    * finished transmitting or is aborted due to an error/cancellation, the
    * ECON1.TXRTS bit will be cleared.
    */

    /* Set transmit buffer start (is this necessary?). */

    enc_wrbreg16(handle, ENC_ETXSTL, PKTMEM_TX_START);

    #if DEBUG_ENCTX > 2
        ENCTX3DEBUG("Tx Start=0x%04x ", PKTMEM_TX_START);
        ENCTX3DEBUG("Tx MaxEnd=0x%04x\n", PKTMEM_TX_ENDP1);
    #endif

  /* Reset the write pointer to start of transmit buffer */

    enc_wrbreg16(handle, ENC_EWRPTL, PKTMEM_TX_START);

    buflen = p->tot_len;
    txend = PKTMEM_TX_START + buflen;

    /* Ensure, the whole buffer will fit into ENC's transmit buffer area */
    if (txend + 8 > PKTMEM_TX_ENDP1) {
        INC(txOvrSizCnt);
        return ENC_ERR_MEM;
    }

    /* Select ENC28J60 chip
    *
    * "The WBM command is started by lowering the CS pin. ..."
    * We explicitly select the ENC28J60 chip because we have to transmits several pieces of
    * information while keeping CS low
    *
    */

    ENC_SPI_Select(true);

    /* Send the write buffer memory command (ignoring the response)
    *
    * "...The [3-bit]WBM opcode should then be sent to the ENC28J60,
    *  followed by the 5-bit constant, 1Ah."
    */
    ENC_SPI_SendWithoutSelection(ENC_WBM);

    /* "...the ENC28J60 requires a single per packet control byte to
    * precede the packet for transmission."
    *
    * POVERRIDE: Per Packet Override bit (Not set):
    *   1 = The values of PCRCEN, PPADEN and PHUGEEN will override the
    *       configuration defined by MACON3.
    *   0 = The values in MACON3 will be used to determine how the packet
    *       will be transmitted
    * PCRCEN: Per Packet CRC Enable bit (Set, but won't be used because
    *   POVERRIDE is zero).
    * PPADEN: Per Packet Padding Enable bit (Set, but won't be used because
    *   POVERRIDE is zero).
    * PHUGEEN: Per Packet Huge Frame Enable bit (Set, but won't be used
    *   because POVERRIDE is zero).
    */
    ENC_SPI_SendWithoutSelection(PKTCTRL_PCRCEN | PKTCTRL_PPADEN | PKTCTRL_PHUGEEN);

    /* Send the buffer
    *
    * "... After the WBM command and constant are sent, the data to
    *  be stored in the memory pointed to by EWRPT should be shifted
    *  out MSb first to the ENC28J60. After 8 data bits are received,
    *  the Write Pointer will automatically increment if AUTOINC is
    *  set. The host controller can continue to provide clocks on the
    *  SCK pin and send data on the SI pin, without raising CS, to
    *  keep writing to the memory. In this manner, with AUTOINC
    *  enabled, it is possible to continuously write sequential bytes
    *  to the buffer memory without any extra SPI command
    *  overhead.
    */

    {
        #if DEBUG_ENCTX > 3 
              uint16_t chklen = 0, segCnt=0;
        #endif
        for ( work = p; work != NULL; work = work->next ) {
            ENC_SPI_SendBufWithoutSelection(handle, (uint8_t *)work->payload, NULL, work->len);
            #if DEBUG_ENCTX > 3 
                chklen += work->len;
                segCnt++;
                ENCTX4DEBUG("Tx write Seg#%d Len=%d\n", segCnt, work->len);
            #endif
        }
        /* De-select ENC28J60 chip */
        ENC_SPI_Select(false);

        ENCTX4DEBUG("Tx %d segments of total sitze %d\n", segCnt, chklen);
    }    

    /* Set the TX End pointer based on the size of the packet to send. Note
    * that the offset accounts for the control byte at the beginning the
    * buffer plus the size of the packet data.
    * txend has to point to the LAST byte of the tx data stream 
    */
    enc_wrbreg16(handle, ENC_ETXNDL, txend);

    ENCTX3DEBUG("Tx End=0x%04x ", txend);

    return ENC_ERR_OK;
}


/****************************************************************************
 * Function: ENC_Transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Parameters:
 *   handle         - Reference to the driver state structure
 *   transmitLength - Length of the frame to transmit EXCLUDING the one
 *                    control byte a begin
 *
 * Returned Value:
 *   none
 *
 * Assumptions:
 *
 ****************************************************************************/
static bool ENC_Transmit(ENC_HandleTypeDef *handle, uint16_t transmitLength)
{
    uint8_t eir, tsv[7];  
    uint16_t work;

    /* A frame is ready for transmission */
    /* Set TXRTS to send the packet in the transmit buffer */

    //enc_bfsgreg(ENC_ECON1, ECON1_TXRTS);
    /* Implement erratas 12, 13 and 15 */
    /* Reset transmit logic */
    handle->retries = 4;


    #if DEBUG_ENCTX > 2
    {   uint16_t start, stop;
        start =  enc_rdbreg(handle, ENC_ETXSTL);
        start |= (uint16_t)enc_rdbreg(handle, ENC_ETXSTH) << 8;
        stop  =  enc_rdbreg(handle, ENC_ETXNDL);
        stop  |= (uint16_t)enc_rdbreg(handle, ENC_ETXNDH) << 8;
        ENCTX3DEBUG("TxBuffer [%04x..%04x], Len=%d\n", start, stop, stop-start+1);
    }
    #endif
    do {
        #if DEBUG_ENCTX > 2
            ENCTX3DEBUG("Transmit: Try#%d, len=%d", 17-handle->retries, transmitLength);
        #endif
        enc_bfsgreg(handle, ENC_ECON1, ECON1_TXRST);
        enc_bfcgreg(handle, ENC_ECON1, ECON1_TXRST);
        enc_bfcgreg(handle, ENC_EIR, EIR_TXERIF | EIR_TXIF);

        /* Start transmission */
        enc_bfsgreg(handle, ENC_ECON1, ECON1_TXRTS);

        /* Wait for end of transmission */
        enc_waitwhilegreg(handle, ENC_EIR, EIR_TXIF | EIR_TXERIF, 0);

        /* Stop transmission */
        enc_bfcgreg(handle, ENC_ECON1, ECON1_TXRTS);

        #if DEBUG_ENCTX > 1
            /* Sanity check: Is ERDPT there where it ought to be ? */
            work =  enc_rdbreg(handle, ENC_ERDPTL);
            work |= (uint16_t)enc_rdbreg(handle, ENC_ERDPTH) << 8;
            ENCTX2DEBUG("ERDPT at %04x\n", work );
        #endif

        work = PKTMEM_TX_START + transmitLength + 1;
        enc_wrbreg16(handle, ENC_ERDPTL, work );

        #if DEBUG_ENCTX > 1
            /* Sanity check: Is ERDPT now a t beginning of TSV ? */
            work =  enc_rdbreg(handle, ENC_ERDPTL);
            work |= (uint16_t)enc_rdbreg(handle, ENC_ERDPTH) << 8;
            ENCTX2DEBUG("ERDPT at %04x\n", work );
        #endif

        ENCTX3DEBUG("Read TSV at %04x\n", work );

         /* read eir and first 4 tytes of tsv */
        enc_rdbuffer(handle, tsv, 4);
        eir = enc_rdgreg(handle, ENC_EIR);

        #if DEBUG_ENCTX > 2
            /* Sanity check: EIR and ESTAT correct ? */
            uint8_t estat = enc_rdgreg(handle, ENC_ESTAT);

            ENCTX3DEBUG("TSV:");
            for ( uint32_t i = 0; i < 4; i++ )
                ENCTX3DEBUG(" %02x", tsv[i]);
            ENCTX3DEBUG(", EIR=0x%02x, ESTAT=0x%02x\n", eir,estat );
        #endif
        
        /* get transmitted frame length */
        work = tsv[1]; work = (work << 8) + tsv[0];

        /* 
         * If the TSV looks suspicious, due to illegal transmitted frame length, do
         * not retry anymore, but return immediately                           
         */
        if (work > CONFIG_NET_ETH_MTU+36) {
            /* Debug: Dump the whole suspicious transmit block */
            #if DEBUG_ENCTX > 2
                uint8_t buff[16];    
                ENCTX3DEBUG("Transmit: Frame Len Err, this is the whole frame:\n");
                work = PKTMEM_TX_START;
                enc_wrbreg(handle, ENC_ERDPTL, work & 0xff);
                enc_wrbreg(handle, ENC_ERDPTH, work >> 8);
                for ( uint32_t i = 0; i <= transmitLength; i += 16 ) {
                    enc_rdbuffer(handle, buff, MIN(i + 16, (uint32_t)transmitLength+1)-i);
                    ENCTX3DEBUG("Txbuff @0x%04x", i+work );
                    for ( uint32_t j = i;  j < MIN(i + 16, (uint32_t)transmitLength+1) ; j++ ) 
                        ENCTX3DEBUG(" %02x", buff[j] );
                    ENCTX3DEBUG("\n");
                } 
                ENCTX3DEBUG("Transmit: This is the whole TSV:\n");
                enc_rdbuffer(handle, buff,7);
                for ( uint32_t i=0; i < 7; i++ )
                    ENCTX3DEBUG(" %02x", buff[i] );
                ENCTX3DEBUG("\n");
            #endif
            INC(txSizErr);
            return false;
        } else if ( (eir & EIR_TXERIF) != 0 || (tsv[3] & TSV_LATECOL) != 0 ) {
            /* Check for Hardware Tx error and late collision */
            if (eir & EIR_TXERIF) { 
                INC(txErrCnt); 
                ENCTX1DEBUG("Transmit: TxErr\n");
            }
            if (tsv[3] & TSV_LATECOL) {
                INC(txLateColl);
                ENCTX1DEBUG("Transmit: Coll\n");
            }
        } else {
            ENCTX2DEBUG("Transmit: Tx ok\n");
           INC(txOkCnt);
           return true;
        }

        handle->retries--;
        INC(txRetries);
    } while (handle->retries > 0);

    /* Transmission finished (but can be unsuccessful) */
    return handle->retries > 0;
}

/****************************************************************************
 * Function: ENC_TransmitBuffer
 *
 * Description:
 *   Transmit a buffer of data.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   p       - A pointer to lwip's pbuf transmit buffer stuct
 *
 * Returned Value:
 *   true      On success
 *   false     On failuer
 *
 * Assumptions:
 *   Disable ENC interrupts during execution. This is due to IRQ handler
 *   itself will interrogate ENC registers, which is a non-atomic SPI-Operation
 *
 ****************************************************************************/
bool ENC_TransmitBuffer(ENC_HandleTypeDef *handle, struct pbuf *p)
{
    bool ret=false;

    ENCTX1DEBUG("Transmission start\n");

    if ( ENC_PrepareTxBuffer(handle, p) == ENC_ERR_OK ) {
        ret = ENC_Transmit(handle, p->tot_len);
    }

    ENCTX1DEBUG("Transmission finished\n");

    return ret;
}



/****************************************************************************
 * Function: ENC_ReceiveStart
 *
 * Description:
 *   Set the ENC read pointer to the start of the received packet and 
 *   read the RSV.
 *   Extract the next rx packet start from rsv and store into handle
 *   Extract the actual packet length from rsv  and store into handle
 *   Read the rx packet status and check fro being ok
 *   If packet is ok, decreas size by 4 ( ie stripoff CRC bytes )
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   ENC_ERR_RX if rx packet is erroneous or out of legal size
 *   ENC_ERR_OK if rx packet is ok
 *
 * Assumptions:
 *   Even if an error is reported, user MUST call "ENC_ReceiveFinish"
 *   in any case to free the occupied rx memory
 *
 ****************************************************************************/

static int32_t ENC_ReceiveStart(ENC_HandleTypeDef *handle)
{
    uint8_t  rsv[6];
    uint16_t rxstat;
    uint16_t pktlen;

    /* Set the read pointer to the start of the received packet (ERDPT) */

    enc_wrbreg16(handle, ENC_ERDPTL, handle->nextpkt);

    /* Read the next packet pointer and the 4 byte read status vector (RSV)
    * at the beginning of the received packet. (ERDPT should auto-increment
    * and wrap to the beginning of the read buffer as necessary)
    */

    enc_rdbuffer(handle, rsv, 6);

    /* Decode the new next packet pointer, and the RSV.  The
    * RSV is encoded as:
    *
    *  Bits 0-15:  Indicates length of the received frame. This includes the
    *              destination address, source address, type/length, data,
    *              padding and CRC fields. This field is stored in little-
    *              endian format.
    *  Bits 16-31: Bit encoded RX status.
    */

    handle->nextpkt = (uint16_t)rsv[1] << 8 | (uint16_t)rsv[0];
    pktlen          = (uint16_t)rsv[3] << 8 | (uint16_t)rsv[2];
    rxstat          = (uint16_t)rsv[5] << 8 | (uint16_t)rsv[4];

    if ( pktlen > 1540 ) {
        ENCRXDEBUG("RxFrame: Spurious length %d\n", pktlen);
        INC(rxLenErr);
        return ENC_ERR_RX;
    }

    ENCRX1DEBUG("RxFrame: Have packet of len=%d\n", pktlen);

  /* Check if the packet was received OK */

    if ((rxstat & RXSTAT_OK) == 0) {
        INC(rxErrCnt);
        ENCRX1DEBUG("RxFrame: Not Ok\n");
        return ENC_ERR_RX;
    } 

    /* regard 4 CRC bytes whecn checking max Rx packet size */
    if (pktlen > ETH_MAX_RX_FRAMESIZE + 4  || pktlen <= (ETH_HDRLEN + 4)) {
       INC(rxHwLenErr);
        ENCRX1DEBUG("RxFrame: Illegal Frame size\n");
        return ENC_ERR_RX;
    }

    /* Remove 4 CRC bytes by reducing the packet length by 4 */
    handle->RxLength = pktlen - 4;
    return ENC_ERR_OK;

}
 
/****************************************************************************
 * Function: ENC_GetReceivedFrame
 *
 * Description:
 *  
 *   Finalize a frame reception, ie move ERXRDPT to start of next packet and
 *   decrement rx packet counter by 1
 *   This will free the rx memory up to ERXRDPT. 
 *  
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void ENC_ReceiveFinish(ENC_HandleTypeDef *handle)
{
    uint16_t work =  handle->nextpkt;

    /* Move the RX read pointer to the start of the next received packet.
    * This frees the memory we just read.
    */

    /* Errata 14 */
    if (work == PKTMEM_RX_START) {
        work = PKTMEM_RX_END;
    } else {
        work--;
    }
    enc_wrbreg16(handle, ENC_ERXRDPTL, work );

    /* Decrement the packet counter indicate we are done with this packet */

    enc_bfsgreg(handle, ENC_ECON2, ECON2_PKTDEC);

}
    

/****************************************************************************
 * Function: ENC_GetReceivedFrame
 *
 * Description:
 *   Check if we have received packet, and if so, retrieve it
 *   into passed buffer
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   rxbuff  - receive buffer to copy received data into
 *   rxlen   - ptr to variable to receive the actual data length
  *
 * Returned Value:
 *   true if new packet is available; false otherwise
 *
 * Assumptions:
 *   we assume, that rxbuff is of sufficient length
 *
 ****************************************************************************/
bool ENC_GetReceivedFrame(ENC_HandleTypeDef *handle, uint8_t *rxbuff, uint32_t *rxlen)
{
    bool ret = false;
    if ( ENC_ReceiveStart(handle) == ENC_ERR_OK ) {
        /* Otherwise, read and process the packet, rxbuff may be NULL in case of no more */
        /* free rxbuffers to allocate, in this case do not copy                          */
        if ( rxbuff ) {
            /* Save the packet length (without the 4 byte CRC) in handle->RxFrameInfos.length*/
            *rxlen = handle->RxLength;

            /* Copy the data data from the receive buffer to priv->dev.d_buf.
             * ERDPT should be correctly positioned from the last call to to
             * end_rdbuffer (above). */
            enc_rdbuffer(handle, rxbuff, *rxlen);
            INC(rxOkCnt);
            ret = true;
        }
    }

    ENC_ReceiveFinish(handle);

    ENCRX1DEBUG("RxFrame: Done\n");

    return ret;
}


/****************************************************************************
 * Function:   ENC_ReceiveIntoPbuf
 *
 * Description:
 *   Recevie function to be used in cooperation with LWIP:
 *   Check if we have received packet, and if so, retrieve it into
 *   a new allocated pbuf
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *   pbuf    - ptr to pbuf variable, which will return the newly
 *             allocated pbuf
 * Returned Value:
 *   true if new packet is available; false otherwise
 *
 * Assumptions:
 *
 ****************************************************************************/
bool  ENC_ReceiveIntoPbuf(ENC_HandleTypeDef *handle, struct pbuf **buf)
{
    bool ret = false;

    if ( ENC_ReceiveStart(handle) == ENC_ERR_OK ) {
        /* allocate pbuf fro received data */
	*buf = pbuf_alloc(PBUF_RAW, handle->RxLength, PBUF_RAM);
        /* if successful allocated,copy data */
	if (*buf) {
            enc_rdbuffer(handle, (*buf)->payload, handle->RxLength);
            INC(rxOkCnt);
            ret = true;
	} else {
            INC(rxPbufErr);
            ENCRX1DEBUG("failed to allocate pbuf of length %u, discarding\n", handle->RxLength);
        }
    }

    /* Free ENC28J60 memory in receive buffer in any case */
    ENC_ReceiveFinish(handle);

    ENCRX1DEBUG("RxFrame: Done\n");

    return ret;
}

/****************************************************************************
 * Function: ENC_RxPacketAvailable
 *
 * Description:
 *  Return the number of Rx packets available in the Rx queue
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   Number of Rx packets fully received
 *
 * Assumptions:
 *
 ****************************************************************************/
uint8_t ENC_RxPacketAvailable(ENC_HandleTypeDef *handle )
{
    uint8_t pktcnt;

    pktcnt = enc_rdbreg(handle, ENC_EPKTCNT);
    ENCRX2DEBUG("RxPackets: %d waiting\n",pktcnt);

    return pktcnt;
}


/****************************************************************************
 * Function: enc_linkstatus
 *
 * Description:
 *   The current link status can be obtained from the PHSTAT1.LLSTAT or
 *   PHSTAT2.LSTAT.
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
static void enc_linkstatus(ENC_HandleTypeDef *handle)
{
  handle->LinkStatus = enc_rdphy(handle, ENC_PHSTAT2);
}

/****************************************************************************
 * Function: ENC_CheckRxStatus
 *
 * Description:
 *   Check the rx packet counter and EIR register and store their values
 *   in then handle variable
 *
 * Parameters:
 *   handle  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/
void ENC_CheckRxStatus(ENC_HandleTypeDef *handle)
{
    handle->PktCnt = enc_rdbreg(handle, ENC_EPKTCNT);
    handle->eir = enc_rdgreg(handle, ENC_EIR);
    #if DEBUG_ENC > 0
        uint8_t eie = enc_rdgreg(handle, ENC_EIE);
        ENC1DEBUG("EIE=%02x, EIR=%02x, PktCnt=%0d\n", eie, handle->eir, handle->PktCnt);
    #endif
}


/****************************************************************************
 * Function: ENC_GetLinkState
 * @brief  Get the link state of ENC28J60 device.
 * @param  handle: Pointer to ENC device handle. 
 * @retval ENC_STATUS_LINK_DOWN  if link is down
 *         ENC_STATUS_10MBITS_FULLDUPLEX  if 10Mb/s FD
 *         ENC_STATUS_10MBITS_HALFDUPLEX  if 10Mb/s HD       
 *         ENC_STATUS_READ_ERROR if connot read register
 *         ENC_STATUS_WRITE_ERROR if connot write to register
 * @note   Enc does only support 10MBIT spped
 *
 * Assumptions:
 *   Disable ENC interrupts during execution. This is due to IRQ handler
 *   itself will interrogate ENC registers, which is a non-atomic SPI-Operation
 *
 ****************************************************************************/

int32_t ENC_GetLinkState(ENC_HandleTypeDef *handle)
{
  int32_t ret;

  /* Inhibit ENC interrupts dureing execution */
  // bool enc_irq_save = ENC_disable_irq();

  enc_linkstatus(handle);  

  /* Check linkstatus bit */
  if ( handle->LinkStatus & PHSTAT2_LSTAT ) {
    /* If set, check fullduplex bit */
    ret = ( handle->LinkStatus & PHSTAT2_DPXSTAT ? ENC_STATUS_10MBITS_FULLDUPLEX : ENC_STATUS_10MBITS_HALFDUPLEX );
  } else {
    /* Return Link Down status */
    ret=  ENC_STATUS_LINK_DOWN;    
  }    

  /* Restore ENC interrupt enable status */
  //ENC_restore_irq(enc_irq_save);

  return ret;
}


/****************************************************************************
 * Function: ENC_DisableInterrupts
 *
 * Description:
 *   Disable NC28J60 global interrupt flag 
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/


/****************************************************************************
 * Function: ENC_RetriggerRxInterrupt
 *
 * Description:
 *   Retrigger the ENC's Rx Interrupt after it has been raised
 *   This has to be done by the sequence
 *    - disable ENC interrupts globally,
 *    - clear interrupt flags
 *    - reenable ENC interrupts globally
 *
 * Parameters:
 *   handle - Ptr to ENC handle
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void ENC_RetriggerRxInterrupt(ENC_HandleTypeDef *handle)
{
    /* Disable ENC Interrupts */
    enc_bfcgreg(handle, ENC_EIE, EIE_INTIE);

    /* Reset ENC28J60 interrupt flags, except PKTIF form which interruption is deasserted when PKTCNT reaches 0 */
    enc_bfcgreg(handle, ENC_EIR, EIR_ALLINTS );

    /* Enable ENC Interrupts  */
    enc_bfsgreg(handle, ENC_EIE, EIE_INTIE);
}

void ENC_RxCpltCallback(void);

/****************************************************************************
 * Function: ENC_IRQHandler
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters, all unused:
 *   pin      - pin number of the associated interrupt pin
 *   pinvalue - value of the associated interrupt pin ( 0 or 1 )
 *   arg - optional user defined argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Ensure that this interrupt will only be triggered, if no other ENC-Operations
 *   are active. This is due to the interrogation of ENC-registers via SPI
 *   which is a non-atomic operation
 *
 ****************************************************************************/
static void ENC_IRQHandler( uint16_t pin, uint16_t pinvalue , void *arg )
{
    UNUSED(pin); UNUSED(pinvalue); UNUSED(arg);


    /* Disable further interrupts by clearing the global interrupt enable bit.
     * "After an interrupt occurs, the host controller should clear the global
     * enable bit for the interrupt pin before servicing the interrupt. Clearing
     * the enable bit will cause the interrupt pin to return to the non-asserted
     * state (high). Doing so will prevent the host controller from missing a
     * falling edge should another interrupt occur while the immediate interrupt
     * is being serviced."
     */

    ENC1DEBUG("iiiii\n");

    ENC_RxCpltCallback();
}


/*===========================================================================*/
/*===========================================================================*/
/* Hardware specific implementation on SPI transfer functions                */
/*===========================================================================*/
/*===========================================================================*/

/**
  * Implement SPI Slave selection and deselection. Must be provided by user code
  * param  select: true if the ENC28J60 slave SPI if selected, false otherwise
  * retval none
  */

static void ENC_SPI_Select(bool select)
{
    if ( select ) {
        SpiNSelLow(SPI_HANDLE);
    } else {
        SpiNSelHigh(SPI_HANDLE);
    }
}

/**
  * Implement SPI single byte send and receive.
  * The ENC28J60 slave SPI must already be selected and wont be deselected after transmission
  * Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

static uint8_t ENC_SPI_SendWithoutSelection(uint8_t command)
{
    return Spi8TxRxByte(SPI_HANDLE, command);
}

/**
  * Implement SPI single byte send and receive. Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

static uint8_t ENC_SPI_Send(uint8_t command)
{
    ENC_SPI_Select(true);
    uint8_t ret = ENC_SPI_SendWithoutSelection(command);
    ENC_SPI_Select(false);
   
    return ret;
}


/**
  * Implement SPI buffer send and receive. Must be provided by user code
  * param  master2slave: data to be sent from host to ENC28J60, can be NULL if we only want to receive data from slave
  * param  slave2master: answer from ENC28J60 to host, can be NULL if we only want to send data to slave
  * retval none
  */

static void ENC_SPI_SendBufWithoutSelection(ENC_HandleTypeDef *handle, uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize)
{
    if ( ENC_SPI_CheckFree(handle) != ENC_ERR_OK ) {
            ENC1DEBUG("SPI Send Error: SPI busy\n");        
    }
    if ( !Spi8TxRxVector(SPI_HANDLE, master2slave, slave2master, bufferSize) ) {
        if (  HAL_SPI_Abort(&SPI_HANDLE->data->hw.myHalHandle ) != HAL_OK ) { ENCDEBUG("SPI IO cannot abort\n"); }
        if ( !Spi8TxRxVector(SPI_HANDLE, master2slave, slave2master, bufferSize) ) {
            INC(spiErrCnt);
            ENC1DEBUG("SPI IO failed\n");
        }
    }
}


/**
  * Implement SPI buffer send and receive. Must be provided by user code
  * param  master2slave: data to be sent from host to ENC28J60, can be NULL if we only want to receive data from slave
  * param  slave2master: answer from ENC28J60 to host, can be NULL if we only want to send data to slave
  * retval none
  */

static void ENC_SPI_SendBuf(ENC_HandleTypeDef *handle, uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize)
{
    ENC_SPI_Select(true);
    ENC_SPI_SendBufWithoutSelection(handle, master2slave, slave2master, bufferSize);
    ENC_SPI_Select(false);
}


/******************************************************************************
 * Check the Hardware SPI interface for being free. If not, wait for becoming
 * free.
 * 
 * Returns: ENC_ERR_TIMEOUT if SPI Hardware is currently locked
 *          ENC_ERR_OK      if SPI hardware is in idle
 * 
 *****************************************************************************/
static int32_t ENC_SPI_CheckFree(ENC_HandleTypeDef *handle)
{
    uint32_t start = HAL_GetTick();
    uint32_t elapsed;
    bool spi_busy;

    /* Loop until the exit condition is met */
    do {
        spi_busy =   ( SPI_HANDLE->data->hw.myHalHandle.Lock != HAL_UNLOCKED 
                  || SPI_HANDLE->data->hw.myHalHandle.State != HAL_SPI_STATE_READY );
        elapsed = HAL_GetTick() - start;
    } while (spi_busy && elapsed < ENC_POLLTIMEOUT);

    if ( elapsed >= ENC_POLLTIMEOUT ) {
        if ( SPI_HANDLE->data->hw.myHalHandle.Lock != HAL_UNLOCKED ) {
            ENC1DEBUG("Spi locked\n");
        }
        if (SPI_HANDLE->data->hw.myHalHandle.State != HAL_SPI_STATE_READY ) {
            ENC1DEBUG("Spi not ready\n");
        }
        INC(spiBusyCnt);
        return ENC_ERR_TIMEOUT;
    } else {
        return ENC_ERR_OK;
    }
}  


static FmtItemT statItems[] = {
    #if DO_ENC_STATISTIC > 0
        {"%5d Link status changes\n",                       &errstat.linkChngCnt,FMT_UINT16 },
        {"Receive statistics: %d packets received ok\n",    &errstat.rxOkCnt,    FMT_UINT32 },
        {"%5d Hardware receive errors\n",                   &errstat.rxErrCnt,   FMT_UINT16 },
        {"%5d Spurious packet len in RxStatusVector\n",     &errstat.rxLenErr,   FMT_UINT16 },
        {"%5d Rx Packet size out of limits\n",              &errstat.rxHwLenErr, FMT_UINT16 },
        {"%5d Error allocating lwip pbuf\n",                &errstat.rxPbufErr,  FMT_UINT16 },
        {"Transmit statistics: %d packets transmitted ok\n",&errstat.txOkCnt,    FMT_UINT32 },
        {"%5d Hardware transmit error\n",                   &errstat.txErrCnt,   FMT_UINT16 },
        {"%5d Late collision\n",                            &errstat.txLateColl, FMT_UINT16 },
        {"%5d Wrong size in TxStatusVector\n",              &errstat.txSizErr,   FMT_UINT16 },
        {"%5d Tx retries due to tx errors\n",               &errstat.txRetries,  FMT_UINT16 },
        {"%5d Timeouts during wait\n",                      &errstat.tmoErrCnt,  FMT_UINT16 },
        {"%5d Oversized Packets\n",                         &errstat.txOvrSizCnt,FMT_UINT16 },
        {"%5d Unexpected busy SPI\n",                       &errstat.spiBusyCnt, FMT_UINT16 },
        {"%5d Hardware SPI error\n",                        &errstat.spiErrCnt,  FMT_UINT16 },
    #else
        {"Statistics not configured\n",                     NULL,               FMT_NULL },
    #endif

};

uint32_t ENCSTAT_GetLineCount(void)
{
    return sizeof(statItems) / sizeof(FmtItemT);
}

FmtItemT *ENCSTAT_GetLine( uint32_t idx )
{
    if ( idx >= ENCSTAT_GetLineCount() ) 
        return NULL;
    else
        return statItems+idx;
}

#endif /* USE_ETH_PHY_ENC28J60 == 1  */

/**
  * @}
  */

/*****END OF FILE****/
