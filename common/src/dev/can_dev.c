/**
 ******************************************************************************
 * @file    can_dev.c
 * @author  Rainer
 * @brief   CAN device functions. 
 */

/** @addtogroup CAN Device Functions
  * @{
  */

#include "config/devices_config.h"

#if USE_CAN > 0 

#if DEBUG_MODE > 0
    #define DEBUG_CAN               1
    #define DEBUG_MIN_ENABLE_DUMP   1
#endif

#include "error.h"
#include "dev/hw_device.h"
#include "dev/can_dev.h"

#include "system/hw_util.h"
#include "config/can_config.h"
#include "debug_helper.h"


/* Supported CAN baudrates in kHz */
static const uint16_t CAN_baudrates[CanSpeedMaxNum] = { 1000, 500, 250, 200, 125, 100, 50, 20, 10 };

/******************************************************************************
 * Neccessary values to setup CAN_BTR properly. Note that these values are the
 * 'real' values, i.e. when wiring them to BTR, they have to be reduced by one,
 * see documentation of CAN_BTR value 
 *****************************************************************************/
struct Can_BaudrateInfo {
    uint16_t psc;    /* Prescaler value                                          */
    uint8_t tq_num;  /* resulting number of time qanta ( valid are 8,12, or 16 ) */
    uint8_t ts1;     /* Number of BS1 Tq                                         */
    uint8_t ts2;     /* Number of BS2 Tq                                         */
    uint8_t sjw;     /* resynchronization Jump Width ( in Tq )                   */   
};

/******************************************************************************
 * Different sample points that can be configured
 *****************************************************************************/
typedef enum {
      CanSample87  = 0,
      CanSample75  = 1,
      CanSample62  = 2,
      CanSampleMaxNum
} Can_SamplePoint;

#if 0
/* CAN_BTR Register Values for differentand different CAN baudrates core speeds */
static const uint32_t I2c_timing[CAN_CORE_SPEED_NUM][I2c_MaxMode] = {
  /*             1000kBd       500kBd      250KBd      125kBd      100kBd      50kBd       20kBd       10kBd       */
  /* 08MHz */ { 0x00050000, 0x001c0000, 0x001c0001, 0x001c0003, 0x001c0004, 0x001c0009, 0x001c0018, 0x001c0031 },
  /* 16MHz */ { 0x001c0000, 0x001c0001, 0x001c0003, 0x001c0007, 0x001c0009, 0x001c0013, 0x001c0031, 0x001c0063 },
  /* 24MHz */ { 0x00050002, 0x001c0002, 0x001c0005, 0x001c000b, 0x001c000e, 0x001c001d, 0x001c004a, 0x001c0095 },
  /* 32MHz */ { 0x001c0001, 0x00050003, 0x001c0007, 0x001c000f, 0x001c0013, 0x001c0027, 0x001c0063, 0x001c00c7 },
  /* 48MHz */ { 0x00D0D2FF, 0x00601851, 0x00400819 },
  /* 80MHz */ { 0x10B0B6CF, 0x00B0298B, 0x0070122A },
};
#endif

/* Samplepoint by default at 75% */
#define CAN_DEFAULT_SAMPLEPOINT     CanSample75

/* Baudrate to use by default */
#define CAN_DEFAULT_BAUDRATE        CanSpeed100

/* Timeout [ms] when polling for state change acknowlege */
#define CAN_TIMEOUT_VALUE 10U

/* local variables -----------------------------------------------------------------*/
static CanTxRxDataT rx0, rx1; /* CAN Receive data structures to pass to user          */ 


/* forward declarations ------------------------------------------------------------*/
bool CAN_InitDev(const HW_DeviceType *self);
void CAN_DeInitDev(const HW_DeviceType *self);



/* Debugging functions ------------------------------------------------------*/
#define PAD_LEN     18

#if DEBUG_CAN >= DEBUG_MIN_ENABLE_DUMP
/* Opmode */
const char * const opmode_txt[]={"Normal","Init", "Sleep"};
static const char* can_get_btr_opmode_txt(uint32_t sel )
{
  if ( sel < sizeof(opmode_txt)/sizeof(char *) ) 
    return opmode_txt[sel];
  else
    return "Illegal";
}

const char * const txrxmode_txt[]={"Normal","Loopback", "Silent", "Silent loopack"};
static const char* can_get_btr_txrxmode_txt(uint32_t sel )
{
  if ( sel < sizeof(txrxmode_txt)/sizeof(char *) ) 
    return txrxmode_txt[sel];
  else
    return "Illegal";
}
const char * const lasterr_txt[]={"No err", "Stuff", "Form", "Ack", "Bit Rec", "Bit Dom", "CRC", "SW"};
static const char* can_get_lasterr_txt(uint32_t sel )
{
  if ( sel < sizeof(lasterr_txt)/sizeof(char *) ) 
    return lasterr_txt[sel];
  else
    return "Illegal";
}


void Can_DecodeBtr( struct Can_BaudrateInfo *br, uint32_t btr )
{
    br->psc = (( btr  & CAN_BTR_BRP_Msk ) >> CAN_BTR_BRP_Pos ) + 1;
    br->ts1 = (( btr  & CAN_BTR_TS1_Msk ) >> CAN_BTR_TS1_Pos ) + 1;
    br->ts2 = (( btr  & CAN_BTR_TS2_Msk ) >> CAN_BTR_TS2_Pos ) + 1;
    br->sjw = (( btr  & CAN_BTR_SJW_Msk ) >> CAN_BTR_SJW_Pos ) + 1;
    br->tq_num = 1 + br->ts1 + br->ts2;
}

void Can_DumpBTR ( uint32_t btr )
{
  struct Can_BaudrateInfo br;  
  uint32_t work;
  char buf[16];
    
  DBG_printf_indent("BTR Settings\n" );
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-2);

  Can_DecodeBtr(&br, btr);
  DBG_dump_number("APB1 Clock", HAL_RCC_GetPCLK1Freq() );
  DBG_dump_number("Can Prescaler", br.psc );
  DBG_dump_number("Can Ts1", br.ts1 );
  DBG_dump_number("Can Ts2", br.ts2 );
  DBG_dump_number("Can SJW", br.sjw );
  DBG_dump_number("Can Tq",  br.tq_num );
  DBG_dump_number("Can baudrate", HAL_RCC_GetPCLK1Freq()/br.psc/br.tq_num );
  work = ( br.ts1+1 ) * 1000 / br.tq_num;
  sprintf(buf,"%d.%d", work/10, work % 10); 
  DBG_dump_textvalue("Sample Point", buf );

  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);
}

void Can_DumpErrorStatus ( uint32_t esr )
{
    
  DBG_printf_indent("CAN error status\n" );
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-2);

  DBG_dump_number("Recv error cnt", (esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos );
  DBG_dump_number("Xmit error cnt", (esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos );
  DBG_dump_textvalue("Last Error", can_get_lasterr_txt((esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos ) );
  DBG_dump_onoffvalue("Bus OFF Flag", esr, CAN_ESR_BOFF_Msk);  
  DBG_dump_onoffvalue("Passive Flag", esr, CAN_ESR_EPVF_Msk);  
  DBG_dump_onoffvalue("Warn Flag", esr, CAN_ESR_EWGF_Msk);  

  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);
}

static void Can_DumpF32(bool bIsMask, uint32_t fr)
{
    bool ide = ( fr >> 2 ) & 1;
    if ( ide )
      DBG_printf_indent("%s: 0x%08x %1x\n", bIsMask ? "Mask" :  "Filt", fr >> 21 | ( fr & 0x1FFFF8 ) << 8 , (fr >> 1) & 1);
    else 
      DBG_printf_indent("%s: 0x%03x %1x\n", bIsMask ? "Mask" :  "Filt", fr >> 21, (fr >> 1) & 1);
}

static void Can_DumpFilter32(bool bIsMask, uint32_t fr1, uint32_t fr2)
{
    Can_DumpF32(false, fr1);
    Can_DumpF32(bIsMask, fr2);

}

static void Can_DumpF16(bool bIsMask, uint16_t fr)
{
    uint32_t fvalue;
    bool ide = ( fr >> 3 ) & 1;
    if ( ide ) {
      fvalue = fr & 0b111;
      fvalue = fvalue << 26 | fr >> 5;
      DBG_printf_indent("%s: 0x%08x %1x\n", bIsMask ? "Mask" :  "Filt", fvalue, (fr >> 4) & 1);
    } else 
      DBG_printf_indent("%s: 0x%03x %1x\n", bIsMask ? "Mask" :  "Filt", fr >> 5, (fr >> 4) & 1);
}

static void Can_DumpFilter16(bool bIsMask, uint16_t fr1, uint16_t fr2)
{
    Can_DumpF16(false, fr1);
    Can_DumpF16(bIsMask, fr2);
}


static void Can_DumpOneFilter ( CAN_TypeDef *hcan, uint32_t idx, uint32_t shift )
{
  DBG_printf_indent("%dbit filter #%02d bound to fifo %d\n", hcan->FS1R & shift ? 32 : 16, idx,  hcan->FFA1R & shift ? 1 : 0 );
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-4);
  if ( hcan->FS1R & shift ) 
    Can_DumpFilter32((hcan->FM1R & shift)==0, hcan->sFilterRegister[idx].FR1, hcan->sFilterRegister[idx].FR2);
  else {
    Can_DumpFilter16((hcan->FM1R & shift)==0, hcan->sFilterRegister[idx].FR1, hcan->sFilterRegister[idx].FR1 >> 16);
    Can_DumpFilter16((hcan->FM1R & shift)==0, hcan->sFilterRegister[idx].FR2, hcan->sFilterRegister[idx].FR2 >> 16);
  }

  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);

    
}

static void Can_DumpReceiveFilters ( CAN_TypeDef *hcan )
{
    
    uint32_t min_bank, max_bank;
    uint32_t can_idx;
    uint32_t shift;

    #if !defined( CAN2 )
        min_bank = 0;
        max_bank = 14;
        can_idx  = 1;
    #else
        if ( CAN1 == hcan ) {
            min_bank = 0;
            max_bank = ( CAN_FMR & CAN_FMR_CANSB_Msk ) >> CAN_FMR_CANSB_Pos;
            can_idx = 1;
        } else {
            min_bank = ( CAN_FMR & CAN_FMR_CANSB_Msk ) >> CAN_FMR_CANSB_Pos;
            max_bank = 28;
            can_idx = 2;
        }
    #endif
    
  DBG_printf_indent("CAN%1d receive filters\n", can_idx );
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-2);

  DBG_dump_onoffvalue("Filter Init Flag", hcan->FA1R, CAN_FMR_FINIT_Msk);

  shift = 1 << min_bank;
  for ( uint32_t i = min_bank; i < max_bank; i++ ) {
    if ( hcan->FA1R & shift ) Can_DumpOneFilter(hcan, i, shift);
    shift <<= 1;
  }
    
  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);
}

static void Can_DumpInterruptSettings( uint32_t reg )
{
  DBG_printf_indent("Interrupt Settingss\n");
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-2);

  DBG_dump_uint32_hex  ("CAN IER raw", reg );  
  DBG_dump_onoffvalue  ("Sleep",  reg, CAN_IER_SLKIE);  
  DBG_dump_onoffvalue  ("WakeUp", reg, CAN_IER_WKUIE);  
  DBG_dump_onoffvalue  ("Glob Err", reg, CAN_IER_ERRIE);  
  DBG_dump_onoffvalue  ("Last Err", reg, CAN_IER_LECIE);  
  DBG_dump_onoffvalue  ("BusOff", reg, CAN_IER_BOFIE);  
  DBG_dump_onoffvalue  ("Passive", reg, CAN_IER_EPVIE);  
  DBG_dump_onoffvalue  ("Warning", reg, CAN_IER_EWGIE);  
  DBG_dump_onoffvalue  ("Fifo1 Overrun", reg, CAN_IER_FOVIE1);  
  DBG_dump_onoffvalue  ("Fifo1 Full", reg, CAN_IER_FFIE1);  
  DBG_dump_onoffvalue  ("Fifo1 Msg Pnd", reg, CAN_IER_FMPIE1);  
  DBG_dump_onoffvalue  ("Fifo0 Overrun", reg, CAN_IER_FOVIE0);  
  DBG_dump_onoffvalue  ("Fifo0 Full", reg, CAN_IER_FFIE0);  
  DBG_dump_onoffvalue  ("Fifo0 Msg Pnd", reg, CAN_IER_FMPIE0);  
  DBG_dump_onoffvalue  ("TxMbx empty", reg, CAN_IER_TMEIE);  

  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);

}

static void Can_DumpReceiverStatus(CAN_TypeDef *inst)
{
  DBG_printf_indent("Interrupt Settingss\n");
  DBG_setIndentRel(+2);
  int oldPadlen = DBG_setPadLen(PAD_LEN-2);
  uint32_t reg;

  reg = inst->RF0R;
  DBG_dump_onoffvalue  ("Overrun0",  reg, CAN_RF0R_FOVR0);  
  DBG_dump_onoffvalue  ("Full0",  reg, CAN_RF0R_FULL0); 
  DBG_dump_number      ("Fifo0 Cnt", ( reg & CAN_RF0R_FMP0_Msk ) >> CAN_RF0R_FMP0_Pos );
  reg = inst->RF1R;
  DBG_dump_onoffvalue  ("Overrun1",  reg, CAN_RF1R_FOVR1);  
  DBG_dump_onoffvalue  ("Full1",  reg, CAN_RF1R_FULL1); 
  DBG_dump_number      ("Fifo1 Cnt", ( reg & CAN_RF1R_FMP1_Msk ) >> CAN_RF1R_FMP1_Pos );

  DBG_setPadLen(oldPadlen);
  DBG_setIndentRel(-2);
}

#endif // DEBUG_CAN > xx

void CAN_DumpCan ( CAN_TypeDef *hcan )
{
  DEBUG_PUTS("CAN configuration -------------------------------" );
  int oldIndent = DBG_setIndentRel(+2);
  #if DEBUG_CAN >= DEBUG_MIN_ENABLE_DUMP
      DBG_setPadLen(PAD_LEN);
      DBG_dump_textvalue("Op mode", can_get_btr_opmode_txt( hcan->MSR & (CAN_MSR_SLAK_Msk|CAN_MSR_INAK_Msk) ));  
      DBG_dump_textvalue("Bus mode", can_get_btr_txrxmode_txt( hcan->BTR  >> CAN_BTR_LBKM_Pos) );  
      Can_DumpBTR(hcan->BTR);
      Can_DumpReceiveFilters(hcan);
       Can_DumpInterruptSettings( hcan->IER );
      Can_DumpErrorStatus(hcan->ESR);
      Can_DumpReceiverStatus(hcan);
  #else
    UNUSED(hcan);
    DBG_printf_indent("Disabled.\n");
  #endif
  DBG_setIndentAbs(oldIndent);
}

 
/* Private or driver functions ------------------------------------------------------*/


/******************************************************************************
 * returns the current can mode bits (INAK, SLAK) from MSR 
 * converted to internal enum CanOpmode
 *****************************************************************************/
CanOpmode Can_GetOpmode ( CanHandleT *h )
{
    CAN_TypeDef *inst = (CAN_TypeDef *)h->hCan.Instance;
    return (CanOpmode)(inst->MSR & ( CAN_MSR_SLAK_Msk | CAN_MSR_INAK_Msk ));
}
/******************************************************************************
 * returns true, if CAN is in NORMAL mode, i.e. SLAK and INAK in MSR are reset
 *****************************************************************************/
static inline bool Can_InNormalMode( CanHandleT *h )
{
     return Can_GetOpmode(h) == CanOpmodeNormal;
}

/******************************************************************************
 * returns true, if CAN is in INIT mode, i.e. INAK in MSR is set
 *****************************************************************************/
static inline bool Can_InInitMode( CanHandleT *h )
{
     return Can_GetOpmode(h) == CanOpmodeInit;
}

/******************************************************************************
 * returns true, if CAN is in SLEEP mode, i.e. SLAK in MSR is set
 *****************************************************************************/
static inline bool Can_InSleepMode( CanHandleT *h )
{
     return Can_GetOpmode(h) == CanOpmodeSleep;
}

/******************************************************************************
 * Switches CAN to INIT mode by setting INRQ and waiting for INAK becoming set
 *****************************************************************************/
static bool Can_ToInitMode( CanHandleT *h )
{
    CAN_TypeDef *inst = (CAN_TypeDef *)h->hCan.Instance;
    /* Get tick */
    uint32_t tickstart = HAL_GetTick();

    inst->MCR |= CAN_MCR_INRQ_Msk;
    inst->MCR &= ~CAN_MCR_SLEEP_Msk;

    while ( ( inst->MSR & CAN_MSR_INAK_Msk ) == 0 ) {
    /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
            h->last_error |= HAL_CAN_ERROR_TIMEOUT;
            return false;
        }
    }
    h->mode = CanOpmodeInit;
    return true;
}

/******************************************************************************
 * Switches CAN to SLEEP mode by setting SLEEP and waiting for SLAK becoming set
 *****************************************************************************/
static bool Can_ToSleepMode( CanHandleT *h )
{
    CAN_TypeDef *inst = (CAN_TypeDef *)h->hCan.Instance;
    /* Get tick */
    uint32_t tickstart = HAL_GetTick();

    inst->MCR |= CAN_MCR_SLEEP_Msk;
    inst->MCR &= ~CAN_MCR_INRQ_Msk;

    while ( ( inst->MSR & CAN_MSR_SLAK_Msk ) == 0 ) {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
            h->last_error |= HAL_CAN_ERROR_TIMEOUT;
            return false;
        }
    }
    h->mode = CanOpmodeSleep;
    return true;
}

/******************************************************************************
 * Switches CAN to NORMAL mode by resetting INRQ and SLEEP and waiting 
 * both corresponding status bits becoming cleared, too
 *****************************************************************************/
static bool Can_ToNormalMode( CanHandleT *h )
{
    CAN_TypeDef *inst = (CAN_TypeDef *)h->hCan.Instance;
    /* Get tick */
    uint32_t tickstart = HAL_GetTick();

    inst->MCR &= ~( CAN_MCR_SLEEP_Msk | CAN_MCR_INRQ_Msk);

    while ( !Can_InNormalMode(h) ) {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE) {
            h->last_error |= HAL_CAN_ERROR_TIMEOUT;
            return false;
        }
    }
    h->mode = CanOpmodeNormal;
    return true;
}

/******************************************************************************
 * Set the can mode to 'm'
 * both corresponding status bits becoming cleared, too
 *****************************************************************************/
static void Can_SetMode ( CanHandleT *h, CanOpmode m )
{
    switch ( m ) {
        case CanOpmodeNormal: Can_ToNormalMode(h); break; 
        case CanOpmodeInit:   Can_ToInitMode(h);   break; 
        case CanOpmodeSleep:  Can_ToSleepMode(h);  break; 
    }
}

/******************************************************************************
 * Clear the entire CanHandleT structure
 *****************************************************************************/
static void CanResetMyHandle ( CanHandleT *handle ) 
{
    memset(handle, 0, sizeof(CanHandleT) );
}


static void Can_GPIO_Init(const HW_DeviceType *self)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable CAN clock */
    HW_SetHWClock((CAN_TypeDef *)self->devBase, 1);

    /* Configure CAN_RX and CAN_TX pins */  

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    GpioAFInitAll(self->devGpioAF, &GPIO_InitStruct);
}

static void CAN_GPIO_DeInit(const HW_DeviceType *self)
{
    /* Disable GPIO Pins */
    GpioAFDeInitAll(self->devGpioAF);
    /* Disable CAN clock */
    HW_SetHWClock((CAN_TypeDef *)self->devBase, 0);
}


/******************************************************************************
 * Set the sample point values according to the alread set number of Tq in "brinfo"
 *****************************************************************************/
static void Can_SetTs(struct Can_BaudrateInfo *br, const Can_SamplePoint sp)
{
    switch ( br->tq_num ) {
        case 16:
            br->ts2 = (sp+1)*2;     /* 2, 4 or 6  for 87,5, 75 or 62,5 % */
            br->ts1 = br->tq_num - 1 - br->ts2;
            return;
        case 12:
            br->ts2 = sp+2;        /* 2, 3, 4  for 83, 75 or 67,5 % */
            br->ts1 = br->tq_num - 1 - br->ts2;
            return;         
        case 8:
            br->ts2 = sp+1;        /* 1, 2, 3  for 87,5, 75 or 62,5 % */
            br->ts1 = br->tq_num - 1 - br->ts2;
            return;
        default:
            DEBUG_PRINTF("Error: No suitable setting for TS1, TS2 in Can_SetTs");
   }
}

/******************************************************************************
 * Set the can Baudrate parameters prescaler, Ts1, Ts2 and Sjw for a desired
 * Can baudrate, the desired sample pont  and the current APB1 Clock
 * 
 * Note: SJW is always set to 1
 * Note: The number of Tq is tried to give 16, if not possible due to PCLK
 *       12 and minimum 8 Tq are tried. If also impossible, false is returned
 * The neccessary parameters are returned in "br" pointer
 *****************************************************************************/
static bool Can_GetBTR ( struct Can_BaudrateInfo *br, const CanSpeedModeEnum my_br, const Can_SamplePoint sp)
{
    /* CAN is connected to APB1, get input clock and convert to kHz */
    uint32_t pclk = HAL_RCC_GetPCLK1Freq() / 1000;

    /* Get plain baudrate [kBd] and caluculate the resulting prescaler value */
    uint16_t baudrate = CAN_baudrates[my_br];
    uint32_t prescaler;

    /* Resync Step with is always */
    br->sjw = 1;

    /* Try with a decreasing number of Tq, max. is 16 */
    ;
    if ( (prescaler = pclk / ( baudrate * 16 ), pclk == prescaler * baudrate * 16) ) {
        br->tq_num = 16;
        br->psc = prescaler;
    } 
    /* next try to achieve 12 Tq */
    else if ( (prescaler = pclk / ( baudrate * 12), pclk == prescaler * baudrate * 12) ) {
        br->tq_num = 12;
        br->psc = prescaler;
    } 
    /* finally try to achieve 8 Tq */
    else if ( (prescaler = pclk / ( baudrate * 8), pclk == prescaler * baudrate * 8) ) {
        br->tq_num = 8;
        br->psc = prescaler;
    } else {
        /* could not calculate a "good" setting for CAN_BTR */
        DEBUG_PRINTF("Could not set CAN_BTR for PCLK=%d and desired CanBR=%dkBd\n",pclk*1000, baudrate);
    }

    Can_SetTs(br, sp);

    DEBUG_PRINTF("CAN_BTR for PCLK=%d and CanBR=%dkBd:\n",pclk*1000, baudrate);
    DEBUG_PRINTF("Nr of Tq =%d\n", br->tq_num);
    DEBUG_PRINTF("Prescaler=%d\n", br->psc);
    DEBUG_PRINTF("TS1      =%d\n", br->ts1);
    DEBUG_PRINTF("TS2      =%d\n", br->ts2);

    return true;
}

/******************************************************************************
 * Set BTR register on basis of br values
 *****************************************************************************/
static void Can_SetBTR(CAN_TypeDef *inst, struct Can_BaudrateInfo *br)
{
    /* Mask for all bits that determine the Can Baudrate */
    #define CAN_BTR_BITRATE_MASK  ( CAN_BTR_SJW_Msk | CAN_BTR_TS2_Msk | CAN_BTR_TS1_Msk | CAN_BTR_BRP_Msk )

    __IO uint32_t *btr = &inst->BTR;

    /* Reset all baudrate determining bits */
    *btr &= ~(CAN_BTR_BITRATE_MASK);

    uint32_t work = br->psc-1;
    work <<= CAN_BTR_BRP_Pos;
    work &= CAN_BTR_BRP_Msk;
    *btr |= work;

    work = br->ts1-1;
    work <<= CAN_BTR_TS1_Pos;
    work &= CAN_BTR_TS1_Msk;
    *btr |= work;

    work = br->ts2-1;
    work <<= CAN_BTR_TS2_Pos;
    work &= CAN_BTR_TS2_Msk;
    *btr |= work;

    work = br->sjw-1;
    work <<= CAN_BTR_SJW_Pos;
    work &= CAN_BTR_SJW_Msk;
    *btr |= work;
}


bool CAN_ChangeBaudrate(CanHandleT *myHandle, CanSpeedModeEnum new_br)
{
    struct Can_BaudrateInfo br;
    CanOpmode old_mode;

    /* Set to INIT mode */
    old_mode = myHandle->mode;
    if ( old_mode != CanOpmodeInit ) Can_ToInitMode(myHandle);

    /* check, whether desired baudrate can be set */
    if ( !Can_GetBTR ( &br, new_br, CAN_DEFAULT_SAMPLEPOINT ) ) return false;

    Can_SetBTR(myHandle->hCan.Instance, &br); 
    DEBUG_PRINTF("CAN_BTR=0x%08x\n", myHandle->hCan.Instance->BTR);

    /* Resume NORMAL mode, if that was the old mode */
    if ( old_mode != CanOpmodeInit ) Can_SetMode(myHandle, old_mode );
    return true;
}

bool CAN_SetBusMode(CanHandleT *myHandle, CanBusMode busmode)
{
    CanOpmode old_mode;
    CAN_TypeDef *inst = myHandle->hCan.Instance;

    /* Set to INIT mode */
    old_mode = myHandle->mode;
    if ( old_mode != CanOpmodeInit ) Can_ToInitMode(myHandle);

    /* Reset SLIM and LBKM Bit */
    inst->BTR &= ~(CAN_BTR_SILM_Msk | CAN_BTR_LBKM_Msk);

    /* Set mode as requested */
    inst->BTR |= busmode << CAN_BTR_LBKM_Pos;

    /* Resume NORMAL mode, if that was the old mode */
    if ( old_mode != CanOpmodeInit ) Can_SetMode(myHandle, old_mode );
    return true;
}


void CAN_Setup32BitFilter( CanFilterT *flt, uint32_t id, uint8_t rtrId, uint32_t mask, uint8_t rtrMask, uint8_t bIsIdList, uint8_t bIsEID, uint8_t fifonum)
{
    if ( bIsEID ) {
        flt->flt[0].flt32Id = MKEXTFLT32(id, rtrId);
        flt->flt[1].flt32Id = MKEXTFLT32(mask, rtrMask);
    } else {
        flt->flt[0].flt32Id = MKSTDFLT32(id, rtrId);
        flt->flt[1].flt32Id = MKSTDFLT32(mask, rtrMask);
    }
    flt->bIsEID    = bIsEID;
    flt->bIsIdList = bIsIdList;
    flt->bIs32Bit  = 1;
    flt->fifonum   = fifonum;
}
void CAN_Setup16BitFilter( CanFilterT *flt, uint8_t fltIdx, uint32_t id, uint8_t rtrId, uint32_t mask, uint8_t rtrMask, uint8_t bIsIdList, uint8_t bIsEID, uint8_t fifonum)
{
    CanFilterElementU *ptr = ( fltIdx ? &(flt->flt[1]) :&(flt->flt[0]) );

    if ( bIsEID ) {
        ptr->flt16Id1 = MKEXTFLT16(id, rtrId);
        ptr->flt16Id2 = MKEXTFLT16(mask, rtrMask);
    } else {
        ptr->flt16Id1 = MKSTDFLT16(id, rtrId);
        ptr->flt16Id2 = MKSTDFLT16(mask, rtrMask);
    }
    flt->bIsEID    = bIsEID;
    flt->bIsIdList = bIsIdList;
    flt->bIs32Bit  = 0;
    flt->fifonum   = fifonum;
}

/******************************************************************************
  * @brief  Configures the CAN reception filter according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @retval true, if filter setting was successful
  ****************************************************************************/
bool CAN_SetFilter(CAN_TypeDef *hcan, CanFilterT *flt, uint8_t fltnum, uint8_t bActivate)
{

    uint32_t filternbrbitpos;
    CAN_TypeDef *can_ip = hcan;
    

#if   defined(CAN2)
    /* CAN1 and CAN2 are dual instances with 28 common filters banks */
    /* Select master instance to access the filter banks */
    can_ip = CAN1;

    /* Check the parameters */
    assert_param(IS_CAN_FILTER_BANK_DUAL(fltnum));
    assert_param(IS_CAN_FILTER_BANK_DUAL(sFilterConfig->SlaveStartFilterBank));
#else
    /* CAN1 is single instance with 14 dedicated filters banks */

    /* Check the parameters */
    assert_param(IS_CAN_FILTER_BANK_SINGLE(fltnum));
#endif

    /* Initialisation mode for the filter */
    SET_BIT(can_ip->FMR, CAN_FMR_FINIT);

#if   defined(CAN2)
    /* Select the start filter number of CAN2 slave instance */
    CLEAR_BIT(can_ip->FMR, CAN_FMR_CAN2SB);
    SET_BIT(can_ip->FMR, sFilterConfig->SlaveStartFilterBank << CAN_FMR_CAN2SB_Pos);

#endif
    /* Convert filter number into bit position */
    filternbrbitpos = (uint32_t)1 << (fltnum & 0x1FU);

    /* Filter Deactivation */
    CLEAR_BIT(can_ip->FA1R, filternbrbitpos);

    if (bActivate) {

        /* Filter Scale */
        if (flt->bIs32Bit) {
            /* 32-bit scale for the filter */
            SET_BIT(can_ip->FS1R, filternbrbitpos);
        } else {
            /* 16-bit scale for the filter */
            CLEAR_BIT(can_ip->FS1R, filternbrbitpos);
        }

        can_ip->sFilterRegister[fltnum].FR1 = flt->flt[0].flt32Id;
        can_ip->sFilterRegister[fltnum].FR2 = flt->flt[1].flt32Id;

        /* Filter Mode */
        if (flt->bIsIdList) {
            /* Identifier list mode for the filter*/
            SET_BIT(can_ip->FM1R, filternbrbitpos);
        }  else { 
            /* Id/Mask mode for the filter*/
            CLEAR_BIT(can_ip->FM1R, filternbrbitpos);
        }

        /* Filter FIFO assignment, anything except 0 for fifo assignment will select fifo 1 */
        if (flt->fifonum) {
            /* FIFO 1 assignation for the filter */
            SET_BIT(can_ip->FFA1R, filternbrbitpos);
        } else {
            /* FIFO 0 assignation for the filter */
            CLEAR_BIT(can_ip->FFA1R, filternbrbitpos);
        }

        /* Activate filter */
        SET_BIT(can_ip->FA1R, filternbrbitpos);
    }

    /* Leave the initialisation mode for the filter */
    CLEAR_BIT(can_ip->FMR, CAN_FMR_FINIT);

    /* Return function status */
    return true;
}

/******************************************************************************
 * Set ALL CAN Callback functions. 
 * Parameter NULL means "no Callback"
 *****************************************************************************/
void CAN_RegisterCallbacks(CanHandleT *myHandle, CanRxCb RxCb, CanTxCb TxCb, CanErrCb ErrCb)
{
    myHandle->CanOnRx  = RxCb;
    myHandle->CanOnTx  = TxCb;
    myHandle->CanOnErr = ErrCb;
}

bool CAN_Start( CanHandleT *me )
{
    /* Nothing to do if already in normal mode */
    if (me->mode == CanOpmodeNormal) return true;

    return Can_ToNormalMode(me);
}


bool CAN_Stop( CanHandleT *me )
{
    if (me->mode == CanOpmodeNormal) {
        /* Request initialisation */
        if  ( !Can_ToInitMode(me) ) return false;
    }
    return true;
}

/******************************************************************************
 * @brief  Return the number of free tx mailboxes
 *****************************************************************************/
bool CAN_IsTxMboxFree(CanHandleT *me)
{
    CAN_TypeDef *inst   = me->hCan.Instance;

    /* Check for any Tx Mailbox empty*/
    return (inst->TSR & ( CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2 )) != 0;
}

/******************************************************************************
 * @brief  Transmit a Can message using the first available tx mailbox
 * @param  me - Handle to us
 * @param  tx - Can transmit data packet
 * @return Number of uses Tx mailbox or 0 in case of failure
 * @note   Before calling this function, check the availability of any tx 
           mailbox. If this fn is called with no free Mbox, 0 will be returned
 *****************************************************************************/
CanTxMboxNum CAN_Transmit( CanHandleT *me, CanTxRxDataT *tx )
{
    CAN_TypeDef *inst   = me->hCan.Instance;
    uint32_t txmbx;

    /* Check availability of Tx mailbox */
    if (!CAN_IsTxMboxFree(inst) ) return 0;

    /* Select an empty transmit mailbox */
    txmbx = (inst->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
   CAN_TxMailBox_TypeDef *mbox = &inst->sTxMailBox[txmbx];
    /* Fill in all data items */
    /* Id: No masking neccessary, ID will be shifted to the top of 32 bit register in any case */
    if (tx->IDE) {
        mbox->TIR = ((tx->Id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE );
    } else {
        mbox->TIR = ((tx->Id << CAN_TI0R_STID_Pos) | tx->RTR);
    }
    if ( tx->RTR ) mbox->TIR |= CAN_TI0R_RTR;

    /* Set up the DLC */
    mbox->TDTR = (tx->DLC);

    /* Set up the Transmit Global Time mode */
    //RHB tbd if (tx->TransmitGlobalTime == ENABLE) SET_BIT(mbox->TDTR, CAN_TDT0R_TGT);

    /* copy data bytes */
    if ( tx->DLC > 0 ) {
        uint8_t *wrptr = (uint8_t*)&mbox->TDLR;
        uint8_t *rdptr = tx->data;
        for ( uint32_t i = 0; i < tx->DLC; i++ ) {
            *(wrptr++) = *(rdptr++);
        }
    }

    /* Request transmission */
    SET_BIT(mbox->TIR, CAN_TI0R_TXRQ);

    return (uint32_t)1 << txmbx;;
}



/*eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 * Always implemented error handler stub
 * What to do: Get Error reason, code into error variable and call user 
 * error handler, if set
 eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee*/
void CAN_ErrorStub( CanHandleT *me )
{
    CAN_TypeDef *inst   = me->hCan.Instance;
    uint32_t msrflags   = inst->MSR;
    uint32_t interrupts = inst->IER;
    uint32_t esrflags   = inst->ESR;

    /* Read master error flag */
    if ((msrflags & CAN_MSR_ERRI) ) {
        /* Check Error Warning Flag */
        if ( (interrupts & CAN_IT_ERROR_WARNING ) && (esrflags & CAN_ESR_EWGF) ) me->last_error |= HAL_CAN_ERROR_EWG;

        /* Check Error Passive Flag */
        if ( (interrupts & CAN_IT_ERROR_PASSIVE) && (esrflags & CAN_ESR_EPVF)) me->last_error |= HAL_CAN_ERROR_EPV;

        /* Check Bus-off Flag */
        if ( (interrupts & CAN_IT_BUSOFF ) &&  (esrflags & CAN_ESR_BOFF) )     me->last_error |= HAL_CAN_ERROR_BOF;

        if ( (interrupts & CAN_IT_LAST_ERROR_CODE) && (esrflags & CAN_ESR_LEC) ) {
            switch (esrflags & CAN_ESR_LEC) {
            case (CAN_ESR_LEC_0):
                /* Set CAN error code to Stuff error */
                me->last_error |= HAL_CAN_ERROR_STF;
                break;
            case (CAN_ESR_LEC_1):
                /* Set CAN error code to Form error */
                me->last_error |= HAL_CAN_ERROR_FOR;
                break;
            case (CAN_ESR_LEC_1 | CAN_ESR_LEC_0):
                /* Set CAN error code to Acknowledgement error */
                me->last_error |= HAL_CAN_ERROR_ACK;
                break;
            case (CAN_ESR_LEC_2):
                /* Set CAN error code to Bit recessive error */
                me->last_error |= HAL_CAN_ERROR_BR;
                break;
            case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
                /* Set CAN error code to Bit Dominant error */
                me->last_error |= HAL_CAN_ERROR_BD;
                break;
            case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
                /* Set CAN error code to CRC error */
                me->last_error |= HAL_CAN_ERROR_CRC;
                break;
            default:
                /* Should not happen, set internal error code */
                me->last_error |= HAL_CAN_ERROR_INTERNAL;
                break;
            } // switch

            /* Clear Last error bits */
            CLEAR_BIT(inst->ESR, CAN_ESR_LEC);
        } // if ( (interrupts & CAN_IT_LAST_ERROR_CODE) && (esrflags & CAN_ESR_LEC) ) 

        /* Clear ERRI Flag by writing 1 */
        SET_BIT(inst->MSR, CAN_MSR_ERRI);
    } // if ((msrflags & CAN_MSR_ERRI) )
    
    /* Check for Fifo 0 and 1 overrun */ 
    if ( (interrupts & CAN_IT_RX_FIFO0_OVERRUN ) &&  (inst->RF0R & CAN_RF0R_FOVR0_Msk) ) me->last_error |= HAL_CAN_ERROR_RX_FOV0;
    if ( (interrupts & CAN_IT_RX_FIFO1_OVERRUN ) &&  (inst->RF1R & CAN_RF1R_FOVR1_Msk) ) me->last_error |= HAL_CAN_ERROR_RX_FOV1;

    /* Call user error handle, if specified */
    if ( me->CanOnErr ) me->CanOnErr(me->last_error);

    #if DEBUG_CAN > 0
        DEBUG_PRINTF("CAN error #%d\n", me->last_error);
    #endif
    /* Reset last Errors */
    me->last_error = 0;
}


static void Can_Receive( CAN_TypeDef *inst, uint32_t RxFifo, CanTxRxDataT *rx )
{
    /* Fill in the header data */
    /* Get the header */
    rx->IDE = ( CAN_RI0R_IDE & inst->sFIFOMailBox[RxFifo].RIR ) != 0;
    if (rx->IDE )    {
        /* Copy extended ID to Id field */
        rx->Id = ((CAN_RI0R_EXID | CAN_RI0R_STID) & inst->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    } else {
        /* Copy std ID to Id field */
        rx->Id = (CAN_RI0R_STID & inst->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos;
    }

    /* Copy other header data itmes to structure */
    rx->RTR         = (CAN_RI0R_RTR   & inst->sFIFOMailBox[RxFifo].RIR)  >> CAN_RI0R_RTR_Pos;
    rx->DLC         = (CAN_RDT0R_DLC  & inst->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    rx->FltMatchIDx = (CAN_RDT0R_FMI  & inst->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    rx->Timestamp   = (CAN_RDT0R_TIME & inst->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Copy Ptr to data */
    rx->data        = (uint8_t *)(&inst->sFIFOMailBox[RxFifo].RDLR);
}

/*eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 * Can Receive Callback: Check for Fifo Overrun 
 * Call user callback, release fifo element
 * error handler, if set
 eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee*/
void CAN_RxIrqStub(CanHandleT *me, uint32_t fifonum )
{
    CAN_TypeDef *inst   = me->hCan.Instance;
    uint32_t interrupts = inst->IER;
 
    /* Check validity of fifo */
    assert_param(IS_CAN_RX_FIFO(fifonum));

    /* Check for Fifo 0 and 1 overrun first */ 
    if ( fifonum == 0 ) {
        if ( ( interrupts & CAN_IT_RX_FIFO0_OVERRUN ) &&  (inst->RF0R & CAN_RF0R_FOVR0_Msk) ) CAN_ErrorStub(me);
        if ( inst->RF0R & CAN_RF0R_FMP0_Msk ) {
            if (  me->CanOnRx ) { 
                Can_Receive(inst, fifonum, &rx0);
                me->CanOnRx(fifonum, &rx0);
            }
            /* Release element */
            SET_BIT( inst->RF0R, CAN_RF0R_RFOM0 );
        }
    } else {
        /* Fifo 1 */
        if ( ( interrupts & CAN_IT_RX_FIFO1_OVERRUN ) &&  (inst->RF1R & CAN_RF1R_FOVR1_Msk) ) CAN_ErrorStub(me);
        if ( inst->RF1R & CAN_RF1R_FMP1_Msk ) {
            if (  me->CanOnRx ) { 
                Can_Receive(inst, fifonum, &rx1);
                me->CanOnRx(fifonum, &rx1);
            }
            /* Release element */
            SET_BIT( inst->RF1R, CAN_RF1R_RFOM1 );
        }
    }
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, whenever CAN is in SLEEP or INIT Mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool CAN_FrqChange(const HW_DeviceType *self)
{
    CanHandleT* me = (CanHandleT*)self->devData;
    return CAN_ChangeBaudrate( me, me->baudrate );
}


/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Check, whether the system may enter Stop 2 mode. 
 * This is the case, whenever CAN is in SLEEP or INIT Mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool CAN_CanStop(const HW_DeviceType *self)
{
    CAN_TypeDef *inst = (CAN_TypeDef*)self->devBase;
    
    return inst->MSR & ( CAN_MSR_SLAK_Msk | CAN_MSR_INAK_Msk );
}


/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Can Device initialization: Reset handle, init GPIO pins and interrupts,
 * set default baudrate and normal bus mode, CAN will remain in sleep mode
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
bool CAN_InitDev(const HW_DeviceType *self)
{
    CanHandleT* me    = (CanHandleT*)self->devData;
    CAN_TypeDef *inst = (CAN_TypeDef *)self->devBase;

    /* Initialize my handle to 'fresh' */
    CanResetMyHandle(me);

    /* In the embedded Handle only set the Instance member */
    me->hCan.Instance = inst;

    /* Init GPIO and Clocks */
    Can_GPIO_Init(self);

    /* Configure the NVIC, enable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, true);

    /* Set to INIT mode */
    Can_ToInitMode(me);

    /* Init to default baudrate and to normal busmode*/
    CAN_ChangeBaudrate(me, CAN_DEFAULT_BAUDRATE);
    CAN_SetBusMode(me, CanBusModeNormal);

    /* Set Error Interrupt Flags */
    inst->IER = CAN_IER_ERRIE | CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE | CAN_IER_LECIE | CAN_IER_FOVIE0 | CAN_IER_FOVIE1;

    /* Set transmit & receive interrupt flasgs */
    inst->IER |= CAN_IER_FOVIE0 | CAN_IER_FMPIE0 | CAN_IER_FOVIE1 | CAN_IER_FMPIE1; 

  return true;
}

/*ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
 * Reset CAN peripheral, deassign interrupts and DeInit GPIO Pins
 *ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd*/
void CAN_DeInitDev(const HW_DeviceType *self)
{
    /*Reset peripherals */
    HW_Reset((CAN_TypeDef *)self->devBase );

    CAN_GPIO_DeInit(self);

    /* Disable interrupts */
    if ( self->devIrqList) HW_SetAllIRQs(self->devIrqList, false);

    /* Disable the DMA, if used */
}

/* Static configurations ---------------------------------------------------------*/

#if defined(CAN1) && defined(USE_CAN1)
    CanHandleT CAN1Handle;

    static const HW_GpioList_AF gpio_can1 = {
        .gpio = { CAN1_RX_PIN, CAN1_TX_PIN },
        .num = 2, 
    };

    #ifdef CAN1_USE_IRQ
        static const HW_IrqList irq_can1 = {
            .num = 4,
            .irq = { CAN1_TX_IRQ, CAN1_RX0_IRQ, CAN1_RX1_IRQ, CAN1_SCE_IRQ  },
        };
    #endif

    const HW_DeviceType HW_CAN1 = {
        .devName        = "CAN1",
        .devBase        = CAN1,
        .devGpioAF      = &gpio_can1,
        .devGpioIO      = NULL,
        .devType        =  HW_DEVICE_CAN,
        .devData        = &CAN1Handle,
        .devIrqList     = 
        #if defined(CAN1_USE_IRQ)
            &irq_can1,
        #else
            NULL,
        #endif
        /* No DMA for CAN */
        .devDmaTx = NULL,
        .devDmaRx = NULL,
        .Init           = CAN_InitDev,
        .DeInit         = CAN_DeInitDev,
        .OnFrqChange    = CAN_FrqChange,
        .AllowStop      = CAN_CanStop,
        .OnSleep        = NULL,
        .OnWakeUp       = NULL,
    };
#endif



#endif // USE_CAN



