/**
  ******************************************************************************
  * @file    encoding.c
  * @author  Rainer 
  * @brief   Implementation of XTEA encode/decode and CMAC Algorithm
  ******************************************************************************
  *
  ******************************************************************************
  * @addtogroup Encoding
  * @{
  */
/* Includes ------------------------------------------------------------------*/

#include "config/config.h"
#include "debug_helper.h"
#include "error.h"
#include "encoding.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Keys[5*2]; // 40 bytes
#define K_mac (Keys+0*2)
#define K_enc (Keys+1*2)
#define K1 (Keys+3*2)
#define K2 (Keys+4*2)
#define K_m (Keys+3*2)  /* share same position as K1 & K2 */

#undef CMACDEBUG
// #define CMACDEBUG 1

static uint8_t Km_upper[8] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
};

/* Private function prototypes -----------------------------------------------*/
static void left_roll_8bytes_with_carry ( uint8_t *src, uint8_t *dest, uint8_t len);
/* Private functions ---------------------------------------------------------*/
/*!
 *******************************************************************************
 *  init crypto keys
 ******************************************************************************/

void encoder_init(void) {
    uint8_t i;
    uint8_t *ptr8;
    // RHB tbd change to security key bytes
    memcpy((uint8_t*)K_m,Km_upper,8);

    memcpy(((uint8_t*)K_m)+8,Km_upper,sizeof(Km_upper)); 
    ptr8 = (uint8_t*)Keys;
    for (i=0;i<3*8;i++) {
        ptr8[i]=0xc0+i;
    }
    XTEA_ENC(K_mac, K_mac, K_m); /* generate K_mac low 8 bytes */
    XTEA_ENC(K_enc, K_enc, K_m); /* generate K_mac high 8 bytes  and K_enc low 8 bytes*/
    XTEA_ENC(K_enc+2, K_enc+2, K_m); /* generate K_enc high 8 bytes */
    
    K1[0]=0; K1[1]=0;
    XTEA_ENC(K1, K1, K_mac);
    left_roll_8bytes_with_carry((uint8_t *)K1, (uint8_t *)K1, 8 );
    left_roll_8bytes_with_carry((uint8_t *)K1, (uint8_t *)K2, 8 );

#if 0
    asm (
    "   movw  R30,%A0   \n"
    "   call left_roll \n" /* generate K1 */ /***C011***/
    "   ldi r30,lo8(" STR(K2) ") \n"
    "   ldi r31,hi8(" STR(K2) ") \n"
    "   call left_roll \n" /* generate K2 */ /***C011***/
    :: "y" (K1)
    :"r26", "r27", "r30","r31" 
    );
#endif
}


static void left_roll_8bytes_with_carry ( uint8_t *src, uint8_t *dest, uint8_t len)
{
  uint8_t i;
  uint8_t data;
  uint8_t carry;
  uint8_t temp;
  data = *(src+len-1);
  carry = data & 0x80;

  for ( i = 0; i < len; i++ ) {
    data = *(src+i);
    temp = data & 0x80;
    data = ( data << 1 ) + ( carry ? 1 : 0 );
    *(dest+i) = data;
    carry = temp;
  }
} 

#if 0
/* internal function for crypto_init */
/* use loop inside - short/slow */
asm (
  
    "left_roll:               \n"
    "   ldd r26,Y+7           \n"
    "   lsl r26               \n"
    "   in r27,__SREG__       \n"   // save carry
    "   ldi r26,7             \n"   // 8 times
    "roll_loop:               \n"
    "   ld __tmp_reg__,Y      \n"
    "   out __SREG__,r27      \n"   // restore carry
    "   rol __tmp_reg__       \n"
    "   in r27,__SREG__       \n"   // save carry
    "   st Z,__tmp_reg__      \n"
    "   adiw r28,1            \n"   // Y++
    "   adiw r30,1            \n"   // Z++
    "   subi r26,1            \n"
    "   brcc roll_loop        \n"   // 8 times loop
    "   sbiw r28,8            \n"   // Y-=8
    "   ret "
);
#endif

#define XTEA_CYCLES 32
void encipher ( uint32_t dest[2], uint32_t v[2], uint32_t const k[4]) {
    unsigned int i;
    const uint32_t delta = 0x9E3779B9;
    uint32_t v0 = v[0], v1 = v[1], sum = 0;
    for (i=0; i < XTEA_CYCLES; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
    }
    dest[0] = v0; dest[1] = v1;
}

void decipher (uint32_t dest[2], uint32_t v[2], uint32_t const k[4]) {
    unsigned int i;
    const uint32_t delta = 0x9E3779B9;
    uint32_t v0 = v[0], v1 = v[1], sum = delta * XTEA_CYCLES;
    for (i=0; i < XTEA_CYCLES; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
    }
    dest[0] = v0; dest[1] = v1;
}

uint32_t cmac_calc (uint8_t* m, uint8_t bytes, uint8_t* data_prefix, uint32_t check) {
/*   reference: http://csrc.nist.gov/publications/nistpubs/800-38B/SP_800-38B.pdf
 *   1.Let Mlen = message length in bits
 *   2.Let n = Mlen / 64
 *   3.Let M1, M2, ... , Mn-1, Mn
 *    denote the unique sequence of bit strings such that
 *     M = M1 || M2 || ... || Mn-1 || Mn,
 *      where M1, M2,..., Mn-1 are complete 8 byte blocks.
 *   4.If Mn is a complete block, let Mn = K1 XOR Mn else,
 *    let Mn = K2 XOR (Mn ||10j), where j = n*64 - Mlen - 1.
 *   5.Let C0 = 0
 *   6.For i = 1 to n, let Ci = ENC_KMAC(Ci-1 XOR Mi).
 *   7.Let MAC = MSB32(Cn). (4 most significant byte)
 *   8.Add MAC to end of "m" 
 */
  
    uint32_t i,j;
    uint8_t buf[8];

	#ifdef CMACDEBUG
    	DEBUG_PRINTF("Bytes to mac:");
		COM_dump_packet(m, bytes, false);
	#endif

    if (data_prefix==NULL) {
        for (i=0;i<8;buf[i++]=0) {;}
    } else {
        memcpy(buf,data_prefix,8);
        XTEA_ENC(buf, buf, K_mac);
    } 
	#ifdef CMACDEBUG
		DEBUG_PRINTF("K_mac bytes:");
		COM_dump_packet(buf, 8, false);
	#endif

    for (i=0; i<bytes; ) { // i modification inside loop
        uint8_t x=i;
        i+=8;
        uint8_t* Kx;	// Kx is uninitialized, it is correct
        if (i>=bytes) Kx=(uint8_t*)((i==bytes)?K1:K2);
        for (j=0;j<8;j++,x++) {
            uint8_t tmp;
            if (x<bytes) tmp=m[x];
            else tmp=((x==bytes)?0x80:0);
            if (i>=bytes) tmp ^= Kx[j];
            buf[j] ^= tmp;
        }
       XTEA_ENC(buf, buf, K_mac);
    }
    if (check) {
        #ifdef CMACDEBUG
                DEBUG_PRINTF("Check:");
        #endif
        for (i=0;i<CMAC_BLOCK_SIZE;i++) {
            #ifdef CMACDEBUG
                    print_hexXX(m[bytes+i]);DEBUG_PUTC('-');print_hexXX(buf[i]); DEBUG_PUTC(' ');
            #endif
            if (m[bytes+i]!=buf[i]) return 0;
        }
        #ifdef CMACDEBUG
                DEBUG_PUTS("");
        #endif
    } else {
        memcpy(m+bytes,buf,CMAC_BLOCK_SIZE);
        #ifdef CMACDEBUG
              DEBUG_PRINTF("Add:");
              for (i=0;i<CMAC_BLOCK_SIZE;i++) {
                  print_hexXX(buf[i]); DEBUG_PUTC(' ');
              }
              DEBUG_PUTS("");
        #endif
    }
    return 1;
}

/*!
 *******************************************************************************
 *  encrypt / decrypt
 *  \note symetric operation 
 ******************************************************************************/

#include "rtc.h"

void encrypt_decrypt (uint8_t* p, uint8_t len) {
    uint8_t i=0;
    uint8_t buf[8];
    while(i<len) {
        XTEA_ENC(buf,&rtc,K_enc);
        do {
            p[i]^=buf[i&7];
            i++;
            if (i>=len) return; //done
        } while ((i&7)!=0);
    }
}

#if ( !defined(NOENCRYPT) )

    #if (DEBUG_DUMP_KEYS>0)
          void crypto_dump(void)
          {
//                  DEBUG_PRINTF("Sec.Key:"));
//                  COM_dump_packet(config.security_key,8, false);

                  DEBUG_PRINTF("K_mac:");
                  COM_dump_packet((uint8_t *)K_mac, 8, 0);

                  DEBUG_PRINTF("K_enc:");
                  COM_dump_packet((uint8_t *)K_enc, 16, 0);

                  DEBUG_PRINTF("K1:");
                  COM_dump_packet((uint8_t *)K1, 8, 0);

                  DEBUG_PRINTF("K2:");
                  COM_dump_packet((uint8_t *)K2, 8, 0);
          }
    #endif
#endif


/**
  * @}
  */
