/*
 ******************************************************************************
 * @file    mt25q.h
 * @author  Rainer
 * @brief   This file contains all the neccessary specific definitions for
 * Micron MT25Q Nor Flash series
 ******************************************************************************/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MX25_XXX35F_H
#define __MX25_XXX35F_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

#define MX25_XXX35F_MINIMUM_FLASH_SIZE       0x10000  /* 512 kBits => 64kByte */

#define MX25R6435F_DEVID                     0x2817   /* MX25R6435F           */
#define MX25L12835F_DEVID                    0x2018   /* MX25L12835F          */

/** 
  * @brief  MX25R6435F Flash geometry  
  */  
#define MX25R6435F_FLASH_SIZE                0x800000  /* 64 MBits => 8MBytes */
#define MX25R6435F_BLOCK_SIZE                0x10000   /* 128 blocks of 64KBytes */
#define MX25R6435F_SUBBLOCK_SIZE             0x8000    /* 256 blocks of 32KBytes */
#define MX25R6435F_SECTOR_SIZE               0x1000    /* 2048 sectors of 4kBytes */
#define MX25R6435F_PAGE_SIZE                 0x100     /* 32768 pages of 256 bytes */

#define MX25R6435F_CHIP_ERASE_MAX_TIME       240000
#define MX25R6435F_BLOCK_ERASE_MAX_TIME      3500
#define MX25R6435F_SUBBLOCK_ERASE_MAX_TIME   3000
#define MX25R6435F_SECTOR_ERASE_MAX_TIME     240

/*
 * dummy cycles for 80 MHz
 */
#define MX25R6435F_DUMMY_CYCLES_READ         8
#define MX25R6435F_DUMMY_CYCLES_READ_DUAL    4
#define MX25R6435F_DUMMY_CYCLES_READ_QUAD    4
#define MX25R6435F_DUMMY_CYCLES_2READ        4
#define MX25R6435F_DUMMY_CYCLES_4READ        6

/** 
  * @brief  MX25L12835F Flash geometry  
  */  
#define MX25L12835F_FLASH_SIZE                0x1000000 /* 128 MBits => 16MBytes */
#define MX25L12835F_BLOCK_SIZE                0x10000   /* 256 blocks of 64KBytes */
#define MX25L12835F_SUBBLOCK_SIZE             0x8000    /* 512 blocks of 32KBytes */
#define MX25L12835F_SECTOR_SIZE               0x1000    /* 4096 sectors of 4kBytes */
#define MX25L12835F_PAGE_SIZE                 0x100     /* 65536 pages of 256 bytes */

#define MX25L12835F_CHIP_ERASE_MAX_TIME       80000
#define MX25L12835F_BLOCK_ERASE_MAX_TIME      650
#define MX25L12835F_SUBBLOCK_ERASE_MAX_TIME   650
#define MX25L12835F_SECTOR_ERASE_MAX_TIME     120

/*
 * dummy cycles for 80 MHz
 */
#define MX25L12835F_DUMMY_CYCLES_READ         8
#define MX25L12835F_DUMMY_CYCLES_READ_DUAL    4
#define MX25L12835F_DUMMY_CYCLES_READ_QUAD    6
#define MX25L12835F_DUMMY_CYCLES_2READ        8
#define MX25L12835F_DUMMY_CYCLES_4READ        8


#define MX25R6435F_ALT_BYTES_PE_MODE         0xA5
#define MX25R6435F_ALT_BYTES_NO_PE_MODE      0xAA


/** 
  * @brief  MX25R6435F Commands  
  */  
/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_READ_CMD                    0x3B
#define DUAL_INOUT_READ_CMD                  0xBB
#define QUAD_OUT_READ_CMD                    0x6B
#define QUAD_INOUT_READ_CMD                  0xEB

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define QUAD_PAGE_PROG_CMD                   0x38

/* Erase Operations */
#define SECTOR_ERASE_CMD                     0x20
#define SUBBLOCK_ERASE_CMD                   0x52
#define BLOCK_ERASE_CMD                      0xD8
#define CHIP_ERASE_CMD                       0x60
#define CHIP_ERASE_CMD_2                     0xC7

#define PROG_ERASE_RESUME_CMD                0x30
#define PROG_ERASE_SUSPEND_CMD               0xB0

/* Identification Operations */
#define READ_ID_CMD                          0x9F
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define READ_FLAG_STATUS_REG_CMD             0x70
#define READ_CFG_REG_CMD                     0x15
#define WRITE_STATUS_CFG_REG_CMD             0x01

#define READ_SEC_REG_CMD                     0x2B
#define WRITE_SEC_REG_CMD                    0x2F

/* No Operation */
#define NO_OPERATION_CMD                     0x00

/* Power Down Operations */
#define DEEP_POWER_DOWN_CMD                  0xB9
#define EXIT_POWER_DOWN_CMD                  0xAB

/* Burst Operations */
#define SET_BURST_LENGTH_CMD                 0xC0

/* One-Time Programmable Operations */
#define ENTER_SECURED_OTP_CMD                0xB1
#define EXIT_SECURED_OTP_CMD                 0xC1

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99
#define RELEASE_READ_ENHANCED_CMD            0xFF

/** 
  * @brief  MX25R6435F Registers  
  */ 
/* Status Register */
#define MX25_XXX35F_SR_WIP                    ((uint8_t)0x01)    /*!< Write in progress */
#define MX25_XXX35F_SR_WEL                    ((uint8_t)0x02)    /*!< Write enable latch */
#define MX25_XXX35F_SR_BP                     ((uint8_t)0x3C)    /*!< Block protect */
#define MX25_XXX35F_SR_QE                     ((uint8_t)0x40)    /*!< Quad enable */
#define MX25_XXX35F_SR_SRWD                   ((uint8_t)0x80)    /*!< Status register write disable */

/* Flag Status Register */
#define MT25Q_FSR_PGM_DONE                    ((uint8_t)0x80)    /* Program or erase done */

/* Configuration Register 1 */
#define MX25R6435F_CR1_TB                    ((uint8_t)0x08)    /*!< Top / bottom */

/* Configuration Register 2 */
#define MX25R6435F_CR2_LH_SWITCH             ((uint8_t)0x02)    /*!< Low power / high performance switch */

/* Security Register */
#define MX25_XXX35F_SECR_SOI                  ((uint8_t)0x01)    /*!< Secured OTP indicator */
#define MX25_XXX35F_SECR_LDSO                 ((uint8_t)0x02)    /*!< Lock-down secured OTP */
#define MX25_XXX35F_SECR_PSB                  ((uint8_t)0x04)    /*!< Program suspend bit */
#define MX25_XXX35F_SECR_ESB                  ((uint8_t)0x08)    /*!< Erase suspend bit */
#define MX25_XXX35F_SECR_P_FAIL               ((uint8_t)0x20)    /*!< Program fail flag */
#define MX25_XXX35F_SECR_E_FAIL               ((uint8_t)0x40)    /*!< Erase fail flag */



#ifdef __cplusplus
}
#endif

#endif /* __MX25_XXX35F_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
