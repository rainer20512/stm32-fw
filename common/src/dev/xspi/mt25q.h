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

/* 
 * Driver supported read and program transfer modes 
 * (Note, that Read and DUAL READ are not implemented yet )
 */
typedef enum
{
  MT25Q_RW_STR_QUAD   = 0,                    /*!< 1-1-4 STR Read mode */
  MT25Q_RW_STR_QUADIO = 1,                    /*!< 1-4-4 STR Read mode */
  MT25Q_RW_DTR_QUAD   = 2,                    /*!< 1-1-4 DTR Read mode */
  MT25Q_RW_DTR_QUADIO = 3,                    /*!< 1-4-4 DTR Read mode */
} MT25Q_RWModeT;

/******************MT25TL01G_Transfer_t**********************/


/** 
  * @brief  MT25QYXXX geometry data
  */  
#define MT25QY064_FLASH_SIZE                 0x800000  /* smallest device has 64 MBits => 8MBytes */
#define MT25QYXXX_BLOCK_SIZE                 0x10000   /* blocks of 64KBytes size*/
#define MT25QYXXX_SUBBLOCK_SIZE              0x8000    /* subblocks of 32KBytes size*/
#define MT25QYXXX_SECTOR_SIZE                0x1000    /* sectors of 4kBytes size */
#define MT25QYXXX_PAGE_SIZE                  0x100     /* pages of 256 bytes size */

/* Maximum erase times in ms */
#define MT25QY512_CHIP_ERASE_MAX_TIME        460000
#define MT25QY512_BLOCK_ERASE_MAX_TIME       1000
#define MT25QY512_SUBBLOCK_ERASE_MAX_TIME    1000
#define MT25QY512_SECTOR_ERASE_MAX_TIME      400
#define MT25QY512_PAGE_PGM_MAX_TIME          2


/*
 * Dummy cycles table
 * Organized as follows: When operating frq [MHz] at or below n-th element, the minimum number of dummy cycles
 * has to be n. Whenever operating frq is higher than last element, the operating frq is too high for this mode
 */
#define MT25Q_DUMMY_CYCLES_STR_QUAD         { 44, 61, 78, 97, 106, 115, 125, 133, }              /* for 1-1-4 STR */
#define MT25Q_DUMMY_CYCLES_STR_QUADIO       { 39, 48, 58, 69, 78, 86, 97, 106, 115, 125, 133, }  /* for 1-4-4 STR */
#define MT25Q_DUMMY_CYCLES_DTR_QUAD         { 26, 40, 59, 65, 75, 83, 90, }                      /* for 1-1-4 DTR */
#define MT25Q_DUMMY_CYCLES_DTR_QUADIO       { 20, 30, 39, 49, 58, 68, 78, 85, 90, }               /* for 1-4-4 DTR */



/** 
  * @brief  MT25Q Commands  
  */  

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_DTR_CMD                    0x0D
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_DTR_CMD           0x3D
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_DTR_CMD           0x6D
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC
#define QUAD_INOUT_FAST_READ_4_BYTE_DTR_CMD  0xEE

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

#define READ_EXT_ADDR_REG_CMD                0xC8
#define WRITE_EXT_ADDR_REG_CMD               0xC5

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2

#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x38

#define QUAD_IN_FAST_PROG_4BYTE_ADDR_CMD     0x34
#define EXT_QUAD_IN_FAST_PROG_4BYTE_ADDR_CMD 0x3E

/* Erase Operations */
#define SECTOR_4K_ERASE_CMD                  0x20
#define SECTOR_4K_ERASE_4_BYTE_ADDR_CMD      0x21

/* Erase Operations */
#define SUBBLOCK_32K_ERASE_CMD               0x52
#define SUBBLOCK_32K_ERASE_4_BYTE_ADDR_CMD   0x5C

#define BLOCK_ERASE_CMD                      0xD8
#define BLOCK_ERASE_4_BYTE_ADDR_CMD          0xDC

#define BULK_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* Quad Operations */
#define ENTER_QUAD_CMD                       0x35
#define EXIT_QUAD_CMD                        0xF5

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              8
#define DUMMY_CLOCK_CYCLES_READ_QUAD         10

#define DUMMY_CLOCK_CYCLES_READ_DTR          6
#define DUMMY_CLOCK_CYCLES_READ_QUAD_DTR     8


/* Power Down Operations */
#define DEEP_POWER_DOWN_CMD                  0xB9
#define EXIT_POWER_DOWN_CMD                  0xAB

#define DS_RECOVERY_TIME_US                  30            /*!< min recovery time from deep sleep */
/** 
  * @brief  MX25R6435F Registers  
  */ 
/* Status Register */
#define MT25Q_SR_WIP                    ((uint8_t)0x01)    /*!< Write in progress */
#define MT25Q_SR_WEL                    ((uint8_t)0x02)    /*!< Write enable latch */
#define MT25Q_SR_BP                     ((uint8_t)0x5C)    /*!< Block protect */
#define MT25Q_SR_BPDIR                  ((uint8_t)0x20)    /*!< Block protection direction: 0 = from top, 1 = from bottom */
#define MT25Q_SR_SRWD                   ((uint8_t)0x80)    /*!< Status register write disable */
#define MT25Q_GET_BP_NORMALIZED(a)      (((a&0x1c)>>2)|((a&0x40) >> 3))

/* Flag Status Register */
#define MT25Q_FSR_PGM_DONE                    ((uint8_t)0x80)    /* Status: Program or erase done */
#define MT25Q_FSR_ERA_SUSP                    ((uint8_t)0x40)    /* Status: Erase suspended */
#define MT25Q_FSR_ERA_ERR                     ((uint8_t)0x20)    /* Error: Erase failed */
#define MT25Q_FSR_PGM_ERR                     ((uint8_t)0x10)    /* Error: Program failed */
#define MT25Q_FSR_PGM_SUSP                    ((uint8_t)0x04)    /* Status: Program suspended */
#define MT25Q_FSR_ACC_ERR                     ((uint8_t)0x02)    /* Error: Erase or PGM access to protected block */
#define MT25Q_FSR_4BYTE_MODE                  ((uint8_t)0x01)    /* Status: Set when 4-Byte addressing is selected */

/* Volatile Configuration Register bits */
#define MT25Q_VCR_DUMMY_CYCLES_Pos            4                  /* at four upper bit positions */
#define MT25Q_VCR_DUMMY_CYCLES_Msk            0x0f               /* 4 bits */
#define MT25Q_VCR_DUMMY_CYCLES                ((uint8_t)(MT25Q_VCR_DUMMY_CYCLES_Msk<<MT25Q_VCR_DUMMY_CYCLES_Pos))
#define MT25Q_VCR_XIP_DISABLED                ((uint8_t)0x08)    /* XIP mode disabled */
#define MT25Q_VCR_WRAPMODE                    ((uint8_t)0x03)    /* address wrap mode, 3 bits */
#define MT25Q_GET_DUMMYCYCLES_NORMALIZED(a)   ((a & MT25Q_VCR_DUMMY_CYCLES) >> 4)
#define MT25Q_GET_WRAPMODE_NORMALIZED(a)      (a & MT25Q_VCR_WRAPMODE)

/* Enhanced Volatile Configuration Register bits */
#define MT25Q_EVCR_QUAD_DISABLED              ((uint8_t)0x80)    /* QUAD mode disabled */
#define MT25Q_EVCR_DUAL_DISABLED              ((uint8_t)0x40)    /* DUAL mode disabled */
#define MT25Q_EVCR_DTR_DISABLED               ((uint8_t)0x20)    /* DTR mode disbaled */
#define MT25Q_EVCR_DRV_STRENGTH               ((uint8_t)0x07)   /* 3 bits for output driver strength */
#define MT25Q_GET_DRV_STRENGTH_NORMALIZED(a)  (a & MT25Q_EVCR_DRV_STRENGTH)

#ifdef __cplusplus
}
#endif

#endif /* __MX25_XXX35F_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
