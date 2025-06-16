/*
 ******************************************************************************
 * @file    lv_4wire_spi.h 
 * @author  Rainer
 * @brief   low level functions for 4-wire-spi
 *
 ******************************************************************************
 */

#include "config/config.h"

#include "./lv_4wire_spi.h"


void LV_DRV_DELAY_MS(uint32_t wait_ms)
{
    BASTMR_DelayUs(wait_ms * 1000);
}

void LV_DRV_DISP_SPI_WR_ARRAY(uint8_t *arr, uint32_t len)
{
    while ( len-- ) {
        LV_DRV_DISP_SPI_WR_BYTE(*(arr++));
    }
}