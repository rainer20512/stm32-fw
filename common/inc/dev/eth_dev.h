/*
 ******************************************************************************
 * @file    eth_dev.h 
 * @author  Rainer
 * @brief   Very, very thin ETH device. Just Init, DeInit and FrqChange 
 *          functions are implemented. All the rest is done by LwIP
 *          see "ethernetif.c"
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ETH_DEV_H
#define __ETH_DEV_H

#include "config/config.h"

#if USE_ETH > 0 

#include "hardware.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* ETH device handle is identical to ETH_HandleTypeDef */
typedef ETH_HandleTypeDef EthHandleT; 

extern const HW_DeviceType  HW_ETH;
extern EthHandleT EthHandle;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // USE_ETH > 0 

#endif /* __ETH_DEV_H */
