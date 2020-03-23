/*
 ******************************************************************************
 * @file    util.h
 * @author  Rainer
 * @brief   common utility functions
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTIL_H
#define __UTIL_H

int16_t average_n ( int16_t avg, int32_t value, uint8_t n  );

// gleitende Mittelung mit exponentieller Relaxation mit n=3..5:
#define average3( pAvg, value )     average_n(pAvg, value, 3)
#define average4( pAvg, value )     average_n(pAvg, value, 4)
#define average5( pAvg, value )     average_n(pAvg, value, 5)

extern int16_t  mintemp, maxtemp;                               // Minimum and maximum temperature ( *100 ) of the day
extern int16_t  abstemp;                                        // temp*100
void ResetMinMaxTemp ( void *arg );
void UpdateMinMaxTemp(int16_t current);                         // Temp value  = °C * 100
void SetTemp(int16_t current);                                  // Temp value  = °C * 100

#endif /* #ifndef __UTIL_H */


