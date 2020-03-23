
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EPD_FONTS_H
#define __EPD_FONTS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

extern sFONT Font20;
extern sFONT Font16;
extern sFONT Font12;
extern sFONT Font8;

#define LINE(x) ((x) * (((sFONT *)EPD_GetFont())->Height))


#ifdef __cplusplus
}
#endif
  
#endif /* __EPD_FONTS_H */
 
