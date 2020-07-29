#include <stdint.h>

extern uint32_t   gflags;
extern uint32_t   errflags;

#define GFLAG_DISPOFF_BIT 		0	// Set, if display is switched off in order to reduce power consumption
						// from PowerSave, See Manual 17.9 Bullet 8
#define GFLAG_TIMECORR_BIT              1       // Time correction of RTCTIMER is active */


#define SetFlagBit(flag,bit)            (flag |= (1<<bit))
#define ClearFlagBit(flag,bit)		(flag &= ~(1<<bit))
#define IsFlagBitSet(flag,bit)          (flag & (1<<bit))

