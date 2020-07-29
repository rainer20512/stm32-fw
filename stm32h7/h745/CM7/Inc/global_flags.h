#include <stdint.h>

extern uint32_t   gflags;
extern uint32_t   errflags;

#define GFLAG_DISPOFF_BIT 		0	// Set, if display is switched off in order to reduce power consumption
						// from PowerSave, See Manual 17.9 Bullet 8
#define GFLAG_TIMECORR_BIT              1       // Time correction of LPTIM1 is active */

#define GFLAG_OOK_BIT 	     		3	// Set if RFM is running in OOK-Mode
#define GFLAG_FORCE_OOK_BIT		4       // Set if OOK is forced on
#define GFLAG_FSK_BIT	       		5	// Set if RFM is running in FSK-Mode ( either send or receive )
#define GFLAG_CALIBRATING_BIT		6	// Set if Timer2 is used for OSCCAL-Adjusting

#define SetFlagBit(flag,bit)            (flag |= (1<<bit))
#define ClearFlagBit(flag,bit)		(flag &= ~(1<<bit))
#define IsFlagBitSet(flag,bit)          (flag & (1<<bit))

#define OOK_Receiver_ForceMode()        (IsFlagBitSet(gflags, GFLAG_FORCE_OOK_BIT))
#define OOK_Receiver_running()		(IsFlagBitSet(gflags, GFLAG_OOK_BIT))
#define FSK_Transceiver_running()	(IsFlagBitSet(gflags, GFLAG_FSK_BIT))
#define Calibrator_running()		(IsFlagBitSet(gflags, GFLAG_CALIBRATING_BIT))

#define RFM_is_idle()			( (gflags & ( (1<<GFLAG_FSK_BIT) | (1<<GFLAG_OOK_BIT) ) ) == 0 )

#if defined(TX18LISTENER)
	extern uint32_t  aflags;

	#define AFLAG_GOT_TEMP_BIT	    	0	// Set if temperature was received
	#define AFLAG_GOT_RH_BIT       		1	// Set if RH was received
	#define AFLAG_GOT_VALID_DATA		2       // Set if we received any valid data packet and next transmission time has been adjusted
	#define AFLAG_DATA_MASK			( (1<<AFLAG_GOT_TEMP_BIT)|(1 <<AFLAG_GOT_RH_BIT) )
        #define AFLAG_OOK_RETRY                 5

        #define OOK_Receiver_retrying()		(IsFlagBitSet(aflags, AFLAG_OOK_RETRY))
#endif