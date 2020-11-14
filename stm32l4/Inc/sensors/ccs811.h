
#ifndef CCS811_H_
#define CCS811_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* BME280 communication interface for higher levels */
#include "sensors/thp_sensor.h"

extern const THPSENSOR_DrvTypeDef CCS811_Driver;

/* Header includes */
#include "sensors/ccs811_defs.h"


#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* CCS8111_H_ */
/** @}*/
