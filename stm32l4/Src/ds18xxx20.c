/******************************************************************************
 * Access Dallas 1-Wire Devices via onewire layer
 * Author: rainer
 ******************************************************************************/

#include "config/config.h"

#if USE_DS18X20 > 0

#include "dev/uart_dev.h"
#include "system/util.h"
#include "debug_helper.h"
#include "onewire.h"
#include "task/minitask.h"
#include "ds18xxx20.h"

/* private structs and typedefs ---------------------------------------------*/

/******************************************************************************
 * The scheduler consists of a state machine function to call and a stateNum
 * Within that statemachine. Everytime an async function is done, it will
 * signal the main scheduler, which will call the ow_taskhandler, which
 * will execute the next state in the selected state machine
 *****************************************************************************/
typedef void (*DS_StateMachine)(uint32_t *statenum);
typedef struct {
    DS_StateMachine          executor;                      /* Statemachine in Use                */
    uint32_t                 stateNum;                      /* actual state of Satemachine        */
    DS18X20StateEnum         exec_status;                   /* execution status and final status  */
    DS18X20_TerminationCB    onTerm;                        /* Callback on successful termination */
    DS18X20_ErrorCB          onErr;                         /* Callback on Error                  */
    uint8_t                  ds18x20_familyID;              /* DS18X20 family ID                  */
} DS_StateScheduler;

#define DS_STATEMACHINE_ACTIVE()    ( ds_scheduler.exec_status == DS18X20_WORKING )

/* private variables --------------------------------------------------------*/
static uint8_t scratchpad[DS18X20_SP_SIZE];

static DS_StateScheduler        ds_scheduler;
static uint8_t *                ds18x20_myrom   = NULL;     /* points to first DS18X20 device within all ROM IDs */
static uint8_t                  ds18x20_unique  = false;    /* true if there is only one OW device on OW bus     */
static uint8_t                  ds_iterator;                /* iterator to step thru all ROM IDs                 */
static uint8_t                  ds_iterating;               /* flag for "within iterating loop"                  */
static uint8_t                  ds18x20_bitstatus;          /* last result when reading any bitstatus            */
static int16_t                  ds18x20_decicelsius;        /* last temperature conversion result in 1/10 degC   */

uint32_t ds_ok_cnt = 0, ds_err_cnt = 0, ds_bad_cnt  = 0;

#if DEBUG_MODE > 0 
    static char * const ds_errrxt[DS18X20_NROF_ELEMENTS] =  DS18X20_ERRTXT;
#endif

/* private function prototypes ----------------------------------------------*/
static void ds_read_one_scratchpad(uint8_t *id);

/* private functions --------------------------------------------------------*/

/******************************************************************************
 * returns true, if ROM ID is one of DS18X20 sensor type
 *****************************************************************************/
static bool ds_is_18x20(uint8_t *id)
{
    return 
        *id == DS18B20_FAMILY_CODE || 
        *id == DS18S20_FAMILY_CODE ||
        *id == DS1822_FAMILY_CODE ;
}


/******************************************************************************
 * iterator function to iterate thru all found ROM IDs
 * Do not call directly, but use thwe following macros
 * FIND_FIRST(id)    and
 * FIND_NEXT(id)   
 * Both will return true, if an ID is found and will return a pointer to that id
 * in "id". False will be returned, if no (more) ROM ID is found
 *****************************************************************************/
static bool ds_find_(bool bFindFirst, uint8_t **id)
{
    if ( bFindFirst) {
        ds_iterator = 0;
        ds_iterating =1;
    } else {
        ds_iterator++;
    }
    register uint8_t *current_id;
    while ( ds_iterator < ow_nSensors ) {
        current_id = ow_SensorIDs[ds_iterator];
        if (ds_is_18x20(current_id)) {
            *id = current_id;
            return true;
        }
        ds_iterator++;
    }
    ds_iterating = 0;
    return false;
}
#define FIND_FIRST(id)  ds_find_(true, id)
#define FIND_NEXT(id)   ds_find_(false, id)

/******************************************************************************
 * find first DS18X20 Sensor in the list of OneWire devices
 * a ROM search must have been executed before
 * returns
 *   true, if exactly one OneWire Device of Type DS18X20 has been found
 *   false, if no or more than one Device of Type DS18X20 has been found
 * in 'idx' the index of first DS18X20 device will be returned
 * if  'idx' == ow_nSensors, no DS18X20 device has been found
 *****************************************************************************/
static bool ds_find_sensor( uint32_t *idx )
{
    uint32_t matches = 0;
    uint8_t *current_id;
    
    *idx = ow_nSensors;
    if ( FIND_FIRST(&current_id)) {
        do {
            *idx = ds_iterator;
             matches++;
        } while (FIND_NEXT(&current_id) );
    }

    return matches == 1;
}

/******************************************************************************
 * check initialization status of underlying OneWire device
 * return treu if initialized/operable
 *****************************************************************************/
static bool ds_is_ow_initialized(void)
{
    bool ret = ow_Initialized();
    #if DEBUG_MODE > 0 
        if (!ret ) DEBUG_PUTS("Error: Using DS18X20 functions on uninit'ed OW-Device");
    #endif

    return ret;
}


#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0
/******************************************************************************
 * Calculate the DS18X20 crc byte and return as result
 * for encoding, append the crc byte to data
 * for decoding/checking: the result (including crc byte) is 0x00 when ok
 *****************************************************************************/
static uint8_t crc8( uint8_t *data, uint16_t number_of_bytes_in_data )
{
	uint8_t  crc;
	uint16_t loop_count;
	uint8_t  bit_counter;
	uint8_t  b;
	uint8_t  feedback_bit;
	
	crc = CRC8INIT;

	for (loop_count = 0; loop_count != number_of_bytes_in_data; loop_count++)
	{
		b = data[loop_count];
		
		bit_counter = 8;
		do {
			feedback_bit = (crc ^ b) & 0x01;
			
			if ( feedback_bit == 0x01 ) {
				crc = crc ^ CRC8POLY;
			}
			crc = (crc >> 1) & 0x7F;
			if ( feedback_bit == 0x01 ) {
				crc = crc | 0x80;
			}
			
			b = b >> 1;
			bit_counter--;
			
		} while (bit_counter > 0);
	}
	
	return crc;
}
/******************************************************************************
 * convert scratchpad data to physical value in unit decicelsius
 * The scratchpad byte sequence is passed as parameter
 *****************************************************************************/
static int16_t ds_rawtemp_to_decicelsius( uint8_t familycode, uint8_t sp[] )
{
	uint16_t measure;
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if( familycode == DS18S20_FAMILY_CODE ) {   // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t)0xfffe;   // Discard LSB, needed for later extended precicion calc
		measure <<= 3;                 // Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;   // Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative 
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if ( familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE ) {
		switch( sp[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
		case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
		case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
		case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
		default:
			// 12 bit - all bits valid
			break;
		}
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) {
		decicelsius = -decicelsius;
	}

	if ( /* decicelsius == 850 || */ decicelsius < -550 || decicelsius > 1250 ) {
		return DS18X20_INVALID_DECICELSIUS;
	} else {
		return decicelsius;
	}
}

/******************************************************************************
 * prepare the oneWire transmission of a command byte ( i.e. one byte 
 * with no further parameters
 * if *id is NULL, the command is preceeded by the SKIP_ROM byte, which will
 * send the command to all OW devices
 * if *id is not null, it is interpreted as a ROM pointer and a MATCH ROM 
 * followed by 8 id bytes will preceede "command".
 * in either case, the byte sequence is encoded in "ow_iobuf"
 * true is returned on success,
 * false otherwise ( the only reason for this is a too short encoding buffer )
 *****************************************************************************/
static bool ds_prepare_command( uint8_t command, uint8_t *id )
{
    bool ret;
    ow_encoder_reset();
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 0
         DEBUG_PRINTF("Sending OW-Cmd 0x%02x to ", command); 
    #endif
    if( id ) {
            #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 0
                 ow_dump_one_rom(id);
                 putchar('\n');
            #endif
            ret = ow_encode_byte(OW_MATCH_ROM);       // to a single device
            ret &= ow_encode_vector(id, OW_ROMCODE_SIZE);
    } 
    else {
            ret = ow_encode_byte( OW_SKIP_ROM );      // to all devices
            #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 0
                 DEBUG_PUTS("ALL");
            #endif
    }
    ret &= ow_encode_byte(command);

    return ret;
}

/******************************************************************************
 * callback on expired timers ( i.e. timeout) for all async functions
 * Will set timeout status and notify scheduler
 *****************************************************************************/
static void ds_timer_cb ( void )
{
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 1
         DEBUG_PRINTTS("Onewire Async Timout\n"); 
    #endif
    ds_scheduler.exec_status = DS18X20_TIMEOUT;
    TaskNotify(TASK_OW);
}

/******************************************************************************
 * callback on normal termination of async function executions
 * This callback will be made within interrupt context 
 * Can also be called manually to force execution of next SM state
 * parameters are unsed
 *****************************************************************************/
static void ds_trigger_sm(void)
{
    TaskNotify(TASK_OW);
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 2
        DEBUG_PRINTTS("TaskBit\n");
    #endif
}

/******************************************************************************
 * Init the state machine scheduler to passed state machine function and
 * to state 0
 *****************************************************************************/
static void ds_scheduler_init( DS_StateMachine executor, DS18X20StateEnum initial_state, bool bExecFirstState, DS18X20_TerminationCB tCB, DS18X20_ErrorCB eCB )
{
    if ( DS_STATEMACHINE_ACTIVE() ) {
        #if DEBUG_MODE > 0
            puts("Cannot init DS scheduler, statemachine active");
        #endif
        return;
    }
    ds_scheduler.executor    = executor;
    ds_scheduler.stateNum    = 0;
    ds_scheduler.exec_status = initial_state;
    ds_scheduler.onTerm      = tCB;
    ds_scheduler.onErr       = eCB;
    if ( bExecFirstState ) ds_trigger_sm();
}

/******************************************************************************
 * Set the lower layer OneWire callbacks to progress to next state and
 * to indicate an error resp,
 * thereafter prepare the rom command in ow_iobuf
 * and start the statmachine
 * Parameter 'id' contains the 8 byte sensor ID, can be NULL.
 * if NULL, all sensors are adressed by issueing a 'skip rom' command
 *****************************************************************************/
static void ds_command( DS_StateMachine sm, uint8_t command, uint8_t *id, DS18X20_TerminationCB tCB, DS18X20_ErrorCB eCB )
{   
    bool ret;
    
    ow_set_callbacks(ds_trigger_sm, ds_timer_cb);
    ret = ds_prepare_command(command, id);
    ds_scheduler_init(sm, (ret ? DS18X20_WORKING : DS18X20_START_FAIL), true, tCB, eCB ); 
}

/******************************************************************************
 * Error callback, currently used by all statemachines
 *****************************************************************************/
static void ds_error_cb ( DS18X20StateEnum ds_err, uint32_t state )
{
    #if DEBUG_MODE > 0
        printf("DS18X20 error %d in state %d: %s\n", ds_err, state, ds_errrxt[ds_err]);
    #else
        printf("DS18X20 error %d in state %d\n", ds_err, state);
    #endif
}

/*-----------------------------------------------------------------------------
 - All different state machines and callbacks
 ----------------------------------------------------------------------------*/

/******************************************************************************
 * State machine and callback to send a CMD byte
 * The execution status ( one of DS18X20StateEnum ) is stored and returned
 * in "ds_scheduler.exec_status"
 *****************************************************************************/
 static void ds_SM_Command ( uint32_t *state )
{
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 1
        DEBUG_PRINTTS("ds_SM_Command in State %d\n", *state);
    #endif
    switch ( (*state)++ ) {
        case 0:
            ow_reset_with_cb(true);
            break;
        case 1:
            if ( !ow_reset_successful() ) {
                /* no presence pulse received -> no oneWire device */
                ds_scheduler.exec_status = DS18X20_NO_DEVICE;
            }
            /* Execute next state */
            ds_trigger_sm();
            break;
        case 2:
            ow_write_buffer_with_cb(true);
            break;
        case 3:
            ds_scheduler.exec_status = DS18X20_OK;
            /* Execute next state */
            ds_trigger_sm();
            break;
        default:
            ds_scheduler.exec_status = DS18X20_SM_ERROR;
            /* Execute next state */
            ds_trigger_sm();
    }
}
/*---------------------------------------------------------------------------*/
static void ds_command_termination_cb ( void )
{
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 1
        DEBUG_PUTS("DS18X20 Statemachine terminated ok");
    #endif
}


/******************************************************************************
 * State machine and callback to read the scratchpad
 * Whenever the scratchpad is read and checked successful,
 * the temperature is converted to decicelsius and stored in "ds18x20_decicelsius"
 * The execution status ( one of DS18X20StateEnum ) is stored and returned
 * in "ds_scheduler.exec_status"
 *****************************************************************************/
 static void ds_SM_ReadSP ( uint32_t *state )
{
    int16_t temptemp;
    #if DEBUG_MODE > 0  && DEBUG_DS18X20 > 1
        DEBUG_PRINTTS("ReadSP#%d\n", *state);
    #endif
    switch ( (*state)++ ) {
        case 0:
            ow_reset_with_cb(true);
            break;
        case 1:
            if ( !ow_reset_successful() ) {
                /* no presence pulse received -> no oneWire device */
                ds_scheduler.exec_status = DS18X20_NO_DEVICE;
            }
            /* Execute next state */
            ds_trigger_sm();
            break;
        case 2:
            ow_write_buffer_with_cb(true);
            break;
        case 3:
            /* preset iobuf with HIGH_PATTERN to read scratchpad */
            ow_encoder_preset(DS18X20_SP_SIZE);
            /* Execute next state */
            ds_trigger_sm();
            break;
        case 4:
            /* Read buffer */
            ow_write_buffer_with_cb(true);
            break;
        case 5:
            /* Read buffer */
             ow_decode_vector(scratchpad);
            if ( crc8( scratchpad, DS18X20_SP_SIZE ) ) {
                    ds_scheduler.exec_status = DS18X20_ERROR_CRC;
            } else {
                    temptemp = ds_rawtemp_to_decicelsius( ds_scheduler.ds18x20_familyID, scratchpad );
                    /* check for wrong readout due to low voltage */
                    if ( temptemp >1200 ) {
                       ds_scheduler.exec_status = DS18X20_BADVALUE;
                    } else {
                        ds18x20_decicelsius = temptemp;    
                        ds_scheduler.exec_status = DS18X20_OK;
                    }
            }
            /* Execute next state */
            ds_trigger_sm();
            break;
        default:
            ds_scheduler.exec_status = DS18X20_SM_ERROR;
            /* Execute next state */
            ds_trigger_sm();
    }
}
/*---------------------------------------------------------------------------*/
static void ds_scratchpad_cb ( void )
{
    #if DEBUG_MODE > 0 
        #if DEBUG_DS18X20 > 1
            DEBUG_PUTS("Read scratchpad terminated ok");
            for ( uint32_t i=0; i < DS18X20_SP_SIZE; i++ ) {
                print_hexXX(scratchpad[i]);putchar(' ');
            }
            puts("");
        #endif
        #if DEBUG_DS18X20 > 0
            DEBUG_PRINTF("Temp=%d.%d\n",ds18x20_decicelsius/10, ds18x20_decicelsius%10 );
        #endif
        SetTemp(ds18x20_decicelsius * 10);
    #endif
    if ( ds_iterating ) {
        uint8_t *id;
        if ( FIND_NEXT(&id) ) ds_read_one_scratchpad(id);
    }
}


/******************************************************************************
 * State machine and callback to read the status bit
 * only valid for commands, that will have an status bit ( eg CONVERT T )
 * Upon termination, the status bit can be read in "ds18x20_bitstatus"
 * The execution status ( one of DS18X20StateEnum ) is stored and returned
 * in "ds_scheduler.exec_status"
 *****************************************************************************/
 static void ds_SM_ReadBit( uint32_t *state )
{
    #if DEBUG_MODE > 0 && DEBUG_DS18X20 > 1
        DEBUG_PRINTF("ds_SM_ReadBit in State %d\n", *state);
    #endif
    switch ( (*state)++ ) {
        case 0:
            ow_read_bit_with_cb(true);
            break;
        case 1:
            ds18x20_bitstatus = ow_get_bitval();
            ds_scheduler.exec_status = DS18X20_OK;
            ds_trigger_sm();
            break;
        default:
            ds_scheduler.exec_status = DS18X20_SM_ERROR;
            /* Execute next state */
            ds_trigger_sm();
    }
}
/*---------------------------------------------------------------------------*/
static void ds_bitstatus_cb ( void )
{
    DEBUG_PRINTF("DS18X20 bitstatus returned %s\n", ds18x20_bitstatus ? "true" : "false");
}


/*-----------------------------------------------------------------------------
 -- public functions ----------------------------------------------------------
 ----------------------------------------------------------------------------*/

/******************************************************************************
 * Trigger a temperature measurement
 *****************************************************************************/
 void DS18X20_start_meas(bool bTriggerAll)
{
    if( !ds_is_ow_initialized() ) return;

    uint8_t *id;

    if ( ds18x20_unique || bTriggerAll ) 
        id = NULL;
    else 
        id = ds18x20_myrom;

    ds_command( ds_SM_Command, DS18X20_CONVERT_T, id, ds_command_termination_cb, ds_error_cb );
}

/******************************************************************************
 * Trigger a scratchpad readout for one ROM ID
 *****************************************************************************/
static void ds_read_one_scratchpad(uint8_t *id)
{
    /* Remember familyID, we need it later when evaluating temperature */
    ds_scheduler.ds18x20_familyID = ( id ? *id : *ds18x20_myrom );
    ds_command( ds_SM_ReadSP, DS18X20_READ, id, ds_scratchpad_cb, ds_error_cb );
}

/******************************************************************************
 * Trigger a scratchpad readout for one or all ROM ID(s)
 *****************************************************************************/
void DS18X20_read_scratchpad(bool bReadAll)
{
    if( !ds_is_ow_initialized() ) return;

    uint8_t *id;

    if ( ds18x20_unique ) {
        id = NULL;
    } else if ( bReadAll ) {
        if (!FIND_FIRST(&id) ) return;
    } else {
        id = ds18x20_myrom;
    }
    ds_read_one_scratchpad(id);  
}


/******************************************************************************
 * Read the status of a previously sent command, that will have an 
 * bitstatus, i.e. Convert T
 *****************************************************************************/
void DS18X20_read_bitstatus(void)
{
    if( !ds_is_ow_initialized() ) return;

    ow_set_callbacks(ds_trigger_sm, ds_timer_cb);
    ds_scheduler_init(ds_SM_ReadBit, DS18X20_WORKING, true, ds_bitstatus_cb, ds_error_cb ); 
}


/******************************************************************************
 * find an DS18X20 device and store its ROM address in ds18x20_myrom
 * if nothing is found, ds18x20_myrom will be set to NULL
 *****************************************************************************/
bool DS18X20_Init(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    uint32_t idx;
#pragma GCC diagnostic pop
    if (ds_find_sensor(&idx) ) {
        ds18x20_myrom = ow_SensorIDs[idx];
        ds18x20_unique = true;
    } else {
        if ( idx == ow_nSensors ) {
            /* no sensor found */
            ds18x20_myrom = NULL;
            #if DEBUG_MODE > 0 
                  DEBUG_PUTS("No DS18X20 device found");
            #endif
        } else {
            /* more than one DS18X20 found */
            ds18x20_myrom = ow_SensorIDs[idx];
            #if DEBUG_MODE > 0 
                  DEBUG_PRINTF("Found %d DS18X20 devices\n", ow_nSensors);
            #endif
        }
        ds18x20_unique = false;
    }

    /* Init scheduler status, no more else to initialize */
    ds_scheduler.exec_status = DS18X20_OK;

    return ds18x20_unique;
}

/******************************************************************************
 * Return true, if at least one DS18x20 sensor is fond on OneWire line
 *****************************************************************************/
bool DS18X20_Found(void) 
{
    return ds18x20_myrom != NULL;
}


/******************************************************************************
 * Return the last temperature measurement result
 * value is in decicelsius
 *****************************************************************************/
int16_t DS18X20_GetTemp(void)
{
    return ds18x20_decicelsius;
}

/******************************************************************************
 * format decicelsius-value into string with decimal-point
 * buffer of sufficient size has to be passed by caller, should be at least of
 * size 7.
 * Will not convert temperatures ouf of the range [-100 .. +100 ]
 *****************************************************************************/
bool DS18X20_GetTempStr( int16_t decicelsius, char str[], size_t n)
{
	bool sign = 0;
	char temp[7];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	uint16_t  work;
	uint16_t  work10;

	// range from -550:-55.0°C to 1250:+125.0°C -> min. 6+1 chars
	if ( n < (6+1) && ( decicelsius <= -10000 || decicelsius >= 10000)  ) return false;

        if ( (sign = decicelsius < 0) ) {
                work = -decicelsius;
        } else {
                work = decicelsius;
        }

        // construct a backward string of the number.
        do {
                work10 = work/10;
                temp[temp_loc++] = work - work10*10 + '0';
                work = work10;
        } while ( work > 0 );

        if ( sign ) {
                temp[temp_loc] = '-';
        } else {
                ///temp_loc--;
                temp[temp_loc] = '+';
        }

        // reverse the string.into the output
        while ( temp_loc >=0 ) {
                str[str_loc++] = temp[(uint8_t)temp_loc--];
                if ( temp_loc == 0 ) {
                        str[str_loc++] = DS18X20_DECIMAL_CHAR;
                }
        }
        str[str_loc] = '\0';

        return true;
}

/******************************************************************************
 * Callback from main task scheduler: Execute selected state in selected 
 * state machine
 *****************************************************************************/
void task_handle_ds(uint32_t arg)
{
    UNUSED(arg);

    if( !ds_is_ow_initialized() ) return;

    #if DEBUG_MODE > 0  && DEBUG_DS18X20 > 1
        DEBUG_PRINTTS("ds task\n");
    #endif
    switch ( ds_scheduler.exec_status ) {
        case DS18X20_OK:
            ds_ok_cnt++;
            /* State machine termionated successfully */
            if ( ds_scheduler.onTerm ) ds_scheduler.onTerm();
            break;
        case DS18X20_WORKING:
            /* State machine still working, execute next state */
            assert(ds_scheduler.executor);
            ds_scheduler.executor(&ds_scheduler.stateNum);
            break;
        default:
            /* Everything else is an error state */
            if ( ds_scheduler.exec_status == DS18X20_BADVALUE )
                ds_bad_cnt++;
            else 
                ds_err_cnt++;
            if ( ds_scheduler.onErr ) ds_scheduler.onErr( ds_scheduler.exec_status, ds_scheduler.stateNum-1 );
    }
}

#include "system/periodic.h"
#include "rtc.h"
#include "eeprom.h"
#include "ui/lcd_status.h"

/******************************************************************************
 * Every second callback for temp measuerment:
 * check, whether the configured measuerment interval has expired and start a
 * new measurement, if so. In the next second do the readout and set time
 * so start next measurement 
 *****************************************************************************/
static uint32_t tempMeasureNext=0;
void CheckForTemp(void *arg)
{
    UNUSED(arg);
    uint32_t secs = RTC_GetSecond();

    if       ( secs == tempMeasureNext ) {
        if ( DS18X20_Found() ) DS18X20_start_meas(true);
    } else if  ( secs == (tempMeasureNext+1)%60 ) {
        if ( DS18X20_Found() ) {
            DS18X20_read_scratchpad(true);
        }
    } else if  ( secs == (tempMeasureNext+2)%60 ) {
        /* schedule next remp measurement */
        tempMeasureNext = (tempMeasureNext + config.temp_interval)%60;
        /* Display temp */
        #if USE_DISPLAY > 0 
            LCD_DisplayStatus(LCD_STATUS_TEMP);
        #endif
    }
}


/******************************************************************************
 * Callback from main task scheduler: Execute selected state in selected 
 * state machine
 *****************************************************************************/
void task_init_ds ( void )
{
    EverySecond(CheckForTemp, (void *)0, "Check for Temp measure");
}

#endif // #if USE_DS18X20 > 0
