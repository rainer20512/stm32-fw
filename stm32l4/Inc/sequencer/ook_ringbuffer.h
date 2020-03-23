

/** Size of the circular receive buffer, must be power of 2 */
#ifndef PULSE_BUFFER_SIZE
#define PULSE_BUFFER_SIZE 64
#endif

/*
** function prototypes
*/

unsigned char pulse_buf_put(uint16_t mdata, uint16_t udata);
unsigned char pulse_buf_get(uint16_t* mdata, uint16_t* udata);
unsigned char pulse_buf_has_data(void);
void 		  pulse_buf_reset(void);
