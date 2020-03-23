
#define MAX_SEQLEN			60	// Max length of a sequence (first element is length byte, so max seq. length is one byte less )
#define MAX_SEQNUM                      6	// Number of sequences that can be stored simultaneously.

extern uint8_t sequences[MAX_SEQNUM][MAX_SEQLEN];
extern uint8_t s_temp[MAX_SEQLEN];
extern uint8_t s_actual;

extern bool bFirstPulse;
extern bool bRisingEdge;

void PulsesInit(void);
void task_handle_pulse(uint32_t arg );

uint8_t ActualSequence();
void CopySequence();
void Store ( char c); 
unsigned char InterpretPulse( void );

void HandleOOKInterrupt(uint16_t pin, uint16_t pinvalue, void *arg);
