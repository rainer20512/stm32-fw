int32_t LogFile_Init      (void);
uint8_t LogFile_IsOpen    (void);
int32_t LogFile_Flush     (void);
int32_t LogFile_Close     (void);
uint8_t LogFile_Write     (const char *str, uint32_t len);
uint8_t LogFile_Write_CRLF(const char *str, uint32_t len);
void    LogFile_CRLF      ();

void    task_handle_log   (uint32_t arg);