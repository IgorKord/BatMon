
#ifndef _ASCII_COMMANDS
#define _ASCII_COMMANDS

#include"GLOBALS.H"
// Custom printf/sprintf for flash strings
void Print_F(char FL* fmt, ...);
int SPRINT_F(char* OutStr, char FL* fmt, ...);

// UART functions
void Send_Char(uint8 Tx_char);
uint16 PutStr(const char* Str);
uint16 PrintConstString(const char FL* str_f_ptr);
uint16 CopyConstString(const char FL* str_f_ptr, char* dest);

// Standard wrappers (if needed)
#ifndef PC
#include <iom644.h>
#define printf Print_F
#define sprintf SPRINT_F
#define cputs PrintConstString
#endif // ifndef PC
/*************************************************************/
extern char FW_PartNumber[];
extern char FW_Date[];
extern uint8  EchoStatus;                         // true - send echo back to COM port, false (default) - no echo
extern Uint32 ErrorStatus;						// 0 - no error, other values - error code
extern SettingsStruct Existing;					// keep current setting to detect change vs SysData.NV_UI
extern uint8  restart_op;							// used to restart system using WDT
extern char printf_buff[0x40];						// [64] to copy flash strings into RAM
extern const char FL* ProtocolNames[];
extern uint8 Num_RCI_commands; // = sizeof(rci) / sizeof(t_rci_commands);
extern uint8 const UnitTypes[4];

/*************************************************************/
extern void Get_EEPROM_params(void);		// "rsto"  restore: GET SYSTEM PARAMETERS FROM FLASH if data is valid
extern void ClearRxBuffer(uint8 pattern);
extern void SetDefaultsInRAM(void);
extern void EEPROM_Write_Mem_Block(uint8* SourceAdr, uint16 EEaddress, uint16 block_size);
extern void Write_LEDs_OnDisplayBoard(uint8 LED_Buzz_bits);
extern Uint32 Convert_4_ASCII_to_Uint32(char* pstr);
extern void SendMsgToPC(char FL* Msg);
// sending upper byte first as in command from PC
extern void Put_CMD_as_chars(void);
extern void Print_FW_Version(void);
extern int NumOfCommands(void);

enum ECHO_STATE {
	ECHO_DISABLED = 0, // convinient to check: if(EchoStatus) send echo - will not send if EchoStatus=ECHO_DISABLED
	ECHO_ENABLED = 1,
	ECHO_VERBOSE = 2,
};


#ifdef PC
#ifndef UCSR0C // = 0x06;                              //no UART_parity, async, 1 stop,
extern long UCSR0C;
#endif
#endif // PC

#endif // _ASCII_COMMANDS
