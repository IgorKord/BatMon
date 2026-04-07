#include "ASCII_commands.h"
#include "GLOBALS.h"
#ifndef PC
// Ensure prototypes are visible
extern void Print_F(char __flash* fmt, ...);
extern int SPRINT_F(char* OutStr, char __flash* fmt, ...);
extern void Send_Char(uint8 Tx_char);
#endif
#ifndef PC
  //#include <iom644.h>
  //#include <ina90.h>
  #include <ctype.h>
  //#include <pgmspace.h>
  #include <string.h>
  //#include <stdio.h>
#endif // ifndef PC

#ifdef PC // for PC,
  //#include <cstdio>
#define INTERRUPT
#include <corecrt_math.h> // for isnan() lib function
typedef int                 BOOL;
#define PutChar PUTCHAR
#define cputs    CPUTS
//Int16 CPUTS(const char* p);
#define CPUTS PutStr
#endif


uint8 line_cntr;					// used in export command to count parameter lines
uint8 Num_RCI_commands; // = sizeof(rci) / sizeof(t_rci_commands);
char CMD_index;
char* CommStr;

uint8  EchoStatus;                         // true - send echo back to COM port, false (default) - no echo
char RCI_message[128];						// for output via UART from RCI

char FL* p_Execution = ">~Execution";
char FL* p_Unrecognized = ">~Unrecognized";	//command was not recognized

void PrintNewLine(void) {
	cputs("\r\n");
}

// *************************************************************
// "Vers"[?]- Get firmware Version
// DO NOT USE COMMAS in name, PC application Battery Monitor Setup looks for them as separator for data.
// *************************************************************/
void Print_FW_Version(void)
{
	//  float tf = ((float)(FW_VERSION) + 0.01f)/10.0f;
#ifdef INCLUDE_DAY_TIME_INTO_FW // this includes "comp @ __DATE__ __TIME__" but in Vsual Studio just-in-time compile will not work
	printf("Battery Monitor SW %s Ver %3.1f, comp %s %s", FW_PartNumber, FirmwareVersion, FW_Date, CompileDate, CompileTime);
#else
	printf("Battery Monitor SW %s Ver %3.1f @ %s", FW_PartNumber, FirmwareVersion, FW_Date);
#endif
}

// if verbose is enabled add verbose comment , then \r\n
void Print_w_verbose(char FL* vrb)
{
	if (EchoStatus == ECHO_VERBOSE)
	{
		//	cputs(" // ");
		cputs(vrb);
	}
	PrintNewLine();
}

void PrintDelayTime(void)
{
	static FL char ms_delay_before_measurement[] = "ms delay before measurement";
	printf("\x1BRW%d\r", SysData.xmt_delay);
	Print_w_verbose((char FL*)ms_delay_before_measurement); // ms
}

/*************************************************************/
// this function is called only when (rt.Host & CharAvailableFlag !=0).
// when it is called, rt.HostRxBuffPtr already incremented
// Enter '\r' should move cursor to the left, and send '\n'to set new line for response
// the '?' in "get" command is still printed but in rt.HostRxBuff it needs to be skipped or response is "ofs1?=0"
void ProcessChar(void) {
#ifndef __cplusplus
	register
#endif
		uint16 uPtr;
#ifndef __cplusplus
	register
#endif
		char tmp_char;
	//IK191119 done in FIQ	if (rt.HostRxBuffPtr >= HOST_BUFF_LEN-3) rt.HostRxBuffPtr = 1; //prevent run-out when PC sending a lot of chars
	// the last received char is located in rt.HostRxBuff[rt.HostRxBuffPtr-1]
	// after command processed, pointers are zeroed: rt.EchoRxBuffPtr = rt.HostRxBuffPtr = 0;
	tmp_char = rt.HostRxBuff[0];
	// if only one char entered and next is backspace, TMC terminal might send extra backspace which gets in first place
	if ((tmp_char == '\b') || (tmp_char == 0x7F)) // if first byte is backspace
	{
		rt.HostRxBuff[0] = 0;											//  clear 1st byte and pointers
		rt.HostRxBuffPtr = 0;
		rt.EchoRxBuffPtr = 0;
	}
	while (uPtr = rt.HostRxBuffPtr, rt.EchoRxBuffPtr != uPtr)
	{
		tmp_char = rt.HostRxBuff[rt.EchoRxBuffPtr];
		// discovery of CR for 7126
		if (tmp_char == 0)  // end of string (or CR got replaced with 0)
		{
			if (rt.Host & CharEchoFlag) SendCrLf();						// send back "\r\n"
			goto exit_ProcessChar;										// command discovered - do not bother showing possible chars after '\r'
		}

		// IK191119 Backspace happen only during manual entering, char-by-char; PC sends correct commands as strings
		// PuTTY sends 0x7F instead of 0x08.
		// because of that, rt.EchoRxBuffPtr is only one index behing rt.HostRxBuffPtr
		else if ((tmp_char == '\b') || (tmp_char == 0x7F))
		{
			if (rt.HostRxBuffPtr == 1)									// first arrived char is backspace, ignore and move cursor back, echo ptr is still zero
			{
				rt.HostRxBuffPtr = 0;
				rt.HostRxBuff[rt.HostRxBuffPtr] = 0;					// and clear 1st byte
			}
			else if (rt.HostRxBuffPtr > 1)								// send "\b \b": CursorBack, space to clear symbol, CursorBack to put cursor on clear space
			{
				rt.HostRxBuffPtr = rt.HostRxBuffPtr - 2;				// move pointer for the next arriving char back 2 times
				rt.HostRxBuff[rt.HostRxBuffPtr] = 0;					// and clear 2 bytes
				rt.HostRxBuff[rt.HostRxBuffPtr + 1] = 0;
				if (rt.EchoRxBuffPtr > 0) rt.EchoRxBuffPtr = rt.EchoRxBuffPtr - 1;	// move pointer for the char-to-echo back 1 index
				if (rt.Host & CharEchoFlag)								// for editing on terminal window: using "Backspace" delete button
				{
					//#ifdef PC	//for PC when 'backspace' is pressed, cursor is moved on previous position.
								//for PC echo should be suppressed, when typing in console window it shows automatically.
					if (rt.EchoRxBuffPtr > 0) rt.EchoRxBuffPtr = rt.EchoRxBuffPtr - 1; // on PC console, cursor was alredy moved back
					cputs("\b \b");	// \b send to terminal 'backspace', clear position by printing 'space' and to terminal 'backspace'
					//#else //for Atmel
					//				if (rt.EchoRxBuffPtr > 0) rt.EchoRxBuffPtr = rt.EchoRxBuffPtr - 1; // this will be incremented at the end of while(Echo!=Host)
					//				cputs("\b \b");	// \b send to terminal 'backspace', clear position by printing 'space' and to terminal 'backspace'
					////				PutChar(rt.HostRxBuff[rt.EchoRxBuffPtr]);// echo ( CR was already discovered, printed and returned from function)
					//#endif // PC - Atmel
				}
			}
		}
		else if (rt.Host & CharEchoFlag)	// send back arrived char
		{
			//IK20220808 #ifndef PC // this causes double print on console" mmeennuu" if echo is ALSO printing in PC_SUPP.c char in function check_ch(int *ptrKEY)
			PutChar(rt.HostRxBuff[rt.EchoRxBuffPtr]);// echo ( CR was already discovered, printed and returned from function)
			//#endif // PC
		}
		if ((tmp_char == '?') &&				// IK20211119 fixed bug: "get" commands ending with '?': "ofs1?\r" returned this '?' "ofs1?=0\r"
			(rt.HostRxBuffPtr > 1)				// exclude '?' command ignore question mark except when it is the first char
			&& (rt.HostRxBuffPtr != 4)			// IK20211201 added "who\r" to RCI parser because '?' is trimmed from "who?" command;
			)
		{
			// the'?' is already in buffer, put by interrupt - replace it with zero
			rt.HostRxBuff[rt.EchoRxBuffPtr] = 0;
			rt.HostRxBuffPtr -= 1;				// move pointer back so next input overwrites '?' HostRxBuffPtr points to the NEXT place where next arriving char will be put
			rt.EchoRxBuffPtr--;
		}
		rt.EchoRxBuffPtr++;
		rt.EchoRxBuffPtr = rt.EchoRxBuffPtr & (HOST_RX_BUFF_LEN - 1);	// prevent boundary violation
	}
exit_ProcessChar:
	clearBit(rt.Host, CharAvailableFlag);								// this char is processed, clear CharAvailableFlag
}

//********************************
// the word 'TwoChars' accepted as two bytes:
// upper and lower byte, each send out
//
//********************************
void PutTwoChars(int TwoChars)
{
#ifndef __cplusplus
	register
#endif
		int t_chr = (TwoChars >> 8) & 0x00FF;
	PutChar(t_chr);
	t_chr = (TwoChars & 0x00FF);
	PutChar(t_chr);
}

/*************************************************************/
Uint32 toLower(Uchar ch)
{
#ifndef __cplusplus
	register
#endif
		int ch2;
#ifndef __cplusplus
	register
#endif
		int ch1 = ch;// & 0x00ff; // strip high byte
	ch2 = ch1 + 32;// ('A' - 'a')= -32, so -('A' - 'a')=+32;
	//   ch = (char)( (unsigned int)(ch2 - 'a') > (unsigned int)('z' - 'a')) ? ch1 : ch2;
	//   return (unsigned int)ch;
	return ((Uint32)(ch2 - 'a') > (Uint32)('z' - 'a')) ? ch1 : ch2; //tolower
}

//int tolower(int ch)
//{
//	register int ch2;
//	register int ch1 = ch;// & 0x00ff; // strip high byte
//	ch2 = ch1 + ('a' - 'A');// ch2 is upper case of ch1; ('A' - 'a')= -32
//	return ((unsigned int)(ch1 - 'A') <= (unsigned int)('Z' - 'A')) ? ch2 : ch1;
//}


/******************************************************************************/
void SendCrLf(void)
{
	PutTwoChars(256 * '\r' + '\n');//CRLF;
}

#ifdef PC
void ClearConsole() {
	ClearScreen();
}
#endif
/******************************************************************************/
/*                                                                            */
/*              send message ending with CrLf to rt.Host on UART              */
/*                                                                            */
/******************************************************************************/
void SendMsgToPC(char FL* Msg)
{
	//CopyConstString(Msg, printf_buff);
	//PutStr(printf_buff);
	cputs(Msg);
	SendCrLf();
}
/******************************************************************************/
void Print_Help(void)
{
	SendMsgToPC("vers	Get FW Version");
	SendMsgToPC("init	Re-initialize");
	SendMsgToPC("dflt	Set Default Params");
	SendMsgToPC("echo	Get/set echo>enable echo>disable");
	SendMsgToPC("save	Save Params");
#ifdef FULL_HELP //IK20251223 expand if FW has enough free FLASH space
	SendMsgToPC("Help	This info");
#endif // FULL_HELP
}
void DoNothing() {
	__no_operation();
}
/*****************************************************************/
void Print_Status(void)
{
	printf("%02x", (rt.OperStatusWord));
}

/********************************/
void IgnoreComment(void) //used for test
{
}

/*************************************************************
|                                                                              |
FWver 3.1| Bat Volt|  +Bus V |  -Bus V |VgndFault|mV ripple|mA Ripple| mA Iout |
---------+---------+---------+---------+---------+---------+---------+----------
Measured |  123.4  |   62.3  |  -62.5  |    0.8  |   180.3 |   22.5  |   12.44 |
---------+---------+---------+---------+---------+---------+---------+----------
Alarm    |    OK   |    OK   |    OK   |    OK   |    OK   |    OK   |    OK   |
---------+---------+---------+---------+---------+---------+---------+----------
AlarmEnab|   YES   |   YES   |   YES   |   NO    |   YES   |   YES   |   NO    |
---------+---------+---------+---------+---------+---------+---------+----------
High Lim |   142   |         |         |   13.0  |   200   |         |         |
Low Lim  |   105   |         |         |         |         |   10.0  |         |
---------+---------+---------+---------+---------+---------+---------+----------
Unit V   =  125V
CurOutput= 4-20 mA | Buzzer  = Enabled | Latch   = Disabled| Test Pulse = OFF  |
Protocol=  DNP-3   | Address =    3    | Baud    = 19200   | Charger  = 1 phase|
AlrmDelay=  2 sec  |interCharGap= 1046 | DLL_timeout= 10 ms| Xmt_delay = 100 ms|




*/
// Helpers for text output
char FL* TEXT_OK[2] = { "OK ", "FAIL" };
//#define TEXT_OK(x)      ((x) ? "OK " : "FAIL")
char FL* TEXT_YESNO[2] = { "YES", "NO " };
//#define TEXT_YESNO(x)   ((x) ? "YES" : "NO ")
char FL* TEXT_EN[2] = { "Enabled ", "Disabled" };
//#define TEXT_EN(x)      ((x) ? "Enabled " : "Disabled")
//#define TEXT_PHASE(x)   ((x) ? "3 phase" : "1 phase")
char FL* TEXT_PHASE[2] = { "3 phase", "1 phase" };
// Helper: 80-column lines
void print_line_80chars(void) {
	// IK20250819 this line takes 67 more FLASH bytes than for() cycle
	// printf("--------------------------------------------------------------------------------\r\n");
	uint8 c;
	for (c = 0; c < 79; c++) {
		PutChar('-'); //IK20250819 on PC simulation, printf("-") works much slower than PutChar('-') !?
	}
	PrintNewLine();
}

void print_table_separator_80chars(void) {
	//printf("---------+-------+-------+-------+-------+-------+-------+-------+-------+------\r\n");
	uint8 i, c;
	cputs("--------");
	for (i = 0; i <= (70); i++) {
		if (i % 8 != 0)
			c = '-';
		else
			c = '+';
		PutChar(c);
	}
	PrintNewLine();
}
// Small helpers for text
char FL* OkFail(uint8 state) { return state ? "FAIL " : " OK  "; }
char FL* Yes_No(uint8 state) { return state ? " YES" : " NO "; }

// Names for alarm bits (shortened for table fit)
char FL* AlarmNames[] = {
	"BatHi ", "BatLo ", "+ GF  ", "- GF  ", "mVrip ",
	"mArip ", " AC   ", " HiZ  ", "Ebat"
};

float CurrentLoopPrediction(void) {
	float t_f = rt.OutData.measured.battery_voltage_f / (SysData.NV_UI.V20 - SysData.NV_UI.V4) - 1;
	float t_c;
	if (SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) {
		t_c = 4.0 + (20 - 4) * t_f;
	}
	else {
		t_c = 0.0 + (1 - 0) * t_f;
	}
	return t_c;
}

#ifndef PC
void clrscr(void)
{
	cputs("\r\033[2J");
}
/***************************************************
 function gotoxy
	Sends ANSI code to move cursor to position

	Inputs:
		int x,y					horizontal and vertical location to move to
--------------------------------------------------*/
void gotoxy(int x, int y)
{
	printf("\033[%d;%dH", y, x);
}

#endif


void SendRTdata(void) //"d" or "data" command
{
	// Row: Measured values
	printf("Measured|%6.1f |%6.1f |%6.1f |%6.1f |%6.1f |%6.1f |%6.3f | %04X\r\n",
		rt.OutData.measured.battery_voltage_f,
		rt.OutData.measured.plus_gnd_volts_f,
		rt.OutData.measured.minus_gnd_volts_f,
		(rt.OutData.measured.plus_gnd_volts_f + rt.OutData.measured.minus_gnd_volts_f), // example VgndFault
		rt.OutData.measured.ripple_mV_f,
		rt.OutData.measured.ripple_mA_f,
		CurrentLoopPrediction(),
		Display_Info.alarm_status);
}

/// <summary>
/// Prints via serial port full or brief snapshot of system status:
/// </summary>
/// <param name="PrintType"> if TRUE, prints full, if FALSE short snapshot</param>
void Print_System_Info(Schar PrintType)
{
#define N_ALARMS  (sizeof(AlarmNames)/sizeof(AlarmNames[0]))
	uint8 i;
	gotoxy(1, 1);

	if (PrintType == FULL_SNAPSHOT) {
		// Header
		Print_FW_Version();
		PrintNewLine();
		print_line_80chars();

		//	printf("UnitType : %4uV |    Charger:  %s  | PulseTest: %s|\r\n",
		printf("UnitType: %4uV | Assuming ", SysData.NV_UI.unit_type);
		cputs(TEXT_PHASE[((SysData.NV_UI.SavedStatusWord & SinglePhase_eq0_3ph_eq1_Bit) == 0)]); //, % s does not work for stings in flash
		cputs(" Battery Charger for ripple monitoring\r\n");

		//printf("Curr Loop Output |%smA | %2s mA => %3uV | %s mA => %3uV | PulseTest: %s |\n\r",
		//	(SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? " 0-1 " : "4-20",
		//	(SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? "4" : "0",
		//	SysData.NV_UI.V4,
		//	(SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? "20" : " 1",
		//	SysData.NV_UI.V20,
		//	TEXT_YESNO[rt.pulse!=0]
		//);
		print_line_80chars(); // puts("-------------------------------------------------------------------------------");
		cputs("Curr Loop Output|");
		cputs((SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? " 0-1" : "4-20");
		cputs("mA | ");
		cputs((SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? "0" : "4");
		printf(" mA => %4uV | ", SysData.NV_UI.V4);
		cputs((SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) ? "1" : "20");
		printf(" mA => %3uV | PulseTest: ", SysData.NV_UI.V20);
		cputs(TEXT_YESNO[rt.pulse == 0]);

		// IK20250825 IAR printf could not pring %s because it expects pointers to DATA space
		// Have to use cputs()
		//printf("\r\nProtocol | %-6s|Adr%4u| Baud = %6u |Tx Delay=%3u ms| InterCharGap=%4u ms\r\n",
		//	(SysData.NV_UI.StartUpProtocol == DNP3) ? "DNP-3 " :
		//	(SysData.NV_UI.StartUpProtocol == MODB) ? "MODBUS" :
		//	(SysData.NV_UI.StartUpProtocol == ASCII_CMDS) ? "ASCII " : "SETUP",
		//	SysData.NV_UI.meter_address,
		//	SysData.NV_UI.baud_rate,
		//	SysData.xmt_delay,
		//	SysData.inter_char_gap
		//);

		cputs("\r\nProtocol| ");
		cputs(
			(SysData.NV_UI.StartUpProtocol == DNP3) ? "DNP-3 " :
			(SysData.NV_UI.StartUpProtocol == MODBUS) ? "MODBUS" :
			(SysData.NV_UI.StartUpProtocol == ASCII_CMDS) ? "ASCII " : "SETUP");
		printf(" |Adr%4.0f| Baud = %5.0f |TxDelay =%3u ms| InterCharGap=%4u ms\r\n",
			SysData.NV_UI.meter_address,
			Existing.baud_rate,
			SysData.xmt_delay,
			SysData.inter_char_gap
		);
		print_line_80chars();
	}

	if (PrintType != REALTIME_SNAPSHOT)  // print static line
		cputs(" SIGNAL |BatVolt|+Bus V |-Bus V |VgndFlt|mV ripp|mA ripp|mA Iout|AlarmWord");
	PrintNewLine();

	// Row: Measured values
	SendRTdata();

	if (PrintType == FULL_SNAPSHOT) {
		print_table_separator_80chars();

		printf("High Lim| %5.1f |       | %5.1f | %5.1f | %5.1f |       |\r\n",
			SysData.NV_UI.high_bat_threshold_V_f,
			SysData.NV_UI.plus_gf_threshold_V_f,
			SysData.NV_UI.minus_gf_threshold_V_f,
			SysData.NV_UI.ripple_V_threshold_mV_f);
		printf("Low Lim |       | %5.1f |       |       |       | %5.1f |\r\n",
			SysData.NV_UI.low_bat_threshold_V_f,
			SysData.NV_UI.ripple_I_threshold_mA_f);
		print_line_80chars();
	}

	if (PrintType != REALTIME_SNAPSHOT)  // print static line
	{
		cputs(" ALARMS | ");
		if (PrintType == FULL_SNAPSHOT) {
			printf("grace period % 4.1f sec | Buzzer: ", SysData.NV_UI.alarm_delay_sec_f);
			cputs(TEXT_EN[((SysData.NV_UI.SavedStatusWord & Buzzer_ON_eq1_Bit) != 0)]);
			cputs("      |  Latch: ");
			cputs(TEXT_EN[((SysData.NV_UI.SavedStatusWord & Latch_ON_eq1_Bit) != 0)]);
			PrintNewLine();

			print_table_separator_80chars();
			cputs("Condition BatHi ");
		}
		else // BRIEF_SNAPSHOT
		{
			cputs("BatHi ");
		}
		for (i = 1; i < N_ALARMS; i++) // creates "| BatHi | BatLo | + GF  | - GF  | mVrip | mArip |  AC   |  HiZ  | E-Bat"
		{
			cputs("| ");
			cputs(AlarmNames[i]);
		}
	}
	PrintNewLine();
	printf("Enabled?");
	for (i = 0; i < N_ALARMS; i++) {
		uint8 enabled = !(SysData.NV_UI.disabled_alarms & (1u << i));
		cputs("| ");
		cputs(Yes_No(enabled));
		if (i < N_ALARMS - 1) cputs("  ");
	}
	PrintNewLine();
	if (PrintType == FULL_SNAPSHOT) print_table_separator_80chars();


	// Alarm row (status bits)
	cputs(" State ");
	for (i = 0; i < N_ALARMS; i++) {
		uint8 active = (Display_Info.alarm_status & (1u << i)) ? 1 : 0;
		cputs(" | ");
		cputs(OkFail(active));
	}
	PrintNewLine();
	if (PrintType == FULL_SNAPSHOT) print_table_separator_80chars();
	else // if (PrintType == BRIF_SNAPSHOT) || REALTIME_SNAPSHOT
		Show_ADCcounts_and_Volts();
}

void Print_System_Snapshot(void) //"w" command
{
	Print_System_Info(FULL_SNAPSHOT);
}

void Print_System_Brief(void) //"b" or "brif" command
{
	Print_System_Info(BRIF_SNAPSHOT);
}

/***************************************************************/
// command "rtmon>enable; rtmon>disable
//

/// <summary>
/// "rtmon>enable" command
/// Important chars
/// the '>enab' must start on 6th position, index [5]
/// anything else disables real time monitoring
/// </summary>
/// <param name=""></param>
void SetRealTimeMonitoring(void) {
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0]
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 2]); //"enab" or "disa" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN + 1] == '>')
	{
		if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
		{
			clrscr();
			Print_System_Brief();
			setBit(rt.OperStatusWord, SendRealTimeData_eq1_Bit);
			timer.RealTimeUpdate = 1;
		}
	}
	else //if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
		clearBit(rt.OperStatusWord, SendRealTimeData_eq1_Bit);

}

/*****************************************************************/
void InitATMEL(void)
{
	SendMsgToPC("Restarting\r\n>~OK");
	restart_op = true;
}

/*************************************************************/
void SetDefaultsInRAM_by_command(void)
{
	SetDefaultsInRAM();//	SET DEFAULTS IN RAM
}


// *************************************************************
void SaveCal(void)
{
#ifndef PC

#endif
	SaveParams();
#ifndef PC
#endif
}

// "disN=54321"  send string to upper Numeric LED display
void Send_to_NumericLEDs() {
	char* num_str = (char*)&rt.HostRxBuff[5];
	Write_Numeric_Display(num_str);
}

// "disI=ARGA"  send string to lower Information ASCII LED display
void Send_to_ASCII_LEDs() {
	char* str = (char*)&rt.HostRxBuff[5];
	Write_ASCII_Display(str);
}

// "disL=107"  send integer to turn ON/OFF LED display
// if value is more than 64 - easier to test firmware setting 107 to turn all LESs on or 0 to disable overwrite
// MAYBE LATER: if upper bit of LED_word is set, it overwrites ON-OFF (assigned by Display board firmware) with 3 lower bits of LED_word
void Send_to_STATUS_LEDs() {
	char* str = &rt.HostRxBuff[5];
	if (!Is_Numeric(str)) {
		ErrorStatus = PARAM_ERROR;
		return;
	}
	uint8 LED_word = (uint8)atol((char*)str);
	Display_Info.Status &= 0x00; // clear all bits. Defined 3 bits LEDs, 1 bit for buzzer and Bit_6 to overwrite Display board firmware setting
	Display_Info.Status |= LED_word;
	Write_LEDs_OnDisplayBoard(LED_word);
}

// this function is called from main loop
// it requests button states from Display board via TWI
// momentary state arrives into twi.buffer[BYTE_3]
// then state copied into Display_Info.butt_states
// NOTE: definitons of instant press are in transmitted in BYTE[3],
// function reads them into Uchar variable
// #define BUTTON_AUTO_INSTANT_PRESS_BIT   Bit_8  // if (buttons & 0x0100)    // instantenious status of Manual/auto button, 0==pressed, without delay, updates each main loop cycle
// #define BUTTON_LIMIT_INSTANT_PRESS_BIT  Bit_9  // if (buttons & 0x0200)    // instantenious status of Limit button, 0==pressed, without delay, updates each main loop cycle
// #define BUTTON_UP_INSTANT_PRESS_BIT     Bit_10 // if (buttons & 0x0400)    // instantenious status of UP button, 0==pressed, without delay, updates each main loop cycle
// #define BUTTON_DOWN_INSTANT_PRESS_BIT   Bit_11 // if (buttons & 0x0800)    // instantenious status of DOWN button, 0==pressed, without delay, updates each main loop cycle


// this test function is called from serial interface
void Get_Buttons() {
	//uint8 buttons_lowByte = 0;
	//uint8 buttons_highByte = 0;
	//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_GET_FRONT_BUTTONS, 0, 0);
	//TWI_Read(DISPLAY_READ);               //get info from display board

	uint8  buttonsHits = Display_Info.buttons_hits; // to avid message in IAR:	Warning[Pa082]: undefined behavior: the order of volatile accesses is undefined in this statement
	if (twi.s.error > 0) {
		printf("TWI_error = %X", twi.s.error);
		return;
	}
	printf("Pressed now 0x%X, was pressed 0x%X", buttonsHits, Display_Info.last_butt_pressed);
}

/******************************************************************************/
void Send_RCI_Param_Error(char* valid_msg)
{
	ErrorStatus = PARAM_ERROR;
	printf(">~ERR BAD param %s; valid: %s", rt.HostRxBuff, valid_msg);
}

/******************************************************************************/
void Send_RCI_Param_Error_as_FlashConst(char FL* valid_msg)
{
	CopyConstString(valid_msg, printf_buff);
	ErrorStatus = PARAM_ERROR;
	printf(">~ERR BAD param %s; valid: %s", rt.HostRxBuff, printf_buff);
}

/******************************************************************************/
void Send_comment(char FL* comment)
{
	//printf(" // %s", comment);
	cputs(" // ");
	cputs(comment);
}

/******************************************************************************/
void Send_verbose_comment(char* comment)
{
	if (testBit(rt.Host, CmdVerboseResponse))
	{
		int chr;

		cputs(" // ");
		while (*comment != 0) // while pointer points NOT to end of string, 0
		{
			chr = *comment;
			PutChar(chr); // print char
			comment++;
		}
	}
}

/******************************************************************************/
void Send_verbose_comment_as_FlashConst(char FL* comment)
{
	//CopyConstString(comment, printf_buff);
	if (testBit(rt.Host, CmdVerboseResponse))	Send_comment(comment);
}

/*************************************************************** /
// SetGet_param() updates / shows EITHER 'Qfloat' OR 'long', can add "verbose" short help
// "float_offset" == 0...15 is where search for float string should begin.
// offset in command parameter string where float starts (i.e, after "="),delay= 6th index, 7th position
// It is additional string position starting from CMD_LEN == 4
// ALSO, if float offset is < 16, show 'Qfloat', otherwise, show 'long'
// *************************************************************/
#define SHOW_LONG     0x10
#define SHOW_Qfloat      0
void SetGet_param(int float_offset, float minValue, float maxValue, float* Qf_var_ptr, char* verb_msg)
{
	float temp_float;
	char* temp_Inp_str = CommStr;										// pointer to RxBuff[0]
	int str_ptr = CMD_LEN + (float_offset & (SHOW_LONG - 1));

	if (temp_Inp_str[str_ptr] == '=')
	{
		str_ptr++;
		if (Is_Numeric(&temp_Inp_str[str_ptr]) != TRUE) goto par_error;	//cannot be converted into numeric

		temp_float = (float)atof((char*)&temp_Inp_str[str_ptr]);
		//		Value = strlen(&temp_Inp_str[CMD_LEN+1]); //Value is string length
		if ((temp_float > maxValue) || (temp_float < minValue))
			goto par_error;
		//		if((temp_float ==0.0f) && (temp_Inp_str[CMD_LEN+1] !='0'))	// converted value is 0 but the first char is not '0'
		//			goto par_error;

		if (float_offset < SHOW_LONG)
			*Qf_var_ptr = (temp_float);									// save as Qfloat
		else
			*Qf_var_ptr = (long)(temp_float);							// save as 'long'

		return;
	}
	else // everything else - query
	//if (temp_Inp_str[str_ptr]=='?') //query <CMD>?
	{
		//int ptr = 0;//CMD_LEN;
		PutStr((char*)temp_Inp_str);
		temp_float = *Qf_var_ptr;
		if (float_offset < SHOW_LONG) // means - print float value with fraction
		{
			char FL* frmt = "=%3.2f";
			if (temp_float < 10) frmt = "=%1.7f";
			else if (temp_float < 100) frmt = "=%2.3f";
			printf(frmt, *Qf_var_ptr);//show float
		}
		else
			printf("=%3.0f", temp_float + 0.499f);//show long as a float without fraction part
		//test printf("=%3.0f", 4.54321f);//show long as a float without fraction part
		//test printf(" printf float %5.2f\r\n", 4.5);   // test

		Send_verbose_comment(verb_msg);
		return;
	}
par_error:
	ErrorStatus = PARAM_ERROR;
	printf(">~ERR VALUE %s, range [%g..%g]", temp_Inp_str, minValue, maxValue);
	//		Send_RCI_Param_Error();
}

/******************************************************************************/
//	ASCIIToHexChar
//	Converts value in_char (ASCII '0'...'9', 'a'...'f') to Hex half-byte, 0...15.
//  Char Case does not matter, returns -1 if non-valid ASCII
//*************************************************************

int ASCIIToHexChar(char in_char)
{
#ifndef __cplusplus
	register
#endif
		int in_int = toupper((int)in_char);
	if ((in_int >= '0') && (in_int <= '9'))				// 0 - 9?
	{
		return (int)(in_int - 0x30);					// binary value
	}

	if ((in_int >= 'A') && (in_int <= 'F'))				// a - f?
	{
		return (int)(in_int - 0x37);					// binary value
	}

	ErrorStatus = BAD_VALUE_ERR;
	return(-1);
}

char FL* CalNames[8] = {// IK20250130 be careful with the length, I reserved only 40 bytes for a temporary string in stack in function SetGetCalParam(void) - char Cal_Name[40];
	"BatteryVolts",		// Y1 X1 Y2 X2 Battery Voltage calibration
	"FaultVolts",		// Y1 X1 Y2 X2 Fault Voltage calibration
	"MinusGndVolts",	// Y1 X1 Y2 X2 Minus Grnd voltage correction factor info
	"RippleVolts1ph",	// Y1 X1 Y2 X2 single phase ripple voltage calibration
	"RippleVolts3ph",	// Y1 X1 Y2 X2 three phase ripple voltage calibration
	"RippleCurr1ph",	// Y1 X1 Y2 X2 single phase ripple current calibration
	"RippleCurr3ph",	// Y1 X1 Y2 X2 three phase ripple current calibration
	"CurrentOut_I420",	// NU X1 NU X2 current loop calibration, value in PWM register to get 4 mA or 20 mA
};

/**************************************************************
"cpar<index1><index2>=ggg.gg" - set/get will define "Calibration PARameter" or "factor" for a particular cal, command takes 2 arguments
"cpar#$=GGG.GGG\r" SET calibration parameter.
"cpar#$": returns ->"cpar#$=ggg.gg\r" - GET calibration parameter
Command itself takes 4 chars in command string, [0] to [3]
caliration allows to use linear interpolation of input value == ADC_counts into Engineerng Unit value, i.e. Voltage, Current, etc.
calibration factors are stored as two sets of X,Y coordinates, {X1,Y1} low cal point, {X2,Y2} high cal point, which define linear interpolation
NOTE: X1 must not be equial to X2, and X1 should be lower value than X2
Argument "#" on 5th place [4] in command string: defines calibration for:
0 =>	Calibr2points BatteryVolts;	// 0x040  16 // Y1 X1 Y2 X2 Battery Voltage calibration
1 =>	Calibr2points FaultVolts;		// 0x050  16 // Y1 X1 Y2 X2 Fault Voltage calibration
2 =>	Calibr2points MinusGndVolts;	// 0x060  16 // Y1 X1 Y2 X2 Minus Grnd voltage correction factor info
3 =>	Calibr2points RippleVolts1ph;	// 0x070  16 // Y1 X1 Y2 X2 single phase ripple voltage calibration
4 =>	Calibr2points RippleVolts3ph;	// 0x080  16 // Y1 X1 Y2 X2 three phase ripple voltage calibration
5 =>	Calibr2points RippleCurr1ph;	// 0x090  16 // Y1 X1 Y2 X2 single phase ripple current calibration
6 =>	Calibr2points RippleCurr3ph;	// 0x0A0  16 // Y1 X1 Y2 X2 three phase ripple current calibration
7 =>	Calibr2points CurrentOut_I420;	// 0x0B0  16 // Y1 NU Y2 NU current loop calibration, value in PWM register to get 4 mA or 20 mA, X coordinates are Not Used

Argument "$" on 6th place [5] in command string, range {0...3}, defines coordinate: 0=X1_low_point, 1=Y1_low_point, 2=X2_high_point, 3=Y2_high_point.
On the 7th place [6] of command string can be '=' the SET command. If it is NOT "=" then it is interpreted as GET command. On the 7th place [6] then can be '?' or nothing.
Value "ggg.ggg" starts on 8th place [7], and calibration factor can be in regular or scientific notation "+/-m.mmE+/-pp"

**************************************************************/
void SetGetCalParam(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	CalPtr CalStructurePtr;
	char Cal_Name[40]; // to fit Send_RCI_Param_Error("arg *# where # is: 0-Y1,1-X1,2-Y2,3-X2");
	char CoordName[16];

	//decode '#' parameter - calibration type, there are 8 calibrations
	int index2 = 0;
	int index1 = ASCIIToHexChar(temp_Inp_str[CMD_LEN]);
	if ((index1 < 0) || (index1 > 7)) goto error_param;
	CopyConstString(CalNames[index1], Cal_Name);
	CalStructurePtr = (CalPtr)&SysData.BatteryVolts; // In Visual Studio, pointer is 0x00319d00.
	// calculate pointer to a defined by command structure
	CalStructurePtr += index1; // IK20250130 Visual Studio changes pointer by 0x10 to 0x00319d10
	// CalStructurePtr += index1 * sizeof(Calibr2points); // Visual Studio changes pointer by 0x100, not by 0x10: to 0x00319e00
	//decode '$' parameter - particular coordinate, in the order Y1,X1,Y2,X2
	index2 = ASCIIToHexChar(temp_Inp_str[CMD_LEN + 1]); //[5] $ param defines coordinate: 0=X1_low_point, 1=Y1_low_point, 2=X2_high_point, 3=Y2_high_point.
	//float Param = CalStructurePtr->Coord[index2]; //test
	if (index2 == X1_low_point) // == 0
	{
		//CopyConstString("Measured", CoordName);
		CopyConstString("<-ADCcounts", CoordName);
		sprintf(RCI_message, "%s\tPoint 1, X coord, 0..32767, %s", CoordName, Cal_Name); // X coordinate is Int16 ADC counts
		SetGet_param(2 + SHOW_LONG, 0.0f, 32767.0f, &CalStructurePtr->Coord[X1_low_point], RCI_message);
	}
	else if (index2 == Y1_low_point) // == 1
	{
		CopyConstString("<-Calibr to", CoordName);
		sprintf(RCI_message, "%s\tPoint 1, Y coord, 0..300000, %s", CoordName, Cal_Name); // Y coordinate is float: Volts or Ripple mV or Ripple Current mA
		SetGet_param(2 + SHOW_Qfloat, 0.0f, 300000.0f, &(CalStructurePtr->Coord[Y1_low_point]), RCI_message);
	}
	else if (index2 == X2_high_point) // == 2
	{
		CopyConstString("<-ADCcounts", CoordName);
		sprintf(RCI_message, "%s\tPoint 2, X coord, 0..32767, %s", CoordName, Cal_Name); // X coordinate is Int16 ADC counts
		SetGet_param(2 + SHOW_LONG, 0.0f, 32767.0f, &((CalPtr)CalStructurePtr)->Coord[X2_high_point], RCI_message);
	}
	else if (index2 == Y2_high_point) // == 3
	{
		CopyConstString("<-Calibr to", CoordName);
		sprintf(RCI_message, "%s\tPoint 2, Y coord, 0..300000, %s", CoordName, Cal_Name); // Y coordinate is float: Volts or Ripple mV or Ripple Current mA
		SetGet_param(2 + SHOW_Qfloat, 0.0f, 300000.0f, &((CalPtr)CalStructurePtr)->Coord[Y2_high_point], RCI_message);
	}
	else
	{
	error_param:
		CopyConstString("arg *# where # is: 0-X1,1-Y1,2-X2,3-Y2", Cal_Name);
		Send_RCI_Param_Error(Cal_Name);
		return; //goto error_gain;
	}
}

// Channel  |NotConn|Battery| Fault |-GndBus|NC/+Bus|RipCurr|RipVolt|NotConn

uint8 ADC_channel_Of_BatteryVolts[] =
{
	1, //ADC channel #1 for BatteryVolts
	2, //ADC channel #2 for FaultVolts
	3, //ADC channel #3 for MinusGndVolts
	6, //ADC channel #5 for RippleVolts1ph
	6, //ADC channel #5 for RippleVolts3ph
	5, //ADC channel #6 for RippleCurr1ph
	5, //ADC channel #6 for RippleCurr3ph
	7  //ADC channel #0 for CurrentOut_I420 - Not connected //-!- IK20251119 - this is special case because firmware does not read ADC channel for current loop calibration
};
/**************************************************************
"cali<index1><index2>=ggg.gg" - set "CALIbration value" or "factor" for a particular cal, command takes 2 arguments
"cali#$=GGG.GGG" \r" SET Calibration value
"cali#$": returns ->"cali#$=ggg.gg\r" - GET calibration parameter
the difference vs. "cpar" command is that "cali" command sets a calibration value supplied via serial interface, based on low or high calibration point (the'$')
Command itself takes 4 chars in command string, [0] to [3]
caliration allows to use linear interpolation of input value == ADC_counts into Engineerng Unit value, i.e. Voltage, Current, etc.
calibration factors are stored as two sets of X,Y coordinates, {X1,Y1} low cal point, {X2,Y2} high cal point, which define linear interpolation
NOTE: X1 must not be equial to X2, and X1 should be lower value than X2
Argument "#" on 5th place [4] in command string: defines calibration for:
0 =>	Calibr2points BatteryVolts;		// 0x040  16 // Y1 X1 Y2 X2 Battery Voltage calibration
1 =>	Calibr2points FaultVolts;		// 0x050  16 // Y1 X1 Y2 X2 Fault Voltage calibration
2 =>	Calibr2points MinusGndVolts;	// 0x060  16 // Y1 X1 Y2 X2 Minus Grnd voltage correction factor info
3 =>	Calibr2points RippleVolts1ph;	// 0x070  16 // Y1 X1 Y2 X2 single phase ripple voltage calibration
4 =>	Calibr2points RippleVolts3ph;	// 0x080  16 // Y1 X1 Y2 X2 three phase ripple voltage calibration
5 =>	Calibr2points RippleCurr1ph;	// 0x090  16 // Y1 X1 Y2 X2 single phase ripple current calibration
6 =>	Calibr2points RippleCurr3ph;	// 0x0A0  16 // Y1 X1 Y2 X2 three phase ripple current calibration
7 =>	Calibr2points CurrentOut_I420;	// 0x0B0  16 // Y1 NU Y2 NU current loop calibration, value in PWM register to get 4 mA or 20 mA, X coordinates are Not Used

Argument "$" on 6th place [5] in command string, range {0...3}, defines coordinate: 0=X1_low_point, 1=Y1_low_point, 2=X2_high_point, 3=Y2_high_point.
On the 7th place [6] of command string can be '=' the SET command. If it is NOT "=" then it is interpreted as GET command. On the 7th place [6] then can be '?' or nothing.
Value "ggg.ggg" starts on 8th place [7], and calibration factor can be in regular or scientific notation "+/-m.mmE+/-pp"
**************************************************************/
void SetCalibrationValue(void) //-!- K20251119 need special treatment for current loop calibration
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	CalPtr CalStructurePtr;
	char Cal_Name[40]; // to fit Send_RCI_Param_Error("arg *# where # is: 0-Y1,1-X1,2-Y2,3-X2");
	char CoordName[16];

	//decode '#' parameter - calibration type, there are 8 calibrations
	int index2 = 0;
	int index1 = ASCIIToHexChar(temp_Inp_str[CMD_LEN]);
	if ((index1 < 0) || (index1 > 7)) goto error_param;
	//decode '$' parameter - particular coordinate, in the order Y1,X1,Y2,X2
	index2 = ASCIIToHexChar(temp_Inp_str[CMD_LEN + 1]); //[5] $ param defines coordinate: 0=X1_low_point, 1=Y1_low_point, 2=X2_high_point, 3=Y2_high_point.
	CopyConstString(CalNames[index1], Cal_Name);
	CalStructurePtr = (CalPtr)&SysData.BatteryVolts; // In Visual Studio, pointer is 0x00319d00.
	// calculate pointer to a defined by command structure
	CalStructurePtr += index1; // IK20250130 Visual Studio changes pointer by 0x10 to 0x00319d10

	if ((index2 == X1_low_point) || (index2 == Y1_low_point))// == 0 or == 1
	{
		CopyConstString("<-ADCcounts", CoordName);
		sprintf(RCI_message, "%s\tPoint 1, X coord, 0..32767, %s", CoordName, Cal_Name); // X coordinate is Int16 ADC counts
		CalStructurePtr->Coord[X1_low_point] = rt.ADC_buff[ADC_channel_Of_BatteryVolts[index1]]; // IK20250130 set X1 to current ADC counts reading for that calibration
		//IK20251119 specifically point NOT to '=' (there is no 2 + SHOW_LONG), so it is interpreted as GET command and shows current ADC value
		SetGet_param(SHOW_LONG, 0.0f, 32767.0f, &CalStructurePtr->Coord[X1_low_point], RCI_message);

		CopyConstString("<-Calibr to", CoordName);
		sprintf(RCI_message, "%s\tPoint 1, Y coord, 0..300000, %s", CoordName, Cal_Name); // Y coordinate is Int32 Voltage, in V or ripple mV (3000 mV = 3.0V) or ripple current in mA
		//IK20251119 now, point to '=', so it is interpreted as SET command and assigns command argument (after '=') as a value to set for Y coordinate
		SetGet_param(2 + SHOW_Qfloat, 0.0f, 300000.0f, &(CalStructurePtr->Coord[Y1_low_point]), RCI_message);
	}
	else if ((index2 == X2_high_point) || (index2 == Y2_high_point))// == 2 or == 3
	{
		CopyConstString("<-ADCcounts", CoordName);
		sprintf(RCI_message, "%s\tPoint 2, X coord, 0..32767, %s", CoordName, Cal_Name); // X coordinate is Int16 ADC counts
		CalStructurePtr->Coord[X1_low_point] = rt.ADC_buff[ADC_channel_Of_BatteryVolts[index1]]; // IK20250130 set X1 to current ADC counts reading for that calibration
		//IK20251119 specifically point NOT to '=' (there is no 2 + SHOW_LONG), so it is interpreted as GET command and shows current ADC value
		SetGet_param(SHOW_LONG, 0.0f, 32767.0f, &((CalPtr)CalStructurePtr)->Coord[X2_high_point], RCI_message);
		CopyConstString("<-Calibr to", CoordName);
		sprintf(RCI_message, "%s\tPoint 2, Y coord, 0..300000, %s", CoordName, Cal_Name); // Y coordinate is Int32 Voltage mV (300000 mV = 300V) or Current mA
		//IK20251119 now, point to '=', so it is interpreted as SET command and assigns command argument (after '=') as a value to set for Y coordinate
		SetGet_param(2 + SHOW_Qfloat, 0.0f, 300000.0f, &((CalPtr)CalStructurePtr)->Coord[Y2_high_point], RCI_message);
	}
	else
	{
	error_param:
		CopyConstString("arg *# where # is: 0-X1,1-Y1,2-X2,3-Y2", Cal_Name);
		Send_RCI_Param_Error(Cal_Name);
		return; //goto error_gain;
	}
}

// *************************************************************
// "echo>enabled"  "echo>disabled"  "echo>verbose" sets echo and verbose flags.
// "echo?" returns state
/*************************************************************/
void Echo_Enab_Disab(void)
{
	Uint32 param = Convert_4_ASCII_to_Uint32(&CommStr[CMD_LEN]); //=dis, =ena; CommStr is pointer to &RxBuff[0] or &RxBuff[1] when command has preffix "`"
	if (param == (('>' + 256 * 'e') + ('n' + 256 * 'a') * 65536))
	{
		setBit(rt.Host, CharEchoFlag);
		clearBit(rt.Host, (CmdVerboseResponse));
	}
	else if (param == (('>' + 256 * 'v') + ('e' + 256 * 'r') * 65536))
	{
		setBit(rt.Host, (CharEchoFlag + CmdVerboseResponse));
	}
	else if (param == (('>' + 256 * 'd') + ('i' + 256 * 's') * 65536))
	{
		clearBit(rt.Host, (CharEchoFlag | CmdVerboseResponse));
	}
	else //if (CommStr[CMD_LEN] =='?')
	{
		Put_CMD_as_chars();
		PutChar('>');
		if (rt.Host & (CmdVerboseResponse)) cputs("Verbose");
		else if (rt.Host & (CharEchoFlag))
			cputs((char FL*)TEXT_EN[0]);
		else cputs((char FL*)TEXT_EN[1]);
	}
}

/*************************************************************/
// "adch"  Report ADC readings in format: chan0, chan1, chan2, chan3, chan4, chan5
void Show_ADCcounts_and_Volts(void)
{
	uint8 ch;
	//	cputs(" Channel  |NotConn|Battery| Fault | -GND  |NotConn|RipCurr|RipVolt|NotConn\r\nADC counts");
	cputs(" Channel  |NotConn|Battery| Fault |-GndBus|NC/+Bus|RipCurr|RipVolt|NotConn");
	cputs("\r\nADC counts");
	for (ch = 0; ch < 8; ch++) {
		printf("| %5d ", rt.ADC_buff[ch]);
	}

	cputs("\n\r Value    ");
	for (ch = 0; ch < 8; ch++) {
		printf("| %5.1f ", rt.OutData.measurement_f[ch]);
	}
}

/*************************************************************/
// Pulse set/get: 'pulse>on', 'pulse>off' - set right now; 'pulse>enab', IT DOES NOT SAVED into flash because it is test command.
void Excitation_Pulse(void)
{
	//if (SetGetStatus() != CALIBRATION_DONE)  //if calibrating shut off pulse
		//clearBit(display_status, DISP_STATE_PulseON_BIT); // &= 0xBF;
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 2]); //"024v" or "048v" or "125v" or "250v" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN + 1] == '>')
	{
		if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
			setBit(Display_Info.Status, DISP_STATE_PulseON_BIT);
		else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
			clearBit(Display_Info.Status, DISP_STATE_PulseON_BIT);
		else if (param == (('o' + 256 * 'n') + ('\0' + 256 * '\0') * 65536))
			setBit(Display_Info.Status, DISP_STATE_PulseON_BIT);
		else if (param == (('o' + 256 * 'f') + ('f' + 256 * '\0') * 65536))
			clearBit(Display_Info.Status, DISP_STATE_PulseON_BIT);
		else Send_RCI_Param_Error_as_FlashConst("on off enab disa");
	}
	else
	{
		Put_CMD_as_chars();
		PutTwoChars(256 * 'e' + '>');// 256 * '\r' + '\n'
		if (Display_Info.Status & DISP_STATE_PulseON_BIT)
			cputs("enabled");
		else cputs("disabled");
		Send_verbose_comment_as_FlashConst("Current status not EEPROM-saved");
	}
}

/*************************************************************/
// Alarm set/get: 'alarm rv>enab', 'alarm rv>disa'; 'alarm ri>enab', 'alarm ri>disa'; 'alarm ac>enab', 'alarm ac>disa'; 'alarm hz>enab', 'alarm hz>disa'
void SetGetAlarm(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	Uint32 alarm_type = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //starting at 5 bytes after command
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 5]); //starting at 5 bytes after command

	if (temp_Inp_str[CMD_LEN + 4] == '>')
	{
		if (alarm_type == ((' ' + 256 * 'r') + ('v' + 256 * '>') * 65536))
		{
			if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
				setBit(Display_Info.alarm_status, Alarm_Ripple_Voltage_Bit);
			else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
				clearBit(Display_Info.alarm_status, Alarm_Ripple_Voltage_Bit);
			else goto Exception;
		}
		else if (alarm_type == ((' ' + 256 * 'r') + ('i' + 256 * '>') * 65536))
		{
			if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
				setBit(Display_Info.alarm_status, Alarm_Ripple_Current_Bit);
			else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
				clearBit(Display_Info.alarm_status, Alarm_Ripple_Current_Bit);
			else goto Exception;
		}
		else if (alarm_type == ((' ' + 256 * 'a') + ('c' + 256 * '>') * 65536))
		{
			if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
				setBit(Display_Info.alarm_status, Alarm_AC_Loss_Bit);
			else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
				clearBit(Display_Info.alarm_status, Alarm_AC_Loss_Bit);
			else goto Exception;
		}
		else if (alarm_type == ((' ' + 256 * 'h') + ('z' + 256 * '>') * 65536))
		{
			if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
				setBit(Display_Info.alarm_status, Alarm_High_Impedance_Bit);
			else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
				clearBit(Display_Info.alarm_status, Alarm_High_Impedance_Bit);
			else goto Exception;
		}
		else Send_RCI_Param_Error_as_FlashConst("rv ri ac hz enab disa only");
	}
	else // ? -- get command
	{
		int ctr;
		char alarm_type[4][4] = { "rv>", "ri>", "ac>", "hz>" };
		int alarm_bit[4] = { Alarm_Ripple_Voltage_Bit, Alarm_Ripple_Current_Bit, Alarm_AC_Loss_Bit, Alarm_High_Impedance_Bit };
		for (ctr = 0; ctr <= 3; ctr++)
		{
			Put_CMD_as_chars(); //this prints "alar"
			PutTwoChars(256 * 'm' + ' ');// this prints "m "; so output is "alarm "
			//if (alarm_status & Alarm_Ripple_Voltage_Bit) {//
			//cputs("rv>");
			printf("%s", (char*)&alarm_type[ctr][0]);
			if (Display_Info.alarm_status & alarm_bit[ctr])
				cputs("enabled");
			else cputs("disabled");
			Send_verbose_comment_as_FlashConst("Current status not EEPROM-saved");
			if (ctr < 3) PrintNewLine();
		}
	}
	return;
Exception:
	Send_RCI_Param_Error_as_FlashConst("rv ri ac hz enab disa only");
}

void SetGetStatus(uint16 Status_bit)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0]
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //"enab" or "disa" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN] == '>')
	{
		if (param == (('e' + 256 * 'n') + ('a' + 256 * 'b') * 65536))
			setBit(SysData.NV_UI.SavedStatusWord, Status_bit);
		else if (param == (('d' + 256 * 'i') + ('s' + 256 * 'a') * 65536))
			clearBit(SysData.NV_UI.SavedStatusWord, Status_bit);
		else if (param == (('o' + 256 * 'n') + ('\0' + 256 * '\0') * 65536))
			setBit(SysData.NV_UI.SavedStatusWord, Status_bit);
		else if (param == (('o' + 256 * 'f') + ('f' + 256 * '\0') * 65536))
			clearBit(SysData.NV_UI.SavedStatusWord, Status_bit);
		else Send_RCI_Param_Error_as_FlashConst("on off enab disa");
	}
	else // ? -- get command
	{
		Put_CMD_as_chars();
		PutChar('>');
		if (SysData.NV_UI.SavedStatusWord & Status_bit)
			cputs("enabled");
		else cputs("disabled");
		Send_verbose_comment_as_FlashConst("StartUp status");
	}
}

/*************************************************************/
// Buzzer set/get: 'buzz>on', 'buzz>off' - set right now; 'buzz>enab', 'buzz>disa'. Setting in flash is updated after 'save' command.
void SetGetBuzzer(void)
{
	SetGetStatus(Buzzer_ON_eq1_Bit);
}

/*************************************************************/
// Get 'latc[h?]' returns Set command syntax 'latch>enab', 'latch>disa'. Setting in flash is updated after 'save' command.
void SetGetLatch(void)
{
	SetGetStatus(Latch_ON_eq1_Bit);
}

/*************************************************************/
// Delay set/get: 'delay? returns Set command syntax 'delay=XX.X' in seconds. Setting in flash is updated after 'save' command.
// precision is 0.1 sec; 2 sec delay is saved in flash as integer 20
// original firmware has 1 sec increment/decrement,
void SetGetDelay(void)
{
	float temp_N = SysData.NV_UI.alarm_delay_sec_f;// uint16, in seconds
	sprintf(RCI_message, "Alarm Delay sec"); // grace period between event happen till alarm is triggered, in seconds
	SetGet_param(1 + SHOW_Qfloat, 0.0f, 59.9f, &temp_N, RCI_message); // this is "delay" command, 5 chars, shows float
	SysData.NV_UI.alarm_delay_sec_f = (uint16)temp_N;
}

/************************************************************* /
uint16 SetProtocolMaxAddress() {
	uint16 MaxAdr = 255;
	if (SysData.NV_UI.StartUpProtocol == DNP3) MaxAdr = 65000; // MAX_DNP3_ADDRESS = ((uint16)65519)
	//else if (SysData.NV_UI.StartUpProtocol == MODBUS) MaxAdr = 255.0f;
	return MaxAdr;
}

/*************************************************************/
void SetGetAddress(uint8 host_0_or_meter_1)	// 0 for host, 1 for meter
{
	float temp_N;
	char* temp_Inp_str = CommStr; // pointer to rt.RxBuff[0]
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //"dnp3" or "modb" starting 1 bytes after command
	uint16 MaxAddress = 65000;
	// set max protocol address
	//if (param == (('d' + 256 * 'n') + ('p' + 256 * '3') * 65536)) {
	//	MaxAddress = 65000;
	//}
	//else
	if (param == (('m' + 256 * 'o') + ('d' + 256 * 'b') * 65536)) {
		MaxAddress = 255;
	}
	if (host_0_or_meter_1 == 0)
		temp_N = (float)SysData.NV_UI.host_address;
	else
		temp_N = SysData.NV_UI.meter_address;

	SetGet_param(4 + SHOW_LONG, 0.0f, (float)MaxAddress, &temp_N, RCI_message); // IK20260312 fixed address command - deleted offset of 2

	if (host_0_or_meter_1 == 0)
		SysData.NV_UI.host_address = (uint16)(temp_N);
	else
		SysData.NV_UI.meter_address = temp_N;
}

// -!- K20260312 - NEED DEBUGGING. expected it does not return protocol after command
// refactored Host and Meter address commands into one function with argument to specify which address,
// because they are very similar and differ only by which variable they read and write and which command string they show.
// Also, added max address limit based on protocol type specified in command argument, because DNP3 and ModBus have different address limits.
//
// DNP/ModBus Address set/get:
// 'hadr>dnp3? returns Set command syntax 'hadr>dnp3=NNNN',
// DNP range up to 65000, // ModBus range up to 255.
// Setting in flash is updated after 'save' command.
void SetGetHostAddress(void)
{
	SetGetAddress(0);	// 0 for host, 1 for meter
}

// DNP/ModBus Address set/get:
// 'madr>modb? returns Set command syntax 'madr>modb=NNNN',
// DNP range up to 65000, ModBus range up to 255.
// Setting in flash is updated after 'save' command.
void SetGetMeterAddress(void)
{
	SetGetAddress(1);	// 0 for host, 1 for meter
}

/*************************************************************/
// Get 'hbat[?]' returns Set command syntax 'hbat=XXX'. Setting in flash is updated after 'save' command.
void SetGetHighBatThreshold(void)
{
	float temp_N = SysData.NV_UI.high_bat_threshold_V_f; // saved as float, volts
	char tmp1[6] = "High";

	sprintf(RCI_message, "%s Bat Threshold, range 20 to 300 V", tmp1);
	SetGet_param(0 + SHOW_LONG, 20.0f, 300.0f, &temp_N, RCI_message);
	SysData.NV_UI.high_bat_threshold_V_f = temp_N; // save in Volts as float
}

/*************************************************************/
// Get 'lbat[?]' returns Set command syntax 'lbat=XXX'. Setting in flash is updated after 'save' command.
void SetGet_LowBatThreshold(void)
{
	float temp_N = SysData.NV_UI.low_bat_threshold_V_f; // saved as float, volts
	char tmp1[6] = "Low";

	sprintf(RCI_message, "%s Bat Threshold, range 20 to 300 V", tmp1);
	SetGet_param(0 + SHOW_LONG, 20.0f, 300.0f, &temp_N, RCI_message);
	SysData.NV_UI.low_bat_threshold_V_f = temp_N; // save in Volts as float
}

/*************************************************************/
// Get 'gflt[?]' returns Set command syntax 'gflt=XXX'. Setting in flash is updated after 'save' command.
void SetGet_NegGroundFaultThreshold(void)
{
	float temp_N = SysData.NV_UI.minus_gf_threshold_V_f; // // saved as float, volts

	sprintf(RCI_message, "+/- GndFlt Threshold, range 1 to 30 V");
	SetGet_param(0 + SHOW_LONG, 1.0f, 30.0f, &temp_N, RCI_message);
	SysData.NV_UI.minus_gf_threshold_V_f = temp_N; // saved in Volts as float
	SysData.NV_UI.plus_gf_threshold_V_f = temp_N; // saved in Volts as float
	; // saved in Volts as float
}

/*************************************************************/
// Get 'vrip[?]' returns Set command syntax 'vrip=XXX'. Setting in flash is updated after 'save' command.
void SetGetRipVoltThreshold(void)
{
	float temp_N = SysData.NV_UI.ripple_V_threshold_mV_f;   // saved in mVolts

	char tmp1[6] = "Rip V";
	char tmp2[3] = "mV";
	//CopyConstString("Rip V", tmp1);
	//CopyConstString("mV", tmp2);
	sprintf(RCI_message, "%s Threshold, range 1 to %d %s", tmp1, MAX_V_I_rip_LIMIT, tmp2);
	SetGet_param(2 + SHOW_LONG, 1.0f, (float)MAX_V_I_rip_LIMIT, &temp_N, RCI_message);
	SysData.NV_UI.ripple_V_threshold_mV_f = (uint16)temp_N; // save in mVolts
}

/*************************************************************/
// Get 'irip[?]' returns Set command syntax 'irip=XXX'. Setting in flash is updated after 'save' command.
void SetGetRipCURRthreshold(void)
{
	float temp_N = SysData.NV_UI.ripple_I_threshold_mA_f; // saved in mAmps
	char tmp1[6] = "Rip I";
	char tmp2[3] = "mA";
	sprintf(RCI_message, "%s Threshold, range 1 to %d %s", tmp1, MAX_V_I_rip_LIMIT, tmp2);
	SetGet_param(2 + SHOW_LONG, 1.0f, (float)MAX_V_I_rip_LIMIT, &temp_N, RCI_message);
	SysData.NV_UI.ripple_I_threshold_mA_f = (uint16)temp_N; // save in mAmps
}

/*************************************************************/
// shows what is saved in EEPROM. Obviously, to execute this ASCII command, acting protocol from RAM must be ASCII
// 'prot>dnp3 sets protocol in EEPROM
// 'prot? returns current protocol in EEPROM, which is used at startup. To change active protocol in RAM, use 'prot>now>' command.
// 'prot>now>dnp3' sets protocol in RAM immediately, It does not change saved protocol in EEPROM, so at next startup,
//  the protocol will be what is saved in EEPROM. This is for test purpose, to switch to DNP3 or ModBus without going through menu and without changing saved startup protocol in EEPROM.
void SetGetProtocol(void)
{
	char* temp_Inp_str = CommStr; // pointer to rt.RxBuff[0]
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //IK20260224 "now>" or "DNP3" or "SETUP"  or "ModBus" or "ASCII" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN] == '>')
	{
		uint8 change_now = 0;
		if (param == (('n' + 256 * 'o') + ('w' + 256 * '>') * 65536))
		{
			change_now = 1;
			param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 5]);
		}
		//SETUP = 0x00,
		//DNP3 = 0x01,
		//MODBUS = 0x02,
		//ASCII_CMDS = 0x03,
		//ASCII_MENU = 0x04
		// IK20260205 excluded // IK20260203 exclude 'setup' from SAVED startup protocol. FW always starts in 'Setup' protocol. Here it is for test
		//if (param == (('s' + 256 * 'e') + ('t' + 256 * 'u') * 65536)) // 'setup'
		//	SysData.NV_UI.StartUpProtocol = SETUP; // set to 0x00
		//else

		{
			uint8 ProtCode = -1; // invalid
			if (param == (('d' + 256 * 'n') + ('p' + 256 * '3') * 65536))
			{
				ProtCode = DNP3; // set to 0x01
			}
			else if (param == (('m' + 256 * 'o') + ('d' + 256 * 'b') * 65536))
			{
				ProtCode = MODBUS; // set to 0x02
			}
			else if (param == (('a' + 256 * 's') + ('c' + 256 * 'i') * 65536))
			{
				ProtCode = ASCII_CMDS;	// set to 0x03
			}
			else {
				Send_RCI_Param_Error_as_FlashConst(">now >DNP3 >ModBus >ASCII");	// IK20260205 excluded setup ("Setup DNP3 ModBus ASCII");
				return;

			}
			//IK20260302 too complicated to save into rt.operating_protocol and to SysData.NV_UI.StartUpProtocol. decided to change one setting in SysData and if needed save it to EEPROM
			SysData.NV_UI.StartUpProtocol = ProtCode;	// change active protocol in RAM immediately, without waiting for 'save' command.
			if (change_now == 0) 						// If 'now>' is not used in the command, save protocol in EEPROM, so at next startup, the protocol will be what is saved in EEPROM.
				SaveToEE(SysData.NV_UI.StartUpProtocol);
		}
		display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message and change SysData.NV_UI.StartUpProtocol
	}
	else // ? -- get command
	{
		uint8 ProtocolIndex = SysData.NV_UI.StartUpProtocol / PROTOCOL_SELECTION_INC_DEC; // 0->0; 0x11->1; 0x22->2, 0x33->3, 0x44->4)
		Put_CMD_as_chars();
		PutChar('>');
		if (ProtocolIndex < 4)
			cputs(ProtocolNames[ProtocolIndex]);
		Send_verbose_comment_as_FlashConst("StartUp protocol");
	}
}

/*************************************************************/
// exit ASCII RCI Protocol and swith to DNP immediately
void SwitchToDNP(void)
{
	cputs("Switching to DNP3\r\n");
	Delay_ms(200);
	SysData.NV_UI.StartUpProtocol = DNP3;
	display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message and change SysData.NV_UI.StartUpProtocol
}

/*************************************************************/
// exit ASCII RCI Protocol and swith to ModBus immediately
void SwitchToModBus(void)
{
	cputs("Switching to ModBus\r\n");
	Delay_ms(200);
	SysData.NV_UI.StartUpProtocol = MODBUS;
	display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message and change SysData.NV_UI.StartUpProtocol
}

/*********************************************************************/
/*              S W I T C H   T O   A S C I I   ( M O D B U S )      */
/*********************************************************************/
/*  Description:  Called from Modbus write handler when host sends a
				  "set protocol = ASCII" command via a Modbus FC06 or
				  FC16 write to the protocol-select register.
				  Mirrors SwitchToDNP() / SwitchToModBus() pattern.
	Inputs:       None
	Outputs:      SysData.NV_UI.StartUpProtocol = ASCII_CMDS
	Notes:        The Modbus parser must call this after validating the
				  register address and confirming the written value
				  equals ASCII_CMDS.
	Revisions:    IK20260302
*/
/*********************************************************************/
void SetASCIIfromModbus(void)
{
	SysData.NV_UI.StartUpProtocol = ASCII_CMDS;
	display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message and switch protocol
}

/*********************************************************************/
/*              S W I T C H   T O   D N P 3   ( M O D B U S )        */
/*********************************************************************/
/*  Description:  Called from Modbus write handler when host sends a
				  "set protocol = DNP3" command via a Modbus FC06 or
				  FC16 write to the protocol-select register.
				  Mirrors SwitchToDNP() / SwitchToModBus() pattern.
	Inputs:       None
	Outputs:      SysData.NV_UI.StartUpProtocol = DNP3
	Notes:        The Modbus parser must call this after validating the
				  register address and confirming the written value
				  equals DNP3.
	Revisions:    IK20260302
*/
/*********************************************************************/
void SetDNP3fromModbus(void)
{
	SysData.NV_UI.StartUpProtocol = DNP3;
	display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message and switch protocol
}
/*************************************************************/
// Get 'phas[e?]' returns Set command syntax 'phase>1-ph', 'phase>3-ph'. Setting in flash is updated after 'save' command.
// Set / Get one-phase (120 Hz) or 3-phase (360 Hz) charger. Calibration parameters have 2 sets for 120 or 360 Hz
void SetGetPhase(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //"3-ph" or "1-ph" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN] == '>')
	{
		if (param == (('3' + 256 * '-') + ('p' + 256 * 'h') * 65536))
			setBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);
		else if (param == (('1' + 256 * '-') + ('p' + 256 * 'h') * 65536))
			clearBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);
		else  Send_RCI_Param_Error_as_FlashConst("1-ph 3-ph");
	}
	else // ? -- get command
	{
		Put_CMD_as_chars();
		PutChar('>');
		if (SysData.NV_UI.SavedStatusWord & SinglePhase_eq0_3ph_eq1_Bit)
			cputs("3-ph");
		else cputs("1-ph");
		Send_verbose_comment_as_FlashConst("StartUp status");
	}
}

/*************************************************************/
// Get 'baud[?]' returns Set command syntax 'baud>XXXXX', reading from SysData which is the copy of EEPROM_SysData created at startup.
// Command accepts only std rates. Setting in EEPROM is updated after 'save' command.
// Setting does not change operational baud rate - or response will be at different baud rate

void SetGetBaudRate(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0]
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); // number starting 1 byte after command
	if (temp_Inp_str[CMD_LEN] == '=')
	{
		int BR = -1; // negative means invalid. Real BaudRate is 4 times bigger than BR, so BR fits in 16-bit value
		// compare 4 ASCII chars in 'param' to known baud rates
		if (param == (('1' + 256 * '1') + ('5' + 256 * '2') * 65536))		// "115200"
			BR = Baud_115200 >> 2;
		else if (param == (('1' + 256 * '9') + ('2' + 256 * '0') * 65536))	// "19200"
			BR = Baud_19200 >> 2;
		else if (param == (('9' + 256 * '6') + ('0' + 256 * '0') * 65536))	// "9600"
			BR = Baud_9600 >> 2;
		else if (param == (('4' + 256 * '8') + ('0' + 256 * '0') * 65536))	// "4800"
			BR = Baud_4800 >> 2;
		else if (param == (('2' + 256 * '4') + ('0' + 256 * '0') * 65536))	// "2400"
			BR = Baud_2400 >> 2;
		else {
			Send_RCI_Param_Error_as_FlashConst("115200 19200 9600 4800 2400");
		}
		if (BR != -1)
		{
			SysData.NV_UI.baud_rate = BR << 2;
		}
	}
	else // ? -- get command
	{
		Uint32 tmp_baud = (Uint32)SysData.NV_UI.baud_rate;
		printf("baud=%u", tmp_baud); // IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.
		Send_verbose_comment_as_FlashConst("StartUp baudrate");
	}
}

/*************************************************************/
// Current loop output Get 'iloo[p?]' returns Set command syntax 'iloop>i0-1', 'iloop>i420'. Setting in flash is updated after 'save' command.
void SetGetCurrentLoopRange(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //"i420" or "i01\0" starting 1 bytes after command
	if (temp_Inp_str[CMD_LEN] == '>')
	{
		if (param == (('i' + 256 * '4') + ('2' + 256 * '0') * 65536))
			clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
		else if (param == (('i' + 256 * '0') + ('1' + 256 * '\0') * 65536))
			setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
		else Send_RCI_Param_Error_as_FlashConst("i01 i420 only");
	}
	else {
		Put_CMD_as_chars();
		PutChar('>');
		if (SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit)
			cputs("I01");
		else cputs("I420");
		Send_verbose_comment_as_FlashConst("mA, Current loop output range");
	}
}

/*************************************************************/
// Get 'unit[?]' returns Set command syntax 'unit>XXXv'. Command accepts only 24v, 48v, 125v, 250v. Setting in flash is updated after 'save' command.
void SetGetVoltageRange(void)
{
	char* temp_Inp_str = CommStr; // pointer to RxBuff[0] or RxBuff[1] when command has preffix "`"
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]); //"024v" or "048v" or "125v" or "250v" starting 1 bytes after command
	uint8 index;
	if (temp_Inp_str[CMD_LEN] == '>')
	{
		if (param == (('0' + 256 * '2') + ('4' + 256 * 'v') * 65536)) {
			index = index24;
		}
		else if (param == (('0' + 256 * '4') + ('8' + 256 * 'v') * 65536)) {
			index = index48;
		}
		else if (param == (('1' + 256 * '2') + ('5' + 256 * 'v') * 65536)) {
			index = index125;
		}
		else if (param == (('2' + 256 * '5') + ('0' + 256 * 'v') * 65536)) {
			index = index250;
		}
		else {
			Send_RCI_Param_Error_as_FlashConst("024v 048v 125v 250v only");
			return;
		}
		SysData.NV_UI.unit_index = index;
		SysData.NV_UI.unit_type = UnitTypes[index];
		SysData.NV_UI.low_bat_threshold_V_f = Alarm_Limits[index].s.low_bat_threshold_V_f[DefSet];
		SaveToEE(SysData.NV_UI.low_bat_threshold_V_f);		// Store_Parameter(LOW_BAT, SysData.NV_UI.low_bat_threshold_V_f); //store new value

		SysData.NV_UI.high_bat_threshold_V_f = Alarm_Limits[index].s.hi_bat_threshold_V_f[DefSet];
		SaveToEE(SysData.NV_UI.high_bat_threshold_V_f);		// Store_Parameter(HIGH_BAT, SysData.NV_UI.high_bat_threshold_V_f); //store new value

		SysData.NV_UI.minus_gf_threshold_V_f = Alarm_Limits[index].s.minus_gf_threshold_V_f[DefSet];
		SaveToEE(SysData.NV_UI.minus_gf_threshold_V_f);		// Store_Parameter(MINUS_GF, SysData.NV_UI.minus_gf_threshold_V_f); //store new value

		SysData.NV_UI.plus_gf_threshold_V_f = SysData.NV_UI.minus_gf_threshold_V_f;		// IK20240206 copy plus threshold from minus when changing unit type from menu
		SaveToEE(SysData.NV_UI.plus_gf_threshold_V_f);		// Store_Parameter(PLUS_GF, SysData.NV_UI.plus_gf_threshold_V_f);				// store new value
	}
	else // ? -- get command
	{
		printf("unit>%03dv", SysData.NV_UI.unit_type); //
		Send_verbose_comment_as_FlashConst("Unit type");
	}
}
// *************************************************************
void Count_Lines(void)
{
	line_cntr++;
	SendCrLf(); // IK 190719 need here
}


/****************************************************************************** /
exports 7 calibration structures, 4 parameters each
********************************************************************************************/
void Export_calibration(void)
{
	int c;
	char* Com_Str = (char*)rt.HostRxBuff;
	cputs("//// CALIBRATION PARAMETERS\r\n");
	//-!- change max index /
	for (c = 0; c < 8; c++) //first index == calibration structure
	{
		cputs("//// ");
		cputs(CalNames[c]);
		PrintNewLine();
#if(1)  // X1 point, Y1 value, X2 point, Y2 value. shorter by 61 byte code
		int v;
		for (v = 0; v < 4; v++)  // second index == calibration coordinates
		{
			sprintf(Com_Str, "cpar%d%d", c, v);
			SetGetCalParam();  Count_Lines();
		}
#else // any order
		sprintf(Com_Str, "cpar%d0", c);  SetGetCalParam();  Count_Lines();  // X1 point
		sprintf(Com_Str, "cpar%d1", c);  SetGetCalParam();  Count_Lines();  // X2 point
		sprintf(Com_Str, "cpar%d2", c);  SetGetCalParam();  Count_Lines();  // Y1 point
		sprintf(Com_Str, "cpar%d3", c);  SetGetCalParam();  Count_Lines();  // Y2 point
#endif
	}
}

//**************************************************************
// exports parameters form the RCI list, starting from "vers" and ending by "expo"
void Export_EssentialParams(void)
{
	int c, v;
	char* Com_Str = rt.HostRxBuff;
	Uint32 Cmd_Code;
	Cmd_Code = (('v' + 256 * 'e') + ('r' + 256 * 's') * 65536);							// Convert_4_ASCII_to_Uint32("vers");
	v = 0;
	while (Cmd_Code != Convert_4_ASCII_to_Uint32((char*)(rci[v].cmd_code))) { v++; }	// if not "vers" - skip
	//cputs("//// version ");
	for (c = v + 1; c < (Num_RCI_commands); c++)										// v+1 becase skip "vers", version is printing in any case, in the beginning
	{
		//if (Convert_4_ASCII_to_Uint32((char*)(rci[c].cmd_code)) == Convert_4_ASCII_to_Uint32("expo"))
		if (Convert_4_ASCII_to_Uint32((char*)(rci[c].cmd_code)) == (('e' + 256 * 'x') + ('p' + 256 * 'o') * 65536))
			break;
		//command send as query
		ErrorStatus = NO_ERROR;															// command found = clear out error
		CMD_index = c;																	// update cmd index for messages
		*((Uint32*)rt.HostRxBuff) = *((Uint32*)rci[c].cmd_code);
		rt.HostRxBuff[CMD_LEN] = 0;														// put end of line, or it will show "gngz esse verb"
		((void(*)())(rci[c].f_ptr))();													// call function
		*((Uint32*)(Com_Str + 4)) = 0;													// clear next 4 chars

		if (ErrorStatus == NO_ERROR)
			Count_Lines();
	} //end of command search: 'for' cycle
}

/*************************************************************/
/*************************************************************
//creates play-back list of commands to restore settings
// "expo" ALL parameters without comments //
// "expo>verb"  ALL parameters with verbose response
*************************************************************/




/*************************************************************/

Uchar ParseRCI(void)
{
	int i = 0;
	Uint32 cmd_word;
	Uint32 cmd_listed;
	//register int RXbuffIndex;
	//	Uint32 cmd_listed =0x706c6568;  //"help" word reversed, p-l-e-h

	CommStr = (char*)&rt.HostRxBuff[0];
#if defined  PC
  #ifndef PC //only one check, for PC there is another check in check_ch()-->get_PC_key
	//IK20220911 was for test	TestBit_on(P04_BIT);
	if (!chrrdy()) {
		//IK20220911 was for test		TestBit_off(P04_BIT);
		return FALSE;
	}
  #else //for PC the bit in rt.Host should be set if a new char
	if (rt.Host & CharAvailableFlag)				// IK20221028 CharAvailableFlag is set in USART0_RX_interrupt
  #endif //PC
	{
		ProcessChar();								// finds backslash and removes chars from rt.HostRxBuff; finds CR, sets 'CmdAvailFlag' and replaces CR with 0
	}// enable next echo once
#else	//#if !(defined PC)

	if (rt.Host & CharAvailableFlag)
		ProcessChar();

#endif //#ifdef PC
	//test	TestBit_off(P04_BIT);

	//IK20210707 early exit
	if ((rt.Host & CmdAvailFlag) == 0)
		return FALSE; // no command yet
#ifdef PC
	SendCrLf(); // IK20220806 For PC, need extra CrLf because there is no echo from controller
#endif // PC
	// here we are when there is '\r' detected and already printed
	// if(rt.Host & CharEchoFlag) PutChar(LF);//if echo enabled

	cmd_word = Convert_4_ASCII_to_Uint32(CommStr);

	//command search start
	ErrorStatus = BAD_SIO_CMD_ERR; //prepare for worst...

	// checking if command can be executed
	//if (rt.Host & In_menu_RCI_cmd) {
	//	for (i = 0; i < (sizeof(banned_inMenu_cmds) / 4); i++)
	//	{
	//		banned_cmd = Convert_4_ASCII_to_Uint32(&banned_inMenu_cmds[1][0]);
	//		if (cmd_word == banned_cmd) {
	//			cputs("\r\n>~ERR CMD not allowed inside menu");
	//			goto ParseEnd;
	//		}
	//	}
	//}

	//command search
	for (i = 0; i < (Num_RCI_commands); i++)
	{
		cmd_listed = (*((Uint32*)rci[i].cmd_code));
		if (cmd_word == cmd_listed)				// command search
		{
			ErrorStatus = NO_ERROR;				// command found = clear out error
			rt.HostRxBuffPtr = 0;				// reset input buffer pointer ( to record and accept next command, for ex. ltf>stop )
			CMD_index = i;						// update cmd index for messages
			((void(*)())(rci[i].f_ptr))();		// call function
			break;
		}
		//			if (rci[i].cmd_code ==0) goto ParseEnd; //end of RCI command table
	} //end of command search: 'for' cycle
//ParseEnd:
	SendCrLf(); // here instead of many other places

	ClearRxBuffer(0);
	clearBit(rt.Host, CmdAvailFlag | CharAvailableFlag);

	if (ErrorStatus == NO_ERROR)
	{
		//if ((rt.OperStatusWord & Command_Executing_eq1_Bit) == 0)
			SendMsgToPC(">~OK");
		//else
		//	SendMsgToPC(">~Doing CMD");
	}
	else
	{
		if (ErrorStatus == BAD_SIO_CMD_ERR)
		{
			cputs(p_Unrecognized);
			printf(" Cmd, Error Code = %d\r\n", ErrorStatus);	// command was not recognized
		}
		else if (ErrorStatus != PARAM_ERROR)					// PARAM error was alredy printed
		{
			cputs(p_Execution);
			printf(" Cmd, Error Code = %d\r\n", ErrorStatus);	// execution
		}
	}
	wrk_str[0] = 0;
	return TRUE; // command was processed
}
/*************************************************************/
// sending upper byte first as in command from PC
void Put_CMD_as_chars(void)
{
#ifndef __cplusplus
	register
#endif
		long word = *((long*)(rci[CMD_index].cmd_code));
#ifndef __cplusplus
	register
#endif
		int t_chr = (char)word;
	PutChar(t_chr);
	t_chr = (word >> 8) & 0xFF;
	PutChar(t_chr);
	t_chr = (word >> 16) & 0xFF;
	PutChar(t_chr);
	t_chr = (word >> 24) & 0xFF;
	PutChar(t_chr);
}

/********************************/
// changes string ToLower and converts 4 chars into Uint32
Uint32 Convert_4_ASCII_to_Uint32(char* pstr)
{
	Uint32 ssss;
	ssss = ((toLower(*pstr)) + toLower(*(pstr + 1)) * 256) + 65536 * ((toLower(*(pstr + 2))) + toLower(*(pstr + 3)) * 256);
	return ssss;
}


void Export_settings(void)
{
	char* temp_Inp_str = (char*)rt.HostRxBuff;
	char Verb[5] = { 'v', 'e', 'r', 'b', 0 }; // string must end with 0 byte
	Uint32 param = Convert_4_ASCII_to_Uint32(&temp_Inp_str[CMD_LEN + 1]);	// >verb
	line_cntr = 1;
	if (strstr(temp_Inp_str, Verb) != NULL)
		setBit(rt.Host, (CharEchoFlag + CmdVerboseResponse));		// "expo>CMND verb" set verbose response for command
	else clearBit(rt.Host, (CmdVerboseResponse));

	SendCrLf();
	cputs("//// version ");
	Print_FW_Version();
	SendCrLf();														// IK 250505 need here

	//	Count_Lines();												// prints new line

//export all
	Export_EssentialParams();

	Export_calibration();

	sprintf(RCI_message, "//// Exported %d params\r\n", line_cntr);
	PutStr(RCI_message);
	if (strstr(temp_Inp_str, Verb) != NULL)							// if command verbose, clear verbose flag for shorter output of next responses
		clearBit(rt.Host, (CmdVerboseResponse));
}

void SetASCIIandSignMsg(void)
{
	Existing.BRate_index = ASCII_BR_INDX;	// override settings from SysData.NV_UI.BRate_index with ASCII baudreate
	Init_UART();							// call from main() after SETUP protocol timeout Set Baud and comm parameters
	UCSR0C = 0x06;							// no UART_parity, async, 1 stop
	Print_FW_Version();
	PrintNewLine();
}

/********************************/

//Remote Commands (first four bytes in command string), >>>lower case letters<<<<, pointer to function
const t_rci_commands rci[] =
{
#ifdef PC
	/*OK*/	"cls\r",	(void*)&ClearConsole,			// "cls" clears console in PC simulation
#endif
	// no parameters
	/*+*/	"////",		(void*)&IgnoreComment,			// "////" - comment line, IGNORES comment in downloading parameter files.
	/*+*/	"clea",		(void*)&clrscr,					// "clea" - clear screen
	/*+*/	"help",		(void*)&Print_Help,				// "help" - list of commands
	/*+*/	"?\0\0\0",	(void*)&Print_Help,				// "?" prints - list of commands
	/*+*/	"adch",		(void*)&Show_ADCcounts_and_Volts,	// "adch"  Report ADC readings in format: chan0, chan1, chan2, chan3, chan4, chan5, chan6, chan7
	/*+*/	"puls",		(void*)&Excitation_Pulse,		// Pulse set/get: 'pulse>on', 'pulse>off' - set right now; 'pulse>enab', IT DOES NOT SAVED into flash because it is test command.
	/*+*/	"dnp3",		(void*)&SwitchToDNP,			// exit ASCII RCI SysData.NV_UI.StartUpProtocol and swith to DNP immediately
	/*+*/	"init",		(void*)&InitATMEL,				// "init": Re-initialize controller, like power-up
	/*+*/	"modb",		(void*)&SwitchToModBus,			// exit ASCII RCI SysData.NV_UI.StartUpProtocol and swith to ModBus immediately
	/*+*/	"s\0\0\0",	(void*)&Print_Status,			// "s"  Print status BYTE (see "stat" command)
	/*+*/	"w\0\0\0",	(void*)&Print_System_Snapshot,	// "w": Print system information (same as "info" command)
	/*+*/	"b\0\0\0",	(void*)&Print_System_Brief,		// "b": Print brief information: measurements and alarms
	/*+*/	"brif",		(void*)&Print_System_Brief,		// "brif": Print brief information: measurements and alarms
	/*+*/	"d\0\0\0",	(void*)&SendRTdata,				// "d": Print measurements and alarm word
	/*+*/	"data",		(void*)&SendRTdata,				// "data": Print measurements and alarm word
	/*+*/	"disn",		(void*)&Send_to_NumericLEDs,	// "disN"  send string to upper Numeric LED display
	/*+*/	"disi",		(void*)&Send_to_ASCII_LEDs,		// "disI"  send string to lower Information ASCII LED display
	/*+*/	"disl",		(void*)&Send_to_STATUS_LEDs,	// "disL"  send LED byte. if upper bit of OverwriteLEDs is set, it overwrites (ON-OFF assigned by Display board firmware) with 3 lower bits of OverwriteLEDs
	/*+*/	"getb",		(void*)&Get_Buttons,			// "getB"  ask for uint16 button word and show it on screen.
	/*+*/	"rsto",		(void*)&Get_EEPROM_params,		// restore: GET SYSTEM PARAMETERS FROM FLASH if data is valid
	/*+*/	"dflt",		(void*)&SetDefaultsInRAM_by_command,// "dflt"(default)- sets default settings, but preserves serial port settings
	/*+*/	"rtmo",		(void*)&SetRealTimeMonitoring,	//  'rtmon>enab', 'rtmon>disa'.Enables / diasbles real time monitoring - send once in a few sec "brief" info to serial port

	//DO NOT MOVE "vers" command from here, it is used to start full parameter dump - "expo" command,
	// PUT ALL PARAMETER COMMANDS below before "expo" command
	/*+*/	"vers" ,	(void*)&Print_FW_Version,
	/*+*/	"hadr",		(void*)&SetGetHostAddress,		// DNP/ModBus Host Address set/get: 'hadr? returns Set command syntax 'hadr=NNNN', DNP range up to 65000, ModBus pange up to 255.  Setting in flash is updated after 'save' command.
	/*+*/	"madr",		(void*)&SetGetMeterAddress,		// DNP/ModBus Meter Address set/get: 'madr? returns Set command syntax 'madr=NNNN', DNP range up to 65000, ModBus pange up to 255.  Setting in flash is updated after 'save' command.
	/*+*/	"prot",		(void*)&SetGetProtocol,			// Protocol set/get: DNP or ModBus after startup
	/*+*/	"baud",		(void*)&SetGetBaudRate,			// Get 'baud[?]' returns Set command syntax 'baud>XXXXX'. Command accepts only std rates. Setting in flash is updated after 'save' command.
	/*+*/	"unit",		(void*)&SetGetVoltageRange,		// Get 'unit[?]' returns Set command syntax 'unit>XXXv'. Command accepts only 24v, 48v, 125v, 250v. Setting in flash is updated after 'save' command.
	/*+*/	"hbat",		(void*)&SetGetHighBatThreshold,	// Get 'hbat[?]' returns Set command syntax 'hbat=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"lbat",		(void*)&SetGet_LowBatThreshold,	// Get 'lbat[?]' returns Set command syntax 'lbat=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"vrip",		(void*)&SetGetRipVoltThreshold,	// Get 'vrip[?]' returns Set command syntax 'vrip=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"irip",		(void*)&SetGetRipCURRthreshold,	// Get 'irip[?]' returns Set command syntax 'irip=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"iloo",		(void*)&SetGetCurrentLoopRange,	// Current loop output Get 'iloo[p?]' returns Set command syntax 'iloop>i0-1', 'iloop>i420'. Setting in flash is updated after 'save' command.
	/*+*/	"phas",		(void*)&SetGetPhase,			// Get 'phas[e?]' returns Set command syntax 'phase>1ph', 'phase>3ph'. Setting in flash is updated after 'save' command.
														// Set / Get one-phase (120 Hz) or 3-phase (360 Hz) charger. Calibration parameters have 2 sets for 120 or 360 Hz
	/*+*/	"alar",		(void*)&SetGetAlarm,			// Alarm set/get: 'alarm rv>enab', 'alarm rv>disa'; 'alarm ri>enab', 'alarm ri>disa'; 'alarm ac>enab', 'alarm ac>disa'; 'alarm hz>enab', 'alarm hz>disa'
	/*+*/	"dela",		(void*)&SetGetDelay,			// Delay set/get: 'delay? returns Set command syntax 'delay=XX.X' in seconds. Setting in flash is updated after 'save' command.
	/*+*/	"buzz",		(void*)&SetGetBuzzer,			// Buzzer set/get: 'buzz>on', 'buzz>off' - set right now; 'buzz>enab', 'buzz>disa' - set/get setting in flash after 'save' command.
	/*+*/	"latc",		(void*)&SetGetLatch,			// Get 'latc[h?]' returns Set command syntax 'latch>enab', 'latch>disa'. Setting in flash is updated after 'save' command.

	/*+*/	"echo",		(void*)&Echo_Enab_Disab,		// "echo>enabled"  "echo>disabled" sets echo flag. "echo?" returns flag
	/*+*/	"expo",		(void*)&Export_settings,		// "expo" //creates play-back list of commands to restore settings
	///////"expo" ends auto param list

	/*+*/	"cpar",		(void*)&SetGetCalParam,			// "cpar#$=GGG.GGG" Set/get calibration parameter.
	/*+*/	"cali",		(void*)&SetCalibrationValue,	// "cali#$=GGG.GGG" Set Calibration value.
	/*+*/	"save",		(void*)&SaveCal,				// "save"  transfers data from RAM to FLASH. Use "SAVE" to permanently update it in FLASH
};

/*************************************************************/
int NumOfCommands(void)
{
	return sizeof(rci) / sizeof(t_rci_commands);
}
