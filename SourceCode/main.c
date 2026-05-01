/********************************************************************/
/*                        COPYRIGHT NOTICE                          */
/*           Copyright (s) 2025 Electroswitch Corporation           */
/*                        180 King Avenue                           */
/* Unauthorized use or duplication of this software is prohibited   */
/********************************************************************/
// device     Flash  EEPROM  RAM   case   maxGPIO
// ATmega644   64K   2K      4K   TQFP44  32
// Xtal clock  16 MHz
/********************************************************************/
/* Filename:    826-501A.C   Rev J                                  */
/* Rev IK @ 05-Dec-2023         & I.K.                              */
/* Description: This software implements the control firmware       */
/*              for the ARGA battery monitor.                       */
/*                                                                  */
/*              ANA_ALARM_IN   -- PA0                               */
/*              NOT USED       -- PA1                               */
/*              NOT USED       -- PA2                               */
/*              XMT/RCV_LED_2  -- PA3                               */
/*              LIMIT          -- PA4                               */
/*              ALARMS_IN      -- PA5                               */
/*              NOT USED       -- PA6                               */
/*              ALARM_IND      -- PA7                               */
/*              PULSE          -- PB0                               */
/*              CUR_SOURCE_OUT -- PB1                               */
/*              SCAN           -- PB2                               */
/*              BB_PULSE_OUT   -- PB3                               */
/*              XMT_RCV_LED3   -- PB4                               */
/*              MOSI           -- PB5                               */
/*              MISO           -- PB6                               */
/*              SCK            -- PB7                               */
/*              SCL            -- PC0                               */
/*              SDA            -- PC1                               */
/*              A2             -- PC2                               */
/*              A3             -- PC3                               */
/*              A4             -- PC4                               */
/*              A5             -- PC5                               */
/*              NOT USED       -- PC6                               */
/*              NOT USED       -- PC7                               */
/*              RS485 XMIT/RCV -- PD0                               */
/*              RS485 XMIT/RCV -- PD1                               */
/*              XMT_ENABLE     -- PD2                               */
/*              A_MUX          -- PD3                               */
/*              B_MUX          -- PD4                               */
/*              C_MUX          -- PD5                               */
/*              MANUAL_AUTO    -- PD6                               */
/*              XMT/RCV_LED_1  -- PD7                               */
/********************************************************************/

#define FW_VERSION   (uint8)30		// increment with new release

//#define LAST_GASP
#ifdef LAST_GASP
  #define EXTRA_DNP_BYTE  1
char FW_PartNumber[] = "826-509-A";	// IK20230706 must be in RAM and not the char* for printf
#else
  #define EXTRA_DNP_BYTE  0
char FW_PartNumber[] = "826-501-A";	// IK20230706 must be in RAM and not the char* for printf
#endif //#ifdef LAST_GASP

char FW_Date[] = "01-May-2026";		// IK20230706 must be in RAM for printf
#define FW_ver_float ((float)(FW_VERSION) + 0.01f)/10.0f
float FirmwareVersion;				// to be shown as float on LED, "3.0", initialization to 'FW_ver_float' in init()

#ifdef INCLUDE_DAY_TIME_INTO_FW // this includes "comp @ __DATE__ __TIME__" but in Vsual Studio just-in-time compile will not work
// IK20230706 char FL* CompileDate = __DATE__; // this places pointer and the __TIME__ string in FLASH but printf_P requires string pointers and arg list to point into RAM
char CompileDate[] = __DATE__; // this places pointer and the __TIME__ string in RAM
char CompileTime[] = __TIME__;
#endif // INCLUDE_DAY_TIME_INTO_FW

/*----------------------  Compiler Pragma's ------------------------*/
// enable use of extended keywords
//#ifdef PC
//#ifdef printf
//#undef printf
//#endif
//#define printf PRINTF
//#endif
#ifndef PC	// PC does not have this #pragma
#pragma language=extended
#endif
//#define INTERRUPT_TIME_TESTING //IK 20240328 used to refactor timimg of interrupt TIMER2_COMPA_interrupt
//#define WAIT_FOR_ADC_in_Measure

#define TIME_TESTING //IK 20250523 added wire to pin 44 (easy solder) which is Port B, bit 4
#define SetTestPin44       setBit(PORTB, Bit_4);    // IK20250523 set test pin #44 (easy to solder to)
#define ClearTestPin44   clearBit(PORTB, Bit_4);    // IK20250523 set test pin #44 (easy to solder to)

#ifdef PC
//  #define ASCII_TESTING // define for direct overwrite and booting into ASCII mode
#endif



#define PHASE_DEBUG     // T/R LED shows 1ph as green, 3-ph as red when in calibration mode

/*----------------------  INCLUDE FILES  ---------------------------*/

#define STRINGS_IN_FLASH // TO WORK, THIS OPTION MUST BE DEFINED in project options->compiler->extra options! --string_literals_in_flash   to place strings in flash
//#define TEST_PRINTF

// #define USE_PRINTF // and derivatives, like sprintf
//  Porting From IAR to AVR GCC
//  https://www.nongnu.org/avr-libc/user-manual/porting.html
#if defined(__ICCAVR__) // IAR C Compiler
#define FLASH_DECLARE(x) __flash x
#endif
#if defined(__GNUC__) // GNU Compiler
#define FLASH_DECLARE(x) x __attribute__((__progmem__))
#endif

#ifndef PC
  #define INTERRUPT __interrupt
  #include <iom644.h>
  #include <ina90.h>
  #include <ctype.h>
  #include <pgmspace.h>
  #include <math.h>
  #include <string.h>
  #include <stdlib.h>
//  #include <stdio.h>
#else // for PC, #ifdef PC
  //#include <cstdio>
  #define INTERRUPT
  #include <corecrt_math.h> // for isnan() lib function
  typedef int                 BOOL;
  #define PutChar PUTCHAR
  #define cputs    CPUTS
  //Int16 CPUTS(const char* p);
  #define CPUTS PutStr
  #define FL
  #define FLP
  long Sim_counter; // free-running increment counter for simulation and GUI update
  BOOL IsFileOutput = false;
  extern int new_thread();

  #define EEsimulator
  extern void D_PutChar(unsigned char c);
  extern void  D_CleanLCD_line(int line); // line is only 1 or 2
#endif // for PC

#include "GLOBALS.H"
#include "Hardware.h"
#include "ASCII_commands.h"

struct twi_variables twi_send;

// default values

#define DEF_BAT_MONITOR_ADDR	2			// default Battery Monitor RS-485 address
#define DEF_HOST_ADDR			3			// default host address
#define DEF_MODBUS_FIRST_REG	0			// default Modbus first reg
#define DEF_INTER_CHAR			1041		// default inter char gap in micro s // ms
#define DEF_DLL_TIMEOUT			1000		// default dll timeout in milli S // ms
#define DEF_NUM_OF_POINTS		5			// IK20260205 CHANGED from 0x55 to regular 0x05  default number of input/analog points // 55 is coding for 5 channels
#define DEF_PROTOCOL			DNP3		// IK20251230 CHANGED to regular number  default SysData.NV_UI.StartUpProtocol
#define DEF_DLL_NUM_OF_RETRIES	10			// default number of dnp DLL retries
#define DEF_DLL_CONFIRM			0			// default status of DLL layer confirms
#define DEF_APP_STATUS			0			// default status of APP layer confirms and UART_parity (shared data)


#ifdef UNI_BI_POLAR_INPUTS
#define DEF_CHANNEL_ONE			BIPOLAR		// default channel 1 type
#define DEF_CHANNEL_TWO			BIPOLAR		// default channel 2 type
#define DEF_CHANNEL_THREE		BIPOLAR		// default channel 3 type
#define DEF_CHANNEL_FOUR		BIPOLAR		// default channel 4 type
#define DEF_CHANNEL_FIVE		BIPOLAR		// default channel 5 type
#endif // #ifdef UNI_BI_POLAR_INPUTS

#define DEF_XMT_DELAY			30			// default xmt delay(response delay)  // ms
#define DEF_BAUD_RATE			Baud_19200	// default baud rate
#define DEF_BAUD_RATE_INDEX		Baud_19200_i	// default baud rate
#define UCSR0A_SETTINGS			0x22		// default set a RX reset (0x20) and double speed mode (0x02) flags, to get 2.4% accuracy at 115200 baud
// Bit 5 - UDREn: USART Data Register Empty
// The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn
// is one, the buffer is empty, and therefore ready to be written. The UDREn Flag can generate a
// Data Register Empty interrupt (see description of the UDRIEn bit).
// UDREn is set after a reset to indicate that the Transmitter is ready.
//  Bit 1 - U2Xn : Double the USART Transmission Speed
// This bit only has effect for the asynchronous operation.Write this bit to zero when using synchronous
// operation.
// Writing this bit to one will reduce the divisor of the baud rate divider from 16 to 8 effectively doubling
// the transfer rate for asynchronous communication.

// IK20250623 default values moved to Globals.h
//
// IK20250728 created initialization union containing structure, access
// Alarm_Limits[0].s.low_bat_threshold_V_f[MIN];
// Alarm_Limits[2].AlarmCriteria[PLUS_GF][MAX];
// values are saved in floats
const union uAlarm_criteria Alarm_Limits[] = {
	{.s = { // index24
		{MIN_24V_low_bat_threshold,  DEF_24V_low_bat_threshold,  MAX_24V_low_bat_threshold},
		{MIN_24V_high_bat_threshold, DEF_24V_high_bat_threshold, MAX_24V_high_bat_threshold},
		{MIN_24V_minus_gf_threshold, DEF_24V_minus_gf_threshold, MAX_24V_minus_gf_threshold},
		{MIN_24V_plus_gf_threshold,  DEF_24V_plus_gf_threshold,  MAX_24V_plus_gf_threshold},
	}},
	{.s = { // index48
		{MIN_48V_low_bat_threshold,  DEF_48V_low_bat_threshold,  MAX_48V_low_bat_threshold},
		{MIN_48V_high_bat_threshold, DEF_48V_high_bat_threshold, MAX_48V_high_bat_threshold},
		{MIN_48V_minus_gf_threshold, DEF_48V_minus_gf_threshold, MAX_48V_minus_gf_threshold},
		{MIN_48V_plus_gf_threshold,  DEF_48V_plus_gf_threshold,  MAX_48V_plus_gf_threshold},
	}},
	{.s = { // index125
		{MIN_125V_low_bat_threshold,  DEF_125V_low_bat_threshold,  MAX_125V_low_bat_threshold},
		{MIN_125V_high_bat_threshold, DEF_125V_high_bat_threshold, MAX_125V_high_bat_threshold},
		{MIN_125V_minus_gf_threshold, DEF_125V_minus_gf_threshold, MAX_125V_minus_gf_threshold},
		{MIN_125V_plus_gf_threshold,  DEF_125V_plus_gf_threshold,  MAX_125V_plus_gf_threshold},
	}},
	{.s = { // index250
		{MIN_250V_low_bat_threshold,  DEF_250V_low_bat_threshold,  MAX_250V_low_bat_threshold},
		{MIN_250V_high_bat_threshold, DEF_250V_high_bat_threshold, MAX_250V_high_bat_threshold},
		{MIN_250V_minus_gf_threshold, DEF_250V_minus_gf_threshold, MAX_250V_minus_gf_threshold},
		{MIN_250V_plus_gf_threshold,  DEF_250V_plus_gf_threshold,  MAX_250V_plus_gf_threshold},
	}}
};


#ifndef PC
  #define PutChar Send_Char
  #ifndef STRINGS_IN_FLASH
	#define printf PRINTF // equivalent of printf, with customized output to UART using cputs and char wrk_str[] buffer, specific for RS-485 chip which needs to enable/disable transmitter or receiver
	#define sprintf SPRINTF // equivalent of sprintf,
	#define cputs    PutStr
	#define FL const
	#define FLP const *
  #else
	#define printf Print_F // equivalent of printf, with customized output to UART using cputs and char wrk_str[] buffer, specific for RS-485 chip which needs to enable/disable transmitter or receiver
	#define sprintf SPRINT_F // equivalent of sprintf when format string is placed in flash
	#define cputs    PrintConstString
	#define FL   __flash
	#define FLP  __flash *
// IK 20230706 instead, use linker option --redirect putchar=Send_Char
//	#ifdef putchar
//	    #undef putchar
//	    #define putchar  Send_Char
//	#endif
//	#ifdef C_PUTCHAR
//	    #undef C_PUTCHAR
//	    #define C_PUTCHAR  Send_Char
//	#endif
  #endif
#else // PC
#endif // PC

const char FL* FiveSpaces = "     "; // used in display blinking

/*-------------------------- variables  ----------------------------*/
//---- General System variables
DisplayInfo Display_Info;
char printf_buff[0x40];						// [64] to copy flash strings into RAM
uint8  tmp_byte;							// general working register
uint8  ADC_Status;							// what operation is the ADC in
uint8  measurement_ID;						// what measurement is being made
uint8  calibr_step;							// what are you calibrating
#ifdef TIME_TESTING							// IK 20250523 added wire to pin 44 (easy solder) which is Port B, bit 4
 //Boolean Timer0_finished;
 //uint8  time_testing;						// test
#endif // TIME_TESTING

uint8  debug = 0;							// IK20250522 ! FOR TEST, debug is not explicitly initialized, zero! delay 0.5 ms before switching to transmit mode

uint8  relay_board_status;					// updated via TWI when read RelayBoard, holds relay board status, AC power loss bit
uint8  RxGreen_TxRed_LED_op;				// determines whether the comm LED is red or green
uint8  tmp_display_status;					// used for detection if Phase or bbpulse selection changed
uint8  io_update;							// need to update IO in Relay board
uint8  restart_op;							// used to restart system using WDT
uint16 old_alarm_status;					// detects new alarm by comparing with this variable to generate event

uint8  send_type;							// state machine in main(),the data type is sent via TWI, alarms or comm params
uint8  I420_calibr_lock;					// Locks (prevents) saving to EEPROM. Calibration Timer is simultaneously set to 5000 ms and starts counting down. After 3000 ms when timer reches below 2000 ms, auto firmware does saving to EEprom
extern uint16 DNP_time_delay;						// sets to 9000, and then shows in case
//---TWI Variables
u_TWI twi;									// IK20250604 union of array and structure instead of unaggregated vars


/*
in DNP3 protocol, the "DLL" stands for "Data Link Layer"
the "IIEN" = Internal Indication Events Notification
These are non-standard, implementation-specific abbreviations, but they commonly refer to:
iien1 and iien2 typically represent bitfields or flags inside DNP3 objects or event class structures.
They often map to DNP3 internal indication bits(e.g., device trouble, restart, etc.) stored in IIN(Internal Indications) bytes.

In the official DNP3 spec, this is more formally called :
IIN Field(Internal Indications Field) :
	A 16 - bit field used in the response to indicate internal status of the outstation.
	Typical flags include :

IIN1.NeedTime
IIN1.DeviceTrouble
IIN2.ConfigCorrupt

So :
iien1 = IIN byte 1
iien2 = IIN byte 2
*/

uint16 crc;									// IK20250812 changed from volatile and encapsulated calculations in _disable - _enable_interrupt. holds locally computed crc

volatile uint8  cal_status;					//-!- IK20260422 no need volatile? used in DNP calibration, indicates what is being calibrated

//---- Meter Variables

//---- Communication variables
// IK20231206 changing order of global variables (arrays before variables as an example) creates a different HEX file, but file size is the same!
uint8 wrk_str[HOST_XMT_BUFF_LEN];			// building & sending msg
uint8 uart_byte;							// holds uart byte, used inside UART interrupt only
volatile uint8 msg_status;					//-!- holds status of any rcvd msgs: MSG_DONE, or MSG_STARTED; MSG_ARRIVED is set but not checked
volatile uint8 num_of_inbytes;				// number of bytes in buffer
volatile uint8 comm_state;				// works with enum UART_Events

uint8  parity_error;
uint8  send_modbus;							// holds what Modbus message to send out, from enum Modbus_Send_Type
uint8  send_setup;							// holds what setup message to send out, from enum Setup_Send_Type
//---- IRIG Variables


/*----------- word definitions -----------*/

//---- Meter variables moved into SysData structure
uint16 RVV_ADC_counts;				//-!- IK20250226 CAN BE REMOVED was not used, now for diagnostic
uint16  ADC_counts;  //-!- IK20250226 added for test

/*----------- Float & Long definitions -----------*/
//---- Meter variables
float filtered_ripple_v_f;
float last_ripple_v_f;
float filtered_ripple_i_f;
float last_ripple_i_f;
float debug_f1;
float debug_f2;

uint8 muxA; // contains mirrored setting of MUXA port            0x08
uint8 muxB; // contains mirrored setting of MUXB port            0x10
uint8 muxC; // contains mirrored setting of MUXC port            0x40

uint16 const Baud_Rates[Last_Baud_Index] = // IK20250724 added /4 (>>2) to keep values inside uint8 range
// IK20250826 changed from uint8 to uint16 to accomodate 115200 >> 2 = 28800
{
	Baud_300 >> 2, Baud_600 >> 2, Baud_1200 >> 2, Baud_1800 >> 2,
	Baud_2400 >> 2,	Baud_3600 >> 2, Baud_4800 >> 2, Baud_9600 >> 2,
	Baud_14400 >> 2, Baud_19200 >> 2, Baud_115200 >> 2
};

uint8 const UnitTypes[4] = {
	UNIT_24V,  // = 24
	UNIT_48V,  // = 48
	UNIT_125V, // = 125
	UNIT_250V, // = 250
};

// Small or capital letters - does not matter, get converted into lower case
const char FL * ProtocolNames[] =
{
	"Setup",
	"DNP3",
	"ModBus",
	"ASCII"
};

/*------------------------- Structures -----------------------------*/

#ifndef PC
SYS_SPECIFIC_DATA DefaultSysdata;
__no_init SYS_SPECIFIC_DATA SysData @ 0x1000;
__eeprom  SYS_SPECIFIC_DATA EEPROM_SysData @ EE_SYS_DATA_OFFSET;
#else
SYS_SPECIFIC_DATA DefaultSysdata;
SYS_SPECIFIC_DATA SysData;
SYS_SPECIFIC_DATA EEPROM_SysData; // located in EEPROM @ EE_SYS_DATA_OFFSET, @ 0x100
#endif

RealTimeVars rt;
SettingsStruct Existing;					// keep current setting to detect change vs SysData.NV_UI
TIMERS timer;

Uint32 ErrorStatus = sizeof(SysData);	// error word, used for test here, it is calculated in bytes
uint16 timeDelta;
/*-------------------------- prototypes ----------------------------*/
//NONE
//void
//Store_Event(uint8,uint16,uint16, uint16);
//void PrintMask(char FL* cmd, uint8 inVal);
void Print_F(char FL* fmt, ...);
int SPRINT_F(char * OutBuf, char FL* fmt, ...);
uint16 PutStr(const char* StrPtr);
void Init_Parameters(void);
void Export_ToFile(void* f_ptr);

// Remote Command Interface (ASCII protocol) functions
extern const t_rci_commands rci[];
void Put_CMD_as_chars(void);
void Echo_Enab_Disab(void);
Uint32 Convert_4_ASCII_to_Uint32(char* pstr);

#ifdef PC
extern FILE* g_fhOutput ;
// Add this to the top of your file, after other #includes if not already present

//extern int putch(int);
int putch(int c)
{
	putchar(c);
	return c;
}
/// <summary>
/// adds percentage of noise to the input value
/// </summary>
/// <param name="InVal"></param>
/// <param name="NSR"> Noise to Signal Ratio. 0.1 adds 10% of noise</param>
/// <returns>InVal + Noise</returns>
double AddNoise(double InVal, double NSR)
{
	// NSR should be 0.10 for +/-5% noise (i.e., 10% peak-to-peak)
	double noise = ((double)rand() / RAND_MAX - 0.5) * NSR;
	return InVal * (1.0 + noise);
}

#endif // PC

/// <summary>
/// Intend to set baud rate for USART communication temporary while changing protocols, or directly by a command.
/// if protocol get changed, the baud rate is set from SysData.NV_UI settings.
/// Sets the baud rate for USART communication based on the provided baud rate index.
/// Updates UART baud rate register, save the settings in the Existing structure,
/// but DOES NOT save the settings in the SysData.NV_UI structure.
/// </summary>
/// <param name="BR_index">The index into the baud rate table. If the index is out of range, defaults to 19200 baud (Baud_19200_i).</param>
/// <returns>The actual baud rate value that was set, calculated by shifting the table value left by 2 bits.</returns>
Uint32 Set_BaudRate(uint8 BR_index)
{
	Uint32 BaudRate;
	if (BR_index >= Last_Baud_Index) {
		BR_index = Baud_19200_i; // default
	}
	BaudRate = (Uint32)Baud_Rates[BR_index] << 2;	// IK20250724 restore full baud rate value by shifting left 2
	Existing.baud_rate = BaudRate;
	Existing.BRate_index = BR_index;
	Set_USART_UBBRregister(BaudRate);
	return BaudRate;
}

/// <summary>
/// IK20260114 function saves in the SysData.NV_UI and in EEPROM
/// a new BR_index and baud rate based on Baud_Rates[BR_index] array
/// if index is outside the range, it is set to default Baud_19200_i
/// </summary>
/// <param name="BR_index">index = enum of Baud_Rate_index, between 0 and Last_Baud_Index</param>
void Save_BaudRate_To_EEPROM(uint8 BR_index)
{
	Uint32 BaudRate = (Uint32)Baud_Rates[BR_index] << 2;	// IK20250724 restore full baud rate value by shifting left 2

	SysData.NV_UI.baud_rate = BaudRate;
	SaveToEE(SysData.NV_UI.baud_rate);				// Store the new baud rate in EEPROM
	SysData.NV_UI.BRate_index = BR_index;			//
	SaveToEE(SysData.NV_UI.BRate_index);			// Store the new baud rate index in EEPROM
}

/// <summary>
/// Finds the index of a baud rate value in the baud rate lookup table.
/// </summary>
/// <param name="BaudRateVal">The baud rate value to search for (expected to be left-shifted by 2 positions).</param>
/// <returns>The index of the matching baud rate in the Baud_Rates[] array, or -1 if not found.</returns>
signed char Find_BR_index(Uint32 BaudRateVal) {
	uint8 i;
	for (i = 0; i < Last_Baud_Index; i++)
	{
		if (((Uint32)Baud_Rates[i] << 2) == BaudRateVal)
			return i;
	}
	return (signed char)-1;
}

/// <summary>
/// Sets the USART baud rate register (UBRR) based on the provided baud rate value.
/// Baud rate values and their corresponding UBRR register settings for a 16 MHz clock are approximately:
/// baudrate    stdmodeUBRR (error %)| double speed UBBR (error %)
/// 9600        103=0x67    (+0.16%) | 207=0xCF          (+0.16%)
/// 14400        68=0x44    (+0.64%) | 138=0x8A)         (-0.08%)
/// 19200        51=0x33    (+0.16%) | 103=0x67)         (+0.16%)
/// 115200        8=0x08    (-3.55%) |  16=0x10)         (+2.12%)
/// exact value is 7.68, 0x7 has +8.5% error, but with rounding to nearest integer, it is 7 (0x07) with 3.5% error;
/// the function rounds to nearest integer, so it gives 8, with -3.55%
/// </summary>
/// <param name="BaudRate">The desired baud rate value.</param>
/// <returns>The calculated UBRR register value for the given baud rate.</returns>
uint16 Calculate_USART_UBRRregister(Uint32 BaudRate) // IK20250206
{
	// 	float RegSetting = ((MASTER_CLOCK / 16) / BaudRate) - 1;	// std mode
	// 	float RegSetting = ((MASTER_CLOCK / 8) / BaudRate) - 1;		// dbl speed
	float RegSetting = ((float)(MASTER_CLOCK >> 3) / BaudRate - 0.5f);	// this rounds to nearest integer, for 115200 it gives 7.68 -> 8, error= -3.5% (real BR = 111111)
	return (Uint32)RegSetting;
}

// IK20251217 sets baud rate register and creates a mirror of setting
void Set_USART_UBBRregister(Uint32 BaudRate) // IK20250206
{
	rt.UBRR0_setting = Calculate_USART_UBRRregister(BaudRate);
	if (UBRR0 != rt.UBRR0_setting) // overwrite if setting is different
		UBRR0 = rt.UBRR0_setting;
}

// IK 20230614 extracted to a function to recover TWI if it is timed out
void RecoverTWI(void){
	TWCR |= 0x90;                          //recover twi bus
	_NOP();                                //wait for prop delays
	_NOP();
	_NOP();
	_NOP();
	//twi_msg = WAITING;                     //set back to get ready for next
}

/* Momentarely switch from receiving to trasmitting to send char
 and switch back to receiving */
void Send_Char(uint8 Tx_char) {
#ifndef PC
	uint8 c;
//#ifdef TIME_TESTING
//	SetTestPin44;    // IK20250523 set test pin #44 (easy to solder to)
//#endif // #ifdef TIME_TESTING

	timer.UART_delay_100us = debug; // IK20250522 delay 0.5 ms before switching to transmit mode
	do {                                        // waiting after char received before switching to transmit mode
		_NOP();                                 //
	} while (timer.UART_delay_100us != 0);      // wait for channel to free

	setBit(XMT_ENABLE, XMT_ON);                 // PORTB connected to RS485 chip to
	clearBit(UCSR0B, RXEN);                     // disable RS232 RCV
	setBit(XMT_ENABLE, XMT_ON);                 // RS485 chip to xmt
	for (c=0; c<16; c++) {
		_NOP();
	}
// cannot use  Delay_ms(2); because it is called from interrupt //add delay before start transmitting?
	__disable_interrupt();
	setBit(UCSR0A, TXC);                         // write to UCSRA the TXC<=>0x40 bit to clear transmitter
	// The TXC Flag can be used to check that the Transmitter has completed all transfers,
	// must be cleared before each transmission (before UDR is written) if it is used for this purpose
	UDR0 = Tx_char;                              // put char in UART buffer
//#ifdef TIME_TESTING
//	ClearTestPin44;    // IK20250523 set test pin #44 (easy to solder to)
//#endif // #ifdef TIME_TESTING

	__enable_interrupt();
#ifndef PC
	do {
	   _NOP();                                   // kill a little time
	}  while ((UCSR0A & TXC) != TXC);            // wait till transmission done TXC <=> Bit_6, USART Transmit Complete
#endif
//    _NOP();                                    // kill a little time
//    UCSRB = UCSRB &~TXEN;                      // DOES NOT WORK:    assembly CBI 0x0A,3; after echoing first char - nothing
	clearBit(XMT_ENABLE,XMT_ON);                 // WORKS xmt off     assembly CBI 0x18,4 resetting PD4 to zero
	setBit(UCSR0B, RXEN);                        // enable RS232 RCV; assembly SBI 0x0A,4 setting UCSRB, bit 4
#else
	putch(Tx_char);
#endif
}

/***************************************************************************** /
* // not used
void USART_Flush( void )
{
	uint8 dummy;
	while ( UCSR0A & RXC) debug = UDR0;
} */

/****************************************************************************
IK 2023-Jun-12           C U S T O M   cputs() F U N C T I O N
*****************************************************************************
 specific for RS-485 chip which needs to enable/disable transmitter or receiver
The output itself is performed by std cputs() which is defined as PutStr()

this function is waiting while string is beeing transmitted via UART!
Thus, the time it takes depend on baudrate and string length, but interrupts are enabled
USE THIS FUNCTION FOR STRINGS IN RAM, USE cputs() FOR STRINGS IN FLASH
*****************************************************************************/
uint16 PutStr(const char  *  Str)
{
	uint16 chr_count = 0;
/**/
	timer.UART_delay_100us = debug; // IK20250522 !FOR TEST, debug is not explicitly initialized, zero! delay 0.5 ms before switching to transmit mode
	do {                                        // waiting after char received before switching to transmit mode
		_NOP();                                 //
	} while (timer.UART_delay_100us != 0);      // wait for channel to free

	setBit(XMT_ENABLE, XMT_ON);                 // XMT_ENABLE is PORTB
	clearBit(UCSR0B, RXEN);                     // RS485 chip to disable RS232 RCV  RXEN
	setBit(XMT_ENABLE, XMT_ON);                 // RS485 chip to xmt
	_NOP();
/*  The Transmit Complete (TXC <=> Bit_6) Flag bit is set one when the entire frame in the transmit Shift
Register has been shifted out and there are no new data currently present in the transmit buffer.
The TXC Flag bit is automatically cleared when a transmit complete interrupt is executed, or it
can be cleared by writing a one to its bit location. The TXC Flag is useful in half-duplex
communication interfaces (like the RS485 standard), where a transmitting application must enter
receive mode and free the communication bus immediately after completing the transmission.
*/
	Delay_ms(2); //add delay before start transmitting?
// The USART transmitter has two flags that indicate its state: USART Data Register Empty
// (UDRE) and Transmit Complete (TXC). Both flags can be used for generating interrupts.
#ifndef PC
	while(*Str != 0)
	{
		__disable_interrupt();
		UCSR0A |= TXC;                               // write to TXC to clear it, for checking complete transmission
		UDR0 = *Str;                                 // send char
		__enable_interrupt();
	  Str++;
	  chr_count++;
		do {
			_NOP();                                     // kill a little time
		} while ((UCSR0A & TXC) != TXC);            // wait till transmission done TXC <=> Bit_6, USART Transmit Complete
		WATCHDOG_RESET();
	}
#else // PC
	chr_count = strlen(Str);
	if(IsFileOutput)
	{
		if(g_fhOutput != NULL) fprintf(g_fhOutput, "%s", Str);
	} // set flag only for PC to write to file
	while (*Str)
	{
		putch(*Str); // PUTCHAR(*p); causes double messages when expo file is created, like "e_am>Static>Static"
		Str++;
	}
#endif
	Delay_ms(3); //add delay after transmitting?

	clearBit(XMT_ENABLE, XMT_ON);                // WORKS xmt off     assembly CBI 0x18,4 resetting PD4 to zero
	setBit(UCSR0B, RXEN);                        // enable RS232 RCV; assembly SBI 0x0A,4 setting UCSRB, bit 4
	return chr_count;
}

#include <stdarg.h> // to get 'va_start', 'va_end'

#ifndef PC
/****************************************************************************
IK 2023-Jun-12           C U S T O M   printf()     F U N C T I O N
*****************************************************************************
 equivalent of printf, with customized output to UART using cputs() and char wrk_str[] buffer,
 specific for RS-485 chip which needs to enable/disable transmitter or receiver
The output itself is performed by std cputs() which is defined as PutStr()

 the printf() and vprintf() Library options: "Auto" (default)
example:
	printf(...%d, 2) decimal calls PrintfTiny, size ~100 bytes;
	printf(" LNP tester vers %5.2f\r\n", 2.3);   // float formatter test
	printf((...%f) float calls _PrintfLargeNoMb, size ~6kB;

	Library  Using Printf Formatter -> "Small" ~2 kB (setting in Options->General>Library Opton 1 )

	The equivalent of printf below, with customized output to UART, specific for RS-485 chip which needs to enable/disable transmitter or receiver
	this REQUIRES two function calls: to sprintf() and to sputs(). These two calls cannot be combined into function because of variable argument list
	{
		sprintf(wrk_str, " LNP tester vers %5.2f\r\n", 2.3);   // test
		cputs (wrk_str );
	}

	The replacement function is below

	Revisions:    2023-Jun-12      IK     Created.
****************************************************************************/

//extern int  vsprintf( char* buf,   char const* fmt, __va_list parm);
void PRINTF(const char* fmt, ...)
{
	va_list argp;
//    __disable_interrupt();
	va_start(argp, fmt);
	vsprintf((char*)wrk_str, fmt, argp);  // wrk_str[] must have enough length!
	va_end(argp);
//    __enable_interrupt();
	PutStr((char*)wrk_str);
}

int SPRINTF( char *OutStr, char * fmt, ...) // OutStr added to be compatible with standard sprintf, it MUST BE ALWAYS  wrk_str[]
{
	va_list argp;
	int written_chars = -1;
	va_start(argp, fmt);
	written_chars = vsprintf(OutStr, fmt, argp);  // OutStr is wrk_str[] must have enough length!
	va_end(argp);
//    sprintf_P(wrk_str, fmt, ...);
//	PutStr(wrk_str);
	return written_chars;
}
#endif //PC



/****************************************************************************
IK 2023-Jun-26    CopyConstString()  F U N C T I O N
*****************************************************************************
Copies string from FLASH (CODE segment) into RAM (DATA segment)
 params:  buffer to copy  from FLASH location into RAM
		  buffer in RAM to copy  to - printf_buff as example

*****************************************************************************/

uint16 CopyConstString(const char FL * str_f_ptr, char* dest){
  char* bufptr = dest;
  char* start_t_buf = dest;
  uint16 str_len;
  do {
	 *bufptr++ = *str_f_ptr;
  } while (*str_f_ptr++ != 0) ;
	str_len = (uint16)(bufptr - start_t_buf) - 1; // minus one to exclude ending zero
	return str_len;
}

// cputs() for flash strings
/****************************************************************************
IK 2023-Jun-26     C U S T O M   cputs() for strings in Flash F U N C T I O N
*****************************************************************************
 specific for RS-485 chip which needs to enable/disable transmitter or receiver
 uses additional buffer to copy "str_f_ptr" from FLASH location into RAM
  - CopyConstString(str_f_ptr, printf_buff);

The output itself is performed by std cputs() which is defined as PutStr()

this function is waiting while string is beeing transmitted via UART!
Thus, the time it takes depend on baudrate and string length, but interrupts are enabled
*****************************************************************************/
uint16 PrintConstString(const char FL * str_f_ptr){
	CopyConstString(str_f_ptr, printf_buff);
	return PutStr(printf_buff);
}

//Print_F for flash
/****************************************************************************
IK 2023-Jun-26           C U S T O M   Print_F()     F U N C T I O N
*****************************************************************************
 equivalent of IAR printf_P(), with customized output to UART using cputs() and char wrk_str[] buffer,
 uses additional buffer to copy "fmt" from FLASH location into RAM - CopyConstString(fmt, printf_buff);
 specific for RS-485 chip which needs to enable/disable transmitter or receiver
The output itself is performed by std cputs() which is dfined as PutStr()

	REQUIRES IAR option --string_literals_in_flash
	Revisions:    2023-Jun-26      IK     Created.
 IT CANNOT PRINT CONSTANT STRINGS, like
  sprintf(RCI_message, "%s Threshold, range 1 to 2000 %s", "Rip V", "mV")
because "Rip V" and "mV" are ALSO placed into flash (CODE space) by compiler, but sprintf expects pointers to DATA space, in RAM.
And, without knowing in advance what arg list is, we cannot always substitute flash string with string in RAM
This must be done before calling sprintf, example:
	char tmp1[6] = "Rip V";	// CopyConstString("Rip V", tmp1);
	char tmp2[3] = "mV";	// CopyConstString("mV", tmp2);
	sprintf(RCI_message, "%s Threshold, range 1 to 2000 %s", tmp1, tmp2); //

****************************************************************************/
void Print_F( char FL * fmt, ...)
{
	va_list argp;
	CopyConstString(fmt, printf_buff);
	va_start(argp, printf_buff);
	vsprintf((char*)wrk_str, printf_buff, argp);  // wrk_str[] must have enough length!
	va_end(argp);
//    sprintf_P(wrk_str, fmt, ...);
	PutStr((char*)wrk_str);
}

int SPRINT_F( char *OutStr, char FL * fmt, ...) // OutStr added to be compatible with standard sprintf, it MUST BE ALWAYS  wrk_str[]
{
	va_list argp;
	int written_chars = -1;
	CopyConstString(fmt, printf_buff);
	va_start(argp, printf_buff);
	written_chars = vsprintf(OutStr, printf_buff, argp);  // OutStr is wrk_str[] must have enough length!
	va_end(argp);
//    sprintf_P(wrk_str, fmt, ...);
//	PutStr(wrk_str);
	return written_chars;
}
//#define Print_F(format, var)   sprint_P(wrk_str, format, var); PutStr(wrk_str);  // IK20230706 limits to one 'var' argument
//#define Print_F   printf_P // IK20230706 requres C_PUTCHAR which is currently empty function; undef - redefining it does not work


/* call this function when "Enter" supposed to finish entry of string (for getfloat, getdec)
IK20210822 added allowance for leading and trailing spaces,
EXCLUDING empty input; otherwise following atol(), atof() rules:
"converting as much as possible from string to a number"
"  -123 \r" returns TRUE  because atol() converts it to -123
" +456  \r" returns TRUE  because atol() converts it to 453
"  \r" returns FALSE (only spaces or empty string)
" 12.3-6 \r" returns TRUE   because atof() converts it to 1.23
"+456-78 \r" returns TRUE because atol() converts it to 456
" -6.21+2.78 " returns TRUE because atof() converts it to -6.21
*/
int Is_Numeric(char* strp)
{
	char* stringStartAddress = strp;
	int CharInString;
	int DigitDetected = FALSE;
	//	int SpaceAfterDigitDetected = FALSE;
	do {
		CharInString = (int)*strp;
		if (CharInString == ' ')
		{
			if (DigitDetected == FALSE) // ignoring leading spaces
				stringStartAddress = strp;
			//			else // trailing spaces
			//				SpaceAfterDigitDetected = TRUE;
		}
		else if (CharInString == CaRet)
		{
			if (strp == stringStartAddress)  //  return only, without new entries
				return FALSE;
			else
				return TRUE; // OK
		}

		else // not a digital imput
			if (((CharInString < '0') || (CharInString > '9')) &&
				((CharInString != '.') && (CharInString != 'e') && (CharInString != 'E')) &&
				((CharInString != '-') && (CharInString != '+')))
			{
				//			if ((SpaceAfterDigitDetected == TRUE) && (CharInString != ' '))
				return FALSE;//error
			}
			else
				DigitDetected = TRUE;
		strp++;//go to next position
	} while (*strp != 0);//end of string
	return TRUE; // OK
}

/*-------------------------- interrupts ----------------------------*/
//most of these interrupts are not used but they were defined so that
//a reti instruction was included.
#pragma vector = INT0_vect
INTERRUPT void INT0_interrupt (void)
{}
#pragma vector = INT1_vect
INTERRUPT void INT1_interrupt (void)
{}
#pragma vector = INT2_vect
INTERRUPT void INT2_interrupt (void)
{}
#pragma vector = TIMER1_COMPA_vect
INTERRUPT void TIMER1_COMPA_interrupt (void)
{}
#pragma vector = TIMER1_COMPB_vect
INTERRUPT void TIMER1_COMPB_interrupt (void)
{}
#pragma vector = TIMER1_CAPT_vect
INTERRUPT void TIMER1_CAPT_interrupt (void)
{}
#pragma vector = TIMER0_COMPA_vect
INTERRUPT void TIMER0_COMPA_interrupt (void)
{
/*
#ifdef TIME_TESTING
	Timer0_finished = true;
	time_testing++;
	if (time_testing & 1) {// create 1 for one period, zero for next
		//SetTestPin44;    // IK20250523 set test pin #44 (easy to solder to)
	}
	else{
		//ClearTestPin44;    // IK20250523 reset test pin #44 (easy to solder to)
	}
#endif // #ifdef TIME_TESTING
*/
}

#pragma vector = TIMER0_COMPB_vect
INTERRUPT void TIMER0_COMPB_interrupt (void)
{}
#pragma vector = TIMER2_COMPB_vect
INTERRUPT void TIMER2_COMPB_interrupt (void)
{}
#pragma vector = TIMER2_OVF_vect
INTERRUPT void TIMER2_OVF_interrupt (void)
{}
#pragma vector = TIMER0_OVF_vect
INTERRUPT void TIMER0_OVF_interrupt (void)
{}
#pragma vector = TIMER1_OVF_vect
INTERRUPT void TIMER1_OVF1_interrupt (void)
{}
#pragma vector = SPI_STC_vect
INTERRUPT void SPI_STC_interrupt (void)
{}
#pragma vector = USART0_UDRE_vect
INTERRUPT void USART0_UDRE_interrupt (void)
{}
#pragma vector = USART0_TX_vect
INTERRUPT void USART0_TX_interrupt  (void)
{}
#pragma vector = ADC_vect
INTERRUPT void ADC_interrupt (void)
{}
#pragma vector = EE_READY_vect
INTERRUPT void EE_READY_interrupt (void)
{}
#pragma vector = ANALOG_COMP_vect
INTERRUPT void ANALOG_COMP_interrupt (void)
{}
#pragma vector = SPM_READY_vect
INTERRUPT void SPM_READY_interrupt (void)
{}

#pragma vector = PCINT0_vect
INTERRUPT void PCINT0_interrupt(void)
{}
#pragma vector = PCINT1_vect
INTERRUPT void PCINT1_interrupt(void)
{}
#pragma vector = PCINT2_vect
INTERRUPT void PCINT2_interrupt(void)
{}
#pragma vector = PCINT3_vect
INTERRUPT void PCINT3_interrupt(void)
{}
#pragma vector = WDT_vect
INTERRUPT void WDT_interrupt(void)
{
	  // This code runs when WDT times out
	_NOP(); // to put break point and exit interrupt in debugger, to see in what part of the program watchdog event happen
}

#pragma vector=TWI_vect
INTERRUPT void TWI_interrupt(void)
{}
/********************************************************************/
/*               T I M E R 2   C O M P A R E    I N T E R R U P T   */
/********************************************************************/
/* DESCRIPTION: This interrupt is driven from the Mega644 Timer. This
		timer is set up as a compare timer with a 1.0 ms tick.
		When the timer get to the compare value an intrrupt
		is generated and the counter automatically reset to 0.
		The timer has a 10 ms component run by the timer extender.
		The 1.0 ms tick is based on a 16 mhz clock crystal.
		Every 576th time this routine is skipped to account for
		a non exact digital integer which improves accuracy.

	Inputs:     1 ms compare interrupt
	Outputs:    timer.Generic
				blink_tmr
				cop_led_tmr
				timer.CommChHoldOff
				time_low
				time_mid
				time_high
				irig_offset_tmr
				timer.RT_correction
				timer.Calibration
				irig_tmr
				timer.UART_delay_100us

	Notes:
	Revisions:  1/20/15      REC     Created.                      */
/*-------------------------- Timer 2 Overflow Interrupt -------------*/
#pragma vector = TIMER2_COMPA_vect
INTERRUPT void TIMER2_COMPA_interrupt(void)
{
#ifdef INTERRUPT_TIME_TESTING
	clearBit(PORTD, TxRxLED_Green_PD7);    // normal state OFF, logic 1 for green LED pin. turn on Green LED for time measuring
	SetTestPin44;
#endif // #ifdef INTERRUPT_TIME_TESTING

	//ISR_TxOutputHandler(); // output to UART

	/********************* Do General Timer Updating ********/
	timer.time_keep++;					// always increasing, set to a value by DNP SysData.NV_UI.StartUpProtocol
	timer.extender++;

	if (timer.ModBus_100us != 0)
		timer.ModBus_100us--;
	if (timer.UART_delay_100us != 0)
		timer.UART_delay_100us--;
	//#ifndef PC
	if (timer.extender >= 10)
		//#endif
	{
		timer.extender = 0;								// reset extender
		timer.FreeRunningCounter++;						// Grok20251231

		if ((rt.operating_protocol == ASCII_CMDS) && (rt.OperStatusWord & SendRealTimeData_eq1_Bit))
		{
			if (timer.RealTimeUpdate > 0)
				timer.RealTimeUpdate++;
			if ((timer.RealTimeUpdate > 200))
			{
				setBit(rt.OperStatusWord, RealTimeDataReady_eq1_Bit); // IK20250820 it will clear in main loop when sending brief info
				timer.RealTimeUpdate = 0;
			}
		}
		//IK20250812 moved here to check every 1 ms alarms
		if ((rt.OutData.measured.battery_voltage_f > 14.0f) &&			// between 14 and 380 vdc
			(rt.OutData.measured.battery_voltage_f < 380.0f))			// waits till getting good battery voltage
			Check_Alarms();							// to start checking alarms

		/********************* Do General Timer Updating ********/
		if (timer.Generic != 0)						// if general timer not zero
			timer.Generic--;						// decrement it.
		if (timer.TxRxLED_blink != 0)				// if LED5 blink timer not zero
			timer.TxRxLED_blink--;					// decrement it.
		if (timer.comm_activity != 0)
			timer.comm_activity--;
		if (timer.CommChHoldOff != 0)
			timer.CommChHoldOff--;
		if (timer.TWI_hangup != 0) // IK202050916 suspected that while(timer.TWI_hangup != 0) loop could miss zero?
			timer.TWI_hangup--;
		if (timer.TWI_reset != 0)
			timer.TWI_reset--;
		if (timer.Calibration != 0)
			timer.Calibration--;
		if (timer.TWI_lockup != 0)					// detects lock up
			timer.TWI_lockup--;
		if (timer.ADC_ms != 0)
			timer.ADC_ms--;
		if (timer.start_up_ms != 0)					// if start up timer not zero
			timer.start_up_ms--;					// then decrement
		if (timer.PWM_calibration != 0)
			timer.PWM_calibration--;
		if (timer.IO_update != 0)
			timer.IO_update--;
		if (timer.limit_mode_timeout_ms <= 600000)	// 10 minutes
			timer.limit_mode_timeout_ms++;

		// display control timers
		if (timer.AlarmLED_blink != 0)
			timer.AlarmLED_blink--;
		if (timer.UpDownChange_rate_ms != 0)		// IK20250710 Limits how fast values are changed if user holds UP or DOWN button. 100 ms timer,
			timer.UpDownChange_rate_ms--;
		if (timer.disp_var_change_ms != 0)			// in auto mode, changes variables with this period
			timer.disp_var_change_ms--;
		if (timer.InfoLED_blink_ms != 0)
			timer.InfoLED_blink_ms--;

		// button press timers
		if (timer.auto_button != 0)
			timer.auto_button++;
		if (timer.limit_button != 0)
			timer.limit_button++;
		if (timer.up_button != 0)
			timer.up_button++;
		if (timer.down_button != 0)
			timer.down_button++;
		if (timer.reset_button != 0)
			timer.reset_button++;
		timer.TWI_request++;
		if (timer.TWI_request >= TWI_UPDATE_PERIOD_ms) // IK20250925 used to check buttons on Display board (send TWI_Write, then Read) once in a few dosen ms, not each main control loop
		{
			timer.TWI_request = 0;
			Display_Info.DisplayNeedsUpdateFlag = SET;
		}
	}       //end of tmr extender
	//#if ((defined TIME_TESTING) || (defined INTERRUPT_TIME_TESTING))
	//	if (timer.RT_correction >990)					// test
	//		SetTestPin44;								// IK20250523 set test pin #44 (short pulse)
	//#endif // #ifdef TIME_TESTING
	//	if (timer.RT_correction >999)					// test
	//	{
	//#if ((defined TIME_TESTING) || (defined INTERRUPT_TIME_TESTING))
	//		SetTestPin44;										// IK20250523 set test pin #44 (short pulse)
	//#endif // #ifdef TIME_TESTING
	//		timer.RT_correction = 0;
	//#if ((defined TIME_TESTING) || (defined INTERRUPT_TIME_TESTING))
	//		ClearTestPin44;										// IK20250523 reset test pin #44 (end of short pulse)
	//#endif // #ifdef TIME_TESTING
	//	}
}

/********************************************************************/
/*                   U A R T   R E C E I V E   I N T E R R U P T    */
/********************************************************************/
/*  DESCRIPTION:  This interrupt is driven from the UART. When a
	character in RX register UDR0 is ready this interrupt is generated.
	The character is moved into rt.HostRxBuff[256] circular Que.
	if there is no msg started then the routine looks
	for a start sequence of a 05 hex followed by a 64H
	The top level will monitor this and will call the
	routine that will then process the message.
	The SETUP protocol:
	initial token: three chars: Enter, Enter, ESC = 0x0d, 0x0d, 0x1b
	following by 4th char:
	Either 'W' if it is set command (i.e. Write)
	Or     'R' if it is get command (i.e. Read)
	following by 5th char associated with a variable
	following by 6th char '\r' for requests or more chars for set command
	comand always ends with final '\r'

ON PC SIDE:
		private void InitManualMode() {
			//Create Commands and Send them, then update Values
			//Function is called from Timer exept when in ASCII mode
			List<string> commands = new List<string>();
			string reply;
			string formatted_command;

			commands.Add("RC"); // Analog Points
			commands.Add("RG"); // Inter Char Timeout
			commands.Add("RM"); // Meter Addr
			commands.Add("RR"); // Baud Rate
			commands.Add("RW"); // Response Delay
			commands.Add("RH"); // Host Addr

			foreach (string command in commands) {
				formatted_command = "\r\r\u001b" + command; // Format Command

				reply = Send_Command(formatted_command); // Send Command, Receive Reply
				if (reply == null) {
					return;
				} else {
					if (int.TryParse(reply, out int tstInt) != true) return;
					switch (command) // Update all Global Variables with Updated Information
					{
						case "RC":
							analog_points = tstInt; break;
						case "RG":
							inter_character_timeout = reply; break;
						case "RM":
							meter_address = reply; break;
						case "RR":
							baud_rate = reply; break;
						case "RW":
							response_time = reply; break;
						case "RH":
							source_addr = reply; break;
					}
				}
			}
			UpdateManualMode(); // Update GUI Application
		}


	Inputs:     UDR0 (UART0 Data register)
	Outputs:    rt.HostRxBuff, msg_status.
	Notes:
	Revisions:  7/25/06      REC     Created.                       */
/*-------------------------- UART Receive Interrupt ----------------*/
#pragma vector = USART0_RX_vect
INTERRUPT void USART0_RX_interrupt(void)
{
#ifdef PC
	delay_us(1000);
#endif
	uart_byte = UDR0;
	if (rt.operating_protocol == SETUP)                // if in startup mode check for a setup msg
	{
		if ((msg_status == MSG_DONE) && (uart_byte == 0x0D))                // msg started yet ?
		{
			msg_status = MSG_STARTED;
			rt.HostRxBuff[0] = 0x0D;	// rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;
			rt.HostRxBuffPtr = 1;		// rt.HostRxBuffPtr++;
			return;						// and wait for the second CR
		}

		else if ((rt.HostRxBuff[0] == 0x0D) && (msg_status == MSG_STARTED) && (uart_byte == 0x0D) && (rt.HostRxBuffPtr == 1))
		{
			msg_status = GOT_DBL_ENTER;
			rt.HostRxBuff[1] = 0x0D;	// rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;
			rt.HostRxBuffPtr = 2;		// rt.HostRxBuffPtr++;
			return;						// and wait for the third byte == ESC
		}
		else if ((msg_status == GOT_DBL_ENTER) && (uart_byte == 0x1B) && (rt.HostRxBuffPtr == 2) )
		{
			msg_status = GOT_FULL_TOKEN;
			rt.HostRxBuff[2] = 0x1B;	// rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;
			rt.HostRxBuffPtr = 3;		// rt.HostRxBuffPtr++;
			//num_of_inbytes = 3;	//IK20251219 'num_of_inbytes' is not used in SETUP protocol
			return;						// and wait for the command code
		}
		else if (msg_status == GOT_FULL_TOKEN)
		{
			rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;
			rt.HostRxBuffPtr++;
			//num_of_inbytes++;	//IK20251219 'num_of_inbytes' is not used in SETUP protocol
			if(uart_byte == 0x0D) // end of command arrived
			{
				msg_status = MSG_ARRIVED;
				rt.HostRxBuff[rt.HostRxBuffPtr] = 0; // put EoL, sting can be checked by IsNumeric(char *).
			}
		}
		if (rt.HostRxBuffPtr >= 12)                             // if at end of Q wrap around
		{
			for (rt.HostRxBuffPtr = 15; rt.HostRxBuffPtr > 0; rt.HostRxBuffPtr--)		// IK20240329 reversed for cycle so it ends with 'rt.HostRxBuffPtr'=0, saving re-assignment rt.HostRxBuffPtr = 0;
				rt.HostRxBuff[rt.HostRxBuffPtr] = 0;                 // clear out last msg
			//rt.HostRxBuffPtr = 0;
			msg_status = MSG_DONE;
		}
	} // end of if (rt.operating_protocol == SETUP)

	else if (rt.operating_protocol == ASCII_CMDS)
	{
#ifdef UART_TEST_SIGNALS	/******TEST SIGNAL********************/
		clearBit(PORTD, TxRxLED_Green_PD7);    // normal state OFF, logic 1 for green LED pin. turn on Green LED for time measuring
#endif	/******TEST SIGNAL********************/
		setBit(rt.Host, CharAvailableFlag);
		if (uart_byte == CaRet) {// discovery of CR, '\r'
			setBit(rt.Host, CmdAvailFlag);
			rt.HostRxBuff[rt.HostRxBuffPtr] = 0;
		}
		else
			rt.HostRxBuff[rt.HostRxBuffPtr] = (char)uart_byte;
		if (rt.HostRxBuffPtr < (HOST_RX_BUFF_LEN - 1)) rt.HostRxBuffPtr++;
		else rt.HostRxBuffPtr = 0;
#ifdef UART_TEST_SIGNALS	/******TEST SIGNAL********************/
		setBit(PORTD, TxRxLED_Green_PD7);    // normal state OFF, logic 1 for green LED pin. turn on Green LED for time measuring
#endif	/******TEST SIGNAL********************/
	}

	else if (SysData.NV_UI.StartUpProtocol == DNP3)
	{
		if (msg_status == MSG_DONE)                 // msg started yet ?
		{
			if ((rt.HostRxBuff[0] == 0x05) && (uart_byte == 0x64)) //last byte 0x05 and this byte 0x64
			{
				// IK20250206 changed ifs to calculation
				DNP_time_delay = (uint16)(Baud_19200 / Existing.baud_rate); // IK20250925 !!! it is not checking, just can be sent on DNP request TIME_DELAY_RESPONSE
				msg_status = MSG_STARTED;
				num_of_inbytes++;                   // compensate for 0x05
			}
		}

		if (msg_status == MSG_STARTED)
		{
			rt.HostRxBuffPtr++;
			num_of_inbytes++;
		}
		else
		{
			rt.HostRxBuffPtr = 0;
			num_of_inbytes = 0;
		}
		if (rt.HostRxBuffPtr >= HOST_RX_BUFF_LEN-1)               // if end of Q wrap around
			rt.HostRxBuffPtr = 0;
		rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;                 // store the new char in Q
		//timer.CommChHoldOff = 20;                         // msgs coming in so hold
	} // end of if (SysData.NV_UI.StartUpProtocol == DNP)

	else if (SysData.NV_UI.StartUpProtocol == MODBUS)
	{
		/*----------- Frame Error Check ---------*/
		if (UCSR0A & FRAME_ERROR_BIT)				// if frame error detected
		{
			parity_error = true;					// this throws it out
		}

		/*----------- Parity Check --------------*/
		if ((rt.protocol_parity == ODD) || (rt.protocol_parity == EVEN))
		{
			if (UCSR0A & PARITY_ERROR_BIT)			// UART_parity error if bit set
			{
				parity_error = true;
			}
		}

		msg_status = MSG_STARTED;
		rt.HostRxBuff[rt.HostRxBuffPtr] = uart_byte;                 // store the new char in Q
		num_of_inbytes++;

		timer.ModBus_100us = 2;								// 2 char times for 19.2
		if ((SysData.inter_char_gap) > 1 && (SysData.inter_char_gap < 65000))
			timer.ModBus_100us = SysData.inter_char_gap / 1000;           // IK20240828 changed timer to 1.0 ms, SysData.inter_char_gap comes in microseconds

		rt.HostRxBuffPtr++;
		if (rt.HostRxBuffPtr >= 50)									// if at end of Q wrap around
			rt.HostRxBuffPtr = 0;
	} // end of if (SysData.NV_UI.StartUpProtocol == MODBUS)
} // end of INTERRUPT void USART0_RX_interrupt(void)

/*-------------------------- functions  -----------------------------*/

/*******************************************************************/
/*               C A L C    D  N P    C R C                        */
/*******************************************************************/
/*
DESCRIPTION: this routine calculates the CCITT CRC for the DNP SysData.NV_UI.StartUpProtocol.
		Polynomial = X15+X13+X10+X9+X7+X5+X4+X3+X2+1
		0x0A6BC (polynomial generator)

Inputs:
Outputs:
Notes:
Revisions:   11/05/99     REC       Created
*/
/********************************************************************/
void Calc_DNPCRC( uint8 * p, uint16 count)
{
	uint8 bits;
	uint8 msg_byte;
	uint8 bit;
	__disable_interrupt();
	crc = 0;
	do {                                       //while (count--)
		msg_byte = *(uint8 *)p;
		p++;    //-!- IK20231214 error: "*p++;" intend is to increment pointer, not its content
		crc ^= msg_byte;
		for (bits = 0; bits < 8; bits++)
		{
			bit = crc & 1;
			crc >>= 1;
			if (bit)
				crc ^= 0x0a6bc;
		}
	} while (--count);
	crc = ~crc;
	__enable_interrupt();
}
/***************************************************************************
			C A L C    M O D B U S   C R C
****************************************************************************
DESCRIPTION: this routine calculates the CCITT CRC for the Modbus RTU SysData.NV_UI.StartUpProtocol.
			Polynomial = X15 + X13 + 1

Inputs:
Outputs:
Notes:
Revisions:   10/05/99     REC       Created
*/
/***************************************************************************/
void Calc_ModbusCRC (uint8 numchars)
{
#define FEEDBACK    0xA001
	uint8  msg_ptr;
	uint8  bit_count;
	uint8  msg_byte;
	uint8  flag;
	uint16 tmp;
	__disable_interrupt();

	crc = 0xFFFF;                               // preload
	for (msg_ptr = 0; msg_ptr <= (numchars - 1); msg_ptr++)
	{
		msg_byte = wrk_str[msg_ptr];             // get byte
		crc ^= (uint16)msg_byte;                //XOR byte with crc
		for (bit_count = 0; bit_count < 8; ++bit_count)
		{
			flag = crc & 0x0001;                  //look at each bit

			crc >>= 1;                            //shift 1 bit to right
			crc &= 0x7FFF;                        //clear msb

			if (flag != 0)                        //if lsb = 1 then
				crc ^= FEEDBACK;                   //Exclusive or with polynomial
		}
		tmp = crc;
	}
	tmp = crc;                               //swap bytes
	crc = crc << 8;                          //since they come in low byte
	tmp = tmp >> 8;                          //first
	crc = crc | tmp;
	__enable_interrupt();
	return;
}

/*********************************************************************/
/*                      R E S E T    T W I                           */
/*********************************************************************/
/*  Description:  This routine resets the TWI. Sending stop in slave rcv
				returns the TWI machine to a known state.

	Inputs:     None
	Outputs:    TWCR
	Notes:      None
	Revisions:  11/02/10     REC     Created

*/
/********************************************************************/
void TWI_Reset(void)
{
	TWCR = 0;                               // shut off TWI
	TWCR = (TWINT | TWSTO | TWEN); //= 0x94;// initiate stop bit
#ifndef PC
	timer.TWI_reset = 6;
#else // for PC simulation
	timer.TWI_reset = 0;
#endif

	while (timer.TWI_reset != 0) {				// give some time to stop
		__no_operation(); // KI20250701 inside is Sleep(1); which greatly decrease CPU load in simulation on PC
	}
	TWCR = (TWINT | TWEA | TWEN); //= 0xC4;		// TWI, addr, & TWI interrupt ARE enabled
	TWCR = (TWINT | TWSTO | TWEN);//= 0x94;		// initiate stop bit
}

/// <summary>
/// IK20250102 replaced repetitive code with function call
/// </summary>
/// <param name=""></param>
/// <returns>register TWSR & 0xF8</returns>
//timer.TWI_hangup appears to roll via zero? probably was called from different parts of the code
uint8 TWI_Wait(void) //IK20250915 this is causing WDT interrupt / reset
{
	while (((TWCR & TWINT) != TWINT) && (timer.TWI_hangup != 0)) {
		__no_operation();
	}
	return TWSR & TWSR_STATUS_MASK; // & 0xF8
}

/*********************************************************************/
/*                W R I T E   T W I                                  */
/*********************************************************************/
/*  Description:  This routine builds and sends a TWI message to write
				a value to the appropriate destination.
				Uses the microprocessor Two Wire Interface (TWI) bus.
	Inputs:     destination, data.
	Outputs:    None
	Notes:      None
	Revisions:  06/28/15     REC     Created
	20250924 IK measured timing: transmits 4 bytes, it takes ~280 us
																	 */
/*********************************************************************/
void TWI_Write(uint8 DestADR, uint8 type, uint8 msg_low, uint8 msg_high)
{
	uint16 w;
	uint8 twi_status_reg;
//#ifdef TIME_TESTING
//		SetTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
//#endif // #ifdef TIME_TESTING
	twi.s.error = TWI_OK;
	TWCR &= 0xFE;								// shut off TWI interrupt
#ifndef PC
	timer.TWI_hangup = TWI_TIMEOUT_ms;   //200
#else // for PC simulation
	timer.TWI_hangup = 0;
#endif
	while (((TWSR & TWSR_STATUS_MASK) != TWSR_STATUS_MASK) && (timer.TWI_hangup != 0)) {
		__no_operation();
	}
	if (timer.TWI_hangup == 0)					// wait here till ready
	{
		twi.s.error = TWI_NOT_READY;			// if not then error
		TWI_Reset();
		return;
	}
#ifndef PC
	timer.TWI_hangup = TWI_TIMEOUT_ms;//200
#else // for PC simulation
	timer.TWI_hangup = 0;
#endif
	TWCR = 0xA4;								// initiate start bit
	twi_status_reg = TWI_Wait();
	if (((twi_status_reg != 0x08) && (twi_status_reg != 0x10))
		|| (timer.TWI_hangup == 0))
	{
		twi.s.error = TWI_START_BIT_ERROR;		// if not then error
		TWI_Reset();
		return;
	}
#ifndef PC
	timer.TWI_hangup = TWI_TIMEOUT_ms;//200
#else // for PC simulation
	timer.TWI_hangup = 0;
#endif
	TWDR = DestADR;
	TWCR = 0x84;								// send destination addr

	twi_status_reg = TWI_Wait();
	if (((twi_status_reg != 0x18)
		&& (twi_status_reg != 0x20)
		&& (twi_status_reg != 0x40))			// has it been sent?
		|| (timer.TWI_hangup == 0))
	{
		twi.s.error = TWI_DEST_ADR_ERROR;		// if not then error
		TWI_Reset();
		return;
	}

#ifndef PC
	timer.TWI_hangup = TWI_TIMEOUT_ms;//200
#else // for PC simulation
	timer.TWI_hangup = 0;
#endif
	TWDR = type;								// load the type of data
	TWCR = 0x84;								// and send it

	twi_status_reg = TWI_Wait();
	if (((twi_status_reg != 0x28) && (twi_status_reg != 0x30))
	 || (timer.TWI_hangup == 0))				// has it been sent?
	{
		twi.s.error = TWI_SEND_1_ERROR;			// if not then error
		TWI_Reset();
		return;
	}

	if ((DestADR == DISPLAY_WRITE_TWI_ADR)
		|| (DestADR == ALARM_WRITE)
		|| (DestADR == TWI_ADR_WRITE_IO))
	{											// these boards need more bytes to be sent
#ifndef PC
		timer.TWI_hangup = TWI_TIMEOUT_ms;//200
#else // for PC simulation
		timer.TWI_hangup = 0;
#endif
		TWDR = msg_low;							// load the low msg byte
		TWCR = 0x84;							// and send it
		twi_status_reg = TWI_Wait();
		if (((twi_status_reg != 0x28) && (twi_status_reg != 0x30))
		 || (timer.TWI_hangup == 0))			// has it been sent?
		{
			twi.s.error = TWI_SEND_2_ERROR;		// if not then error
			TWI_Reset();
			return;
		}
		if (DestADR != TWI_ADR_WRITE_IO)		// IO Write does not send this byte
		{
#ifndef PC
			timer.TWI_hangup = TWI_TIMEOUT_ms;//200
#else // for PC simulation
			timer.TWI_hangup = 0;
#endif
			TWDR = msg_high;					// load the high msg byte
			TWCR = 0x84;						// and send it
			twi_status_reg = TWI_Wait();
			if (((twi_status_reg != 0x28) && (twi_status_reg != 0x30))
			 || (timer.TWI_hangup == 0))		// has it been sent?
			{
				twi.s.error = TWI_SEND_2_ERROR;	// if not then error
				TWI_Reset();
				return;
			}
		} // end display and alarm writes
	} // end display,alarm, and IO writes

	TWCR = 0x94;								// initiate stop bit
//#ifdef TIME_TESTING
//	ClearTestPin44;										// IK20250924 time stesting
//	_NOP();
//	SetTestPin44;
//#endif // #ifdef TIME_TESTING

	for (w = 0; w < 100; w++)	// wait a tad, IK202050924, was 600. for 600 cycles, ~ 320 us. for 200 cycles ~ 110 us,for 100 cycles ~ 58 us
		__no_operation();
//#ifdef TIME_TESTING
//	ClearTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
//#endif // #ifdef TIME_TESTING
}


//void Ask_Write_TWI(uint8 DestADR, uint8 type, uint8 msg_low, uint8 msg_high)
//{}
/*********************************************************************/
/*                R E A D   T W I                                    */
/*********************************************************************/
/*  Description:  This routine builds and sends a TWI message to read
				data from the appropriate destination.
				Uses the microprocessor Two Wire Interface (TWI) bus.
	IK20250603  CALL ONLY FROM MAIN LOOP, not from interrupt!
				Processor does not have interrupt priority; it means
				if one interrupt is active, others are waiting.
				interrupts must be enabled for timeout:
				timer.TWI_hangup is decreased in Timer2 interrupt
				and checked here in a while(){} loop
	Inputs:     destination.
	Outputs:    to the 'twi.buffer[twi_ptr]' up to 4 bytes
	Notes:      None
	Revisions:  06/28/15     REC     Created
																	 */
/*********************************************************************/
void TWI_Read (uint8 dest_ADR)
{
#ifdef PC // create simulated signals, simulation calibrated on 20250730
	uint16 Simulated_ADC_counts = 0;
	if (measurement_ID == ADC_BATT_VOLTS)  // 1
	{
		Simulated_ADC_counts = 20914; // counts @ 125 V 21080
		Simulated_ADC_counts = (uint16)AddNoise(UserValue[Vbat], 0.02) * 167;
	}
	else if (measurement_ID == ADC_FAULT_VOLTS) // 2
	{
		Simulated_ADC_counts = 15948; // counts 15945
		Simulated_ADC_counts = (uint16)AddNoise(UserValue[Vgnd], 0.02) * 257;
	}
	else if (measurement_ID == ADC_MINUS_GND_VOLTS) // 3
	{
		Simulated_ADC_counts = 20865; // counts 20771
		Simulated_ADC_counts = (uint16)AddNoise(UserValue[Vbat], 0.03) * 163;
	}
	else if (measurement_ID == ADC_RIPPLE_CURRENT)  // 5
	{
		Simulated_ADC_counts = 4283; // counts, shows 59 mA
		Simulated_ADC_counts = (uint16)AddNoise(UserValue[Irip], 0.05) * 124; // UserValue[Irip] in mA
	}
	else if (measurement_ID == ADC_RIPPLE_VOLTAGE)  // 6
	{
		Simulated_ADC_counts = 148; // counts, should show 216 mV
		Simulated_ADC_counts = (uint16)AddNoise(UserValue[Vrip], 0.005) * 0.685; // UserValue[Vrip] in mV IK20251119: added transformer in line with Vbat, @ 60hz: 216 mV = 148 counts, 1.46 mV/count ratio, 0.685 count/mV
	}
	else // for channels which are not read in firmware, #0, #4, #7.
		Simulated_ADC_counts = (uint16)AddNoise(1000, 0.05);

	// Reconstruction: result = (twi.buffer[BYTE_1] * 256) + twi.buffer[BYTE_2];
	// decomposition of simulated values 'Simulated_ADC_counts' into bytes
	twi.buffer[BYTE_1] = (Simulated_ADC_counts>>8) & 0xFF;
	twi.buffer[BYTE_2] = Simulated_ADC_counts & 0xFF;

#else  // ATMEL
#define DONE                10	// used in void TWI_Read (uint8 dest_ADR): while (twi_data != DONE)
#define NOT_DONE            0	// used in void TWI_Read (uint8 dest_ADR) initially twi_data = NOT_DONE;

	uint8 twi_status_reg;
	uint8 twi_ptr, twi_data;
	uint16 bt;

	twi.s.error = TWI_OK;
	clearBit(TWCR, TWIE); // TWCR &= 0xFE;      // shut off interrupt
	timer.TWI_hangup = TWI_TIMEOUT_ms;
	while (((TWSR & TWSR_STATUS_MASK) != TWSR_STATUS_MASK) && (timer.TWI_hangup != 0))
	{
		__no_operation();
	}	//wait here till TWI is ready
	if (timer.TWI_hangup == 0)
	{
		twi.s.error = TWI_NOT_READY;            // if not then error
		TWI_Reset();
		return;
	}
	timer.TWI_hangup = TWI_TIMEOUT_ms;
	TWCR = (TWINT | TWSTA | TWEN); //TWCR = 0xA4;// initiate start bit; TWINT Flag must be cleared by software by writing a logic '1' (one)to it.
	twi_status_reg = TWI_Wait();
	if (((twi_status_reg != 0x08) && (twi_status_reg != 0x10))
	|| (timer.TWI_hangup == 0))
	{
		twi.s.error = TWI_START_BIT_ERROR;      // if not then error
		TWI_Reset();
		return;
	}
	timer.TWI_hangup = TWI_TIMEOUT_ms;
	TWDR = dest_ADR;
	TWCR = (TWINT | TWEN); //TWCR = 0x84;       // send destination addr

	twi_status_reg = TWI_Wait();            // check status
	if ((twi_status_reg != 0x40)
	|| (timer.TWI_hangup == 0))                 // has it been sent?
	{
		twi.s.error = TWI_DEST_ADR_ERROR;       // if not then error
		TWI_Reset();
		return;
	}

	TWCR = (TWINT | TWEA | TWEN); //TWCR = 0xC4;// enable ACK
	timer.TWI_hangup = TWI_TIMEOUT_ms;
	twi_ptr = 0;                                // point to start of buffer
	twi_data = NOT_DONE;
	while (twi_data != DONE)
	{
		twi_status_reg = TWI_Wait();        // chk status

		if (((twi_status_reg != 0x50) && (twi_status_reg != 0x58))
		|| (timer.TWI_hangup == 0))
		{
			twi.s.error = TWI_READ_ERROR;       // got an error
			TWI_Reset();
			return;
		}
		twi.buffer[twi_ptr] = TWDR;             // get rcvd byte
		twi_ptr++;                              // increment ptr
		if (twi_ptr >= 2)
			TWCR = (TWINT | TWEN); // = 0x84;   // writes 1 to TWINT to clear it and shuts off ACK
		else // twi_ptr is 0 or 1
			TWCR = (TWINT | TWEA | TWEN); // = 0xC4; // writes 1 to TWINT to clear it
		if (twi_ptr >= 3)                       // all the bytes in?
		{
			twi_data = DONE;                    // yep so finish
			timer.TWI_lockup = 13000;             // timer.TWI_lockup is not checked in Comm firmware, only in display firmware
		}
	}

	TWCR = 0x94;//(TWINT | TWSTO | TWEN); // = 0x94;   // initiate stop bit
	for (bt = 0; bt < 100; bt++)               // wait a tad. was 1024, delay ~310 us. IK20290924 changed for 100 cycles, ~58 us
	{
		__no_operation();
	}
#endif  //PC
}

char * ToUpper(char* in_str)
{
	char* chr_ptr = in_str;
	char chr;
	while (*chr_ptr != '\0') {
		chr = *chr_ptr;
		chr = toupper(chr);
		*chr_ptr = chr;
		chr_ptr++;
	}
	return in_str;
}

/******************************************************************************/
#ifdef PC
void LCDSendTxt(const char* a)
{
	int i, len = strlen(a);
	for (i = 0; i < len; i++)
	{
		D_PutChar(a[i]);
	}
}
#endif

//IK20250102 write up to 10 chars to upper LED (up to 5 chars and and up to 5 dots)
void Write_Numeric_Display(char* num_str)
{
#define MAX_NUMERIC_STR_LENGTH 11 // 10 chars + EndOfLine ('\0', EoL). 10 because period is part of 7-seg LED  and can be included: "5.6.7.8.9."
	char tmpStr[MAX_NUMERIC_STR_LENGTH];
	char* p_str = tmpStr;
	strncpy(tmpStr, num_str, MAX_NUMERIC_STR_LENGTH); // local copy not to change original string
	tmpStr[MAX_NUMERIC_STR_LENGTH-1] = 0; // EoL just in case
	ToUpper((char*)tmpStr); // LED can show only capital letters and digits
#ifdef PC
	D_CleanLCD_line(1); // LCDSendTxt will print it
	LCDSendTxt(p_str);
#endif
	char chr1st = *p_str; p_str++;
	char chr2nd = *p_str; p_str++;
	//-!- IK20250102 check for End of String '\0'?
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_UPPER_STR_01, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_UPPER_STR_23, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_UPPER_STR_45, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_UPPER_STR_67, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; //num_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_UPPER_STR_89, chr1st, chr2nd);
}

void Write_ASCII_Display(char* str)
{
#define MAX_INFO_STR_LENGTH 9 // 8 chars + EndOfLine ('\0', EoL). 8 because period is part of 14-seg LED  and can be included: "A.R.G.A."
	char tmpStr[MAX_INFO_STR_LENGTH];
	char* p_str = tmpStr;
	strncpy(tmpStr, str, MAX_INFO_STR_LENGTH); // local copy not to change original string
	tmpStr[MAX_INFO_STR_LENGTH-1] = 0; // EoL just in case
	ToUpper((char*)tmpStr); // LED can show only capital letters and digits
#ifdef PC
	D_CleanLCD_line(2); // LCDSendTxt will print it
	LCDSendTxt(str);
#endif
	ToUpper((char*)p_str); // LED can show only capital letters and digits
	//-!- IK20250102 check for End of String '\0'?
	char chr1st = *p_str; p_str++;
	char chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_LOWER_STR_01, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_LOWER_STR_23, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; p_str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_LOWER_STR_45, chr1st, chr2nd);
	chr1st = *p_str; p_str++;
	chr2nd = *p_str; //str++;
	TWI_Write(DISPLAY_WRITE_TWI_ADR, DISPLAY_LOWER_STR_67, chr1st, chr2nd);
}

void Write_ASCII_Display_FlashString(char FL * str_f_ptr) {
	if(Display_Info.Info_segments_on_off == ON) // == 0
		CopyConstString(str_f_ptr, Display_Info.InfoStr);
	else
		CopyConstString(FiveSpaces, Display_Info.InfoStr);
	Write_ASCII_Display(Display_Info.InfoStr);
}

void Write_LEDs_OnDisplayBoard (uint8 LED_Buzz_bits)
{
	// Display board firmware has 'uint8 OverwriteLEDs' variable
	// the 'LED_Buzz_bits' function argument gets transferred to Display board's variable 'OverwriteLEDs' and can overwrite their status
	// if upper bit of OverwriteLEDs is set, it overwrites ON-OFF assigned by Display board firmware
	// #define DISP_LED_Auto_ON_BIT           Bit_0	// in Display_Info.Status. Use 65 to turn on, 64 to turn all LEDs off
	// #define DISP_LED_Alarm_ON_BIT          Bit_1	// in Display_Info.Status. Use 66 to turn on, 64 to turn all LEDs off
	// #define DISP_LED_Pulse_ON_BIT          Bit_2	// in Display_Info.Status. Use 68 to turn on, 64 to turn all LEDs off
	// #define DISP_Buzzer_ON_BIT             Bit_3	// in Display_Info.Status. Use 72 to turn on, 64 to turn all LEDs and Buzzer off
	// the tx/Rx LED is controlled directly by Comm board
	TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_SET_DISPLAY_LEDs, LED_Buzz_bits, 0);
#ifdef PC // for simulation only
	if (LED_Buzz_bits >= LEDsControlledByCommBrd) // if value is more than 64 - easier to test firmware setting 107 to turn all LESs on or 0 to disable overwrite
	{
		if (LED_Buzz_bits & DISP_LED_Auto_ON_BIT) { // Bit_0
			SetAutoLEDred;
		}
		else {
			SetAutoLEDoff;
		}
		if (LED_Buzz_bits & DISP_LED_Alarm_ON_BIT) { // Bit_1
			SetAlarmLEDred;
		}
		else {
			SetAlarmLEDoff;
		}
		if (LED_Buzz_bits & DISP_LED_Pulse_ON_BIT) { // Bit_2
			SetPulseLEDred;
		}
		else {
			SetPulseLEDoff;
		}
	}
#endif //PC
}

void SetCurrentLoopVoltagePoints()
{
	if (SysData.NV_UI.unit_type == UNIT_24V)
	{
		SysData.NV_UI.V20 = 36;								// = 24 * 1.5
		SysData.NV_UI.V4 = 18;								// = 24 * 0.75
	}
	else if (SysData.NV_UI.unit_type == UNIT_48V)
	{
		SysData.NV_UI.V20 = 72;								// = 48 * 1.5
		SysData.NV_UI.V4 = 36;								// = 48 * 0.75
	}
	else if (SysData.NV_UI.unit_type == UNIT_250V)
	{
		SysData.NV_UI.V20 = 360;							// = 250 * 1.44
		SysData.NV_UI.V4 = 180;								// = 250 * 0.72
	}
	else // it ought to be 125V
	{
		SysData.NV_UI.unit_type = UNIT_125V;				// Just in case, set default value UNIT_125V)
		SysData.NV_UI.V20 = 180;							// = 125 * 1.44
		SysData.NV_UI.V4 = 90;								// = 125 * 0.72
	}
}

/***************************************************************************
			   C H E C K  A L A R M S
****************************************************************************
DESCRIPTION: This routine checks the alarm limits and sets the appropriate
			 bits in the alarm status byte which is sent to the comm board
			 to be reported via serial comms.

Inputs:      high_bat_threshold_V_f, low_bat_threshold_V_f, plus_gf_threshold_V_f,
			 minus_gf_threshold_V_f, ripple_volts_threshold,
			 ripple_I_threshold_mA_f, alarm_delay_sec_f.
Outputs:     alarm_status
Notes:       alarm status bits:  NiZ AC_loss Ripple_I  Ripple_V  -GND_Fault  +GND_Fault  Low_Bat  High_Bat
Revisions:   02/07/16     REC       Created
			 08/10/21     REC       Removed code that prevented RVV alarm if pulser on.
			 20240228     IK        refactored the code
*/
/***************************************************************************/
/***************************************************************************
* The function checks Real Time Value against Threshold and
	if (RTV > Trheshold), sets AlarmBit after 'debounce_time_ms' in
	'alarm_detected' and 'latched_alarm_status';
	if (RTV <= Trheshold) clears AlarmBit after 'debounce_time_ms' in
	'alarm_detected'; if persistent alarm (latched alarm) is disabled,
	clears bit in 'latched_alarm_status'
***************************************************************************/

#define Set_alarm_if_value_ABOVE_threshold  1 // TRUE // = 1
#define Set_alarm_if_value_BELOW_threshold  0 // FALSE // = 0
#define Set_alarm_if_BIT_is_set  -1
/// <summary>
/// function sets/resets alarm bit after 'debounce_time_ms' in global vars 'latched_alarm_status' and 'alarm_detected'
/// </summary>
/// <param name="RealTimeValue">measured value</param>
/// <param name="threshold">boundary to check against</param>
/// <param name="SetAlarmIfCondition">above or below boundary?</param>
/// <param name="AlarmBit">which bit to set or clear</param>
/// <param name="debounce_time_ms">ms time to ignore; if condition lasts longer - set or reset bit</param>
void CheckAlarmBit( float RealTimeValue, float threshold, signed char SetAlarmIfCondition, uint16 AlarmBit, uint8* debounce_time_ms)
{
	uint16 debouncer = * debounce_time_ms;
	Boolean AlarmDetected = FALSE;
	if ((SetAlarmIfCondition == Set_alarm_if_value_ABOVE_threshold) && (RealTimeValue >= threshold))
		AlarmDetected = TRUE;
	if ((SetAlarmIfCondition == Set_alarm_if_value_BELOW_threshold) && (RealTimeValue <= threshold))
		AlarmDetected = TRUE;
	if ((SetAlarmIfCondition == Set_alarm_if_BIT_is_set) && (RealTimeValue != 0))
		AlarmDetected = TRUE;

	if (AlarmDetected == TRUE)
	{
		if (debouncer < ALARM_COUNTER_CAP_ms) debouncer++;		// increment but cap it at twice the DELAY_ms_BEFORE_ALARM_ACCEPTED
	}
	else
	{
		if (debouncer > 1) debouncer--;
	}
	if (debouncer >= DELAY_ms_BEFORE_ALARM_ACCEPTED)			// ? real persistent fault, not dissapering for DELAY_ms_BEFORE_ALARM_ACCEPTED
	{
		if ((SysData.NV_UI.disabled_alarms & AlarmBit) == 0)	// an alarm is enabled
		{
			setBit(Display_Info.alarm_status, AlarmBit);		// set detected alarm bit
			setBit(latched_alarm_status, AlarmBit);				// set latched alarm bit
		}
	}
	else
	{
		clearBit(Display_Info.alarm_status, AlarmBit);					// clear detected alarm bit
		if ((SysData.NV_UI.SavedStatusWord & Latch_ON_eq1_Bit) == 0)	// if not latched (persistent) alarms
		{
			clearBit(latched_alarm_status, AlarmBit);					// clear latched alarm bit
		}
	}
	*debounce_time_ms = debouncer;
}

//-!- IK20250729 this function should be called from 1 ms interrupt
// currently it is called from main loop
//-!- IK20250812 moved to interrupt
//-!- check timing
void Check_Alarms(void)
{
	float tmp_fault_volts;

	//****** Check for high battery alarm *******
	CheckAlarmBit(rt.OutData.measured.battery_voltage_f, SysData.NV_UI.high_bat_threshold_V_f, Set_alarm_if_value_ABOVE_threshold, Alarm_BatVoltageHIGH_Bit, &rt.h_battery_fault_cntr); // Bit_0

	//****** Check for low battery alarm *******
	CheckAlarmBit(rt.OutData.measured.battery_voltage_f, SysData.NV_UI.low_bat_threshold_V_f, Set_alarm_if_value_BELOW_threshold, Alarm_BatVoltageLOW_Bit, &rt.l_battery_fault_cntr);   // Bit_1

	//****** Check for plus ground fault alarm *******
	if (rt.OutData.measured.G_fault_voltage_f >= 0)
		tmp_fault_volts = rt.OutData.measured.G_fault_voltage_f;				// positive fault voltage
	else
		tmp_fault_volts = -rt.OutData.measured.G_fault_voltage_f;				// negative fault voltage

	CheckAlarmBit(tmp_fault_volts, SysData.NV_UI.minus_gf_threshold_V_f, Set_alarm_if_value_ABOVE_threshold, Alarm_MinusGND_FAULT_Bit, &rt.m_gf_cntr); // Bit_3

	// ****** Check for ripple voltage alarm *******
	CheckAlarmBit(rt.OutData.measured.ripple_mV_f, SysData.NV_UI.ripple_V_threshold_mV_f, Set_alarm_if_value_ABOVE_threshold, Alarm_Ripple_Voltage_Bit, &rt.rv_cntr);

	//****** Check for ripple current alarm *******
	CheckAlarmBit(rt.OutData.measured.ripple_mA_f, SysData.NV_UI.ripple_I_threshold_mA_f, Set_alarm_if_value_BELOW_threshold, Alarm_Ripple_Current_Bit, &rt.ri_cntr);

	//****** Check for Hi Z alarm **********
	if (SysData.NV_UI.disabled_alarms & Alarm_High_Impedance_Bit)	// if ((disabled_alarms & 0x08) == 0x08) // and not disabled
	{
		rt.high_impedance_cntr = 0;
	}
	else  // Hi Z alarm enabled
	{
		if ((rt.ri_cntr > DELAY_ms_BEFORE_ALARM_ACCEPTED) &&
			(rt.rv_cntr > DELAY_ms_BEFORE_ALARM_ACCEPTED))			// when high rv and low ri
		{
			if (rt.high_impedance_cntr < ALARM_COUNTER_CAP_ms) rt.high_impedance_cntr++;	//cap it
		}
		else
		{
			if (rt.high_impedance_cntr > 0) rt.high_impedance_cntr--;
		}

		if (rt.high_impedance_cntr > DELAY_ms_BEFORE_ALARM_ACCEPTED)
		{
			setBit(Display_Info.alarm_status, Alarm_High_Impedance_Bit);		// alarm_status |= 0x80;        // set Hi-Z alarm
			setBit(latched_alarm_status, Alarm_High_Impedance_Bit);				// latched_alarm_status |= 0x80;  // set Hi-Z alarm
		}
		else
		{
			clearBit(Display_Info.alarm_status, Alarm_High_Impedance_Bit);		// alarm_status &= 0x7F;
			if ((SysData.NV_UI.SavedStatusWord & Latch_ON_eq1_Bit) == 0)		// if not latched (persistent) alarms
			{
				clearBit(latched_alarm_status, Alarm_High_Impedance_Bit);		// latched_alarm_status &= 0x7F;
			}
		}
	}

	//****** Check for AC fail alarm *******
	CheckAlarmBit((float)(relay_board_status & RELAY_BRD_AC_FAIL_BIT), 0, Set_alarm_if_BIT_is_set, Alarm_High_Impedance_Bit, &rt.ac_cntr);

	//******* Alarm Housekeeping **********

	//Flash LED for  AC Fail, HI-Z, Ripple I, and Ripple V
	if (latched_alarm_status & AlarmStatus_Instant_BITS)
	{
		if (Display_Info.Info_segments_on_off == ON)	// time to flash
		if (timer.AlarmLED_blink == 0)
		{
			if (Display_Info.Status & DISP_LED_Alarm_ON_BIT)		// is it on?
				Display_AlarmLED_OFF;					// turn off alarm lED
			else
				Display_AlarmLED_ON;					// turn it on
			timer.AlarmLED_blink = ALARM_BLINK_RATE_ms;	// 0.3 sec
		}
		if (SysData.NV_UI.SavedStatusWord & Latch_ON_eq1_Bit)	// if latched (persistent) alarms
			Display_Buzzer_ON; // PORTF &= 0xFE;		// turn it on
		else
			Display_Buzzer_OFF;							// turn off buzzer
	}
	else			//if not one of the flashing types then other alarm active
	{
		if (latched_alarm_status == 0)
		{
			Display_AlarmLED_OFF;						// turn off alarm lED
			Display_Buzzer_OFF;							// turn off buzzer
		}
		else		// if not latched (persistent) alarms
		{
			Display_AlarmLED_ON;						// turn led on
			if (SysData.NV_UI.SavedStatusWord & Buzzer_ON_eq1_Bit)	// if alarm buzzer is enabled
				Display_Buzzer_ON;						// turn it on
			else
				Display_Buzzer_OFF;						// turn buzzer off
		}
	}
	if (relay_board_status & RELAY_BRD_REQ_RESET_BIT)	// ext. reset from relay board?
	{
#ifndef PC
		Display_Info.alarm_status = 0;					// ext reset clear alarms
		latched_alarm_status = 0;
		Display_Buzzer_OFF;								// turn buzzer off
//-!- IK20240227 disabled for test        display_mode = VOLTS;
#endif // not PC
	}

	if (limit_mode == TRUE)								// if limit mode kill any alarms
	{													// so you can view the settings
		//latched_alarm_status = 0;
		//clearBit(alarm_detected, AlarmStatus_ALL_BITS);
	}
	Display_Info.alarm_status = latched_alarm_status;
} // end Check_Alarms()

/********************************************************************/
/*                      S E N D    2 3 2                            */
/********************************************************************/
/*  Description:  This routine sends the characters in the msg string
				and sets the parity bit for each character depending
				on what the parity is set to. It also enables the
				RS485 bus transmitter and puts it back to receive
				when done.
	IK20250811	blocking function; processor waits for transmission to finish
	Inputs:     pointer to string.
	Outputs:    None
	Notes:      None
	Revisions:  09/25/99     REC    Created
				10/09/07     REC    Changed order of CLearing TXC
									added disabling int's while
									writing to the UART register.
				1/23/08      REC    Lengthened prop delay time to
									100 us. (xmt enable to send)
				20240122     IK     Port PD2 (U1.pin11) controls XMT_ENABLE
*/
/********************************************************************/

void Send_232(uint8 chars_to_send)
{
	uint8 wt;
#ifndef PC
	WATCHDOG_RESET();
	if (SysData.NV_UI.StartUpProtocol == SETUP)
		timer.CommChHoldOff = 5;
	else
		timer.CommChHoldOff = SysData.xmt_delay;// set by SysData.xmt_delay
	do {										// waiting more than 0.015 sec
		_NOP();									// IK2024011 added NOP(), wait for prop delays
	} while (timer.CommChHoldOff != 0);			// wait for channel to free
	setBit(XMT_ENABLE, XMT_ON);					// set RS485 chip to transmit
	clearBit(UCSR0B, RXEN);						// UCSR0B = UCSR0B & 0xEF;    disable RS232 Receiver *****
	setBit(XMT_ENABLE, XMT_ON);					// RS485 chip to xmt

	for (wt = 255; wt > 0; wt--)				// IK2024011 reverse count ends with wt=0, avoiding wt=0; assignment
	{
		_NOP();									// wait for prop delays
		_NOP();									// wait for prop delays
		_NOP();									// wait for prop delays
	}
	//wt = 0;
	WATCHDOG_RESET();
	do
	{
		_NOP();									// kill a little time
		_NOP();
		_NOP();
		_NOP();
		__disable_interrupt();
		setBit(UCSR0A, TXC);					// write to TXC, (which is Bit_6 = 0x40) to clear it
		UDR0 = wrk_str[wt];						// put char in UART buffer
		__enable_interrupt();
		while ((UCSR0A & TXC) == 0) {
			_NOP();
		};			//wait till done; TXC bit is set when the entire frame in the Transmit Shift Register has been shifted out
		wt++;									// increment to next item
		_NOP();									// kill a little time
		_NOP();
		_NOP();
		_NOP();
	} while (wt <= (chars_to_send - 1));

	clearBit(XMT_ENABLE, XMT_ON);				// set RS485 chip to receive, xmt off
	setBit(UCSR0B, RXEN);						// UCSR0B = UCSR0B | 0x10; enable RS232 RCV
	for (wt = 10; wt > 0; wt--)					// IK20250714 added some time after switching to receive mode
	{
		_NOP();                                  //wait for prop delays
	}
#else // for PC
	cputs("RawStr:");
	for (wt = 0; wt < chars_to_send; wt++)
	{
		putch(wrk_str[wt]);						// put char on console as binary; printable carectars will aper as string but DNP and ModBus would appear as garbage
	}
	cputs("\r\nHex:");
	for (wt = 0; wt < chars_to_send; wt++)
	{
		printf(" %02X",wrk_str[wt]);			// put char on console as binary; printable carectars will aper as string but DNP and ModBus would appear as garbage
	}
	printf("\r\nSent %d bytes\r\n", chars_to_send);
#endif
} // end of Send_232()

/********************************************************************/
/*                 S E N D   S E T U P    M S G                     */
/********************************************************************/
/*  DESCRIPTION: This routine builds the various messages to be sent
				 while in the setup SysData.NV_UI.StartUpProtocol.
				 It then sets the "send" variable to send nothing so
				 the message is only sent once.

   Inputs: send setup,type
   Outputs: output message string (wrk_str)
   Notes:

   Revision:    05/26/15      REC     Created.
*/
/********************************************************************/


#ifdef UNI_BI_POLAR_INPUTS
// IK2024011 'inp' is an index starting from 1, i.e. replaces former 'input_2' with input_type[2], where argument passed to function is 2
// the input_type[0] is former variable 'analog_points'
// ALWAYS input_type[1] = BIPOLAR;  //UNIPOLAR removed 5-24-2018, force it
int SetInputType(uint8 inp) {
	uint8 InpState = SysData.input_type[inp]; // zero == BIPOLAR, one == UNIPOLAR input
	int chars = sprintf((char*)&wrk_str[0], "\x1BR%d1\r", inp - 1); // passing index 1 to function ( which replaces former 'input_1') creates "ESC+R0"+'0'
	return chars;
}
#endif // UNIPOLAR_INPUTS

void Send_Setup_Msg (uint8 type)
{
	//uint16 tmp_baud;
	uint8 bytes;
	bytes = 5;										// how many bytes to send
	if (type == SEND_NUM_OF_POINTS)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRC%d\r", SysData.analog_points);// returns how many bytes to send
		//bytes = sprintf((char*)&wrk_str[0], "\x1BRC5\r");	// IK20250715 analog points were depricated, here is backward compatibility return
	}
#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
	if ((type >= SEND_INPUT1) && (type <= SEND_INPUT5))
	{
		uint8 inp = type - SEND_INPUT1;
		bytes = SetInputType(inp);// sets in wrk_str[0] ESC+R0+'0' or '1' = character indicating Bipolar or unipolar
	}
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
	//if (type == SEND_INPUT2)
	//{
	//	bytes = SetInputType(2);// sets in wrk_str[0] ESC+R1+'0' or '1' = character indicating Bipolar or unipolar
	//}
	//if (type == SEND_INPUT3)
	//{
	//	bytes = SetInputType(3);// sets in wrk_str[0] ESC+R2+'0' or '1' = character indicating Bipolar or unipolar
	//}
	//if (type == SEND_INPUT4)
	//{
	//	bytes = SetInputType(4);// sets in wrk_str[0] ESC+R3+'0' or '1' = character indicating Bipolar or unipolar
	//}
	//if (type == SEND_INPUT5)
	//{
	//	bytes = SetInputType(5);// sets in wrk_str[0] ESC+R4+'0' or '1' = character indicating Bipolar or unipolar
	//}

	if (type == SEND_PROTOCOL)
	{
		wrk_str[0] = 0x1B;							// ESC
		wrk_str[1] = 'R';
		wrk_str[2] = 'D';							// character indicating SysData.NV_UI.StartUpProtocol
		// IK20260428 rt.operating_protocol is always SETUP there if (rt.operating_protocol < ASCII_CMDS)
		wrk_str[3] = SysData.NV_UI.StartUpProtocol; // show SAVED protocol, not active
		//else // error, should not be here
		//	wrk_str[3] = '0';						// return 0
		wrk_str[4] = 0x0D;							// CR
		bytes = 5;									// how many bytes to send
	}
	if (type == SEND_DLL_CONFIRM)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRL%d\r", SysData.dll_confirm);		// returns how many bytes to send
	}
	if (type == SEND_APP_CONFIRM_STATUS)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRP%d\r", SysData.app_confirm);		// returns how many bytes to send
	}
	if (type == SEND_DLL_RETRIES)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRE%d\r", SysData.dll_retries);		// sets in wrk_str[0] "ESC+RE"+ number
	}
	if (type == SEND_INTER_CHAR)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRG%d\r", SysData.inter_char_gap);
	}
	if (type == SEND_HOST_ADDR)														// also used for modbus first register
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRH%.0f\r", SysData.NV_UI.host_address);
	}
	if (type == SEND_MODBUS_FIRST_REG)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRF%.0f\r", SysData.NV_UI.modbus_first_reg);
	}
	if (type == SEND_METER_ADDR)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRM%.0f\r", SysData.NV_UI.meter_address);
	}
	if (type == SEND_DLL_TIMEOUT)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRS%d\r", SysData.dll_timeout);
	}
	if (type == SEND_BAUD_RATE)
	{
		Uint32 tmp_baud = (Uint32)Existing.baud_rate;
		bytes = sprintf((char*)&wrk_str[0], "\x1BRR%d\r", tmp_baud);				// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.
	}
	if (type == SEND_XMT_DELAY)
	{
		bytes = sprintf((char*)&wrk_str[0], "\x1BRW%d\r", SysData.xmt_delay);
	}

	Send_232(bytes);																// send it
	send_setup = 0;
	comm_state = RCVANDXMT;  // in enum UART_Events
}

// IK20250717 verified on FW 826-501A_v_K_event_corected.c using Battery Monitor Setup C#
int Add_ModBus_BinaryLongToWorkString(int start_pos,long value)
{	// handles both positive and negative values (e.g., -268523 mV):
	// Split into integer and fractional parts
	int integerValue = (int)(value / 1000);          // e.g., -268
	// In C, (abs(value) % 1000)) and abs(value % 1000) are fine, but abs(value % 1000) is slightly more direct and efficient - we don't compute the full abs(value).
	int fractionValue = (int)(abs(value % 1000));    // e.g., 523, always positive

	// Transmit integer part (MSB first)
	wrk_str[start_pos++] = (uint8)(integerValue >> 8);
	wrk_str[start_pos++] = (uint8)(integerValue & 0xFF);

	// Transmit fractional part (always positive, MSB first)
	wrk_str[start_pos++] = (uint8)(fractionValue >> 8);
	wrk_str[start_pos] = (uint8)(fractionValue & 0xFF);

	return start_pos;
}

/*********************************************************************/
/*          S E N D   M O D B U S   E X C E P T I O N                */
/*********************************************************************/
/* Helper function to send Modbus exception responses.
   Reduces code duplication for error cases.

   Inputs:  exception_code - Modbus exception code:
			0x01 = Illegal function
			0x02 = Illegal address  
			0x03 = Illegal data
			0x04 = Device failure
			0x06 = Device busy
   Outputs: Sends 5-byte exception response via RS-485

   Revision: 2025-May-01  IK  Created from refactored code
*/
static void Send_Modbus_Exception(uint8 exception_code)
{
	wrk_str[0] = (uint8)SysData.NV_UI.meter_address;
	wrk_str[1] = wrk_str[1] | 0x80;						// exception response flag
	wrk_str[2] = exception_code;
	Calc_ModbusCRC(3);
	wrk_str[3] = crc >> 8;
	wrk_str[4] = crc & 0x00FF;
	Send_232(5);
}

/*********************************************************************/
/*                      S E N D   M O D B U S   M S G                */
/*********************************************************************/
/*  DESCRIPTION: This routine builds the various messages to be sent.
				 and then sets the "send" variable to send nothing so
				 the message is only sent once.

	Inputs: send_modbus
	Outputs: output message string
	Notes:

	Revision:    05/29/15      REC     Created.
	IK20250714
	Modbus itself only defines 16-bit register access (Read Holding Registers, etc.).
	For 32-bit values, how those rt.registers are packed is not standardized in the Modbus spec - it's vendor- or implementation-specific.

	It appears that ElectroSwitch transfers:
	in upper 2-bytes --- (word millivolts / 1000), i.e., round volts, MSB first
	in lower 2-bytes --- (word millivolts); with resolution 1 mV equals one count. the 523 mV is saved and transfered as 523

	IK20260501 Refactored exception responses into helper function

	*********************************************************************/
void Send_Modbus_Msg(uint8 type)
{
	uint8 i, bytes;
	Int32 battery_milliVolts = (Int32)(rt.OutData.measured.battery_voltage_f * V_to_mV);	// for DNP and Modbus transmission, 1 mV = 1 count
	Int32 fault_millivolts = (Int32)(rt.OutData.measured.G_fault_voltage_f * V_to_mV);		// convert float to int for fault milli voltage
	Int32 minus_gnd_millivolts = (Int32)(rt.OutData.measured.minus_gnd_volts_f * V_to_mV);	// convert float to int for minus gnd milli voltage
	Int32 ripple_milliAmpers = (Int32)(rt.OutData.measured.ripple_mA_f);					// convert float to int for ripple current
	Int32 ripple_milliVolts = (Int32)(rt.OutData.measured.ripple_mV_f);						// convert float to int for ripple voltage
	uint16 address = (uint16) SysData.NV_UI.meter_address;
	switch (type)
	{
	case MODBUS_SEND_NOTHING:
		break;
	case SEND_CMD_ECHO:										// just echo received msg
		Send_232(8);										// send it
		break;
	case SEND_DATA:											// build address msg
		wrk_str[0] = address & 0xFF;	//-!- K20240113 meter address is uint16, high byte was not sent
		//IK20240115 ModBus has only 255 addresses.
		//IK20260501 do not overwrite [1], it is modbus identifier == 3! wrk_str[1] = 0;	//SysData.NV_UI.meter_address >> 8;	//-!- K20240113 send higher byte

		wrk_str[2] = rt.registers * 4;						// how many bytes are coming

		for (i = 0; i < rt.registers; i++) 				// 'rt.registers' comes from Modbus request, in function Parse_Modbus_Request() active channels to transmit, valid range 1-5
		{
			if (i == 0)										// if 1st reg then send status
			{
				wrk_str[3] = 0;              // pad byte
				wrk_str[4] = 0;              // pad byte
				wrk_str[5] = (uint8)Display_Info.alarm_status>>8;	// IK20250811 added upper byte //-!- IK20240113 add upper byte of alarm_status2 pad byte
				wrk_str[6] = (uint8)Display_Info.alarm_status;		// alarm status lower byte
			}
			if (i == 1)										// if 2nd reg then send batt volts
				Add_ModBus_BinaryLongToWorkString(3 + (1 * 4), battery_milliVolts);
			if (i == 2)										// if 3rd reg then send fault volts
				Add_ModBus_BinaryLongToWorkString(3 + (2 * 4), fault_millivolts);
			if (i == 3)										// if 4th reg then send minus gnd volts
				Add_ModBus_BinaryLongToWorkString(3 + (3 * 4), minus_gnd_millivolts);
			if (i == 4)										// if 5th reg then send ripple volts
				Add_ModBus_BinaryLongToWorkString(3 + (4 * 4), ripple_milliVolts);
			if (i == 5)										// if 6th reg then send ripple current
				Add_ModBus_BinaryLongToWorkString(3 + (5 * 4), ripple_milliAmpers);
		}

		Calc_ModbusCRC(3 + (i * 4));						// calc the crc on right num of bytes
		wrk_str[7 + ((i - 1) * 4)] = crc >> 8;				// append the crc
		wrk_str[8 + ((i - 1) * 4)] = crc & 0x00FF;
		bytes = 5 + (rt.registers * 4);
		Send_232(bytes);									// send it
		break;
	case NOT_SUPPORTED_MODBUS:
		Send_Modbus_Exception(0x01);					// illegal function
		break;
	case ILLEGAL_DATA:
		Send_Modbus_Exception(0x03);					// illegal data
		break;
	case ILLEGAL_ADDRESS:
		Send_Modbus_Exception(0x02);					// illegal address
		break;
	case SLAVE_DEVICE_BUSY:
		Send_Modbus_Exception(0x06);					// device busy
		break;
	case SLAVE_DEVICE_FAILURE:
		Send_Modbus_Exception(0x04);					// device failure
		break;
	default:												// none of the above
		break;												// so do nothing
	}
	send_modbus = SEND_NOTHING;								// reset send msg type
	// IK20231214 not checked //meter_status = meter_status & 0xFD;          //clr dev trouble bit
	comm_state = RCVANDXMT;								// in enum UART_Events
}


/*********************************************************************/
/*                    P A R S E   M O D B U S   M S G                */
/*********************************************************************/
/* Description: This function is called by the top level if the
				RS232 interrupt routine has built a complete message.
				this function will parse the incoming message a perform
				the correct operation and response as described in the
				MODBUS RTU specification and the Electroswitch SDS
				document.
   Inputs:      pointer to scada_msg
   Outputs:     scada_op, send
   Notes:

   Revision:    05/12/15        REC     Created.
																	 */
/*********************************************************************/
void Parse_Modbus_Msg(void)
{
	uint8 i, broadcast;
	uint8 msg;
	uint16 received_crc = 0;
	// volatile float tmp_cal;

	comm_state = RCV;								// in enum UART_Events
	timer.Generic = 17;

	if (Existing.baud_rate >= Baud_9600)			// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.
		timer.Generic = 2;
	if (Existing.baud_rate <= Baud_600)			// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.
		timer.Generic = 600;
	while (timer.Generic != 0) {						// kill some time for more chars
		_NOP();
	};

	msg = ModBusGOOD;
	broadcast = false;
	send_modbus = SEND_NOTHING;
	// ---------------- Check address first -------------

	// ---------------- Is it intended for this CIM -----
	if ((rt.HostRxBuff[0] == SysData.NV_UI.meter_address) || (rt.HostRxBuff[0] == 0))
	{
		if (rt.HostRxBuff[0] == 0)						// find out if a Broadcast msg
			broadcast = true;							// cause you don't reply
		if (num_of_inbytes <= 49)
			for (i = 0; i <= num_of_inbytes; i++)		// copy incoming data to outgoing data
				wrk_str[i] = rt.HostRxBuff[i];
		_WDR();

		/*----------------- Check CRC ----------------------*/
		if (num_of_inbytes > 1)
		{
			received_crc = rt.HostRxBuff[num_of_inbytes - 2];
			received_crc = received_crc << 8;
			received_crc = received_crc | rt.HostRxBuff[num_of_inbytes - 1];
		}
		else
			received_crc = 0;
		Calc_ModbusCRC(num_of_inbytes - 2);				// take 2 off for CRC bytes
		if ((received_crc != crc) || (parity_error == true))
			msg = ModBusBAD;
		else
			msg = ModBusGOOD;
		rt.device_register = wrk_str[2] * 256 + wrk_str[3];
		rt.registers = wrk_str[4] * 256 + wrk_str[5];

		switch (wrk_str[1])								// get function
		{
		case NO_FUNCTION:								// no function 0
			break;

		case READ_HOLDING_REGISTERS:					// case / Function 3 Read holding regs IK20250718 for BatMon, holding rt.registers contain votage and current analog values
		case READ_INPUT_REGISTERS:						// case / Function 4 Read Input Registers
			if (msg == ModBusGOOD)						// CRC check passed
			{
				send_modbus = ILLEGAL_ADDRESS;			// asked for addr not implemented
				if (rt.device_register == rt.first_register)	// reg addr is equal to 1st reg
				{
					rt.registers = rt.registers >> 1;			// divide by 2
					if ((rt.registers <= SysData.analog_points + 1) && // number of active points <= active?
						((rt.registers >= 1) && (rt.registers <= 6)))
					{									// the 1 for fault alarms
						send_modbus = SEND_DATA;		// send the voltage values
					}
					else
						send_modbus = ILLEGAL_DATA;		// asked for more points than active
				}
			}
			else										// doesn't implement for this function.
				send_modbus = SEND_NOTHING;				// send no reply cause msg was bad
			break;

		case DIAGNOSTICS:								// Function 8 Diagnostics
			if (msg == ModBusGOOD)						// if msg is good
			{
				if (rt.device_register == 0)
				{										// if subfunction is return
					send_modbus = SEND_CMD_ECHO;		// query data then echo cmd
				}
				else									// else some other subfunction
					send_modbus = NOT_SUPPORTED_MODBUS;	// so send not supported
			}
			else										// msg wasn't good so don't
				send_modbus = SEND_NOTHING;				// reply
			break;

		case PRESET_SINGLE_REGISTER:					// IK20260302 Function 6 Write Single Register
			if (msg == ModBusGOOD)						// CRC check passed
			{
				if (rt.device_register == ByteCmdSetAsciiProtocol)		// register 0x30 = 48: switch to ASCII_CMDS protocol
				{
					// low byte of rt.registers carries the new protocol code (ignored here - register address selects protocol)
					send_modbus = SEND_CMD_ECHO;				// echo FC06 frame as success response
					rt.operating_protocol = ASCII_CMDS;
					//SaveToEE(SysData.NV_UI.StartUpProtocol);	// do not persist to EEPROM
					display_mode = SELECT_PROTOCOL; // Front_menu.c::DisplayPrepare() will show LED message
				}
				else if (rt.device_register == ModbusCmdSetDNPprotocol)	// register 0x33 = 51d: switch to DNP3 protocol
				{
					send_modbus = SEND_CMD_ECHO;				// echo FC06 frame as success response
					SysData.NV_UI.StartUpProtocol = DNP3;		//-!- IK20260319 save to EEPROM, but do not switch protocol yet
					SaveToEE(SysData.NV_UI.StartUpProtocol);	// persist to EEPROM
					display_mode = SELECT_PROTOCOL;		// Front_menu.c::DisplayPrepare() will show LED message and change SysData.NV_UI.StartUpProtocol
				}
				else
					send_modbus = ILLEGAL_ADDRESS;		// register not implemented
			}
			else
				send_modbus = SEND_NOTHING;				// bad CRC - no reply
			break;

		default:										// not any of above functions;
			// IK20250718 cases below are replaced by default case
			//case READ_COIL_STATUS:						// Function 1  READ COIL STATUS
			//case READ_INPUT_STATUS:						// Function 2  READ INPUT STATUS
			//case FORCE_SINGLE_COIL:						// Function 5  Force single coil
			//case READ_EXCEPTION_STATUS:					// Function 7  Read Exception Status
			//case PROGRAM_484:								// Function 9  Program 484
			//case POLL_484:								// Function 10 POLL 484
			//case FETCH_COMM_EVENT_COUNTER:				// Function 11 Fetch Comm Event Cntr
			//case FETCH_COMM_EVENT_LOG:					// Function 12 Fetch Comm Event Log
			//case PROGRAM_CONTROLLER:						// Function 13 Program Controller
			//case POLL_CONTROLLER:							// Function 14 Poll Controller
			//case FORCE_MULTIPLE_COILS:					// Function 15 Force Multiple Coils
			//case PRESET_MULTIPLE_REGISTERS:				// Function 16 Preset Multiple Regs
			if (msg == ModBusGOOD)						// good msg but this function
				send_modbus = NOT_SUPPORTED_MODBUS;		// is not a supported in this product
			else
				send_modbus = SEND_NOTHING;				// bad msg so don't reply
			break;

		}//END SWITCH STATEMENT
		if (broadcast == true)							// don't reply to Broadcast msgs
			send_modbus = SEND_NOTHING;
	}//END ADDRESS MATCH
	else{
		//debug = 11;
	}
	// --------------------- End of command checks ----------
	ClearRxBuffer(0x00);						// at this point all parsing is done so clear out the buffer with nonsense
}

/*********************************************************************/
/*                P A R S E   S E T U P   M S G                      */
/*********************************************************************/
/*  Description:  This routine parses the setup msg and cause the
				  required action to occur.

	Inputs:       rt.HostRxBuff.
	Outputs:      Send
	Notes:        None
	Revisions:    05/18/15     REC     Created
																	 */
/*********************************************************************/
void Parse_Setup_Msg(void)
{
	uint8 tmpByte = 0;

	timer.Generic = 15;										// wait 15 ms for all chars to come in
	while (timer.Generic != 0) {
		_NOP();
	};
	if ((rt.HostRxBuff[3] == 'R') || (rt.HostRxBuff[3] == 'W'))	// Handle the ones you can read or write
	{
		comm_state = RCV; // in enum UART_Events
		timer.start_up_ms = 300000;							// keeps it in this SysData.NV_UI.StartUpProtocol for 5 more minutes
		switch (rt.HostRxBuff[4])							// get command
		{
		case 'C':								// "WC###"	// number of analog points
			send_setup = SEND_NUM_OF_POINTS;				// send number of analog points
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				tmpByte = rt.HostRxBuff[5] - 0x30;			// get num of points and convert to decimal
				if (tmpByte > 5) tmpByte = 5;				// max is 5 points
				// IK20250718 this is just gimmick to convert 1,2,3,4,5 points to special values 0x11, 0x22, 0x33, 0x44, 0x55 !
				tmpByte = (tmpByte << 4) | (tmpByte & 0x0F);// store as special value,
				SysData.analog_points = tmpByte;			// store number of analog points
				SaveToEE(SysData.analog_points);			// EEPROM_Write_byte(adr_NUM_OF_POINTS, tmpByte);      // EEPROM[165) num of channels
			}
			break;
		case 'D':								// "WD###"	// SysData.NV_UI.StartUpProtocol
			tmpByte = rt.HostRxBuff[5];
			send_setup = SEND_PROTOCOL;						// send SysData.NV_UI.StartUpProtocol
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if ((tmpByte == '1') || (tmpByte == '2'))	// Set SysData.NV_UI.StartUpProtocol
				{
					SysData.NV_UI.StartUpProtocol = tmpByte - '0';	// char '1' is conveted to binary 1 which is DNP3
					SaveToEE(SysData.NV_UI.StartUpProtocol);		// char '2' is conveted to binary 2 which is Modbus
				}
				display_mode = SELECT_PROTOCOL;							// IK20260212 show change on display
				// SysData.NV_UI.StartUpProtocol = SysData.NV_UI.StartUpProtocol; // IK20260205 commented out because this would kick out of setup
			}
			break;
		case 'E':								// "WE###" DLL number of retries, IK20250805, Battery Monitor Setup never sets it, just reads from BM and sends it back
			send_setup = SEND_DLL_RETRIES;					// send DLL Retries
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					int tmp_int = atol(&rt.HostRxBuff[5]);
					if ((tmp_int < 0) || (tmp_int > 255))	//-!- IK20250811 what is the max number of retries?
						break;
					else
						SysData.dll_retries = tmp_int;
				}

				SaveToEE(SysData.dll_retries);				// Store_Parameter(DLL_NUM_OF_RETRIES, SysData.dll_retries);//store this new value
			}
			break;
		case 'F':								// IK20260501 new command "WF###" Starting register for modbus or host address for DNP
			send_setup = SEND_MODBUS_FIRST_REG;				// first addr
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					long tmp_long = atol(&rt.HostRxBuff[5]);
					if ((tmp_long < 0) || (tmp_long > 255))
						break;
					else
						SysData.NV_UI.modbus_first_reg = (uint8)tmp_long;
				}
				SaveToEE(SysData.NV_UI.modbus_first_reg);		// store this new value
			}
			break;
		case 'G':							// "WG###" Inter character gap (used for Modbus), IK20250805 limited in Battery Monitor Setup to 65500 ms
			send_setup = SEND_INTER_CHAR;					// send Inter character gap
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					long tmp_long = atol(&rt.HostRxBuff[5]);
					if ((tmp_long < 0) || (tmp_long > 65500))
						break;
					else
						SysData.inter_char_gap = (uint16)tmp_long;
				}
				SaveToEE(SysData.inter_char_gap);			// Store_Parameter(INTER_CHAR, SysData.inter_char_gap);//store this new value
			}
			break;
		case 'H':								// "WH###" Starting register for modbus or host address for DNP
			send_setup = SEND_HOST_ADDR;					// send Host / first addr
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					long tmp_long = atol(&rt.HostRxBuff[5]);
					if ((tmp_long < 0) || (tmp_long > 65500))
						break;
					else
						SysData.NV_UI.host_address = (uint16)tmp_long;
				}
				SaveToEE(SysData.NV_UI.host_address);		// Store_Parameter(HOST_ADDR, SysData.NV_UI.host_address);  //store this new value
			}
			break;
		case 'L':											// DLL Confirm Status
			send_setup = SEND_DLL_CONFIRM;					// send dll confirm status
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				SysData.dll_confirm = (rt.HostRxBuff[5] - 0x30);	// Get dll confirm status
				SaveToEE(SysData.dll_confirm);				// Store_Parameter(DLL_CONFIRM, SysData.dll_confirm);	// store this new value
			}
			break;
		case 'M':								// "WM###" Meter address
			send_setup = SEND_METER_ADDR;					// send meter address
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					long tmp_long = atol(&rt.HostRxBuff[5]);
					if ((tmp_long < 0) || (tmp_long > 65500))
						break;
					else
						SysData.NV_UI.meter_address = (uint16)tmp_long;
				}
				SaveToEE(SysData.NV_UI.meter_address);		// Store_Parameter(METER_ADDR, SysData.NV_UI.meter_address);//store this new value
			}
			break;
		case 'P':								// "WP" Application confirm status/SysData.UART_parity
			send_setup = SEND_APP_CONFIRM_STATUS;			// send status of application confirms
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				SysData.app_confirm = rt.HostRxBuff[5] - 0x30;	// get confirm status for DNP & convert to decimal
				rt.protocol_parity = SysData.app_confirm;		// UART_parity for modbus
				SaveToEE(SysData.app_confirm);					// Store_Parameter(APP_STATUS, SysData.app_confirm);// store this new value/UART_parity in modbus units
			}
			break;
		case 'S':								// "WS###" dll timeout in ms
			send_setup = SEND_DLL_TIMEOUT;					// send DLL Timeout
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					int tmp_int = atol(&rt.HostRxBuff[5]);
					if ((tmp_int < 0) || (tmp_int > 300))
						break;
					else
						SysData.dll_timeout = tmp_int;
				}
				SaveToEE(SysData.dll_timeout);				// Store_Parameter(DLL_TIMEOUT, SysData.dll_timeout);//store this new value
			}
			break;

		case 'W':							// "WW###" XMT Timeout, IK20250805 limited in Battery Monitor Setup to 300 ms
			send_setup = SEND_XMT_DELAY;					// send xmt timeout
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					int tmp_int = atol(&rt.HostRxBuff[5]);
					if ((tmp_int < 0) || (tmp_int > 999))
						break;
					else
						SysData.xmt_delay = tmp_int;
				}

				SaveToEE(SysData.xmt_delay);				// Store_Parameter(XMT_DELAY, SysData.xmt_delay);//store this new value
			}
			break;
		case 'R':								// "WR###" Baud rate
			send_setup = SEND_BAUD_RATE;					// send Baud Rate
			if (rt.HostRxBuff[3] == 'W')					// is it a write?
			{
				if (Is_Numeric(&rt.HostRxBuff[5]) == true)	// can be converted into numeric
				{
					Uint32 tmp_int = atol(&rt.HostRxBuff[5]);
					if ((tmp_int < Baud_300) || (tmp_int > Baud_115200))  //-!- IK20260114 NEED TO SAVE Existing.BRate_index
						break;
					else
					{
						int result = Find_BR_index (tmp_int); // IK20260114 find index corresponding to baud rate
						if (result == INVALID_BR_INDEX)
							break;

						Save_BaudRate_To_EEPROM(result); // -!- IK20260204 permanently saves nex BR_index and new baudrate into SysData.NV_UI
					}
				}

				// Set operating baud and inter char gap to match IK20250206 removed ifs, substituted with calculations
				// Formula: SysData.inter_char_gap = 20 millions / baug_rate == 2.0E7 / Existing.baud_rate
				// but, 2.0E7 does not fit into uint16.
				SysData.inter_char_gap = (uint16)(((long)20000000)/ Existing.baud_rate);

				SaveToEE(SysData.inter_char_gap);	// Store_Parameter(INTER_CHAR, SysData.inter_char_gap);//store this new value since baud changed
				SaveToEE(SysData.NV_UI.baud_rate);	// Store_Parameter(TWI_MSG_DISPLAY_BAUD, SysData.NV_UI.baud_rate); //store this new value
			}
			break;
		default:									// not any of above functions
			send_setup = SEND_NOTHING;				// don't reply
			break;
		}
	} // End of Read or Write
	else	//a COMMAND instead
	{
		if (rt.HostRxBuff[3] == 'T')				// put in test mode?
		{
			//    test_mode = true; // IK20231214 not checked
			timer.start_up_ms = 300000;				// keeps it in this SysData.NV_UI.StartUpProtocol for 5 more minutes
		}

		if (rt.HostRxBuff[3] == 'X')				// exit test mode?
		{
			timer.start_up_ms = 0;					// causes top level to switch to operating params
		}
		if (rt.HostRxBuff[3] == 'I')				// init to defaults?
		{
			send_setup = SEND_NOTHING;				// send nothing
			timer.start_up_ms = 300000;				// keeps it in this SysData.NV_UI.StartUpProtocol for 5 more minutes
			Init_Parameters();						// Initialize to defaults
		}
	}
	ClearRxBuffer(0x00);						// at this point all parsing is done so clear out the buffer with nonsense
} //End of Parse Setup

/*********************************************************************/
/*                M E A S U R E                                      */
/*********************************************************************/
/*  Description:  This routine will setup the MUX to measure the
				  voltage requested. It will send the IsquareC commands
				  to the A/D converter to start the conversion. if the
				  conversion has started then it will keep checking
				  everytime called and if the conversion is ready it will
				  request the value and store it in the proper place holder.
	Inputs:       ADC_Status
	Outputs:      Battery_voltage, Fault_Voltage, Bus_to_Gnd,Fault_value
				  5_volts, GND_Ref,EXT_Analog
	Notes:        None
	Revisions:    06/17/15     REC     Created                       */
	/*********************************************************************/
float LinInterpolation1(float inp,
	float X1, // = CalStructPtr->Y1_lowCalibrVal;
	float X2, // = CalStructPtr->Y2_highCalibrVal;
	float Y1, // = CalStructPtr->X1_lowADCcounts;
	float Y2) // = CalStructPtr->X2_highADCcounts;
{
	return (X1 + ((X2 - X1) * (inp - Y1) / (Y2 - Y1)));
}

float t_float=0;
//volatile Calibr2pnts* sPtr; // 20231221 volatile global does not help >>>IAR bug> on first assignment,  CalStructPtr becomes zero instead of 0x140
float LinInterpolation(float inp, Calibr2points* CalStructPtr)
{
	Calibr2points* sPtr = CalStructPtr;
//                                     V_bat     GND     mV_rip   Irip_mA
	float X1 = sPtr->Y1_lowCalibrVal;  //  90.0          40.0
	float X2 = sPtr->Y2_highCalibrVal; // 130.0         300.0
	float Y1 = sPtr->X1_lowADCcounts;  // 13367          27
	float Y2 = sPtr->X2_highADCcounts; // 21736         205

	t_float= (X1 + ((X2 - X1) * (inp - Y1) / (Y2 - Y1)));
	return t_float;
}

/// <summary>
/// interpolates and returns float
/// </summary>
/// <param name="CalStructPtr"> particular calibration structure pointer</param>
/// <returns>float </returns>
float Read_TWI_ADC_and_interpolate(Calibr2points* CalStructPtr)
{
	Uint32 tLong = (Uint32)(twi.buffer[BYTE_1]) << 8;
	tLong += twi.buffer[BYTE_2];
	return LinInterpolation((float)tLong, CalStructPtr);
}

//IK20240327 removed unused parameters (uint8 measurement, uint8 ADC_status) because function uses global vars measurement_ID, ADC_Status
void Measure(void)
{
uint16 result;
	// ADC_MUX X0 <--- CH 5
	// ADC_MUX X1 <--- CH 3  Plus GND Fault,  , Analog Brd, U1B.7 via 4.7k
	// ADC_MUX X2 <--- CH 0  Battery voltage, , Analog Brd, U3.5
	// ADC_MUX X3 <--- CH 7  Hi-Z detected Open circuit, Analog Brd, U4D.14
	// ADC_MUX X4 <--- CH 4
	// ADC_MUX X5 <--- CH 1 GND FAULT VOLTAGE , Analog Brd, U2.5
	// ADC_MUX X6 <--- CH 6 Ripple current    , Analog Brd, U8D.16
	// ADC_MUX X7 <--- CH 2 MINUS_GND_VOLTS  , Analog Brd, U1A.1 via 4.7k
	// PortD.3 == A_MUX
	// PortD.4 == B_MUX
	// PortA.6 == C_MUX
#define A_MUX 0x08 //Bit_3, port D
#define B_MUX 0x10 //Bit_4, port D
#define C_MUX 0x40 //Bit_6, port A

	// IK20250605 for diagnostic
	muxA = MUXA; // contains mirrored setting of MUXA port
	muxB = MUXB; // contains mirrored setting of MUXB port
	muxC = MUXC; // contains mirrored setting of MUXC port

	switch (ADC_Status) {
	default:
	case ADC_SETUP: //==8
	{
		if (measurement_ID == ADC_BATT_VOLTS)  // 1
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// addr lines off
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x10; // setBit(MUXAB_PORT, B_MUX)			// address 2 batt volts
		}
		if (measurement_ID == ADC_FAULT_VOLTS) // 2
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// zero it
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x08; // setBit(MUXAB_PORT, A_MUX);			// address 5
			MUXC_PORT |= 0x40;  // setBit(MUXC_PORT, C_MUX);			// address 5
		}
		if (measurement_ID == ADC_MINUS_GND_VOLTS) // 3
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// zero it
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x18; // setBit(MUXAB_PORT, A_MUX | B_MUX);	// address 7
			MUXC_PORT |= 0x40;  // setBit(MUXC_PORT, C_MUX);			// address 7
		}
		if (measurement_ID == ADC_RIPPLE_CURRENT)  // 5
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// zero it
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x10;  // setBit(MUXAB_PORT, B_MUX);			// address 6
			MUXC_PORT |= 0x40;   // setBit(MUXC_PORT, C_MUX);			// address 6
		}
		if (measurement_ID == ADC_RIPPLE_VOLTAGE)  // 6
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// zero it
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x18; // setBit(MUXAB_PORT, A_MUX | B_MUX);	// address 3
		}
		if (measurement_ID == ADC_EXTRA_ANALOG_CH)      // 7
		{
			MUXAB_PORT &= 0xE7; // clearBit(MUXAB_PORT, A_MUX | B_MUX);	// zero it
			MUXC_PORT &= 0xBF;  // clearBit(MUXC_PORT, C_MUX);			// addr lines off
			MUXAB_PORT |= 0x30;   // setBit(MUXAB_PORT, Bit_5 | B_MUX);	// address 2 plus Current source out ???
		}
#ifndef PC
		timer.ADC_ms = 25;// IK20251118 was 13 ms, and reading was low, varied MUX delay: stable battery voltage starts at >=18 ms. changed: give things 25 ms to settle.
#else //PC simulation
		timer.ADC_ms = 0;
#endif // PC
		ADC_Status = ADC_START_CONVERSION;	// =9
	}
	break;
	case ADC_START_CONVERSION:	// ==9
		if (timer.ADC_ms > 0) return;
#ifdef TIME_TESTING
		//	SetTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
#endif // #ifdef TIME_TESTING
		memset(twi.buffer, 0xEE, 4);		// clear buffer
		// send TWI command Start conversion
		TWI_Write(A_TO_D_WRITE, 0x88, NULL_BYTE, NULL_BYTE);	// Set ADC 15sps for 16 bits
		ADC_Status = ADC_WAIT_CONVERSION;
#ifndef PC
		timer.ADC_ms = 125; // IK20251118 changed from 120 to 125, some more time margin; tried settng conversion delay 100 ms and shown battery voltage does not follow supply voltage
#else //PC simulation
		timer.ADC_ms = 0;
#endif // PC
		break;
	case ADC_WAIT_CONVERSION:									// ==10
		if (timer.ADC_ms > 0) return;							// timer.ADC_ms decrement in Timer 2 interrupt
		ADC_Status = ADC_READ_RESULT;
		break;
	case ADC_READ_RESULT:	// ==13
		if ((measurement_ID != DISPLAY_DATA)	// 20
			&& (measurement_ID != RELAY_DATA)	// 21
			&& (measurement_ID != IO_DATA)		// 22
			)
		{
#ifdef TIME_TESTING
		//	ClearTestPin44;					// IK20250523 reset test pin #44 (easy to solder to)
#endif // #ifdef TIME_TESTING
			// program is here when timer.ADC_ms reaches zero
			// get result from ADC via TWI
			TWI_Read(A_TO_D_READ);								// TWI_Read updates twi.buffer
			result = (twi.buffer[BYTE_1] * 256) + twi.buffer[BYTE_2];
			if(measurement_ID < 8 )
				rt.ADC_buff[measurement_ID] = result;
			CalibrationSteps(result);
			// set next channel
			//if (ADC_channel < 8) ADC_channel++;
			//else ADC_channel = 0;
			// repeat state machine
			ADC_Status = ADC_SETUP;
			break;
		}
	}
}

/// <summary>
/// Function saves calibration structure from SysData to EEPROM, calculates offset of start address vs &SysData
/// </summary>
/// <param name="CalStructureAdr">calibration structure start address, must be in SysData</param>
/// <param name="BitSize">now many bytes to save</param>
void EEPROM_SaveCal(Calibr2points* CalStructureAdr)
{
	uint16 BitSize = sizeof(Calibr2points);							// size of calibration structure
	uint16 i;
	int16 EE_address = (int16)CalStructureAdr - (int16)&SysData;	// calculate offset of start address vs &SysData
	if (EE_address < 0)												// if start address is not in SysData
		return; // do nothing
	if (EE_address + BitSize > sizeof(SysData))						// if start address + size is out of SysData
		return; // do nothing
	EE_address += EE_SYS_DATA_OFFSET;
	WATCHDOG_RESET();
	__disable_interrupt(); // disable interrupts
	for (i = 0; i < BitSize; i++)
	{
		EEPROM_Write_byte(EE_address + i, *((uint8*)CalStructureAdr + i)); // write byte by byte
	}
	__enable_interrupt(); // enable interrupts
}

/// <summary>
/// if we have received externally measured Low value, save it to calibration structure and set timer.Calibration to zero.
/// timer decrements in Timer2 ISR TIMER2_COMPA_interrupt(): if (timer.Calibration != 0) timer.Calibration--;
/// if we have received externally measured High value, save it to calibration structure and set timer.Calibration to 15000 ms.
/// if we get both bits SAVED_LOW_MEASUREMENT and SAVED_HIGH_MEASUREMENT, save calibration structure to EEPROM.
/// NOTE: 15 seconds for NEXT calibration seems to be too long, should be enough to have 1.5 seconds to measure and send externally measured value via DNP
/// </summary>
/// <param name="CalStructAdr">address of structure in SysData</param>
/// <param name="RawADC_float">value to save</param>
/// <returns>false if calibration in process, true when calibration is finished</returns>
uint8 PerformCalibration(Calibr2points* CalStructAdr, float RawADC_float)
{
	if (((cal_status & RECEIVED_EXT_LO_VALUE) != 0) && (timer.Calibration != 0)) // we have gotten extenally measured Low value already via DNP, (cal_status & 0x01) == 0x01)
	{
		CalStructAdr->X1_lowADCcounts = RawADC_float;			// save externally measured counts for low point
		clearBit(cal_status, RECEIVED_EXT_LO_VALUE);	// clear cal low measurement rcvd = cal_status &= 0xFE;
		setBit(cal_status, SAVED_LOW_MEASUREMENT);		// set low cal measurement saved  = cal_status |= 0x04;
		timer.Calibration = 0;
	}
	if (((cal_status & RECEIVED_EXT_HI_VALUE) != 0) && (timer.Calibration != 0)) // Got high value
	{
		clearBit(cal_status, RECEIVED_EXT_HI_VALUE);	// clear cal high measurement rcvd = cal_status &= 0xFD;
		setBit(cal_status, SAVED_HIGH_MEASUREMENT);		// set high cal measurement saved  = cal_status |= 0x08;
		CalStructAdr->X2_highADCcounts = RawADC_float;			//measured counts for high point
		timer.Calibration = DEF_CALIBRATION_DELAY;		//-!-IK20250807 check, 15 seconds is too long? Should be enough to have 1.5 seconds
	}
	if ((cal_status & (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT)) == (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT))
	{
		clearBit(cal_status, (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT));	// cal_status &= 0xF3;  //clear low and high measurement saved

		EEPROM_SaveCal(CalStructAdr);					// save to EEPROM
		calibr_step = CALIBRATION_DONE;
		return TRUE;									// calibration done
	}
		return FALSE;									// calibration in process
}

void CalibrationSteps(uint16 RawADCcounts)
{
	float tmp_result, a, b;
	float RawADC_f = (float)RawADCcounts;
	float ReciprocalDeltaV_f = 1.0f / (float)(SysData.NV_UI.V20 - SysData.NV_UI.V4);
	Calibr2points* CalStructPtr;
	ADC_counts = RawADCcounts;				//-!- IK20250226 ADC_counts added for test
	if (measurement_ID == ADC_BATT_VOLTS)
	{
		CalStructPtr = &SysData.BatteryVolts;
		if (calibr_step == ADC_BATT_VOLTS)	// ADC_BAT_VOLTS is in indicated by cal_status |= 0x08;        //set low cal measurement rcvd
		{
			if (PerformCalibration(CalStructPtr, RawADC_f) == TRUE) {
				SysData.Bat_Cal_Offset_Volts_f = 0;                   //calibrated so zero offset
				SaveToEE(SysData.Bat_Cal_Offset_Volts_f); //EEPROM_Write_float(adr_v_cal_f, SysData.Bat_Cal_Offset_Volts_f); // EEPROM[140] IK20231219
			}
		}  //End calibr_step batt volts

		//X1 = CalStructPtr->Y1_lowCalibrVal;  // 9.0E04
		//X2 = CalStructPtr->Y2_highCalibrVal; // 1.3E05
		//Y1 = CalStructPtr->X1_lowADCcounts;  // 1.3367E04
		//Y2 = CalStructPtr->X2_highADCcounts; // 2.1736E04
		//rt.OutData.measured.battery_voltage_f = X1 + ((X2 - X1) * (inp - Y1) / (Y2 - Y1));  //128891 mV

		rt.OutData.measured.battery_voltage_f = Read_TWI_ADC_and_interpolate(CalStructPtr) + SysData.Bat_Cal_Offset_Volts_f; //IK20251029 offset rarely changes, only by user from a front panel CALibration menu.
		//Typically, operator with sertified calibrated meter measures battery voltage and adjusts offset to match displayed voltage to his meter.
#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
		if (SysData.input_type[1] == UNIPOLAR)                   //if unipolar
			rt.OutData.measured.battery_voltage_f *= 2;                 //multiply by 2
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24

		//rt.battery_deciVolts = (int16)(rt.OutData.measured.battery_voltage_f * 10);

		a = (SysData.CurrentOut_I420.Y2_highCalibrVal - SysData.CurrentOut_I420.Y1_lowCalibrVal) * ReciprocalDeltaV_f;
		b = ((SysData.CurrentOut_I420.Y1_lowCalibrVal * (float)SysData.NV_UI.V20) - (SysData.CurrentOut_I420.Y2_highCalibrVal * (float)SysData.NV_UI.V4)) * ReciprocalDeltaV_f;
		if(SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit)
		{
			a = (2.95f * SysData.CurrentOut_I420.Y2_highCalibrVal - 1.135f * SysData.CurrentOut_I420.Y1_lowCalibrVal) * ReciprocalDeltaV_f;
			b = ((1.135f * SysData.CurrentOut_I420.Y1_lowCalibrVal * (float)SysData.NV_UI.V20) -
				(2.95f * SysData.CurrentOut_I420.Y2_highCalibrVal * (float)SysData.NV_UI.V4)) * ReciprocalDeltaV_f;
		}
		tmp_result = (a * (rt.OutData.measured.battery_voltage_f  * 0.001f) + b) * 65535;
		if ((rt.i_cal_active == true) && (rt.cal_4mA == true))						// calibrating 0 or 4 ma
		{
			tmp_result = SysData.CurrentOut_I420.Y1_lowCalibrVal * 65535;
			if (SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit)		// I01 == 1, I420 == 0
				tmp_result = (SysData.CurrentOut_I420.Y1_lowCalibrVal * 1.135f) * 65535;
		}
		if ((rt.i_cal_active == true) && (rt.cal_20mA == true))						// calibrating 1 or 4ma
		{
			tmp_result = SysData.CurrentOut_I420.Y2_highCalibrVal * 65535;
			if (SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit)		// I01 == 1, I420 == 0
				tmp_result = (SysData.CurrentOut_I420.Y2_highCalibrVal * 2.95f) * 65535;
		}
		if (tmp_result > 65535)														// keep it 16 bits
			tmp_result = 65535;
		if (tmp_result < 0)
			OCR1A = 65535;															// PWM of 1%
		else
			OCR1A = (uint16)tmp_result;
	}
	if (measurement_ID == ADC_FAULT_VOLTS)
	{
		CalStructPtr = &SysData.FaultVolts;
		if (calibr_step == ADC_FAULT_VOLTS)
		{
			PerformCalibration(CalStructPtr, RawADC_f);
		}
		rt.OutData.measured.G_fault_voltage_f = Read_TWI_ADC_and_interpolate(CalStructPtr);

#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
		if (SysData.input_type[2] == UNIPOLAR)						// if unipolar
		{
			if (rt.OutData.measured.G_fault_voltage_f > 0)
				rt.OutData.measured.G_fault_voltage_f *= 2;								// multiply by 2
			else
				rt.OutData.measured.G_fault_voltage_f *= 2;								// else report 0
		}
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
	}

	if (measurement_ID == ADC_MINUS_GND_VOLTS)
	{
		CalStructPtr = &SysData.MinusGndVolts;
		if (calibr_step == ADC_MINUS_GND_VOLTS)
		{
			PerformCalibration(CalStructPtr, RawADC_f);
		}
		rt.OutData.measured.minus_gnd_volts_f = -Read_TWI_ADC_and_interpolate(CalStructPtr);

#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
		if (SysData.input_type[3] == UNIPOLAR)							// if unipolar
			if (rt.OutData.measured.minus_gnd_volts_f < 0)
				rt.OutData.measured.minus_gnd_volts_f *= 2;									// multiply by 2
			else
				rt.OutData.measured.minus_gnd_volts_f = 0;									// else report 0
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
//		rt.minus_gnd_deciVolts = (int)(-rt.OutData.measured.minus_gnd_volts_f  * 0.01f);		// convert to integer for twi transmission
	}  //End calibr_step minus gnd volts

	if (measurement_ID == ADC_RIPPLE_CURRENT)
	{
		if ((Display_Info.Status & DISP_SELECTED_3PH_BIT) == 0)	// if single phase selected
			CalStructPtr = &SysData.RippleCurr1ph;						// 1-phase address
		else //if ((display_status & DISP_SELECTED_3PH_BIT))				// if three phase selected
			CalStructPtr = &SysData.RippleCurr3ph;						// 3-phase address

		if (calibr_step == ADC_RIPPLE_CURRENT)
		{
			if (PerformCalibration(CalStructPtr, RawADC_f) == TRUE)
			{
				//-!- IK20250807 I think multiplication is not needed, test it
				CalStructPtr->X1_lowADCcounts *= 0.000449f;					// convert to mA
				CalStructPtr->X2_highADCcounts *= 0.000449f;					// convert to mA
			}
		}  //End calibrating ripple current

		rt.OutData.measured.ripple_mA_f = RawADC_f;
		// 1st order Low Pass Filtering
		filtered_ripple_i_f = last_ripple_i_f * (1 - LPF_factor) + LPF_factor * rt.OutData.measured.ripple_mA_f;
		last_ripple_i_f = rt.OutData.measured.ripple_mA_f;
		debug_f1 = filtered_ripple_i_f * 0.000449f;

		debug_f2 = LinInterpolation(debug_f1, CalStructPtr);

		// Test for -Infinity, +Infinity and Not a Number.
		if (isnan(debug_f2))
			rt.OutData.measured.ripple_mA_f = 0;
		else if ((debug_f2 > 9999) || (debug_f2 < 0))
			rt.OutData.measured.ripple_mA_f = 1;
		else
			rt.OutData.measured.ripple_mA_f = debug_f2;
	}

	if (measurement_ID == ADC_RIPPLE_VOLTAGE)  // ADC_counts: RipV measured with MM = 484 mV @120Hz,
	{
		// ADC_counts come from ADC: RipV measured with handheld MM == MultiMeter
		// Frequency ||    120 Hz     ||    360 Hz     ||
		// ACPS, VAC ||RipV MM | ADC  ||RipV MM | ADC  ||
		//    0      ||  2.2   | 1    ||  2.2   | 1    || (test relay off)
		//    0.1    ||  2.6   | 1    ||  3.9   | 1    ||
		//    1      ||  23.7  | 14   ||  23.7  | 7    ||
		//    2      ||  47.6  | 35   ||  47.5  | 18   ||
		//    3      ||  71.6  | 56   ||  71.3  | 29   ||
		//    4      ||  95.8  | 77   ||  95.4  | 41   ||
		//    5      ||  120   | 99   ||  119.5 | 53   ||
		//   10      ||  241   | 204  ||  240   | 113  ||
		//   20      ||  484   | 418  ||  481   | 234  ||
		//   28      ||  682   | 592  ||  675   | 333  ||
		//   40      ||  974   | 849  ||  970   | 479  ||
		//   50      ||  1218  | 1064 || 1216   | 602  ||
		//   60      ||  1463  | 1279 || 1461   | 726  ||
		//   70      ||  1708  | 1495 || 1707   | 849  ||
		//   80      ||  1955  | 1711 || 1951   | 973  ||
		//   100     ||  2447  | 2148 || 2445   | 1221 ||

		RVV_ADC_counts = RawADCcounts;
		if ((Display_Info.Status & DISP_SELECTED_3PH_BIT) == 0)	// if single phase is selected
			CalStructPtr = &SysData.RippleVolts1ph;
		else
			CalStructPtr = &SysData.RippleVolts3ph;

		if (calibr_step == ADC_RIPPLE_VOLTAGE)							// in calibrating mode
		{ /*
			if (((cal_status & RECEIVED_EXT_LO_VALUE) != 0) && (timer.Calibration != 0)) //Got Low value
			{
				CalStructPtr->X1_lowADCcounts = RawADC_f * 0.000112f;	// write externally measured counts for low point result by pointer
				clearBit(cal_status, RECEIVED_EXT_LO_VALUE);	// clear cal low measurement rcvd = cal_status &= 0xFE;
				setBit(cal_status, SAVED_LOW_MEASUREMENT);		// set low cal measurement saved  = cal_status |= 0x04;
				timer.Calibration = 0;
			}
			if (((cal_status & RECEIVED_EXT_HI_VALUE) != 0) && (timer.Calibration != 0)) // Got high value
			{
				clearBit(cal_status, RECEIVED_EXT_HI_VALUE);	// clear cal high measurement rcvd = cal_status &= 0xFD;
				setBit(cal_status, SAVED_HIGH_MEASUREMENT);		// set high cal measurement saved  = cal_status |= 0x08;
				CalStructPtr->X2_highADCcounts = RawADC_f * 0.000112f;	// measured counts for high point
				timer.Calibration = 15000;  // 15 seconds
			}
			if ((cal_status & (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT)) == (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT))
			{
				clearBit(cal_status, (SAVED_LOW_MEASUREMENT | SAVED_HIGH_MEASUREMENT));	// cal_status &= 0xF3;        //clear low and high measurement saved

				EEPROM_SaveCal(CalStructPtr); // save to EEPROM

				calibr_step = CALIBRATION_DONE;
			}*/
			//if (PerformCalibration(CalStructPtr, RawADC_f) == TRUE)
			//{
			//	//-!- IK20250807 i think multiplication is not needed, test it
			//	CalStructPtr->X1_lowADCcounts *= 0.000112f;					// convert to mA
			//	CalStructPtr->X2_highADCcounts *= 0.000112f;					// convert to mA
			//}
		}  //End calibrating ripple V

		rt.OutData.measured.ripple_mV_f = RawADC_f;  // counts: for test = 2000
		filtered_ripple_v_f = last_ripple_v_f * (1 - LPF_factor) + LPF_factor * rt.OutData.measured.ripple_mV_f;
		last_ripple_v_f = rt.OutData.measured.ripple_mV_f; // test = 2000

		debug_f1 = filtered_ripple_v_f;// *0.000112f;						// in Volts, test: =2.24000007E-1 = 0.224000007

		debug_f2 = LinInterpolation(debug_f1, CalStructPtr);			// in Volts, test: = 2.30606532

		//Test for NaN
		if (isnan(debug_f2))
			rt.OutData.measured.ripple_mV_f = 0;
		else if ((debug_f2 > 9999) || (debug_f2 < 0))
			rt.OutData.measured.ripple_mV_f = 1;
		else
			rt.OutData.measured.ripple_mV_f = debug_f2;
	}
} // end of CalibrationSteps(uint16 RawADCcounts)

/*********************************************************************/
/*              C O M M   L E D   O P                                */
/*********************************************************************/
/*  Description:  This routine flashes the comm led appropriately.

	Inputs:       type, value.
	Outputs:      None
	Notes:        None
	Revisions:    03/06/16     REC     Created
	Port pins: PD7 -> XMT/REV_LED_1 ->FlexCable.J1.2,19 ->FrontBrd U2.13->U2_inverts->U2.12->R5=820->BicolorLED L4.pin A=Red Anode,   logic 1 shines RED,   logic 0 shines GREEN
	Port pins: PA3 -> XMT/REV_LED_2 ->FlexCable.J1.3,18 ->FrontBrd U2.11->U2_inverts->U2.10->        BicolorLED L4.pin K=Red Cathode, logic 1 shines GREEN, logic 0 shines RED
	To shine the LED on front panel, the PB1 and PA3 must be in different logic levels
	PD7 = 1, PA3 = 0 -> shines GREEN
	PD7 = 0, PA3 = 1 -> shines RED
*/
/*********************************************************************/
void Comm_LED_Op(uint8 type)
{
	if (type == GREEN)									// rcving comms? set PD7 = 1, PA3 = 0
	{
		clearBit(PORTA, TxRxLED_Red_PA3);				// PORTA &= 0xF7; // xmt_rcv_2 GND to make green
		if (timer.TxRxLED_blink == 0)					// blink tmr = 0?
		{
			timer.TxRxLED_blink = UART_BLINK_MilliSec;	// reset for off period?
			if (testBit(PORTD, TxRxLED_Green_PD7))		// if ((PORTD & 0x80) == 0x80)         //if on
			{
				clearBit(PORTD,TxRxLED_Green_PD7);		// PORTD &= 0x7F;                      //turn off
#ifdef PC
				SetTxRxLEDoff;
#endif
			}
			else {
				setBit(PORTD, TxRxLED_Green_PD7);		// PORTD |= 0x80;                     // else Green LED turn on
#ifdef PC
				SetTxRxLEDgreen;
#endif
			}
		}
	}
	if (type == RED)									// xmting comms? set PD7 = 0, PA3 = 1
	{
		clearBit(PORTD,TxRxLED_Green_PD7);				// PORTD &= 0x7F;
		if (timer.TxRxLED_blink == 0)
		{
			timer.TxRxLED_blink = UART_BLINK_MilliSec;
			if (testBit(PORTA, TxRxLED_Red_PA3))		// if ((PORTA & 0x08) == 0x08)
			{
				clearBit(PORTA,TxRxLED_Red_PA3);		// PORTA &= 0xF7;
#ifdef PC
				SetTxRxLEDoff;
#endif
			}
			else {
				setBit(PORTA, TxRxLED_Red_PA3);			// PORTA |= 0x08;                     // Red LED on
#ifdef PC
				SetTxRxLEDred;
#endif
			}
		}
	}
	if (comm_state == NO_ACTIVITY)					// not rcv or xmt
	{
#ifdef PHASE_DEBUG // Tx Rx LED used to show phase state: green - 1 phase calibration, red == three phase, no light - not in calibration mode
		if (rt.ripple_calibration_phase == CalibrateSinglePhase) {
			//RxGreen_TxRed_LED_op = GREEN;
			PORTA &= 0xF7;								// xmt_rcv_2 GND to make green
			PORTD |= 0x80;								// turn GREEN on
  #ifdef PC
				SetTxRxLEDgreen;
  #endif
		}
		else if (rt.ripple_calibration_phase == Calibrate_ThreePhase)
		{
			//RxGreen_TxRed_LED_op = RED;
			PORTD &= 0x7F;
			PORTA |= 0x08;								// red led on
  #ifdef PC
				SetTxRxLEDred;
  #endif
		}
		else  //No calibration - LEDs are off
		{
// RxGreen_TxRed_LED_op = NONE;
			clearBit(PORTD,TxRxLED_Green_PD7);			// PORTD &= 0x7F;  //turn off comm led 1
			clearBit(PORTA,TxRxLED_Red_PA3);			// PORTA &= 0xF7;  //turn off comm led 2
  #ifdef PC
				SetTxRxLEDoff;
  #endif
		}
#else // no PHASE_DEBUG
			clearBit(PORTD,TxRxLED_Green_PD7);			// PORTD &= 0x7F;  //turn off comm led 1
			clearBit(PORTA,TxRxLED_Red_PA3);			// PORTA &= 0xF7;  //turn off comm led 2
  #ifdef PC
				SetTxRxLEDoff;
  #endif
#endif
	}
}

/*********************************************************************/
/*                P A R S E  D I S P L A Y  D A T A                  */
/*********************************************************************/
/* Description: This routine handles the user input calibration data
				and faults detected from the display board to calibrate
				the PWM on this board and determine events.

	Inputs:     'alarm_status', bits button presses UP/Down in 'display_mode',
	Outputs:    PWM calibration constant SysData.CurrentOut_I420.Y2_highCalibrVal and SysData.CurrentOut_I420.Y1_lowCalibrVal
				and events.
	Notes:      None
	Revisions:  01/02/17     REC     Created						 */

/*********************************************************************/

/* IK20250811 use for general case when INDEX can be any number, not on-to-one linked to Bit Number
// Mapping table: alarm bit position -> event index
static const uint8 AlarmEventIndex[NUM_ALARMS] =
{
	HIGH_BAT_EVENT_INDEX,  // bit 0
	LOW_BAT_EVENT_INDEX,   // bit 1
	PLUS_GF_EVENT_INDEX,   // bit 2
	MINUS_GF_EVENT_INDEX,  // bit 3
	RIPPLE_VOLT_EVENT_INDEX, // bit 4
	RIPPLE_CURR_EVENT_INDEX, // bit 5
	AC_LOSS_EVENT_INDEX,     // bit 6
	HI_Z_EVENT_INDEX,        // bit 7
	EBAT_EVENT_INDEX         // bit 8
}; */

void Parse_Display_Data(void){
	uint16 changed_bits;
	uint8 bit;
	if (old_alarm_status != Display_Info.alarm_status)		//new alarm came in?
	{
		changed_bits = old_alarm_status ^ Display_Info.alarm_status;			// Exclusive OR: mask==sets different bits; which alarm is new

		for (bit = 0; bit < NUM_ALARMS; bit++)
		{
			uint16 mask = (1 << bit);

			if (changed_bits & mask) // this alarm changed state
			{
				// Store_Event(AlarmEventIndex[bit], (Display_Info.alarm_status & mask) >> bit); //IK20250811 general case
				Store_Event(bit, (Display_Info.alarm_status & mask) >> bit);	// one-to-one event-index to bit-number mapping
			}
		}
		old_alarm_status = Display_Info.alarm_status;		// not again till change
	}


	if ((Display_Info.Status & DISP_BRD_CAL_MODE) != 0)		// in one of the cal modes
	{
		if ((Display_Info.Status & DISP_BRD_I_CAL_MODE) != 0)
			rt.i_cal_active = true;
		timer.PWM_calibration = 5000;							// to store values when timer.PWM_calibration becomes less than 2000
	}
	if (Display_Info.Status & DISP_STATE_LOmA_BIT)				// doing 4ma or 0ma?
	{
		rt.cal_4mA = true;
		rt.cal_20mA = false;
		if (I420_calibr_lock == I420calibUNLOCKED)				// if calibration not attempted
		{
			if (Display_Info.Status & DISP_STATE_0_1mA_BIT)
				setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);	// doing 0 mA; I01 == 1, I420 == 0
			else
				clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);	// doing 4 mA; I01 == 1, I420 == 0
		}
	}
	if (Display_Info.Status & DISP_STATE_HImA_BIT)				// doing 20 mA or 1 mA?
	{
		rt.cal_4mA = false;
		rt.cal_20mA = true;
		if (I420_calibr_lock == I420calibUNLOCKED)				// if calibration not attempted
		{
			if (Display_Info.Status & DISP_STATE_0_1mA_BIT)
				setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);	// doing 1 mA; I01 == 1, I420 == 0
			else
				clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);	// doing 20 mA; I01 == 1, I420 == 0
		}
	}

	// IK20251031 below is calibration adjustment by UP/DOWN buttons in CAL mode
	/* IK20251110 program never stops here because Display_Info.Status buton bits are cleared on exit from Operation()

	if (Display_Info.Status & DISP_STATE_ButtonUP_BIT)			// pressed the up button
	{
		if (Display_Info.Status & DISP_STATE_HImA_BIT)			// 20 mA setting
			SysData.CurrentOut_I420.Y2_highCalibrVal -= 0.0001f;
		if (Display_Info.Status & DISP_STATE_LOmA_BIT)			// 4 mA setting
			SysData.CurrentOut_I420.Y1_lowCalibrVal -= 0.0001f;
		if (Display_Info.Status & DISP_STATE_VoltCal_BIT)		// volt offset calibration setting
			SysData.Bat_Cal_Offset_Volts_f += 0.1f;			// IK20251029 was +=1
		I420_calibr_lock = I420calibLOCKED;
		clearBit(Display_Info.Status, DISP_STATE_ButtonUP_BIT);	// clear up bit 0x08
	}
	if (Display_Info.Status & DISP_STATE_ButtonDOWN_BIT)		// pressed the down arrow
	{
		if (Display_Info.Status & DISP_STATE_HImA_BIT)			// 20 mA setting
			SysData.CurrentOut_I420.Y2_highCalibrVal += 0.0001f;
		if (Display_Info.Status & DISP_STATE_LOmA_BIT)			// 4 mA setting
			SysData.CurrentOut_I420.Y1_lowCalibrVal += 0.0001f;
		if (Display_Info.Status & DISP_STATE_VoltCal_BIT)		// volt cal setting
			SysData.Bat_Cal_Offset_Volts_f -= 0.1f;			// IK20251029 was -=1
		I420_calibr_lock = I420calibLOCKED;
		clearBit(Display_Info.Status, DISP_STATE_ButtonDOWN_BIT);	// clear down bit
	}
	*/
		// after 2 sec of not in calibration mode
	if ((timer.PWM_calibration < 2000) && (timer.PWM_calibration > 0))	//	timer.PWM_calibration is decreasing in interrupt
	{
		timer.PWM_calibration = 0;								// only store once
		//***  store duty cycle for low mA voltage point (DC4)
		EEPROM_SaveCal(&SysData.CurrentOut_I420); // save to EEPROM
		//EEPROM_Write_float(adr_cal14_dc4_cal, SysData.CurrentOut_I420.Y1_lowCalibrVal);	// EEPROM[146] store duty cycle for low mA voltage point (DC4)
		//EEPROM_Write_float(adr_cal15_dc20_cal, SysData.CurrentOut_I420.Y2_highCalibrVal);// EEPROM[150] store duty cycle for Hi voltage point (DC20)
		SaveToEE(SysData.Bat_Cal_Offset_Volts_f);			// EEPROM_Write_float(adr_v_cal_f, SysData.Bat_Cal_Offset_Volts_f);	// EEPROM[140] store v_cal (bat_offset)
		SaveToEE(SysData.NV_UI.SavedStatusWord);

		rt.i_cal_active = false;
		I420_calibr_lock = I420calibUNLOCKED;
	}//end store cal values
} // endof void Parse_Display_Data(void)

/// <summary>
/// Function saves variables from SysData to EEPROM, struct EEPROM_SysData, calculates offset of start address vs &SysData
/// </summary>
/// <param name="CalStructureAdr"> start address, must be in SysData</param>
/// <param name="BitSize">now many bytes to save</param>
void EEPROM_Save(void* StartAdr, uint16 BitSize)
{
	uint16 i;
	int16 EE_address = (int16)StartAdr - (int16)&SysData;		// calculate offset of start address vs &SysData
	if (EE_address < 0)											// if start address is not in SysData
		return;													// do nothing
	if (EE_address + BitSize > sizeof(SysData))					// if start address + size is out of SysData
		return;													// do nothing
	EE_address += EE_SYS_DATA_OFFSET;
	WATCHDOG_RESET();
	__disable_interrupt();										// disable interrupts
	for (i = 0; i < BitSize; i++)
	{
		EEPROM_Write_byte(EE_address + i, *((uint8*)StartAdr + i)); // write byte by byte
	}
	__enable_interrupt(); // enable interrupts
}

/// <summary>
/// Function loads variables from EEPROM, from struct EEPROM_SysData to RAM-located SysData, calculates offset of start address vs &SysData
/// The read addreess is automatically calculated from &EEPROM_SysData.
/// </summary>
/// <param name="StartAdr"> start address, must be in RAM-located SysData</param>
/// <param name="BitSize">now many bytes to save</param>
void EEPROM_Load_to_SysData(void* StartAdr, uint16 BitSize)
{
	uint16 i;
	int16 EE_address = (int16)StartAdr - (int16)&SysData;		// calculate offset of start address vs &SysData
	uint8* SysDataPtr = (uint8*)StartAdr;						// pointer to SysData location to write to
	if (EE_address < 0)											// if start address is not in SysData
		return;													// do nothing
	if (EE_address + BitSize > sizeof(SysData))					// if start address + size is out of SysData
		return;													// do nothing
	EE_address += EE_SYS_DATA_OFFSET;							// re-map address to EEPROM_SysData address
	WATCHDOG_RESET();
	__disable_interrupt();										// disable interrupts
	for (i = 0; i < BitSize; i++)
	{
		*SysDataPtr = EEPROM_Read_byte(EE_address + i);			// read byte from EEPROM(EE_address + i); write to SysData
		SysDataPtr++;											// increment pointer to SysData location
	}
	__enable_interrupt();										// enable interrupts
}


/*********************************************************************/

void EEPROM_Read_Mem_Block(uint16 EEaddress, uint8* DestAdr, uint16 block_size)
{
	uint16 ctr = 0;
	do {
		uint8 aByte = EEPROM_Read_byte(EEaddress);		// read byte from EEPROM
		*DestAdr = aByte;								// write to destination in RAM
		EEaddress++;
		DestAdr++;
	} while ((ctr++) < block_size);
}

/// <summary>
/// Writes a block of memory from RAM to EEPROM, byte by byte.
/// to reduce wearing of EEPROM, it only writes differences.
/// checks if the EEPROM byte is the same or different from whats needs to be written.
/// If the new byte is different, it writes the new byte to EEPROM.
/// </summary>
/// <param name="SourceAdr">Pointer to the source memory block in RAM.</param>
/// <param name="EEaddress">Starting address in EEPROM where the data will be written.</param>
/// <param name="block_size">Number of bytes to write from RAM to EEPROM.</param>
void EEPROM_Write_Mem_Block(uint8* SourceAdr, uint16 EEaddress, uint16 block_size)
{
	uint16 ctr = 0;
	do {
		uint8 rByte = *SourceAdr;						// read byte from RAM
		uint8 eByte = EEPROM_Read_byte(EEaddress);		// read byte from EEPROM to compare
		if (rByte != eByte)								// if the bytes are NOT the same, write
			EEPROM_Write_byte(EEaddress, rByte);		// write to destination in EEPROM

		SourceAdr++;									// increase RAM pointer to next byte
		EEaddress++;									// increase EEPROM index = address
	} while ((ctr++) < block_size);
}

/*********************************************************************/
float EEPROM_Read_float(uint16 EEaddress) // IK20231215
{
	float tFloat;
	Uint32 tmplong;										// usage: to convert a byte to 32-bit long for shift
	Uint32 QuasiFloat;									// where float is assembled as bytes
	uint8 aByte = EEPROM_Read_byte(EEaddress);	// read lower byte
	QuasiFloat = aByte;									// save into QuasiFloat
	EEaddress++;										// increment EE address
	aByte = EEPROM_Read_byte(EEaddress);		// read 2nd byte
	tmplong = aByte;									// convert to 32-bit long for shift
	tmplong = tmplong << 8;								// shift up 32-bit long - value in a 2nd byte
	QuasiFloat |= tmplong;								// add 32nd byte QuasiFloat
	EEaddress++;										// increment EE address
	aByte = EEPROM_Read_byte(EEaddress);		// read 3rd byte
	tmplong = aByte;
	tmplong = tmplong << 16;							// shift up 32-bit long - value in a 3rd byte
	QuasiFloat |= tmplong;								// add 3rd byte QuasiFloat
	EEaddress++;										// increment EE address
	aByte = EEPROM_Read_byte(EEaddress);		// read 4th upper byte
	tmplong = aByte;
	tmplong = tmplong << 24;							// shift up 32-bit long - value in a 4th byte
	QuasiFloat |= tmplong;								// add 4th byte QuasiFloat
	tFloat = *((float*)(&QuasiFloat));					// convert 4 bytes into float: treat address of QuasiFloat as pointer to float,
	return tFloat;										// read from this address value as a float and function returns it as a float
}

/*********************************************************************/
uint16 EEPROM_Read_two_bytes(uint16 EEaddress) // IK20231214
{
	uint16 value = 0;
	uint16 upper_byte = 0;
#ifdef EEsimulator
	value= EEPROM[EEaddress];
	upper_byte=EEPROM[EEaddress+1];
	return value + (upper_byte << 8);
#else
	WATCHDOG_RESET();
	//read from first EE address lower byte
	while ((EECR & 0x02) != 0)
	{
#ifdef PC
		clearBit(EECR, 0x02);
#endif
	};//wait till ready
	EEAR = EEaddress;								// load starting address
	EECR |= 0x01;									// send command read data
	value = EEDR;									// read lower byte data from first EE address
	//read from next EE address upper byte
	while ((EECR & 0x02) != 0)
	{
#ifdef PC
		clearBit(EECR, 0x02);
#endif
	};//wait till ready
	EEAR = EEaddress + 1;							// load next address
	EECR |= 0x01;									// send command read data from next EE address
	upper_byte = EEDR;								// read upper byte data
	value |= (upper_byte << 8);						// combine upper and lower bytes
	return value;
#endif
}

/*********************************************************************/
uint8 EEPROM_Read_byte(uint16 EEaddress) // IK20231214
{
#ifdef EEsimulator
	return EEPROM[EEaddress];
#else
	uint8 value = 0;
	WATCHDOG_RESET();
	//read from EE address a byte
	while ((EECR & 0x02) != 0)
	{
#ifdef PC
		clearBit(EECR, 0x02);
#endif
	};//wait till ready
	EEAR = EEaddress;								// load starting address
	EECR |= 0x01;									// send command read data
	value = EEDR;									// read data
	return value;
#endif
}


/*********************************************************************/
void EEPROM_Write_byte(uint16 EEaddress, uint8 value) // IK20231214
{
#ifdef EEsimulator
	EEPROM[EEaddress] = value;
#else
#endif
	WATCHDOG_RESET();
	//store into EE address a byte
	while ((EECR & 0x02) != 0)
	{
#ifdef PC
		clearBit(EECR, 0x02);
#endif
	};//wait till ready
	EEAR = EEaddress;								// load starting address
	EEDR = value;									// load byte of value
	EECR |= 0x04;									// enable master write
	EECR |= 0x02;									// write data
}

/// <summary>
/// IK20250825 Checks if EEPROM has valid data, then retrieves system parameters from EEPROM into SysData struct in RAM
/// call this function ONLY AFTER the default parameters in SysData are set
/// because in case of empty EEPROM, function saves default parameters to EEPROM.
/// </summary>
/// <param name=""></param>
void Get_EEPROM_params(void)
{
	uint8 EE_reads = 0;
	uint16 IsDataValid = 0;
	uint16 DataFWversion = 0;
Read_EE:
	IsDataValid = EEPROM_Read_two_bytes(DATA_VALID_OFFSET + EE_SYS_DATA_OFFSET);
	DataFWversion = EEPROM_Read_two_bytes(FW_VERSION_OFFSET + EE_SYS_DATA_OFFSET);
	//EEPROM_Save(&SysData.NV_UI.BRate_index, 1);
	if ((IsDataValid != 0xFFFF) && (DataFWversion == FW_VERSION))	// valid data
	{
		//EEPROM_Read_Mem_Block(EE_source,*destination, size);
		EEPROM_Read_Mem_Block(EE_SYS_DATA_OFFSET, (uint8*)&SysData, sizeof(SysData));
		//SetStatusLEDyellow; // SysData is loaded, change LED to Yellow
		clearBit(rt.OperStatusWord, EE_DATA_0_NO_DATA_1_Bit);		// data valid - clear bit
		return;
	}
	else
	{
		setBit(rt.OperStatusWord, EE_DATA_0_NO_DATA_1_Bit);			// data not valid - set bit
		EEPROM_Write_Mem_Block( (uint8*)&SysData, EE_SYS_DATA_OFFSET, sizeof(SysData)); // save content of SysData into EEPROM - at the beginning, it was set with default data.
		EE_reads ++;
		if (EE_reads < 2) goto Read_EE;			// attempt to read again
		// else something wrong with EEPROM, bit EE_DATA_0_NO_DATA_1_Bit stays set
	}
	// update after loading flash data
#ifdef PC // simulate different than default settings from FLASH

#endif //PC
}

/*************************************************************/
void SetSysDataDefaultsInRAM(void)
{
	int ctr =0;										//index should be int type, or unnessesary char<->long conversion happen
	SYS_SPECIFIC_DATA * Sp = &DefaultSysdata;
												// adr offset size
	Sp->Data_Valid = TRUE;						// 0x000  2  // should not be FF
	Sp->FWversion = FW_VERSION;					// 0x002  2  // FW version, 0030
	Sp->NV_UI.high_bat_threshold_V_f = DEF_125V_high_bat_threshold;			// high alarm threshold 142.0
	Sp->NV_UI.low_bat_threshold_V_f = DEF_125V_low_bat_threshold;			// low alarm threshold 105.0
	Sp->NV_UI.minus_gf_threshold_V_f = DEF_125V_minus_gf_threshold;			// minus ground fault alarm threshold
	Sp->NV_UI.plus_gf_threshold_V_f = DEF_125V_plus_gf_threshold;			// plus ground fault threshold 13.0
	Sp->NV_UI.ripple_V_threshold_mV_f = DEF_ripple_voltage_threshold;	// ripple voltage threshold 200 mV
	Sp->NV_UI.ripple_I_threshold_mA_f = DEF_ripple_current_threshold;	// ripple current threshold 10 mA
	Sp->NV_UI.alarm_delay_sec_f = INIT_time_delay;							// grace period delay after alarm condition is detected, before setting alarm

	Sp->NV_UI.BRate_index = DEF_BAUD_RATE_INDEX;						// baud rate, index of default Baud_19200 ==9
	Sp->NV_UI.baud_rate = DEF_BAUD_RATE;								// baud rate, default Baud_19200
	Sp->NV_UI.meter_address = DEF_BAT_MONITOR_ADDR;						// battery monitor address
	Sp->NV_UI.host_address = DEF_HOST_ADDR;								// DNP host address. NOT user selectable from the menu, but from the serial interface on Comm boards
	Sp->NV_UI.modbus_first_reg = DEF_MODBUS_FIRST_REG;					// Modbus first register NOT user selectable from the menu, but from the serial interface on Comm boards
	Sp->NV_UI.unit_type = UNIT_125V;									//-!- BatMon_V_range type of unit (hardware - defined): if ((SysData.NV_UI.unit_type != 24) && (SysData.NV_UI.unit_type != 48) && (SysData.NV_UI.unit_type != 125) && (SysData.NV_UI.unit_type != 250)) SysData.NV_UI.unit_type = 125;
	Sp->NV_UI.unit_index = index125;									// corresponds to unit_type, range (0...3), used to access array Alarm_Limits[]
	Sp->NV_UI.V4 = 90;													// 0x028  2  // 4mA voltage point (SysData.NV_UI.V4)
	Sp->NV_UI.V20 = 180;												// 0x02A  2  // 20 ma voltage point (SysData.NV_UI.V20) Low byte
	setBit(Sp->NV_UI.disabled_alarms, DIS_ALARM_ALL_BITS) ;				// DisAllow alarms by setting all "disable" bits
	clearBit(SysData.NV_UI.SavedStatusWord, (Buzzer_ON_eq1_Bit | Latch_ON_eq1_Bit | SinglePhase_eq0_3ph_eq1_Bit | CurOut_I420_eq0_I01_eq1_Bit));	// no buzzer or latch alarms, 1-phase, 4-20 mA output

	Sp->dll_timeout = DEF_DLL_TIMEOUT;									// dll timeout
	Sp->inter_char_gap = DEF_INTER_CHAR;								// inter-char gap, default = 1041
	Sp->xmt_delay = DEF_XMT_DELAY;										// xmt delay
																		// whether or not latch on some event or when condition clears, restore normal operation
	rt.pulse = OFF;

	Sp->NV_UI.StartUpProtocol = DNP3;									// SysData.NV_UI.StartUpProtocol byte, enum DNP or MODBUS or ASCII_CMDS or ASCII_MENU
	rt.protocol_parity = NONE;											// 0 == NONE; 1 == rt.UART_parity EVEN; 2 == rt.UART_parity ODD
	Sp->dll_confirm = 0;												// dll confirm status
	Sp->app_confirm = 0;												// Modbus UART_parity
	Sp->dll_retries = 0;												// dll retries
	for (ctr = 0; ctr < 11; ctr++)										// future use
		Sp->extra1_bytes[ctr] = 0;

#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
	Sp->input_type[1] = BIPOLAR;						// 0x021  1  // type of analog input - uni-polar or bi-polar
	Sp->input_type[2] = BIPOLAR;						// 0x022  1  // type of analog input - uni-polar or bi-polar
	Sp->input_type[3] = BIPOLAR;						// 0x023  1  // type of analog input - uni-polar or bi-polar
	Sp->input_type[4] = BIPOLAR;						// 0x024  1  // type of analog input - uni-polar or bi-polar
	Sp->input_type[5] = BIPOLAR;						// 0x025  1  // type of analog input - uni-polar or bi-polar
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
	Sp->analog_points = DEF_NUM_OF_POINTS;				// 0x026  1  // number of active analog input points

	// LCD board settings start at 0x030
	for (ctr = 0; ctr < 16; ctr++)
	Sp->FrontBoardBytes[ctr] = 0;						// 0x030  16  // future use

	// Calibration floats start at 0x040
	Sp->BatteryVolts.X1_lowADCcounts  = 13367;				// 0x040  4  // X1 Battery Voltage calibration, ADC counts
	Sp->BatteryVolts.Y1_lowCalibrVal = 90.0f;				// 0x044  4  // Y1 Battery Voltage calibration, desired Volts = 90 V
	Sp->BatteryVolts.X2_highADCcounts = 21736;				// 0x048  4  // X2 Battery Voltage calibration, ADC counts
	Sp->BatteryVolts.Y2_highCalibrVal = 130.0f;				// 0x04C  4  // Y2 Battery Voltage calibration, desired Volts = 130 V

	Sp->FaultVolts.X1_lowADCcounts  = 15947;				// 0x050  4  // X1 Fault Voltage calibration, ADC counts
	Sp->FaultVolts.Y1_lowCalibrVal = 0;						// 0x054  4  // Y1 Fault Voltage calibration, desired Volts = 0 V
	Sp->FaultVolts.X2_highADCcounts = 26989;				// 0x058  4  // X2 Fault Voltage calibration, ADC counts
	Sp->FaultVolts.Y2_highCalibrVal = 130.145f;				// 0x05C  4  // Y2 Fault Voltage calibration, desired Volts = 130 V

	Sp->MinusGndVolts.X1_lowADCcounts  = 17965;				// 0x060  4  // X1 Minus Grnd voltage correction factor info, ADC counts
	Sp->MinusGndVolts.Y1_lowCalibrVal = 79.998f;			// 0x064  4  // Y1 Minus Grnd voltage correction factor info, desired Volts = 80 V
	Sp->MinusGndVolts.X2_highADCcounts = 9639;				// 0x068  4  // X2 Minus Grnd voltage correction factor info, ADC counts
	Sp->MinusGndVolts.Y2_highCalibrVal = 130.145f;			// 0x06C  4  // Y2 Minus Grnd voltage correction factor info, desired Volts = 130 V

	// IK20251119: added transformer in line with Vbat, @ 60hz: 216 mV = 148 counts, 1.46 mV / count ratio, 0.685 count / mV
	// real frequency should be 120 Hz or 360 Hz, so riple voltage sensor output should be higher, BUT T HAS NOT BEEN TESTED YET
	Sp->RippleVolts1ph.X1_lowADCcounts  = 27.0f;			// 0x070  4  // X1 single phase ripple voltage calibration, ADC counts
	Sp->RippleVolts1ph.Y1_lowCalibrVal = 40.0f;				// 0x074  4  // Y1 single phase ripple voltage calibration, desired Volts = 40 mV
	Sp->RippleVolts1ph.X2_highADCcounts = 206.f;			// 0x078  4  // X2 single phase ripple voltage calibration, ADC counts
	Sp->RippleVolts1ph.Y2_highCalibrVal = 300.0f;			// 0x07C  4  // Y2 single phase ripple voltage calibration, desired Volts = 300 mV

	Sp->RippleVolts3ph.X1_lowADCcounts  = 50.0f;			// 0x080  4  // X1 three phase ripple voltage calibration, ADC counts
	Sp->RippleVolts3ph.Y1_lowCalibrVal = 40.0f;				// 0x084  4  // Y1 three phase ripple voltage calibration, desired mVolts = 50 mV
	Sp->RippleVolts3ph.X2_highADCcounts = 410.0f;			// 0x088  4  // X2 three phase ripple voltage calibration, ADC counts
	Sp->RippleVolts3ph.Y2_highCalibrVal = 300.0f;			// 0x08C  4  // Y2 three phase ripple voltage calibration, desired mVolts = 400 mV

	// IK20251119: from current measurement: 3phase signal is approximately 2.75 times higher than single phase signal
	// IK20251120: transformer is connected to current sensor inputs, produces 217 mV @ 60 Hz, and measured ADC counts = 4443. Ratio = 20.47 counts / mV
	// According to "Copy of ABB vs ANNEALED Electroswitch Cur Sensor 11-10-23.xlsx", 100 mA current @ 60 Hz produces 446.3 mV output from the current sensor.
	// thus, 217 mV is approximately 443/217 = 48.6 mA current but this is for 60 Hz.
	// This test setup with this transformer @60 Hz ratio = 48.6 mA / 217 mV = 0.224 mA / mV, or 48.6 ma / 4443 counts = 0.01093 mA / count; or 91.4 counts / mA
	// At 120 Hz, current sensor output is higher by 660/446.3 = 1.48 times, setup should produce 4443 * 1.48 = 6579 counts, ratio = 6579 / 48.6 mA = 135.4 counts / mA
	// At 360 Hz, current sensor output is higher by 994.2/446.3 = 2.23 times, setup should produce 4443 * 2.23 = 9909 counts, ratio = 9909 / 48.6 mA = 203.9 counts / mA

	Sp->RippleCurr1ph.X1_lowADCcounts = 2031.0f;			// 0x090  4  // X1 single phase ripple current calibration, ADC counts = 15 * 135.4 counts/mA = 2031 counts
	Sp->RippleCurr1ph.Y1_lowCalibrVal = 15.0f;				// 0x094  4  // Y1 single phase ripple current calibration, desired mA = 15 mA
	Sp->RippleCurr1ph.X2_highADCcounts = 9749.0f;			// 0x098  4  // X2 single phase ripple current calibration, ADC counts = 72 * 135.4 counts/mA = 9748.8 counts
	Sp->RippleCurr1ph.Y2_highCalibrVal = 72.0f;				// 0x09C  4  // Y2 single phase ripple current calibration, desired mA = 72 mA

	Sp->RippleCurr3ph.X1_lowADCcounts = 3058.0f;			// 0x0A0  4  // X1 three phase ripple current calibration, ADC counts = 15 mA * 203.9 counts/mA = 3058 counts
	Sp->RippleCurr3ph.Y1_lowCalibrVal = 15.0f;				// 0x0A4  4  // Y1 three phase ripple current calibration, desired mA = 15 mA
	Sp->RippleCurr3ph.X2_highADCcounts = 23448.0f;			// 0x0A8  4  // X2 three phase ripple current calibration, ADC counts = 115 mA * 203.9 counts/mA = 23448.5 counts
	Sp->RippleCurr3ph.Y2_highCalibrVal = 115.0f;			// 0x0AC  4  // Y2 three phase ripple current calibration, desired mA = 115 mA

	// IK20251120 : 4-20 mA current output calibration, SPECIAL CASE, ADC does not measure it.
	// Instead, external Digital Multi Meter measures current loop output current at low point (at 0 mA or 4 mA) and at high point (at 1mA or 20 mA)
	// the X coordinate is a measured current value
	// the Y coordinate is a duty cycle value to produce desired current output
	Sp->CurrentOut_I420.X1_lowADCcounts = 4.0f;				// 0x0B0  4  // X1 SysData.CurrentOut_I420.Y1_lowCalibrVal  // duty cycle for low mA voltage point (DC4)
	Sp->CurrentOut_I420.Y1_lowCalibrVal = 0.86721f;			// 0x0B4  4  // Y1 SysData.Bat_Cal_Offset_Volts_f = 0;      // battery offset
	Sp->CurrentOut_I420.X2_highADCcounts = 20.0f;			// 0x0B8  4  // X2 SysData.CurrentOut_I420.Y2_highCalibrVal // duty cycle for high mA voltage point (DC20)
	Sp->CurrentOut_I420.Y2_highCalibrVal = 0.33497f;		// 0x0BC  4  // Y2 dummy_cal_f = -1; 0x0BC  4  // future use

	for (ctr = 0; ctr < EXTRA_FLOATS_NUM; ctr++)			// 0x0C0  4*EXTRA_FLOATS_NUM     // future use
		Sp->extra_floats[ctr] = 0;							// 0x0D0  4*EXTRA_FLOATS_NUM     // future use

	//--- if EXTRA_FLOATS_NUM = 8 next address is  0x0E0
}

/*****************************************************************/
void SetDefaultsInRAM(void)
{														// adr offset, bytes  size
	SetSysDataDefaultsInRAM();							// set defaults for specific system, from init.c file
	// memcpy(destination, source, size);
	memcpy((void*)&SysData, (void*)&DefaultSysdata, sizeof(SysData));
	//manual_mode = TRUE;
	Is_in_auto_mode = FALSE;						// IK20240227 auto mode is negated manual mode
	limit_mode = FALSE;
}

/*********************************************************************/
/*                      I N I T   P A R A M E T E R S                */
/*********************************************************************/
/*  Description:  Firmware first loads default data into SysData, then checks EEPROM signature byte
	(example: saved by FW version 0x30) and if starting firmware (after upgrade) has FW version 0x31 -
	the EEPROM is not read - instead the default SysData structure is saved into EEPROM.
	If signatures match - the contents of the EEPROM copy of SysData are replacing default settings.
	Inputs:       None.
	Outputs:      SysData gets either default or customized data
	Notes:        None
	Revisions:    05/26/15     REC     Created
					20240103   IK      Refactored
																	 */
/*********************************************************************/
void Init_Parameters(void)
{
	__disable_interrupt();
	SetDefaultsInRAM();	// IK20240104 first - load default data

	Get_EEPROM_params(); // IK20240104 than, GET SYSTEM PARAMETERS FROM FLASH if data is valid
	// ===== PROTOCOL STARTUP - ALWAYS SETUP =====
	// ALWAYS start with SETUP protocol @ 9600 baud, regardless of saved protocol
	rt.operating_protocol = SETUP;              // Acting protocol = SETUP
	SysData.NV_UI.baud_rate = Baud_9600;        // Force 9600 baud
	Existing.BRate_index = Baud_9600_i;
	Existing.baud_rate = Baud_9600;

	// Initialize UART with SETUP protocol settings
	//Init_UART();
	// Start the 6-second startup timer
	//timer.start_up_ms = 6000;                    // 6 seconds in SETUP mode
	//timer.TWI_lockup = 13000;                    // 13 sec TWI lockup timer

	// Note: SysData.NV_UI.StartUpProtocol contains the STORED protocol
	// It will be activated after timer.start_up_ms expires
	// ===== END PROTOCOL STARTUP FIX =====

	SetCurrentLoopVoltagePoints();

	// *** Now check 4-20 ma calibration *****
	// ***  check duty cycle for low mA voltage point (DC4)    ****
	if ((isnan(SysData.CurrentOut_I420.Y1_lowCalibrVal)) || (SysData.CurrentOut_I420.Y1_lowCalibrVal < 0.2f) || (SysData.CurrentOut_I420.Y1_lowCalibrVal > 1.1f)) // the "FF FF FF FF"" accessed as a float gives -NAN
		SysData.CurrentOut_I420.Y1_lowCalibrVal = 0.86666f;			// default 4-20ma if new unit

	// ***  check duty cycle for high mA voltage point (DC20)    ****
	if ((isnan(SysData.CurrentOut_I420.Y2_highCalibrVal)) || (SysData.CurrentOut_I420.Y2_highCalibrVal < 0.1f) || (SysData.CurrentOut_I420.Y2_highCalibrVal > 1.1f)) // the "FF FF FF FF"" accessed as a float gives -NAN
		SysData.CurrentOut_I420.Y2_highCalibrVal = 0.3333f;			// default if out of range

	// ***      check Battery voltage correction factor info    ***
	if (isnan(SysData.Bat_Cal_Offset_Volts_f)) // IK20231218 empty EEPROM? the "FF FF FF FF"" accessed as a float gives -NAN
		SysData.Bat_Cal_Offset_Volts_f = 0;								// make it zero
	__disable_interrupt();							// store three copies

	display_mode = INIT;							// set to start off with init screen, shows "ARGA"

	iien1 = 0x80;                              // set restart
	iien2 = 0;                                 // clr iin2

	Existing.BRate_index = SysData.NV_UI.BRate_index; // call from Init_Parameters() after Get_EEPROM_params()
	__enable_interrupt();												// enable global interrupts
}

#ifndef PC
__no_init volatile uint8 reset_cause @ 0x100;  // place in RAM, not initialized

__intrinsic void capture_reset_cause(void) {
	reset_cause = MCUSR;   // store reset reason
	MCUSR = 0;             // clear flags
	WDTCSR = (WDCE) | (WDE); // disable WDT
	WDTCSR = 0;
}
#endif //not aPC

void WDT_Prescaler_Change(void)
{
	__disable_interrupt();
	WATCHDOG_RESET();
	// Start timed sequence
	WDTCSR |= (WDCE | WDE);
	// within 4 cycles, set new prescaler(time-out) value = 64K cycles (~0.5 s)
	WDTCSR = (WDE | WDP2 | WDP0);
	__enable_interrupt();
}

void WDT_off(void)
{
	__disable_interrupt();
	__watchdog_reset();
	// Clear WDRF in MCUSR
	clearBit(MCUSR, WDRF);//This bit is set if a Watchdog Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag.
	// Write logical one to WDCE and WDE
	// Keep Existing prescaler setting to prevent unintentional time-out
	setBit(WDTCSR,(WDCE | WDE)); // Watchdog Timer Control Register - prepare to disable WTD by writing 1 to both bits
	// write to WDT within 4 cycles
	WDTCSR = 0x00; // Turn off WDT
	__enable_interrupt();
}

void WDT_set_2sec_reset(void)
{
	__disable_interrupt();
	WDTCSR = (WDCE | WDE);  // unlock, to set seconds in watchdog
	WDTCSR = (WDCE| WDP2 | WDP1 | WDP0) ; //0x0F;	// 2 sec
/* IK20250915 why setting these flags - they update on reset?
// bits   Bit7 Bit6 Bit5  Bit4   Bit3   Bit2   Bit1   Bit0
	MCUSR &= (0 | 0 | 0 | JTRF | WDRF | BORF | EXTRF |PORF); //0xF7;
	*/
	__enable_interrupt();
}

void WDT_set_2sec_interrupt(void)
{
	__disable_interrupt();
	WDTCSR = (WDCE | WDE);  // unlock, to set seconds in watchdog
	WDTCSR = (WDIE| WDP2 | WDP1 | WDP0) ; //0x47;	// interrupt enabled, 2 sec
	__enable_interrupt();
}

/// <summary>
/// function checks and initializes UART baud rate based on Existing.BRate_index, which is set from EEPROM or default value in Init_Parameters()
/// then saves in the SysData.NV_UI and in EEPROM
/// if index is outside the range, it is set to default Baud_19200_i
/// a new BR_index and baud rate based on Baud_Rates[BR_index] array
/// Then it intialyzes UART baudrate register and if BR is different, re-inits UART
/// </summary>
void Init_UART() {
	uint8 c;
	UCSR0C = 0x06;				// no UART_parity, async, 1 stop
	UCSR0A = UCSR0A_SETTINGS;	// default set a RX reset (0x20) and double speed mode (0x02) flags, to get 2.4% accuracy at 115200 baud
	UCSR0B = 0x98;				// enable rcv & xmt

	Set_BaudRate(Existing.BRate_index);
	for (c = 0; c < 16; c++)	//IK20260203 some delay to settle UART
	{
		_NOP();
	}
}

/*********************************************************************/
/*                      I N I T                                      */
/*********************************************************************/
/*  Description:  This routine initializes all variables and registers.
	Inputs:       None.
	Outputs:      None
	Notes:        None
	Revisions:    07/12/06     REC     Created
																	 */
/*********************************************************************/
void init(void)
{
	uint16 i;
	EchoStatus = ECHO_ENABLED;						// AT20230615 enable echo by default
	FirmwareVersion = FW_ver_float;					// initialization of global variable with firmware version number
#ifdef PC
	initEEsimulator();
#endif
	Init_Parameters();								// Initialize to defaults, then read data from EEPROM
	/*-------- Initialize variables ---------*/
	timer.comm_activity = 0;
	comm_state = NO_ACTIVITY;					// in enum UART_Events
	all_stations_msg = false;

	// Note: SysData.NV_UI.StartUpProtocol contains the STORED protocol
	// It will be activated after timer.start_up_ms expires

	// Initialize UART with SETUP protocol settings
	Init_UART();
	// Start the 6-second startup timer
	timer.start_up_ms = 6000;                    // 6 seconds in SETUP mode
	timer.IO_update = 2000;							// Initially, two sec to update IO board (501C firmware)
	timer.TWI_lockup = 13000;                    // 13 sec TWI lockup timer

#ifdef TEST_MODE
	WDT_off();
#else
	WDT_set_2sec_interrupt(); //IK2020509015
#endif

	CLKPR = 0x80; // IK20260203 Clock Prescale Register, set to 1:1 vs. input clock signal

	/*------- Set Up Timers -----*/
	/***     Timing based on a 16 MHz crystal(0.0000000625 Second tick ****/
	/*-------- Timer 0 8-bit Timer/Counter used for ADC acquisition ---*/

	TCCR0A = 2; // TCCR0A_WGM01; <<<BUG, this supposed to be 2 but gives 0   // Clear Timer on Compare Match (CTC) mode
	TCCR0B = TIMER_0_PRESCALER_SET;					// IK20250528 enable counter, prescaler divides clk by 64
	OCR0A = 249;									// IK20250528 defines period, OCR0A = 249 at clk/64 gives 1 ms interrupt
	// at TIMER_0_PRESCALER_SET:
	//          clk/8    clk/64  clk/1024
	// setting -> period  us
	//   2                 12
	//   4                 24
	//   10                44
	//   64               260
	//   128              516
	//   156                     10000
	//   199     100
	//   200     101      804
	//   249             1000    16000
	//   250             1004    16000

// IK20250530 - to disable interrupts, set all flags in TIMSK0 to zero
	TIMSK0 = 0x00;			// enable timer 0 interrupt: bit0=Overflow, Bit1==Compare Match A, Bit2==Compare Match B
							// The OCF0B. OCF0A, TOV0 bits are set when a Compare Match occurs between the Timer/Counter and the data in OCR0x or overflow - Output Compare Register 0 'x'.
							// OCF0B is cleared by hardware when executing the corresponding interrupt handling vector. Alternatively, OCF0B is cleared by writing a logic one to the flag.
	TIFR0 = 0x07;									// clear timer 0 Flag register: : bit0=Overflow, Bit1==Compare Match A, Bit2==Compare Match B

	/* -- Timer 1, 16-bit, used for 4-20ma (PWM)--- */
	ICR1 = 0xFFFF;			// set max
	TCCR1A = 0x82;			// Clear OC1A on CMP match-Timer mode 14
	TCCR1B = 0x19;			// Timer mode 15 - no prescale
	TCCR1C = 0x00;
	OCR1A = 65535;			// init PWM to 0
	TIMSK1 = 0x03;			// enable timer 1
	OCR1A = 65535;			// PWM of 1%

	/* -- Timer 2 - System 8-bit Timer with prescaler counts UP to TIMING_INTERRUPT_SETTING, then resets to zero value */
							// IK20240328 changed divide by 128 instead of by 8, changed OCR2A setting to 124 instead of 200, removed prescaler = 10
							// communication was not stable
	OCR2A = TIMING_INTERRUPT_SETTING;	// IK20250925 corrected timer settings from 200 to 199, to get exactly 100us. IK20241217 restored timing, it WAS 1 ms:  refactored timing: changed for 1000 us (1 ms) instead of 100 us
	TCCR2A = 0x02;			// normal counter, clear tmr on compare
	TCCR2B = 0x02;			// IK20241217 restored timing:
/* snip while working OK
ASSR =0
GTCCR= 0
OCR2A  = 0xc8
OCR2B  = 0x00
TCCR2A = 0x02
TCCR2B = 0x00
TCNT2  = 0x02 <<< changing
TIFR2  = 0x06
TIMSK2 = 0x02

snip while producing comm errors
ASSR =0
GTCCR= 0
OCR2A  = 0x7c ----- different
OCR2B  = 0x00
TCCR2A = 0x02
TCCR2B = 0x05 ----- different
TCNT2  = 0x02 <<< changing
TIFR2  = 0x06
TIMSK2 = 0x02
*/
	TIMSK2 = 0x02;			// enable timer 2A compare int
	ASSR = 0x00;			// clk from xtal
	ACSR = 0x80;			// shut off analog comparator
	/*-------- Set up TWI -------*/
	TWBR = 03;				// TWI baud rate 77 kHz
	TWSR = 0x02;			// TWI prescaler set to 16
	TWCR = 0xC4;			// TWI, addr, & TWI intrpt not enabled
	/*-------- Set Up I/O -------*/
	//This code leaves all outputs inactive			// I = input; O = output; A = Alternate Function
	DDRA = PORTA_DDR;		// I I I O O I I O
	PORTA = PORTA_INIT;		// Pull ups on inputs
	DDRB = PORTB_DDR;		// I O I O O A A A
	PORTB = PORTB_INIT;		// Pull ups on inputs
	DDRC = PORTC_DDR;		// A A I I I I I I
	PORTC = PORTC_INIT;		// Pull ups on inputs
	DDRD = PORTD_DDR;		// O I O O O O I I
	PORTD = PORTD_INIT;		// Pull ups on inputs

	__disable_interrupt();

	WATCHDOG_RESET();
	WDTCSR = 0x18;			// 2 second watchdog
	WDTCSR = 0x0F;			// 2 sec
	MCUSR &= 0xF7;
	__enable_interrupt();

	WATCHDOG_RESET();

	TCNT1 = 64535;			// Set Timer 1 for 1 ms intervals.

	clearBit(XMT_ENABLE, XMT_ON); //IK20250523 set recieve mode
	send_setup = SEND_NOTHING;
	send_modbus = SEND_NOTHING;
	send_dnp = SEND_NOTHING;
	__enable_interrupt();
	restart_op = false;
	/*   get device parameters */
	WATCHDOG_RESET();

	/*   get initial status   */
	timer.Generic = 5;
	WATCHDOG_RESET();
	while (timer.Generic != 0);						// wait for the others
	WATCHDOG_RESET();
	iien1 = iien1 & 0xBF;							// clr dev trouble bit
	measurement_ID = ADC_BATT_VOLTS;				//start off getting the battery voltage
	ADC_Status = ADC_SETUP;							//start off setting up mux address
	for (i = 0; i < HOST_RX_BUFF_LEN; i++)			// init incoming buffer
		rt.HostRxBuff[i] = 0xFF;					// so clear out the buffer with nonsense
	WATCHDOG_RESET();
	iien1 = 0x80;									// set restart bit
}

/*************************************************************/
/*                    Delay_ms                               */
/*************************************************************/
/* Description: This function is called by anything required some delay
   It uses timer.Generic variable which is decremented in interrupt
   Inputs:      required delay in milliseconds
   Outputs:     none
   Notes: DO NOT USE FROM WITHIN INTERRUPT, ONLY USE FROM MAIN LOOP

   Revision:    2023-Jun-01        IK     Created.           */
void Delay_ms(uint16 Delay)
{
	timer.Generic = Delay;
	while (timer.Generic != 0)
	{
#ifdef PC
		Sleep(0);
#endif
	}
}

/***************************************************************/
void SaveParams(void)
{
	__disable_interrupt();
	// reverce operation vs. EEPROM_Read_Mem_Block(EE_SYS_DATA_OFFSET, (uint8*)&SysData, sizeof(SysData));
	EEPROM_Write_Mem_Block((uint8*)&SysData, EE_SYS_DATA_OFFSET, sizeof(SysData));
	clearBit(rt.OperStatusWord, EE_DATA_0_NO_DATA_1_Bit); //data valid - reset bit
	__enable_interrupt();
}
uint8 ReadButtons() {
	uint8 momentaryButtonsState = 0;
	//#ifdef TIME_TESTING
	//	SetTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
	//#endif // #ifdef TIME_TESTING

#ifndef PC
	TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_GET_FRONT_BUTTONS, 0, 0);
	// Display board answers with something like this:	TWI_Write(TWI_SEND_FRONT_BUTTONS, 0x11, 0x22); // IK20250114 test send 2-bytes of buttons states: [0]=0x0C=12D; [1]=0x11=17D; [2]=0x22=34D
	TWI_Read(DISPLAY_READ);               //get info from display board
	if (twi.s.error > 0) {
		return 0x80; // upper bit set - receiving error
	}
	if (twi.buffer[BYTE_1] == TWI_SEND_FRONT_BUTTONS) // Display board answers with first byte = TWI_MSG_ALARMS==14 or TWI_SEND_FRONT_BUTTONS ==12 (0x0C
	{
		Display_Info.buttons_hits = twi.buffer[BYTE_2]; // IK20250924 confirmed, it gets bits from Display board
		if (twi.buffer[BYTE_2] != 0) // IK20250924 for test, remeber last pressed button (it is not cleared to 0 if button is released)
			Display_Info.last_butt_pressed = twi.buffer[BYTE_2];
	}
#endif // #ifndef PC
	momentaryButtonsState = (uint8)Display_Info.buttons_hits;
	//#ifdef TIME_TESTING
	//	ClearTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
	//#endif // #ifdef TIME_TESTING
	return momentaryButtonsState;
}
/*************************************************************/

/*************************************************************/
/*                    M A I N                                */
/*************************************************************/
#ifdef PC
BOOL PC_LOOP_INITIALIZED = FALSE; //let main thread initialize / substitute variables
#endif


void main(void)
{
#ifdef PC
	int i = 0;
	PC_LOOP_INITIALIZED = FALSE;								// let main thread initialize / substitute variables
	new_thread();
#endif
	Num_RCI_commands = NumOfCommands();
	init();														// initialize the device
#ifdef PC
	clrLCD();
	gotoxy(1, 1);
	Delay_ms(200);												// give time to LCD window to paint itself
	PC_LOOP_INITIALIZED = TRUE;
#endif
#ifdef ASCII_TESTING
	rt.operating_protocol = ASCII_CMDS;							//-!- IK20241222 overwrite for test
	setBit(rt.Host , CharEchoFlag);
	Print_FW_Version();
	SendCrLf();
#endif //#ifdef ASCII_TESTING

#ifdef TEST_PRINTF
	int NumOfChars= 12;
	// equivalent of printf below, with customized output to UART, specific for RS-485 chip which needs to enable/disable transmitter or receiver
	sprintf(wrk_str, "sprintf const int %5d\r\n", 23);			// test conversion
	PutStr (wrk_str );  // test
	sprintf(wrk_str, "sprintf var int %5d\r\n", NumOfChars);	// test conversion
	PutStr (wrk_str );  // test
	sprintf(wrk_str, "sprintf float %5.2f\r\n", 4.5);			// test conversion
	PutStr (wrk_str );  // test

	//IK20230612 printf(...%d) decimal calls PrintfTiny, size ~100 bytes; default (auto) printf((...%f) float calls _PrintfLargeNoMb, size ~6kB;  Printf Formatter -> "Small" ~2 kB (setting in Options->General>Library Opton 1 )
	printf(" printf const int %d\r\n", 23);						// test
	printf(" printf var int %d\r\n", NumOfChars);				// test
	printf(" printf float %5.2f\r\n", 4.5);						// test
#endif // TEST_PRINTF
	// now loop 'forever', taking input when interrupted
	while (1)
	{
#ifdef PC //instead of interrupt
		delay_us(1000); //Sleep(1);
		TIMER2_COMPA_interrupt();								// CHECK IF CMD RECEIVED FROM HOST
		check_ch(&i);											// PC input stored in rt.HostRxBuff, button emulation is in "i"
#endif
		WATCHDOG_RESET();
		// periodic display refresh
		if (Display_Info.DisplayNeedsUpdateFlag == SET)			// flag is set each 20 ms in a timer interrupt
		{
#ifdef TIME_TESTING
			SetTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
#endif // #ifdef TIME_TESTING
			Write_Numeric_Display(Display_Info.DigitalStr);		// DigitalStr is prepared in PrepareDisplay(), here it is just sent to display board
			Write_ASCII_Display(Display_Info.InfoStr);			// InfoStr is prepared in PrepareDisplay(), here it is just sent to display board
			Write_LEDs_OnDisplayBoard(((uint8)Display_Info.Status) | LEDsControlledByCommBrd);
			Display_Info.DisplayNeedsUpdateFlag = CLR;
#ifdef TIME_TESTING
			ClearTestPin44;										// IK20250523 reset test pin #44 (easy to solder to)
#endif // #ifdef TIME_TESTING
		}
		CheckExecuteFrontPanelCmd();							// inside is front panel menu
		//if (timer.TWI_request == 0) // IK20250919 added to have stable periodic check of front panel buttons
		//{
		//	timer.TWI_request = 20 ; // once in 20 ms check front buttons on display board
		//}
		// IK20250812 moved into interrupt to have more precise timing
		//if ((rt.battery_deciVolts > 140) &&					// between 14 and 380 vdc
		//	(rt.battery_deciVolts < 3800))						// waits till getting good battery voltage
		//	Check_Alarms();										// to start checking alarms

		//if (limit_mode == TRUE)									// if limit mode kill any alarms
		//	Display_Info.alarm_status = 0;							// so you can view the settings

		// IK20251219 re-arranged protocol check in the order of likiness of use
		if (msg_status == MSG_STARTED)							// New RS485 Msg?
		{
			if (rt.operating_protocol == DNP3)					// if in DNP
			{
				Parse_DNP_Msg();								// parse it as such
				msg_status = MSG_DONE;							// tell all that its done
				num_of_inbytes = 0;								// reset number of incoming bytes
			}

			else if (rt.operating_protocol == MODBUS)			// if in Modbus
			{
				if (timer.ModBus_100us == 0)					// end of msg ?
				{
					Parse_Modbus_Msg();							// no so process it
					msg_status = MSG_DONE;						// tell all that its done
					parity_error = false;						// init to look for UART_parity error
					num_of_inbytes = 0;							// reset number of incoming bytes
				}
			}
		}

		if (rt.operating_protocol == SETUP)
		{
			if (msg_status == MSG_ARRIVED)						// if in setup mode
			{
				Parse_Setup_Msg();								// parse it as such
				msg_status = MSG_DONE;							// tell all that its done
				num_of_inbytes = 0;								// reset number of incoming bytes
			}
			// Just in case recovery
			Existing.BRate_index = Baud_9600_i;
			Set_BaudRate(Existing.BRate_index);					// it also sets Existing.baud_rate = Baud_9600;


			//timer.TWI_lockup = 13000;							// keep this board from timing out, 13 sec
			TWI_Write(ALARM_WRITE, TWI_MSG_ALARMS, 0, 0);		// Also, Send TWI stuff
			TWI_Read(ALARM_READ);								// to keep other boards from timing out and resetting

			//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_BATT_VOLTS, 0, 0);	// and resetting

			//---------> this executes ONCE after initial 6 seconds in SETUP protocol if there is no commands arrive <-------------------------------------------
			if (timer.start_up_ms == 0)
			{
				// Timer expired and still in SETUP mode - switch to stored protocol
				Get_EEPROM_params();							// Get all the stored parameters into SysData

				// Don't switch to ASCII_CMDS from EEPROM (it's production-only)
				if (SysData.NV_UI.StartUpProtocol == ASCII_CMDS) // Prevent accidents, ASCII is not saved into EEPROM
				{
					SysData.NV_UI.StartUpProtocol = DNP3;  // Default to DNP3
					SaveToEE(SysData.NV_UI.StartUpProtocol);
				}

				// Now activate the stored protocol
				rt.operating_protocol = SysData.NV_UI.StartUpProtocol;

				// Set appropriate baud rate for the protocol
				if (rt.operating_protocol == DNP3 || rt.operating_protocol == MODBUS)
				{
					Existing.BRate_index = SysData.NV_UI.BRate_index;
					Existing.baud_rate = SysData.NV_UI.baud_rate;
				}

				// Reinitialize UART with new settings
				Init_UART();
				// Clear receive buffer for new protocol
				rt.HostRxBuffPtr = rt.EchoRxBuffPtr = 0;
				msg_status = MSG_DONE;

				display_mode = VOLTS;				// IK20260205 after 6 sec of SETUP and shoving "ARGA", switch to show "BAT" voltage

				if (rt.operating_protocol == MODBUS)
				{
					rt.protocol_parity = SysData.app_confirm;			// IK20260205 update realtime copy of MODBUS parity
					if (rt.protocol_parity == EVEN)						// ==1? is rt.UART_parity even?
						UCSR0C = 0x26;									// make it even
					if (rt.protocol_parity == ODD)						// ==2? is SysData.UART_parity odd?
						UCSR0C = 0x36;									// make it odd
					rt.first_register = SysData.NV_UI.modbus_first_reg;	// IK20260501 separate location
				}
				else if (rt.operating_protocol == DNP3) 					// init incoming buffer
				{
					UCSR0C = 0x06;										// no UART_parity, async, 1 stop
					rt.host_address = SysData.NV_UI.host_address;		// DNP3 uses host address
					memset(rt.HostRxBuff, 0xFF, HOST_RX_BUFF_LEN);		// so clear out the buffer with nonsense
				}
				else // (rt.operating_protocol == ASCII)
				{
					SetASCIIandSignMsg();
				}
			}	// end of if ((timer.start_up == 0)
		}		// && (SysData.NV_UI.StartUpProtocol == SETUP))	//start up timed out

		else if (rt.operating_protocol == ASCII_CMDS)			// New IK20241030
		{
			//	CHECK PC communication
			// IK20250527 happen each 150 ms: program does not go there for 125ms, and parces for 12.5 ms??
			// IK20250606 fixed - replaced 120 ms while(ADC_timer ) {} delay in ADC conversion with check and return if ADC_timer >0
			ParseRCI();
			msg_status = MSG_DONE;								// tell all that its done
		}

		// IK20231214 not set to any value //
		if (timer.TWI_hangup == 0)
		{
			iien1 = iien1 | 0x40;                 //set dev trouble bit
		}

		if (((cal_status & (RECEIVED_EXT_LO_VALUE | RECEIVED_EXT_HI_VALUE)) != 0) && (timer.Calibration == 0))	// battery cal
		{
			cal_status = 0;                      // didn't get measurements in time
			calibr_step = CALIBRATION_DONE;      // so end it.//added 8/14/19
		}

		if ((send_dnp != SEND_NOTHING) || (send_modbus != SEND_NOTHING) || (send_setup != SEND_NOTHING))
		{
			if (SysData.NV_UI.StartUpProtocol == DNP3)
				Send_DNP_Msg(send_dnp);            //yeh go send it
			if (SysData.NV_UI.StartUpProtocol == MODBUS)
				Send_Modbus_Msg(send_modbus);      //yeh go send it
			if (SysData.NV_UI.StartUpProtocol == SETUP)
				Send_Setup_Msg(send_setup);        //yeh go send it
		}

		if (
// never			(measurement_ID != 0) &&
//            (timer.ADC_ms == 0) &&
#ifndef PC
			(SysData.NV_UI.StartUpProtocol != SETUP) && 	//-!- IK20250304 enable measurement during setup?
#endif
			 (msg_status == MSG_DONE))
		{
			//IK20250528 Measure() takes 121 ms due to while() loop inside waiting for ADC conversion to finish
			//IK20250605 refactored Measure() so it does not have the while() loop inside
			Measure(); // uses globals measurement_ID, ADC_Status
			//IK20250605 at the end of Measure() when state machine is reset, ADC_Status changes to ADC_SETUP but timer.ADC_ms is not set yet
			if ((timer.ADC_ms == 0)
				&& (ADC_Status == ADC_SETUP)) // ==8
			{
				// update display and set next measurement
				if (measurement_ID == ADC_BATT_VOLTS)			// if == 1 just got battery volts then send it
				{
					//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_BATT_VOLTS, rt.battery_deciVolts & 255, rt.battery_deciVolts >> 8);//to the display board
					measurement_ID = ADC_FAULT_VOLTS;			// = 2
				}												// send low first then high byte
				else if (measurement_ID == ADC_FAULT_VOLTS)		// if == 2 just got fault volts then send it
				{
					//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_FAULT_VOLTS, rt.gnd_fault_deciVolts & 255, rt.gnd_fault_deciVolts >> 8);  //to the display board
					measurement_ID = ADC_MINUS_GND_VOLTS;		// = 3
				}
				else if (measurement_ID == ADC_MINUS_GND_VOLTS)	// if == 3
				{
					//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_MINUS_GND_VOLTS, rt.minus_gnd_deciVolts & 255, rt.minus_gnd_deciVolts >> 8);//to the display board
					measurement_ID = ADC_RIPPLE_CURRENT;		// = 5
				}
				else if (measurement_ID == ADC_RIPPLE_CURRENT)	// if == 5 just got ripple current then send it
				{
					//TWI_Write(DISPLAY_WRITE_TWI_ADR, TWI_RIPPLE_CURRENT, rt.ripple_milliAmpers & 255, rt.ripple_milliAmpers >> 8);//to the display board
					measurement_ID = ADC_RIPPLE_VOLTAGE;		// = 6
				}
				else if (measurement_ID == ADC_RIPPLE_VOLTAGE)	// if == 6 just got ripple volts then send it
				{
					measurement_ID = DISPLAY_DATA;		// = 20 = 0x14
				}
			}//end of if (ADC_Status == ADC_SETUP)

			if (measurement_ID == DISPLAY_DATA) // if == 20 0x14 time to send & rcv display data to/from Display board
			{
			// since display menu code incorporated into Comm board code, no need to send messages to Display board
				if ((((tmp_display_status ^ Display_Info.Status) & DISP_STATE_PulseON_BIT) == 0) ||		//  CHECK Bitwise exclusive OR
					((tmp_display_status ^ Display_Info.Status) & DISP_SELECTED_3PH_BIT) == 0)
				{												// Phase or bbpulse selection changed
					io_update = true;
					tmp_display_status = Display_Info.Status;
				}
				Parse_Display_Data();

				measurement_ID = RELAY_DATA;
			}//end if (measurement_ID == DISPLAY_DATA)

			else if (measurement_ID == RELAY_DATA)														// == 21 time to send & rcv relay data
			{
				if (cal_status != CALIBRATION_DONE)														// if calibrating shut off pulse
					clearBit(Display_Info.Status, DISP_STATE_PulseON_BIT);
				TWI_Write(ALARM_WRITE, TWI_MSG_ALARMS, Display_Info.alarm_status, Display_Info.Status);
				TWI_Read(ALARM_READ);
				relay_board_status = twi.buffer[BYTE_2];												// get AC power fail bit
				{
					float t_f = SysData.NV_UI.alarm_delay_sec_f * 0.1f;
					uint16 delay_deci_sec = (uint16)(t_f);
					tmp_byte = delay_deci_sec >> 8; // msg_high
					TWI_Write(ALARM_WRITE, TWI_ALARM_DELAY, delay_deci_sec & 0xFF, tmp_byte );
				}
#ifdef LAST_GASP
				tmp_byte = Display_Info.alarm_status >> 8;
				TWI_Write(ALARM_WRITE, SEND_TWI_ALARMS2, tmp_byte, NULL_BYTE);							// send 2nd byte of alarms
#endif // #ifdef LAST_GASP
				measurement_ID = IO_DATA;
			}//end RELAY_DATA

			else if (measurement_ID == IO_DATA) // time to send & rcv data to IO
			{
				if ((io_update == true)								// only do when necessary i.e. changed (change is detected by main() )
					|| (timer.IO_update == 0))						// and occasionally, once in 1 sec
				{
					TWI_Write(TWI_ADR_WRITE_IO, 3, 0, 0);			// config for outputs
					// IK20250228 added serial control to ripple voltage amplifier hardware gain
					if (rt.ripple_calibration_phase == CalibrateSinglePhase)
						tmp_byte = 0x00;							// single phase
					else if (rt.ripple_calibration_phase == Calibrate_ThreePhase)
						tmp_byte = 0x01;							// 3 phase
					else // everything else if (rt.ripple_calibration_phase == 0)	// in normal operation? - it takes settings from Display board, 1 phase or 3 phase, and changes scale factor set accordingly
					{                                           // yes - use display_status word sent by Display board - it tells if uint in 1 phase or 3 phase
						if ((Display_Info.Status & DISP_SELECTED_3PH_BIT) != 0)		// get phase type   //-!- IK20250226 HERE IF FU#%^##% BUG in rev J! during calibration, we DO need to set phase via menu on Display board
							tmp_byte = 0x01;						// 3 phase
						else
							tmp_byte = 0x00;						// single phase
					}
					TWI_Write(TWI_ADR_WRITE_IO, 1, tmp_byte, 0);	// set outputs
					io_update = false;
					timer.IO_update = 1000; //every 1.0 sec
				}
				measurement_ID = ADC_BATT_VOLTS;					// = 1, Back to beginning
				ADC_Status = ADC_SETUP;								// = 8, Back to beginning
			}//end IO_DATA
		}   //end if (msg_status == MSG_DONE)

		if (restart_op == true)                 //restart request
		{
#ifndef PC
			TWI_Write(DISPLAY_WRITE_TWI_ADR, RESTART, 0, 0);		// reset display board
			TWI_Write(ALARM_WRITE, RESTART, 0, 0);					// reset relay board
			while (1) { _NOP(); };									// reset myself
#endif // PC
		}

#ifndef PC
		if (timer.TWI_lockup == 0)									// haven't rcvd twi in awhile (in 13 seconds)
			//IK20250612 replaced global reset with just TWI_reset
			// while (1);                          //then reset
			TWI_Reset();
#endif
		if ((TWSR & TWSR_STATUS_MASK) == 0)							// illegal start or start
			TWCR |= 0x90;											// releases SCL and SDA in rcv

		if (comm_state != NO_ACTIVITY)
		{
			if (comm_state == RCV) // in enum UART_Events
			{
				if (timer.comm_activity == 0)
				{
					timer.comm_activity = UART_NO_ACTIVITY_TIMEOUT;	//-!- IK20240110 timer.comm_activity was decrementing TWICE in TIMER2_COMPA_interrupt, removed another decrement and reduced timeout from 700 to 350
					if (RxGreen_TxRed_LED_op == GREEN)
					{
						comm_state = NO_ACTIVITY;					// in enum UART_Events
						RxGreen_TxRed_LED_op = NONE;
					}
				}
				else
				{
					RxGreen_TxRed_LED_op = GREEN;
					Comm_LED_Op(RxGreen_TxRed_LED_op);
				}
			}//end comm activity is RCV
			if (comm_state == RCVANDXMT)							// in enum UART_Events
			{
				if (timer.comm_activity == 0)
				{
					timer.comm_activity = UART_NO_ACTIVITY_TIMEOUT;	//175ms //-!- IK20240110 timer.comm_activity was decrementing TWICE in TIMER2_COMPA_interrupt, removed another decrement and reduced timeout from 700 to 350
					if (RxGreen_TxRed_LED_op == GREEN)
						RxGreen_TxRed_LED_op = RED;
					else
						if (RxGreen_TxRed_LED_op == RED)
						{
							comm_state = NO_ACTIVITY;			// in enum UART_Events
							RxGreen_TxRed_LED_op = NONE;
						}
						else
							RxGreen_TxRed_LED_op = GREEN;
				}//end comm_activity tmr is 0
				else
				{
					Comm_LED_Op(RxGreen_TxRed_LED_op);
				}
			}// end if (comm activity != NO_ACTIVITY)
		}
		else // there is NO activity
		{
#ifndef INTERRUPT_TIME_TESTING
			Comm_LED_Op(RxGreen_TxRed_LED_op);
#endif // #ifndef INTERRUPT_TIME_TESTING
		}
		if (timer.PWM_calibration == 0)								// if not calibrating
			I420_calibr_lock = I420calibUNLOCKED;							// keep cal type ready

		// IK20251219 comment: this is for testing only
		if ((rt.OperStatusWord & (RealTimeDataReady_eq1_Bit | SendRealTimeData_eq1_Bit)) == (RealTimeDataReady_eq1_Bit | SendRealTimeData_eq1_Bit)) // IK20250820 it will clear in main loop when sending brief info
		{
			clearBit(rt.OperStatusWord, RealTimeDataReady_eq1_Bit);
			// Print_System_Info(REALTIME_SNAPSHOT);
#ifdef PC // for debugging, show delta time
			gotoxy(1, 20); printf("D=%5d", timeDelta); // IK20260128
#endif //PC
			timer.RealTimeUpdate = 1; // to start incrementing timer again
		}

		// IK20251217 comment: below "Just in case" repair settings to the correct values

		//-- Set up COMMS ---
		UCSR0A = UCSR0A_SETTINGS;	// default set a RX reset (0x20) and double speed mode (0x02) flags, to get 2.4% accuracy at 115200 baud
		UCSR0B = 0x98;										// enable rcv & xmt,
		if (UBRR0 != rt.UBRR0_setting)		// Baud register or baud rate changed
		{
			Set_USART_UBBRregister((Uint32)Existing.baud_rate);					// set UART to new baud rate
			if (SysData.NV_UI.StartUpProtocol == ASCII_CMDS) // if ASCII protocol send sign-on message
			{
				Print_FW_Version();
				SendMsgToPC("\r\n>~OK");
			}
		}
		//-------- Set Up I/O -------
		//This code leaves all outputs inactive
		DDRA = PORTA_DDR;									// I I I I I I I I
		DDRB = PORTB_DDR;									// I I I O I I I I
		DDRC = PORTC_DDR;									// I I I I I I TWI TWI
		DDRD = PORTD_DDR;									// O I O O O O I I
	/* IK20241217 remarked for test
		//if (OCR2A != TIMING_INTERRUPT_SETTING)			// if compare value change
		//	OCR2A = TIMING_INTERRUPT_SETTING;				// change it back for 100us
		// IK20240328 remarked for test
		if (OCR2A != TIMING_INTERRUPT_SETTING)				// if compare value change
			OCR2A = TIMING_INTERRUPT_SETTING;				// change it back for 100us
		if (TCCR2A != 0x02)									// if normal port op or CTC changed
			TCCR2A = 0x02;									// make it right
		if (TCCR2B != 0x02)									// if divide by 8 changed
			TCCR2B = 0x02;									// fix it to divide by 8
		*/
		if (TIMSK2 != 0x02)			// IK20250925 if interrupt got shut off - turn it back on. Bit 1 - OCIE2A: Timer/Counter2 Output Compare Match A Interrupt Enable
			TIMSK2 = 0x02;			// When the OCIE2A bit is written to one and the I-bit in the Status Register is set (one), the Timer / Counter2 Compare Match A interrupt is enabled.
		ASSR = 0x00;
		if (ACSR != TWINT)			// IK20250925 if Analog Comparator Control and Status ACD bit is cleared comparator is enabled. shut off analog comparator.
			ACSR = 0x80;			// When this bit is written logic one, the power to the Analog Comparator is switched off.This will reduce power consumption in Active and Idle mode.

		//-- Set up TWI ---
		TWBR = 0x03;										// TWI baud rate 77 kHz
		TWSR |= 0x02;										// TWI prescaler set to 16
		TWSR &= 0xFE;										// TWI prescaler set to 16
		if ((TWCR & TWEN) == 0)								// twi became disabled
			setBit(TWCR, TWEN);								// re-enable it
		if ((SREG & 0x80) == 0)								// did global interrupts get turned off?
			__enable_interrupt();							// then turn back on
#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
		SysData.input_type[1] = BIPOLAR;					// UNIPOLAR removed 5-24-18 forces it
		SysData.input_type[2] = BIPOLAR;					// UNIPOLAR removed 5-24-18
		SysData.input_type[3] = BIPOLAR;					// UNIPOLAR removed 5-24-18
		SysData.input_type[4] = BIPOLAR;					// UNIPOLAR removed 5-24-18
		SysData.input_type[5] = BIPOLAR;					// UNIPOLAR removed 5-24-18
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
	}
} // main() end



extern void Do_Front_menu(void);

void CheckExecuteFrontPanelCmd(void) //in firmware, it is called from main loop. Only when waiting char input in simulation, is called from int get_ch(void)
{
	ReadButtons();	 // read momentary states via TWI
	Get_Button_Press();//	CHECK IF BUTTONS HAVE BEEN PUSHED in firmware
	if ((Display_Info.ButtonStateChanged != CLR) && (Display_Info.DisplayNeedsUpdateFlag == CLR))		// Button pushed? actual button states are reflected in "Butt_states"
	{
		if (Display_Info.butt_states) // avoid too short button presses when no bits are set
			if ((Display_Info.ProcessingButton != SET) && (Display_Info.ButtonStateChanged == BTN_RELEASED))
			{
				Display_Info.ProcessingButton = SET;
				//Display_Info.ProcessingButton = CLR;
//				Display_Info.DisplayNeedsUpdateFlag = SET;
			}
	}
	Do_Front_menu();								// Buttons : BIT_0 = 1 LEFT pressed, BIT_3 = 1 central ENTER pressed, BIT_4 = 1 RIGHT pressed, BIT_5 = 1 UP pressed, BIT_6 = 1 DOWN pressed
	if (Display_Info.DisplayNeedsUpdateFlag == SET)
		DisplayPrepare(); // create needed display messages

	if (rt.InfoLED_blink_eq1 == TRUE)
	{
		if (timer.InfoLED_blink_ms == 0) {
			timer.InfoLED_blink_ms = SegLEDblinkPeriod_ms >> 1; // =/2
			Display_Info.Info_segments_on_off = 1 - Display_Info.Info_segments_on_off;	// toggle between ON (1) and off (0)
		}
	}
	else {
		Display_Info.Info_segments_on_off = ON;
	}
#ifdef PC
	static Uint32 Time_Stamp;
	if ((timer.time_keep - Time_Stamp) < TICKS_IN_mSEC * 10) return;				// slow down, wait 200 ms (update 5 times per sec)
	Time_Stamp = timer.time_keep; //save time stamp for next entry
#endif //PC
}

void SetLED(int LEDindex, int ColorIndex) {
#ifdef PC
	hbr_LEDcolor_ptr[LEDindex] = hbr_Color_ptr[ColorIndex];
#else // COMM board

#endif
}