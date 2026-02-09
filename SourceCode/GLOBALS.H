/*****************************************************************************
* Comm Board FW
* Filename: Globals.h
*
* $Source:  $
* Last Saved Revision Number $Revision:  $
* Last Saved Revision $Date: $
* Last Saved Revision $Author:  $
*
* Original Author:  Igor Kordunsky
* Date: 18-Jan-2024
*
* File Description: All global variables, macros and function definitions
*
*****************************************************************************/

#ifndef GLOBALS
#define GLOBALS

extern float FirmwareVersion;     //Version 3.0
#define STRINGS_IN_FLASH // FOR IAR compiler; TO WORK, THIS OPTION MUST BE DEFINED in project options->compiler->extra options! --string_literals_in_flash   to place strings in flash

#define DEF_BAT_MONITOR_ADDR	2			// default Battery Monitor RS-485 address
#define DEF_HOST_ADDR			3			// default host address/Modbus first reg
#define DEF_BAUD_RATE			Baud_19200	// default baud rate

#define MAX_V_I_rip_LIMIT		2000
#define MAX_DNP3_ADDRESS	((uint16)65519)

// default timeouts
#define LIMIT_SAFETY_INACTIVITY_TIMEOUT 150000 // after ten minute of no button press revert to manual mode

// unit type us defined as nominal battery voltage, 125 V (125V = 10 x (12.5V batteries) in series is most typcal)
#define UNIT_24V                        24
#define UNIT_48V                        48
#define UNIT_125V                       125
#define UNIT_250V                       250



// default values
#define V_to_mV                             1000  // milli voltage scale: voltage 125V = 125000 mV, use 32 bit int to store
//#define V_to_centiV                          100  // voltage scale: save voltage in 10 mV increments into Uint16, i.e. 125V = 12500 counts fits in the range [0...65535]
//#define V_to_deciV                           10   // voltage scale: save voltage in 100 mV increments into Uint16, i.e. 125V = 1250 counts fits in the range [0...65535]
//#define CentiV_to_Volt                    (0.01)  // how many counts in 1 Volt, use to convert stored int16 voltage to float volts

#define DEF_UnitType                    UNIT_125V  // default unit type 125 V
// voltage definitions in 10th of Volts
#define MIN_24V_high_bat_threshold          25.0f  // 24V min allowed high alarm threshold 25.0V
#define DEF_24V_high_bat_threshold          28.5f  // 24V default high alarm threshold     28.5V
#define MAX_24V_high_bat_threshold          30.0f  // 24V max allowed high alarm threshold 30.0V

#define MIN_24V_low_bat_threshold           20.0f  // 24V min allowed low alarm threshold  20.0V
#define DEF_24V_low_bat_threshold           21.0f  // 24V default low alarm threshold      21.0V
#define MAX_24V_low_bat_threshold           25.0f  // 24V max allowed low alarm threshold  25.0V

#define MIN_24V_minus_gf_threshold           2.6f  // 24V min allowed minus ground fault threshold     2.6V
#define DEF_24V_minus_gf_threshold           2.6f  // 24V default minus ground fault alarm threshold   2.6V
#define MAX_24V_minus_gf_threshold          20.0f  // 24V max allowed minus ground fault threshold    20.0V

#define MIN_24V_plus_gf_threshold         MIN_24V_minus_gf_threshold // 24V min allowed plus ground fault threshold
#define DEF_24V_plus_gf_threshold         DEF_24V_minus_gf_threshold // 24V default plus ground fault threshold
#define MAX_24V_plus_gf_threshold         MAX_24V_minus_gf_threshold // 24V max allowed plus ground fault threshold

#define MIN_48V_high_bat_threshold          50.0f  // 48V min allowed high alarm threshold 50.0V
#define DEF_48V_high_bat_threshold          57.0f  // 48V default high alarm threshold     57.0V
#define MAX_48V_high_bat_threshold          60.0f  // 48V max allowed high alarm threshold 60.0V(

#define MIN_48V_low_bat_threshold           40.0f  // 48V min allowed low alarm threshold  40.0V
#define DEF_48V_low_bat_threshold           42.0f  // 48V default low alarm threshold      42.0V
#define MAX_48V_low_bat_threshold           50.0f  // 48V max allowed high alarm threshold 50.0V

#define MIN_48V_minus_gf_threshold           5.2f  // 24V min allowed minus ground fault threshold     5.2V
#define DEF_48V_minus_gf_threshold           5.4f  // 48V default minus ground fault alarm threshold   5.4V
#define MAX_48V_minus_gf_threshold          40.0f  // 24V max allowed minus ground fault threshold    40.0V

#define MIN_48V_plus_gf_threshold         MIN_48V_minus_gf_threshold // 48V min allowed plus ground fault threshold
#define DEF_48V_plus_gf_threshold         DEF_48V_minus_gf_threshold // 48V default plus ground fault threshold
#define MAX_48V_plus_gf_threshold         MAX_48V_minus_gf_threshold // 48V max allowed plus ground fault threshold

#define MIN_125V_high_bat_threshold         125.0f // 125V min allowed high alarm threshold 125.0V
#define DEF_125V_high_bat_threshold         142.0f // 125V default high alarm threshold     142.0V
#define MAX_125V_high_bat_threshold         150.0f // 125V max allowed high alarm threshold 150.0V

#define MIN_125V_low_bat_threshold          100.0f // 125V min allowed low alarm threshold  100.0V
#define DEF_125V_low_bat_threshold          104.0f // 125V default low alarm threshold      104.0V
#define MAX_125V_low_bat_threshold          125.0f // 125V max allowed high alarm threshold 125.0V

#define MIN_125V_minus_gf_threshold          13.0f // 125V min allowed minus ground fault threshold     13.0V
#define DEF_125V_minus_gf_threshold          13.0f // 125V default minus ground fault alarm threshold   13.0V
#define MAX_125V_minus_gf_threshold         100.0f // 125V max allowed minus ground fault threshold    100.0V

#define MIN_125V_plus_gf_threshold        MIN_125V_minus_gf_threshold // 125V min allowed plus ground fault threshold
#define DEF_125V_plus_gf_threshold        DEF_125V_minus_gf_threshold // 125V default plus ground fault alarm threshold
#define MAX_125V_plus_gf_threshold        MAX_125V_minus_gf_threshold // 125V max allowed plus ground fault threshold

#define MIN_250V_high_bat_threshold         250.0f // 250V min allowed high alarm threshold 250.0V
#define DEF_250V_high_bat_threshold         277.0f // 250V default high alarm threshold     277.0V
#define MAX_250V_high_bat_threshold         300.0f // 250V max allowed high alarm threshold 300.0V

#define MIN_250V_low_bat_threshold          200.0f // 250V min allowed low alarm threshold  200.0V
#define DEF_250V_low_bat_threshold          210.0f // 250V default low alarm threshold      210.0V
#define MAX_250V_low_bat_threshold          250.0f // 250V max allowed high alarm threshold 250.0V

#define MIN_250V_minus_gf_threshold          26.0f // 250V min allowed minus ground fault threshold     26.0V
#define DEF_250V_minus_gf_threshold          27.0f // 250V default minus ground fault alarm threshold   27.0V
#define MAX_250V_minus_gf_threshold         200.0f // 250V max allowed minus ground fault threshold    200.0V

#define MIN_250V_plus_gf_threshold        MIN_250V_minus_gf_threshold // 250V min allowed plus ground fault threshold
#define DEF_250V_plus_gf_threshold        DEF_250V_minus_gf_threshold // 250V default plus ground fault alarm threshold
#define MAX_250V_plus_gf_threshold        MAX_250V_minus_gf_threshold // 250V max allowed plus ground fault threshold

#define MIN_ripple_voltage_threshold           5 // min allowed ripple voltage threshold     5 mV
#define DEF_ripple_voltage_threshold         200 // default ripple voltage threshold        20 mV
#define MAX_ripple_voltage_threshold        2000 // max allowed ripple voltage threshold  2000 mV

#define MIN_ripple_current_threshold           5 // min allowed ripple current threshold     5 mA
#define DEF_ripple_current_threshold          10 // default ripple current threshold        10 mA
#define MAX_ripple_current_threshold        2000 // max allowed ripple current threshold  2000 mA

// time_delay for trigger alarm saved as 1 count = 100 ms (0.1 sec, deciSecond)
#define MIN_time_delay                        1.0f  // min allowed time delay      1 sec
#define INIT_time_delay                       2.0f  // time delay set              2 sec
#define MAX_time_delay                       59.0f  // max allowed time delay      59 sec
#define INCREMENT_time_delay                  1.0f  // time delay increment        1 sec

// dealing with ripple created by battery charger type. =1 for single phase charger (120 Hz ripple), =3 for three phase charger (360 Hz ripple)
// 1 = one (single) phase or 3 (three phases), correlates to 120Hz or 360Hz ripple
#define Single_Phase_Setting            1
#define Triple_Phase_Setting            3
#define DEF_Phase_Setting               Single_Phase_Setting    // 1 = one (single) phase, 120Hz ripple

#define DEF_BuzzerState                 OFF
#define DEF_PulseState                  OFF
#define DEF_LatchState                  OFF


//#define CREATE_FAKE_DATA
#define OUTPUT_TEST_DATA_TO_FILE

// standard: the Comm board firmware is 501A, Display board 501B
#define TWI_TIMEOUT_ms          5	//IK20250924 changed to 5 ms // IK20250103 not more than 255, used with volatile uint8 TWI_hangup_tmr;
#define TWI_UPDATE_PERIOD_ms    50 // 50 ms between TWI communication with Display board (read buttons and update LEDs and displays, used with volatile uint16 timer.TWI_request;
// IK20240319 replaced enums with #define
// used with 'calibration' and 'measurement_ID' and it is the index of ADC_buff[]
#define ADC_BATT_VOLTS         1 // = TWI_BATT_VOLTS
#define ADC_FAULT_VOLTS        2 // = TWI_FAULT_VOLTS
#define ADC_MINUS_GND_VOLTS    3 // = TWI_MINUS_GND_VOLTS
#define ADC_RIPPLE_CURRENT     5 // = TWI_RIPPLE_CURRENT
#define ADC_RIPPLE_VOLTAGE     6 // = TWI_RIPPLE_VOLTAGE
// IK20250102 TWI message types
#define TWI_BATT_VOLTS         ADC_BATT_VOLTS      // 1 if (calibr_step == TWI_BATT_VOLTS); if (measurement_ID == TWI_BATT_VOLTS); TWI_Write(DISPLAY_WRITE, TWI_BATT_VOLTS, 0, 0); //Comm writes to the display board
#define TWI_FAULT_VOLTS        ADC_FAULT_VOLTS     // 2 if (calibr_step == TWI_FAULT_VOLTS); if (measurement_ID == TWI_FAULT_VOLTS); TWI_Write(DISPLAY_WRITE, TWI_FAULT_VOLTS, gnd_fault_deciVolts & 255, gnd_fault_deciVolts >> 8); //Comm writes to the display board
#define TWI_MINUS_GND_VOLTS    ADC_MINUS_GND_VOLTS // 3 if (calibr_step == TWI_MINUS_GND_VOLTS); if (measurement_ID == TWI_MINUS_GND_VOLTS); TWI_Write(DISPLAY_WRITE, TWI_MINUS_GND_VOLTS, minus_gnd_deciVolts & 255, minus_gnd_deciVolts >> 8); //to the display board
#define TWI_ALARM_VOLTS        4  // 4 not used in RelayBoardFW nor CommBoardFW
#define TWI_RIPPLE_CURRENT     ADC_RIPPLE_CURRENT  // 5 if (calibr_step == TWI_RIPPLE_CURRENT); if (measurement_ID == TWI_RIPPLE_CURRENT); TWI_Write(DISPLAY_WRITE, TWI_RIPPLE_CURRENT, ripple_i & 255, ripple_i >> 8); //Comm writes to the display board
#define TWI_RIPPLE_VOLTAGE     ADC_RIPPLE_VOLTAGE // 6 if (calibr_step == TWI_RIPPLE_VOLTAGE); if (measurement_ID == TWI_RIPPLE_VOLTAGE); TWI_Write(DISPLAY_WRITE, TWI_RIPPLE_VOLTAGE, ripple_v & 255, ripple_v >> 8); //Comm writes to the display board

#define TWI_EXTRA_ANALOG_CH     7       //-!- IK20240319 who is setting it? if (measurement_ID == TWI_EXT_ANALOG)
#define TWI_SETUP_ADC           8 // if (ADC_Status == TWI_SETUP_ADC)
#define TWI_START_CONVERSION    9 // if (ADC_Status == TWI_START_CONVERSION)
#define TWI_DISPLAY_BD_STATUS   10 // Comm writes: TWI_Write(DISPLAY_WRITE, TWI_DISPLAY_BD_STATUS, relay_board_status, unit_type); then reads: TWI_Read(DISPLAY_READ); then alarm_status = twi.buffer[BYTE_2]; alarm_status2 = alarm_status;
#define TWI_GET_FRONT_BUTTONS   11 // IK20250106 - Display board reports status of the buttons
#define TWI_SEND_FRONT_BUTTONS  12 // 0x0C IK20250106 - return status of front board buttons to Comm board
#define TWI_SET_DISPLAY_LEDs    13 // 0x0D IK20250106 - command to Display board to turn ON / OFF LEDs

#define TWI_MSG_ALARMS          14 // 0x0E if (send_type == TWI_ALARMS) { TWI_Write(DISPLAY_WRITE, TWI_DISPLAY_BD_STATUS, relay_board_status, unit_type);...
#define TWI_ALARM_DELAY         15 // 0x0F Comm writes if (measurement_ID == RELAY_DATA)(...TWI_Write(ALARM_WRITE, ALARM_DELAY, tmp_word, tmp_byte);
#define TWI_MSG_DISPLAY_BAUD    16 // used in CommBoard
#define TWI_MSG_COMM_ADDRESS    17 // if (send_type == TWI_COMM_ADDRESS) { ... TWI_Write(DISPLAY_WRITE, TWI_COMM_ADDRESS, tmp_byte, meter_address);

#define RESTART_CMD             19 // not used in CommBoard
#define SEND_TWI_ALARMS         TWI_ALARMS
//#define SEND_TWI_ALARMS2        1

//#define SEND_VIA_TWI_BAUD_RATE  TWI_MSG_DISPLAY_BAUD //-!- IK20240319 mixed local identifier and between-the-boards TWI identifier
#define DISPLAY_DATA            20 // if (measurement_ID != DISPAY_DATA) { ... TWI_Write(A_TO_D_WRITE, 0x88, NULL_BYTE, NULL_BYTE);
#define RELAY_DATA              21 // if (measurement_ID != RELAY_DATA)  && (measurement_ID != IO_DATA) { ... TWI_Write(A_TO_D_WRITE, 0x88, NULL_BYTE, NULL_BYTE);
#define IO_DATA                 22 // if (measurement_ID != IO_DATA)
#define DISPLAY_UPPER_STR_01    32 // IK20250102 - send Display board a string to be shown on upper 5-digits 7-segments LED, left-most position 1 & 2 - index 0 and index 1
#define DISPLAY_UPPER_STR_23    33 // IK20250102 - send Display board a string to be shown on upper 5-digits 7-segments LED, position 3 and pos 4 - index 2 and index 3
#define DISPLAY_UPPER_STR_45    34 // IK20250102 - send Display board a string to be shown on upper 5-digits 7-segments LED, right-most position 5 - index 4 and index 5 if there is one dot
#define DISPLAY_UPPER_STR_67    35 // IK20250103 - send Display board a string to be shown on upper 5-digits 7-segments LED, index 6 and index 7 if there are dots
#define DISPLAY_UPPER_STR_89    36 // IK20250103 - send Display board a string to be shown on upper 5-digits 7-segments LED, index 8 and index 9 if there are dots

#define DISPLAY_LOWER_STR_01    40 // IK20250102 - send Display board a string to be shown on lower 4-symbols 14-segments LED,left positions 1 & 2 - index 0 and index 1
#define DISPLAY_LOWER_STR_23    41 // IK20250102 - send Display board a string to be shown on lower 4-symbols 14-segments LED,right positions 3 & 4 - index 2 and index 3
#define DISPLAY_LOWER_STR_45    42 // IK20250103 - send Display board a string WITH DOTS to be shown on lower 4-symbols 14-segments LED, positions 5 & 6, - index 4 and index 5
#define DISPLAY_LOWER_STR_67    43 // IK20250103 - send Display board a string WITH DOTS to be shown on lower 4-symbols 14-segments LED, positions 7 & 8, - index 6 and index 7


#define DISPLAY_WRITE_TWI_ADR   DISPLAY_WRITE       // 0x40 = 64D
#define TWI_ADR_WRITE_IO        IO_WRITE            // 0x82 = 130D

#define ADC_EXTRA_ANALOG_CH     TWI_EXTRA_ANALOG_CH // = 7   //-!- IK20240319 who is setting it? if (measurement_ID == TWI_EXT_ANALOG)
#define ADC_SETUP               TWI_SETUP_ADC       // = 8   // setting multiplexer and waiting timer.ADC_ms
#define ADC_START_CONVERSION    TWI_START_CONVERSION// = 9
#define ADC_WAIT_FOR_TWI_WRITE  10
#define ADC_WAIT_CONVERSION     11
#define ADC_ASK_TO_READ_TWI     12
#define ADC_READ_RESULT         13

#define  testBitNo(var, bit_no)   (var & (1 << bit_no)) // values 0 or NOT_ZERO
#define  setBitNo(var, bit_no)    (var |= (1 << bit_no))
#define  clearBitNo(var, bit_no)    (var &= ~(1 << bit_no))
#define  updateBitNo(var, bit_no, source)   (var = (var & (~(1 << bit_no))) | (source & (1 << bit_no)))
#define  writeBitNo(var, bit_no, value)   (var = (var & (~(1 << bit_no))) | (value << bit_no))

#define  setBit(var, bit_value)      (var |= bit_value)      //bit_value is Bit_XX, =2^Bit_XX
#define  clearBit(var, bit_value)    (var &= ~(bit_value))
#define  toggleBit(var, bit_value)   (var ^= bit_value)      //0-bit becomes 1, 1-bit becomes 0
#define  testBit(var, bit_value)     (var & bit_value)
#define GLOBAL_Q 16 // DO NOT CHANGE, this gives 16 bits (+-32767) to integer part and 16 bits to a fraction part of quasi float
#define ConvertToQfUnsigned(V)  (Qfloat)((float)V *(1<<GLOBAL_Q) + 0.5f)
#define ConvertToQf(V)  (Qfloat)(V>=0? ((float)V *(1<<GLOBAL_Q) + 0.5f):((float)V * (1<<GLOBAL_Q) - 0.5f))
#define ConvQf_to_F(Q)      ((float)Q)*(1.0f /(1<<GLOBAL_Q))


#define  If_int16_is_negative(var)   if (var & 0x8000)   //check to see if negative number


// define only one option!
// #define  GENERIC		// all variants customized by settings
// #define STANDARD_PANEL
#define DUAL_VOLTAGE_ALARM_TRIP

//#define  TEST_FW		//  does not exist yet


#ifdef PC  // MS Visual C++ compiler: block these words to avoid extra warnings and errors
#define INTERRUPT
#ifndef __cplusplus
 #define register
#endif
#define IN_RAM
#define __ram
//#define inline

//void asm(char* ASM_CMD);
#ifdef __cplusplus
extern "C" {
#endif
__declspec(dllimport) void __stdcall Sleep(unsigned long ms); //for compilation under Windows in MS Visual C++
#ifdef __cplusplus
}
#endif
extern void delay_us(unsigned long long us);
extern void Wait_in_Visual_C(unsigned int Delay);
extern void __no_operation(void);
extern void __watchdog_reset(void);
extern void clrLCD(void);
extern void gotoxy(int x, int y);
char check_ch(int* ptrKEY);
extern void clrscr(void);
extern void ClearScreen(void);
int PUTCHAR(int c); //returning same char
typedef int                 BOOL;
#define FL
#define FLP
#include <windef.h>
#include <winuser.h>
extern HBRUSH hbr_LED_off;//    = RGB(0,0,0);
extern HBRUSH hbr_LED_Yellow;// = RGB(255,255,0);
extern HBRUSH hbr_LED_Green;//  = RGB(0,255,0);
extern HBRUSH hbr_LED_Red;//    = RGB(255,0,0);
extern HWND LedParent;

#define LEDindx_ASCAN	0 // Auto - manual scan
#define LEDindx_ALARM	1
#define LEDindx_TX_RX	2
#define LEDindx_PULSE	3
#define number_of_LEDs	4

extern HBRUSH* hbr_LEDcolor_ptr[number_of_LEDs];
extern RECT* rect_LED[number_of_LEDs];

#define ColorOFF		0
#define ColorRED		1
#define ColorGREEN		2
#define ColorYELLOW		3 // Red+green
#define Total_Colors	4

extern HBRUSH* hbr_Color_ptr[Total_Colors];

#define  SetAutoLEDoff      SetLED(LEDindx_ASCAN,ColorOFF);
#define  SetAutoLEDgreen    SetLED(LEDindx_ASCAN,ColorGREEN);
#define  SetAutoLEDred      SetLED(LEDindx_ASCAN,ColorRED);
#define  SetAutoLEDyellow   SetLED(LEDindx_ASCAN,ColorYELLOW);

#define  SetAlarmLEDoff     SetLED(LEDindx_ALARM,ColorOFF);
#define  SetAlarmLEDgreen   SetLED(LEDindx_ALARM,ColorGREEN);
#define  SetAlarmLEDred     SetLED(LEDindx_ALARM,ColorRED);
#define  SetAlarmLEDyellow  SetLED(LEDindx_ALARM,ColorYELLOW);

#define  SetTxRxLEDoff      SetLED(LEDindx_TX_RX,ColorOFF);
#define  SetTxRxLEDgreen    SetLED(LEDindx_TX_RX,ColorGREEN);
#define  SetTxRxLEDred      SetLED(LEDindx_TX_RX,ColorRED);
#define  SetTxRxLEDyellow   SetLED(LEDindx_TX_RX,ColorYELLOW);

#define  SetPulseLEDoff     SetLED(LEDindx_PULSE,ColorOFF);
#define  SetPulseLEDgreen   SetLED(LEDindx_PULSE,ColorGREEN);
#define  SetPulseLEDred     SetLED(LEDindx_PULSE,ColorRED);
#define  SetPulseLEDyellow  SetLED(LEDindx_PULSE,ColorYELLOW);


#else //ATMEL
 #define INTERRUPT __interrupt
 #define IN_RAM __ram

 #define PutChar Send_Char
 #ifndef STRINGS_IN_FLASH
  #define printf PRINTF // equivalent of printf, with customized output to UART using cputs and char wrk_str[] buffer, specific for RS-485 chip which needs to enable/disable transmitter or receiver
  #define sprintf SPRINTF // equivalent of sprintf,
  #define cputs    PutStr
  #define FL
  #define FLP
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


#endif

// typedef void (*pointer_to_ISR)(void);
// then you can place the address of the IRQ_handler function into interrupt vector using:
// *(pointer_to_ISR *)0x38 = IRQ_handler
//typedef int			(*pointer_to_func)();
typedef signed char			Schar;
typedef unsigned char       Uchar;
typedef unsigned short int  Uint16;
typedef unsigned long       Uint32;
typedef unsigned int        Uint;
typedef short int           Int16;
typedef long                Int32;
typedef char                Boolean;
typedef unsigned char       uint8;
typedef long                Qfloat;	    //quasi-float: float = ((float)(long))/65536; range: -32768...+32767.999984741

#ifdef PC
typedef unsigned short      uint16;
typedef short                int16;
#else // ATMEL
typedef unsigned int        uint16;
typedef int                  int16;
#endif

//typedef const char * const  ConstStrPtrConst; //this line causes compiler to create wrong code, when menu does not select names.
// Int16 -32768....+32767


#ifndef NO_EXT_VAR // all extern definitions
  #define EXT_VAR extern
#else
  #define EXT_VAR
#endif

#define GET_LOW_NIBBLE(IOPORT)		(char)(IOPORT & 0x0F)
#define GET_HIGH_NIBBLE(IOPORT)		(char)((IOPORT & 0xF0)>>4)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')
// USAGE: printf("Leading text "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(byte)); You need all the extra quotes, unfortunately.
//This approach has the efficiency risks of macros (don't pass a function as the argument to BYTE_TO_BINARY) but avoids the memory issues and multiple invocations of strcat

/********* SYSTEM CONFIGURATION SWITCHES *********/
#ifdef GENERIC
//--------------------------------------- 95-36125-01 rev.A
//	#include ".\SYSTEMS\GENERIC\Init.h"
	#define S_GENERIC 1
#else
	#define S_GENERIC 0
#endif
#ifdef TEST_FW
//#include ".\SYSTEMS\TEST_FW\Init.h"
	#define S_TEST_FW 1
#else
	#define S_TEST_FW 0
#endif
#ifdef DUAL_VOLTAGE_ALARM_TRIP
	#define S_DW_FW 1
#else
	#define S_DW_FW 0
#endif
#if (  (S_GENERIC + S_TEST_FW + S_DW_FW) == 0)
	#error define ONE compilation option
#else // sum is not zero
  #if ((S_GENERIC + S_TEST_FW +S_DW_FW) > 1)
	#error More than 1 option defined, define ONLY ONE compilation option
  #endif // sum is more than one
#endif // sum is one


#include "Structure_defs.h"
#include <stdlib.h>
#ifdef __cplusplus
  #include <cstdio>
#endif

/*-------------------------- literals ------------------------------*/

//IK20231205 moved 'literals' from "main.c"
#define INCREMENT           1
#define DECREMENT           -1
#define FALSE               0
#define TRUE                1
#define false               0
#define true                1
#define NULL_BYTE           0
//#define OK                  1
//#define NOT_OK              0
#define GREEN               1
#define RED                 2
#define NONE                0
#define EVEN                1
#define ODD                 2
#define ON                  1
#define OFF                 0
//#define INACTIVE            0
//#define ACTIVE              1
//#define ENABLED             1
//#define DISABLED            0
#define I420calibUNLOCKED   0	// this state is checked: if (I420_calibr_lock == I420calibUNLOCKED)
#define I420calibLOCKED     1	// this state is set during calibration
#define ModBusGOOD          1
#define ModBusBAD           0
//#define FAILED              0 // not used in ATMEL, but defined in VS and causes compilation error
//#define PASSED              1
//#define NOT_READ            5
//#define READ_DONE           6
//#define READY               6
//#define NOT_READY           5
//#define CLEAR               0
#define BTN_PRESSED			1	//IK20251230 need for PC simulation: case WM_LBUTTONDOWN: >>>> Display_Info.ButtonStateChanged = BTN_PRESSED;
#define BTN_RELEASED		2	//IK20251230 need for PC simulation: case WM_LBUTTONDBLCLK: >>>> Display_Info.ButtonStateChanged = BTN_RELEASED;
#define SET		1
#define CLR		0
#define CaRet               0x0D // === '\r' IK20230601 replaced CR with CaRet because Visual Studio C has this defined and redefinition causes errors
#define LF                  0x0A // === '\n'
//#define RESTART_MONITOR     0x99

//#define OCTET_DATA          0x17
//#define TWO_OCTET_DATA      0x28
//#define DLL_NUM_OF_RETRIES  2
//#define RECEIVED            0x33
//#define NOT_RECEIVED        0x66
//#define APP_ERROR           1
//#define ADDR_CLR            0
//#define ALL_CALL            1
//#define MY_ADDR             2
//#define NO_POINT            255
//#define CENTER              0

#define FULL_SNAPSHOT       TRUE
#define BRIF_SNAPSHOT       FALSE
#define REALTIME_SNAPSHOT   -1

#define TxRxLED_Red_PA3   Bit_3		// 0x08
#define TxRxLED_Green_PD7 Bit_7		// 0x80

#define UART_NO_ACTIVITY_TIMEOUT   350  //-!- IK20240110 timer.comm_activity was decrementing TWICE in TIMER2_COMPA_interrupt, removed another decrement. Apparent time increased twice, reduced from 700 to 350
#define UART_BLINK_MilliSec        2    //-!- IK20240110 timer.TxRxLED_blink = 2;  2 ms blink is not perceptable!

/*----------- INPUTS ---------------------*/
/*----------- INPUT MASKS ----------------*/
/*----------- OUTPUTS --------------------*/
#define XMT_ENABLE       PORTD
#define MUXAB_PORT       PORTD
#define MUXC_PORT        PORTA

/*---------- OUTPUT MASKS ----------------*/

#define XMT_ON           Bit_2          // 0x04 for XMT_ENABLE=== PORTD
// PORTD, bit 2 (U1.pin 11) - net XMT-ENABLE - is connected to U2 pins 4,5 -to enable transmitter on RS-485 chip
// the enable transmitter on RS-485 chip is connected to PORTD, bit 2
//#define XMT_OFF          ~XMT_ON        // 0xFB

#define MUXA             0x08
#define MUXB             0x10
#define MUXC             0x40

/************* ERROR HANDLING ***************************************/
#define OK                     0x00  // No error
#define SIO_CMD_ERROR          0x01  // Bad SI/O cmd error - unknown or unintelligible command or function
#define PARAM_ERROR            0x02  // Bad SI/O parameter value - command or function syntax error
#define SYSTEM_NOT_INITIALIZED 0x03  // system is not initialized
#define SYSTEM_BUSY            0x04  // system is busy
#define FLASH_ERASE_ERROR      0x05  // FLASH erase timeout
#define FLASH_WRITE_ERROR      0x06  // FLASH write timeout
#define OPERATION_TIMEOUT      0x07  // timeout
#define OPERATION_WRONG_RESULT 0x08  // function returned wrong result
#define RANGE_ERROR            0x09  // out of range parameter value - command or function syntax error
#define USER_INTERRUPT         0x0A  // user stopped execution of command, AutoCal for example

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR				OK	 //	0			 All's well
#define BAD_SIO_CMD_ERR			SIO_CMD_ERROR	//	 Bad SI/O cmd error bit
#define BAD_VALUE_ERR			PARAM_ERROR		//	 Bad SI/O parameter value

#define LOOP_SAMPLE_FREQUENCY    (float)(10000.0)   //not less than 1000!, it drives Tick_timer ISR and buttons
#define LOOP_RATE                ((long)LOOP_SAMPLE_FREQUENCY)	// used as long. derivative of LOOP_SAMPLE_FREQUENCY, which is float const
#define TICKS_IN_mSEC				(LOOP_RATE/1000)    // !!! LOOP_RATE MUST BE >=1000 !!!

// IK20250814 Bits used with former 'Display_Info.LED_status', now lower byte of Display_Info.Status
// IK20250814 if combine PULSE_BIT used for Pulse LED and DISP_STATE_PulseON_BIT for producing pulses on Relay board,
// using Pulse LED for time measuring would cause pulce generation on Relay board
// better to keep them separate, so that Pulse LED can be used for time measuring and DISP_STATE_PulseON_BIT can be used for Relay board pulse generation

//#define led_status                  Display_Info.Status
// lower byte of Display_Info.Status is used to control LEDs and buzzer on Front board
#define DISP_LED_Auto_ON_BIT        Bit_0	// 0x0001 in Status
#define DISP_LED_Alarm_ON_BIT       Bit_1	// 0x0002 in Status
#define DISP_LED_Pulse_ON_BIT       Bit_2	// 0x0004 in Status
#define DISP_Buzzer_ON_BIT          Bit_3	// 0x0008 in Status

#define LEDsControlledByCommBrd     Bit_6	// 0x0040 if (OverwriteLEDs > LEDsControlledByCommBrd) // if value is more than 64 - easier to test firmware setting 107 to turn all LESs on or 0 to disable overwrite

// IK20250814 these bits used with former 'Display_Info.display_status', now upper byte of Display_Info.Status

#define DISP_STATE_LOmA_BIT         Bit_8	// 0x0100 set for calibration, display_status |= 0x01; // set 4mA bit (or 0mA)
#define DISP_STATE_HImA_BIT         Bit_9	// 0x0200 set for calibration, display_status |= 0x02; // set 20mA bit(or 1mA)
#define DISP_STATE_ButtonDOWN_BIT   Bit_10	// 0x0400 'DOWN' button pressed bit
#define DISP_STATE_ButtonUP_BIT     Bit_11	// 0x0800 'UP' button pressed bit
#define DISP_STATE_VoltCal_BIT      Bit_12	// 0x1000 display_status |= 0x10;    //set Volt cal bit
#define DISP_SELECTED_3PH_BIT       Bit_13	// 0x2000 3phase mode selected - Comm bord uses different calibration settings
#define DISP_STATE_PulseON_BIT      Bit_14	// 0x4000 if (pulse == FALSE)   //set/clr bit in display status; IK20250204 SHOULD NOT BE SAVED IN FLASH! It creates pulses in battery charging line
#define DISP_STATE_0_1mA_BIT        Bit_15	// 0x8000 display_status |= 0x80; //set 0-1ma bit

#define Display_AutoLED_OFF      clearBit(Display_Info.Status, DISP_LED_Auto_ON_BIT)	// Bit_0
#define Display_AutoLED_ON         setBit(Display_Info.Status, DISP_LED_Auto_ON_BIT)	// Bit_0
#define Display_AlarmLED_OFF     clearBit(Display_Info.Status, DISP_LED_Alarm_ON_BIT)	// Bit_1
#define Display_AlarmLED_ON        setBit(Display_Info.Status, DISP_LED_Alarm_ON_BIT)	// Bit_1
#define Display_PulseLED_OFF     clearBit(Display_Info.Status, DISP_LED_Pulse_ON_BIT)	// Bit_2
#define Display_PulseLED_ON        setBit(Display_Info.Status, DISP_LED_Pulse_ON_BIT)	// Bit_2
#define Display_Buzzer_OFF       clearBit(Display_Info.Status, DISP_Buzzer_ON_BIT)		// Bit_3
#define Display_Buzzer_ON          setBit(Display_Info.Status, DISP_Buzzer_ON_BIT)		// Bit_3


#define DISP_BRD_I_CAL_MODE (DISP_STATE_LOmA_BIT | DISP_STATE_HImA_BIT)
#define DISP_BRD_CAL_MODE   (DISP_BRD_I_CAL_MODE | DISP_STATE_VoltCal_BIT)



// used with 'relay_board_status' and 'disabled_alarms'
#define RELAY_BRD_AC_FAIL_BIT       Bit_0 // if ((relay_board_status & 0x01)            //ac pwr fail from alarm board?
#define RELAY_BRD_REQ_RESET_BIT     Bit_1 // if ((relay_board_status & 0x02) == 0x02)   //ext. reset from relay board?

#define ALARM_BLINK_RATE_ms                 300

// used with 'alarm_status', 'latched_alarm_status'
#define DELAY_ms_BEFORE_ALARM_ACCEPTED      25 // wait 25 ms (counter reaches 25) before raising the 'alarm detected' event / set the flag
#define ALARM_COUNTER_CAP_ms                DELAY_ms_BEFORE_ALARM_ACCEPTED*2 // 50 ms maximum counter allowed to increment, because it would count down from 2x25 ms to 25 ms to clear alarm


// to access buttons as array or shift bits according to button
#define BTN_INDEX_AUTO      0
#define BTN_INDEX_LIMIT     1
#define BTN_INDEX_UP        2
#define BTN_INDEX_DOWN      3
#define BTN_INDEX_RESET     4 // mapped to BUTTON_BIT_ENTER
#define BTN_MAX_INDEX       BTN_INDEX_RESET

// definitions for simulation
#define BTN_INDEX_RETURN    BTN_INDEX_AUTO  // mapped to BUTTON_BIT_AUTO
#define BTN_INDEX_ENTER     BTN_INDEX_LIMIT // mapped to BUTTON_BIT_LIMIT
#define BTN_INDEX_MNS       BTN_INDEX_DOWN  // mapped to BUTTON_BIT_DOWN
#define BTN_INDEX_PLS       BTN_INDEX_UP    // mapped to BUTTON_BIT_UP

// used with 'disabled_alarms', setting bit DISABLES alarm
// IK20240228 changed DIS_ALARM_BITs from Bit_0...Bit_3 to Bit_4...Bit_7, matching alarm status bits
#define DIS_ALARM_BIT_RippleVolt        Alarm_Ripple_Voltage_Bit	// Bit_4: if (!(disabled_alarms & 0x10)) // Ripple Voltage alarm enabled
#define DIS_ALARM_BIT_RippleCurr        Alarm_Ripple_Current_Bit	// Bit_5: if (!(disabled_alarms & 0x20)) // Ripple Current alarm enabled
#define DIS_ALARM_BIT_AC                Alarm_AC_Loss_Bit			// Bit_6: if (!(disabled_alarms & 0x40)) // AC alarm enabled
#define DIS_ALARM_BIT_HiZ               Alarm_High_Impedance_Bit	// Bit_7: if (!(disabled_alarms & 0x80)) // Hi Z alarm enabled
#define DIS_ALARM_ALL_BITS              AlarmStatus_Instant_BITS	// if (disabled_alarms > 0xF0) disabled_alarms = 0xF0; //Disable all
#define SegLEDblinkPeriod_ms    1000

//---- General System variables
extern uint8 debug;
extern uint8 I420_calibr_lock;					// Locks (prevents) saving to EEPROM. Calibration Timer is simultaneously set to 5000 ms and starts counting down. After 3000 ms when timer reches below 2000 ms, auto firmware does saving to EEprom

// IK20231214 not checked //extern uint8  debug1;
extern uint8  tmp_byte;                            //general working register
//extern uint8  i;
extern uint8 wrk_str[HOST_XMT_BUFF_LEN];         // building & sending msg
//extern uint8 num_of_inchars;
extern volatile uint8 msg_status;               // holds status of any rcvd msgs
extern volatile uint8 num_of_inbytes;           // number of bytes in buffer

extern uint8  limit_mode;                          //indicates whether limit mode is active
extern uint8  Is_in_auto_mode;                  // IK20251007 combined auto and manual mode in one variable. == TRUE means 'Auto', == FALSE means 'Manual' //indicate whether auto mode is active
//extern uint8  manual_mode;                         //indicates whether manual mode is active
extern uint16 latched_alarm_status;                  // defined in Front_Menu.c, indicates alarms active for local use

#ifdef LNP_TESTER
extern uint8 Zcmd_received;                     // IK20230601 added for parsing
#endif //#ifdef LNP_TESTER

/*------------------------- Structures -----------------------------*/
//typedef struct    // Encapsulate into structure to allow addressing in assembler with increment
//{
//	volatile Uint32 TimeDelayTick;		// use this for delay: set to 0, wait until it reaches delay count
//	volatile Uint32 frame_counter;		// FREE-RUNNING COUNTER to synchronize LCD, buttons, blinking of status LED, etc.
//} TIMER_TICKS;
#ifdef PC
extern long Sim_counter; // free-running increment counter for simulation and GUI update
#endif

/*---------  TWI variables --------*/
struct twi_variables                    // this structure must be initialized
{                                       // to send a msg with retries
	uint8 status;                       // TWI XMT MSG Status (below)
	uint8 destination;                  // board addr to send to (below)
	uint8 msg_type;                     // Type msg (below)
	uint8 data_high;                    // data to be sent high byte
	uint8 data_low;                     // data to be sent low byte
};                                      // Some msgs require NULL BYTES

extern struct twi_variables twi_send;

/*---------  Date & Time for Modbus--------*/
struct date                               //Modbus time
{
   uint8 seconds;
   uint8 minutes;
   uint8 hours;
   uint8 day;
   uint8 month;
   uint8 year;
};
// struct date Day_and_Time;

extern uint8  display_mode;				// used in Front_menu.c, indicates what is being displayed

extern char FL * mode_strings[];/* = {				// used in Front_menu.c, strings for display modes
"LMIT",			// 0   LIMIT_START,
"HBAT",			// 1   HI_BAT_THRESHOLD,
"LBAT",			// 2   LOW_BAT_THRESHOLD,
"+GND",			// 3   PLUS_GF_THRESHOLD,
"-GND",			// 4   MINUS_GF_THRESHOLD,
"RIPV",			// 5   RippleVOLT_TR_HOLD,
"RIPI",			// 6   RippleCURR_TR_HOLD,
"TD  ",			// 7   TIME_DELAY_SET,
"PH  ",			// 8   PHASE_STATE_SET,
"PULS",			// 9   PULSE_STATE_SET,
"BUZZ",			// 10  BUZZER_STATE_SET,	// 10
"LTCH",			// 11  LATCHED_STATE_SET,
"CAL1",			// 12  CAL1_SET_4_20_MODE,	// Calibration mode 4-20 mA
"I LO",			// 13  CAL2_4mA,			// 4mA
"I HI",			// 14  CAL3_20mA,			// 20mA
"CAL5",			// 15  CAL5_SET_0_1_MODE,	// 0-1mA
"0 MA",			// 16  CAL6_0mA,			//-!- not used anymore 0mA; instead show "I Lo"
"1 MA",			// 17  CAL7_1mA,			//-!- not used anymore 1mA; instead show "I Hi"
"VBAT",			// 18  CAL_V_BAT,			// calibrate bat volts             18th item
"SYS ",			// 19  SHOW_FW_VER,		// Show firmware version
"ALAC",			// 20  ALARM_AC,			// AC Alarm Enable/Disable
"ALRV",			// 21  ALARM_RIV,			// Ripple Voltage Enable/Disable   21th item
"ALRI",			// 22  ALARM_RII,			// Ripple Current Enable/Disable
"ALHZ",			// 23  ALARM_HI_Z,			// HI Z Alarm Enable/Disable
"ADDR",			// 24  COMM_ADDR,			// system communication address
"BAUD",			// 25  COMM_BAUD,			// system baud rate
"LMIT",			// 26  PRE_LIMIT_START,
"AUTO",
"BAT ",			// 28  VOLTS,				// 28th item
"+BUS",			// 29  PLUS_GND_VOLTS,
"-BUS",			// 30  MINUS_GND_VOLTS,
"GFV ",			// 31  GND_FAULT_VOLTS,
"RVV ",			// "RIPV",			// 32  RIPPLE_VOLTS,
"RIV ",			// "RIPI",			// 33  RIPPLE_CURRENT,
"TD  ",			// 34  TIME_DELAY_SHOW,
"PH  ",			// 35  PHASE_SHOW,			// 35th
"ARGA",			// 36  INIT,
"OUTU",			// 37  OUTPUT_UPPER_STRING,
"OUTL",			// 38  OUTPUT_LOWER_STRING,
};
*/
/*-------------------------- enums ---------------------------------*/
enum Display_Modes {	// -- used with display_mode --
	// index  //    enum name            // What info display shows if display_mode is set to this enum
					EXIT_MENU = -2,
					END_MENU = -1,
	// LIMT menu, used in LIMIT mode
	/* 0 0x00 */    LIMIT_START = 0,	 // "LMIT"
	/* 1 0x01 */    HI_BAT_THRESHOLD,	 // "HBAT"
	/* 2 0x02 */    LOW_BAT_THRESHOLD,	 // "LBAT"
	/* 3 0x03 */    PLUS_GF_THRESHOLD,	 // "+GND"
	/* 4 0x04 */    MINUS_GF_THRESHOLD,	 // "-GND"
	/* 5 0x05 */    RippleVOLT_TR_HOLD,	 // "RIPV"
	/* 6 0x06 */    RippleCURR_TR_HOLD,	 // "RIPI"
	/* 7 0x07 */    TIME_DELAY_SET,		 // "TD  "
	/* 8 0x08 */    PHASE_STATE_SET,	 // "PH  "
	/* 9 0x09 */    PULSE_STATE_SET,	 // "PULS"
	/*10 0x0A */    BUZZER_STATE_SET,	 // "BUZZ"
	/*11 0x0B */    LATCHED_STATE_SET,	 // "LTCH"
	/*12 0x0C */    SHOW_FW_VER,		 // "SYS "
	/*13 0x0D */    CAL1_SET_4_20_MODE,	 // "CAL1"

// CAL menu, used in CURRENT CALIBRATION mode
	/*14 0x0E */    CAL2_4mA,			 // "I LO"
	/*15 0x0F */    CAL3_20mA,			 // "I HI"
	/*16 0x10 */    CAL5_SET_0_1_MODE,	 // "CAL5"
	/*17 0x11 */    CAL6_0mA,			 // "0 MA"
	/*18 0x12 */    CAL7_1mA,			 // "1 MA"
	/*19 0x13 */    CAL_V_BAT,			 // "VBAT"

// SYS menu, Alarm Disabling/Enabling
	/*20 0x14 */    ALARM_AC,			 // "ALAC"
	/*21 0x15 */    ALARM_RIV,			 // "ALRV"
	/*22 0x16 */    ALARM_RII,			 // "ALRI"
	/*23 0x17 */    ALARM_HI_Z,			 // "ALHZ"
	/*24 0x1A */    SELECT_PROTOCOL,	 // "DNP3"
	/*25 0x18 */    COMM_ADDR,			 // "ADDR"
	/*26 0x19 */    COMM_BAUD,			 // "BAUD"
	/*27 0x1B */    AUTO_MAN_MODE,		 // "AUTO",

//  IK20250724 do not change order of enums from VOLTS to PHASE_SHOW, they are used in front menu to change next value in manual mode
	/*28 0x1C */    VOLTS,				 // "BAT "
	/*29 0x1D */    PLUS_GND_VOLTS,		 // "+BUS"
	/*30 0x1E */    MINUS_GND_VOLTS,	 // "-BUS"
	/*31 0x1F */    GND_FAULT_VOLTS,	 // "GFV "
	/*32 0x20 */    RIPPLE_VOLTS,		 // "RVV "
	/*33 0x21 */    RIPPLE_CURRENT,		 // "RIV "
	/*34 0x22 */    TIME_DELAY_SHOW,	 // "TD  "
	/*35 0x23 */    PHASE_SHOW,			 // "PH  "

// IK20250724 do not change order of items, they are used in front menu to change next value in manual mode
	/*36 0x24 */    INIT,				 // "ARGA"
	/*37 0x25 */    OUTPUT_UPPER_STRING, // "OUTU"
	/*38 0x26 */    OUTPUT_LOWER_STRING, // "OUTL"
};


/*----------Baud Rates -----------------------*/

// these numbers are set in the UBBR to generate the correct rate. Higher setting == lower baud rate
// IK20241030 but, baudrate REGISTER SETTINGS are not used on Display board
// IK20250206 changed baud definitions to be ACTUAL baudrate, NOT the damned UBRR setting, confusing!
enum Baud_Rate_Setting
{ // Enum is actual BR     UBRR setting is calculated
   Baud_300   =   300,  // 3332,                      // IK20231214 BAUD_300 defined in Visual C, replaced with Baud_300
   Baud_600   =   600,  // 1666,                      // Baud_600
   Baud_1200  =  1200,  // 832,
   Baud_1800  =  1800,  // 555,
   Baud_2400  =  2400,  // 416,
   Baud_3600  =  3600,  // 277,
   Baud_4800  =  4800,  // 207,
   Baud_9600  =  9600,  // 103,
   Baud_14400 = 14400,  // 68,
   Baud_19200 = 19200,  // 51
   Baud_115200 = 115200,  // 8, real BR = 111111
};
enum Baud_Rate_index
{
	INVALID_BR_INDEX = -1, // -1 = 0xFF
	Baud_300_i = 0,    // 0
	Baud_600_i,        // 1
	Baud_1200_i,       // 2
	Baud_1800_i,       // 3
	Baud_2400_i,       // 4
	Baud_3600_i,       // 5
	Baud_4800_i,       // 6
	Baud_9600_i,       // 7
	Baud_14400_i,      // 8
	Baud_19200_i,      // 9
	Baud_115200_i,     // 10
	Last_Baud_Index    // 11 - not a baud rate, just number of items in array
};
#define ASCII_BR_INDX	Baud_115200_i   // default baud rate for ASCII commands
extern uint16 const Baud_Rates[Last_Baud_Index];	// IK20250724 to simplify baud rate selection

/*--------- Protocols ------------------------*/
enum Protocols	// IK20251230 CHANGED to regular number
{
   SETUP = 0x00,
   DNP3 =  0x01,		// 17d
   MODBUS = 0x02,		// 34d
   ASCII_CMDS = 0x03,	// 51d
   ASCII_MENU = 0x04	// 68d not implemented
};
#define PROTOCOL_SELECTION_INC_DEC  (DNP3 - SETUP) //

/*---------  Calibration States --------------*/
//**** if tmp_cal value is greater than unit type volts -> converted to mV; example (48 V -> 48000 mV)
// or for RV or RI tmp_cal is above 51 mV (or mA)
// it is concidered  high point calibration, indicated by cal_status bit 2 is set

enum Calibration_Steps
{
	CALIBRATION_DONE, // = 0 calibr_step = CALIBRATION_DONE;
	RECEIVED_EXT_LO_VALUE = Bit_0,	// Low point value calibration RECEIVED via DNP, = setBit(cal_status, Bit_0);
	RECEIVED_EXT_HI_VALUE = Bit_1,	// High point value calibration RECEIVED via DNP, = setBit(cal_status, Bit_1);
	SAVED_LOW_MEASUREMENT = Bit_2,	// received measurement from ADC and saved in X1_lowADCcounts, = setBit(cal_status, Bit_2);
	SAVED_HIGH_MEASUREMENT = Bit_3,	// received measurement from ADC and saved in X2_highADCcounts, = setBit(cal_status, Bit_3);
	NOT_A_CALIBRATION = 100 // calibr_step = NOT_A_CALIBRATION;
};

enum TWI_errors
{
   TWI_OK,				// = 0
   TWI_NOT_READY,		// = 1
   TWI_START_BIT_ERROR,	// = 2
   TWI_DEST_ADR_ERROR,	// = 3
   TWI_SEND_1_ERROR,	// = 4
   TWI_SEND_2_ERROR,	// = 5
   TWI_READ_ERROR		// = 6
};


/*---------  485 Message Status --------------*/
enum RS485_CMD_RCVd_Status
{
	MSG_DONE    = 0,
	MSG_STARTED = Bit_0,	// when '\r' is received from UART Data Register
	GOT_DBL_ENTER = Bit_1,	// when second '\r','\r' is received
	GOT_FULL_TOKEN = Bit_2,	// when '\r','\r', ESC  is received
	MSG_ARRIVED = Bit_3		// set when CaRet '\r' arrives once more, indicating that command string is ready to be parsed
};

/*---------  Board Addresses --------*/
enum Board_Addresses
{
	ALARM_WRITE = 0x10,		// Write address of relay board, used only in main() to send relay states to relay board
	ALARM_READ = 0x11,		// Read address of relay board, returns AC_lost. Also, Send TWI stuff to keep other boards from timing out and resetting
	DISPLAY_WRITE = 0x40,	// IK20241226 used only in main() to send something to Display board
	DISPLAY_READ = 0x41,	// IK20241226 used after TWI_Write(DISPLAY_WRITE,xxxx
	IO_WRITE = 0x82,		// IK20250417 used to set up analog switch and change transfer function of voltage ripple preamp for 1 Phase (Off) or 3 Phase (On)
	IO_READ = 0x83,			// IK20240122 Display board does not communicate with Relay board
	A_TO_D_WRITE = 0xD0,	// used in Measure() -> TWI_Write(A_TO_D_WRITE, 0x88, NULL_BYTE, NULL_BYTE);     //15sps for 16 bits
	A_TO_D_READ = 0xD1,		// used in Measure() -> TWI_Read(A_TO_D_READ);
};

/*---------  TWI RCV'd Bytes --------*/
enum TWI_RCVd_ByteNum
{
	BYTE_1 = 0, // uint8 DestADR,
	BYTE_2 = 1, // uint8 type,
	BYTE_3 = 2, // uint8 msg_low,
	BYTE_4 = 3, // uint8 msg_high
};


/*---------  TWI Data Type  -----------------------*/
enum TWI_Data_Type
{
   NACK = 10,		// IK20241226 not used
   ACK = 11,		// IK20241226 not used
   COMM_STATUS = 12,// IK20241226 not used
   RESTART = 13
};

/*---------  DLL Function Codes ----------*/
enum DLL_Function_Codes
{
   RESET_REMOTE_LINK,
   RESET_USER_LINK,
   TEST_FUNCTION,
   USER_DATA,
   UNCONFIRMED_DATA,
   REQUEST_LINK_STATUS = 9
};

/*---------  APP Layer Function Codes ----*/
enum APP_Function_Codes
{
   CONFIRM,
   READ,
   WRITE,
   SELECT,						// IK20241226 not used
   OPERATE,						// IK20241226 not used
   DIRECT_OPERATE,				// IK20241226 not used
   DIRECT_OPERATE_NO_ACK,		// IK20241226 not used
   COLD_RESTART = 0x0D,
   WARM_RESTART = 0x0E,			// IK20241226 not used
   DELAY_MEASUREMENT = 0x17		// IK20241226 not used
};

/*---------  IIEN2 ERROR INDICATIONS -----*/
enum IIEN2_Errors
{
   GOOD_APDU,
   BAD_FUNCTION,
   OBJECT_UNKNOWN,
   PARAMETER_ERROR = 4,
   BUFFER_OVERFLOW = 8,			// IK20241226 not used
   ALREADY_EXEC = 16,			// IK20241226 not used
   BAD_CONFIG = 32				// IK20241226 not used
};

/*---------- DNP Replies ---------------------*/
#define DNP_FRAME_SIZE   16 //The data payload in the link frame contains a pair of CRC octets for every 16 data octets
enum DNP_Replies
{
   SEND_NOTHING,
   DLL_ACK_CONFIRM = 1,
   LINK_STATUS = 2,
   ERROR_RESPONSE = 4,
   OBJECT_30_RESPONSE = 8,
   CLASS_0 = 16,
   CLASS_1 = 32,
   ALL_DATA = 64,
   BATTERY_VOLTAGE = 128,
   TIME_DELAY_RESPONSE = 512,
   OBJECT_1_RESPONSE = 1024
};

/*--------  Modbus Function Codes --------------*/
enum Modbus_Function_Codes
{
	NO_FUNCTION, //Remove?******************
	READ_COIL_STATUS,
	READ_INPUT_STATUS,
	READ_HOLDING_REGISTERS,
	READ_INPUT_REGISTERS,
	FORCE_SINGLE_COIL,
	PRESET_SINGLE_REGISTER,
	READ_EXCEPTION_STATUS,
	DIAGNOSTICS,
	PROGRAM_484,
	POLL_484,
	FETCH_COMM_EVENT_COUNTER,
	FETCH_COMM_EVENT_LOG,
	PROGRAM_CONTROLLER,
	POLL_CONTROLLER,
	FORCE_MULTIPLE_COILS,
	PRESET_MULTIPLE_REGISTERS
};

/*----------------- Modbus Reply MSGS --------*/
enum Modbus_Replies
{
   MODBUS_SEND_NOTHING,
   SEND_PRESET_REGISTERS_RESPONSE,
   SEND_HISTORY,
   SEND_CMD_ECHO,
   SEND_DATA,
   SEND_ALARMS,
   SEND_DATE_TIME,
   NOT_SUPPORTED_MODBUS,
   ILLEGAL_DATA,
   ILLEGAL_ADDRESS,
   SLAVE_DEVICE_BUSY,
   SLAVE_DEVICE_FAILURE,
   SEND_DEBUG
};

/*--------- Setup Replies ----------------*/
enum Setup_Replies
{
   //SEND_NOTHING,
   SEND_NUM_OF_POINTS = 1,	// IK20241226 should be depricated - analog points are not used
   SEND_PROTOCOL = 2,
   SEND_DLL_RETRIES = 3,
   SEND_INTER_CHAR = 4,
   SEND_HOST_ADDR = 5,
   SEND_DLL_CONFIRM = 6,
   SEND_METER_ADDR = 7,
   SEND_DLL_TIMEOUT = 8,
   SEND_APP_CONFIRM_STATUS = 9,
   SEND_XMT_DELAY = 10,
   SEND_BAUD_RATE = 11,
#ifdef UNI_BI_POLAR_INPUTS //UNIPOLAR_INPUTS were depricated on 2018-May-24
   SEND_INPUT1 = 12, //-!- IK20250714 should be removed, always BIPOLAR,
   SEND_INPUT2 = 13, // meanwhile, up to SEND_INPUT5 must be consequitive
   SEND_INPUT3 = 14,
   SEND_INPUT4 = 15,
   SEND_INPUT5 = 16
#endif //UNI_BI_POLAR_INPUTS, UNIPOLAR_INPUTS were depricated on 2018-May-24
};

/*-----------  Confirmations -------------------*/
enum Confirmations // for DNP3
{
   NOTHING_PENDING,
   CONFIRM_PENDING,
//   SOMETHING		// IK20241226 not used
};

/*-----------  UART Events ----------------------*/
enum UART_Events
{
   NO_ACTIVITY,
   RCVANDXMT,
   RCV
};

/*---------  Alarm Events  ----------------------*/
#define MAX_EVENT_NUMBER        10
#define DNP_STATE_ON_PT_ONLINE        0x81  // DNP state on and point is online
#define DNP_STATE_OFF_PT_ONLINE       0x01  // DNP state went off and is online

enum Alarm_Event_IDs {	// in Structure_defs.h: #define NUM_ALARMS  9	// Current number of alarms defined
	// IK20250805 DNP bug Fix: event indexes coded as numbers (not powers of 2), the same as for Class 0 polling
	HIGH_BAT_EVENT_INDEX = 0,		// High Battery Voltage Event
	LOW_BAT_EVENT_INDEX = 1,		// Low Battery Voltage Event
	PLUS_GF_EVENT_INDEX = 2,		// Plus GF Event
	MINUS_GF_EVENT_INDEX = 3,		// Minus GF Event
	RIPPLE_VOLT_EVENT_INDEX = 4,	// Ripple voltage Event
	RIPPLE_CURR_EVENT_INDEX = 5,	// Ripple current event
	AC_LOSS_EVENT_INDEX = 6,		// AC Fail event
	HI_Z_EVENT_INDEX = 7,			// Hi impedance fault event
	EBAT_EVENT_INDEX =8,			// Emergency Low Battery event
	NO_EVENT = 0xFF
};

// The TWSR only contains relevant status information when the TWI Interrupt Flag is asserted.
// At all other times, the TWSR contains a special status code indicating that no relevant status information is available.

//#define TWSR_STATUS_NOT_AVAIL 0xF8

/*-------------------------- literals ------------------------------*/
//IK20230601 moved 'literals' from "main.c"

// IK20250116 added definitions for DNP3 commands from ASE2000 monx file ( IK20250116 verified )
#define CmdCalibrateVrange          39  // index = 39 (send this command with param = Calibrate_xxxV)
#define CmdCalibrateFaultV          40  // index = 40
#define CmdCalibrateMinusGround     41  // index = 41
#define CmdCalibrateRipleCurrent    42  // index = 42 (send this command with param = CalibrateRipCurr_xxx_mA)
#define CmdCalibrateRippleVolt      43  // index = 43 (send this command with param = CalibrateRipVolt_xxxmV
#define CmdSetUnitVoltage           44  // index = 44 (send this command with param = Set_xxxV_unit
#define CmdSetPwmControl            45  // index = 45 (send this command to start PWM calibration of current loop output)
#define CmdSetPwmValue              46  // index = 46 (send this command to set PWM value of current loop output with param = Set_PWM_Value)
#define CmdCalibrate1or3Phase       47  // index = 47 (send this command with param = 127 for single phase, 255 for 3 phase, 0 to stop)

//IK20250205 new DNP commands to control communication protocol
#define CmdSetAsciiCmds             48  // Point 48 (0x30) - Exit DNP protocol, start ASCII_CMDS
#define CmdSetSetup                 49  // Point 49 (0x31) - Exit DNP protocol, start SETUP
#define CmdSetModbus                50  // Point 50 (0x32) - Exit DNP protocol, start MODBUS

#define CalibrateSinglePhase        127 // use with CmdCalibrate1or3Phase = 47
#define Calibrate_ThreePhase        255 // index = 47
#define CalibrateStopPhaseCal       0   // index = 47

/*-------------------------- prototypes ----------------------------*/


//void ISR_TxOutputHandler(void);  // called from interrupt periodically with 1 ms intervals
int Is_Numeric(char* strp);
void Send_Response_to_UART(uint8);
uint8  EEPROM_Read_byte(uint16 EEaddress); // IK20231214
void EEPROM_Write_byte(uint16 EEaddress, uint8 value); // IK20231214

#if(0) // old code, not used
float  EEPROM_Read_float(uint16 EEaddress); // IK20231215
uint16 EEPROM_Read_two_bytes(uint16 EEaddress); // IK20231214
void EEPROM_Write_float(uint16 EEaddress, float value); // IK20231215
void EEPROM_Write_two_bytes(uint16 EEaddress, uint16 value); // IK20231214
#endif //#if(0) // old code, not used

void Delay_ms(uint16 Delay);
Uchar ParseRCI(void);
void DoNothing();
void EEPROM_Save(void* StartAdr, uint16 BitSize); // IK20250804 save data from SysData structure to EEPROM_SysData
#define SaveToEE(VAR)     EEPROM_Save(&(VAR), sizeof(VAR))
void EEPROM_Load_to_SysData(void* StartAdr, uint16 BitSize); // IK20250804 load data from EEPROM_SysData to SysData structure
#define LoadFromEE(VAR)     EEPROM_Load_to_SysData(&(VAR), sizeof(VAR))
int CheckVariableRangeAndChange(uint8 UnitTypeIndex, uint8 CriteriaIndex, float* value, float DELTA_value, signed char IncEq_pls1_DecEq_mns1);

void CalibrationSteps(uint16 RawADCcounts);
void SendCrLf(void);

void ClearConsole(void);
void IgnoreComment(void); //used for test
void Print_Help(void);
void CheckExecuteFrontPanelCmd(void);
void Check_Alarms(void);

void Show_ADCcounts_and_Volts(void);	// "adch"  Report ADC readings in format: chan0, chan1, chan2, chan3, chan4, chan5
void Excitation_Pulse(void);		// Pulse set/get: 'pulse>on', 'pulse>off' - set right now; 'pulse>enab', IT DOES NOT SAVED into flash because it is test command.
void SetGetAlarm(void);				// Alarm set/get: 'alarm rv>enab', 'alarm rv>disa'; 'alarm ri>enab', 'alarm ri>disa'; 'alarm ac>enab', 'alarm ac>disa'; 'alarm hz>enab', 'alarm hz>disa'
void Get_Button_Press(void);
void SetLED(int LEDindex, int ColorIndex);
void Operation(void);
void ProcessUPbutton();
void ProcessDOWNbutton();
void Write_Numeric_Display(char* num_str);	//IK20250102 write up to 10 chars to upper LED (up to 5 chars and and up to 5 dots)
void Write_ASCII_Display(char* str);
void Write_ASCII_Display_FlashString(char FL* str_f_ptr);
void DisplayPrepare(void);
uint8 IsNumeric(const char* str);
uint16 PrintConstString(const char FL* str_f_ptr);
int SPRINT_F( char *OutStr, char FL * fmt, ...);
uint16 PutStr(const char* Str);
uint16 CopyConstString(const char FL* str_f_ptr, char* dest);
void Set_USART_UBBRregister(Uint32 BaudRate); // IK20250206
uint16 Calculate_USART_UBRRregister(Uint32 BaudRate) ;
void Set_and_Save_New_BaudRate(uint8 BR_index);
void Init_UART(void);
void SetASCIIandSignMsg(void);
/* without redefining sprintf IAR gives warning
	Warning[w6]: Type conflict for external/entry "SPRINT_F", in module "Front_menu" against external/entry in module "main" ; prototyped function vs K&R function   // In module Front_menu: //
	  // K&R Function, args 0, attr 0 //
	  int (__version_4 SPRINT_F)();
	  // In module main: //
	  // Function, args 3, attr 0 //
	  int (__version_4 SPRINT_F)(char *, char __flash *, ...);
*/

/*-------------------------- interrupts ----------------------------
//most of these interrupts are not used but they were defined so that
//a reti instruction was included.
#pragma vector = INT0_vect
INTERRUPT void INT0_interrupt (void)
;
#pragma vector = INT1_vect
INTERRUPT void INT1_interrupt(void)
;
#pragma vector = INT2_vect
INTERRUPT void INT2_interrupt(void)
;
#pragma vector = TIMER1_COMPA_vect
INTERRUPT void TIMER1_COMPA_interrupt(void)
;
#pragma vector = TIMER1_COMPB_vect
INTERRUPT void TIMER1_COMPB_interrupt(void)
;
#pragma vector = TIMER1_CAPT_vect
INTERRUPT void TIMER1_CAPT_interrupt(void)
;
#pragma vector = TIMER0_COMPA_vect
INTERRUPT void TIMER0_COMPA_interrupt(void)
;
#pragma vector = TIMER0_COMPB_vect
INTERRUPT void TIMER0_COMPB_interrupt(void)
;
#pragma vector = TIMER2_COMPB_vect
INTERRUPT void TIMER2_COMPB_interrupt(void)
;
#pragma vector = TIMER2_OVF_vect
INTERRUPT void TIMER2_OVF_interrupt (void)
;
#pragma vector = TIMER0_OVF_vect
INTERRUPT void TIMER0_OVF_interrupt (void)
;
#pragma vector = TIMER1_OVF_vect
INTERRUPT void TIMER1_OVF1_interrupt(void)
;
#pragma vector = SPI_STC_vect
INTERRUPT void SPI_STC_interrupt (void)
;
#pragma vector = USART0_UDRE_vect
INTERRUPT void USART0_UDRE_interrupt (void)
;
#pragma vector = USART0_TX_vect
INTERRUPT void USART0_TX_interrupt  (void)
;
#pragma vector = ADC_vect
INTERRUPT void ADC_interrupt (void)
;
#pragma vector = EE_READY_vect
INTERRUPT void EE_READY_interrupt (void)
;
#pragma vector = ANALOG_COMP_vect
INTERRUPT void ANALOG_COMP_interrupt (void)
;
#pragma vector = SPM_READY_vect
INTERRUPT void SPM_READY_interrupt (void)
;
#pragma vector = PCINT0_vect
INTERRUPT void PCINT0_interrupt (void)
;
#pragma vector = PCINT1_vect
INTERRUPT void PCINT1_interrupt (void)
;
#pragma vector = PCINT2_vect
INTERRUPT void PCINT2_interrupt (void)
;
#pragma vector = PCINT3_vect
INTERRUPT void PCINT3_interrupt (void)
;
#pragma vector = WDT_vect
INTERRUPT void WDT_interrupt (void)
;
#pragma vector=TWI_vect
INTERRUPT void TWI_interrupt(void)
;
*/
#ifdef PC
#define NUM_INPUT_FIELDS   4 // on windows simulator form, text boxes for Vbat, vGnd, Vrip, Irip
extern double UserValue[NUM_INPUT_FIELDS];
enum SimInputIndex {
	Vbat = 0,
	Vgnd = 1,
//	V_gf = 2,
	Vrip,// = 3,
	Irip,// = 4
};
  extern int dummy_function();

  #ifndef _WDR
  #define _WDR _watchdog_reset
  #endif //_WDR

  #ifndef _watchdog_reset
  #define _watchdog_reset dummy_function
  #endif //_watchdog_reset

  #ifndef _NOP
  #define _NOP dummy_function
  #endif //_NOP

  #ifndef __disable_interrupt
  #define __disable_interrupt dummy_function
  #endif //__disable_interrupt

  #ifndef __enable_interrupt
  #define __enable_interrupt dummy_function
  #endif //__enable_interrupt
  #endif //#ifdef PC
#endif //GLOBAL
