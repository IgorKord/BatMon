#define CMD_LEN   4 // 4-char commands

#define Bit_0                    1 //0x0001
#define Bit_1                    2 //0x0002
#define Bit_2                    4 //0x0004
#define Bit_3                    8 //0x0008
#define Bit_4                   16 //0x0010
#define Bit_5                   32 //0x0020
#define Bit_6                   64 //0x0040
#define Bit_7                  128 //0x0080

#define Bit_8                  256 //0x0100
#define Bit_9                  512
#define Bit_10                1024
#define Bit_11                2048
#define Bit_12                4096
#define Bit_13                8192
#define Bit_14               16384
#define Bit_15               32768

#define Bit_16               65536
#define Bit_17              131072
#define Bit_18              262144
#define Bit_19              524288
#define Bit_20             1048576
#define Bit_21             2097152
#define Bit_22             4194304
#define Bit_23             8388608

#define Bit_24            16777216
#define Bit_25            33554432
#define Bit_26            67108864
#define Bit_27           134217728
#define Bit_28           268435456
#define Bit_29           536870912
#define Bit_30          1073741824
#define Bit_31          2147483648


//#define HOST_RX_BUFF_LEN				32		// Host Rx & LCD buff length
//#define HOST_TX_BUFF_LEN			128		// Host Tx buff length should be capable to output one terminal line 80 chars
#define HOST_RX_BUFF_LEN    256 // IK20241224 was 292	// Host Rx buff length
#define HOST_XMT_BUFF_LEN   256 // IK20241224 was 250	// Host Tx buff length should be capable to output one terminal line 80 chars


// EEPROM            byte address   // var type size
#define EE_SYS_DATA_OFFSET		256	// 0x100
#define adr_cal1_low_meas		60	// float    4    // Battery Voltage calibration
#define adr_cal3_calptlow		64	// float    4
#define adr_cal2_high_meas		68	// float    4
#define adr_cal4_calpthigh		72	// float    4

#define adr_cal5_low_meas		76	// float    4    // Fault voltage correction factor info
#define adr_cal7_calptlow		80	// float    4
#define adr_cal6_high_meas		84	// float    4
#define adr_cal8_calpthigh		88	// float    4

#define adr_cal9_low_meas		92	// float    4     // Minus Grnd voltage correction factor info
#define adr_cal11_calptlow		96 	// float    4
#define adr_cal10_high_meas		100	// float    4
#define adr_cal12_calpthigh		104	// float    4

#define D_adr_high_bat_thresholdHi			16	// uint16   2
#define D_adr_high_bat_thresholdLo			17	//-!- address is not adjacent!

#define D_adr_low_bat_thresholdHi			18	// uint16   2
#define D_adr_low_bat_thresholdLo			19

#define D_adr_plus_GF_thresholdHi			20	// uint16   2
#define D_adr_plus_GF_thresholdLo			21

#define D_adr_minusGFthresholdHi 			22	// uint16   2
#define D_adr_minusGFthresholdLo 			23

#define D_adr_ripple_voltage_threshold_Hi	24	// uint16   2
#define D_adr_ripple_voltage_threshold_Lo	25

#define D_adr_ripple_current_threshold_Hi	26	// uint16   2
#define D_adr_ripple_current_threshold_Lo	27

#define D_adr_time_delay_Hi					28	// uint16   2
#define D_adr_time_delay_Lo					29

#define D_adr_phase							32	// uint8   1
#define D_adr_pulse							33	// uint8   1
#define D_adr_buzzer						34	// uint8   1
#define D_adr_latch_state					35	// uint8   1
#define D_adr_disabled_alarms				36	// uint8   1

#define adr_calrv1_low_meas		108	// float    4     // single phase ripple voltage calibration
#define adr_calrv3_calptlow		112	// float    4
#define adr_calrv2_high_meas	116	// float    4
#define adr_calrv4_calpthigh	120	// float    4

#define adr_calri1_low_meas		124	// float    4     // single phase ripple current calibration
#define adr_calri3_calptlow		128	// float    4
#define adr_calri2_high_meas	132	// float    4
#define adr_calri4_calpthigh	136	// float    4

#define adr_v_cal_f				140	// float    4     // battery offset
#define adr_V4					144	// uint8    1     // 4mA voltage point (V4)
#define adr_V20L				145	// uint16   1     //-!- FIX! 20 ma voltage point (V20) Low byte
#define adr_cal14_dc4_cal		146	// float    4     // duty cycle for low mA voltage point (DC4)

#define adr_cal15_dc20_cal		150	// float    4     // duty cycle for high mA voltage point (DC20)
#define adr_METER_ADDR			154	// uint16   2     // battery monitor address
#define adr_HOST_ADDR			156	// uint16   2     // Modbus first register
#define adr_DLL_TIMEOUT			158	// uint16   2     // dll timeout
#define adr_XMT_DELAY			160	// uint16   2     // xmt delay
#define adr_INTER_CHAR			162	// uint16   2     // inter-char gap
#define D_adr_unit_type			163	// uint8   1 <<< address is the same for Comm board and for Display board
#define adr_PROTOCOL			164	// uint8    1     // protocol byte
#define adr_NUM_OF_POINTS		165	// uint8    1     // num of channels
#define adr_CHANNEL_ONE			166	// uint8    1     // channel 1 EEPROM address, channel one type i.e. bipolar or unipolar
#define adr_CHANNEL_TWO			167	// uint8    1     // channel 2 EEPROM address, channel one type i.e. bipolar or unipolar
#define adr_CHANNEL_THREE		168	// uint8    1     // channel 3 EEPROM address, channel one type i.e. bipolar or unipolar
#define adr_CHANNEL_FOUR		169	// uint8    1     // channel 4 EEPROM address, channel one type i.e. bipolar or unipolar
#define adr_CHANNEL_FIVE		170	// uint8    1     // channel 5 EEPROM address, channel one type i.e. bipolar or unipolar
#define adr_DLL_CONFIRM			171	// uint8    1     // dll confirm status
#define adr_BAUD_RATE			172	// uint16   2     // baud rate
#define adr_APP_STATUS			174	// uint8    1     // Modbus UART_parity
#define adr_DLL_NUM_OF_RETRIES	175	// uint8    1     // dll retries
#define adr_MONITOR_V_RANGE		176	// uint8    1     // BM Voltage Range (hardware - defined): if ((SysData.NV_UI.unit_type != 24) && (SysData.NV_UI.unit_type != 48) && (SysData.NV_UI.unit_type != 125) && (SysData.NV_UI.unit_type != 250)) SysData.NV_UI.unit_type = 125;
#define adr_V20H				177	//-!- FIX! 20 ma voltage point (V20) High byte
#define adr_true_1mA_false_20mA	191 // uint8    1     // NOT USED

#define adr_calrv5_low_meas		208	// float    4     three phase ripple voltage calibration
#define adr_calrv7_calptlow		212	// float    4
#define adr_calrv6_high_meas	216	// float    4
#define adr_calrv8_calpthigh	220	// float    4
#define adr_calri5_low_meas		224	// float    4     3 phase ripple current calibration
#define adr_calri7_calptlow		228	// float    4
#define adr_calri6_high_meas	232	// float    4
#define adr_calri8_calpthigh	236	// float    4

#define MAX_ADC_CHANNELS		8

/* IK20251030: IAR does not like structure name before {} - example below, Visual Studio C allows both
typedef struct Calibr2points         // this structure must be initialized, DO NOT CHANGE ORDER due to IAR bug, see LinInterpolation()
{
	float X1_lowADCcounts;     // lower 'Y1' coordinate, scaled ADC counts
	float Y1_lowCalibrVal;     // current or voltage - lower 'X1' coordinate, in 10th of mV or singles mA
	float X2_highADCcounts;    // higher 'Y2' coordinate, scaled ADC counts
	float Y2_highCalibrVal;    // current or voltage - higher 'X2' coordinate, in 10th of mV or singles mA
};

AIR accepts the below definition
*/
typedef struct          // this structure must be initialized, DO NOT CHANGE ORDER due to IAR bug, see LinInterpolation()
{
	float X1_lowADCcounts;		// lower 'X1' coordinate, ADC counts
	float Y1_lowCalibrVal;		// current or voltage - lower 'Y1' coordinate, in Volts or mA
	float X2_highADCcounts;		// higher 'X2' coordinate, ADC counts
	float Y2_highCalibrVal;		// current or voltage - higher 'Y2' coordinate, in Volts or mA
}Calibr2points;

#define X1_low_point			0	//index 1 of float[] array in union Calibration
#define Y1_low_point			1	//index 0 of float[] array in union Calibration
#define X2_high_point			2	//index 3 of float[] array in union Calibration
#define Y2_high_point			3	//index 2 of float[] array in union Calibration

typedef union          // this union is an alias of a structure, allows to acsess Calibr2points members as if they are in array of floats.
{
	float Coord[4];
	Calibr2points Cal;
} Calibration, *CalPtr;

typedef struct                      // this structure must be initialized, DO NOT CHANGE ORDER due to IAR bug, see LinInterpolation()
{
	float high_bat_threshold_V_f;	// 0x000 IK20251029 saved as float, menu changes value by 0.1 V increments/decrements
	float low_bat_threshold_V_f;	// 0x004 IK20251029 saved as float, menu changes value by 0.1 V increments/decrements
	float minus_gf_threshold_V_f;	// 0x008 IK20251029 saved as float, menu changes value by 0.1 V increments/decrements
	float plus_gf_threshold_V_f;	// 0x00C IK20251029 saved as float, menu changes value by 0.1 V increments/decrements
	float ripple_V_threshold_mV_f;	// 0x010 ripple voltage in mVolts, , menu changes value by 1 mV increments/decrements
	float ripple_I_threshold_mA_f;	// 0x014 ripple current in mAmperes, menu changes value by 1 mA increments/decrements
	float alarm_delay_sec_f;		// 0x018 grace period delay from event is triggered until alarm is set, menu changes value by 1 sec increments/decrements
	float baud_rate;				// 0x01C baud rate, default is Baud_19200. //IK20250826 included 115200, thus, need Uint32, menu changes value based on a list of possible baud rates
	float meter_address;			// 0x020 battery monitor address, menu changes value by 1 increment/decrement. If hold longer, changes by 10, even longer - by 100
	uint8  StartUpProtocol;			// 0x024 only valid values:    SETUP = 0x00,	DNP3 = 0x01,	MODBUS = 0x02,	ASCII_CMDS = 0x03,	ASCII_MENU = 0x04
	uint8  BRate_index;				// IK202601714 index of baud rate in Baud_Rates[] array, used to set SysData.NV_UI.baud_rate and Existing.baud_rate
	//uint8  UART_parity;				// 0x025 only valid values 0 = NONE; 1 = EVEN; 2 = ODD
	uint8  unit_type;				// 0x026 only valid values 24, 48, 125, 250
	uint8  unit_index;				// 0x027 corresponds to unit_type, range (0...3), used to access array Alarm_Limits[]
	uint16 host_address;			// 0x028 host address, not changed during operation
	uint16 disabled_alarms;			// 0x02A the bit == 1 disables a partcular alarm, Two bytes to include Last Gasp bit 8
	uint16 V4;						// 0x02C  4 mA voltage point (Voltage when current loop produces 4 mA or 0 mA)
	uint16 V20;						// 0x02E 20 ma voltage point (Voltage when current loop produces 20 mA or 1 mA)
	uint16 SavedStatusWord;			// 0x030 non-volatile user-chosen saved states as bits: buzzer on/off, latch on/off, 1 or 3 phase, current output I01 or I420
	//uint8  buzzer;				// REPLACED with bit in SavedStatusWord, indicates whether buzzer is on or off
	//uint8  phase;					// REPLACED with bit in SavedStatusWord, dealing with ripple created by battery charger type. =1 for single phase charger (120 Hz ripple), =3 for three phase charger (360 Hz ripple)
	//uint8  latch_state;			// REPLACED with bit in SavedStatusWord, whether or not latch on some event or when condition clears, restore normal operation
	//uint8  true_1mA_false_20mA;	// REPLACED with bit in SavedStatusWord, type of current output: 1 mA or 20 mA, used in CurrentOut_I420 calibration
}SettingsStruct;

// using pointer to Calibr2points to access members directly by name: float param = BatteryVolts.Y1_lowCalibrVal; param = BatteryVolts.Y2_highCalibrVal
// using pointer to Calibration to access members as array: float param = ((CalPtr)BatteryVolts)->Coord[1]; param = ((CalPtr)BatteryVolts->Coord[3];

typedef struct 	//size of SysData should be less, than 2048 - size of EEPROM block
{
									// adr    offset size
	uint16 Data_Valid;				// 0x000  2  // should not be FF
#define DATA_VALID_OFFSET       0
	uint16 FWversion;				// 0x002  2  // FW version, 0030
#define FW_VERSION_OFFSET       2
	uint16 dll_timeout;				// 0x004  2  //-!- IK20250203 only set/get not used in code !!  dll timeout
	uint16 inter_char_gap;			// 0x006  2  //-!- IK20250203 only set/get not used in code !!  inter-char gap, default = 1041 microseconds
	uint16 xmt_delay;				// 0x008  2  // xmt delay
	uint8 input_type[6];			// 0x00A  6  // type of analog input - uni-polar or bi-polar  index [0] is former variable 'analog_points'

	uint8 dll_confirm;				// 0x010  1  //-!- IK20250203 only set/get not used in code !!  dll confirm status
	uint8 app_confirm;				// 0x011  1  // Modbus UART_parity
	uint8 dll_retries;				// 0x012  1  //-!- IK20250203 only set/get not used in code !!  dll retries
	uint8 extra1_bytes[13];			// 0x015  11 // future use

	// LCD board settings start at 0x020
	uint8 FrontBoardBytes[16];		// 0x020  16  // future use

	// Calibration floats start at 0x030
	Calibr2points BatteryVolts;		// 0x030  16 // Y1 X1 Y2 X2 Battery Voltage calibration
	Calibr2points FaultVolts;		// 0x040  16 // Y1 X1 Y2 X2 Fault Voltage calibration
	Calibr2points MinusGndVolts;	// 0x050  16 // Y1 X1 Y2 X2 Minus Grnd voltage correction factor info
	Calibr2points RippleVolts1ph;	// 0x060  16 // Y1 X1 Y2 X2 single phase ripple voltage calibration
	Calibr2points RippleVolts3ph;	// 0x070  16 // Y1 X1 Y2 X2 three phase ripple voltage calibration
	Calibr2points RippleCurr1ph;	// 0x080  16 // Y1 X1 Y2 X2 single phase ripple current calibration
	Calibr2points RippleCurr3ph;	// 0x090  16 // Y1 X1 Y2 X2 three phase ripple current calibration
	Calibr2points CurrentOut_I420;	// 0x0A0  16 // NU X1 NU X2 current loop calibration, value in PWM register to get 4 mA or 20 mA
	SettingsStruct   NV_UI;			// 0x0B0 // Non Volatile User Interface settings controlled by user
	float Bat_Cal_Offset_Volts_f;	// 0x0C0  4 IK20251029 offset rarely changes, only by user from front panel CALibration menu. Typically, operator with sertified calibrated meter measures battery voltage and adjusts offset to match displayed voltage to his meter.
#define EXTRA_FLOATS_NUM			4
	float extra_floats[EXTRA_FLOATS_NUM];	// 0x0C4  4*EXTRA_FLOATS_NUM  // future use
	//--- if EXTRA_FLOATS_NUM = 7 next address is  0x0E0
}SYS_SPECIFIC_DATA;
extern SYS_SPECIFIC_DATA SysData; //IK20250620 defined in main.c

#ifdef PC
extern  SYS_SPECIFIC_DATA DefaultSysdata;
extern SYS_SPECIFIC_DATA EEPROM_SysData;
#else //ATMEL
extern __no_init SYS_SPECIFIC_DATA DefaultSysdata;
extern __eeprom  SYS_SPECIFIC_DATA EEPROM_SysData;// located in EEPROM @ EE_SYS_DATA_OFFSET, @ 0x100
#endif// PC


#define NUM_ALARMS   9  // Current number of alarms defined

//********** rt.OperStatusWord definitions
#define Alarm_BatVoltageHIGH_BitNum     0	// Battery Voltage High, above Vmax setting
#define Alarm_BatVoltageLOW_BitNum      1	// Battery Voltage LOW, below Vmin setting
#define Alarm_PlusGND_FAULT_BitNum      2	// Ground Voltage High, above V_GND_Plus setting
#define Alarm_MinusGND_FAULT_BitNum     3	// Ground Voltage LOW, below V_GND_Minus setting
#define Alarm_Ripple_Voltage_BitNum     4	// Ripple voltage outside limit setting; ALSO SAVED IN SysData.NV_UI.SavedStatusWord
#define Alarm_Ripple_Current_BitNum     5	// Ripple current BELOW limit setting
#define Alarm_AC_Loss_BitNum            6	// NO AC voltage detected - no battery charge is possible
#define Alarm_High_Impedance_BitNum     7	// Seems to be no charging current - battery is not charging
#define Alarm_BatVoltCRITICAL_BitNum    8	// LAST GASP, Battery Voltage CRITICALLY LOW, below Vcrit setting

#define Alarm_BatVoltageHIGH_Bit        (1<<Alarm_BatVoltageHIGH_BitNum )	// 0x0001 Battery Voltage High, above Vmax setting
#define Alarm_BatVoltageLOW_Bit         (1<<Alarm_BatVoltageLOW_BitNum  )	// 0x0002 Battery Voltage LOW, below Vmin setting
#define Alarm_PlusGND_FAULT_Bit         (1<<Alarm_PlusGND_FAULT_BitNum  )	// 0x0004 Ground Voltage High, above V_GND_Plus setting
#define Alarm_MinusGND_FAULT_Bit        (1<<Alarm_MinusGND_FAULT_BitNum )	// 0x0008 Ground Voltage LOW, below V_GND_Minus setting
#define Alarm_Ripple_Voltage_Bit        (1<<Alarm_Ripple_Voltage_BitNum )	// 0x0010 Ripple voltage outside limit setting; ALSO SAVED IN SysData.NV_UI.SavedStatusWord
#define Alarm_Ripple_Current_Bit        (1<<Alarm_Ripple_Current_BitNum )	// 0x0020 Ripple current BELOW limit setting
#define Alarm_AC_Loss_Bit               (1<<Alarm_AC_Loss_BitNum        )	// 0x0040 NO AC voltage detected - no battery charge is possible
#define Alarm_High_Impedance_Bit        (1<<Alarm_High_Impedance_BitNum )	// 0x0080 Seems to be no charging current - battery is not charging
#define Alarm_BatVoltCRITICAL_Bit       (1<<Alarm_BatVoltCRITICAL_BitNum)	// 0x0100 LAST GASP, Battery Voltage CRITICALLY LOW, below Vcrit setting
#define Buzzer_ON_eq1_Bit               Bit_9	// 0x0200 Bit enables / disables buzzer immediately, use bit in SysData.NV_UI.SavedStatusWord to be saved into flash.
#define Latch_ON_eq1_Bit                Bit_10	// 0x0400 Bit enables / disables alarm latching immediately, use bit in SysData.NV_UI.SavedStatusWord to be saved into flash.
#define SinglePhase_eq0_3ph_eq1_Bit     Bit_11	// 0x0800 Bit in SysData.NV_UI.SavedStatusWord to be saved into flash, selects 1 phase or 3-phase.
#define CurOut_I420_eq0_I01_eq1_Bit     Bit_12	// 0x1000 Bit in SysData.NV_UI.SavedStatusWord to be saved into flash, selects 1 phase or 3-phase.

#define AlarmStatus_Instant_BITS        (Alarm_Ripple_Voltage_Bit | Alarm_Ripple_Current_Bit | Alarm_AC_Loss_Bit | Alarm_High_Impedance_Bit)
#define AlarmStatus_ALL_BITS            (Alarm_BatVoltageHIGH_Bit | Alarm_BatVoltageLOW_Bit | Alarm_PlusGND_FAULT_Bit | Alarm_MinusGND_FAULT_Bit | AlarmStatus_Instant_BITS)

//#define Test_Excitation_Bit           Bit_13	// =1 inject white noise or sine wave, =0 - not
//#define In_DNP_mode_eq1_Bit           Bit_xx	// =1 test in progress (inject pulses), 0=normal work
//#define In_ModBus_mode_eq1_Bit        Bit_xx	// protocol used
//#define In_Menu_eq1_Bit               Bit_xx	// to be able to terminate menu from LCD / buttons
//#define Exit_Menu_eq1_Bit             Bit_xx	// to be able to terminate menu from LCD / buttons

#define RealTimeDataReady_eq1_Bit       Bit_13	// 0x2000  Set this bit in ASCII_CMDS mode in timer interrupt, so serial output happens in main loop periodically, sending into to serial port
#define SendRealTimeData_eq1_Bit        Bit_14	// 0x4000  Set this bit in ASCII_CMDS mode to periodically send into to serial port
//#define Command_Executing_eq1_Bit       Bit_14	// 0x4000  Set this bit if command needs time to run and not finishing immediately
#define EE_DATA_0_NO_DATA_1_Bit         Bit_15	// 0x8000 in rt.OperStatusWord. If EEPROM has no valid data after upgrade, status LED will blink yellow


#define   CharAvailableFlag             Bit_0	// maybe make it as a whole lower byte, use add instead of set, process chars subtracting it until it is zero
//#define   suppress_PC_continuous_update  Bit_1	// temporary suppress PC output when showing real-time diagnostic on PC, enables alarm messages
#define   CharEchoFlag                  Bit_2	// host echos =1, no echoes =0
//#define   CmdPendingFlag              Bit_3	// not used
#define   CmdAvailFlag                  Bit_4	// initiates parsing of a command on '\r' detection
#define   Waiting_User_Input            Bit_5	// temporary suppress PC output when showing real-time diagnostic on PC and entering some value in variable,
//#define   In_menu_RCI_cmd             Bit_6	// tells Parse_RCI to check excluded from "in-menu" command, like call "`oltf" or "`menu" from menu.
#define   CmdVerboseResponse            Bit_7	// "echo>verbose" sets diagnostics verbosity.  Affects return messages: returns will show command comments, like scaling factors names, etc.

//==============================================================

typedef struct          // this structure must be initialized
{
	float zero_f;				// ADC channel #0 is not used
	float battery_voltage_f;	// ADC channel #1 converted to Volts, 145V saved as 145.00
	float G_fault_voltage_f;	// ADC channel #2 converted to Volts
	float minus_gnd_volts_f;	// ADC channel #3 converted to Volts
	float plus_gnd_volts_f;		// ADC channel #4 is not used, +GndV is calculated
	float ripple_mV_f;			// ADC channel #5 converted to mVolts IK20251119: 216 mV = 148 counts
	float ripple_mA_f;			// ADC channel #5 converted to mAmpers
	float zero_f2;				// ADC channel #7 is not used
}Measurements_f;

typedef union          // this union is an alias of a structure, allows to acsess Calibr2points members as if they are in array of floats.
{
	float measurement_f[8];
	Measurements_f measured;
} MeasuredData;

typedef struct  { // RealTimeVars
	volatile uint16 OperStatusWord;				// each bit calls different diagnostic or test, bit definitions in main.h
	// IK20251029 changed several variables to float for better resolution and easier menu operation
	MeasuredData OutData;	//IK20251118 encapsulate all float measurement results into union to easy show n terminal
	/*
	float battery_voltage_f;					// in Volts, 145V saved as 145.00
	float G_fault_voltage_f;
	float minus_gnd_volts_f;
	float plus_gnd_volts_f;
	float ripple_mV_f;
	float ripple_mA_f;
*/
	// used in DNP calibration, indicates which current loop setting is active
	uint8 ripple_calibration_phase;				// CalibrateSinglePhase=127 is single phase calibration , 255 is 3-phase calibration, 0 means not in calibration mode
	uint8 cal_4mA;								// calibrating 4 mA
	uint8 cal_20mA;								// calibrating 20 mA
	uint8 i_cal_active;							// used in DNP calibration, indicates if current loop calibration is active
	uint16 UBRR0_setting;						// IK20251217 to keep a mirror of UBRR0

	uint8 operating_protocol;

	uint8  InfoLED_blink_eq1;					//-!- should be a bit in OperStatusWord
	uint8  pulse;								//-!- IK20250204 SHOULD NOT BE SAVED IN FLASH! It creates pulses in battery charging line //#define Pulse_Test_eq1_Bit            Bit_0	//  =1 test in progress (inject pulses), 0=normal work

	//---- Modbus Variables ----
	uint8  registers;							// ModBus: how many bytes are coming: 4*registers
	uint16 first_register;						// if (rt.operating_protocol == MODBUS) first_register = host_address;      //same location
	uint16 device_register;
	uint8  protocol_parity;						// == 0 No parity; == 1 UART_parity even; == 2 UART_parity odd

//    volatile uint8 Transmitting; 	// this flag is set by main level output function like cputs() when buffer gets new string, reset by interrupt when has been transmitted
	volatile uint8 Host;
//	volatile uint8 HostTx_StrLen; 				// the length of the string written tp the buffer, setting by main level output function like cputs(), reset by interrupt when transmitted
//	volatile uint8 HostTxPtr_IN; 				// index of writing to buffer, increasing by main level output function like cputs(), reset by interrupt when transmitted
//	volatile uint8 HostTxPtr_OUT; 				// index of reading buffer and writing to a serial channel, increasing and reset by a timer interrupt when transmitted
	volatile uint16 HostRxBuffPtr;				// pointers == indexes in array should be int, or compiler inserts conversion from/to char
	volatile uint16 EchoRxBuffPtr;				// to send echo
	char   HostTxBuff[HOST_XMT_BUFF_LEN];		// [256] used with UART
	char   HostRxBuff[HOST_RX_BUFF_LEN];		// [256] used with UART
	uint16 ADC_buff[8];							//-!- IK20250602 ! for test ! holds results of ADC conversion
	// alarm event counters, used in Check_Alarms()
	uint8  h_battery_fault_cntr;               // High Battery Voltage detection event counter
	uint8  l_battery_fault_cntr;               // Low Battery Voltage detection event counter
	uint8  p_gf_cntr;                          // Positive Ground Fault detection event counter
	uint8  m_gf_cntr;                          // Negative Ground Fault detection event counter
	uint8  rv_cntr;                            // Ripple Voltage detection event counter
	uint8  ri_cntr;                            // Ripple Current detection event counter
	uint8  high_impedance_cntr;                // High Z (impedance) detection event counter
	uint8  ac_cntr;                            // AC loss detection event counter
} RealTimeVars;//real time variables in a structure
extern RealTimeVars rt;


// GENERAL COMMANDS
typedef struct {
	const char cmd_code[5];	    //Uint32 command coding (4 chars)
	void* f_ptr;			//pointer to a function
} t_rci_commands;
extern const t_rci_commands rci[];


/*---------------- TWI WRITE STRUCTURE ----------------*/
//uint8 DestADR, uint8 type, uint8 msg_low, uint8 msg_high
typedef struct  {
	uint8 DestTWI_ADR;	// address to write
	uint8 CMDtype;		// Battery monitor command type
	Uchar msgLow;		// lower byte of data word
	uint8 msgHigh;		// upper byte of data word
	uint8 Status;	// Bit_0: false (0) when writing, true (1) when finished. Bit_1: = false (0) when writing, true (1) if timeout or error; Bit_2: false (0) when reading, true (1) when finished. Bit_3: = false (0) when reading, true (1) if timeout or error
	uint8 error;	// instead of twi_error
}s_TWI_packet; // 6 bytes

typedef union {
	uint8 buffer[6];	// as array of bytes
	s_TWI_packet s;	// as structure
} u_TWI; // 6 bytes
extern u_TWI twi;		//IK20250604 union of array and structure instead of unaggregated vars

typedef struct 	// Encapsulate into structure to keep timer variables together
{
	volatile uint8  TWI_hangup;
	//uint8  extender; // IK20240328 timer interrupt refactored, not needed anymore  //used to generate a 1ms tick from the system 100 us tick
	volatile uint8  extender;			// IK20241217 added it back due to unstable communication.  used to generate a 1.0 ms tick from the system 100us tick
	volatile uint8  ADC_ms;				// used in the ADC routine
	volatile uint8  TWI_reset;			// allows 6 ms for TWI hold in reset
	volatile uint16 Generic;			// used for general purpose timing.
	volatile uint16 Calibration;		// allows multiplexed ADC chanel fully settle
	volatile uint16 TxRxLED_blink;		// usage: blink green /red during receiving or transmitting data
	volatile uint16 IO_update;
	volatile long   start_up_ms;		// determines how long to be in the start up mode
	//-!- IK20250623 WAS IT A TEST VARIABLE? it does not checked in firmware !?! used in DNP answer TIME_DELAY_RESPONSE. sent out when TIME_DELAY_RESPONSE DNP command is received.
	// always increasing, set to a value by DNP SysData.NV_UI.StartUpProtocol: in DNP command 'COLD_RESTART' it set to 90000 before answering, and set to 1-2-4-16 if baud is 19200-9600-4800-1200
//	volatile uint16 RT_correction;		// IK20250925 not needed if OCR2A is set to 199. Was increasing 0 to 575 and reset. It caused to skip all timers decrement once in 576 ms
	volatile uint16 TWI_lockup;			// detects lockup
	volatile uint16 TWI_request;		// counter to synchronize (read from and update display board) LCD, buttons, blinking of status LED  etc.
	volatile uint16 UART_delay_100us;	// IK20250522 used to insert delays (decrements each 100 us) between received char and transmitted char, too quick switching from receive to transmit could corrupt stop bit
	volatile uint16 CommChHoldOff;		// tmr for hold off
	volatile uint16 comm_activity;
	volatile uint16 ModBus_100us;		// used in ModBus protocol
	volatile uint16 PWM_calibration;
	// button press timers
	volatile uint16 auto_button;
	volatile uint16 limit_button;
	volatile uint16 up_button;
	volatile uint16 down_button;
	volatile uint16 reset_button;
	volatile uint16 FreeRunningCounter; // Grok20251231
	// display mode timers
	volatile uint16 UpDownChange_rate_ms;	// IK20250710 Limits how fast values are changed if user holds UP or DOWN button. 100 ms timer, replace with uint8?
	volatile uint16 AlarmLED_blink;	// controls LED 'Alarm' blinking
	volatile uint16 disp_var_change_ms;		// in auto mode, changes variables with this period
	volatile uint16 RealTimeUpdate;			// in ASCII protocol, periodically send brief info if command enables it
	volatile uint16 InfoLED_blink_ms;		// controls InfoLED indicators blink mode
	volatile Uint32 limit_mode_timeout_ms;	// allows automatic switch back from LIMIT mode after 10 min of user inactivity
	volatile Uint32 time_keep;				// increasing with 100 us interval
	// alarm timers
} TIMERS;
extern TIMERS timer;

/*---------------- FRONT INTERFACE ----------------*/

//*********************************** BUTTONS on Front Panel ***********************************
// buttons_highByte = twi.buffer[BYTE_3]; // upper byte of TWI transmission - instantaneous state, goes to Display_Info.buttons_hits
// buttons_low_Byte = twi.buffer[BYTE_2]; // lower byte of TWI transmission - Display board might transfer detected short press or long press, but it is easier to detect it in Comm board firmware

// bits of Display_Info.buttons_hits. bits 0-4 reflect current state received from Display board. Byte is overwritten each time from TWI buffer. Starts button timer
#define BUTTON_AUTO_INSTANT_PRESS_BIT   Bit_0  // if (buttons & 0x01)      // instantaneous status of Manual/auto button, 0==pressed, without delay, updates each main loop cycle
#define BUTTON_LIMIT_INSTANT_PRESS_BIT  Bit_1  // if (buttons & 0x02)      // instantaneous status of Limit button, 0==pressed, without delay, updates each main loop cycle
#define BUTTON_UP_INSTANT_PRESS_BIT     Bit_2  // if (buttons & 0x04)      // instantaneous status of UP button, 0==pressed, without delay, updates each main loop cycle
#define BUTTON_DOWN_INSTANT_PRESS_BIT   Bit_3  // if (buttons & 0x08)      // instantaneous status of DOWN button, 0==pressed, without delay, updates each main loop cycle
#define BUTTON_RESET_INSTANT_PRESS_BIT  Bit_4  // if (buttons & 0x10)      // instantaneous status of RESET button, 0==pressed, without delay, updates each main loop cycle

#ifndef PC //IK20251230
// bits of Display_Info.button_states, changed by CommBoard
// this is minimum debounce interval, pressing for shorter interval would not create "button pressed" event. SHORT_PRESS_BIT and LONG_PRESS_BIT are stay cleared
#define DEBOUNCE_DELAY        50 // ms, 0.05 sec

// this is holding, then releasing after at least 150 ms button interval, SHORT_PRESS_BIT is set on the RELEASE of the button unless it is pressed for more than LONG_PRESS_DELAY ms
#define SHORT_PRESS_DELAY    150 // ms, 0.15 sec

// this is minimum holding button interval to get into different menu for the "long hold". pressing for that longer interval would create "button hold" event and clear "button pressed" event
#define LONG_PRESS_DELAY    3000 // ms, 3.0 sec.
// More than that: SHORT_PRESS_BIT is cleared, LONG_PRESS_BIT is set. Button timer should not stop and continue counting press and hold duration.
// This is useful for UD or DOWN buttons to accelerate increment / decrement of the value.

#define INC_DEC_BY_10_HOLD_TIME_ms		6000	// 10 sec
#define DEC_INC_BY_100_HOLD_TIME_ms		9000	// 20 sec
#define UpDownChange_rate_ms_START		 200	// Repeat interval, used by timer.UpDownChange_rate_ms
#else // For PC, reduced timing for quicker debug
#define DEBOUNCE_DELAY					  50	// ms, 0.05 sec
#define SHORT_PRESS_DELAY				 100	// ms, 0.15 sec
#define LONG_PRESS_DELAY				1000	// ms, 3.0 sec.
#define INC_DEC_BY_10_HOLD_TIME_ms		2000	// 10 sec
#define DEC_INC_BY_100_HOLD_TIME_ms		4000	// 20 sec
#define UpDownChange_rate_ms_START		 100	// Repeat interval, used by timer.UpDownChange_rate_ms
#endif

// bits are used in Display_Info.butt_states
// *_PRESS_BITs auto-clear in Operation() after processing menu
// in order to fit state bits for 5 buttons into uint16: short press -lower byte 5 bits
#define BUTTON_AUTO_SHORT_PRESS_BIT     Bit_0	// if (buttons & 0x0001)	// short press of Manual/auto, bit auto-clears in Operation() after processing menu
#define BUTTON_LIMIT_SHORT_PRESS_BIT    Bit_1	// if (buttons & 0x0002)	// Short press of Limit button, bit auto-clears in Operation() after processing menu
#define BUTTON_UP_SHORT_PRESS_BIT       Bit_2	// if (buttons & 0x0004)	// Short press of up button, bit auto-clears in Operation()
#define BUTTON_DOWN_SHORT_PRESS_BIT     Bit_3	// if (buttons & 0x0008)	// Short press of down button, bit auto-clears in Operation()
#define BUTTON_RESET_SHORT_PRESS_BIT    Bit_4	// if (buttons & 0x2000)	// Short press of reset button
// long press - next 5 bits, to access button states and set bits as in array
#define BUTTON_AUTO_LONG_PRESS_BIT      (BUTTON_AUTO_SHORT_PRESS_BIT  << 5)	// Bit_5	// if (buttons & 0x0100)	// Long press of Manual/auto, bit auto-clears in Operation() after processing menu
#define BUTTON_LIMIT_LONG_PRESS_BIT     (BUTTON_LIMIT_SHORT_PRESS_BIT << 5)	// Bit_6	// if (buttons & 0x0200)	// Long press of Limit button
#define BUTTON_UP_LONG_PRESS_BIT        (BUTTON_UP_SHORT_PRESS_BIT    << 5)	// Bit_7	// if (buttons & 0x0400)	// Long press of up button
#define BUTTON_DOWN_LONG_PRESS_BIT      (BUTTON_DOWN_SHORT_PRESS_BIT  << 5)	// Bit_8	// if (buttons & 0x0800)	// Long press of down button
#define BUTTON_RESET_LONG_PRESS_BIT     (BUTTON_RESET_SHORT_PRESS_BIT << 5)	// Bit_9	// if (buttons & 0x1000)	// Long press of reset button

// still hold - next 5 bits; these bits are set when button timer reaches LONG_PRESS_DELAY
// and reset when user releases button
#define BUTTON_AUTO_STILL_HELD_BIT      (BUTTON_AUTO_SHORT_PRESS_BIT  << 10)	// Bit_10 set after holding AUTO for more than LONG_PRESS_DELAY
#define BUTTON_LIMIT_STILL_HELD_BIT     (BUTTON_LIMIT_SHORT_PRESS_BIT << 10)	// Bit_11 set after holding AUTO for more than LONG_PRESS_DELAY
#define BUTTON_UP_STILL_HELD_BIT        (BUTTON_UP_SHORT_PRESS_BIT    << 10)	// Bit_12 status of UP button, 1 == still holding after LONG_PRESS_DELAY, updates when BUTTON_UP_LONG_PRESS_BIT is set
#define BUTTON_DOWN_STILL_HELD_BIT      (BUTTON_DOWN_SHORT_PRESS_BIT  << 10)	// Bit_13 status of DOWN button, 1==still holding after LONG_PRESS_DELAY, updates when BUTTON_UP_LONG_PRESS_BIT is set
#define BUTTON_RESET_STILL_HELD_BIT     (BUTTON_RESET_SHORT_PRESS_BIT << 10)	// Bit_14 status of DOWN button, 1==still holding after LONG_PRESS_DELAY, updates when BUTTON_UP_LONG_PRESS_BIT is set

// for keyboard simulation of buttons using arrow keys, used in simulation - see "PC_SUPP.c"
#define BUTTON_BIT_LEFT      BUTTON_AUTO_INSTANT_PRESS_BIT		// Bit_0
#define BUTTON_BIT_RIGHT     BUTTON_LIMIT_INSTANT_PRESS_BIT		// Bit_1
#define BUTTON_BIT_UP        BUTTON_UP_INSTANT_PRESS_BIT		// Bit_2
#define BUTTON_BIT_DOWN      BUTTON_DOWN_INSTANT_PRESS_BIT		// Bit_3

#define NO_PRESSED_BUTTONS   0 // the (~(BIT_BUT_RIGHT | BIT_BUT_UP | BIT_BUT_DOWN | BIT_BUT_LEFT | BIT_BUT_RESET)) produces int = 0xffffffe0
#define BUTTON_THRESHOLD_TIME_ms   10

#define NUM_BUTTONS    5

typedef struct  {				// use received from front board (via TWI communication) states of the front buttons, process button presses and output to upper or lower LED displays
	char DigitalStr[11];					// upper 5-chars LED, but it could be up to 5 additional dots '.', extra byte at the end in case End of String is needed
	char InfoStr[9];						// lower 4-chars LED, but it could be up to 4 additional dots '.', extra byte at the end in case End of String is needed
	volatile uint8  buttons_hits;			// lower 5 bits hold mirror of front board button presses - if button pressed bit is set to 1, released == 0, arrives via TWI from front board
											// upper two bits reflect event if user continues holding UP or DOWN button more then LONG_PRESS_DELAY. Used to increase delta increment of a limit voltage or DNP /ModBus address
	volatile uint16  butt_states;			// holds events SHORT_PRESS, LONG_PRESS of button states. Comm board handles states and does menu
	uint16 PressTimeStamp[NUM_BUTTONS];		// works with FreeRunningCounter to calculate duration of hold for auto increment/decrement 
	volatile char  ButtonStateChanged;		// indicates new button pressed 0=CLR; 1=SET (pressed); 2=BTN_RELEASED
	volatile char  ProcessingButton;		// indicates that menu is or was called. Blocks other button presses in firmware or, in simulation, Windows multiple events
	volatile char  DisplayNeedsUpdateFlag;	// display needs update. Sets after button pressed or measurement (ex: battery voltage) is ready and needs to be shown
	volatile uint8  last_butt_pressed;		// updates only on new hit, does not reset to zero. if button pressed bit is set to 1
	char  Info_segments_on_off;				// this creates blinking info display
	char  Alarm_LED_on_off;					// this creates blinking ALARM LED on front panel
	uint16 Status;							// status of Front board, LEDs, Buzzer, and states of calibration, etc.
	// uint8 LED_status;					// lower byte of Display_Info.Status is used to control LEDs and buzzer on Front board // keeps status of LEDs on front panel
	// uint8 display_status;				// IK20250814 these bits used with former 'Display_Info.display_status', now upper byte of Display_Info.Status // status of display etc.
	uint16 alarm_status;					// IK20250811 increased to 2 bytes to fit LastGasp. status of active alarms
}DisplayInfo;
extern DisplayInfo Display_Info;

enum UnitIndex { // used to access array Alarm_Limits[]
	index24,  // = 0
	index48,  // = 1
	index125, // = 2
	index250, // = 3
};

enum SetRange { // used to access settings set in Alarm_Limits[]
	MinSet, // = 0
	DefSet, // = 1
	MaxSet, // = 2
};

enum CriteriaIndex { // used to access settings set in Alarm_Limits[]
	index_LOW_BAT = 0,
	index_HI_BAT  = 1,
	index_MINUS_GF= 2,
	index_PLUS_GF = 3,
};

typedef struct                    // this structure must be initialized
{
	float low_bat_threshold_V_f[3];           // MIN, DEF, MAX; MIN - MAX is allowed range to change using UP-DOWN Display_Info.buttons
	float hi_bat_threshold_V_f[3];            //
	float minus_gf_threshold_V_f[3];          //
	float plus_gf_threshold_V_f[3];           //
}sAlarm_criteria;

union uAlarm_criteria { // used to access members as array
	float AlarmCriteria[4][3]; // 4*3 = 12 uint16 values
	sAlarm_criteria s; // as structure
} ;

extern const union uAlarm_criteria Alarm_Limits[]; // values are saved in centiVolts, for example, 120.5 V is saved as 12050

/*
IK20250715 copilot suggested functions
typedef struct {
	volatile uint16 timer;
	uint8 state; // pressed, released, etc.
	uint8 short_press_bit;
	uint8 long_press_bit;
} ButtonHandler;
extern ButtonHandler button_handlers[NUM_BUTTONS];

void ButtonHandler_Init(ButtonHandler* handler, uint8 short_bit, uint8 long_bit);
void ButtonHandler_Update(ButtonHandler* handler, uint8 is_pressed, uint16 short_delay, uint16 long_delay, volatile uint16* butt_states);
*/
