/*****************************************************************************
*
* Filename: Front_menu.c
*
* $Source:  $
* Last Saved Revision Number $Revision:  $
* Last Saved Revision $Date: $
* Last Saved Revision $Author:  $
*
* Author:  Igor Kordunsky
* Date: 06/17/2025
*
* File Description: user interface code ported from Display board
*****************************************************************************/
#include "GLOBALS.H"
#include <string.h> // for memset, memcpy

#define BTN_IGNORE_MS         100 // DEBOUNCE_DELAY
#define BTN_SHORT_MAX_MS     3000 // For short press on release
//#define BTN_AUTOINC_MS       300 // LONG_PRESS_DELAY
#define BTN_DELTA_1_MS       3000 // INC_DEC_BY_10_HOLD_TIME_ms
#define BTN_DELTA_10_MS      6000 // INC_DEC_BY_10_HOLD_TIME_ms
#define BTN_DELTA_100_MS     9000 // DEC_INC_BY_100_HOLD_TIME_ms
#define BTN_AUTOINC_PERIOD    200 // UpDownChange_rate_ms_START
typedef enum
{
	DIR_DOWN = -1,
	DIR_OTHER = 0,
	DIR_UP = +1
} ButtonDirection_t;

// Button state machine
typedef enum {
	BUT_IDLE,
	BUT_PRESSED,  // Held, waiting for release or long threshold
	BUT_AUTOINC   // UP/DOWN only: Auto inc/dec during hold
} ButtonState_t;
//-!- IK20250808  modes should be unified into one structure?
uint8  last_display_mode;
uint8  In_setup_alarm_limits_mode;				// indicates if in the setup alarm limits mode (after LIMIT button was pressed for 3+ sec). allows to setup HBAT, LBAT, etc., limits. to disable alarms
uint8  In_calibr_menu_mode;
uint8  last_mode;							// to remeber last mode when entering limit menu
uint8  Is_in_auto_mode;						// IK20251007 combined auto and manual mode in one variable. == TRUE means 'Auto', == FALSE means 'Manual' //indicates whether auto mode is active
//uint8  manual_mode;							// indicates whether manual mode is active
uint8  limit_mode;							// indicates whether limit mode is active. Activates after pressing LIMIT button for 3+ sec
uint8  display_mode;						// indicates what is being displayed and the actions available to user. The pointer 'p_InfoStr' holds what is shown on the bottom 14-segmets 4 indicators Info display

//---- Display Board variables
extern SettingsStruct Existing;				// keep current setting to detect change vs SysData.NV_UI
extern const char FL* ProtocolNames;
extern uint16 timeDelta;
uint8  comm_value_change;					// indicates if a value is being changed
uint8  baud_value_change;
uint16 latched_alarm_status;				// remebers alarms if latch is enabled. persistent latched alarm can be cleaned by a reset or by disabling latch bit

#define Delta_short_press_Voltages		0.1f	// IK20251111 used in manual mode to increment/decrement voltages
#define Delta_long_press_Voltages		1.0f	// IK20251111 used in manual mode to increment/decrement voltages
#define Delta_very_long_press_Voltages	10.0f	// IK20251230 used in manual mode to increment/decrement voltages
float Delta_Voltages = Delta_short_press_Voltages; // IK20251111 used in manual mode to increment/decrement voltages

//static float CalculateAndClampNewValue(float currentValue, float minValue, float maxValue, float delta, signed char direction)
//{
//	float newValue = currentValue + (delta * direction);
//	// clamp
//	if (newValue < minValue) newValue = minValue;
//	if (newValue > minValue) newValue = maxValue;
//	return newValue;
//}


//#define INC_DEC_BY_10_HOLD_TIME_ms		10000	// 10 sec
//#define DEC_INC_BY_100_HOLD_TIME_ms		20000	// 20 sec

//extern void cputs(char* );

char FL * mode_strings[] = {				// used in Front_menu.c, strings for display modes
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
"SYS ",			// 12  SHOW_FW_VER,		// Show firmware version
"CAL1",			// 13  CAL1_SET_4_20_MODE,	// Calibration mode 4-20 mA
"I LO",			// 14  CAL2_4mA,			// 4mA
"I HI",			// 15  CAL3_20mA,			// 20mA
"CAL5",			// 16  CAL5_SET_0_1_MODE,	// 0-1mA
"0 MA",			// 17  CAL6_0mA,			//-!- not used anymore 0mA; instead show "I Lo"
"1 MA",			// 18  CAL7_1mA,			//-!- not used anymore 1mA; instead show "I Hi"
"VCAL",			// 19  CAL_V_BAT,			// calibrate bat volts             18th item
"ALAC",			// 20  ALARM_AC,			// AC Alarm Enable/Disable
"ALRV",			// 21  ALARM_RIV,			// Ripple Voltage Enable/Disable   21th item
"ALRI",			// 22  ALARM_RII,			// Ripple Current Enable/Disable
"ALHZ",			// 23  ALARM_HI_Z,			// HI Z Alarm Enable/Disable
"PROT",			// 24  SELECT_PROTOCOL,		// IK20251209 new menu item - selecting communication protocol
"ADDR",			// 25  COMM_ADDR,			// system communication address
"BAUD",			// 26  COMM_BAUD,			// system baud rate
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

/*********************************************************************/
/*                G E T   B U T T O N   P R E S S                    */
/*********************************************************************/
/*  Description:  This routine parses the setup msg and cause the
				  required action to occur.

	buttons have pullup resistors; when button is pressed the port reads logic zero
	Inputs:       Port Data
	Outputs:      Button_Status
	Notes:        None
	Revisions:    20250618 IK
*/
/*********************************************************************/

//#define SHOW_BUTTON_PRESS

#define DISPLAY_MENU	// the Comm board performing menu logic and send strings to upper and lower LED indicators
// this is minimum debounce interval, pressing for shorter interval would not create "button pressed" event.
//IK20250708 in "Structure_defs.h" #define SHORT_PRESS_DELAY    100 // ms, 0.1 sec
// this is minimum holding button interval, pressing for longer interval would create "button hold" event and clear "button pressed" event
//IK20250708 in "Structure_defs.h" #define LONG_PRESS_DELAY    3000 // ms, 3.0 sec
//uint16 timer_ms; //IK202050929 global variable for test
void RecognizeButtonState(uint8 Btn_Index, volatile uint16 *p_timer)
{
	// if a timer was already running (>0) it means the button was pressed;
	// if running interval = timer.auto_button = (button released - button pressed) is between 0.1 & 3.0 sec
	// set "Short Press" event
#ifdef DISPLAY_MENU
	uint16 timer_ms;
	if (Btn_Index > BTN_MAX_INDEX) return; // safety exit if index is wrong
	timer_ms = (uint16)(*p_timer); // read a timer once; this is a quick function for button capture, timer would not change more than a ms, not important for menu operation

	// start from debouncing, clear both bits
	clearBit(Display_Info.butt_states, ((BUTTON_AUTO_SHORT_PRESS_BIT | BUTTON_AUTO_LONG_PRESS_BIT) << Btn_Index));

	//check state of a button, it arrives via TWI from Front board as a bit in byte
	// button currently pressed? (buttons_hits is sampled via TWI once in 20 ms)
	if ((Display_Info.buttons_hits & (BUTTON_AUTO_INSTANT_PRESS_BIT << Btn_Index)) != 0)
	{
		if (timer_ms == 0) {
			// start counting (1 means enabled and ~1 ms elapsed after next ISR tick)
			*p_timer = 1;
			Display_Info.PressTimeStamp[Btn_Index] = timer.FreeRunningCounter; // remember time of pressing the button
			// On new press:
			clearBit(Display_Info.long_press_fired, (1 << Btn_Index));
		}
		else if (timer_ms >= LONG_PRESS_DELAY) {
			// On long-press trigger:
			setBit(Display_Info.long_press_fired, (1 << Btn_Index));
			// reached a long-hold block: issue one long event and restart timer for next block
			setBit(Display_Info.butt_states, ((BUTTON_AUTO_LONG_PRESS_BIT | BUTTON_AUTO_STILL_HELD_BIT) << Btn_Index)); // set "still_holding" for auto increment/decrement
			// Prevent short-press from firing on release
			clearBit(Display_Info.butt_states, (BUTTON_AUTO_SHORT_PRESS_BIT << Btn_Index));

			//if (Btn_Index == BTN_INDEX_UP)
			//	setBit(Display_Info.butt_states, BUTTON_UP_STILL_HELD_BIT);
			//if (Btn_Index == BTN_INDEX_DOWN)
			//	setBit(Display_Info.butt_states, BUTTON_DOWN_STILL_HELD_BIT);

			*p_timer = 1;// restart counting for the next LONG_PRESS_DELAY cycle (use 1, not 0)
		}
		// otherwise keep counting; do not set short-press while held
	}
	else // button released
	{
		// if released between short and long thresholds -> short press event
		// Check bitmap instead of array
		if (!(Display_Info.long_press_fired & (1 << Btn_Index)) &&
			timer_ms >= SHORT_PRESS_DELAY &&
			timer_ms < LONG_PRESS_DELAY)
		{
			setBit(Display_Info.butt_states, (BUTTON_AUTO_SHORT_PRESS_BIT << Btn_Index));
		}		// stop timer on release
		*p_timer = 0;
	}
#endif // #ifdef DISPLAY_MENU
}

void Get_Button_Press(void)	// IK20260127 included into Visual Studio
{
	RecognizeButtonState(BTN_INDEX_AUTO, &timer.auto_button);	//check state of Manual/Auto button, it arrives via TWI from Front board
	RecognizeButtonState(BTN_INDEX_LIMIT, &timer.limit_button);
	RecognizeButtonState(BTN_INDEX_UP, &timer.up_button);
	RecognizeButtonState(BTN_INDEX_DOWN, &timer.down_button);
	RecognizeButtonState(BTN_INDEX_RESET, &timer.reset_button);
	//if ((((Display_Info.buttons_hits & BTN_INDEX_UP) != 0) && (timer.up_button > LONG_PRESS_DELAY))
	//|| (((Display_Info.buttons_hits & BTN_INDEX_DOWN) != 0) && (timer.down_button > LONG_PRESS_DELAY)))
	//	Delta_Voltages = Delta_long_press_Voltages; // IK20251111 used in manual mode to increment/decrement voltages
	//else
	//	Delta_Voltages = Delta_short_press_Voltages; // IK20251111 used in manual mode to increment/decrement voltages
}

void Do_Front_menu(void)
{
	//Buttons : BIT_0 = 1 LEFT pressed, BIT_3 = 1 central ENTER pressed, BIT_4 = 1 RIGHT pressed, BIT_5 = 1 UP pressed, BIT_6 = 1 DOWN pressed
	Operation();
	Display_Info.DisplayNeedsUpdateFlag = SET;
}

void ProcessUPbutton() {
	if (limit_mode == TRUE)
	{
		if (timer.UpDownChange_rate_ms == 0)				// and timer is 0
		{
			timeDelta = timer.FreeRunningCounter - Display_Info.PressTimeStamp[BTN_INDEX_UP]; // calculate time passed from the press
			timer.UpDownChange_rate_ms = BTN_AUTOINC_PERIOD;				// set for 0.1 sec, decrements in timer ISR
			if (In_setup_alarm_limits_mode == TRUE) // change in SYS menu
			{
				if (display_mode == ALARM_RIV)	clearBit(SysData.NV_UI.disabled_alarms, Alarm_Ripple_Voltage_Bit);
				if (display_mode == ALARM_RII)	clearBit(SysData.NV_UI.disabled_alarms, Alarm_Ripple_Current_Bit);
				if (display_mode == ALARM_AC)	clearBit(SysData.NV_UI.disabled_alarms, Alarm_AC_Loss_Bit       );
				if (display_mode == ALARM_HI_Z)	clearBit(SysData.NV_UI.disabled_alarms, Alarm_High_Impedance_Bit);
			}

			// ************   LONG PRESS UP    **************
			if (Display_Info.butt_states & BUTTON_UP_LONG_PRESS_BIT)		//Long press of up button
			{
				if ((display_mode == CAL2_4mA) || (display_mode == CAL3_20mA) ||
					(display_mode == CAL6_0mA) || (display_mode == CAL7_1mA))
				{
					setBit(Display_Info.Status, DISP_STATE_ButtonUP_BIT);			// Display_Info.Status |= 0x08;    //Set up button
					clearBit(Display_Info.Status, DISP_STATE_ButtonDOWN_BIT);		// Display_Info.Status &= 0xFB;    //clear down button
					if ((display_mode == CAL6_0mA) || (display_mode == CAL7_1mA))
					{
						//setBit(Display_Info.Status, DISP_STATE_0_1mA_BIT);		// Display_Info.Status |= 0x80; //set 0-1ma bit
						setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
					}
					else
					{
						//clearBit(Display_Info.Status, DISP_STATE_0_1mA_BIT);		// Display_Info.Status &= 0x7F; //clr 0-1ma bit
						clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
					}
				}

				timer.limit_mode_timeout_ms = 0;									// keeps it going for another ten minutes
			} // end of BUTTON_UP_LONG_PRESS_BIT
			// below, both short and long press

			// Accelerated increment logic
			if ((display_mode >= HI_BAT_THRESHOLD) && (display_mode <= MINUS_GF_THRESHOLD))
			{
				Delta_Voltages = Delta_short_press_Voltages;			// IK20251111 decrease increment to 0.1V Volts
				if (Display_Info.butt_states & BUTTON_UP_STILL_HELD_BIT)
				{
					if (timeDelta > BTN_DELTA_100_MS)
						Delta_Voltages = Delta_very_long_press_Voltages; // 10.0f
					else if (timeDelta > BTN_DELTA_10_MS)
						Delta_Voltages = Delta_long_press_Voltages;				// IK20251111 increase increment to 1.0f Volts
				}
			}

			if (display_mode == HI_BAT_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_HI_BAT, &SysData.NV_UI.high_bat_threshold_V_f, Delta_Voltages, INCREMENT); // increment if in range
			}
			else if (display_mode == LOW_BAT_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_LOW_BAT, &SysData.NV_UI.low_bat_threshold_V_f, Delta_Voltages, INCREMENT); // increment if in range
			}
			else if (display_mode == PLUS_GF_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_PLUS_GF, &SysData.NV_UI.plus_gf_threshold_V_f, Delta_Voltages, INCREMENT); // increment if in range
			}
			else if (display_mode == MINUS_GF_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_MINUS_GF, &SysData.NV_UI.minus_gf_threshold_V_f, Delta_Voltages, INCREMENT); // increment if in range
			}
			else if (display_mode == RippleVOLT_TR_HOLD)
			{
				if ((SysData.NV_UI.ripple_V_threshold_mV_f >= 5) && (SysData.NV_UI.ripple_V_threshold_mV_f < MAX_V_I_rip_LIMIT))
					SysData.NV_UI.ripple_V_threshold_mV_f++;
			}
			else if (display_mode == RippleCURR_TR_HOLD)
			{
				if ((SysData.NV_UI.ripple_I_threshold_mA_f >= 5) && (SysData.NV_UI.ripple_I_threshold_mA_f < MAX_V_I_rip_LIMIT))
					SysData.NV_UI.ripple_I_threshold_mA_f++;
			}
			else if (display_mode == TIME_DELAY_SET)
			{
				//Increase_time_delay();
				if (SysData.NV_UI.alarm_delay_sec_f <= (MAX_time_delay - INCREMENT_time_delay))
					SysData.NV_UI.alarm_delay_sec_f += INCREMENT_time_delay;            //plus 1 second
			}

			else if ((display_mode == CAL2_4mA) || (display_mode == CAL6_0mA))// (Display_Info.Status & DISP_STATE_LOmA_BIT)			// 4 mA setting
			{
				SysData.CurrentOut_I420.Y1_lowCalibrVal += 0.0001f;
				I420_calibr_lock = I420calibLOCKED;
			}
			else if ((display_mode == CAL3_20mA) || (display_mode == CAL7_1mA)) // if (Display_Info.Status & DISP_STATE_HImA_BIT)		// 20 mA setting
			{
				SysData.CurrentOut_I420.Y2_highCalibrVal += 0.0001f;
				I420_calibr_lock = I420calibLOCKED;
			}

			else if (display_mode == CAL_V_BAT)
			{
				if (SysData.Bat_Cal_Offset_Volts_f < SysData.NV_UI.plus_gf_threshold_V_f)			// IK20251110 limit the adjustable range
				{
					SysData.Bat_Cal_Offset_Volts_f += 0.1f;
					SaveToEE(SysData.Bat_Cal_Offset_Volts_f);								// Store the bat offset in EEPROM
				}
			}
			else if (display_mode == SELECT_PROTOCOL)
			{
				if (SysData.NV_UI.StartUpProtocol < ASCII_CMDS)			// IK20251110 limit the adjustable range
				{
					SysData.NV_UI.StartUpProtocol += PROTOCOL_SELECTION_INC_DEC;
					SaveToEE(SysData.NV_UI.StartUpProtocol);								// Store the protocol in EEPROM
				}
			}

			else if (display_mode == COMM_ADDR) //-!- IK20251111 extend to ModBus, range 0 to 255
			{
				float addrDelta;
				float tmpAdr;
				if (comm_value_change == FALSE)
					Existing.meter_address = SysData.NV_UI.meter_address;	//Do once upon 1st change
				comm_value_change = TRUE;
				if (timeDelta <= INC_DEC_BY_10_HOLD_TIME_ms)
					addrDelta = 1.0f;
				else if (timeDelta < DEC_INC_BY_100_HOLD_TIME_ms)
					addrDelta = 10.0f;
				else
					addrDelta = 100.0f;

				tmpAdr = Existing.meter_address;
				tmpAdr += addrDelta;
				if (tmpAdr > MAX_DNP3_ADDRESS) 
					tmpAdr = MAX_DNP3_ADDRESS;
				Existing.meter_address = tmpAdr;
			}
			else if (display_mode == COMM_BAUD)
			{
				long t_long; // ATMEL could not correctly shift left uint16 if it gets into uint32 and later converted to float
				if (BaudRateIndex < (Last_Baud_Index - 1))
					BaudRateIndex++;													// Increment baud rate index
				else
					BaudRateIndex = (Last_Baud_Index - 1);								// Ensure BaudRateIndex does not exceed the maximum index
				t_long = Baud_Rates[BaudRateIndex]; // IK20251224 ATMEL could not correctly shift left uint16 if it gets into uint32 and later converted to float
				Existing.baud_rate = t_long << 2;										// IK20250826  Baud_Rates are saved divided by 4 to keep values inside uint16 range
				//Existing.baud_rate = Baud_Rates[BaudRateIndex] * 4.0f;				// IK20251224  using float math, it takes extra 8 bytes of flash
				Set_and_Save_New_BaudRateIndex(BaudRateIndex);
			}

			else if (display_mode == PHASE_STATE_SET)
				setBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);		// Triple_Phase_Setting

			else if (display_mode == PULSE_STATE_SET)
				rt.pulse = ON;

			else if (display_mode == BUZZER_STATE_SET)
				setBit(SysData.NV_UI.SavedStatusWord, Buzzer_ON_eq1_Bit);

			else if (display_mode == LATCHED_STATE_SET)
				setBit(SysData.NV_UI.SavedStatusWord, Latch_ON_eq1_Bit);				// set latched (persistent) alarms

			else if (display_mode == AUTO_MAN_MODE)
			{
				Is_in_auto_mode = TRUE;													// changes to Auto
				//manual_mode = FALSE;
				limit_mode = FALSE;
			}
		} // end of timer 100 ms
	}//end limit mode
}	// end of ProcessUPbutton

void ProcessDOWNbutton() {
	timeDelta = timer.FreeRunningCounter - Display_Info.PressTimeStamp[BTN_INDEX_DOWN];
	if (limit_mode == TRUE)
	{
		if (timer.UpDownChange_rate_ms == 0)
		{
			timer.UpDownChange_rate_ms = BTN_AUTOINC_PERIOD;
			if (In_setup_alarm_limits_mode == TRUE)
			{
				if (display_mode == ALARM_RIV)	setBit(SysData.NV_UI.disabled_alarms, Alarm_Ripple_Voltage_Bit);
				if (display_mode == ALARM_RII)	setBit(SysData.NV_UI.disabled_alarms, Alarm_Ripple_Current_Bit);
				if (display_mode == ALARM_AC)	setBit(SysData.NV_UI.disabled_alarms, Alarm_AC_Loss_Bit);
				if (display_mode == ALARM_HI_Z)	setBit(SysData.NV_UI.disabled_alarms, Alarm_High_Impedance_Bit);
			}

			// Immediate DOWN actions (do not require long-press)
			if (display_mode == PULSE_STATE_SET)
			{
				rt.pulse = OFF;
			}
			else if (display_mode == BUZZER_STATE_SET)
			{
				clearBit(SysData.NV_UI.SavedStatusWord, Buzzer_ON_eq1_Bit);
			}
			else if (display_mode == LATCHED_STATE_SET)
			{
				clearBit(SysData.NV_UI.SavedStatusWord, Latch_ON_eq1_Bit);
			}
			else if (display_mode == PHASE_STATE_SET)
			{
				clearBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);
			}
			else if (display_mode == AUTO_MAN_MODE)
			{
				Is_in_auto_mode = FALSE;
				limit_mode = FALSE;
			}

			// Accelerated decrement logic - same structure as UP button
			if ((display_mode >= HI_BAT_THRESHOLD) && (display_mode <= MINUS_GF_THRESHOLD))
			{
				Delta_Voltages = Delta_short_press_Voltages;  // default 0.1f
				if (Display_Info.butt_states & BUTTON_DOWN_STILL_HELD_BIT)
				{
					if (timeDelta > BTN_DELTA_100_MS)
						Delta_Voltages = Delta_very_long_press_Voltages;  // 10.0f
					else if (timeDelta > BTN_DELTA_10_MS)
						Delta_Voltages = Delta_long_press_Voltages;       // 1.0f
				}
			}

			// LONG PRESS DOWN (keep for calibration direction selection etc.)
			if (Display_Info.butt_states & BUTTON_DOWN_LONG_PRESS_BIT)
			{
				if ((display_mode == CAL2_4mA) || (display_mode == CAL3_20mA) ||
					(display_mode == CAL6_0mA) || (display_mode == CAL7_1mA))
				{
					setBit(Display_Info.Status, DISP_STATE_ButtonDOWN_BIT);
					clearBit(Display_Info.Status, DISP_STATE_ButtonUP_BIT);
					if ((display_mode == CAL6_0mA) || (display_mode == CAL7_1mA))
						setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
					else
						clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
					timer.limit_mode_timeout_ms = 0;
				}

				else if (display_mode == PHASE_STATE_SET)
				{
					clearBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);
				}
				else if (display_mode == PULSE_STATE_SET)
				{
					rt.pulse = OFF;
				}
				else if (display_mode == BUZZER_STATE_SET)
				{
					clearBit(SysData.NV_UI.SavedStatusWord, Buzzer_ON_eq1_Bit);
				}
				else if (display_mode == LATCHED_STATE_SET)
				{
					clearBit(SysData.NV_UI.SavedStatusWord, Latch_ON_eq1_Bit);
				}
				if (display_mode == AUTO_MAN_MODE)
				{
					Is_in_auto_mode = FALSE;
					limit_mode = FALSE;
				}
				timer.limit_mode_timeout_ms = 0;
			}

			// Existing numeric decrement handling continues below...
			if (display_mode == HI_BAT_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_HI_BAT, &SysData.NV_UI.high_bat_threshold_V_f, Delta_Voltages, DECREMENT);
			}
			else if (display_mode == LOW_BAT_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_LOW_BAT, &SysData.NV_UI.low_bat_threshold_V_f, Delta_Voltages, DECREMENT);
			}
			else if (display_mode == PLUS_GF_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_PLUS_GF, &SysData.NV_UI.plus_gf_threshold_V_f, Delta_Voltages, DECREMENT);
			}
			else if (display_mode == MINUS_GF_THRESHOLD)
			{
				CheckVariableRangeAndChange(SysData.NV_UI.unit_index, index_MINUS_GF, &SysData.NV_UI.minus_gf_threshold_V_f, Delta_Voltages, DECREMENT);// decrement if in range
			}
			else if (display_mode == RippleVOLT_TR_HOLD)
			{
				if ((SysData.NV_UI.ripple_V_threshold_mV_f > 5) && (SysData.NV_UI.ripple_V_threshold_mV_f <= 2000))
					SysData.NV_UI.ripple_V_threshold_mV_f--;
			}
			else if (display_mode == RippleCURR_TR_HOLD)
			{
				if ((SysData.NV_UI.ripple_I_threshold_mA_f > 5) && (SysData.NV_UI.ripple_I_threshold_mA_f <= 2000))
					SysData.NV_UI.ripple_I_threshold_mA_f--;
			}
			else if (display_mode == TIME_DELAY_SET)
			{
				//Decrease_time_delay();
				if (SysData.NV_UI.alarm_delay_sec_f >= (MIN_time_delay + INCREMENT_time_delay))
					SysData.NV_UI.alarm_delay_sec_f -= INCREMENT_time_delay;            //minus 1 second

			}
			else if (display_mode == PHASE_STATE_SET)
			{
				clearBit(SysData.NV_UI.SavedStatusWord, SinglePhase_eq0_3ph_eq1_Bit);
			}

			else if ((display_mode == CAL2_4mA) || (display_mode == CAL6_0mA))// (Display_Info.Status & DISP_STATE_LOmA_BIT)			// 4 mA setting
			{
				SysData.CurrentOut_I420.Y1_lowCalibrVal -= 0.0001f;
				I420_calibr_lock = I420calibLOCKED;
			}
			else if ((display_mode == CAL3_20mA) || (display_mode == CAL7_1mA)) // if (Display_Info.Status & DISP_STATE_HImA_BIT)			// 20 mA setting
			{
				SysData.CurrentOut_I420.Y2_highCalibrVal -= 0.0001f;
				I420_calibr_lock = I420calibLOCKED;
			}

			else if (display_mode == CAL_V_BAT)
			{
				if (SysData.Bat_Cal_Offset_Volts_f > -SysData.NV_UI.minus_gf_threshold_V_f) {
					SysData.Bat_Cal_Offset_Volts_f -= 0.1f;
					SaveToEE(SysData.Bat_Cal_Offset_Volts_f);								// Store the new bat offset in EEPROM
				}
			}

			else if (display_mode == SELECT_PROTOCOL)
			{
				if (SysData.NV_UI.StartUpProtocol > SETUP)			// IK20251110 limit the adjustable range
				{
					SysData.NV_UI.StartUpProtocol -= PROTOCOL_SELECTION_INC_DEC;
					SaveToEE(SysData.NV_UI.StartUpProtocol);								// Store the protocol in EEPROM
				}
			}

			else if (display_mode == COMM_ADDR)
			{
				float addrDelta;
				float tmpAdr;
				if (comm_value_change == FALSE)
					Existing.meter_address = SysData.NV_UI.meter_address;		// Do once upon 1st change
				comm_value_change = TRUE;
				if (timeDelta <= INC_DEC_BY_10_HOLD_TIME_ms)
					addrDelta = 1.0f;
				else if (timeDelta < DEC_INC_BY_100_HOLD_TIME_ms)
					addrDelta = 10.0f;
				else
					addrDelta = 100.0f;
				tmpAdr = Existing.meter_address;
				tmpAdr -= addrDelta;
				if (tmpAdr < 1)
					tmpAdr = 1;
				Existing.meter_address = tmpAdr;
			}

			else if (display_mode == COMM_BAUD)
			{
				long t_long; // ATMEL could not correctly shift left uint16 if it gets into uint32 and later converted to float
				if (BaudRateIndex > (Last_Baud_Index - 1))
					BaudRateIndex = Last_Baud_Index - 1;						// Ensure BaudRateIndex does not exceed the maximum index
				if (BaudRateIndex > Baud_300_i)
					BaudRateIndex--;											// Decrement baud rate index
				t_long = Baud_Rates[BaudRateIndex];							// keep Existing mirror consistent for DisplayPrepare()
				Existing.baud_rate = t_long << 2;								// IK20250826 Baud_Rates are stored divided by 4

				Set_and_Save_New_BaudRateIndex(BaudRateIndex);
			}
		} // end of timer 100 ms
	}//end limit mode
}//end of down button press

/// <summary>
/// IK20250728
/// Checks if a value is within the allowed range for a specific unit type and criteria index.
/// ----Values are saved in centiVolts, for example, 120.5 V is saved as 12050
/// IK20251029 values are saved as float in Volts or mV or mA
/// </summary>
/// <param name="UnitTypeIndex">The index specifying the unit voltage type in the Alarm_Limits array.</param>
/// <param name="CriteriaIndex">The index specifying the alarm criteria within the unit type.</param>
/// <param name="*value">The pointer to the value to check against the allowed range and change.</param>
/// <param name="DELTA_value">The increment/decrement step value.</param>
/// <returns>Returns -1 if the value is below the minimum, +1 if above the maximum, and 0 if within the range and increment/decrement if succeed.</returns>
int CheckVariableRangeAndChange(uint8 UnitTypeIndex, uint8 CriteriaIndex, float *value, float DELTA_value, signed char IncEq_pls1_DecEq_mns1)
{
	float min = Alarm_Limits[UnitTypeIndex].AlarmCriteria[CriteriaIndex][MinSet]; // MinSet is from enum SetRange
	float max = Alarm_Limits[UnitTypeIndex].AlarmCriteria[CriteriaIndex][MaxSet];
	float temp = *value;
	int return_val = 0;// assuming *value is in range

	if (*value < min) return_val = -1; // WAS too low  (will be clamped)
	else if (*value > max) return_val = 1;  // WAS too high (will be clamped)

	// perform operation
	temp += DELTA_value * IncEq_pls1_DecEq_mns1;// Apply signed delta

	// Clamp
	if (temp > max)
		temp = max;
	else if (temp < min)
		temp = min;

	*value = temp;
	return return_val;
}

typedef struct   {
	Schar current_mode;	// can be up to +126, negative value indicates end of the menu lookup
	Schar next_mode;	// can be up to +126, negative value indicates end of the menu lookup
}NextMenuItem;

NextMenuItem next_LIMIT_display_mode[] =
{	// this mode        next mode
	LIMIT_START,		VOLTS,
	VOLTS,				HI_BAT_THRESHOLD, // IK20260202 added to the beginning of the menu as in old firmware
	HI_BAT_THRESHOLD,	LOW_BAT_THRESHOLD,
	LOW_BAT_THRESHOLD,	PLUS_GF_THRESHOLD,
	PLUS_GF_THRESHOLD,	MINUS_GF_THRESHOLD,
	MINUS_GF_THRESHOLD,	RippleVOLT_TR_HOLD,
	RippleVOLT_TR_HOLD,	RippleCURR_TR_HOLD,
	RippleCURR_TR_HOLD,	TIME_DELAY_SET,
	TIME_DELAY_SET,		PHASE_STATE_SET,
	PHASE_STATE_SET,	PULSE_STATE_SET,
	PULSE_STATE_SET,	BUZZER_STATE_SET,
	BUZZER_STATE_SET,	LATCHED_STATE_SET,
	LATCHED_STATE_SET,	SHOW_FW_VER,
	SHOW_FW_VER,		CAL1_SET_4_20_MODE,
	CAL1_SET_4_20_MODE,	LIMIT_START,
	END_MENU,			END_MENU, // to indicate the end of the menu
};

NextMenuItem next_ALARM_display_mode[] =
{	// this mode        next mode
	SHOW_FW_VER,		ALARM_AC,
	ALARM_AC,			ALARM_RIV,
	ALARM_RIV,			ALARM_RII,
	ALARM_RII,			ALARM_HI_Z,
	ALARM_HI_Z,			SELECT_PROTOCOL,
	SELECT_PROTOCOL,	COMM_ADDR,
	COMM_ADDR,			COMM_BAUD,
	COMM_BAUD,			ALARM_AC,
	END_MENU,			END_MENU, // to indicate the end of the menu
};

NextMenuItem next_AUTO_display_mode[] =
{	// this mode        next mode
	VOLTS,				PLUS_GND_VOLTS,
	PLUS_GND_VOLTS,		MINUS_GND_VOLTS,
	MINUS_GND_VOLTS,	GND_FAULT_VOLTS,
	GND_FAULT_VOLTS,	RIPPLE_VOLTS,
	RIPPLE_VOLTS,		RIPPLE_CURRENT,
	RIPPLE_CURRENT,		TIME_DELAY_SHOW,
	TIME_DELAY_SHOW,	PHASE_SHOW,
	PHASE_SHOW,			VOLTS,
	END_MENU,			END_MENU, // to indicate the end of the menu
};

NextMenuItem next_CAL_display_mode[] =
{	// this mode        next mode
	CAL1_SET_4_20_MODE,	CAL2_4mA,
	CAL2_4mA,			CAL3_20mA,
	CAL3_20mA,			CAL_V_BAT,
	CAL_V_BAT,			CAL2_4mA,
	END_MENU,			END_MENU, // to indicate the end of the menu
};

Schar SelectNextMode(Schar this_mode, NextMenuItem * acting_menu) {
	uint8 dm; //display_mode;
	Schar retVal = this_mode;
	NextMenuItem* this_menu = acting_menu;
	for (dm = 0; dm < 50; dm++) // IK20251030 assuming we have less than 50 menu items, search through them
	{
		if(this_menu[dm].current_mode == this_mode)
		{
			if (this_menu[dm].next_mode != END_MENU) // If not the end of menu
				retVal = this_menu[dm].next_mode;
			else
				retVal = this_mode; // stay in the same mode if reached the end of the menu
			return retVal;
		}
	}
	return retVal; // stay in the same mode if reached the end of the menu
}

/*********************************************************************/
/*                O P E R A T I O N                                  */
/*********************************************************************/
/*  Description:  This routine operates the display board and causes
				  required action to occur for different button presses.

	Inputs:       Display_Info.butt_states,display_mode, manual_mode, Is_in_auto_mode,
				  timer.disp_var_change_ms, UpDownChange_rate_ms
	Outputs:      none
	Notes:        None
	Revisions:    12/06/15     REC     Created
*/
/*********************************************************************/
#ifdef DISPLAY_MENU
void Operation(void)
{   //Check Display_Info.butt_states
	//Check Manual/Auto button
//#ifdef PC // menu diagnostic -> console
#if (0) // menu diagnostic -> console
	uint8 ch;
	gotoxy(1, 24);
	PutStr(" Channel  |NotConn|Battery| Fault | -GND  |NotConn|RipCurr|RipVolt|NotConn\r\n");
//	gotoxy(1, 25);
	PutStr("ADC counts" );
	for (ch = 0;ch < 8;ch++) {
		printf("| %5d ", rt.ADC_buff[ch]);
	}
	printf("Buttons Hits 0x%02X, states 0x%04X ", Display_Info.buttons_hits, Display_Info.butt_states);
	printf("| ShowMode %s ", (Is_in_auto_mode == FALSE? "Manual":" Auto "));
	printf("| DispMode %d, %s ", display_mode, mode_strings[display_mode]);
	printf("| SetupMode %s ", (In_setup_alarm_limits_mode == TRUE? "Setup":" Work "));

#endif
	// change "Auto - manual" mode
	if (Display_Info.butt_states & BUTTON_AUTO_SHORT_PRESS_BIT)			// short press
	{
		if (Is_in_auto_mode == FALSE)									// display in manual mode, click  selects next value to show
		{
			if (display_mode < VOLTS) display_mode = VOLTS;				// <28? =28. in manual mode limit range between VOLTS and PHASE_SHOW
			else  display_mode++; //increment display mode
			if (display_mode > PHASE_SHOW) display_mode = VOLTS;		// >35

			rt.InfoLED_blink_eq1 = FALSE;
			last_display_mode = display_mode;
		}
		clearBit(Display_Info.butt_states, BUTTON_AUTO_SHORT_PRESS_BIT);	// clear short press
	}  // End of short press

	if (Display_Info.butt_states & BUTTON_AUTO_LONG_PRESS_BIT)				// Long press of Manual/auto
	{
		Is_in_auto_mode = 1 - Is_in_auto_mode;								// toggle auto/manual mode
		if (Is_in_auto_mode == TRUE)
			Display_AutoLED_ON;												// turn on AUTO LED
		else
			Display_AutoLED_OFF;											// turn off AUTO LED

		display_mode = VOLTS;
		clearBit(Display_Info.butt_states, BUTTON_AUTO_LONG_PRESS_BIT);
	}//end of long press ops

	//Check Limit Button
	if (Display_Info.butt_states & BUTTON_LIMIT_SHORT_PRESS_BIT)			//Short press of Limit button
	{
		if (limit_mode == TRUE)            //in limit mode so change functions
		{
			timer.disp_var_change_ms = 400;

			if (In_setup_alarm_limits_mode == TRUE)
			{
				display_mode = SelectNextMode(display_mode, next_ALARM_display_mode);
			}
			else if (In_calibr_menu_mode == TRUE)
			{
				display_mode = SelectNextMode(display_mode, next_CAL_display_mode);
			}
			else
			{
				display_mode = SelectNextMode(display_mode, next_LIMIT_display_mode);
				if ((display_mode > CAL1_SET_4_20_MODE)
					&& (display_mode != VOLTS) // enum LIMIT_MODE is zero
					) // below 0 < display_mode > above 11
					display_mode = LIMIT_START; // = 0
			}
		}
		clearBit(Display_Info.butt_states, BUTTON_LIMIT_SHORT_PRESS_BIT);// Display_Info.butt_states &= 0xFFFB;    //clear short press of limit
	}//end of limit button short press

	if (Display_Info.butt_states & BUTTON_LIMIT_LONG_PRESS_BIT)              // Long press of Limit button
	{
		if ((display_mode == CAL1_SET_4_20_MODE)		// == 13 if it is ALREADY in a current loop calibration mode low limit (4 mA or 0mA)
			|| (display_mode == CAL5_SET_0_1_MODE)		// == 16
			&& (In_calibr_menu_mode == FALSE))			// but calibration has not been started yet - start it
		{
			In_calibr_menu_mode = TRUE;
			if (display_mode == CAL1_SET_4_20_MODE)		// == 13
				display_mode = CAL2_4mA;				// == 14 low I 4 mA msg
			else
				display_mode = CAL6_0mA;				// == 17 low I 0 mA msg
		}
		else if ((display_mode == SHOW_FW_VER) && (In_setup_alarm_limits_mode == FALSE)) // if display show "SYS " and not in setup mode, means in a System mode where soft alarms are enabled/disabled - enter setup (LIMIT) mode
		{
			In_setup_alarm_limits_mode = TRUE;
			rt.InfoLED_blink_eq1 = FALSE;
			display_mode = ALARM_AC;					// == 20 AC alarm msg, info LED shows "ALAC"
		}
		else if ((display_mode >= ALARM_AC) && (display_mode <= ALARM_HI_Z) && (In_setup_alarm_limits_mode == TRUE))
		{
			SaveToEE(SysData.NV_UI.disabled_alarms);	// store potentially new value
			In_setup_alarm_limits_mode = FALSE;			// exit from setup mode
			display_mode = SHOW_FW_VER;					// = 12 // display show "SYS "
		}

		else if (limit_mode == FALSE)					// main entry point from the normal mode to the limit mode
		{
			limit_mode = TRUE;							// different menus
			last_mode = In_calibr_menu_mode;			// upon entering remember what mode we were in: we can enter from top menu, or return from CAL menu, or from SYS / ALARM setup menu
			In_calibr_menu_mode = FALSE;				// change mode to limit mode
			Is_in_auto_mode = FALSE;					// set to false, manual mode
			In_setup_alarm_limits_mode = FALSE;
			rt.InfoLED_blink_eq1 = TRUE;				// info display blinking indicating limit mode
			display_mode = LIMIT_START;					// = 0, display shows 'LMIT'
			memcpy(&Existing, &SysData.NV_UI, sizeof(SysData.NV_UI)); //remember current values in the Existing structure to see the changes. To extend life of EEPROM, do not write to it if nothing changed

			clearBit(Display_Info.butt_states, BUTTON_LIMIT_SHORT_PRESS_BIT); // - prevent release from advancing menu
		}
		else //if (limit_mode == TRUE)					// if is ALREADY in the limit mode - exit from it
		{
			limit_mode = FALSE;							// go back to last mode
			timer.limit_mode_timeout_ms = 0;
			In_calibr_menu_mode = last_mode;				//-!- IK20251014 was Is_in_auto_mode = last_mode, but last_mode was set to In_calibr_menu_mode above ???
			rt.InfoLED_blink_eq1 = FALSE;				// info display NOT blinking indicating normal mode

			display_mode = VOLTS;
			comm_value_change = FALSE; // IK20251031 allow to reload Comm address from EEPROM next time in the menu
			if (Existing.high_bat_threshold_V_f != SysData.NV_UI.high_bat_threshold_V_f)	//is it changed?
				SaveToEE(SysData.NV_UI.high_bat_threshold_V_f);
			if (Existing.low_bat_threshold_V_f != SysData.NV_UI.low_bat_threshold_V_f)		//is it changed?
				SaveToEE(SysData.NV_UI.low_bat_threshold_V_f);
			if (Existing.plus_gf_threshold_V_f != SysData.NV_UI.plus_gf_threshold_V_f)		//is it changed?
				SaveToEE(SysData.NV_UI.plus_gf_threshold_V_f);
			if (Existing.minus_gf_threshold_V_f != SysData.NV_UI.minus_gf_threshold_V_f)	//is it changed?
				SaveToEE(SysData.NV_UI.minus_gf_threshold_V_f);
			if (Existing.ripple_V_threshold_mV_f != SysData.NV_UI.ripple_V_threshold_mV_f)	//is it changed?
				SaveToEE(SysData.NV_UI.ripple_V_threshold_mV_f);
			if (Existing.ripple_I_threshold_mA_f != SysData.NV_UI.ripple_I_threshold_mA_f)	//is it changed?
				SaveToEE(SysData.NV_UI.ripple_I_threshold_mA_f);
			if (Existing.alarm_delay_sec_f != SysData.NV_UI.alarm_delay_sec_f)				//is it changed?
				SaveToEE(SysData.NV_UI.alarm_delay_sec_f);
			if ((Existing.SavedStatusWord) != (SysData.NV_UI.SavedStatusWord))				//is it changed?
				SaveToEE(SysData.NV_UI.SavedStatusWord);
// IK20250812 not saving pulse status in EEPROM, there is possibility that it would be enabled after power-off - on
			//if (Existing.phase != SysData.NV_UI.phase) //is it changed?
			//	SaveToEE(SysData.NV_UI.phase);
			// buzzer and latch are bits in SavedStatusWord
			//if (Existing.pulse != SysData.NV_UI.pulse) //is it changed?
			//	SaveToEE(SysData.NV_UI.pulse);
			//if (Existing.latch_state != SysData.NV_UI.latch_state) //is it changed?
			//	SaveToEE(SysData.NV_UI.latch_state);
			//-!- IK20251007 add storage of calibration factors
		}//end of limit mode true

		clearBit(Display_Info.butt_states, BUTTON_LIMIT_LONG_PRESS_BIT); // Display_Info.butt_states&= 0xFFF7;//clear long press of limit
	}//end of long press of limit button

	//Check Up Button
	if (Display_Info.buttons_hits & BUTTON_UP_INSTANT_PRESS_BIT)    // up button is pressed
		ProcessUPbutton(); // IK20250804 moved to a separate function
	if (Display_Info.butt_states & BUTTON_UP_SHORT_PRESS_BIT)    //Short press AND RELEASE of up button
	{
		// ProcessUPbutton(); // sets Bit (Display_Info.butt_states, BUTTON_UP_STILL_HELD_BIT)
		clearBit(Display_Info.butt_states, BUTTON_UP_SHORT_PRESS_BIT);	// IK20251111 allow to set accelerated increment delta. resets on release of button
	}//end short press of up button

	if (Display_Info.butt_states & BUTTON_UP_LONG_PRESS_BIT)			// happen after 3 sec hold
	{
		// ProcessUPbutton(); // sets Bit (Display_Info.butt_states, BUTTON_UP_STILL_HELD_BIT)
		clearBit(Display_Info.butt_states, BUTTON_UP_LONG_PRESS_BIT); // Display_Info.butt_states &= 0xFFDF;  //clear long press of UP
	}//end long press of up

	//Check Down Button
	if (Display_Info.buttons_hits & BUTTON_DOWN_INSTANT_PRESS_BIT)    // down button is pressed
		ProcessDOWNbutton(); // IK20250804 moved to a separate function
	if (Display_Info.butt_states & BUTTON_DOWN_SHORT_PRESS_BIT)    //Short press and RELEASE of Down button
	{
		//ProcessDOWNbutton(); // IK20250804 moved to a separate function
		//clearBit(Display_Info.butt_states, BUTTON_DOWN_STILL_HELD_BIT);	// IK20251111 allow to set accelerated decrement delta. resets on release of button
		clearBit(Display_Info.butt_states, BUTTON_DOWN_SHORT_PRESS_BIT); // Display_Info.butt_states &= 0xFFBF;  //clear short down press
	}//end short press of down button

	if (Display_Info.butt_states & BUTTON_DOWN_LONG_PRESS_BIT)			// Long press and RELEASE of down button
	{
		//ProcessDOWNbutton(); // IK20250804 moved to a separate function
		clearBit(Display_Info.butt_states, BUTTON_DOWN_LONG_PRESS_BIT);	//  Display_Info.butt_states &= 0xFF7F;  //clear long press of down
	}//end long press of down

	//Display_Info.Status housekeeping, handle pulse LED
	if (rt.pulse == FALSE) {											// set/clr bit in display status
		Display_PulseLED_OFF;	// clearBit(Display_Info.Status, DISP_LED_Pulse_ON_BIT); // Bit_2
		clearBit(Display_Info.Status, DISP_STATE_PulseON_BIT);			// clear Bit_6);
	}
	else //if (SysData.NV_UI.pulse == TRUE)
	{
		Display_PulseLED_ON;											// setBit(Display_Info.Status, DISP_LED_Pulse_ON_BIT);	// Bit_2
		setBit(Display_Info.Status, DISP_STATE_PulseON_BIT);			// set Bit_6;
	}

	//Check Reset Button
	if (Display_Info.butt_states & BUTTON_RESET_SHORT_PRESS_BIT)		//Short press of Reset button
	{
		if (limit_mode == TRUE)
		{ //Nothing done for short press of reset
		}
		clearBit(Display_Info.butt_states, BUTTON_RESET_SHORT_PRESS_BIT); //  Display_Info.butt_states &= 0xFEFF; // clear short press
	}//nothing done for short press of reset

	if (Display_Info.butt_states & BUTTON_RESET_LONG_PRESS_BIT)			// Long press of Reset button
	{
		Display_Info.alarm_status = 0;									// ext reset clear alarms
		latched_alarm_status = 0;										// and latched states
		clearBit(SysData.NV_UI.SavedStatusWord, Buzzer_ON_eq1_Bit);		// turn buzzer off
		clearBit(Display_Info.butt_states, BUTTON_RESET_LONG_PRESS_BIT);// Display_Info.butt_states &= 0xFDFF; // clear long press
		display_mode = VOLTS;											// go back to batt volts after clearing
	}//end long press
//End check Display_Info.butt_states code

//handle changing display items while in auto mode
	if ((timer.disp_var_change_ms == 0) && (Is_in_auto_mode == TRUE) && (latched_alarm_status == 0)) // Increment display item
	{
		display_mode = SelectNextMode(display_mode, next_AUTO_display_mode);
		/*
		if (display_mode < VOLTS) display_mode = VOLTS; //in auto mode limit range between VOLTS and PHASE_SHOW
		else  display_mode++; //increment display mode
		if (display_mode > PHASE_SHOW) display_mode = VOLTS;
		*/
		timer.disp_var_change_ms = 1000;                   //change again in 2 seconds
	}

	if ((display_mode == CAL2_4mA) || (display_mode == CAL6_0mA))
	{
		setBit(Display_Info.Status, DISP_STATE_LOmA_BIT);								// Display_Info.Status |= 0x01;	// set 4mA bit(0mA)
		clearBit(Display_Info.Status, DISP_STATE_HImA_BIT);								// Display_Info.Status &= 0xFD;	// clr 20 mA bit(1ma)
	}
	if ((display_mode == CAL3_20mA) || (display_mode == CAL7_1mA))
	{
		setBit(Display_Info.Status, DISP_STATE_HImA_BIT);  // Display_Info.Status |= 0x02;	// set 20mA bit
		clearBit(Display_Info.Status, DISP_STATE_LOmA_BIT); // Display_Info.Status &= 0xFE;	// clr 4 mA bit(1ma)
	}
	if ((display_mode != CAL2_4mA) && (display_mode != CAL3_20mA) &&
		(display_mode != CAL6_0mA) && (display_mode != CAL7_1mA)) //not doing cal
	{
		clearBit(Display_Info.Status, (DISP_STATE_LOmA_BIT | DISP_STATE_HImA_BIT));		// Display_Info.Status &= 0x7C; //clr both 4 and 20 ma & 0-1ma bit
	}

	if (display_mode == CAL_V_BAT)
		setBit(Display_Info.Status, DISP_STATE_VoltCal_BIT);							// Display_Info.Status |= 0x10; //set Volt cal bit
	else
		clearBit(Display_Info.Status, DISP_STATE_VoltCal_BIT);							// Display_Info.Status &= 0xEF; //clr volt cal bit

	// housekeeping
	if (timer.limit_mode_timeout_ms == LIMIT_SAFETY_INACTIVITY_TIMEOUT)					// after 150 seconds of no button press revert to manual mode
	{
		Display_Info.butt_states = BUTTON_LIMIT_LONG_PRESS_BIT;							// IK20251014 this will cause exit from limit mode as if someone pressed and held limit button for 3 seconds
		timer.limit_mode_timeout_ms = 0;
	}
	if (limit_mode == FALSE)
		timer.limit_mode_timeout_ms = 0;
}//END of Operation


#endif // #ifdef DISPLAY_MENU

//typedef struct  {
//	Schar mode;
//	float * NumericLEDshow;
//} DisplayMode;

float NoShow = -999.9f; // to indicate no value to show
/*********************************************************************/
float* NumLED_Display_Var[] =
{
	// index  //    enum name            // What info display shows if display_mode is set to this enum
// invalid index for array		EXIT_MENU = -2,
// invalid index for array					END_MENU = -1,
	// LIMT menu, used in LIMIT mode
	&NoShow,								// 0 0x00 */    LIMIT_START = 0,	 // "LMIT"
	&SysData.NV_UI.high_bat_threshold_V_f,	// 1 0x01 */    HI_BAT_THRESHOLD,	 // "HBAT"
	&SysData.NV_UI.low_bat_threshold_V_f,	// 2 0x02 */    LOW_BAT_THRESHOLD,	 // "LBAT"
	&SysData.NV_UI.plus_gf_threshold_V_f,	// 3 0x03 */    PLUS_GF_THRESHOLD,	 // "+GND"
	&SysData.NV_UI.minus_gf_threshold_V_f,	// 4 0x04 */    MINUS_GF_THRESHOLD,	 // "-GND"
	&SysData.NV_UI.ripple_V_threshold_mV_f,	// 5 0x05 */    RippleVOLT_TR_HOLD,	 // "RIPV"
	&SysData.NV_UI.ripple_I_threshold_mA_f,	// 6 0x06 */    RippleCURR_TR_HOLD,	 // "RIPI"
	&SysData.NV_UI.alarm_delay_sec_f,		// 7 0x07 */    TIME_DELAY_SET,		 // "TD  "
	&NoShow,								// 8 0x08 */    PHASE_STATE_SET,	 // "PH  "
	&NoShow,								// 9 0x09 */    PULSE_STATE_SET,	 // "PULS"
	&NoShow,								//10 0x0A */    BUZZER_STATE_SET,	 // "BUZZ"
	&NoShow,								//11 0x0B */    LATCHED_STATE_SET,	 // "LTCH"
	&SOFTWARE_VERSION,						//12 0x0C */    SHOW_FW_VER,		 // "SYS "

	// CAL menu, used in CURRENT CALIBRATION mode
	&NoShow,								//13 0x0D */    CAL1_SET_4_20_MODE,	 // "CAL1"
	&SysData.CurrentOut_I420.Y1_lowCalibrVal,		//14 0x0E */    CAL2_4mA,			 // "I LO"
	&SysData.CurrentOut_I420.Y2_highCalibrVal,		//15 0x0F */    CAL3_20mA,			 // "I HI"
	&NoShow,								//16 0x10 */    CAL5_SET_0_1_MODE,	 // "CAL5"
	&NoShow,								//17 0x11 */    CAL6_0mA,			 // "0 MA"
	&NoShow,								//18 0x12 */    CAL7_1mA,			 // "1 MA"
	&rt.OutData.measured.battery_voltage_f,					//19 0x13 */    CAL_V_BAT,			 // "VCAL"

	// SYS menu, Alarm Disabling/Enabling
	&NoShow,								//20 0x14 */    ALARM_AC,			 // "ALAC"
	&SysData.NV_UI.ripple_V_threshold_mV_f,	//21 0x15 */    ALARM_RIV,			 // "ALRV"
	&SysData.NV_UI.ripple_I_threshold_mA_f,	//22 0x16 */    ALARM_RII,			 // "ALRI"
	&NoShow,								//23 0x17 */    ALARM_HI_Z,			 // "ALHZ"
	&NoShow,								//24 0x18 */    SELECT_PROTOCOL,	 // "COMP"
	//&SysData.NV_UI.StartUpProtocol,		//24 0x18 */    SELECT_PROTOCOL,	 // "COMP"
	&Existing.meter_address,				//25 0x19 */    COMM_ADDR,			 // "ADDR"
	& Existing.baud_rate,					//26 0x1A */    COMM_BAUD,			 // "BAUD"
	&NoShow,								//27 0x1B */    AUTO_MAN_MODE,		 // "AUTO",

	//  IK20250724 do not change order of enums from VOLTS to PHASE_SHOW, they are used in front menu to change next value in manual mode
	&rt.OutData.measured.battery_voltage_f,					//28 0x1C */    VOLTS,				 // "BAT "
	&rt.OutData.measured.plus_gnd_volts_f,					//29 0x1D */    PLUS_GND_VOLTS,		 // "+BUS"
	&rt.OutData.measured.minus_gnd_volts_f,					//30 0x1E */    MINUS_GND_VOLTS,	 // "-BUS"
	&rt.OutData.measured.G_fault_voltage_f,					//31 0x1F */    GND_FAULT_VOLTS,	 // "GFV "
	&rt.OutData.measured.ripple_mV_f,						//32 0x20 */    RIPPLE_VOLTS,		 // "RVV "
	&rt.OutData.measured.ripple_mA_f,						//33 0x21 */    RIPPLE_CURRENT,		 // "RIV "
	& SysData.NV_UI.alarm_delay_sec_f,		//34 0x22 */    TIME_DELAY_SHOW,	 // "TD  "
	&NoShow,								//35 0x23 */    PHASE_SHOW,			 // "PH  "

	// IK20250724 do not change order of items, they are used in front menu to change next value in manual mode
	&SOFTWARE_VERSION,						//36 0x24 */    INIT,				 // "ARGA"
	&NoShow,								//37 0x25 */    OUTPUT_UPPER_STRING, // "OUTU"
	&NoShow,								//38 0x26 */    OUTPUT_LOWER_STRING, // "OUTL"

};

char* ReplaceDoubleII(char* str)
// Replace possible double 'I','I' or '1','1' with a char '`' 0x60, 96D. Display board will show II on one 7-seg numeric or 14-seg ASCII LED
{
	char* charPtr = str;
	uint8 indx = 0;
	while (charPtr[indx] != 0)
	{
		char ch1 = charPtr[indx];
		char ch2 = charPtr[indx + 1];
		if ((ch1 == 'I' || ch1 == '1') && (ch2 == 'I' || ch2 == '1'))
		{
			charPtr[indx] = 0x60; // '`' char to indicate double I
			//shift rest of string to the left by one
			while (charPtr[indx + 1] != 0)
			{
				charPtr[indx + 1] = charPtr[indx + 2];
				indx++;
			}
			//charPtr[indx] = 0; // terminate string
			indx = 0; // restart scanning from beginning
		}
		indx++;
	}
	return str;
}

// works in main loop
void DisplayPrepare(void)
{
	float display_value = rt.OutData.measured.battery_voltage_f;
	char FL * p_InfoStr = "  "; //-!- IK20250710 replace with FiveSpaces ??
	// IK20251029 IAR does not allow to change format string if strings are in flash, cannot do: char frmt[6] = "%6.1f"; then frmt[3]='0' to get frmt = "%6.0f";
	// 	char format_float = true; // "%6.1f";
#ifdef DISPLAY_MENU

	rt.OutData.measured.plus_gnd_volts_f = rt.OutData.measured.battery_voltage_f + rt.OutData.measured.minus_gnd_volts_f; //+gnd volts = bat - negative_gnd_V

	display_value = *NumLED_Display_Var[display_mode];	//Get value to display

	if (In_calibr_menu_mode == TRUE)
	{
		if ((display_mode > CAL1_SET_4_20_MODE) && (display_mode <= CAL7_1mA))
		{
			uint8 current;
			if ((display_mode == CAL6_0mA) || (display_mode == CAL7_1mA)) {
				setBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
			}
			else if ((display_mode == CAL2_4mA) || (display_mode == CAL3_20mA)) {
				clearBit(SysData.NV_UI.SavedStatusWord, CurOut_I420_eq0_I01_eq1_Bit);
			}
			if (SysData.NV_UI.SavedStatusWord & CurOut_I420_eq0_I01_eq1_Bit) // I 0-1 mA
			{
				if (display_mode == CAL6_0mA) current = 0;
				if (display_mode == CAL7_1mA) current = 1;
			}
			else
			{
				if (display_mode == CAL2_4mA) current = 4;
				if (display_mode == CAL3_20mA) current = 20;
			}
			sprintf(Display_Info.DigitalStr, "Cur%2d", current); // show "current XX mA" on upper display (Cur20)
		}
		else if (display_mode == CAL_V_BAT) {
			sprintf(Display_Info.DigitalStr, "%6.1f", rt.OutData.measured.battery_voltage_f); // show "battery voltage" on upper display
		}
		else
			Display_Info.DigitalStr[0] = 0; // empty string
	}
	else if (display_mode == SELECT_PROTOCOL) {
		CopyConstString("PrtCL", Display_Info.DigitalStr);
		//if (SysData.NV_UI.StartUpProtocol == SETUP)
		//	CopyConstString("SEtuP", Display_Info.DigitalStr);
		//else if (SysData.NV_UI.StartUpProtocol == DNP3)
		//	CopyConstString("DnP-3", Display_Info.DigitalStr);
		//else if (SysData.NV_UI.StartUpProtocol == MODBUS)
		//	CopyConstString("Modbu", Display_Info.DigitalStr);
		//else if (SysData.NV_UI.StartUpProtocol == ASCII_CMDS)
		//	CopyConstString("ASCII", Display_Info.DigitalStr);
		//else
		//	CopyConstString("SELCT", Display_Info.DigitalStr);
	}
	else if (display_value > NoShow) // "do not show" value is -999.9f, if greater - show value on numeral display
	{
		if ((display_mode == COMM_ADDR) || (display_mode == COMM_BAUD)) // takes less FLASH: 53467, 56 bytes less vs comparing pointers.
			//if((NumLED_Display_Var[display_mode] == &Existing.meter_address) ||(NumLED_Display_Var[display_mode] == &Existing.baud_rate))  // takes more FLASH: 53523
			sprintf(Display_Info.DigitalStr, "%5.0f", display_value); // show float as integer, 5 digit size
		else{
			if ((Display_Info.InfoStr[0] == 'A') && (Display_Info.InfoStr[1] == 'R')) // ARGA initial message
			{
				uint8 ProtocolIndex = SysData.NV_UI.StartUpProtocol;
#ifdef PROT_NAMES_3CHARS
				char BriefProtocolNames[4][3] = {
					'I','n','i',
					'D','n','P',
					'M','O','d',
					'A','S','C',
				char ch1 = BriefProtocolNames[ProtocolIndex][0];
				char ch2 = BriefProtocolNames[ProtocolIndex][1];
				char ch3 = BriefProtocolNames[ProtocolIndex][2];
				sprintf(Display_Info.DigitalStr,"%2.1f%c%c%c",display_value,ch1,ch2, ch3);
#else
				char BriefProtocolNames[4][2] = {
					'I','n',
					'D','n',
					'M','b',
					'A','S',
				};
				char ch1 = BriefProtocolNames[ProtocolIndex][0];
				char ch2 = BriefProtocolNames[ProtocolIndex][1];
				sprintf(Display_Info.DigitalStr,"%2.1f %c%c",display_value,ch1,ch2);
#endif
			}
			else {
				sprintf(Display_Info.DigitalStr, "%6.1f", display_value); // show float with one number after dot which will be packed into the second to right 7-seg LED with dot.
			}
		}
		if (display_mode == COMM_BAUD)
			ReplaceDoubleII(Display_Info.DigitalStr);// Replace possible double '1','1' with a char '`' 0x60, 96D. Display board will show 11 on the leftmost 7-seg numeric LED
	}
	else
		Display_Info.DigitalStr[0] = 0; // empty string
#endif // #ifdef DISPLAY_MENU

	Write_Numeric_Display(Display_Info.DigitalStr);

#ifdef DISPLAY_MENU

	//select what to display on info LED
	// assume there is no need to blink; if needed, set it in the mode below
	if (limit_mode == FALSE) rt.InfoLED_blink_eq1 = FALSE; // but not in limit mode

	if ((display_mode == PHASE_STATE_SET) || (display_mode == PHASE_SHOW) || (display_mode == AUTO_MAN_MODE))
	{
		if(SysData.NV_UI.SavedStatusWord & SinglePhase_eq0_3ph_eq1_Bit)
			p_InfoStr = ("3PH ");
		else
			p_InfoStr = ("1PH ");
	}
	else if (display_mode == PULSE_STATE_SET)
	{
		if (rt.pulse == ON) // should NOT be saved in NV memory, it is a test signal. defined in Real Time structure
			p_InfoStr = ("P_ON");
		else
			p_InfoStr = ("P_OF");
	}
	else if (display_mode == BUZZER_STATE_SET)
	{
		if (SysData.NV_UI.SavedStatusWord & Buzzer_ON_eq1_Bit)
			p_InfoStr = ("BZ Y");					// IK20240130 was "BON "
		else
			p_InfoStr = ("BZ N");					// IK20240130 was "BOFF"
	}
	else if (display_mode == ALARM_AC)
	{
		//rt.InfoLED_blink_eq1 = FALSE;
		if ((SysData.NV_UI.disabled_alarms & Alarm_AC_Loss_Bit) == 0)	// AC alarm enabled
			p_InfoStr = ("AC Y");
		else									// AC Alarm not enabled
			p_InfoStr = ("AC N");
	}
	else if (display_mode == ALARM_RIV)
	{
		//rt.InfoLED_blink_eq1 = FALSE;
		if ((SysData.NV_UI.disabled_alarms & Alarm_Ripple_Voltage_Bit) == 0)	// RIV alarm enabled by CLEARING bit
			p_InfoStr = ("RV Y");
		else									// RIV Alarm not enabled
			p_InfoStr = ("RV N");
	}

	else if (display_mode == ALARM_RII)
	{
		//rt.InfoLED_blink_eq1 = FALSE;
		if ((SysData.NV_UI.disabled_alarms & Alarm_Ripple_Current_Bit) == 0)	// Ripple Current alarm enabled
			p_InfoStr = ("RI Y");
		else									// Ripple Current Alarm not enabled
			p_InfoStr = ("RI N");
	}
	else if (display_mode == ALARM_HI_Z)
	{
		//rt.InfoLED_blink_eq1 = FALSE;
		if ((SysData.NV_UI.disabled_alarms & Alarm_High_Impedance_Bit) == 0)	// High Z alarm enabled
			p_InfoStr = ("HZ Y");
		else									// High Z Alarm not enabled
			p_InfoStr = ("HZ N");
	}
	else if (display_mode == LATCHED_STATE_SET)
	{
		if (SysData.NV_UI.SavedStatusWord & Latch_ON_eq1_Bit)			// if latched (persistent) alarms
			p_InfoStr = ("L ON");
		else
			p_InfoStr = ("LOFF");
	}
	//IK20251007 because blink is disabled at the beginning of the check, use 'else if' for cases where blink is needed;
	// otherwise, set p_InfoStr at the end in 'else' to the mode_strings[display_mode]
	else if (display_mode == CAL1_SET_4_20_MODE)
	{
		rt.InfoLED_blink_eq1 = TRUE;
		p_InfoStr = mode_strings[CAL1_SET_4_20_MODE]; // ("CAL ");
	}
	//else if (display_mode == CAL2_4mA)
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[CAL2_4mA]; // ("ILO ");
	//}
	//else if (display_mode == CAL3_20mA)
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[CAL3_20mA]; // ("IHI ");
	//}
	else if (display_mode == CAL5_SET_0_1_MODE)
	{
		rt.InfoLED_blink_eq1 = TRUE;
		p_InfoStr = mode_strings[CAL5_SET_0_1_MODE]; // ("CAL2");
	}
	//else if (display_mode == CAL6_0mA)				// No longer used remove for more memory; instead, uses "I LO"
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[CAL6_0mA]; // ("0 MA");
	//}
	//else if (display_mode == CAL7_1mA)				// No longer used remove for more memory; instead, uses "I HI"
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[CAL7_1mA]; // ("1 MA");
	//}
	//else if (display_mode == CAL_V_BAT)
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[CAL_V_BAT]; // ("VCAL");
	//}
	else if (display_mode == SHOW_FW_VER)			// system setup
	{
		rt.InfoLED_blink_eq1 = TRUE;
		p_InfoStr = mode_strings[SHOW_FW_VER]; // ("SYS ");
	}
	//else if (display_mode == COMM_ADDR)
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[COMM_ADDR]; // ("ADDR");
	//}
	//else if (display_mode == COMM_BAUD)
	//{
	//	// set above rt.InfoLED_blink_eq1 = FALSE;
	//	p_InfoStr = mode_strings[COMM_BAUD]; // ("BAUD");
	//}
	else if (display_mode == SELECT_PROTOCOL) {
		{
			if (SysData.NV_UI.StartUpProtocol == DNP3) {
				p_InfoStr = ("DNP3");
				Existing.baud_rate = Baud_19200;
				BaudRateIndex = Baud_19200_i;
			}
			else if (SysData.NV_UI.StartUpProtocol == MODBUS) {
				p_InfoStr = ("MBUS");
				Existing.baud_rate = Baud_19200;
				BaudRateIndex = Baud_19200_i;
			}
			else if (SysData.NV_UI.StartUpProtocol == ASCII_CMDS) {
				p_InfoStr = ("ASC`");
				Existing.baud_rate = Baud_115200;
				BaudRateIndex = Baud_115200_i;			// IK20250826 set to 115200 baud, for quicker screen update
			}
			else // default to SETUP
			// if (SysData.NV_UI.StartUpProtocol == SETUP)
			{
				SysData.NV_UI.StartUpProtocol = SETUP;
				p_InfoStr = ("INIT");
				timer.TWI_lockup = 13000;						// set twi lockup tmr to 13 sec
				timer.start_up_ms = 6000;						// setup SysData.NV_UI.StartUpProtocol for 6 seconds
				Existing.baud_rate = Baud_9600;
				BaudRateIndex = Baud_9600_i;
			}
		}
		if (rt.operating_protocol != SysData.NV_UI.StartUpProtocol) {
#ifdef PC
			PutStr(p_InfoStr); SendCrLf();		// output to console == 'serial port' to see the string which is going to Info display
#endif
			rt.operating_protocol = SysData.NV_UI.StartUpProtocol;// IK20251217 in main(), the difference will be detected and applied
			// IK20251210 check if baud rate need to be changed - then do it; this happens in main loop.
			// otherwise writing to UBBR0 disrupts UART work
			rt.UBRR0_setting = Calculate_USART_UBRRregister((Uint32)Existing.baud_rate); // IK20251217 in main(), the difference will be detected and applied
			rt.HostRxBuffPtr = rt.EchoRxBuffPtr = 0;	//IK20251219 reset buffer pointer
			msg_status = MSG_DONE;	// and status of message -> anew
		}
	}
	else
		p_InfoStr = mode_strings[display_mode];
	/*
	else if (display_mode == INIT)
	{
		p_InfoStr = mode_strings[INIT]; // ("ARGA");
	}
	if (display_mode == VOLTS)
	{
		p_InfoStr = mode_strings[VOLTS]; // ("BAT ");
	}
	if (display_mode == PLUS_GND_VOLTS)
	{
		p_InfoStr = mode_strings[PLUS_GND_VOLTS]; // ("+BUS");
	}
	if (display_mode == MINUS_GND_VOLTS)
	{
		p_InfoStr = mode_strings[MINUS_GND_VOLTS]; // ("-BUS");
	}
	if (display_mode == GND_FAULT_VOLTS)
	{
		p_InfoStr = mode_strings[GND_FAULT_VOLTS]; // ("GFV ");
	}
	if (display_mode == RIPPLE_VOLTS)
	{
		p_InfoStr = mode_strings[RIPPLE_VOLTS]; // ("RVV ");
	}
	if (display_mode == RIPPLE_CURRENT)
	{
		p_InfoStr = mode_strings[RIPPLE_CURRENT]; // ("RIV ");
	}
	if ((display_mode == TIME_DELAY_SET) || (display_mode == TIME_DELAY_SHOW))
	{
		p_InfoStr = mode_strings[TIME_DELAY_SET]; // ("TD  ");
	}

	if ((display_mode == LIMIT_START) || (display_mode == PRE_LIMIT_START))
	{
		p_InfoStr = mode_strings[LIMIT_START]; // ("LMIT");
	}
	if (display_mode == HI_BAT_THRESHOLD)
	{
		p_InfoStr = mode_strings[HI_BAT_THRESHOLD]; // ("HBAT");
	}
	if (display_mode == LOW_BAT_THRESHOLD)
	{
		p_InfoStr = mode_strings[LOW_BAT_THRESHOLD]; // ("LBAT");
	}
	if (display_mode == PLUS_GF_THRESHOLD)
	{
		p_InfoStr = mode_strings[PLUS_GF_THRESHOLD]; // ("+GND");
	}
	if (display_mode == MINUS_GF_THRESHOLD)
	{
		p_InfoStr = mode_strings[MINUS_GF_THRESHOLD]; // ("-GND");
	}
	if (display_mode == RippleVOLT_TR_HOLD)
	{
		p_InfoStr = mode_strings[RippleVOLT_TR_HOLD]; // ("RVV ");
	}
	if (display_mode == RippleCURR_TR_HOLD)
	{
		p_InfoStr = mode_strings[RippleCURR_TR_HOLD]; // ("RIV ");
	}
*/
if (limit_mode != TRUE) // not in limit mode, normal operation
	{
		//Set Alarm displays if any
		if (Display_Info.alarm_status)
			rt.InfoLED_blink_eq1 = TRUE;
		else
			rt.InfoLED_blink_eq1 = FALSE;			// turn off flashing after fault is cleared

		// Arrange alarm checks so highest priority alarm is checked first, then next priority, etc
		if (Display_Info.alarm_status & Alarm_MinusGND_FAULT_Bit)				//== 0x08) // Highest priority alarm
		{
			p_InfoStr = mode_strings[MINUS_GF_THRESHOLD];			// ("-GND");
		}
		else if (Display_Info.alarm_status & Alarm_PlusGND_FAULT_Bit)			// == 0x04)
		{
			p_InfoStr = mode_strings[PLUS_GF_THRESHOLD];			// ("+GND");
		}
		else if (Display_Info.alarm_status & Alarm_BatVoltageLOW_Bit)			// == 0x02)
		{
			p_InfoStr = mode_strings[LOW_BAT_THRESHOLD];			// ("LBAT");
		}
		else if (Display_Info.alarm_status & Alarm_BatVoltageHIGH_Bit)			// == 0x01)
		{
			p_InfoStr = mode_strings[HI_BAT_THRESHOLD];				// ("HBAT");
		}
		else if (Display_Info.alarm_status & Alarm_High_Impedance_Bit)			// == 0x80)
		{
			p_InfoStr = mode_strings[ALARM_HI_Z];					// ("HI Z");
		}
		else if (Display_Info.alarm_status & Alarm_AC_Loss_Bit)				// == 0x40)
		{
			p_InfoStr = mode_strings[ALARM_AC];						// ("AC F");
		}
		else if (Display_Info.alarm_status & Alarm_Ripple_Voltage_Bit)			// == 0x10)
		{
			p_InfoStr = mode_strings[RippleVOLT_TR_HOLD];			// ("RVV ");
		}
		else if (Display_Info.alarm_status & Alarm_Ripple_Current_Bit)			// == 0x20)//Lowest Priority Alarm
		{
			p_InfoStr = mode_strings[RippleCURR_TR_HOLD];			// ("RIV ");
		}
	}
	Write_ASCII_Display_FlashString(p_InfoStr);
#endif // #ifdef DISPLAY_MENU
	Display_Info.DisplayNeedsUpdateFlag = CLR;
}

uint8 IsNumeric(const char* str) {
	char* endptr;
	strtod(str, &endptr);
	return (*str != '\0' && *endptr == '\0');
}