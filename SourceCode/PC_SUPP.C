/*****************************************************************************
*
* Filename: PC_Supp.c
*
* $Source:  $
* Last Saved Revision Number $Revision:  $
* Last Saved Revision $Date:  $
* Last Saved Revision $Author: igork $
*
* Original Author:  Igor Kordunsky
* Date: 05/31/2023
*
* File Description: support file in order to be able to compile project on PC
*
*****************************************************************************/

#pragma once

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>
// C RunTime Header Files
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <process.h>
#include <commctrl.h> // Needed for SetWindowSubclass
#include <stdint.h> // for uint64_t

#include <math.h>

#include "Globals.h"
#include "resource.h"

extern BOOL PC_LOOP_INITIALIZED; //let main thread initialize / substitute variables
extern BOOL IsFileOutput;
extern char* g_pszOutput;
extern FILE* g_fhOutput;
extern FILE* OpenOutputFile(char* szFile);
extern volatile Uchar UDR0;

// Link with comctl32.lib: Project Properties ? Linker ? Input ? Additional Dependencies -> Add: comctl32.lib
// or use #pragma
#pragma comment(lib, "comctl32.lib")

int PUTCHAR(int c) //returning same char
{
	char out_c[2];
	out_c[0] = c;
	out_c[1] = 0;
	if (IsFileOutput)
	{
		if (g_fhOutput != NULL) fprintf(g_fhOutput, "%s", out_c);
	} // set flag only for PC to write to file
	printf("%c", out_c[0]);
	return c;
}


//void asm(char* strptr)
//{
//
//}
#define inversion 1
#define noninv 0

void gotoxy(int x, int y);
void clrscr(void);
extern void TIMER2_COMPA_interrupt(void);
// in PC these files are separate source files

#ifdef __cplusplus
extern "C" {
#endif
int __cdecl putch(int);
int __cdecl getch(void);
int __cdecl cputs(const char *);
int kbhit(void);
#ifdef __cplusplus
}
#endif
int FrontLED = 0;

// defined in "globals.h"
//#define CaRet		0x0d
//#define LineFeed	0x0a // LF


#define	D_AT(x,y)	(0x80 + ((y)%2)*0x40 + ((y)/2)*0x14 + (x)) /* packs x y position into char*/

#define	D_CLEAR		0x01
#define	D_HOME		0x02
#define D_INIT		0x38
#define D_ON		0x0C
#define D_OFF		0x08
#define D_CURSON	0x0E
#define D_CURSOFF	0x0C

void  D_PutChar(unsigned char c);
void  D_PutCtrl(unsigned char c);
unsigned char D_ReadChar(void);
unsigned char D_ReadCtrl(void);
void  D_InitLCD(void);

extern char TestBuffer[80];

//extern char ScreenBuffer[80];

extern char ScreenPos, ScreenCursor;
MSG Msg;
BOOL EndControlLoop = FALSE;
char			ScreenPos;
char			ScreenCursor;
volatile LONG LCD_Changed = 0;  //flag to update window
int LCDx[2] = { 0,0 }; //tracks 1st and 2nd line current position, depends on LCDy
int LCDy = 0; //global for windows dialog
#define LCD_BUF_LEN     21 //string  of 20 chars + 0x00 terminator
#define LCD_BUF_DEPTH   2
unsigned char LCD_buffer[LCD_BUF_DEPTH][LCD_BUF_LEN * 2]; //LCD_buffer[LCDy][LCDx]//actually need only 20, 40 for sprintf - it can create long strings
/* In order to address it like string, X index MUST be the LAST one!
one more line and row, not using index 0 */
//int LCD_line_number = 1;

// IK20251001
#define MCU_CYCLE_NS (1000000000ULL / MASTER_CLOCK)  // ~62.5 ns per cycle
// In simulation, we cannot realistically wait 62.5 ns,
// so we scale: 1 NOP = 100 us delay
#define SIM_NOP_DELAY_US    100ULL

static  uint64_t now_us(void) {
	LARGE_INTEGER freq, counter;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&counter);
	return (uint64_t)(counter.QuadPart * 1000000ULL / freq.QuadPart);
}

//static inline
void delay_us(unsigned long long us)
{
	uint64_t start = now_us();
	while ((now_us() - start) < us) {
		SwitchToThread();   // better than Sleep(0);   // yield, keep CPU usage low
	}
}

void RealTimeCode(void)
{
	// Simulate 5 × 1ms ticks to match embedded timing
	//for (int i = 0; i < 5; i++)
	{
		TIMER2_COMPA_interrupt(); // decrements counters
	}
}

//empty functions for PC simulation
int dummy_function(void)
{
	delay_us(SIM_NOP_DELAY_US);// Sleep(1);
	return 0;
}

//void _NOP(void) {};
void __watchdog_reset(void) {};
void __no_operation(void) { delay_us(SIM_NOP_DELAY_US); };
//void __disable_interrupt(void) {};
//void __enable_interrupt(void) {};
//void _WDR(void) {};
void _CLI(void) {};
void _SEI(void) {};

/****************************************************************
 function chrrdy
	returns 1 if character waiting at UART, 0 otherwise
*****************************************************************/
int chrrdy(void)
{
	int t_int; //allow more cycles in simulation
	for (t_int = 0; t_int < 10; t_int++)
	{
		//		RealTimeCode();
	}
	delay_us(SIM_NOP_DELAY_US);// Sleep(1);
	t_int = kbhit();
	if (t_int != 0) {
		//setBit(msg_status, MSG_STARTED);
		setBit(rt.Host, CharAvailableFlag);
	}
	return t_int;
}

// IK20250710  - not used
//int getch_pc(void)
//{
//	while (!chrrdy())
//	{
//		CheckExecuteFrontPanelCmd(); // System - specific code INSIDE
//		//	PC_emulator();		// calculate emulation stimula
//		//	IRQ_Handler();		// call control interrupt for emulation
//	};
//	return getch();
//}


/****************************************************************
*
*	char * convert_to_binary(long b_data, int invert)
*  invert=0 - no inversion
*  invert=1 - inversion
*  invert=12 - 12 bit conversion  invert=13 - 12 bit inverted conversion
*  invert=16 - 16 bit conversion  invert=17 - 16 bit inverted conversion
*  invert=24 - 24 bit conversion  invert=25 - 24 bit inverted conversion
*  invert=32 - 32 bit conversion  invert=33 - 32 bit inverted conversion
*
***************************************************************** /
char * convert_to_binary(long b_data, int invert)
{
	long b,d,l;
	char buffer[80];

	d=b_data;

	//if ((d>2047) || (d<-2048)) k=15; // convert to unsigned 16 bits integer

	if (invert&1)
	{
		b=(d^0xffffffff);
		invert--;
	}
	else b=d;
	if(invert==0) invert=16; // minimum conversion =16 bit
	for (l=0; l<=invert-1;l++)
	{
		buffer[l]= (b & 1) + 48;
		b>>=1;
	}
	buffer[l]='\0';
	l=0;
	for(b=invert-1;b>=0;b--)  //convert string backwards
	{
		buff1[l++] = buffer[b];
		buff1[l++]=32;
		switch (b)
		{
			case 4:
			case 8:
			case 12:
			case 16:
			case 20:
			case 24:
			case 28:
			buff1[l++]=32;
			break;
		}
	}
	if(invert==16)
	{
		if ((d>-2049) && (d<2048)) // 12-bits, set first 4 bits to " "
		{
			buff1[0]=buff1[2]=buff1[4]=buff1[6]=' ';
		}
	}
	buff1[l++]='\0';
	return buff1;
}

/********* GUI FRONT PANEL ********/
HBRUSH hbr_LED_off;
HBRUSH hbr_LED_Red;
HBRUSH hbr_LED_Green;
HBRUSH hbr_LED_Yellow;
HBRUSH* hbr_LEDcolor_ptr[number_of_LEDs];

//#define ColorOFF		0
//#define ColorRED		1
//#define ColorGREEN	2
//#define ColorYELLOW	3 // Red+green
//  colors array  pointers indexes      ColorOFF=0    ColorRED=1    ColorGREEN=2    ColorYELLOW=3
HBRUSH* hbr_Color_ptr[Total_Colors] = { &hbr_LED_off, &hbr_LED_Red, &hbr_LED_Green, &hbr_LED_Yellow };

#define FrontPanel_WIDTH	350	//nWidth The width, in device units, of the window.
#define FrontPanel_HEIGHT	370	// nHeight
#define LED_size      14

#define AUTO_LED_x1   70
#define AUTO_LED_y1   60
#define AUTO_LED_x2   (AUTO_LED_x1+LED_size)
#define AUTO_LED_y2   (AUTO_LED_y1+LED_size)
#define ALARM_LED_x1  228
#define ALARM_LED_y1  60
#define ALARM_LED_x2  (ALARM_LED_x1+LED_size)
#define ALARM_LED_y2  (ALARM_LED_y1+LED_size)
#define TX_RX_LED_x1  222
#define TX_RX_LED_y1  170
#define TX_RX_LED_x2  (TX_RX_LED_x1+LED_size)
#define TX_RX_LED_y2  (TX_RX_LED_y1+LED_size)
#define PULSE_LED_x1  80
#define PULSE_LED_y1  170
#define PULSE_LED_x2  (PULSE_LED_x1+LED_size)
#define PULSE_LED_y2  (PULSE_LED_y1+LED_size)


RECT rctAutoLED = { AUTO_LED_x1, AUTO_LED_y1, AUTO_LED_x2, AUTO_LED_y2 };		// create Auto LED
RECT rctAlarmLED = { ALARM_LED_x1, ALARM_LED_y1, ALARM_LED_x2, ALARM_LED_y2 };	// create Alarm LED
RECT rctRxTxLED = { TX_RX_LED_x1, TX_RX_LED_y1, TX_RX_LED_x2, TX_RX_LED_y2 };	// create Tx/Rx LED
RECT rctPulseLED = { PULSE_LED_x1, PULSE_LED_y1, PULSE_LED_x2, PULSE_LED_y2 };	// create Pulse LED
//initialization: assign them in case WM_INITDIALOG:
// to array of rectangles rect_LED
RECT * rect_LED[number_of_LEDs]; // pointers to static rectangles


double UserValue[NUM_INPUT_FIELDS] = { 0 };   // Parsed numeric values
int editControlIDs[NUM_INPUT_FIELDS] = {
	IDC_V_BATT, IDC_V_GND, IDC_V_RIP, IDC_I_RIP
};

typedef enum {
	STATE_UNTOUCHED,
	STATE_VALID,
	STATE_INVALID,
	STATE_CONFIRMED
} FieldState;

// for partially entering (char-by-char by user) strings
typedef enum {
	NUM_INVALID,
	NUM_PARTIAL,   // e.g., "1.5e", "3e-", "7e+"
	NUM_VALID
} NumericState;

FieldState inputState[NUM_INPUT_FIELDS] = { STATE_UNTOUCHED };
static BOOL suppressChangeEvent = FALSE;

COLORREF colorValid = RGB(255, 255, 255);   // White
COLORREF colorInvalid = RGB(255, 255, 0);   // Yellow
COLORREF colorConfirmed = RGB(200, 255, 200); // light green
COLORREF colorPartial = RGB(220, 220, 220); // light grey

HBRUSH hbrValid = NULL;
HBRUSH hbrInvalid = NULL;
HBRUSH hbrPartial = NULL; //CreateSolidBrush(RGB(220, 220, 220));

WNDPROC OldEditProc[NUM_INPUT_FIELDS] = { 0 };
/*************************/
int new_thread( void );             // Thread 1: main
void ShutDown( void );              // Program shutdown
void WriteTitle( int ThreadNum );   // Display title bar information
void Set_Thread_Name(const char* threadName);

HANDLE  hConsoleOut;                // Handle to the console
HANDLE  hRunMutex;                  // "Keep Running" mutex
HANDLE  hScreenMutex;               // "Screen update" mutex
int     ThreadNr = 0;                // Number of threads started
CONSOLE_SCREEN_BUFFER_INFO csbiInfo; // Console information

#include "resource.h"

HWND g_hToolbar = NULL;
volatile int win_btn_lines = NO_PRESSED_BUTTONS; // all zeroes. logic 1 == button pressed; physically, ATMEL port line reads zero when button is pressed
volatile int win_btn_pressed = CLR;
static HFONT hFontLarge = NULL;

#define LED_UPDATE_TIMER_ID  1
#define LED_UPDATE_INTERVAL_MS 50

#define WM_UPDATE_LCD (WM_USER + 100)

//#define BIT_BUT_LEFT    (BUTTON_AUTO_INSTANT_PRESS_BIT >> 8)	// Bit_0
//#define BIT_BUT_RIGHT   (BUTTON_LIMIT_INSTANT_PRESS_BIT >> 8)	// Bit_1
//#define BIT_BUT_UP      (BUTTON_UP_INSTANT_PRESS_BIT >> 8)	// Bit_2
//#define BIT_BUT_DOWN    (BUTTON_DOWN_INSTANT_PRESS_BIT >> 8)	// Bit_3

//#define BUTTON_BIT_AUTO    BIT_BUT_LEFT // Bit_0
//#define BUTTON_BIT_LIMIT   BIT_BUT_RIGHT// Bit_1
//#define BUTTON_BIT_UP      BIT_BUT_UP   // Bit_2
//#define BUTTON_BIT_DOWN    BIT_BUT_DOWN // Bit_3

/***********************************************************************/
// this PC_SimulationApplication runs in separate thread on timer-driven events
/***********************************************************************/
LARGE_INTEGER ticksPerSecond;
LARGE_INTEGER tick;   // A point in time
BOOL ten_times_faster;        // receives check box status
LONGLONG tickDiff = 0;
HWND LedParent;
#define RMS_averaging_time 1.0 // sec
double RMS_avg_old_value_factor = (RMS_averaging_time * LOOP_SAMPLE_FREQUENCY - 1) / (RMS_averaging_time * LOOP_SAMPLE_FREQUENCY);
double RMS_added_value_factor = 1 / (RMS_averaging_time * LOOP_SAMPLE_FREQUENCY);
double AvgLoopTime = 0;
BOOL AvgLoopTimeIsReset = false;
//  LARGE_INTEGER time;   // For converting tick into real time
void PC_SimulationApplication(void* pArg)
{
	(void)pArg;  // avoid unused parameter warning
	long ticks_per_loop = 0, prev_ms_cntr = 0, this_ms_cntr = 0;
	double time_scale;
	LARGE_INTEGER prev_tick = { 0,0 };   // A point in time
	Set_Thread_Name("RT_sim");

	// get the high resolution counter's accuracy
	QueryPerformanceFrequency(&ticksPerSecond);
	//	ticksPerSecond.LowPart = 0;
	if (ticksPerSecond.LowPart == 0) //no high-resolution counter, use system ticks
		ticks_per_loop = (long)(1000.0f / LOOP_SAMPLE_FREQUENCY + 1);
	else // usually 10000000 = 10 millions 10 MHz
		ticks_per_loop = (long)(ticksPerSecond.QuadPart / LOOP_RATE);
	// what time is it?
	// convert the tick number into the number of seconds
	// since the system was started...
	//time.QuadPart = tick.QuadPart/ticksPerSecond.QuadPart;
	prev_ms_cntr = GetTickCount64();
	while (EndControlLoop != TRUE)
	{
		ten_times_faster = IsDlgButtonChecked(g_hToolbar, IDC_CHECK_10X);
		if (ten_times_faster == TRUE)
		{
			if (AvgLoopTimeIsReset == false)
			{
				AvgLoopTime = 0;
				AvgLoopTimeIsReset = true;
			}
			time_scale = 0.1;
		}
		else {
			time_scale = 1.0;
			AvgLoopTimeIsReset = false;
		}
		if (ticksPerSecond.LowPart != 0)
		{
			QueryPerformanceCounter(&tick);
			tickDiff = tick.QuadPart - prev_tick.QuadPart;
			if (tickDiff > ticks_per_loop)
			{
				Sim_counter++;
			}
			if (tickDiff > ticks_per_loop * time_scale)
			{
				AvgLoopTime = (AvgLoopTime * RMS_avg_old_value_factor) + (tickDiff * RMS_added_value_factor); // LPF'd loopTime in ticks
				prev_tick.QuadPart = tick.QuadPart;
				//	if( PC_LOOP_INITIALIZED == TRUE) //let main thread initialize / substitute variables
					RealTimeCode();//Sleep(1): 60 frames per second only??? Sleep(0): 487000 frames per second ???
				//	if((GetTickCount64()-prev_ms) >= 1000)
				//		MessageBox(NULL, "1000ms", "test", MB_OK);
				//	Sleep(1); //release time slot
			}
			// Sleep(#): A value of zero causes the thread to relinquish the remainder of its time slice to any other thread that is ready to run.
			//	If there are no other threads ready to run, the function returns immediately, and the thread continues execution.
			else 	// if Sleep(1) GREATELY reduced Real Time because of Windows internal tick can be 16 ms, so it could wait between 1 and 15 ms  !!!  performance release time slot
				Sleep(0);
		}
		else //no high-resolution counter, use system ticks
		{
			this_ms_cntr = (long)GetTickCount64(); //current ms value
			if ((this_ms_cntr - prev_ms_cntr) > ticks_per_loop)//"this_ms_cntr - prev_ms_cntr"
			{
				prev_ms_cntr = this_ms_cntr; //current ms value
				//	if( PC_LOOP_INITIALIZED == TRUE) //let main thread initialize / substitute variables
				RealTimeCode();
				delay_us(SIM_NOP_DELAY_US);// Sleep(1); //release time slot
			}
			else Sleep(0); //release time slot
		}
	}
	//end_control_thrend:
	//	return Msg.wParam;
		// Tell thread to die and record its death.
	ReleaseMutex(hRunMutex);
}

// detecting button press and duration:

// IK20250715 copilot suggested function
//void UpdateAllButtons(uint8* pressed_states, volatile uint16* butt_states) {
//	for (int i = 0; i < NUM_BUTTONS; ++i) {
//		ButtonHandler_Update(&button_handlers[i], pressed_states[i], SHORT_PRESS_DELAY, LONG_PRESS_DELAY, butt_states);
//	}
//}

ULONGLONG btnPressStartTime[NUM_BUTTONS] = { 0 };

BOOL btnPressedState[NUM_BUTTONS] = { FALSE, FALSE, FALSE, FALSE , FALSE };
BOOL btnLongPressActive[NUM_BUTTONS] = { FALSE, FALSE, FALSE, FALSE, FALSE };
static bool ignoreNextButtonUp[NUM_BUTTONS] = { false };

LRESULT CALLBACK ButtonSubclassProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam,
	UINT_PTR uIdSubclass, DWORD_PTR dwRefData)
{
	int btnIndex = (int)dwRefData;
	char msg_str[80];

	switch (uMsg)
	{
	case WM_LBUTTONDOWN: // detects button press
		btnPressedState[btnIndex] = TRUE;
		btnPressStartTime[btnIndex] = GetTickCount64();
		setBit(win_btn_lines, (BUTTON_AUTO_INSTANT_PRESS_BIT << btnIndex)); // (Bit_0<<btnIndex)
		Display_Info.ButtonStateChanged = BTN_PRESSED; // event flag rised - will be cleared in main thread after processing via menu
		win_btn_pressed = SET;
		InvalidateRect(hWnd, NULL, TRUE); // force repainting
	  break;

	case WM_LBUTTONUP:
	  {
		if (ignoreNextButtonUp[btnIndex]) {
			ignoreNextButtonUp[btnIndex] = false;
			return 0; // skip processing
		}
		ULONGLONG duration = GetTickCount64() - btnPressStartTime[btnIndex];
		btnPressStartTime[btnIndex] = 0;
		btnPressedState[btnIndex] = FALSE;
		clearBit(win_btn_lines, (BUTTON_AUTO_INSTANT_PRESS_BIT << btnIndex)); // (Bit_0<<btnIndex)
		win_btn_pressed = CLR;
		Display_Info.ButtonStateChanged = BTN_RELEASED; // event flag rised - will be cleared in main thread after processing via menu
		InvalidateRect(hWnd, NULL, TRUE);  // RedrawAvg with normal color

		sprintf(msg_str, "Button %d held for %llu ms\n", btnIndex, duration);
		SetDlgItemText(g_hToolbar, IDC_BUT_HELD_TIME, (LPCSTR)msg_str);
		// 20250710 here is simulation of Get_Button_Press() function
		// Clear any previous state for this button
		clearBit(Display_Info.butt_states, ((BUTTON_AUTO_SHORT_PRESS_BIT | BUTTON_AUTO_LONG_PRESS_BIT) << btnIndex));

		if (duration >= SHORT_PRESS_DELAY)
		{
			if (duration >= LONG_PRESS_DELAY)
				setBit(Display_Info.butt_states, (BUTTON_AUTO_LONG_PRESS_BIT << btnIndex));
			else
				setBit(Display_Info.butt_states, (BUTTON_AUTO_SHORT_PRESS_BIT << btnIndex));
		}

		// Optionally send message to parent dialog
		HWND hParent = GetParent(hWnd);
		if (hParent)
			PostMessage(hParent, WM_APP + 1, (WPARAM)btnIndex, (LPARAM)duration);
	  }
	  break;

	case WM_LBUTTONDBLCLK:
	  {
		// Treat double-click as a SHORT PRESS
		btnPressedState[btnIndex] = FALSE;
		btnPressStartTime[btnIndex] = 0;

		clearBit(Display_Info.butt_states,
			((BUTTON_AUTO_SHORT_PRESS_BIT | BUTTON_AUTO_LONG_PRESS_BIT) << btnIndex));
		setBit(Display_Info.butt_states, (BUTTON_AUTO_SHORT_PRESS_BIT << btnIndex));

		Display_Info.ButtonStateChanged = BTN_RELEASED;
		win_btn_pressed = CLR;
		InvalidateRect(hWnd, NULL, TRUE);

		SetDlgItemText(g_hToolbar, IDC_BUT_HELD_TIME,
			TEXT("Button double-click ? treated as short press"));

		HWND hParent = GetParent(hWnd);
		if (hParent)
			PostMessage(hParent, WM_APP + 1, (WPARAM)btnIndex, (LPARAM)SHORT_PRESS_DELAY);

		// Prevent next WM_LBUTTONUP from processing
		ignoreNextButtonUp[btnIndex] = true;

		return 0;// IMPORTANT: return 0 to prevent Windows from sending another WM_LBUTTONUP
	  }
	}

	Sleep(3); //delay_us(3000); // give time to update window
	return DefSubclassProc(hWnd, uMsg, wParam, lParam);
}

NumericState IsNumericGUI(const char* str)
{
	if (str == NULL)
		return NUM_INVALID;

	// Skip leading whitespace
	while (isspace(*str)) str++;

	if (*str == '\0')
		return NUM_INVALID;

	// strtod will parse as much as it can and stop at invalid character
	char* endPtr;
	double val = strtod(str, &endPtr);

	// Skip trailing whitespace
	while (isspace(*endPtr)) endPtr++;

	if (*endPtr == '\0') {
		return NUM_VALID;
	}

	// Handle incomplete but syntactically plausible scientific notation
	size_t len = strlen(str);

	// Allow ending in 'e' or 'E'
	if (len >= 1 && (str[len - 1] == 'e' || str[len - 1] == 'E')) {
		// Must not be just "e" or "E"
		if (len == 1) return NUM_INVALID;
		return NUM_PARTIAL;
	}

	// Allow ending in 'e+' or 'e-'
	if (len >= 2 &&
		(str[len - 2] == 'e' || str[len - 2] == 'E') &&
		(str[len - 1] == '+' || str[len - 1] == '-')) {
		return NUM_PARTIAL;
	}

	return NUM_INVALID;

}

LRESULT CALLBACK EditBoxProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CHAR:
		if (wParam == VK_RETURN) {
			char buf[64];
			GetWindowText(hWnd, buf, sizeof(buf));

			for (int i = 0; i < NUM_INPUT_FIELDS; i++) {
				// Identify which edit control this is
				if (GetDlgItem(GetParent(hWnd), editControlIDs[i]) == hWnd) {
					if (Is_Numeric(buf)) {
						double value = atof(buf);
						UserValue[i] = value;
						inputState[i] = STATE_CONFIRMED;
						// Convert back to string
						char cleaned[64];
						snprintf(cleaned, sizeof(cleaned), "%.10g", value);  // Compact format

						// Avoid triggering EN_CHANGE
						suppressChangeEvent = TRUE;
						SetWindowText(hWnd, cleaned);
						suppressChangeEvent = FALSE;

						InvalidateRect(hWnd, NULL, TRUE);
					}
					else {
						inputState[i] = STATE_INVALID;
					}
					InvalidateRect(hWnd, NULL, TRUE);
					break;
				}
			}
			return 0; // prevent ding
		}
		break;
	}
	// Forward to original
	for (int i = 0; i < NUM_INPUT_FIELDS; i++) {
		if (GetDlgItem(GetParent(hWnd), editControlIDs[i]) == hWnd) {
			return CallWindowProc(OldEditProc[i], hWnd, msg, wParam, lParam);
		}
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

BOOL CALLBACK ToolDlgProc(HWND hDlg, UINT Message, WPARAM wParam, LPARAM lParam)
{
	static HBRUSH hbrBkcolor;
	static HBRUSH hbrLEDcolor;
	static BOOL fDrawEllipse[4];   // TRUE if LED ellipse is drawn
	char msg_str[50];
	long CntrSnipValue;
	CntrSnipValue = Sim_counter % 100;
	if ((CntrSnipValue >= 0) && (CntrSnipValue < 10))
	{
		Sim_counter += 10;
		//	sprintf(msg_str,"Avg loop ms: %7.2f", AvgLoopTime);
		sprintf(msg_str, "Ticks %d Avg %6.1f", (long)tickDiff, AvgLoopTime);
		SetDlgItemText(g_hToolbar, IDC_TIME_TEST, (LPCSTR)msg_str);
		// A value of zero causes the thread to relinquish the remainder of its time slice to any other thread that is ready to run.
		//	If there are no other threads ready to run, the function returns immediately, and the thread continues execution.
		Sleep(0); //release time slot
	}
	//static HFONT hFontLarge = NULL;
	BOOL anyHeld = FALSE;
	switch (Message)
	{
		case WM_TIMER:
		if (wParam == LED_UPDATE_TIMER_ID) {
			for (int i = 0; i < number_of_LEDs; ++i) {
				InvalidateRect(hDlg, rect_LED[i], FALSE);  // Repaint all LEDs
			}
		}
		// show holding button duration in real time
		for (int i = 0; i <= BTN_INDEX_RESET; ++i)
		{
			if (btnPressedState[i])
			{
				ULONGLONG held_ms = GetTickCount64() - btnPressStartTime[i];

				char msg_str[64];
				sprintf(msg_str, "Button %d held for %llu ms", i, held_ms);
				SetDlgItemText(g_hToolbar, IDC_BUT_HELD_TIME, msg_str);

				btnLongPressActive[i] = (held_ms >= LONG_PRESS_DELAY); // track for drawing
				InvalidateRect(GetDlgItem(hDlg, IDC_BUT_RETURN + i), NULL, TRUE); // force repaint

				anyHeld = TRUE;
				break; // Only show one active button's time
			}
			else
			{
				btnLongPressActive[i] = FALSE; // clear when not held
			}
			//if (!anyHeld) // clear real time holding string
			//	SetDlgItemText(g_hToolbar, IDC_BUT_HELD_TIME, "");
			//return TRUE;
		}
		if (InterlockedExchange(&LCD_Changed, 0))  // it checks and returns previous value (i.e. if it was set to '1' if will be executed) and set new value as 0
		{
			//if (GetDlgItem(hDlg, IDC_LCD_L1) && GetDlgItem(hDlg, IDC_LCD_L2))
			if (GetDlgItem(hDlg, IDC_LCD_L1) == NULL || GetDlgItem(hDlg, IDC_LCD_L2) == NULL)
			{
				MessageBox(hDlg, TEXT("LCD controls not found!"), TEXT("Error"), MB_OK | MB_ICONERROR);
				InterlockedExchange(&LCD_Changed, 0);
				break;
			}
			SetDlgItemText(hDlg, IDC_LCD_L1, (const char*)&LCD_buffer[0][0]);//line 1
			SetDlgItemText(hDlg, IDC_LCD_L2, (const char*)&LCD_buffer[1][0]);//line 2
			SendMessage(GetDlgItem(hDlg, IDC_LCD_L1), WM_SETFONT, (WPARAM)hFontLarge, TRUE);
			SendMessage(GetDlgItem(hDlg, IDC_LCD_L2), WM_SETFONT, (WPARAM)hFontLarge, TRUE);
			//InterlockedExchange(&LCD_Changed, 0);
		}
		break;
	case WM_INITDIALOG:
	{
		LedParent = GetParent(hDlg);
		SetTimer(hDlg, LED_UPDATE_TIMER_ID, LED_UPDATE_INTERVAL_MS, NULL); // 100ms timer
		hbrBkcolor = CreateSolidBrush(RGB(255, 232, 232)); //pink
		CheckDlgButton(g_hToolbar, IDC_CHECK_10X, ten_times_faster);
		// Create a larger font for LCD display
		hFontLarge = CreateFont(
			32, 0, 0, 0,          // height, width, escapement, orientation
			FW_BOLD,             // weight
			TRUE, FALSE, FALSE, // italic, underline, strikeout
			ANSI_CHARSET,
			OUT_DEFAULT_PRECIS,
			CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY,
			FIXED_PITCH | FF_MODERN,
			TEXT("Consolas")     // or "Courier New" or any mono font
		);
		if (hFontLarge == NULL) {
			MessageBox(hDlg, TEXT("Failed to create font"), TEXT("Error"), MB_OK);
		}
		HWND hButReturn = GetDlgItem(hDlg, IDC_BUT_RETURN);
		HWND hButEnter = GetDlgItem(hDlg, IDC_BUT_ENTER);
		HWND hButMns = GetDlgItem(hDlg, IDC_BUT_MNS);
		HWND hButPls = GetDlgItem(hDlg, IDC_BUT_PLS);
		HWND hButReset = GetDlgItem(hDlg, IDC_BUT_RESET);

		// Set BS_OWNERDRAW style
		LONG style;
		style = GetWindowLong(hButReturn, GWL_STYLE);
		SetWindowLong(hButReturn, GWL_STYLE, style | BS_OWNERDRAW);
		style = GetWindowLong(hButEnter, GWL_STYLE);
		SetWindowLong(hButEnter, GWL_STYLE, style | BS_OWNERDRAW);
		style = GetWindowLong(hButMns, GWL_STYLE);
		SetWindowLong(hButMns, GWL_STYLE, style | BS_OWNERDRAW);
		style = GetWindowLong(hButPls, GWL_STYLE);
		SetWindowLong(hButPls, GWL_STYLE, style | BS_OWNERDRAW);
		style = GetWindowLong(hButReset, GWL_STYLE);
		SetWindowLong(hButReset, GWL_STYLE, style | BS_OWNERDRAW);

		SetWindowSubclass(hButReturn, ButtonSubclassProc, 1, BTN_INDEX_RETURN);
		SetWindowSubclass(hButEnter, ButtonSubclassProc, 2, BTN_INDEX_ENTER);
		SetWindowSubclass(hButMns, ButtonSubclassProc, 3, BTN_INDEX_MNS);
		SetWindowSubclass(hButPls, ButtonSubclassProc, 4, BTN_INDEX_PLS);
		SetWindowSubclass(hButReset, ButtonSubclassProc, 5, BTN_INDEX_RESET);

		// initialize  array of ponters to rectangles
		rect_LED[LEDindx_ASCAN]  = &rctAutoLED;
		rect_LED[LEDindx_ALARM]  = &rctAlarmLED;
		rect_LED[LEDindx_TX_RX]  = &rctRxTxLED;
		rect_LED[LEDindx_PULSE]  = &rctPulseLED;

		// initialize  array of ponters to colors
		hbr_LED_off = CreateSolidBrush(RGB(127, 127, 127));
		hbr_LED_Red = CreateSolidBrush(RGB(255, 0, 0));
		hbr_LED_Green = CreateSolidBrush(RGB(0, 255, 0));
		hbr_LED_Yellow = CreateSolidBrush(RGB(255, 255, 0));
		hbr_LEDcolor_ptr[LEDindx_ASCAN] = hbr_Color_ptr[ColorOFF];
		hbr_LEDcolor_ptr[LEDindx_ALARM] = hbr_Color_ptr[ColorRED];
		hbr_LEDcolor_ptr[LEDindx_TX_RX] = hbr_Color_ptr[ColorGREEN];
		hbr_LEDcolor_ptr[LEDindx_PULSE] = hbr_Color_ptr[ColorYELLOW];

		hbrValid = CreateSolidBrush(colorValid);
		hbrInvalid = CreateSolidBrush(colorInvalid);
		hbrPartial = CreateSolidBrush(colorPartial);

		for (int i = 0; i < NUM_INPUT_FIELDS; i++) {
			HWND hEdit = GetDlgItem(hDlg, editControlIDs[i]);
			OldEditProc[i] = (WNDPROC)SetWindowLongPtr(hEdit, GWLP_WNDPROC, (LONG_PTR)EditBoxProc);
		}

		UserValue[Vbat] = 126.67;	// Volts
		UserValue[Vgnd] = 62.5;		// Volts
		UserValue[Vrip] = 222.2;	// mVolts
		UserValue[Irip] = 55.5;		// mAmperes
		return TRUE;
	}
	break;
	case WM_CTLCOLORSTATIC:
	{
		HDC hdc = (HDC)wParam;
		HWND hwndStatic = (HWND)lParam;

		if (hwndStatic == GetDlgItem(hDlg, IDC_LCD_L1) ||
			hwndStatic == GetDlgItem(hDlg, IDC_LCD_L2))
		{
			SetTextColor(hdc, RGB(255, 0, 0));
			SetBkMode(hdc, TRANSPARENT);
			return (LRESULT)hbrBkcolor;
		}
	}
	break;
	case WM_CTLCOLOREDIT:
	{
		HDC hdcEdit = (HDC)wParam;
		HWND hEdit = (HWND)lParam;

		for (int i = 0; i < NUM_INPUT_FIELDS; i++) {
			if (hEdit == GetDlgItem(hDlg, editControlIDs[i])) {
				char buf[64];
				GetWindowText(hEdit, buf, sizeof(buf));

				NumericState state = IsNumericGUI(buf);
				switch (state) {
				case NUM_VALID:
					if (inputState[i] == STATE_CONFIRMED) {
						SetBkColor(hdcEdit, colorConfirmed); // light green
						return (INT_PTR)colorConfirmed;
					}
					else {
						SetBkColor(hdcEdit, colorValid); // white
						return (INT_PTR)colorValid;
					}

				case NUM_PARTIAL:
					SetBkColor(hdcEdit, colorPartial);  // light gray
					return (INT_PTR)hbrPartial;

				case NUM_INVALID:
					if (inputState[i] == STATE_UNTOUCHED) {
						SetBkColor(hdcEdit, colorValid); // white
						return (INT_PTR)colorValid;
					}
					else {
						SetBkColor(hdcEdit, colorInvalid);    // yellow
						return (INT_PTR)colorInvalid;
					}
				}

				//switch (inputState[i]) {
				//case STATE_UNTOUCHED:

				//case STATE_VALID:
				//	SetBkColor(hdcEdit, colorValid); // white
				//	return (INT_PTR)colorValid;

				//case STATE_INVALID:
				//	SetBkColor(hdcEdit, colorInvalid); // yellow
				//	return (INT_PTR)colorInvalid;

				//case STATE_CONFIRMED:
				//}
			}
		}
	}
	break;
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hDlg,&ps);
		HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, *hbr_LEDcolor_ptr);
		HBRUSH hOldPen = (HBRUSH)SelectObject(hdc, GetStockObject(BLACK_PEN)); // No border; BLACK_PEN

		for (int i = 0; i < number_of_LEDs; ++i) {
			hOldBrush = (HBRUSH)SelectObject(hdc, *hbr_LEDcolor_ptr[i]);
			Ellipse(hdc,
				rect_LED[i]->left,
				rect_LED[i]->top,
				rect_LED[i]->right,
				rect_LED[i]->bottom);
		}
/*
		// Fill and draw AUTO LED ellipse
		SelectObject(hdc, *hbr_LEDcolor_ptr[LEDindx_ASCAN]);
		Ellipse(hdc, AUTO_LED_x1 - 2, AUTO_LED_y1 - 2, AUTO_LED_x2 + 2, AUTO_LED_y2 + 2);

		// Fill and draw ALARM LED ellipse
		SelectObject(hdc, *hbr_LEDcolor_ptr[LEDindx_ALARM]);
		Ellipse(hdc, ALARM_LED_x1 - 2, ALARM_LED_y1 - 2, ALARM_LED_x2 + 2, ALARM_LED_y2 + 2);

		// Fill and draw TX/RX LED ellipse
		SelectObject(hdc, *hbr_LEDcolor_ptr[LEDindx_TX_RX]);
		Ellipse(hdc, TX_RX_LED_x1 - 2, TX_RX_LED_y1 - 2, TX_RX_LED_x2 + 2, TX_RX_LED_y2 + 2);

		// Fill and draw PULSE LED ellipse
		SelectObject(hdc, *hbr_LEDcolor_ptr[LEDindx_TX_RX]);
		Ellipse(hdc, PULSE_LED_x1 - 2, PULSE_LED_y1 - 2, PULSE_LED_x2 + 2, PULSE_LED_y2 + 2);

		// Restore old objects
		SelectObject(hdc, hOldBrush);
		SelectObject(hdc, hOldPen);
*/
		EndPaint(hDlg, &ps);
		return 0;
	}
	break;
	case WM_COMMAND: // triggered when button is released, but AFTER ButtonSubclassProc() case WM_LBUTTONUP: is executed
	{
		int wmId = LOWORD(wParam);
		int code = HIWORD(wParam);
		HWND hCtrl = (HWND)lParam;

		if (code == EN_CHANGE && !suppressChangeEvent) {
			for (int i = 0; i < NUM_INPUT_FIELDS; i++) {
				if (wmId == editControlIDs[i]) {
					char buf[64];
					GetWindowText(hCtrl, buf, sizeof(buf));

					NumericState state = IsNumericGUI(buf);

					switch (state) {
					case NUM_VALID:    inputState[i] = STATE_VALID; break;
					case NUM_PARTIAL:  inputState[i] = STATE_VALID; break; // treat as valid for now
					case NUM_INVALID:  inputState[i] = STATE_INVALID; break;
					}
					//if (strlen(buf) == 0) {
					//	inputState[i] = STATE_UNTOUCHED;
					//}
					//else if (IsNumeric(buf)) {
					//	inputState[i] = STATE_VALID;
					//}
					//else {
					//	inputState[i] = STATE_INVALID;
					//}

//					isValid[i] = IsNumeric(buf);
					InvalidateRect(hCtrl, NULL, TRUE); // Force redraw for color update
					break;
				}
			}
		}
		else if (code == EN_MAXTEXT) {
			// Optional: Handle overflows
		}
	}
	break;
	case WM_DRAWITEM:
	{
		LPDRAWITEMSTRUCT lpdis = (LPDRAWITEMSTRUCT)lParam;
		HBRUSH hBrush;
		COLORREF bgColor;

		int btnIndex = -1;
		switch (lpdis->CtlID)
		{
		case IDC_BUT_RETURN: btnIndex = BTN_INDEX_RETURN; break;
		case IDC_BUT_ENTER:  btnIndex = BTN_INDEX_ENTER;  break;
		case IDC_BUT_MNS:    btnIndex = BTN_INDEX_MNS;    break;
		case IDC_BUT_PLS:    btnIndex = BTN_INDEX_PLS;    break;
		case IDC_BUT_RESET:  btnIndex = BTN_INDEX_RESET;  break;
		}

		if (btnIndex != -1 && btnPressedState[btnIndex]) {
			// Set color based on long press flag
			bgColor = btnLongPressActive[btnIndex] ? RGB(255, 255, 100) : RGB(200, 200, 255); // light blue when pressed, becomes yellow if pressed more than 3 sec
		}
		else
			bgColor = RGB(200, 240, 200);  // default green-gray

		hBrush = CreateSolidBrush(bgColor);
		FillRect(lpdis->hDC, &lpdis->rcItem, hBrush);
		DeleteObject(hBrush);

		// Draw simple border rectangle
		//FrameRect(lpdis->hDC, &lpdis->rcItem, GetStockObject(BLACK_BRUSH));

		// Draw 3D style border rectangle
		UINT edgeType = (btnPressedState[btnIndex]) ? BDR_SUNKENINNER : BDR_RAISEDINNER;
		DrawEdge(lpdis->hDC, &lpdis->rcItem, edgeType, BF_RECT);

		// Draw the button text
		TCHAR text[64];
		GetWindowText(lpdis->hwndItem, text, 64);
		SetBkMode(lpdis->hDC, TRANSPARENT);
		DrawText(lpdis->hDC, text, -1, &lpdis->rcItem,
			DT_CENTER | DT_VCENTER | DT_SINGLELINE);

		return TRUE;
	}
	case WM_APP + 1:
	{
		int btnIndex = (int)wParam;
		ULONGLONG duration = (ULONGLONG)lParam;
		// Use the timing info as needed
		// For example, map index to bit and call clearBit(), etc.
	}
	break;

	case WM_UPDATE_LCD:
		SetDlgItemText(hDlg, IDC_LCD_L1, (const char*)LCD_buffer[0]);
		SetDlgItemText(hDlg, IDC_LCD_L2, (const char*)LCD_buffer[1]);
		break;

	case WM_CLOSE:
		DestroyWindow(hDlg);
		break;
	case WM_DESTROY:
		if (hFontLarge) DeleteObject(hFontLarge);
		PostQuitMessage(0);
		KillTimer(hDlg, LED_UPDATE_TIMER_ID);
		if (hbrValid) DeleteObject(hbrValid);
		if (hbrInvalid) DeleteObject(hbrInvalid);
		if (hbrPartial) DeleteObject(hbrPartial);

		for (int i = 0; i < Total_Colors; ++i) {
			if (hbr_Color_ptr[i] && *hbr_Color_ptr[i]) {
				DeleteObject(*hbr_Color_ptr[i]);
			}
		}
		break;
	default:
		return DefWindowProc(hDlg, Message, wParam, lParam);
	}
	return 0;
}

const char g_szClassName[] = "myWindowClass";

// Step 4: the Window Procedure
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_CLOSE:
		DestroyWindow(hwnd);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hwnd, msg, wParam, lParam);
	}
	return 0;
}

/*************************/
void LCD_dialog(void* MyParam)
{
	HINSTANCE hInstance = NULL;
	int nCmdShow = SW_SHOW;

	WNDCLASSEX wc;
	HWND hwnd = NULL;

	UINT32 processId;

	//Step 1: Registering the Window Class
	wc.cbSize        = sizeof(WNDCLASSEX);
	wc.style         = 0;
	wc.lpfnWndProc   = WndProc;
	wc.cbClsExtra    = 0;
	wc.cbWndExtra    = 0;
	wc.hInstance     = hInstance;
	wc.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
	wc.lpszMenuName  = NULL;
	wc.lpszClassName = g_szClassName;
	wc.hIconSm       = LoadIcon(NULL, IDI_APPLICATION);

	if (!RegisterClassEx(&wc))
	{
		MessageBox(NULL, "Window Registration Failed!", "Error!",
			MB_ICONEXCLAMATION | MB_OK);
		goto end_thrend;//return 0;
	}

	// Step 2: Creating the Window
	hwnd = CreateWindowEx(
		WS_EX_CLIENTEDGE,
		g_szClassName,
		"        Electroswitch",
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT,
		FrontPanel_WIDTH,//nWidth The width, in device units, of the window.
		FrontPanel_HEIGHT,//nHeight
		NULL, NULL, hInstance, NULL);

	if (hwnd == NULL)
	{
		MessageBox(NULL, "Window Creation Failed!", "Error!",
			MB_ICONEXCLAMATION | MB_OK);
		goto end_thrend;//return 0;
	}

	ShowWindow(hwnd, nCmdShow);
	UpdateWindow(hwnd);
	{
		g_hToolbar = CreateDialog(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_DIALOGBAR),
			hwnd, ToolDlgProc);
		if (g_hToolbar != NULL)
		{
			ShowWindow(g_hToolbar, SW_SHOW);
		}
		else
		{
			MessageBox(hwnd, "CreateDialog returned NULL", "Warning!", MB_OK | MB_ICONINFORMATION);
		}
	}
	processId = SendMessage(    // returns LRESULT in lResult
		GetDlgItem(g_hToolbar, IDC_LCD_L1),           // (HWND) handle to destination control
		WM_SETFONT,            // (UINT) message ID
		(WPARAM)hFontLarge, //GetStockObject(SYSTEM_FIXED_FONT ), // = () wParam; DEFAULT_GUI_FONT
		TRUE                 // = () lParam;
	);
	processId = SendMessage(    // returns LRESULT in lResult
		GetDlgItem(g_hToolbar, IDC_LCD_L2),           // (HWND) handle to destination control
		WM_SETFONT,            // (UINT) message ID
		(WPARAM)hFontLarge, // = () wParam; DEFAULT_GUI_FONT
		TRUE                 // = () lParam;
	);

	// Step 3: The Message Loop
	// while (PeekMessage(&Msg, hwnd,  0, 0, PM_REMOVE)) //this momentarely creates window and hides or closes it
	while (GetMessage(&Msg, NULL, 0, 0) > 0)
	{
		TranslateMessage(&Msg);
		DispatchMessage(&Msg);
		if (InterlockedExchange(&LCD_Changed, 0)) // it checks and returns previous value and set new value as 0
		{
			//SendMessage(GetDlgItem(g_hToolbar, IDC_LCD_L1), WM_SETFONT, (WPARAM)hFontLarge, TRUE);
			//SendMessage(GetDlgItem(g_hToolbar, IDC_LCD_L2), WM_SETFONT, (WPARAM)hFontLarge, TRUE);
			//SetDlgItemText(hDlg, IDC_LCD_L1, &LCD_buffer[0][0]);//line 1
			//SetDlgItemText(hDlg, IDC_LCD_L2, &LCD_buffer[1][0]);//line 2
			//--- IK20250703: Chat GPT said - it is bad practice to  update and repaint text box in a separate thread
			//--- SetDlgItemText(g_hToolbar, IDC_LCD_L1, (LPCSTR)&LCD_buffer[0][0]);//line 1
			//--- SetDlgItemText(g_hToolbar, IDC_LCD_L2, (LPCSTR)&LCD_buffer[1][0]);//line 2
			//--- RedrawWindow(g_hToolbar, 0, 0, RDW_INVALIDATE);
			//--- insted, post message and let GUI thread update it
			PostMessage(g_hToolbar, WM_UPDATE_LCD, 0, 0); // send to main thread
		}
	}
end_thrend:
	// return Msg.wParam;
	// Tell thread to die and record its death.
	ReleaseMutex(hRunMutex);
	EndControlLoop = TRUE;
	ShutDown();
}

BOOL Terminate_Main_Thread = false;
int Main_Thread_Is_Running = 0;
BOOL Terminate_ISR_Thread = false;
BOOL Terminate_LCD_Thread = false;
//
// Usage: Set_Thread_Name (-1, "MainThread");
//
typedef struct tagTHREADNAME_INFO
{
	DWORD dwType; // must be 0x1000
	LPCSTR szName; // pointer to name (in user addr space)
	DWORD dwThreadID; // thread ID (-1=caller thread)
	DWORD dwFlags; // reserved for future use, must be zero
} THREADNAME_INFO;

void Set_ThreadName(DWORD dwThreadID, LPCSTR szThreadName)
{
	THREADNAME_INFO info;
	{
		info.dwType = 0x1000;
		info.szName = szThreadName;
		info.dwThreadID = dwThreadID;
		info.dwFlags = 0;
	}
	__try
	{
		RaiseException(0x406D1388, 0, sizeof(info) / sizeof(DWORD), (DWORD*)&info);
	}
	__except (EXCEPTION_CONTINUE_EXECUTION)
	{
	}
}

void Set_Thread_Name(const char* threadName)
{
	Set_ThreadName(GetCurrentThreadId(), threadName);
}

int new_thread() // Thread One
{
	// Get display screen information & clear the screen.
	hConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfo(hConsoleOut, &csbiInfo);
	//	ClearScreen();
	//	WriteTitle( 0 );

		// Create the mutexes and reset thread count.
	hScreenMutex = CreateMutex(NULL, FALSE, NULL);  // Cleared
	hRunMutex = CreateMutex(NULL, TRUE, NULL);      // Set
	ThreadNr = 1;
	_beginthread(LCD_dialog, 0, 0);//&ThreadNr
	ThreadNr = 2;
	_beginthread(PC_SimulationApplication, 0, nullptr);//&ThreadNr

	// Start waiting for keyboard input to dispatch threads or exit.
//	KbdFunc();
	return(0);
}

void ShutDown(void) // Shut down threads
{
	while (ThreadNr > 0)
	{
		// Tell thread to die and record its death.
		ReleaseMutex(hRunMutex);
		ThreadNr--;
	}

	// Clean up display when done
	WaitForSingleObject(hScreenMutex, INFINITE);
	// All threads done. Clean up handles.
	CloseHandle(hScreenMutex);
	CloseHandle(hRunMutex);
	CloseHandle(hConsoleOut);
}


void WriteTitle(int ThreadNum)
{
	enum {
		sizeOfNThreadMsg = 80
	};
	char    NThreadMsg[sizeOfNThreadMsg];

	//	sprintf_s( NThreadMsg, sizeOfNThreadMsg,
	sprintf(NThreadMsg,
		"Threads running: %02d.  Press 'A' "
		"to start a thread,'Q' to quit.", ThreadNum);
	SetConsoleTitle(NThreadMsg);
}

void ClearScreen(void)
{
	DWORD    dummy;
	COORD    Home = { 0, 0 };
	FillConsoleOutputCharacter(hConsoleOut, ' ',
		csbiInfo.dwSize.X * csbiInfo.dwSize.Y,
		Home, &dummy);
}


/*************************/
//#endif //block windows code
void gotoxy(int x, int y)
{
	COORD xy;
	xy.X = x - 1;
	xy.Y = y - 1;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), xy);
}

/*************************/
void clrscr(void)
{
	DWORD chrs = 0;
	COORD xy;
	xy.X = 0;
	xy.Y = 0;
	/*
	FillConsoleOutputCharacterA(
	HANDLE hConsoleOutput,
	CHAR  cCharacter,
	DWORD  nLength,
	COORD  dwWriteCoord,
	LPDWORD lpNumberOfCharsWritten
	);
	*/
	FillConsoleOutputCharacter(GetStdHandle(STD_OUTPUT_HANDLE),
		' ',
		(30 * 120), // Win 7 has default size 80x25, Win 10 has 30*120
		xy,
		&chrs);// "&chrs" gives compilation warnings, "chrs" gives handled exception 0xC000005 at run-time
	xy.X = 0;
	xy.Y = 0;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), xy);
}

int wherex()
{
	CONSOLE_SCREEN_BUFFER_INFO con_win;
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &con_win);
	return(con_win.dwCursorPosition.X + 1);
}
int wherey()
{
	CONSOLE_SCREEN_BUFFER_INFO con_win;
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &con_win);
	return(con_win.dwCursorPosition.Y + 1);
}
/*************************/
void pc_print_tab(int tab)
{
	int currentY = wherey();
	gotoxy(tab, currentY);
}

/*************************/
char D_pack(int x, int y)
{
	return (0x80 + ((y) % 2) * 0x40 + ((y) / 2) * 0x14 + (x));
}

/*************************/
void D_unpack(char c, int xpos, int ypos)
{
	int  p;
	if (c & 0x80) /* unpack D_AT */
	{
		/*	#define	D_AT(x,y)	(0x80 + ((y)%2)*0x40 + ((y)/2)*0x14 + (x)) /* packs x y position into char*/
		/*	#define	D_AT(x,y)	( 128 + ((y)%2) * 64 + ((y)/2) * 20 + (x)) /* packs x y position into char*/
		xpos = ypos = p = 0;
		p = c & 0x7f;		/* clear upper bit, do not change c  */
		if (p < 20)
		{
			xpos = p; ypos = 0;
		}
		if ((p >= 20) && (p <= 39))
		{
			xpos = p - 20; ypos = 2;
		}
		if ((p >= 64) && (p <= 83))
		{
			xpos = p - 64; ypos = 1;
		}
		if ((p >= 84) && (p <= 103))
		{
			xpos = p - 84; ypos = 3;
		}
	}
}

/*************************/
//lcd.h
// deals with 20x2 dot matrix LCD  with backlight, for example Lumex part number LCM-S02002DSF, DigiKey # 67-1762-ND

#define     CLR_DISP        (char)0x01
#define     CUR_HOME        (char)0x02
#define     DISP_ON         (char)0x0C
#define     DISP_OFF        (char)0x08
#define     CUR_OFF         (char)0x0C
#define     CUR_ON_UNDER    (char)0x0E
#define     CUR_ON_BLINK    (char)0x0F
#define     CUR_LEFT        (char)0x10
#define     CUR_RIGHT       (char)0x14
#define     CUR_UP          (char)0x80
#define     CUR_DOWN        (char)0xC0
#define     ENTER           (char)0xC0
#define     DD_RAM_ADDR     (char)0x80
#define     DD_RAM_ADDR2    (char)0xC0
#define CURSOR_TO_FIRST_LINE    CUR_UP    // LCD_RS(bit#5=0), lcd_enable(bit#5=1), "upper" half byte{(bit#(3-0)=8)
#define CURSOR_TO_SECND_LINE    CUR_DOWN  // LCD_RS(bit#5=0), lcd_enable(bit#5=1), "upper" half byte{(bit#(3-0)=c)
//#define LCD_LIN_SIZE         20

void clrLCD(void)
{
	//0000000000111111111122222222223333333333444
	//0123456789012345678901234567890123456789012
	gotoxy(1, 1);
	//	puts("Front Panel PC  keys:    | (PREVIOUS Button) = 'Home'  (SELECT  button) = 'End'");
	//	puts("LCD window keys also work| ( DOWN(-)button) = 'PgDwn'   (UP(+)  Button) = 'PgUp'");
	memset(LCD_buffer, 0, sizeof(LCD_buffer));
	LCDx[0] = 0; LCDx[1] = 0; LCDy = 0;
	InterlockedExchange(&LCD_Changed, 1);	//signal to Windows update dialog
}

void SetLCDLine(int lineIndex, const char* text) {
	if (lineIndex < 0 || lineIndex >= LCD_BUF_DEPTH) return;

	// Safe copy to internal buffer
	strncpy((char*)LCD_buffer[lineIndex], text, LCD_BUF_LEN - 1);
	LCD_buffer[lineIndex][LCD_BUF_LEN - 1] = '\0'; // Ensure null-termination

	// Notify main thread to update display
	PostMessage(g_hToolbar, WM_UPDATE_LCD, 0, 0);
}
/*************************/
void  D_PutText(const char* pstr)
{
	char* local_p = (char*)pstr;
	int len = strlen(pstr);		//withOUT terminating zero
	int StartIndex = LCDx[LCDy];	//start index for "for" loop
	//update windows buffer, TWO INDEPENDENT POINTERS for line 1 and line 2
	for (; LCDx[LCDy] < StartIndex + len;)//len here DOES NOT include End of String 0x00
	{
		if ((LCDx[LCDy]) >= (LCD_BUF_LEN - 1)) break;	// do not overwrite default string terminator 0x00 at pos 21
		LCD_buffer[LCDy][LCDx[LCDy]] = *local_p;	// save char in the shadow buffer
		LCDx[LCDy]++;								// move index
		local_p++;									// move pointer
		InterlockedExchange(&LCD_Changed, 1);		// signal to Windows update dialog
	}
	LCD_buffer[LCDy][LCDx[LCDy]] = 0;		//add terminating zero
}

/*************************/
void  D_CleanLCD_line(int line) // line is only 1 or 2
{
	if (line > 2) line = 2;
	if (line < 1) line = 1;
	LCDy = line - 1;
	LCDx[LCDy] = 0;
	memset(&LCD_buffer[LCDy], 0, sizeof(LCD_buffer[LCDy])); //clean only one line
}

/*************************/
void  D_PutChar(unsigned char c)
{
	//update windows buffer
	if ((LCDx[LCDy]) < (LCD_BUF_LEN - 1));
	{
		LCD_buffer[LCDy][LCDx[LCDy]] = c;	// save char in the shadow buffer
		LCDx[LCDy]++;						// move pointer
		InterlockedExchange(&LCD_Changed, 1);	//signal to Windows update dialog
	}
}

/*************************/
void  D_PutCtrl(unsigned char c)
{
	/*
		int xpos, ypos, p;
		xpos = ypos = p = 0;
		if (c & 0x80 ) // unpack D_AT
		{
	//		#define	D_AT(x,y)	(0x80 + ((y)%2)*0x40 + ((y)/2)*0x14 + (x)) // packs x y position into char
	//		#define	D_AT(x,y)	( 128 + ((y)%2) * 64 + ((y)/2) * 20 + (x)) // packs x y position into char
			p = c & 0x7f;		// clear upper bit, do not change c
			if (p <20)
			{
				xpos=p+1; ypos=1;
			}
			if ((p>=20) && (p<=39))
			{
				xpos=p-19; ypos=3;
			}
			if ((p>=64) && (p<=83))
			{
				xpos=p-63; ypos=2;
			}
			if ((p>=84) && (p<=103))
			{
				xpos=p-83; ypos=4;
			}
			cputsxy("_",xpos,ypos);	// put cursor into position
			gotoxy(xpos,ypos);
		}
	*/
	if (c == D_CLEAR) clrLCD();
	else if (((char)c) == D_HOME)
	{
		memset(LCD_buffer, 0, sizeof(LCD_buffer));
		InterlockedExchange(&LCD_Changed, 1);	//signal to Windows update dialog

		LCDx[0] = LCDx[1] = LCDy = 0;
	}
	else if (((char)c) == D_INIT) D_InitLCD();
	/*	else if (c==D_ON	    ) clrscr();
		else if (c==D_OFF	) clrscr();*/
		//	else if (c==D_CURSON ) putch('_');
		//	else if (c==D_CURSOFF) putch(' ');
	else if (((char)c) == (DD_RAM_ADDR))
	{
		LCDy = 0;
		LCDx[LCDy] = 0;//put pointer on first position
	}
	else if (((char)c) == DD_RAM_ADDR2)
	{
		LCDy = 1;
		LCDx[LCDy] = 0;//put pointer on second position
	}
}

/*************************/
unsigned char  D_ReadChar(void)
{
	return LCD_buffer[LCDy][LCDx[LCDy]];
}

/*************************/
unsigned char  D_ReadCtrl(void)
{
	return D_AT(LCDx[LCDy], LCDy);
}

/*************************/
void D_InitLCD(void)
{
	clrLCD();
	ScreenPos = 0x00;
	LCDx[0] = LCDx[1] = LCDy = 0;
}

/* --------- int get_PC_key(char *s) ------ I.K. 5-27-2005 -----
 *
 *  RECEIVES PC command sequence (2 bytes) if default PC (#ifdef PC) and translates it to ANSI
 *  ANSI sequence: "ESC [ A" - cursor UP, "ESC [ B" - cursor DOWN, "ESC [ C" - cursor RIGHT, "ESC [ D" - cursor LEFT
 *  PC sequence: "\x0 H" - cursor UP, "\x0 P" - cursor DOWN, "\x0 M" - cursor RIGHT, "\x0 K" - cursor LEFT
 *  F11 = \x0 \x85, F12 = \x0 \x86,
 *  Home = "\x0 G"(0x47), End = "\x0 O"(0x4f), PageUp = "\x0 I"(0x49), PageDown = "\x0 Q"(0x51),
 *  get_PC_key returns 0, if common or unrecognized key was pressed,
 *         returns 1, if cursor key was pressed
 *         pressed key symbol (if return 0) or cursor action label (if return 1) is in the first byte of s
 * */

#define KEY_LEFT	0x1
#define KEY_RIGHT	0x2
#define KEY_BACK	0x4
#define KEY_ENTER	0x8
#define KEY_SECRET_COMBINATION	(KEY_RIGHT + KEY_ENTER)

#define KEY_MASK	0xF

// MAPPING rocker switches ON THE PC keyboard

#define KEY_F11			0x85	//F11 = \x0 \x85
#define KEY_F12			0x86	//F12 = \x0 \x86
#define KEY_HOME		0x47	//Home = "\x0 G"(0x47)
#define KEY_END			0x4f	//End = "\x0 O"(0x4f)
#define KEY_PAGE_UP		0x49	//PageUp = "\x0 I"(0x49, 73D)
#define KEY_PAGE_DOWN	0x51	//PageDown = "\x0 Q"(0x51)

/*
CONTROLLER PUSH BUTTONS:
		,------.
		| +,UP |
		`------'
,------.        ,------.
|RETURN|  MENU  |ENTER |
`------'        `------'
		,------.
		|-,DOWN|
		`------'

   PC KEYBOARD KEYS
 --------    ----------
|  Home  |   | PageUp |
 --------    ----------
 --------    ----------
|   End  |   |PageDown|
 --------    ----------

 RE-MAPPING for PC EMULATION
 --------     -------
|  Home  |   | PageUp |
| RETURN |   | +,UP   |
 --------     -------
		  MENU
 --------     -------
|   End  |   |PageDown|
|  ENTER |   | -,DOWN |
 --------     -------
		 CURSOR
*/
#define MENU_RETURN     KEY_HOME
#define MENU_ENTER      KEY_END
#define CURSOR_MINUS    KEY_PAGE_DOWN
#define CURSOR_PLUS     KEY_PAGE_UP
//#define SECRET          KEY_SECRET_COMBINATION


/*************************/
extern void USART0_RX_interrupt(void);

	void CheckAndAddRxDataToBuffer(void)
{

}
// NO KEYBOARD ACTIVITY: returns 0 and puts in &key 0
// REGULAR KEYBOARD BUTTON PRESSED: returns 0 and puts in &key = ASCII of key button pressed
// PUSHBUTTON SWITCH EMULATION KEY PRESSED ON KEYBOARD (Home, End, PgUp, PgDwn, F12): returns switch code and puts in &key = 0
int get_PC_key(char* key)
{
	char s = 0;
	char ch;
	*key = 0;
	Sleep(1);//delay_us(1000);
	if (chrrdy())
	{
		ch = (getch());     /*for PC keyboard cursor sequence string starts from 0 */
		if ((ch == -32) || (ch == 0))		/*or  string starts from 0xE0 */
		{  /*  wait10ms(10);  */
			delay_us(1000);// Sleep(1);
			if (chrrdy())            /* if immediately was sent second byte */
			{
				ch = getch();            /* substitute PC symbol with BUTTON CODE */
				// >>>>>>>>>> !!! KEYBOARD INPUT !!!   bits active HIGH   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				s = 0;//~(BUTTON_BIT_RIGHT + BUTTON_BIT_UP + BUTTON_BIT_DOWN + BUTTON_BIT_LEFT); // Start from "NOTHING IS PRESSED", bits active HIGH
				if (ch == MENU_ENTER)         setBit(s, BUTTON_BIT_RIGHT); /* O, 0x4F, 79D, "end", ENTER or RIGHT */
				else if (ch == MENU_RETURN)   setBit(s, BUTTON_BIT_LEFT);  /* G, 0x47, 71D, "home" BACK or LEFT*/
				else if (ch == CURSOR_PLUS)   setBit(s, BUTTON_BIT_UP);    /* I, 0x49, 73D, "page up" */
				else if (ch == CURSOR_MINUS)  setBit(s, BUTTON_BIT_DOWN);  /* Q, 0x51, 81D, "page down" */
				// else if (((int)ch & 0x00FF) == 0x0086)	 clearBit(s, KEY_SECRET_COMBINATION); //VC does not like char's upper bit=1, considering it is negative char
				else return(0); //no button recognized

				//s = s & 0xFF; //mask to have byte
				Display_Info.buttons_hits = s;	//any button pressed - appropriate bit =0
				Display_Info.ButtonStateChanged = BTN_PRESSED;
				return s; //return NOT ZERO means keyboard button pressed
			}
		}
		else {
			*key = ch;     //  first bit was NOT =0 or 0xE0, update character via pointer
			Display_Info.ButtonStateChanged = BTN_RELEASED;
		}
		//  rt.Host &= ~CharAvailableFlag; //clear flag
	}

	// >>>>>>>>>> !!! MOUSE INPUT !!!   bits active (i.e. when button pressed) HIGH  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//Display_Info.ButtonStateChanged = win_btn_pressed;
	Display_Info.buttons_hits = win_btn_lines;// update byte from win_btn_lines, which is updated in CALLBACK ButtonSubclassProc(..)
	if (win_btn_pressed == SET) //user pressed button on interface using mouse
	{
		//Display_Info.butt_states &= 0x00FF;	 // clear second byte
		//win_btn_pressed = CLR; //clear flag
		delay_us(1000);// Sleep(1);
		return win_btn_lines; //return NOT ZERO means button pressed using mouse
	}
	return 0; //return ZERO means UART byte arrived IF *key updated (if *key != zero)
}

int toupper(int ch)
{
	int ch2;
	int ch1 = ch;// & 0x00ff; // strip high byte
	ch2 = ch1 + ('A' - 'a');// ch2 is upper case of ch1; ('A' - 'a')= -32
	return ((unsigned int)(ch1 - 'a') <= (unsigned int)('z' - 'a')) ? ch2 : ch1;
}

int tolower(int ch)
{
	int ch2;
	int ch1 = ch;// & 0x00ff; // strip high byte
	ch2 = ch1 + ('a' - 'A');// ch2 is upper case of ch1; ('A' - 'a')= -32
	return ((unsigned int)(ch1 - 'A') <= (unsigned int)('Z' - 'A')) ? ch2 : ch1;
}

/*	Character get from the background loop if there is one.
 *	Does not wait till character is received.
 */
char check_ch(int* ptrKEY)
{
	// character to be returned
	char received_char;

	// NO KEYBOARD ACTIVITY: returns 0 and puts "0" in address &ptrKEY
	// REGULAR KEY PRESSED: returns ASCII of key button pressed, and puts "0" in address &ptrKEY
	// PUSH BUTTON SWITCH EMULATION KEY PRESSED (Home, end, PgUp, PgDwn,F12):
	//returns 0 and puts in &ptrKEY button code, also updates Display_Info.butt_states and Display_Info.ButtonStateChanged
	char PushButtonKey;

	PushButtonKey = get_PC_key(&received_char);//returns pushbatton switch code, not a symbol
	if (PushButtonKey != 0) // button pressed
	{
		*ptrKEY = PushButtonKey;
		return (0); //no UART symbol, empty
	}
	else // UART symbol or nothing
	{
		*ptrKEY = NO_PRESSED_BUTTONS; //return "NO BUTTONS PRESSED", any button pressed - appropriate bit =0
		if (received_char != 0)
		{
			UDR0 = toupper(received_char); //place received char into UART data register to simulate UART reception
			USART0_RX_interrupt(); //call UART RX interrupt handler to process received char
			//putchar( received_char); // echo on console
			//received_char = tolower(received_char);

			/* IK20251219 Replaced this logic with logic in USART0_RX_interrupt
			if (rt.operating_protocol == ASCII_CMDS) {
				if (received_char == CaRet)
				{
					setBit(rt.Host, CmdAvailFlag);
					rt.HostRxBuff[rt.HostRxBuffPtr] = 0;			//store the EoS = 0 instead of '\r'
					msg_status = MSG_ARRIVED;						// message is complete for parsing
					//putchar(LF); // move to the next line in console
				}
				else
				{
					if (msg_status != MSG_DONE) {
						rt.HostRxBuff[rt.HostRxBuffPtr] = received_char;             //store the new char in Q
						rt.HostRxBuffPtr++;
					}
				}
			}
			else if (msg_status != MSG_DONE)
			{
				rt.HostRxBuff[rt.HostRxBuffPtr] = received_char;             //store the new char in Q
				rt.HostRxBuffPtr++;
			}
			else if (msg_status == MSG_DONE)
			{
				// echo it on console (already done above)
			}
			*/
			*ptrKEY = NO_PRESSED_BUTTONS; //return "NO BUTTONS PRESSED", any button pressed - appropriate bit =0
			return (received_char);
		}
		else	return (0);//  nothing
	}
}

/*************************/
int get_ch(void) // for PC
{
	int Buttons;
	int ret_char;
	while (1)
	{
		//instead of interrupt
		// IK20230612 not checked char_timer--;			// need for emulatiton
		//		PC_emulator();					// calculate emulation stimula
		//		IRQ_Handler();					// call control interrupt for emulation
		//	CHECK IF CMD RECEIVED FROM HOST
		ret_char = check_ch(&Buttons);
		delay_us(1000);// Sleep(1);
		// NO KEYBOARD ACTIVITY: returns 0 and puts "0" in address &Buttons
		// REGULAR KEY PRESSED: returns ASCII of PC key pressed, and puts "0" in address &Buttons
		// PUSH BUTTON SWITCH EMULATION KEY PRESSED (Home, end, PgUp, PgDwn,F12):
		//returns 0 and puts in '&Buttons' button code, also updates Display_Info.butt_states and Display_Info.ButtonStateChanged
		//		if (Buttons != (BIT_BUT_RIGHT + BIT_BUT_UP + BIT_BUT_DOWN + BIT_BUT_LEFT)) // do LCD_menu
		CheckExecuteFrontPanelCmd(); //check LCD command
		//		else
		//		Sleep(1);
		if (ret_char != 0)
			return (ret_char);
	}
}

/*************************/
void Wait_in_Visual_C(unsigned int Delay)
{
	unsigned long DelayTick = 0;				// clear out ctr
	Delay = Delay * TICKS_IN_mSEC;
	while (DelayTick < Delay)
	{
		DelayTick++;
		delay_us(1000);// Sleep(1);
	}
}

char* g_pszOutput = "BatMon_CommFW_export_file";
//// File name, BatMon_CommFW_export_file 2023-08-03 12-43-51.param
FILE* g_fhOutput = NULL;

FILE*
OpenOutputFile(char* szFile)
{
	FILE* fhOut;
	unsigned long ulSize;

	fhOut = fopen(szFile, "r"); // try to open
	if (fhOut) // file exists
	{
		// Close the file.
		fclose(fhOut);
	}

	// If we get here, it is safe to open and, possibly, overwrite the output file.
	printf("Opening output file %s\n", szFile);

	// Open the file in ASCII mode
	fhOut = fopen(szFile, "w");

	if (fhOut)
	{
		char tmp[100];
		//file_name example "DC-2020_test_export_file 2019-05-03 23-43-51.param"
		// We are writing an ASCII C output file so add a header to
		// describe what the file contains.
		//
		fprintf(fhOut, "//// File name,");//// File name,
		fprintf(fhOut, szFile);//// File name, DC-2020 parameters 2019-05-03 23-43-51.param
		fprintf(fhOut, "\n//// LNP tester, This file was automatically generated");
		ulSize = strlen(g_pszOutput);
		strncpy(tmp, &szFile[ulSize + 1], 10);
		tmp[10] = '\0';
		fprintf(fhOut, "\n//// Date, ");
		fprintf(fhOut, tmp);
		strncpy(tmp, &szFile[ulSize + 12], 8);
		tmp[2] = ':'; tmp[5] = ':';
		tmp[8] = '\0';
		fprintf(fhOut, "\n//// Time, ");
		fprintf(fhOut, tmp);
		fprintf(fhOut, "\n");
		//fprintf(fhOut, "//***************************************************************************\n");
		//fprintf(fhOut, "//\n");
		//fprintf(fhOut, "// Test output of param file\n");
		//fprintf(fhOut, "//\n");
		//fprintf(fhOut, "//This file was automatically generated\n");
		//fprintf(fhOut, "//\n");
		//fprintf(fhOut, "//***************************************************************************\n\n");
	}
	return(fhOut);
}


// open text file to save output
#include <time.h> // for text file name

/********************************************************************/
/********************************************************************/
/*        E X P O R T  T O  F I L E  ( only for PC simulation )     */
/*        >>> NOT FOR ATMEL <<<                                     */
/********************************************************************/
/*  DESCRIPTION: This is a 'wrapper' function to capture output into file
*				If this function is called when IsFileOutput is true,
*				it will open text file and output there enything going
*				to console using functions printf() putchar() cputs()
*				If IsFileOutput is false, function otputs to console only
*				This allows to save console output for analysis
*	Input:      pointer to function which output should be captured
*	Outputs:    to console and to file.
*	Notes:
* Revisions:    2023-06-17    IK   Created.                     */
/********************************************************************/

void Export_ToFile(void* f_ptr)
{
#ifdef PC // open param file to save output
	char file_name[100];
#ifdef OUTPUT_TEST_DATA_TO_FILE
	IsFileOutput = true;
#endif //OUTPUT_TEST_DATA_TO_FILE
	if (IsFileOutput)
	{
		time_t rawtime = time(NULL);
		//	char * time_str = ctime(&rawtime);
		//	time_str[strlen(time_str)-1] = '\0';
		//	sprintf(file_name,"%s %s.param",g_pszOutput,time_str);

		struct tm* timeinfo;
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		sprintf(file_name, "%s %02d-%02d-%02d %02d-%02d-%02d.txt", g_pszOutput, timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
		// Open the output file for writing in ASCII mode.

		g_fhOutput = OpenOutputFile(file_name);
	}
#endif // PC

	((void(*)())(f_ptr))();		//call function

#ifdef PC
	if (IsFileOutput)
	{
		// Close the output file.
		fclose(g_fhOutput);
		g_fhOutput = NULL; // detroy handle, or next text output to console will cause error
		printf("\n\rPC output file closed: %s\n", file_name);
		IsFileOutput = false;
	}
#endif //PC
}

